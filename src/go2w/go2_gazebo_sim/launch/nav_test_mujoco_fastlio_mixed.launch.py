#!/usr/bin/env python3
"""Heterogeneous dual-robot MuJoCo + Fast-LIO2 + FAR nav benchmark launch.

robot_a = Go2W (wheeled-legged), robot_b = Go2 (legs only, passive feet).
Both share one MuJoCo process and one combined URDF; each runs its own
Fast-LIO2 / octomap / FAR / nav stack. A shared CFPA2 coordinator
partitions frontier goals across the two.

Scene: `demo3_mixed.xml` (Go2W at (4, 2) and Go2 at (4, -6), 24×16m).

Key deltas from `nav_test_mujoco_fastlio_dual.launch.py`:
  - MJCF: demo3_mixed.xml  (robot_b Go2W body swapped for Go2 passive-foot body)
  - ros_control: ros_control_mixed_mujoco_nav.yaml (robot_b has no wheel controller)
  - URDF: Go2W xacro for A + Go2 xacro b-prefixed for B, merged via
    `build_mixed_mujoco_urdf`
  - Controllers: robot_b gets no `robot_b_wheel_velocity_controller`
    (``wheel_controller_name=None``)
  - Cmd routing: robot_b skips `go2w_hybrid_cmd_router`; cmd_vel pipes
    directly to CHAMP's `cmd_vel_legged`.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

# sys.path must be amended BEFORE the `from modules.*` imports below — when
# ros2 launch loads this file it doesn't add the launch dir to sys.path.
_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)

# Workspace root resolved via the launch file's realpath — symlink-install
# means __file__ points into install/, but the actual src lives at
# <ws>/src/go2w/go2_gazebo_sim/launch/. Walk 4 dirs up to <ws>/. Used to
# locate scripts under <ws>/scripts/ that aren't installed as a ROS package.
_ws_root = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", ".."))

import xacro
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from modules import _find_mujoco_plugin_dir
from modules.assets import build_dual_robot_stack, build_namespaced_robot_description
from modules.dual_urdf import build_mixed_mujoco_urdf, build_robot_b_urdf


def _as_bool(v: str) -> bool:
    return str(v).strip().lower() in {"1", "true", "yes", "on"}


def _get(ctx, key: str) -> str:
    return LaunchConfiguration(key).perform(ctx)


def _workspace_root() -> Path:
    for candidate in (Path(__file__).resolve(), Path.cwd()):
        for parent in (candidate, *candidate.parents):
            if (parent / "scripts" / "runtime").is_dir() and (parent / "src").is_dir():
                return parent
    # Symlink-install launches normally hit the source tree above. This
    # fallback preserves the old relative assumption for unusual installs.
    return Path(__file__).resolve().parents[4]


# Executables we KEEP visible on the terminal when `debug:=true`. Everything
# else (mujoco, fast_lio, octomap, champ, ekf, sensor bridges, terrain
# analysis, rviz, map_merge, collision/session monitors, cfpa2 coordinator,
# RSP, controller spawners, …) is rerouted to ~/.ros/log so the operator can
# read planner/controller output without scrolling past unrelated noise.
#
# Match is substring-based against the node's executable string so it works
# for both bare binaries (`astar_nav_node`) and python entrypoints
# (`twist_bridge.py`).
_NAV_DEBUG_KEEP_EXECUTABLES = (
    # Path planners / nav
    "astar_nav_node",
    "hybrid_astar_nav_node",
    "nav2_hybrid_astar_nav_node",
    "far_planner",
    "localPlanner",
    "pathFollower",
    "far_status_adapter",
    # Controller / cmd_vel pipeline
    "twist_bridge",
    "go2w_hybrid_cmd_router",
    # Goal source — without this, an agent reading the terminal can't see
    # WHERE the planner was told to drive, so wall hits / stuck events are
    # un-attributable to a goal selection.
    "cfpa2_coordinator",
    # Safety + diagnostic monitor — emits the WALL CONTACT / TIP-OVER /
    # PLANNER STUCK warnings, NEW GOAL info, the periodic STATE+SAFETY
    # summary lines, AND the rolling state-history snapshot block on each
    # crash event. Without it on the keep-list, debug:=true hides exactly
    # the diagnostic stream this whole feature is meant to surface.
    "dual_robot_collision_monitor",
    # Map augmenter — logs how many unknown cells got filled from the
    # merged map every ~10 republishes. Useful in debug to confirm B's
    # local map is actually being enriched with A's exploration.
    "map_augmenter",
    # robot_self_filter — logs every 10 s how many points it dropped from
    # peer-body returns. Quiet when peers aren't in line-of-sight; useful
    # to see "filter is doing work" in debug.
    "robot_self_filter",
)

# Of the kept executables, these emit per-cycle INFO at 5-10 Hz without
# throttling (CMU stack + cfpa2's frontier-allocation cycle log). Left at
# default INFO they bury the safety monitor's signal under ~1000+ lines per
# 2-minute trial. In debug mode we drop them to WARN — they still emit
# warnings on real problems (graph orphan, blocked rotation primitives,
# allocation timeouts), which is exactly the diagnostic signal an agent
# needs and nothing more. The remaining four kept nodes (astar_nav_node,
# twist_bridge, go2w_hybrid_cmd_router, far_status_adapter) are already
# throttled / brief, and the monitor itself stays at INFO so its periodic
# STATE+SAFETY+COVERAGE summary remains visible.
_NAV_DEBUG_VERBOSE_NODES = (
    "far_planner",
    "localPlanner",
    "pathFollower",
    "cfpa2_coordinator",
)


def _exec_string(node) -> str:
    """Best-effort string repr of a Node/ExecuteProcess executable for matching.

    `Node.node_executable` and `ExecuteProcess.process_details['cmd']` are
    populated only after launch context resolution, so we peek at the raw
    constructor args. For Node, that's `_Node__node_executable` (a list of
    Substitutions). For ExecuteProcess, the first element of `__cmd` carries
    the executable.
    """
    raw = getattr(node, "_Node__node_executable", None)
    if raw is None:
        raw = getattr(node, "_ExecuteLocal__cmd", None) \
            or getattr(node, "_ExecuteProcess__cmd", None)
    if raw is None:
        return ""
    if isinstance(raw, str):
        return raw
    try:
        return " ".join(
            getattr(s, "text", str(s)) for s in (raw if isinstance(raw, list) else [raw])
        )
    except Exception:
        return str(raw)


def _silence_action(act) -> None:
    """Reroute `act`'s stdout/stderr to the log file (output='log'). Mutates
    the launch_ros / launch action in place — references held elsewhere
    (e.g. RegisterEventHandler.target_action) remain valid.
    """
    try:
        act._ExecuteLocal__output = [TextSubstitution(text="log")]
    except Exception:
        # Older launch versions named the slot differently; try the legacy
        # ExecuteProcess private name as a fallback.
        try:
            act._ExecuteProcess__output = [TextSubstitution(text="log")]
        except Exception:
            pass


def _drop_log_level_to_warn(act) -> None:
    """Append `--ros-args --log-level WARN` to a Node's arguments so its
    process-default ROS log threshold is WARN (kills per-cycle INFO floods
    while keeping warnings/errors). For nodes that ALREADY have a
    `--log-level <name>:=<level>` arg pinned to a specific logger, this
    override layers on top of theirs (last wins for the default level), so
    we don't accidentally re-enable INFO on terrain_analysis etc.

    `_Node__arguments` is None when the Node was constructed without an
    explicit `arguments=` kwarg (true for most of the kept nodes). Replace
    it with a fresh list rather than failing silently, otherwise the WARN
    override never makes it onto the cmd line.
    """
    extra = [
        TextSubstitution(text="--ros-args"),
        TextSubstitution(text="--log-level"),
        TextSubstitution(text="WARN"),
    ]
    args = getattr(act, "_Node__arguments", None)
    if isinstance(args, list):
        args.extend(extra)
    else:
        try:
            act._Node__arguments = list(extra)
        except Exception:
            pass


def _filter_actions_to_nav_only(actions) -> None:
    """Walk `actions` recursively (TimerAction.actions, RegisterEventHandler
    target_action). For every Node / ExecuteProcess whose executable is NOT
    in _NAV_DEBUG_KEEP_EXECUTABLES, switch its output to 'log'. Nodes built
    inside helpers (build_dual_robot_stack, _build_sensor_bridges,
    _build_fastlio_nav_stack) are reached through the TimerActions they
    return.
    """
    from launch.actions import (
        ExecuteProcess as _ExecuteProcess,
        TimerAction as _TimerAction,
        RegisterEventHandler as _RegEvt,
    )

    def _visit(act):
        if isinstance(act, _ExecuteProcess):
            es = _exec_string(act)
            if not any(keep in es for keep in _NAV_DEBUG_KEEP_EXECUTABLES):
                _silence_action(act)
            elif any(verbose in es for verbose in _NAV_DEBUG_VERBOSE_NODES):
                # Kept on screen, but its INFO floods would drown the
                # safety monitor — pin its default log level to WARN.
                _drop_log_level_to_warn(act)
            return
        if isinstance(act, _TimerAction):
            for child in (act.actions or []):
                _visit(child)
            return
        if isinstance(act, _RegEvt):
            tgt = getattr(act.event_handler, "_target_action", None)
            if tgt is not None:
                _visit(tgt)
            return

    for a in actions:
        _visit(a)


def _load_yaml_params(yaml_path: str) -> dict:
    """Load a ROS2 YAML param file and return the ros__parameters dict."""
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}
    for _node_name, inner in data.items():
        if isinstance(inner, dict) and "ros__parameters" in inner:
            return dict(inner["ros__parameters"])
    return data


def _build_cleanup_stale_cmd() -> str:
    """Kill any leftover sim/nav processes from a prior run.

    Pattern list mirrors benchmark_fastlio.sh's cleanup_procs so we cover
    everything that could pollute DDS discovery and break controller_manager
    service lookup in the next launch.
    """
    # Target only processes known to leak between runs. Previously included
    # "/go2w_perception/" but that patterned-killed this same launch's qos_bridge
    # and pointcloud_adapter when they spawned at T=0 alongside cleanup_stale —
    # breaking the LiDAR → Fast-LIO → octomap → /map → CFPA2 pipeline from
    # step 1. External benchmark scripts (benchmark_fastlio.sh) handle stale
    # perception procs before launching; no need to duplicate here.
    patterns = [
        "ros2 launch go2_gazebo_sim nav_test_mujoco",
        "mujoco_ros2_control",
        "/mujoco_sensor_bridge/",
        "/champ_base/",
        "/fast_lio/",
        "/far_planner/",
        "/local_planner/",
        "/terrain_analysis",
        "/octomap_server/",
        "/cfpa2_collaborative_autonomy/",
        "/robot_state_publisher",
        "/robot_localization/",
        "/opt/ros/.*/lib/controller_manager/spawner",
        "session_reporter.py",
        "dual_robot_collision_monitor.py",
    ]
    cmd = [
        "SELF=$$; PARENT=$PPID; ",
        "kill_pattern(){ ",
        "  PATTERN=\"$1\"; SIGNAL=\"$2\"; ",
        "  for PID in $(pgrep -f \"$PATTERN\" 2>/dev/null || true); do ",
        "    [ \"$PID\" = \"$SELF\" ] && continue; ",
        "    [ \"$PID\" = \"$PARENT\" ] && continue; ",
        "    kill -\"$SIGNAL\" \"$PID\" 2>/dev/null || true; ",
        "  done; ",
        "}; ",
    ]
    for p in patterns:
        cmd.append(f"kill_pattern '{p}' TERM; ")
    cmd.append("sleep 1; ")
    for p in patterns:
        cmd.append(f"kill_pattern '{p}' KILL; ")
    cmd.append(
        "rm -f /dev/shm/sem.fastrtps_* /dev/shm/sem.fastdds_* "
        "/dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null || true; "
    )
    cmd.append("sleep 0.5")
    return "".join(cmd)


def _build_sensor_bridges(ns: str, mjcf_path: str, base_body: str, imu_site: str,
                          pose_sensor: str, imu_sensor: str, links_config: str,
                          use_sim_time: bool, contact_odom_topic: str = "odom/ground_truth"):
    """Per-robot MuJoCo sensor bridges (ground-truth odom + foot contacts).

    The mujoco plugin publishes raw sensor topics under its own namespace
    (`/mujoco_sim/...`). We subscribe to those in the per-robot bridge
    and republish under /{ns}/odom/ground_truth, /{ns}/imu/data,
    /{ns}/foot_contacts.
    """
    return [
        Node(
            package="mujoco_sensor_bridge",
            executable="mujoco_odom_bridge",
            namespace=ns,
            name="mujoco_odom_bridge",
            parameters=[{
                "use_sim_time": use_sim_time,
                "mjcf_path": mjcf_path,
                "publish_rate": 50.0,
                "base_body_name": base_body,
                "odom_frame": "odom",
                "base_frame": base_body,
                # 2026-04-29: TF chain ownership moved to fast_lio_tf_adapter
                # (publishes map→base_link from Fast-LIO's `/<ns>/Odometry`).
                # mujoco_odom_bridge keeps publishing the *topic*
                # /<ns>/odom/ground_truth for sim-only consumers
                # (collision_monitor / session_reporter / GT bootstrap), but
                # NO LONGER writes TF — sim and real now share the same TF
                # source (Fast-LIO via adapter). Without this change, sim's
                # mujoco_odom_bridge would compete with the adapter on the
                # same `odom→base_link` link, producing duplicate TFs and
                # confusing TF lookups.
                "publish_tf": False,
                "pose_topic": f"/mujoco_sim/{pose_sensor}/pose",
                "imu_topic": f"/mujoco_sim/{imu_sensor}/imu",
                "republish_imu_topic": "imu/data",
            }],
            remappings=[
                ("/tf", f"/{ns}/tf"),
                ("/tf_static", f"/{ns}/tf_static"),
            ],
            output="screen",
        ),
        Node(
            package="mujoco_sensor_bridge",
            executable="mujoco_contact_node",
            namespace=ns,
            name="mujoco_contact_bridge",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"mjcf_path": mjcf_path},
                {"publish_rate": 50.0},
                {"odom_topic": contact_odom_topic},
                links_config,
            ],
            output="screen",
        ),
    ]


def _build_robot_state_publisher(ns: str, robot_description: str, use_sim_time: bool):
    """Minimal TF publisher for sensor/SLAM-only validation runs."""
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            {"use_tf_static": False},
            {"publish_frequency": 200.0},
            {"ignore_timestamp": True},
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/tf", f"/{ns}/tf"),
            ("/tf_static", f"/{ns}/tf_static"),
        ],
        output="screen",
    )


def _build_fastlio_nav_stack(
    *,
    ns: str,
    mujoco_lidar_topic: str,
    base_frame: str,
    imu_frame: str,
    use_sim_time: bool,
    nav_backend: str,
    slam_delay: float,
    nav_delay: float,
    go2w_config_pkg: str,
    slam_config_path: str,
    adapter_num_rings: int,
    adapter_min_vert_angle_deg: float,
    adapter_max_vert_angle_deg: float,
    local_planner_paths_dir: str,
    far_tuning_yaml: str,
    far_default_yaml: str,
    octomap_min_z: float = 0.20,
    enable_nav: bool = True,
    has_wheels: bool = True,
    peer_namespaces: list | None = None,
    loop_closure: bool = False,
    loop_closure_backend: str = "auto",
    bootstrap_from_gt: bool = True,
    peer_obstacle_enabled: bool = False,
    slam_backend: str = "fast_lio_scpgo",
    dynamic_filter_backend: str = "none",
):
    """Per-robot Fast-LIO + octomap + FAR nav stack.

    Returns a list of Node / TimerAction actions. Keep them grouped
    under TimerActions so they start in the right order:
      T=slam_delay        — Fast-LIO, octomap, static TFs
      T=slam_delay+0.5    — pointcloud_frame_bridge (body → map)
      T=nav_delay         — FAR stack (terrain_analysis × 2, far_planner,
                            localPlanner, pathFollower)
    """
    tf_remaps = [("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")]
    actions = []

    # ── QoS bridge: BE LiDAR → Reliable (Fast-LIO needs Reliable) ──
    actions.append(
        Node(
            package="go2w_perception",
            executable="qos_bridge.py",
            namespace=ns,
            name="qos_bridge",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": mujoco_lidar_topic,
                "output_topic": f"/{ns}/registered_scan_reliable",
                "input_reliability": "best_effort",
                "output_reliability": "reliable",
            }],
            output="screen",
        )
    )

    # ── robot_self_filter: strip LiDAR returns from peer-robot bodies ──
    # Without this, A's LiDAR sees B's chassis when in line-of-sight,
    # octomap marks the cells as obstacles in A's local map, and the
    # pollution propagates: /{ns}/map → /merged_map → (via map_augmenter)
    # → other robots' planning view. With this, octomap consumes the
    # peer-filtered cloud /{ns}/registered_scan_octomap; Fast-LIO and
    # other consumers keep using the raw /{ns}/registered_scan_reliable
    # so their point density is unaffected.
    #
    # peer_namespaces lists the OTHER robots — for robot_a it's
    # [robot_b], for robot_b it's [robot_a]. Each filter subscribes to
    # /{peer}/odom/ground_truth (sim) and drops any cloud point whose
    # planar position lies within `peer_filter_radius_m` of any peer.
    # On real hardware the peer-pose source becomes whatever the swarm-
    # comm layer broadcasts; only this parameter changes.
    if peer_namespaces:
        actions.append(
            Node(
                package="go2w_perception",
                executable="robot_self_filter",
                namespace=ns,
                name="robot_self_filter",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "input_topic": "registered_scan_reliable",
                    "output_topic": "registered_scan_octomap",
                    "peer_namespaces": list(peer_namespaces),
                    "peer_pose_topic": "/odom/ground_truth",
                    # Go2W body half-diag 0.45 m, Go2 0.36 m. Radius
                    # bumped 0.50 → 0.80 (2026-04-26): 0.50 was barely
                    # larger than the body and missed:
                    #   • pose-update lag (LiDAR 10 Hz, odom 50 Hz, up
                    #     to 100 ms = 5 cm slip at 0.5 m/s)
                    #   • CHAMP gait swing-foot extending beyond body box
                    #   • body rotation: rectangle corners protrude past
                    #     the half-diag during yaw motion
                    # Bumped 0.80 → 1.20 (2026-04-26): demo3_mixed dual
                    # showed asymmetric drop rates A=2.5/scan vs
                    # B=14.1/scan. Either pose-frame mismatch (peer
                    # odom-frame xy vs cloud lidar-frame xy can drift
                    # 0.1-0.3 m) or geometric difference (Go2W taller,
                    # sees over B's body) caused A to leave B-imprint
                    # in its octomap → permanent fake walls when robots
                    # crossed. 1.20 m absorbs the worst-case frame
                    # mismatch + body envelope + pose lag.
                    "peer_filter_radius_m": 1.20,
                    # Stale tolerance tightened 2.0 → 0.3 s. With 2 s
                    # peer could be 0.6 m off (0.3 m/s × 2 s) — filter
                    # circle drawn in wrong place, peer body untouched
                    # by filter, octomap stamps peer as a moving wall.
                    # 0.3 s = 6 ticks at 20 Hz, enough slack for one
                    # missed odom tick but no more.
                    # Bumped 0.3 → 5.0 (2026-04-26): demo3_mixed showed
                    # asymmetric drop A=0/scan B=217/scan. A's filter
                    # subscribed to peer odom but dropped nothing —
                    # pose freshness check rejected every received
                    # message. Likely sim_time vs wall_time stamp
                    # mismatch at startup. 5 s is generous; if peer
                    # is truly disconnected we'll know fast.
                    "peer_pose_stale_sec": 5.0,
                    "stats_log_period_sec": 10.0,
                }],
                output="screen",
            )
        )
    else:
        # Single-robot fallback: feed the raw cloud directly so octomap
        # gets data without us building a no-op filter.
        actions.append(
            Node(
                package="go2w_perception",
                executable="qos_bridge.py",
                namespace=ns,
                name="cloud_passthrough",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "input_topic": f"/{ns}/registered_scan_reliable",
                    "output_topic": f"/{ns}/registered_scan_octomap",
                    "input_reliability": "reliable",
                    "output_reliability": "reliable",
                }],
                output="screen",
            )
        )

    # ── pointcloud_adapter: registered_scan → velodyne_points for Fast-LIO ──
    # Consumes the PEER-FILTERED cloud (`registered_scan_octomap`, despite
    # the misleading name kept for back-compat — see robot_self_filter
    # block above). Without this, Fast-LIO's ICP front-end sees the peer
    # robot's chassis as a moving rigid body inside its scan, drags pose
    # estimate toward those points, and the published TF map→base_link
    # drifts. Downstream, astar_nav uses that drifted pose to compute
    # commands → robot ends up walking on top of obstacles (the
    # "climbed cross_v_n" failure in 2026-04-25 demo3_mixed).
    fastlio_input_topic = (
        f"/{ns}/registered_scan_octomap"
        if peer_namespaces
        else f"/{ns}/registered_scan_reliable"
    )
    actions.append(
        Node(
            package="go2w_perception",
            executable="pointcloud_adapter.py",
            namespace=ns,
            name="pointcloud_adapter",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"input_topic": fastlio_input_topic},
                {"output_topic": f"/{ns}/velodyne_points"},
                {"num_rings": adapter_num_rings},
                {"min_vert_angle_deg": adapter_min_vert_angle_deg},
                {"max_vert_angle_deg": adapter_max_vert_angle_deg},
            ],
            output="screen",
        )
    )

    # ── pointcloud_to_laserscan: 3D → 2D for visualisation / secondary use ──
    actions.append(
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            namespace=ns,
            name="pc_to_laserscan",
            parameters=[{
                "use_sim_time": use_sim_time,
                "target_frame": base_frame,
                "transform_tolerance": 0.1,
                "min_height": 0.05,
                "max_height": 0.60,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.3,
                "range_max": 30.0,
                "use_inf": True,
            }],
            remappings=[
                # astar_nav uses /{ns}/scan_3d for the obstacle-stop /
                # slow check. Feeding it the peer-filtered cloud means
                # the robot won't false-stop on a peer's body returns
                # in line-of-sight — same rationale as filtering
                # octomap and Fast-LIO above. Single-robot case still
                # uses the raw cloud since no filter exists.
                ("cloud_in",
                 f"/{ns}/registered_scan_octomap" if peer_namespaces
                 else f"/{ns}/registered_scan_reliable"),
                ("scan", f"/{ns}/scan_3d"),
            ] + tf_remaps,
            output="screen",
        )
    )

    # ── Static TFs: world ≡ map ≡ odom ≡ body (all identity to base) ──
    # Fast-LIO publishes the /cloud_registered_body PointCloud2 with
    # header.frame_id="body" regardless of URDF. pointcloud_frame_bridge
    # needs to resolve `body → map` — if missing, registered_scan_map →
    # terrain_analysis → FAR never produces output and robots can't move.
    # The per-robot URDF's `imu` link is NOT connected to `base_link` via
    # robot_state_publisher (the xacro has `imu` as an orphan), so we can
    # NOT chain through imu. Attach `body` directly to the per-robot
    # base frame (base_link or b_base_link).
    for parent, child in [("world", "map"), ("map", "odom"), (base_frame, "body")]:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=ns,
                name=f"{parent}_to_{child}_tf".replace("-", "_"),
                arguments=[
                    "--frame-id", parent, "--child-frame-id", child,
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                ],
                remappings=[("/tf_static", f"/{ns}/tf_static")],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        )

    # ── Primary / shadow SLAM backend ──
    fastlio_slam_nodes = [
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            namespace=ns,
            name="slam_node",
            parameters=[slam_config_path, {"use_sim_time": use_sim_time}],
            # Fast-LIO hard-codes a `camera_init -> body` TF
            # (laserMapping.cpp:654). Letting it hit /{ns}/tf gives `body`
            # two parents (ours: base_link, Fast-LIO's: camera_init) and
            # breaks `body -> map` lookup. Route Fast-LIO's /tf to a sink;
            # nobody consumes it. /tf_static is still shared normally.
            remappings=[
                ("/velodyne_points", f"/{ns}/velodyne_points"),
                ("/imu/data", f"/{ns}/imu/data"),
                ("/Odometry", f"/{ns}/Odometry"),
                ("/cloud_registered_body", f"/{ns}/cloud_registered_body"),
                ("/tf", f"/{ns}/fastlio_tf_sink"),
                ("/tf_static", f"/{ns}/tf_static"),
            ],
            output="screen",
        ),
        # fast_lio_tf_adapter: replaces both slam_odom_relay (frame
        # remap) and the mujoco_odom_bridge's TF role. Subscribes
        # Fast-LIO's `/<ns>/Odometry` (frame camera_init→body), applies
        # one-shot GT bootstrap so map frame's origin aligns with the
        # world origin, then publishes:
        #   - /<ns>/odom/nav         (Odometry, frame=map child=base_link)
        #   - TF map → base_link     (the canonical pose for Nav2)
        # Real-robot compatible: doesn't depend on mujoco_odom_bridge or
        # CHAMP's broken state_estimation/odom_raw chain. On real, the
        # same code path runs; with SC-PGO ported (loop_closure:=true),
        # adapter prefers /<ns>/corrected_odom over raw when fresh.
        ExecuteProcess(
            cmd=[
                "python3", "-u",
                os.path.join(_ws_root, "scripts/runtime/fast_lio_tf_adapter.py"),
                "--ros-args",
                "-p", f"namespace:={ns}",
                "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
                "-p", "input_topic:=Odometry",
                "-p", "output_topic:=odom/nav",
                # TF parent must match local_costmap's `global_frame: odom`.
                # The static `map → odom = identity` from the launch's TF
                # publishers connects this to the map frame for global
                # planning. With GT bootstrap the alignment offset (dx,
                # dy, yaw_offset) is baked into the published pose, so
                # robot's pose in `odom` already equals world coords.
                "-p", "output_frame_id:=odom",
                "-p", f"output_child_frame_id:={base_frame}",
                "-p", "publish_tf:=true",
                "-p", f"bootstrap_from_gt:={'true' if bootstrap_from_gt else 'false'}",
                "-p", "gt_topic:=odom/ground_truth",
                "-p", "corrected_topic:=corrected_odom",
                # TransformBroadcaster publishes to global /tf by default;
                # in this namespaced dual-robot setup, all consumers
                # subscribe /<ns>/tf. Without this remap, the adapter's
                # TF is invisible to nav2 (Could not find a connection
                # between 'odom' and 'base_link').
                "-r", f"/tf:=/{ns}/tf",
                "-r", f"/tf_static:=/{ns}/tf_static",
            ],
            name=f"fast_lio_tf_adapter_{ns}",
            output="screen",
        ),
    ]
    swarm_adapter_node = Node(
        package="slam_backend_adapters",
        executable="swarm_lio2_ros2_adapter_node",
        namespace=ns,
        name="swarm_lio2_ros2_adapter_node",
        parameters=[{
            "use_sim_time": use_sim_time,
            "namespace": ns,
            "slam_backend": slam_backend,
            "base_frame": base_frame,
            "dynamic_filter_backend": dynamic_filter_backend,
            "publish_tf": True,
            "output_odom_frame_id": "odom" if slam_backend == "swarm_lio2_primary" else f"{ns}/odom",
            "output_child_frame_id": base_frame if slam_backend == "swarm_lio2_primary" else f"{ns}/base_link",
            "output_map_frame_id": "map" if slam_backend == "swarm_lio2_primary" else f"{ns}/map",
            "output_static_cloud_frame_id": base_frame if slam_backend == "swarm_lio2_primary" else f"{ns}/base_link",
            "output_map_cloud_frame_id": "map" if slam_backend == "swarm_lio2_primary" else f"{ns}/map",
        }],
        remappings=[
            ("/tf", f"/{ns}/tf"),
            ("/tf_static", f"/{ns}/tf_static"),
        ],
        output="screen",
    )
    if slam_backend == "swarm_lio2_primary":
        slam_nodes = [swarm_adapter_node]
    elif slam_backend == "swarm_lio2_shadow":
        slam_nodes = [*fastlio_slam_nodes, swarm_adapter_node]
    else:
        slam_nodes = fastlio_slam_nodes
    actions.append(TimerAction(period=slam_delay, actions=slam_nodes))

    # ── Optional: SC-PGO loop-closure post-processor on top of Fast-LIO ──
    # Backend options:
    #   auto          - prefer native ROS 2 sc_pgo if installed; otherwise
    #                   expect Docker ROS1 SC-PGO + ros1_bridge and start the
    #                   PoseStamped->Odometry adapter.
    #   ros2_sc_pgo  - launch only native ROS 2 sc_pgo.
    #   ros1_bridge  - Docker owns ROS 1 SC-PGO; this launch only converts
    #                   bridged /<ns>/sc_pgo/pose_stamped into
    #                   /<ns>/corrected_odom (nav_msgs/Odometry).
    if loop_closure and slam_backend != "swarm_lio2_primary":
        def _launch_ros1_bridge_adapter(reason: str) -> None:
            adapter_path = os.path.join(
                _ws_root, "scripts", "runtime", "scpgo_pose_to_odom_adapter.py"
            )
            actions.append(LogInfo(msg=(
                f"[nav_test_mujoco_fastlio_mixed] loop_closure:=true ns={ns}: "
                f"using ROS1 bridge adapter ({reason}). Start Docker bridge with: "
                f"bash scripts/launch/scpgo_ros1_bridge.sh"
            )))
            actions.append(
                TimerAction(
                    period=slam_delay + 3.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                "python3", "-u", adapter_path,
                                "--ros-args",
                                "-p", f"namespace:={ns}",
                                "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
                                "-p", "pose_topic:=sc_pgo/pose_stamped",
                                "-p", "raw_odom_topic:=Odometry",
                                "-p", "output_topic:=corrected_odom",
                                "-p", f"child_frame_id:={base_frame}",
                            ],
                            name=f"scpgo_pose_to_odom_adapter_{ns}",
                            output="screen",
                        ),
                    ],
                )
            )

        if loop_closure_backend in {"auto", "ros2_sc_pgo"}:
            try:
                import ament_index_python.packages as _ament_pkg
                _sc_pgo_share = _ament_pkg.get_package_share_directory("sc_pgo")
                _sc_pgo_config_candidates = [
                    os.path.join(_sc_pgo_share, "config", "sc_pgo_params.yaml"),
                    os.path.join(_sc_pgo_share, "config", "params.yaml"),
                ]
                _sc_pgo_config = next(
                    (p for p in _sc_pgo_config_candidates if os.path.exists(p)),
                    None,
                )
                if _sc_pgo_config:
                    actions.append(
                        TimerAction(
                            period=slam_delay + 3.0,
                            actions=[
                                Node(
                                    package="sc_pgo",
                                    executable="sc_pgo_node",
                                    namespace=ns,
                                    name="sc_pgo",
                                    parameters=[
                                        _sc_pgo_config,
                                        {"use_sim_time": use_sim_time},
                                    ],
                                    remappings=[
                                        ("/aft_mapped_to_init", f"/{ns}/Odometry"),
                                        ("/cloud_registered", f"/{ns}/cloud_registered_body"),
                                        ("/corrected_odom", f"/{ns}/corrected_odom"),
                                        ("/corrected_path", f"/{ns}/corrected_path"),
                                        ("/corrected_cloud", f"/{ns}/corrected_cloud"),
                                        ("/corrected_map", f"/{ns}/corrected_map"),
                                    ] + tf_remaps,
                                    output="screen",
                                ),
                            ],
                        )
                    )
                elif loop_closure_backend == "ros2_sc_pgo":
                    actions.append(LogInfo(msg=(
                        f"[nav_test_mujoco_fastlio_mixed] loop_closure_backend:=ros2_sc_pgo "
                        f"for ns={ns}, but no sc_pgo_params.yaml/params.yaml exists in "
                        f"{_sc_pgo_share}/config; skipping loop closure."
                    )))
                else:
                    _launch_ros1_bridge_adapter(
                        f"native ROS2 sc_pgo has no config in {_sc_pgo_share}/config"
                    )
            except Exception as _e:
                if loop_closure_backend == "ros2_sc_pgo":
                    actions.append(LogInfo(msg=(
                        f"[nav_test_mujoco_fastlio_mixed] loop_closure_backend:=ros2_sc_pgo "
                        f"requested for ns={ns}, but `sc_pgo` is unavailable ({_e}); "
                        f"skipping loop closure."
                    )))
                else:
                    _launch_ros1_bridge_adapter(f"native ROS2 sc_pgo unavailable: {_e}")
        elif loop_closure_backend == "ros1_bridge":
            _launch_ros1_bridge_adapter("loop_closure_backend:=ros1_bridge")
        else:
            actions.append(LogInfo(msg=(
                f"[nav_test_mujoco_fastlio_mixed] invalid loop_closure_backend="
                f"{loop_closure_backend!r}; skipping loop closure for ns={ns}."
            )))

    # ── pointcloud_frame_bridge: body-frame Fast-LIO cloud → map frame for FAR ──
    actions.append(
        TimerAction(
            period=slam_delay + 0.5,
            actions=[
                Node(
                    package="go2w_perception",
                    executable="pointcloud_frame_bridge.py",
                    namespace=ns,
                    name="registered_scan_frame_bridge",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"input_topic": f"/{ns}/cloud_registered_body"},
                        {"output_topic": f"/{ns}/registered_scan_map"},
                        {"target_frame": "map"},
                        {"tf_timeout_sec": 0.15},
                        {"transform_wait_sec": 0.10},
                        {"max_cloud_age_sec": 0.80},
                    ],
                    remappings=tf_remaps,
                    output="screen",
                ),
            ],
        )
    )

    # ── Octomap: /{ns}/map OccupancyGrid from Fast-LIO cloud ──
    octomap_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        namespace=ns,
        name="octomap_server",
        parameters=[{
            "use_sim_time": use_sim_time,
            "resolution": 0.05,
            "frame_id": "map",
            "base_frame_id": base_frame,
            "sensor_model.max_range": 6.0,
            "sensor_model.hit": 0.8,
            "sensor_model.miss": 0.35,
            "sensor_model.min": 0.12,
            "sensor_model.max": 0.97,
            "point_cloud_min_z": octomap_min_z,
            "point_cloud_max_z": 1.10,
            "occupancy_min_z": octomap_min_z,
            "occupancy_max_z": 1.00,
            # filter_ground_plane would need min_z <= 0 to see ground; our
            # min_z=0.20 already excludes the floor, so leave ground-filter
            # off (it spams "No ground plane found in scan" at 10 Hz otherwise).
            "filter_ground_plane": False,
            "filter_speckles": False,
            "compress_map": True,
            "latch": True,
            "publish_free_space": False,
        }],
        remappings=[
            # Octomap consumes the PEER-FILTERED cloud, not the raw one.
            # robot_self_filter (added below) strips LiDAR returns that
            # hit other robots' bodies; otherwise A's chassis would show
            # up as a moving "wall" cluster in B's local map (and vice
            # versa), pollute /merged_map, and propagate back into the
            # other robot's planning view via map_augmenter.
            ("cloud_in", f"/{ns}/registered_scan_octomap"),
            # Octomap publishes the raw, purely-local-LiDAR-derived map
            # to /{ns}/map_raw. The map_augmenter below folds in cells
            # from the swarm-shared /merged_map that this robot's own
            # LiDAR hasn't observed yet, and republishes the result to
            # /{ns}/map — the topic everyone downstream (astar_nav,
            # safety monitor, RViz) was already consuming. Drop-in
            # replacement; no other node knows or cares.
            ("projected_map", f"/{ns}/map_raw"),
        ] + tf_remaps,
        output="screen",
    )
    actions.append(TimerAction(period=slam_delay + 1.0, actions=[octomap_node]))

    # ── map_augmenter: enrich this robot's local map with swarm-shared
    #    cells from /merged_map. Runs alongside octomap, consumes the
    #    raw octomap output + the merged map, publishes the union as
    #    /{ns}/map. Local cells always win when known; merged cells
    #    only fill the gaps. Architecture mirrors what real-robot
    #    multi-agent deployment looks like: each platform maintains its
    #    own occupancy representation, peer contributions arrive via
    #    /merged_map (or whatever swarm-comm channel replaces it on
    #    real hardware) and get reconciled into the local map.
    map_augmenter_script = str(
        _workspace_root() / "scripts" / "runtime" / "map_augmenter.py"
    )
    actions.append(
        TimerAction(
            period=slam_delay + 2.0,  # after octomap is up
            actions=[
                ExecuteProcess(
                    cmd=["python3", "-u", map_augmenter_script,
                         "--ros-args",
                         "-r", f"__ns:=/{ns}",
                         "-p", "use_sim_time:=true",
                         "-p", "local_map_topic:=map_raw",
                         "-p", "merged_map_topic:=/merged_map",
                         "-p", "augmented_map_topic:=map",
                         "-p", "heartbeat_rate_hz:=1.0"],
                    name=f"map_augmenter_{ns}",
                    output="screen",
                ),
            ],
        )
    )

    if not enable_nav:
        return actions

    # ── A* / Hybrid A* branch: nav_node + twist_bridge (+ hybrid_router on Go2W) ──
    # Both backends share the same I/O contract and supporting infra — they
    # differ only in the search/smoothing core. Selecting executable + yaml
    # by `nav_backend`:
    #   astar         → astar_nav_node          (8-conn A* + reactive Option B)
    #   hybrid_astar  → hybrid_astar_nav_node   (Hybrid A* + OMPL RS + Ceres)
    # Much lighter than FAR — no terrain_analysis, no localPlanner/pathFollower.
    # Uses the same /{ns}/map (from octomap above) directly for planning.
    if nav_backend in ("astar", "hybrid_astar", "nav2_hybrid_astar"):
        if nav_backend == "astar":
            astar_config_yaml = os.path.join(
                go2w_config_pkg, "config", "nav", "astar_nav_go2w.yaml"
            )
            nav_executable = "astar_nav_node"
            nav_node_name  = "astar_nav"
        elif nav_backend == "hybrid_astar":
            astar_config_yaml = os.path.join(
                go2w_config_pkg, "config", "nav", "hybrid_astar_nav_go2w.yaml"
            )
            nav_executable = "hybrid_astar_nav_node"
            nav_node_name  = "hybrid_astar_nav"
        else:  # nav2_hybrid_astar
            astar_config_yaml = os.path.join(
                go2w_config_pkg, "config", "nav", "nav2_hybrid_astar_nav_go2w.yaml"
            )
            nav_executable = "nav2_hybrid_astar_nav_node"
            nav_node_name  = "nav2_hybrid_astar_nav"
        # ── Per-robot body geometry overrides ──
        # The shared yaml has Go2W defaults (wheeled, body 0.65×0.45). Go2
        # menagerie body (robot_b) is narrower and needs different
        # gait-engagement params:
        #
        #   Param                  | Go2W (A)         | Go2 (B)            | Why
        #   -----------------------|------------------|--------------------|------
        #   footprint_length       | 0.65 (yaml)      | 0.65               | body length similar
        #   footprint_width        | 0.45 (yaml)      | 0.30               | Go2 narrower → more clearance
        #   global_inflation_m     | 0.25 (yaml)      | 0.18               | inflation ~= width/2 + safety;
        #                          |                  |                    | B's body is 33% narrower so its
        #                          |                  |                    | inflation should be smaller, not
        #                          |                  |                    | larger. Old 0.30 made B's spawn
        #                          |                  |                    | area (4,-6) pivot_unsafe in all
        #                          |                  |                    | directions because the corridor
        #                          |                  |                    | walls had a 0.30 m phantom buffer
        #                          |                  |                    | wider than the gaps B needed to
        #                          |                  |                    | turn. demo3_mixed 2026-04-26:
        #                          |                  |                    | 17 T1_BRAKE + 7 T2_BACKTRACK on B
        #                          |                  |                    | in 200 s, B moved only 1.9 m total.
        #   legs_pivot_v_floor     | 0.0  (override)  | 0.05 (yaml)        | Go2W cmd_router converts ω to
        #                          |                  |                    | wheel diff; Go2 needs forward
        #                          |                  |                    | creep to engage CHAMP gait
        #
        # Letting B use Go2W's 0.45 body width meant CFPA2 forbade many
        # genuinely-passable corridors and astar's footprint validation
        # rejected paths Go2 could physically take. Letting A use Go2's
        # 0.05 leg-pivot creep meant A's wheel router got 0.05 m/s
        # forward bias during pure turns — a small but persistent
        # forward push that contributed to A drifting into walls.
        body_overrides = {}
        if has_wheels:
            # Go2W (robot_a): keep yaml defaults, just kill the leg
            # creep (yaml has 0.05; Go2W doesn't need it).
            # The legs_pivot_v_floor knob exists only in astar_nav_node's
            # legged-pivot recovery FSM; hybrid_astar_nav_node has no
            # pivot-recovery FSM (Hybrid A* output is kinematically
            # feasible by construction), so the override is astar-only.
            if nav_backend == "astar":
                body_overrides["legs_pivot_v_floor"] = 0.0
        else:
            # Go2 (robot_b): narrower body → smaller inflation halo.
            # Inflation rule of thumb = body_half_width + safety_margin:
            #   A:  0.45/2 + 0.025 = 0.25
            #   B:  0.30/2 + 0.030 = 0.18
            body_overrides["footprint_width"] = 0.30
            # `global_inflation_m` is an astar/hybrid_astar param;
            # nav2_hybrid_astar uses `smoother_inflation_radius_m` instead.
            if nav_backend in ("astar", "hybrid_astar"):
                body_overrides["global_inflation_m"] = 0.18
            else:  # nav2_hybrid_astar
                body_overrides["smoother_inflation_radius_m"] = 0.18
            # Yaml footprint_length (0.85) is sized for Go2W body+head;
            # Go2 menagerie is shorter and B has known-working past runs
            # at 0.65. Two attempts to bump (0.85, 0.70) both clipped at
            # narrow passage (4.51, -3.77) just north of B's spawn,
            # making astar report no_plan_repeated → B parked. Keep B
            # at 0.65 (= prior yaml default before A's head-fix bump);
            # B has not exhibited the head-on-wall climb that motivated
            # the 0.85 increase for A. demo3_mixed 2026-04-26.
            body_overrides["footprint_length"] = 0.65
            # leg_pivot_v_floor stays at yaml's 0.05 (Go2 needs creep).

        # Common remappings for both astar and hybrid_astar. The
        # `astar_nogo_disks` viz topic is only published by astar_nav_node;
        # appended below conditionally.
        nav_remaps = [
            # CFPA2 publishes goals on /{ns}/way_point_coord; the planner
            # subscribes on /way_point (remapped into the namespace).
            ("/way_point", f"/{ns}/way_point_coord"),
            ("/odom/ground_truth", f"/{ns}/odom/nav"),
            ("/scan", f"/{ns}/scan_3d"),
            ("/cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"),
            ("/nav_status", f"/{ns}/nav_status"),
            ("/planned_path", f"/{ns}/planned_path"),
            ("/global_planned_path", f"/{ns}/global_planned_path"),
            ("/robot_trajectory", f"/{ns}/robot_trajectory"),
            ("/final_goal_marker", f"/{ns}/final_goal_marker"),
            ("/robot_pose_marker", f"/{ns}/robot_pose_marker"),
            ("/frontier_replan", f"/{ns}/frontier_replan"),
            ("/stop", f"/{ns}/stop"),
        ]
        if nav_backend == "astar":
            nav_remaps.append(("/astar_nogo_disks", f"/{ns}/astar_nogo_disks"))

        astar_nodes = [
            Node(
                package="go2w_nav",
                executable=nav_executable,
                namespace=ns,
                name=nav_node_name,
                parameters=[
                    astar_config_yaml,
                    {"use_sim_time": use_sim_time},
                    body_overrides,
                    {
                        "map_frame": "map",
                        # base_frame is per-robot — robot_a uses bare URDF
                        # (base_link) but robot_b's URDF is `b_`-prefixed
                        # via build_robot_b_urdf, so its base is `b_base_link`.
                        # astar_nav_node looks up `map → base_frame_` in TF;
                        # passing the wrong frame leaves it stuck in
                        # `warming_up:no_tf` and no plan ever publishes.
                        "base_frame": base_frame,
                        "map_topic": f"/{ns}/map",
                        "frontier_replan_topic": f"/{ns}/frontier_replan",
                        "stop_topic": f"/{ns}/stop",
                    },
                ],
                remappings=nav_remaps + tf_remaps,
                output="screen",
            ),
            Node(
                package="go2w_perception",
                executable="twist_bridge.py",
                namespace=ns,
                name="twist_bridge",
                remappings=[
                    ("/cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"),
                    ("/cmd_vel", f"/{ns}/cmd_vel"),
                ],
                output="screen",
            ),
        ]
        if has_wheels:
            # Go2W: hybrid_cmd_router splits cmd_vel into cmd_vel_legged (CHAMP)
            # and wheel velocity commands. In the mixed launch the ros2_control
            # controller_manager lives under /mujoco_sim, NOT /{ns}, so the
            # wheel controller publishes/listens at
            #     /mujoco_sim/{ns}_wheel_velocity_controller/commands
            # Pass that as an ABSOLUTE topic so the router's namespace (robot_a)
            # doesn't prepend. Without the leading slash the router publishes
            # to /robot_a/robot_a_wheel_velocity_controller/commands where
            # nobody listens → wheels silent → robot stuck the whole time
            # hybrid mode is "wheel".
            astar_nodes.append(
                Node(
                    package="go2w_control",
                    executable="go2w_hybrid_cmd_router.py",
                    namespace=ns,
                    name="go2w_hybrid_cmd_router",
                    parameters=[
                        os.path.join(
                            go2w_config_pkg, "config", "control",
                            "go2w_hybrid_motion.yaml",
                        ),
                        {
                            "use_sim_time": use_sim_time,
                            "wheel_command_topic":
                                f"/mujoco_sim/{ns}_wheel_velocity_controller/commands",
                        },
                    ],
                    output="screen",
                )
            )
        actions.append(TimerAction(period=nav_delay, actions=astar_nodes))
        return actions

    # ── Nav2 MPPI branch: planner_server + controller_server + ─────────
    # behavior_server + bt_navigator + lifecycle_manager + the
    # CFPA2-way_point → Nav2-goal_pose bridge + hybrid_cmd_router. The
    # full Nav2 stack runs INSIDE this namespace via PushRosNamespace +
    # RewrittenYaml so per-robot params load correctly. CFPA2 is still
    # the goal source (via /<ns>/way_point_coord); the bridge synthesises
    # an orientation pointing toward the goal and republishes as
    # PoseStamped on /<ns>/goal_pose for bt_navigator to pick up.
    if nav_backend == "nav2_mppi":
        from launch_ros.actions import PushRosNamespace
        from nav2_common.launch import RewrittenYaml

        # Per-platform Nav2 yaml. Go2W (has_wheels=True) uses the wheeled
        # config (0.70 × 0.40 m footprint, vx_max=0.50, MPPI tuned for
        # skid-steer wheel mode). Go2 (legged-only) uses the slimmer
        # 0.65 × 0.30 m footprint, vx_max=0.30, minimum_turning_radius=
        # 0.05 (CHAMP can pivot in place). Same plugins (SmacHybrid +
        # MPPI), different geometry/velocity caps.
        _nav2_yaml_filename = (
            "nav2_go2w_full_stack.yaml" if has_wheels
            else "nav2_go2_full_stack.yaml"
        )
        nav2_yaml = os.path.join(
            go2w_config_pkg, "config", "nav", _nav2_yaml_filename
        )
        rewritten_nav2 = RewrittenYaml(
            source_file=nav2_yaml,
            root_key=ns,
            param_rewrites={"use_sim_time": str(use_sim_time).lower()},
            convert_types=True,
        )

        nav2_inner_nodes = [
            PushRosNamespace(ns),
            Node(
                package="nav2_controller", executable="controller_server",
                name="controller_server",
                parameters=[rewritten_nav2],
                remappings=tf_remaps, output="screen",
            ),
            Node(
                package="nav2_planner", executable="planner_server",
                name="planner_server",
                parameters=[rewritten_nav2],
                remappings=tf_remaps, output="screen",
            ),
            Node(
                package="nav2_behaviors", executable="behavior_server",
                name="behavior_server",
                parameters=[rewritten_nav2],
                remappings=tf_remaps, output="screen",
            ),
            Node(
                package="nav2_bt_navigator", executable="bt_navigator",
                name="bt_navigator",
                parameters=[rewritten_nav2],
                remappings=tf_remaps, output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager", executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[rewritten_nav2],
                output="screen",
            ),
        ]

        # CFPA2 → Nav2 bridge: subscribes /<ns>/way_point_coord
        # (PointStamped, RELIABLE), publishes /<ns>/goal_pose (PoseStamped,
        # BEST_EFFORT to match bt_navigator's QoS). Lives outside an
        # installed package so we run it via ExecuteProcess with an
        # absolute path.
        bridge_path = os.path.join(_ws_root, "scripts/runtime/cfpa2_to_nav2_bridge.py")
        bridge_node = ExecuteProcess(
            cmd=[
                "python3", "-u", bridge_path,
                "--ros-args",
                "-p", f"namespace:={ns}",
                "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
                "-p", "waypoint_topic:=way_point_coord",
            ],
            name=f"cfpa2_to_nav2_bridge_{ns}",
            output="screen",
        )

        # Path relay: rename Nav2's /plan → /planned_path so the existing
        # nav_test_mixed.rviz config (carried over from astar_nav layout)
        # picks up the visualisation without RViz config changes.
        path_relay_path = os.path.join(_ws_root, "scripts/runtime/path_relay.py")
        path_relay_node = ExecuteProcess(
            cmd=[
                "python3", "-u", path_relay_path,
                "--ros-args",
                "-p", f"namespace:={ns}",
                "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
            ],
            name=f"path_relay_{ns}",
            output="screen",
        )

        # stuck_watchdog: outer-loop self-recovery. Detects 10 s of no
        # motion under an active goal, fires a Nav2 BackUp action then
        # republishes the goal so SmacHybrid replans from the new
        # (post-backup) pose. Pivot-lock and pivot-stuck-on-tight-turn
        # are both handled by this — the robot physically backs up by
        # 0.40 m, clearance recomputes, the new plan can include
        # forward+reverse Reeds-Shepp segments. Mirrors Nav2's built-in
        # Backup recovery behaviour, but triggered by EXTERNAL motion
        # check rather than a controller-reported failure (MPPI in
        # narrow-pivot scenarios outputs v≈ω≈0 without ever reporting
        # failure, so the BT never enters its recovery branch).
        stuck_watchdog_path = os.path.join(_ws_root, "scripts/runtime/stuck_watchdog.py")
        stuck_watchdog_node = ExecuteProcess(
            cmd=[
                "python3", "-u", stuck_watchdog_path,
                "--ros-args",
                "-p", f"namespace:={ns}",
                "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
            ],
            name=f"stuck_watchdog_{ns}",
            output="screen",
        )

        peer_obstacle_node = None
        if peer_obstacle_enabled and peer_namespaces:
            peer_ns = str(peer_namespaces[0]).strip("/")
            peer_base_frame = base_frame
            peer_obstacle_node = Node(
                package="reconstruction_awareness",
                executable="peer_obstacle_scan_node",
                name=f"peer_obstacle_scan_{ns}",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "namespace": ns,
                    "peer_namespace": peer_ns,
                    "base_frame": peer_base_frame,
                    # Was 0.55 (1.10 m diameter). Smoke 2026-04-30 stage-1
                    # caught two robots freezing inside a 0.425 m corridor
                    # because the peer disc + 0.45 m local_inflation made
                    # every MPPI sample infeasible. 0.30 = peer body
                    # half-width 0.20 + 0.10 m peer-pose latency buffer
                    # (peer_pose_stale_sec ≤ 0.5 → ≤ 0.15 m worst-case lag
                    # at v=0.30 m/s). Diameter 0.60 m leaves ~0.025 m
                    # clearance in the narrowest demo3_mixed corridor —
                    # tight but feasible.
                    "peer_radius_m": 0.30,
                    "range_cap_m": 6.0,
                    "publish_rate_hz": 10.0,
                    "stale_timeout_sec": 0.5,
                }],
                output="screen",
            )

        # hybrid_cmd_router: only relevant for Go2W (it splits cmd_vel
        # into wheel vs legged based on curvature). Go2 (no wheels) has
        # no `*_wheel_velocity_controller` to publish to; CHAMP for
        # robot_b listens to /<ns>/cmd_vel directly (the /cmd_vel/smooth
        # remap is set in the per-namespace champ launch). So for
        # legged-only platforms we skip the router entirely.
        if has_wheels:
            router_node = Node(
                package="go2w_control",
                executable="go2w_hybrid_cmd_router.py",
                namespace=ns,
                name="go2w_hybrid_cmd_router",
                parameters=[
                    os.path.join(go2w_config_pkg, "config", "control",
                                 "go2w_hybrid_motion.yaml"),
                    {
                        "use_sim_time": use_sim_time,
                        "wheel_command_topic":
                            f"/mujoco_sim/{ns}_wheel_velocity_controller/commands",
                    },
                ],
                output="screen",
            )
        else:
            router_node = None

        # ros2 bag recorder — captures the full cmd_vel chain so a
        # post-mortem of any wall-hit can show whether MPPI was
        # commanding forward (planner / critic issue) or reverse
        # (inertia issue). Records to /tmp/nav2_run_{ns}/ as MCAP.
        # Tear off the previous run's bag dir so we don't append.
        # Wheel-controller topic + cmd_vel_legged + mobility_mode are
        # only relevant for Go2W (has_wheels). For Go2 we drop them and
        # keep just cmd_vel + plan + waypoint + goal + collisions.
        bag_dir = f"/tmp/nav2_run_{ns}"
        _wheel_bag_topics = (
            f"/{ns}/cmd_vel_legged "
            f"/mujoco_sim/{ns}_wheel_velocity_controller/commands "
            f"/{ns}/mobility_mode "
        ) if has_wheels else ""
        bag_record = ExecuteProcess(
            cmd=[
                "bash", "-lc",
                f"rm -rf {bag_dir} && "
                "ros2 bag record "
                f"-o {bag_dir} "
                f"/{ns}/cmd_vel "
                f"{_wheel_bag_topics}"
                # CHAMP-side joint cmd: trajectory_msgs/JointTrajectory
                # carrying hip/thigh/calf positions. If CHAMP commands
                # large positions while cmd_vel=0, that explains body
                # drift via leg-pose changes.
                f"/mujoco_sim/{ns}_joint_group_effort_controller/joint_trajectory "
                # actual realised joint states from MuJoCo (post-actuator).
                # Compare with commands above to see if ros2_control is
                # tracking. If joint_states.velocity for foot_joints stays
                # nonzero despite wheel cmd=0, the velocity actuator is
                # being bypassed.
                "/mujoco_sim/joint_states "
                f"/{ns}/odom/nav "
                f"/{ns}/plan "
                f"/{ns}/way_point_coord "
                f"/{ns}/goal_pose "
                "/collision_events"
            ],
            name=f"nav2_bag_record_{ns}",
            output="screen",
        )

        # Wrap nav2 inner-nodes in a GroupAction so PushRosNamespace
        # scopes to them only; bridge + router are separate Node entries
        # at root namespace (their own namespace= arg handles scoping).
        from launch.actions import GroupAction
        _nav2_actions = [
            GroupAction(actions=nav2_inner_nodes),
            # Bridge + relay + watchdog run at root NS; they compute
            # namespaced topics internally from their `namespace`
            # parameter.
            bridge_node,
            path_relay_node,
            stuck_watchdog_node,
            bag_record,
        ]
        if peer_obstacle_node is not None:
            _nav2_actions.insert(4, peer_obstacle_node)
        if router_node is not None:
            _nav2_actions.insert(3, router_node)  # before bag_record
        actions.append(
            TimerAction(period=nav_delay, actions=_nav2_actions)
        )
        return actions

    if nav_backend != "far":
        return actions  # unknown backend — skip nav

    # ── FAR stack: sensor_scan_generation, terrain_analysis ×2, far_planner,
    #    localPlanner, pathFollower ──
    far_scan_topic = f"/{ns}/registered_scan_map"
    far_odom_topic = f"/{ns}/odom/nav"
    far_max_speed = 0.2

    far_nodes = [
        Node(
            package="sensor_scan_generation",
            executable="sensorScanGeneration",
            namespace=ns,
            name="sensor_scan_generation",
            arguments=["--ros-args", "--log-level", "WARN"],
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[
                ("/state_estimation", far_odom_topic),
                ("/registered_scan", far_scan_topic),
                ("/state_estimation_at_scan", f"/{ns}/state_estimation_at_scan"),
                ("/sensor_scan", f"/{ns}/sensor_scan"),
            ] + tf_remaps,
            output="screen",
        ),
        Node(
            package="terrain_analysis",
            executable="terrainAnalysis",
            namespace=ns,
            name="terrain_analysis",
            arguments=["--ros-args", "--log-level", "WARN"],
            parameters=[{"use_sim_time": use_sim_time, "maxRelZ": 0.8}],
            remappings=[
                ("/state_estimation", far_odom_topic),
                ("/registered_scan", far_scan_topic),
                ("/joy", f"/{ns}/joy"),
                ("/map_clearing", f"/{ns}/map_clearing"),
                ("/terrain_map", f"/{ns}/terrain_map"),
            ],
            output="screen",
        ),
        Node(
            package="terrain_analysis_ext",
            executable="terrainAnalysisExt",
            namespace=ns,
            name="terrain_analysis_ext",
            arguments=["--ros-args", "--log-level", "WARN"],
            parameters=[{"use_sim_time": use_sim_time, "maxRelZ": 0.8}],
            remappings=[
                ("/state_estimation", far_odom_topic),
                ("/registered_scan", far_scan_topic),
                ("/joy", f"/{ns}/joy"),
                ("/cloud_clearing", f"/{ns}/cloud_clearing"),
                ("/terrain_map", f"/{ns}/terrain_map"),
                ("/terrain_map_ext", f"/{ns}/terrain_map_ext"),
            ],
            output="screen",
        ),
        Node(
            package="far_planner",
            executable="far_planner",
            namespace=ns,
            name="far_planner",
            parameters=[
                _load_yaml_params(far_default_yaml),
                # Must go through _load_yaml_params(): the tuning yaml's top
                # key is `far_planner:` which only matches an unnamespaced
                # node. Under `/robot_a/` ROS 2 silently drops every override.
                # _load_yaml_params() strips the node-name key and returns a
                # flat dict, which applies regardless of namespace.
                _load_yaml_params(far_tuning_yaml),
                {
                    "use_sim_time": use_sim_time,
                    # robot_id is an unused launch param in FAR source
                    # (see graph_msger.cpp:99) — set for future-proofing.
                    "graph_msger/robot_id": 0 if ns == "robot_a" else 1,
                },
            ],
            remappings=[
                ("/odom_world", far_odom_topic),
                ("/terrain_cloud", f"/{ns}/terrain_map_ext"),
                ("/scan_cloud", f"/{ns}/terrain_map"),
                ("/terrain_local_cloud", far_scan_topic),
                ("/goal_point", f"/{ns}/way_point_coord"),
                ("/way_point", f"/{ns}/way_point"),
                ("/joy", f"/{ns}/joy"),
                ("/navigation_boundary", f"/{ns}/navigation_boundary"),
                ("/runtime", f"/{ns}/far_runtime"),
                ("/planning_time", f"/{ns}/far_planning_time"),
                ("/robot_vgraph", f"/{ns}/robot_vgraph"),
                ("/decoded_vgraph", f"/{ns}/decoded_vgraph"),
            ] + tf_remaps,
            output="screen",
        ),
        # Bridge FAR's native outputs → nav_status/v1 so CFPA2 can
        # fast-blacklist goals whose V-graph connection orphans (the
        # `reach_nav_node == NULL` path in graph_planner.cpp:239 leaves
        # FAR silent on /way_point without ever signalling failure).
        Node(
            package="go2w_nav",
            executable="far_status_adapter.py",
            namespace=ns,
            name="far_status_adapter",
            parameters=[{
                "use_sim_time": use_sim_time,
                "way_point_timeout_sec": 2.0,
                "unreachable_timeout_sec": 3.0,
                "far_heartbeat_timeout_sec": 5.0,
                "publish_rate_hz": 5.0,
            }],
            remappings=[
                ("/nav_status", f"/{ns}/nav_status"),
                ("/far_reach_goal_status", f"/{ns}/far_reach_goal_status"),
                ("/goal_point", f"/{ns}/way_point_coord"),
                ("/way_point", f"/{ns}/way_point"),
                ("/far_planning_time", f"/{ns}/far_planning_time"),
                ("/odom/ground_truth", far_odom_topic),
            ],
            output="screen",
        ),
        Node(
            package="local_planner",
            executable="localPlanner",
            namespace=ns,
            name="localPlanner",
            # Config aligned with the proven single-robot FAR config in
            # navigation.launch.py (Config A, 7/10 demo1 PASS). The old
            # mixed-launch numbers (obstacleHeightThre=0.20, maxRelZ=1.2,
            # vehicleLength=0.70) were classifying too many terrain points
            # as obstacles and/or over-constraining rotation primitives,
            # so every path-library sweep failed validation → localPlanner
            # emitted the degenerate single-pose path → pathFollower stuck
            # at zero velocity. Key changes vs. pre-2026-04-24:
            #   obstacleHeightThre: 0.20 → 0.50  (real walls only, not
            #       low ground noise from LiDAR tilt)
            #   maxRelZ:            1.2  → 0.25  (ignore ceiling/tall
            #       wall tops; keep only near-ground cloud)
            #   vehicleLength×Width 0.70×0.40 → 0.3×0.7 (CMU default, Go2
            #       footprint approx; swap dims so rotation swept area
            #       is narrower across the body)
            #   checkRotObstacle:   True → False (Config A kept it True
            #       for tight demo1 walls but for demo3_mixed open areas
            #       it over-rejects; we rely on terrain-driven obstacle
            #       check instead)
            parameters=[{
                "use_sim_time": use_sim_time,
                "pathFolder": local_planner_paths_dir,
                "vehicleLength": 0.3,
                "vehicleWidth": 0.7,
                "sensorOffsetX": 0.0,
                "sensorOffsetY": 0.0,
                # True matches Config-A demo1 (twoWayDrive+checkRotObstacle+
                # obs_inflate=2 is the 7/10 PASS combo). Earlier live run
                # with checkRotObstacle=True oscillated in place because
                # obs_inflate_size=0 let FAR route along walls and every
                # rotation primitive clipped — fixing FAR inflation
                # unblocks this path.
                # twoWayDrive disabled 2026-04-26: B spent 60+ s reversing
                # at -0.19 m/s along cross_v_s, leg-scuffing the wall on the
                # way back; A reversed into corners during recovery turns.
                # Forward-only forces FAR/pathFollower to plan a real
                # turn-around when blocked, surfacing stuck conditions to
                # CFPA2 instead of grinding backward through clutter.
                "twoWayDrive": False,
                "laserVoxelSize": 0.05,
                "terrainVoxelSize": 0.2,
                "useTerrainAnalysis": True,
                "checkObstacle": True,
                # Restored to True — with obstacleHeightThre=0.50 (was
                # 0.20) rotation primitives can now find valid free sweeps
                # where before every sweep was blocked. Keeping this True
                # matches Config A and prevents rear/side scraping during
                # in-place turns (the "FAR path into wall" failure mode).
                "checkRotObstacle": True,
                "adjacentRange": 3.0,
                # Lowered 0.50 → 0.20 to match the FAR terrain_free_Z=0.15
                # fix (must be > terrain_free_Z per CMU docs). Demo3_mixed
                # walls at long range project to low intensity; 0.50 was
                # letting wall points through as "ground" and localPlanner
                # then approved path primitives that clip the wall.
                # Lowered to 0.02 (2026-04-26) for zero-scuff. With 0.10,
                # cross_h_e wall base at z=0.08 was below threshold →
                # localPlanner saw the wall as ground, drove A's head
                # straight INTO it, then climbed (tilt 56°, 1100+
                # contacts). 0.02 catches anything with intensity > 2 cm
                # — well above MuJoCo flat-ground noise floor (mathematic
                # ground plane, no physics jitter). Real-robot floor is
                # 0.10-0.50 m noise so this only works in sim.
                "obstacleHeightThre": 0.02,
                "groundHeightThre": 0.1,
                "costHeightThre": 0.1,
                # Cost-based path scoring enabled (was useCost=False).
                # localPlanner picks among precomputed path candidates;
                # without cost, it picks the shortest one regardless of
                # how close it shaves walls. costScore 0.10 (was 0.02)
                # adds a meaningful clearance preference: paths near
                # obstacles get penalized so the planner picks a wider
                # arc when one exists.
                "costScore": 0.10,
                "useCost": True,
                # Lowered 2 → 1 (2026-04-26) for zero-scuff. CMU's
                # localPlanner only blocks a path candidate if ≥
                # pointPerPathThre obstacle points fall inside the
                # candidate's swept volume. At 24 m range, Mid-360's
                # angular resolution leaves sparse hits on far walls
                # (often 0-1 pts/voxel); with threshold 2, A's path
                # to wall_east had only 1 wall hit and slid through.
                # Threshold 1 = "any obstacle point blocks the path",
                # which is what we want for zero-scuff in clean
                # MuJoCo. Real-robot would need 2 to reject sensor
                # noise.
                "pointPerPathThre": 1,
                "minRelZ": -0.5,
                # Bumped 0.25 → 0.8 (2026-04-26) for zero-scuff. With
                # 0.25 the localPlanner dropped any LiDAR voxel >25 cm
                # above local ground from its obstacle map. Mid-360 (or
                # sim equivalent) mounted at ~0.4 m height scans far
                # walls (24 m east in demo3_mixed) at z=0.3-0.5 m due
                # to grazing angles → those wall points filtered → far
                # walls invisible to localPlanner → A's RViz path went
                # straight through wall_east. Bumping to 0.8 matches
                # terrainAnalysis's own maxRelZ so the local obstacle
                # map sees the same walls FAR's V-graph does.
                "maxRelZ": 0.8,
                "maxSpeed": far_max_speed,
                "dirWeight": 0.02,
                "dirThre": 90.0,
                "dirToVehicle": False,
                "pathScale": 0.75,
                "minPathScale": 0.5,
                "pathScaleStep": 0.25,
                "pathScaleBySpeed": True,
                "minPathRange": 1.0,
                "pathRangeStep": 0.5,
                "pathRangeBySpeed": True,
                "pathCropByGoal": True,
                "autonomyMode": True,
                "autonomySpeed": far_max_speed,
                "joyToSpeedDelay": 2.0,
                "joyToCheckObstacleDelay": 5.0,
                "goalClearRange": 0.5,
                "goalX": 0.0,
                "goalY": 0.0,
            }],
            remappings=[
                ("/state_estimation", far_odom_topic),
                ("/registered_scan", far_scan_topic),
                ("/way_point", f"/{ns}/way_point"),
                ("/terrain_map", f"/{ns}/terrain_map"),
                ("/overall_map", f"/{ns}/terrain_map"),
                ("/joy", f"/{ns}/joy"),
                # Route localPlanner's chosen path through the safety
                # filter (octomap hard-collision shield) before
                # pathFollower consumes it. See path_safety_filter.py.
                ("/path", f"/{ns}/local_path_raw"),
                ("/freePaths", f"/{ns}/free_paths"),
            ],
            output="screen",
        ),
        # ── path_safety_filter: octomap-based hard collision shield ──
        # Sits between localPlanner (publishes raw chosen path) and
        # pathFollower (consumes safe path). Truncates or rejects path
        # if any pose's footprint intersects an occupied cell in /map.
        # Trade-off: occasional "no path" stalls vs zero wall-crossing
        # execution. CFPA2's stuck detection eventually reassigns goal
        # if FAR keeps emitting blocked paths.
        Node(
            package="go2w_nav",
            executable="path_safety_filter.py",
            namespace=ns,
            name="path_safety_filter",
            # CRITICAL: namespace TF — without /tf → /{ns}/tf the filter's
            # TF lookup (vehicle → map) silently fails and every path is
            # passthrough'd unchecked. We discovered this only because
            # the filter's status published "passthrough_no_tf" — RViz
            # then showed local_path crossing walls because filter never
            # truncated.
            remappings=tf_remaps,
            parameters=[{
                "use_sim_time": use_sim_time,
                # 0.65 m radius. Go2W body diagonal half = 0.50, head
                # extends 0.10-0.15 m forward of body, plus 0.05 m
                # turning margin = 0.65. 0.55 still let head_lower
                # graze ne_div_h_west at z=0.30 m. Per-pose disk check
                # is conservative-pessimistic in tight corridors —
                # we accept some narrow-passage rejections in exchange
                # for actual zero-contact.
                # 0.50 m radius — middle ground. 0.65 + occ_thr=30
                # rejected too aggressively (everything within
                # inflation halo blocked → cov stuck at 46.9% / 100 s).
                # 0.50 covers Go2W body + small wheel-margin without
                # being so wide that narrow corridors are unpassable.
                "footprint_radius_m": 0.50,
                # Restored 50: only count clearly-occupied cells (the
                # actual wall), not inflation halos. Filter's job is
                # to catch FAR's "path through wall" topology errors,
                # not to enforce extra clearance (that's localPlanner's
                # cost-based scoring).
                "occ_threshold": 50,
                "check_stride_m": 0.05,
                "min_safe_path_len": 1,
                "input_topic": f"/{ns}/local_path_raw",
                "output_topic": f"/{ns}/local_path",
                "map_topic": f"/{ns}/map",
                "status_topic": f"/{ns}/path_safety_status",
                # Fall back to base_link if vehicle frame missing.
                "base_frame_fallback": base_frame,
            }],
            output="screen",
        ),
        Node(
            package="local_planner",
            executable="pathFollower",
            namespace=ns,
            name="pathFollower",
            # Aligned with Config-A demo1 pathFollower: twoWayDrive=True
            # (reverse recovery), stopDisThre=0.15, dirDiffThre=0.4.
            parameters=[{
                "use_sim_time": use_sim_time,
                "sensorOffsetX": 0.0,
                "sensorOffsetY": 0.0,
                "pubSkipNum": 1,
                # twoWayDrive disabled 2026-04-26: B spent 60+ s reversing
                # at -0.19 m/s along cross_v_s, leg-scuffing the wall on the
                # way back; A reversed into corners during recovery turns.
                # Forward-only forces FAR/pathFollower to plan a real
                # turn-around when blocked, surfacing stuck conditions to
                # CFPA2 instead of grinding backward through clutter.
                "twoWayDrive": False,
                "lookAheadDis": 0.5,
                "yawRateGain": 1.5,
                "stopYawRateGain": 1.5,
                "maxYawRate": 80.0,
                "maxSpeed": far_max_speed,
                "maxAccel": 2.0,
                "switchTimeThre": 1.0,
                "dirDiffThre": 0.4,
                "omniDirDiffThre": 1.5,
                "noRotSpeed": 10.0,
                "stopDisThre": 0.15,
                "slowDwnDisThre": 0.75,
                "useInclRateToSlow": False,
                "inclRateThre": 120.0,
                "slowRate1": 0.25,
                "slowRate2": 0.5,
                "slowTime1": 2.0,
                "slowTime2": 2.0,
                "useInclToStop": False,
                "inclThre": 45.0,
                "stopTime": 5.0,
                "noRotAtStop": False,
                "noRotAtGoal": True,
                "autonomyMode": True,
                "autonomySpeed": far_max_speed,
                "joyToSpeedDelay": 2.0,
                "goalCloseDis": 0.4,
                "is_real_robot": False,
            }],
            remappings=[
                ("/state_estimation", far_odom_topic),
                ("/path", f"/{ns}/local_path"),
                # Route cmd_vel through cmd_vel_safety_shield before
                # twist_bridge / hybrid router. See shield wiring below.
                ("/cmd_vel", f"/{ns}/cmd_vel_stamped_raw"),
                ("/joy", f"/{ns}/joy"),
                ("/speed", f"/{ns}/speed"),
                ("/stop", f"/{ns}/stop"),
            ],
            output="screen",
        ),
        # ── cmd_vel_safety_shield: execution-time clearance check ──
        # Kills angular cmd when robot is hugging a wall (clearance disk
        # within /map intersects an occupied cell). Translation passes
        # through so the robot can drive out of the corridor; only
        # in-place rotation that would sweep wheels through a wall is
        # suppressed. Mirrors CFPA2's pivot-lock at execution layer so
        # held goals demanding rotation can't cause wall scraping.
        Node(
            package="go2w_nav",
            executable="cmd_vel_safety_shield.py",
            namespace=ns,
            name="cmd_vel_safety_shield",
            # CRITICAL: same TF remap reason as path_safety_filter above
            # — without it, shield can't transform base_link to map and
            # falls back to odom-only pose (no yaw → footprint check
            # broken). This was the silent failure that let A scuff
            # ne_div_v_north for 240+ ticks.
            remappings=tf_remaps,
            parameters=[{
                "use_sim_time": use_sim_time,
                "clearance_radius_m": 0.50,
                "occ_threshold": 50,
                "angular_kill_threshold_rad_s": 0.10,
                # Predictive footprint check: simulate requested ω over
                # this horizon, kill ω only if rotated body clips.
                # 0.4 s × max ω 1.4 rad/s ≈ 32° sweep — enough lookahead
                # to catch corner sweeps but small enough to allow
                # genuine away-from-wall rotation.
                "predict_horizon_sec": 0.4,
                # Footprint per-robot (Go2W wider than Go2)
                "footprint_length_m": 0.85 if has_wheels else 0.65,
                "footprint_width_m":  0.45 if has_wheels else 0.30,
                "input_topic": f"/{ns}/cmd_vel_stamped_raw",
                "output_topic": f"/{ns}/cmd_vel_stamped",
                "map_topic": f"/{ns}/map",
                "odom_topic": f"/{ns}/odom/nav",
                "status_topic": f"/{ns}/cmd_vel_shield_status",
                "base_frame": base_frame,
                "map_frame": "map",
                "use_tf": True,
            }],
            output="screen",
        ),
        # CMU convention: static sensor↔vehicle + sensor↔camera TFs
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=ns,
            name="far_vehicle_tf",
            arguments=["0", "0", "0", "0", "0", "0", "sensor", "vehicle"],
            remappings=tf_remaps,
            output="screen",
        ),
        # Bridge "vehicle" frame into the main TF tree. CMU's stack
        # only publishes sensor→vehicle, but in our SLAM tree "sensor"
        # isn't connected to map (we use sensor_at_scan from slam_relay
        # plus odom→base_link). Without this bridge, lookup_transform
        # (map ↔ vehicle) fails — path_safety_filter then silently
        # passthroughs every path because it can't transform vehicle-
        # frame poses to map-frame for collision check.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=ns,
            name="base_link_to_vehicle_bridge",
            arguments=["0", "0", "0", "0", "0", "0", base_frame, "vehicle"],
            remappings=tf_remaps,
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=ns,
            name="far_camera_tf",
            arguments=["0", "0", "0", "-1.5707963", "0", "-1.5707963", "sensor", "camera"],
            remappings=tf_remaps,
            output="screen",
        ),
        # twist_bridge: cmd_vel_stamped → cmd_vel for CHAMP
        Node(
            package="go2w_perception",
            executable="twist_bridge.py",
            namespace=ns,
            name="twist_bridge",
            remappings=[
                ("/cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"),
                ("/cmd_vel", f"/{ns}/cmd_vel"),
            ],
            output="screen",
        ),
    ]
    if has_wheels:
        # Go2W: route cmd_vel → legged controller + wheel controller via hybrid router.
        # Wheel topic must be ABSOLUTE to /mujoco_sim (see astar branch above
        # for the full explanation). Otherwise the router publishes to
        # /{ns}/wheel_velocity_controller/commands which nobody listens to.
        far_nodes.append(
            Node(
                package="go2w_control",
                executable="go2w_hybrid_cmd_router.py",
                namespace=ns,
                name="go2w_hybrid_cmd_router",
                parameters=[
                    os.path.join(
                        go2w_config_pkg, "config", "control",
                        "go2w_hybrid_motion.yaml",
                    ),
                    {
                        "use_sim_time": use_sim_time,
                        "wheel_command_topic":
                            f"/mujoco_sim/{ns}_wheel_velocity_controller/commands",
                    },
                ],
                output="screen",
            )
        )
    # else: Go2 — CHAMP subscribes to /{ns}/cmd_vel directly (see
    # `cmd_vel_input_topic="cmd_vel"` in the robot_b build_dual_robot_stack
    # call). No hybrid router / no relay needed.
    actions.append(TimerAction(period=nav_delay, actions=far_nodes))

    return actions


def _launch_setup(context):
    use_sim_time = True
    gui = _as_bool(_get(context, "gui"))
    rviz = _as_bool(_get(context, "rviz"))
    explore = _as_bool(_get(context, "explore"))
    slam_only = _as_bool(_get(context, "slam_only"))
    cleanup_stale = _as_bool(_get(context, "cleanup_stale"))
    mujoco_cameras = _as_bool(_get(context, "mujoco_cameras"))
    debug = _as_bool(_get(context, "debug"))
    loop_closure_on = _as_bool(_get(context, "loop_closure"))
    loop_closure_backend = (_get(context, "loop_closure_backend").strip().lower() or "auto")
    deployment_mode = (_get(context, "deployment_mode").strip().lower() or "sim_ros2")
    slam_backend = (_get(context, "slam_backend").strip().lower() or "fast_lio_scpgo")
    dynamic_filter_backend = (
        _get(context, "dynamic_filter_backend").strip().lower() or "none"
    )
    static_map_cleanup_backend = (
        _get(context, "static_map_cleanup_backend").strip().lower() or "none"
    )
    erasor_trigger_mode = (_get(context, "erasor_trigger_mode").strip().lower() or "manual")
    require_swarm_loop_agreement = _as_bool(_get(context, "require_swarm_loop_agreement"))
    swarm_loop_agreement_max_translation = float(
        _get(context, "swarm_loop_agreement_max_translation").strip() or "0.5"
    )
    swarm_loop_agreement_max_yaw_deg = float(
        _get(context, "swarm_loop_agreement_max_yaw_deg").strip() or "5.0"
    )
    map_merge_enabled = _as_bool(_get(context, "map_merge"))
    relative_pose_source = (_get(context, "relative_pose_source").strip().lower() or "none")
    inter_robot_loop_closure = _as_bool(_get(context, "inter_robot_loop_closure"))
    team_alignment_min_matches = int(_get(context, "team_alignment_min_matches").strip() or "2")
    team_alignment_max_translation_disagreement = float(
        _get(context, "team_alignment_max_translation_disagreement").strip() or "1.0"
    )
    team_alignment_max_yaw_disagreement_deg = float(
        _get(context, "team_alignment_max_yaw_disagreement_deg").strip() or "10.0"
    )
    team_alignment_single_match_debug = _as_bool(_get(context, "team_alignment_single_match_debug"))
    team_alignment_enable_map_merge = _as_bool(_get(context, "team_alignment_enable_map_merge"))
    enable_gt_drift_metrics = _as_bool(_get(context, "enable_gt_drift_metrics"))
    scan_context_num_rings = int(_get(context, "scan_context_num_rings").strip() or "20")
    scan_context_num_sectors = int(_get(context, "scan_context_num_sectors").strip() or "60")
    scan_context_max_radius = float(_get(context, "scan_context_max_radius").strip() or "80.0")
    registration_backend = (_get(context, "registration_backend").strip().lower() or "icp_2d")
    robust_min_inliers = int(_get(context, "robust_min_inliers").strip() or "7")
    robust_min_inlier_ratio = float(_get(context, "robust_min_inlier_ratio").strip() or "0.25")
    robust_max_median_rmse = float(_get(context, "robust_max_median_rmse").strip() or "0.45")
    robust_max_translation_spread_m = float(
        _get(context, "robust_max_translation_spread_m").strip() or "1.0"
    )
    robust_max_yaw_spread_deg = float(_get(context, "robust_max_yaw_spread_deg").strip() or "12.0")
    robust_prefilter_max_rmse = float(_get(context, "robust_prefilter_max_rmse").strip() or "0.45")
    robust_prefilter_min_inlier_ratio = float(
        _get(context, "robust_prefilter_min_inlier_ratio").strip() or "0.35"
    )
    robust_prefilter_min_correspondences = int(
        _get(context, "robust_prefilter_min_correspondences").strip() or "0"
    )
    robust_prefilter_max_descriptor_distance = float(
        _get(context, "robust_prefilter_max_descriptor_distance").strip() or "0.45"
    )
    robust_deduplicate_by_query_keyframe = _as_bool(
        _get(context, "robust_deduplicate_by_query_keyframe")
    )
    robust_deduplicate_by_match_keyframe = _as_bool(
        _get(context, "robust_deduplicate_by_match_keyframe")
    )
    robust_deduplicate_transform_bin_translation_m = float(
        _get(context, "robust_deduplicate_transform_bin_translation_m").strip() or "0.0"
    )
    robust_deduplicate_transform_bin_yaw_deg = float(
        _get(context, "robust_deduplicate_transform_bin_yaw_deg").strip() or "0.0"
    )
    alignment_reject_timeout_sec = float(
        _get(context, "alignment_reject_timeout_sec").strip() or "60.0"
    )
    alignment_reject_min_verified_matches = int(
        _get(context, "alignment_reject_min_verified_matches").strip() or "7"
    )
    team_pose_graph_backend = (_get(context, "team_pose_graph_backend").strip().lower() or "auto")
    team_alignment_allow_export_only_gate = _as_bool(
        _get(context, "team_alignment_allow_export_only_gate")
    )
    use_dynamic_filter = (
        _as_bool(_get(context, "use_dynamic_filter"))
        or _as_bool(_get(context, "dynamic_filter_enabled"))
        or dynamic_filter_backend != "none"
        or slam_backend == "swarm_lio2_primary"
    )
    decentralized_mode = _as_bool(_get(context, "decentralized_mode"))
    robot_id = _get(context, "robot_id").strip().strip("/") or "robot_a"
    peer_robot_id = _get(context, "peer_robot_id").strip().strip("/") or "robot_b"
    team_comm_mode = (_get(context, "team_comm_mode").strip().lower() or "dds")
    mujoco_model_path = _get(context, "mujoco_model_path").strip()
    session_duration_sec = float(_get(context, "session_duration_sec"))
    session_output_dir = _get(context, "session_output_dir").strip()
    scene_area_m2 = float(_get(context, "scene_area_m2"))
    collision_output = _get(context, "collision_output_path").strip()
    # Per-robot nav backend. Default keeps the historical behaviour
    # (both FAR), but callers can mix: e.g. Go2W on astar, Go2 on far.
    # Accepted values: "far" | "astar".
    nav_backend_a = (_get(context, "nav_backend_a").strip().lower() or "far")
    nav_backend_b = (_get(context, "nav_backend_b").strip().lower() or "far")
    sensor_trust_robot_a = _get(context, "sensor_trust_robot_a").strip() or "1.00"
    sensor_trust_robot_b = _get(context, "sensor_trust_robot_b").strip() or "1.00"
    frontier_trust_enabled = _as_bool(_get(context, "frontier_trust_enabled"))
    frontier_validation_confidence_threshold = float(
        _get(context, "frontier_validation_confidence_threshold").strip() or "0.40"
    )
    frontier_trust_info_gain_penalty_weight = float(
        _get(context, "frontier_trust_info_gain_penalty_weight").strip() or "0.20"
    )
    frontier_validation_trusted_bonus = float(
        _get(context, "frontier_validation_trusted_bonus").strip() or "0.25"
    )
    frontier_validation_untrusted_penalty = float(
        _get(context, "frontier_validation_untrusted_penalty").strip() or "0.25"
    )
    frontier_validation_robot_namespace = (
        _get(context, "frontier_validation_robot_namespace").strip().strip("/")
    )
    role_awareness_enabled = _as_bool(_get(context, "role_awareness_enabled"))
    loop_candidates_enabled = _as_bool(_get(context, "loop_candidates_enabled"))
    morphology_risk_enabled = _as_bool(_get(context, "morphology_risk_enabled"))
    peer_obstacle_enabled = _as_bool(_get(context, "peer_obstacle_enabled"))
    reconstruction_quality_enabled = _as_bool(_get(context, "reconstruction_quality_enabled"))
    scene_graph_enabled = _as_bool(_get(context, "scene_graph_enabled"))
    loop_risk_output_dir = _get(context, "loop_risk_output_dir").strip()
    # Back-compat aliases from the removed planners.
    # `hybrid` → our v0.1 Hybrid A* + Ceres-smoothed planner.
    # `nav2`   → B-route: nav2_smac_planner library integration.
    _alias = {"rrt_star": "astar", "far_rrt_star": "astar", "mppi": "astar",
              "reactive": "astar", "default": "astar",
              "hybrid": "hybrid_astar",
              "nav2":   "nav2_hybrid_astar"}
    nav_backend_a = _alias.get(nav_backend_a, nav_backend_a)
    nav_backend_b = _alias.get(nav_backend_b, nav_backend_b)
    # "none" → spawn perception + SLAM + octomap but NO path planner / controller.
    # Use this when running an external nav stack alongside (e.g. Nav2's
    # planner_server + DWB) — the launch supplies the /map and TF, the
    # external stack publishes /cmd_vel.
    # "nav2_mppi" → spawn the Nav2 stack (SmacPlannerHybrid planner +
    # MPPI controller + lifecycle_manager) plus the CFPA2 → Nav2 goal
    # bridge and the hybrid_cmd_router. Replaces the entire astar /
    # hybrid_astar / far custom-controller chain with battle-tested
    # nav2 nodes; planner is still SmacHybrid so RS arcs are produced,
    # but MPPI samples velocity space (no pure-pursuit overshoot).
    _allowed = {"far", "astar", "hybrid_astar", "nav2_hybrid_astar",
                "nav2_mppi", "none"}
    if nav_backend_a not in _allowed:
        raise ValueError(
            f"nav_backend_a must be one of {_allowed}, got '{nav_backend_a}'")
    if nav_backend_b not in _allowed:
        raise ValueError(
            f"nav_backend_b must be one of {_allowed}, got '{nav_backend_b}'")
    if loop_closure_backend not in {"auto", "ros2_sc_pgo", "ros1_bridge"}:
        raise ValueError(
            "loop_closure_backend must be 'auto' | 'ros2_sc_pgo' | "
            f"'ros1_bridge', got '{loop_closure_backend}'")
    if deployment_mode not in {
        "sim_ros2",
        "sim_hybrid_ros1_slam_ros2_nav",
        "real_hybrid_ros1_slam_ros2_nav",
        "real_ros1_only_experimental",
    }:
        raise ValueError(
            "deployment_mode must be 'sim_ros2' | 'sim_hybrid_ros1_slam_ros2_nav' | "
            f"'real_hybrid_ros1_slam_ros2_nav' | 'real_ros1_only_experimental', got '{deployment_mode}'")
    if slam_backend not in {"fast_lio_scpgo", "swarm_lio2_shadow", "swarm_lio2_primary"}:
        raise ValueError(
            "slam_backend must be 'fast_lio_scpgo' | 'swarm_lio2_shadow' | "
            f"'swarm_lio2_primary', got '{slam_backend}'")
    if dynamic_filter_backend not in {
        "dynamic_lio_port",
        "dynamic_lio_wrapper",
        "temporal_voxel_fallback",
        "none",
    }:
        raise ValueError(
            "dynamic_filter_backend must be 'dynamic_lio_port' | 'dynamic_lio_wrapper' | "
            f"'temporal_voxel_fallback' | 'none', got '{dynamic_filter_backend}'")
    if static_map_cleanup_backend not in {"erasor_wrapper", "temporal_voxel_fallback", "none"}:
        raise ValueError(
            "static_map_cleanup_backend must be 'erasor_wrapper' | "
            f"'temporal_voxel_fallback' | 'none', got '{static_map_cleanup_backend}'")
    if erasor_trigger_mode not in {"manual", "periodic", "benchmark"}:
        raise ValueError(
            f"erasor_trigger_mode must be 'manual' | 'periodic' | 'benchmark', got '{erasor_trigger_mode}'")
    if team_pose_graph_backend not in {"auto", "gtsam", "gtsam_python", "gtsam_cpp", "g2o_export_only"}:
        raise ValueError(
            "team_pose_graph_backend must be 'auto' | 'gtsam' | 'gtsam_python' | 'gtsam_cpp' | "
            f"'g2o_export_only', got '{team_pose_graph_backend}'")
    cpp_pose_graph_owner = team_pose_graph_backend in {"auto", "gtsam_cpp"}
    if team_comm_mode not in {"dds", "udp_json"}:
        raise ValueError(f"team_comm_mode must be 'dds' | 'udp_json', got '{team_comm_mode}'")
    if relative_pose_source not in {"none", "gt", "discovered"}:
        raise ValueError(
            "relative_pose_source must be 'none' | 'gt' | 'discovered', "
            f"got '{relative_pose_source}'")
    slam_bootstrap_from_gt = relative_pose_source == "gt"
    if relative_pose_source == "none":
        if map_merge_enabled:
            map_merge_enabled = False
    elif relative_pose_source == "discovered":
        map_merge_enabled = map_merge_enabled and team_alignment_enable_map_merge
    no_gt_runtime = relative_pose_source != "gt" and not enable_gt_drift_metrics
    peer_filter_namespaces = relative_pose_source == "gt"
    contact_odom_topic = "odom/nav" if no_gt_runtime else "odom/ground_truth"
    if slam_only:
        explore = False

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2w_config_pkg = get_package_share_directory("go2w_config")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")
    champ_base_pkg = get_package_share_directory("champ_base")
    far_pkg = get_package_share_directory("far_planner")
    local_planner_pkg = get_package_share_directory("local_planner")

    if not mujoco_model_path:
        mujoco_model_path = os.path.join(go2_gazebo_pkg, "mujoco", "demo3_mixed.xml")

    ros2_control_config = os.path.join(
        go2_gazebo_pkg, "config", "ros_control", "ros_control_mixed_mujoco_nav.yaml"
    )

    # ── URDF generation (heterogeneous: Go2W for A, Go2 for B) ──
    go2w_urdf = xacro.process_file(
        os.path.join(go2_gazebo_pkg, "urdf", "go2w", "go2w_description_3d_lidar.xacro"),
    ).documentElement.toxml()
    go2_urdf = xacro.process_file(
        os.path.join(go2_gazebo_pkg, "urdf", "go2", "go2_description_3d_lidar.xacro"),
    ).documentElement.toxml()
    combined_urdf = build_mixed_mujoco_urdf(go2w_urdf, go2_urdf)
    robot_a_urdf = build_namespaced_robot_description(
        go2w_urdf, "robot_a",
        os.path.join(go2_gazebo_pkg, "config", "ros_control", "ros_control_go2w_robot_a.yaml"),
    )
    # robot_b is Go2 (not Go2W); b-prefix its standalone URDF for its RSP.
    robot_b_urdf = build_robot_b_urdf(go2_urdf)

    # CHAMP configs
    joints_a = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "joints.yaml")
    joints_b = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "joints_robot_b.yaml")
    links_a = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "links.yaml")
    links_b = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "links_robot_b.yaml")
    gait_config = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "gait.yaml")
    ekf_base = os.path.join(champ_base_pkg, "config", "ekf", "base_to_footprint.yaml")
    ekf_odom = os.path.join(champ_base_pkg, "config", "ekf", "footprint_to_odom.yaml")

    far_tuning_yaml = os.path.join(go2w_config_pkg, "config", "nav", "far_planner_tuning.yaml")
    far_default_yaml = os.path.join(far_pkg, "config", "default.yaml")
    local_planner_paths_dir = os.path.join(local_planner_pkg, "paths")
    slam_config_l1 = os.path.join(go2w_config_pkg, "config", "slam", "pointlio_gazebo_l1.yaml")
    slam_config_mid360 = os.path.join(go2w_config_pkg, "config", "slam", "pointlio_gazebo_mid360.yaml")
    workspace_root = _workspace_root()

    mujoco_plugin_dir = _find_mujoco_plugin_dir()
    sim_ns = "mujoco_sim"

    actions = [LogInfo(msg="[nav_test_mujoco_fastlio_mixed] starting heterogeneous dual-robot nav (Go2W + Go2)")]
    actions.append(LogInfo(msg=(
        f"[nav_test_mujoco_fastlio_mixed] deployment_mode:={deployment_mode} "
        f"slam_backend:={slam_backend} "
        f"dynamic_filter_backend:={dynamic_filter_backend} "
        f"static_map_cleanup_backend:={static_map_cleanup_backend}"
    )))
    actions.append(LogInfo(msg=(
        f"[nav_test_mujoco_fastlio_mixed] relative_pose_source:={relative_pose_source} "
        f"bootstrap_from_gt:={'true' if slam_bootstrap_from_gt else 'false'} "
        f"map_merge:={'true' if map_merge_enabled else 'false'}"
    )))
    if relative_pose_source == "none":
        actions.append(LogInfo(msg=(
            "[nav_test_mujoco_fastlio_mixed] no initial inter-robot pose: "
            "GT map_merge and shared-frame inter-robot candidates are disabled."
        )))
    elif relative_pose_source == "discovered":
        actions.append(LogInfo(msg=(
            "[nav_test_mujoco_fastlio_mixed] discovered inter-robot pose: "
            "team_loop_closure will estimate robot_a/map -> robot_b/map from LiDAR keyframes."
        )))
    if slam_only:
        actions.append(LogInfo(msg="[nav_test_mujoco_fastlio_mixed] slam_only:=true — starting MuJoCo, RSP, sensor bridges, Fast-LIO, octomap; skipping CHAMP/controller/nav/explore stacks"))
    if use_dynamic_filter:
        actions.append(LogInfo(msg=(
            "[dynamic_scene_filter] enabled: raw Fast-LIO cloud remains unchanged; "
            "team loop-closure keyframes use /robot_*/cloud_static."
        )))

    # ── T=0: cleanup stale ──
    if cleanup_stale:
        actions.append(
            ExecuteProcess(cmd=["bash", "-lc", _build_cleanup_stale_cmd()], output="screen")
        )

    # ── T=3: MuJoCo (one process with dual URDF) ──
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=sim_ns,
        parameters=[
            {"robot_description": combined_urdf},
            ros2_control_config,
            {"robot_model_path": mujoco_model_path},
            {"simulation_frequency": 500.0},
            {"real_time_factor": 1.0},
            {"clock_publisher_frequency": 100.0},
            {"show_gui": gui},
            {"enable_cameras": mujoco_cameras},
        ],
        remappings=[
            (f"/{sim_ns}/controller_manager/robot_description", f"/{sim_ns}/robot_description"),
        ],
        additional_env={"MUJOCO_PLUGIN_DIR": mujoco_plugin_dir},
        output="screen",
    )
    actions.append(TimerAction(period=3.0, actions=[mujoco_node]))

    # ── T=5: Sensor bridges (odom + contact) for both robots ──
    sensor_actions = []
    sensor_actions.extend(
        _build_sensor_bridges(
            ns="robot_a", mjcf_path=mujoco_model_path,
            base_body="base_link", imu_site="imu",
            pose_sensor="base_link_site_pose_sensor",
            imu_sensor="imu_imu_sensor",
            links_config=links_a,
            use_sim_time=use_sim_time,
            contact_odom_topic=contact_odom_topic,
        )
    )
    sensor_actions.extend(
        _build_sensor_bridges(
            ns="robot_b", mjcf_path=mujoco_model_path,
            base_body="b_base_link", imu_site="b_imu",
            pose_sensor="b_base_link_site_pose_sensor",
            imu_sensor="b_imu_imu_sensor",
            links_config=links_b,
            use_sim_time=use_sim_time,
            contact_odom_topic=contact_odom_topic,
        )
    )
    actions.append(TimerAction(period=5.0, actions=sensor_actions))

    # ── T=7 / T=10: Per-robot CHAMP stacks (staggered to share controller_manager) ──
    # Same pattern as dual_go2w_mujoco_door.launch.py — ROS service calls to
    # controller_manager are serial, so stagger spawners to avoid collisions.
    # Door-launch pattern: wrap each robot's stack in an outer TimerAction
    # so all its sub-actions (RSP, CHAMP, spawners, standup) use their
    # DEFAULT internal sub-delays but start 7s / 10s after launch. This
    # gives mujoco_ros2_control time to come up before controller_manager
    # service calls begin, and staggers A's spawners before B's to avoid
    # load_controller service-timeout races.
    if slam_only:
        actions.append(
            TimerAction(
                period=6.5,
                actions=[
                    _build_robot_state_publisher("robot_a", robot_a_urdf, use_sim_time),
                    _build_robot_state_publisher("robot_b", robot_b_urdf, use_sim_time),
                ],
            )
        )
    else:
        robot_a_stack = build_dual_robot_stack(
            ns="robot_a",
            spawn_x="4.0", spawn_y="2.0", spawn_yaw="0.0",
            use_sim_time=use_sim_time,
            robot_description=robot_a_urdf,
            joints_config=joints_a, links_config=links_a,
            gait_config=gait_config,
            ekf_base_to_footprint=ekf_base,
            ekf_footprint_to_odom=ekf_odom,
            activate_controllers_on_spawn=True,
            stand_up_joint_preset="go2",
            cmd_vel_input_topic="cmd_vel_legged",
            wheel_controller_name="robot_a_wheel_velocity_controller",
            use_mujoco=True,
            controller_manager_name=f"/{sim_ns}/controller_manager",
        )
        actions.append(TimerAction(period=7.0, actions=robot_a_stack))

        robot_b_stack = build_dual_robot_stack(
            ns="robot_b",
            spawn_x="4.0", spawn_y="-6.0", spawn_yaw="0.0",
            use_sim_time=use_sim_time,
            robot_description=robot_b_urdf,
            joints_config=joints_b, links_config=links_b,
            gait_config=gait_config,
            ekf_base_to_footprint=ekf_base,
            ekf_footprint_to_odom=ekf_odom,
            activate_controllers_on_spawn=True,
            stand_up_joint_preset="go2",
            # robot_b's joints in the MJCF / ros2_control yaml are b-prefixed
            # (b_FL_hip_joint, b_FL_thigh_joint, ...). Without this prefix,
            # stand_up_slowly publishes unprefixed joint names to the
            # /mujoco_sim/robot_b_joint_group_effort_controller/joint_trajectory
            # topic, which the controller rejects with
            # "Incoming joint FL_hip_joint doesn't match the controller's joints."
            # → standup never fires → robot_b stays in MJCF default pose.
            stand_up_joint_prefix="b_",
            # Go2 has no wheels → no hybrid router → CHAMP subscribes to /cmd_vel
            # directly (no need for a cmd_vel → cmd_vel_legged relay).
            cmd_vel_input_topic="cmd_vel",
            wheel_controller_name=None,
            use_mujoco=True,
            controller_manager_name=f"/{sim_ns}/controller_manager",
        )
        actions.append(TimerAction(period=10.0, actions=robot_b_stack))

    # ── Per-robot Fast-LIO + FAR nav stacks ──
    slam_delay = 10.0 if slam_only else 20.0   # after RSP/sensors or both standups
    nav_delay = slam_delay + 5.0

    actions.extend(
        _build_fastlio_nav_stack(
            ns="robot_a",
            # Plugin publishes under /mujoco_sim/ — the sim's controller_manager
            # namespace — NOT per-robot. Robot A now uses the mounted Livox
            # MID-360 by default; the old Unitree L1 site is retained only for
            # explicit sensing-asymmetry ablations.
            mujoco_lidar_topic="/mujoco_sim/mujoco_lidar_sensor/registered_scan",
            # Robot A's URDF uses bare link names (no prefix) — its TF tree
            # has `base_link` as the root and `imu` as the IMU link.
            base_frame="base_link",
            imu_frame="imu",
            use_sim_time=use_sim_time,
            nav_backend=nav_backend_a,
            slam_delay=slam_delay,
            nav_delay=nav_delay,
            go2w_config_pkg=go2w_config_pkg,
            slam_config_path=slam_config_mid360,
            adapter_num_rings=20,
            adapter_min_vert_angle_deg=-7.0,
            adapter_max_vert_angle_deg=52.0,
            local_planner_paths_dir=local_planner_paths_dir,
            far_tuning_yaml=far_tuning_yaml,
            far_default_yaml=far_default_yaml,
            octomap_min_z=0.30,
            enable_nav=not slam_only,
            has_wheels=True,  # robot_a = Go2W
            peer_namespaces=["robot_b"] if peer_filter_namespaces else [],
            loop_closure=loop_closure_on,
            loop_closure_backend=loop_closure_backend,
            bootstrap_from_gt=slam_bootstrap_from_gt,
            peer_obstacle_enabled=peer_obstacle_enabled,
            slam_backend=slam_backend,
            dynamic_filter_backend=dynamic_filter_backend,
        )
    )
    actions.extend(
        _build_fastlio_nav_stack(
            ns="robot_b",
            # Robot B's LiDAR site is "b_livox_mid360" (b_ prefix in MJCF) →
            # plugin names the topic accordingly.
            mujoco_lidar_topic="/mujoco_sim/b_mujoco_lidar_sensor/registered_scan",
            # Robot B's URDF is `b_`-prefixed (via build_robot_b_urdf) →
            # robot_state_publisher emits `b_base_link` into /robot_b/tf.
            # Every downstream consumer (octomap, laser_scan target_frame,
            # slam_odom_relay output child frame) must use that same name
            # or TF lookup fails. IMU link is `b_imu` for the same reason.
            base_frame="b_base_link",
            imu_frame="b_imu",
            use_sim_time=use_sim_time,
            nav_backend=nav_backend_b,
            slam_delay=slam_delay,
            nav_delay=nav_delay,
            go2w_config_pkg=go2w_config_pkg,
            slam_config_path=slam_config_mid360,
            adapter_num_rings=20,
            adapter_min_vert_angle_deg=-7.0,
            adapter_max_vert_angle_deg=52.0,
            local_planner_paths_dir=local_planner_paths_dir,
            far_tuning_yaml=far_tuning_yaml,
            far_default_yaml=far_default_yaml,
            octomap_min_z=0.30,
            enable_nav=not slam_only,
            has_wheels=False,  # robot_b = Go2 (no wheels)
            peer_namespaces=["robot_a"] if peer_filter_namespaces else [],
            loop_closure=loop_closure_on,
            loop_closure_backend=loop_closure_backend,
            bootstrap_from_gt=slam_bootstrap_from_gt,
            peer_obstacle_enabled=peer_obstacle_enabled,
            slam_backend=slam_backend,
            dynamic_filter_backend=dynamic_filter_backend,
        )
    )

    team_alignment_active = inter_robot_loop_closure or relative_pose_source == "discovered"
    if team_alignment_active:
        keyframe_cloud_topic = "/team_slam/static_keyframe_clouds" if use_dynamic_filter else "/team_slam/keyframe_clouds"
        keyframe_input_cloud_topic = "cloud_static" if use_dynamic_filter else "cloud_registered_body"
        team_alignment_nodes = []
        if use_dynamic_filter:
            selected_dynamic_backend = (
                dynamic_filter_backend
                if dynamic_filter_backend != "none"
                else "temporal_voxel_fallback"
            )
            team_alignment_nodes.extend([
                Node(
                    package="slam_backend_adapters",
                    executable="dynamic_lio_filtering_node",
                    name="dynamic_lio_filtering_node",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "namespaces": ["robot_a", "robot_b"],
                        "dynamic_filter_backend": selected_dynamic_backend,
                    }],
                    output="screen",
                ),
            ])
        if decentralized_mode:
            team_alignment_nodes.append(
                Node(
                    package="team_loop_closure",
                    executable="team_slam_peer_node",
                    name="team_slam_peer_node",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "robot_id": robot_id,
                        "peer_robot_id": peer_robot_id,
                        "team_comm_mode": team_comm_mode,
                    }],
                    output="screen",
                )
            )
        team_pose_graph_export_dir = (
            os.path.join(loop_risk_output_dir, "team_pose_graph")
            if loop_risk_output_dir else os.path.join(str(workspace_root), "logs")
        )
        team_pose_graph_metrics_path = (
            os.path.join(loop_risk_output_dir, "team_pose_graph_metrics.json")
            if loop_risk_output_dir else os.path.join(str(workspace_root), "logs", "team_pose_graph_metrics.json")
        )
        team_pose_graph_export_metrics_path = (
            os.path.join(loop_risk_output_dir, "team_pose_graph_export_metrics.json")
            if loop_risk_output_dir else os.path.join(
                str(workspace_root), "logs", "team_pose_graph_export_metrics.json")
        )
        team_alignment_nodes.extend([
                    Node(
                        package="team_loop_closure",
                        executable="loop_keyframe_exporter_node",
                        name="loop_keyframe_exporter_node",
                        parameters=[{
                            "use_sim_time": use_sim_time,
                            "namespaces": ["robot_a", "robot_b"],
                            "output_topic": "/team_slam/keyframes",
                            "keyframe_cloud_topic": keyframe_cloud_topic,
                            "cloud_topic": keyframe_input_cloud_topic,
                            "scan_context_num_rings": scan_context_num_rings,
                            "scan_context_num_sectors": scan_context_num_sectors,
                            "scan_context_max_radius": scan_context_max_radius,
                            "keyframe_min_translation": 1.0,
                            "keyframe_min_yaw_deg": 10.0,
                            "keyframe_min_time_sec": 1.0,
                        }],
                        output="screen",
                    ),
                    Node(
                        package="team_loop_closure",
                        executable="cross_robot_loop_matcher_node",
                        name="cross_robot_loop_matcher_node",
                        parameters=[{
                            "use_sim_time": use_sim_time,
                            "keyframe_topic": "/team_slam/keyframes",
                            "keyframe_cloud_topic": keyframe_cloud_topic,
                            "candidate_topic": "/team_slam/cross_robot_candidates",
                            "match_topic": "/team_slam/cross_robot_matches",
                            "reference_robot": "robot_a",
                            "target_robot": "robot_b",
                            "registration_backend": registration_backend,
                        }],
                        output="screen",
                    ),
                    Node(
                        package="team_loop_closure",
                        executable="robust_loop_selector_node",
                        name="robust_loop_selector_node",
                        parameters=[{
                            "use_sim_time": use_sim_time,
                            "match_topic": "/team_slam/cross_robot_matches",
                            "robust_inliers_topic": "/team_slam/robust_loop_inliers",
                            "reference_robot": "robot_a",
                            "target_robot": "robot_b",
                            "parent_frame": "robot_a/map",
                            "child_frame": "robot_b/map",
                            "team_alignment_min_matches": team_alignment_min_matches,
                            "team_alignment_max_translation_disagreement": team_alignment_max_translation_disagreement,
                            "team_alignment_max_yaw_disagreement_deg": team_alignment_max_yaw_disagreement_deg,
                            "robust_min_inliers": robust_min_inliers,
                            "robust_min_inlier_ratio": robust_min_inlier_ratio,
                            "robust_max_median_rmse": robust_max_median_rmse,
                            "robust_max_translation_spread_m": robust_max_translation_spread_m,
                            "robust_max_yaw_spread_deg": robust_max_yaw_spread_deg,
                            "robust_prefilter_max_rmse": robust_prefilter_max_rmse,
                            "robust_prefilter_min_inlier_ratio": robust_prefilter_min_inlier_ratio,
                            "robust_prefilter_min_correspondences": robust_prefilter_min_correspondences,
                            "robust_prefilter_max_descriptor_distance": robust_prefilter_max_descriptor_distance,
                            "robust_deduplicate_by_query_keyframe": robust_deduplicate_by_query_keyframe,
                            "robust_deduplicate_by_match_keyframe": robust_deduplicate_by_match_keyframe,
                            "robust_deduplicate_transform_bin_translation_m": (
                                robust_deduplicate_transform_bin_translation_m
                            ),
                            "robust_deduplicate_transform_bin_yaw_deg": (
                                robust_deduplicate_transform_bin_yaw_deg
                            ),
                        }],
                        output="screen",
                    ),
                    Node(
                        package="team_loop_closure",
                        executable="team_pose_graph_node",
                        name="team_pose_graph_node",
                        parameters=[{
                            "use_sim_time": use_sim_time,
                            "robots": ["robot_a", "robot_b"],
                            "keyframe_topic": "/team_slam/keyframes",
                            "match_topic": "/team_slam/cross_robot_matches",
                            "robust_inliers_topic": "/team_slam/robust_loop_inliers",
                            "metrics_topic": "/team_slam/pose_graph_metrics",
                            "factors_topic": "/team_slam/team_pose_graph_factors",
                            "team_pose_graph_backend": team_pose_graph_backend,
                            "export_dir": team_pose_graph_export_dir,
                            "metrics_path": (
                                team_pose_graph_export_metrics_path
                                if cpp_pose_graph_owner else team_pose_graph_metrics_path
                            ),
                            "publish_backend_metrics": not cpp_pose_graph_owner,
                            "allow_export_only_outputs": team_alignment_allow_export_only_gate,
                        }],
                        output="screen",
                    ),
                    *([
                        Node(
                            package="team_pose_graph_optimizer",
                            executable="team_pose_graph_optimizer_node",
                            name="team_pose_graph_optimizer_node",
                            parameters=[{
                                "use_sim_time": use_sim_time,
                                "factors_topic": "/team_slam/team_pose_graph_factors",
                                "metrics_topic": "/team_slam/pose_graph_metrics",
                                "factors_json_path": os.path.join(
                                    team_pose_graph_export_dir, "team_pose_graph_factors.json"),
                                "metrics_path": team_pose_graph_metrics_path,
                            }],
                            output="screen",
                        )
                    ] if cpp_pose_graph_owner else []),
                    Node(
                        package="team_loop_closure",
                        executable="relative_transform_manager_node",
                        name="relative_transform_manager_node",
                        parameters=[{
                            "use_sim_time": use_sim_time,
                            "robust_inliers_topic": "/team_slam/robust_loop_inliers",
                            "pose_graph_metrics_topic": "/team_slam/pose_graph_metrics",
                            "status_topic": "/team_slam/alignment_status",
                            "relative_transform_topic": "/team_slam/relative_transform",
                            "parent_frame": "robot_a/map",
                            "child_frame": "robot_b/map",
                            "team_alignment_allow_export_only_gate": team_alignment_allow_export_only_gate,
                            "require_swarm_loop_agreement": (
                                require_swarm_loop_agreement
                                and slam_backend == "swarm_lio2_primary"
                            ),
                            "swarm_loop_relative_transform_topic": (
                                "/team_slam/swarm_lio2_relative_transform"
                            ),
                            "swarm_loop_agreement_max_translation": (
                                swarm_loop_agreement_max_translation
                            ),
                            "swarm_loop_agreement_max_yaw_deg": (
                                swarm_loop_agreement_max_yaw_deg
                            ),
                            "alignment_reject_timeout_sec": alignment_reject_timeout_sec,
                            "alignment_reject_min_verified_matches": alignment_reject_min_verified_matches,
                            "publish_tf": False,
                        }],
                        output="screen",
                    ),
        ])
        actions.append(
            TimerAction(
                period=slam_delay + 4.0,
                actions=team_alignment_nodes,
            )
        )
    if static_map_cleanup_backend != "none":
        actions.append(
            TimerAction(
                period=slam_delay + 8.0,
                actions=[
                    Node(
                        package="slam_backend_adapters",
                        executable="erasor_adapter_node",
                        name="erasor_adapter_node",
                        parameters=[{
                            "use_sim_time": use_sim_time,
                            "namespaces": ["robot_a", "robot_b"],
                            "static_map_cleanup_backend": static_map_cleanup_backend,
                            "erasor_trigger_mode": erasor_trigger_mode,
                            "export_dir": (
                                os.path.join(loop_risk_output_dir, "erasor")
                                if loop_risk_output_dir
                                else os.path.join(str(workspace_root), "logs", "erasor")
                            ),
                        }],
                        output="screen",
                    ),
                ],
            )
        )

    def _loop_risk_artifact(name: str) -> str:
        if not loop_risk_output_dir:
            return ""
        return os.path.join(loop_risk_output_dir, name)

    # ── Loop-closure proxy + mobility-risk awareness layer ──
    awareness_nodes = []
    if explore and (
        role_awareness_enabled
        or loop_candidates_enabled
        or reconstruction_quality_enabled
        or scene_graph_enabled
    ):
        awareness_nodes.append(
            Node(
                package="reconstruction_awareness",
                executable="pose_graph_health_node",
                name="pose_graph_health_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "namespaces": ["robot_a", "robot_b"],
                    "output_path": _loop_risk_artifact("pose_graph_health.json"),
                    "require_team_alignment_for_inter_robot": relative_pose_source == "discovered",
                    "alignment_status_topic": "/team_slam/alignment_status",
                    "enable_gt_drift_metrics": enable_gt_drift_metrics,
                }],
                output="screen",
            )
        )
    if explore and loop_candidates_enabled:
        awareness_nodes.append(
            Node(
                package="reconstruction_awareness",
                executable="loop_closure_candidate_node",
                name="loop_closure_candidate_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "output_path": _loop_risk_artifact("loop_candidates.json"),
                    "map_topic": "/merged_map",
                    "inter_robot_enabled": inter_robot_loop_closure or relative_pose_source == "gt",
                    "require_team_alignment_for_inter_robot": relative_pose_source == "discovered",
                    "alignment_status_topic": "/team_slam/alignment_status",
                }],
                output="screen",
            )
        )
    if explore and morphology_risk_enabled:
        awareness_nodes.append(
            Node(
                package="reconstruction_awareness",
                executable="morphology_risk_node",
                name="morphology_risk_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "namespaces": ["robot_a", "robot_b"],
                    "map_topic": "/merged_map",
                    "output_path": _loop_risk_artifact("morphology_risk.json"),
                }],
                output="screen",
            )
        )
    if explore and reconstruction_quality_enabled:
        awareness_nodes.append(
            Node(
                package="reconstruction_awareness",
                executable="reconstruction_quality_node",
                name="reconstruction_quality_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "namespaces": ["robot_a", "robot_b"],
                    "map_topic": "/merged_map",
                    # Stage-7 3D upgrade: legacy file kept; two new
                    # canonical outputs per proposal §10.1 deliverables.
                    "output_path": _loop_risk_artifact("reconstruction_quality.json"),
                    "summary_output_path": _loop_risk_artifact("reconstruction_quality_summary.json"),
                    "voxels_output_path": _loop_risk_artifact("reconstruction_voxels.json"),
                }],
                output="screen",
            )
        )
        awareness_nodes.append(
            Node(
                package="reconstruction_awareness",
                executable="accumulated_pointcloud_node",
                name="accumulated_pointcloud_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "namespaces": ["robot_a", "robot_b"],
                    "voxel_size_m": 0.10,
                    "expected_frame": "map",
                    "output_dir": loop_risk_output_dir,
                    "output_basename": "accumulated_cloud",
                }],
                output="screen",
            )
        )
    if explore and scene_graph_enabled:
        awareness_nodes.append(
            Node(
                package="reconstruction_awareness",
                executable="scene_graph_builder_node",
                name="scene_graph_builder_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "map_topic": "/merged_map",
                    "output_path": _loop_risk_artifact("scene_graph.json"),
                }],
                output="screen",
            )
        )
    if explore and reconstruction_quality_enabled and loop_risk_output_dir:
        # Keyframe logger for §10.2 offline 3DGS (Stage 9). The MuJoCo
        # RGBD plugin publishes to /<camera_name>/color/image_raw with
        # no ns prefix (front_camera for robot_a, b_front_camera for
        # robot_b — derived from MJCF camera names in demo3_mixed.xml).
        # Pass explicit overrides so the logger binds without needing
        # the auto-detect heuristic at runtime.
        awareness_nodes.append(
            Node(
                package="reconstruction_awareness",
                executable="keyframe_logger_node",
                name="keyframe_logger_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "namespaces": ["robot_a", "robot_b"],
                    "camera_match": "front_camera",
                    "camera_topic_overrides": [
                        "robot_a=/front_camera/color/image_raw",
                        "robot_b=/b_front_camera/color/image_raw",
                    ],
                    "output_dir": loop_risk_output_dir,
                    "min_translation_m": 0.50,
                    "min_rotation_deg": 25.0,
                    "period_sec": 4.0,
                }],
                output="screen",
            )
        )
    if awareness_nodes:
        actions.append(TimerAction(period=nav_delay + 1.0, actions=awareness_nodes))

    # ── CFPA2 dual-robot coordinator (shared) ──
    if explore:
        cfpa2_config_path = os.path.join(cfpa2_pkg, "config", "cfpa2_coordinator.yaml")
        if not os.path.exists(cfpa2_config_path):
            cfpa2_config_path = os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml")
        actions.append(
            TimerAction(
                period=nav_delay + 2.0,
                actions=[
                    Node(
                        package="cfpa2_collaborative_autonomy",
                        executable="cfpa2_coordinator_node",
                        name="cfpa2_coordinator",
                        parameters=[
                            cfpa2_config_path,
                            {
                                "use_sim_time": use_sim_time,
                                "namespaces": ["robot_a", "robot_b"],
                                "sensor_trust_by_namespace": [
                                    f"robot_a={sensor_trust_robot_a}",
                                    f"robot_b={sensor_trust_robot_b}",
                                ],
                                "frontier_trust_enabled": frontier_trust_enabled,
                                "frontier_validation_confidence_threshold": frontier_validation_confidence_threshold,
                                "frontier_trust_info_gain_penalty_weight": frontier_trust_info_gain_penalty_weight,
                                "frontier_validation_trusted_bonus": frontier_validation_trusted_bonus,
                                "frontier_validation_untrusted_penalty": frontier_validation_untrusted_penalty,
                                "frontier_validation_robot_namespace": frontier_validation_robot_namespace,
                                "role_awareness_enabled": role_awareness_enabled,
                                "role_loop_enabled": loop_candidates_enabled,
                                "role_mobility_risk_enabled": morphology_risk_enabled,
                                "scene_area_m2": scene_area_m2,
                                "goal_topic_suffix": "/way_point_coord",
                                "marker_frame_override": "map",
                                # ── Shared-map frontier extraction ──
                                # Without this, CFPA2 extracts frontiers
                                # from EACH robot's /{ns}/map independently
                                # and dedupes targets only by spatial
                                # nearness. A cell that's "free with
                                # unknown neighbour" in B's small local
                                # map can be FREE-ALL-AROUND in A's
                                # already-explored region — but B's-side
                                # extraction marks it a frontier and
                                # _merge_targets keeps it. Result: B
                                # gets dispatched to a "frontier" that's
                                # actually known free space from A's
                                # perspective, wasting time + LiDAR.
                                #
                                # multirobot_map_merge publishes the union
                                # at /merged_map. Pointing CFPA2 at it
                                # makes "frontier" mean "boundary between
                                # the swarm's combined known region and
                                # genuinely-unknown space" — the correct
                                # definition. CFPA2 fails-open to per-ns
                                # for the first ~30 s while map_merge
                                # bootstraps GT init poses, then auto-
                                # switches once the merged map arrives.
                                "use_shared_map": map_merge_enabled,
                                "shared_map_topic": "/merged_map",
                                # In discovered mode, alignment needs robots
                                # to move first and produce multiple LiDAR
                                # keyframes. Waiting for /merged_map here
                                # deadlocks that process because /merged_map
                                # itself is gated on robust alignment.
                                # Fail open immediately to per-robot maps;
                                # CFPA2 will still switch to /merged_map
                                # automatically after map_merge appears.
                                "shared_map_wait_sec": 0.0 if relative_pose_source == "discovered" else 35.0,
                            },
                        ],
                        output="screen",
                    ),
                ],
            )
        )

    # ── Dual-robot safety monitor: wall contacts (per robot), tip-over,
    #    planner-stuck, plus the legacy inter-robot collision pair tracker.
    #    Same script name kept for backward-compatible launch wiring; see
    #    dual_robot_collision_monitor.py for the expanded checker stack.
    collision_monitor_script = str(
        workspace_root / "scripts" / "runtime" / "dual_robot_collision_monitor.py"
    )
    collision_args = [
        "python3", "-u", collision_monitor_script,
        "--robots", "robot_a", "robot_b",
        # Same denominator the benchmark / session_reporter uses, so the
        # debug-mode coverage column matches the trial-summary 90% PASS bar.
        "--scene-area-m2", str(scene_area_m2),
    ]
    if collision_output:
        collision_args += ["--output", collision_output]
    if not slam_only and not no_gt_runtime:
        actions.append(
            TimerAction(
                period=3.5,  # right after MuJoCo comes up (T=3)
                actions=[
                    ExecuteProcess(
                        cmd=collision_args,
                        name="dual_robot_collision_monitor",
                        output="screen",
                    ),
                ],
            )
        )

    # ── Session reporter(s) ──
    # Per-robot reporter if session_duration_sec > 0 and output dir given.
    if session_duration_sec > 0 and session_output_dir:
        os.makedirs(session_output_dir, exist_ok=True)
        reporter_script = str(workspace_root / "scripts" / "bench" / "session_reporter.py")
        trust_reporter_script = str(workspace_root / "scripts" / "bench" / "frontier_trust_reporter.py")
        last_reporter = None
        for ns in ("robot_a", "robot_b"):
            out_path = os.path.join(session_output_dir, f"{ns}.json")
            proc = ExecuteProcess(
                cmd=[
                    "python3", "-u", reporter_script,
                    "--duration", str(session_duration_sec),
                    "--namespace", ns,
                    "--output", out_path,
                    "--scene-area-m2", str(scene_area_m2),
                ],
                name=f"session_reporter_{ns}",
                output="screen",
            )
            actions.append(TimerAction(period=nav_delay + 3.0, actions=[proc]))
            last_reporter = proc
        if frontier_trust_enabled:
            actions.append(
                TimerAction(
                    period=nav_delay + 2.5,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                "python3", "-u", trust_reporter_script,
                                "--duration", str(session_duration_sec),
                                "--topic", "/cfpa2/frontier_trust",
                                "--output", os.path.join(session_output_dir, "frontier_trust.json"),
                            ],
                            name="frontier_trust_reporter",
                            output="screen",
                        ),
                    ],
                )
            )
        # Shut down the whole launch when the last reporter exits.
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=last_reporter,
                    on_exit=[
                        LogInfo(msg="[dual] session reporter exited — shutdown"),
                        Shutdown(reason="session complete"),
                    ],
                )
            )
        )

    # ── multirobot_map_merge ──
    # robot_a and robot_b each publish `/robot_*/map` (OccupancyGrid,
    # frame_id=map). The map_merge node takes both + per-robot
    # `init_pose_{x,y,yaw}` in world_frame and emits a merged `/merged_map`.
    # relative_pose_source:=gt uses the legacy map-origin helper with GT
    # logging. relative_pose_source:=discovered waits for team_loop_closure
    # to publish an aligned robot_a/map -> robot_b/map transform first.
    if map_merge_enabled:
        try:
            get_package_share_directory("multirobot_map_merge")
        except PackageNotFoundError:
            actions.append(
                LogInfo(
                    msg=(
                        "[map_merge] package 'multirobot_map_merge' not found; "
                        "skipping map_merge. Per-robot map_augmenter will pass "
                        "local octomap output through to /robot_*/map."
                    )
                )
            )
        else:
            merge_params_path = (
                "/tmp/map_merge_params_discovered.yaml"
                if relative_pose_source == "discovered"
                else "/tmp/map_merge_params.yaml"
            )
            if relative_pose_source == "discovered":
                bootstrap_proc = ExecuteProcess(
                    cmd=[
                        "ros2", "run", "team_loop_closure",
                        "discovered_map_merge_bootstrap_node",
                        "--robots", "robot_a", "robot_b",
                        "--alignment-status-topic", "/team_slam/alignment_status",
                        "--output", merge_params_path,
                        "--timeout-sec", "180",
                        "--merged-map-topic", "merged_map",
                        "--merging-rate", "2.0",
                        "--discovery-rate", "0.5",
                    ],
                    name="discovered_map_merge_bootstrap",
                    output="screen",
                )
            else:
                bootstrap_script = str(
                    workspace_root / "scripts" / "runtime" / "bootstrap_map_merge_poses.py"
                )
                bootstrap_proc = ExecuteProcess(
                    cmd=[
                        "python3", "-u", bootstrap_script,
                        "--robots", "robot_a", "robot_b",
                        "--gt-topic-suffix", "odom/ground_truth",
                        "--output", merge_params_path,
                        "--timeout-sec", "30",
                        "--merged-map-topic", "merged_map",
                        "--merging-rate", "2.0",
                        "--discovery-rate", "0.5",
                    ],
                    name="bootstrap_map_merge_poses",
                    output="screen",
                )
            map_merge_node = Node(
                package="multirobot_map_merge",
                executable="map_merge",
                name="map_merge",
                parameters=[merge_params_path, {"use_sim_time": use_sim_time}],
                output="screen",
            )
            # Start discovered map-merge bootstrap only after the team
            # alignment front-end/back-end nodes exist. This avoids accepting
            # a stale aligned status if a previous benchmark launch was not
            # fully torn down yet.
            actions.append(TimerAction(period=slam_delay + 6.0, actions=[bootstrap_proc]))

            def _on_bootstrap_exit(event, _context):
                rc = getattr(event, "returncode", None)
                if rc == 0:
                    return [map_merge_node]
                return [
                    LogInfo(
                        msg=(
                            f"[map_merge] map-merge bootstrap exited with "
                            f"code {rc}; skipping map_merge (no valid init poses)."
                        )
                    )
                ]

            actions.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=bootstrap_proc,
                        on_exit=_on_bootstrap_exit,
                    )
                )
            )

    # ── RViz ──
    # Namespaced /tf is invisible to RViz's default global /tf listener. We
    # fan `/robot_a/tf` + `/robot_b/tf` (and _static) into `/tf` so RViz can
    # render both trees at once. Then spawn RViz pointing at nav_test.rviz.
    if rviz:
        actions.append(
            TimerAction(
                period=slam_delay,
                actions=[
                    (
                        Node(
                            package="go2w_perception",
                            executable="multi_tf_relay",
                            name="multi_tf_relay",
                            parameters=[
                                {"use_sim_time": use_sim_time},
                                {"sources": ["robot_a", "robot_b"]},
                            ],
                            output="screen",
                        )
                        if relative_pose_source == "gt"
                        else LogInfo(msg=(
                            "[rviz] skipping global multi_tf_relay because "
                            f"relative_pose_source:={relative_pose_source}; "
                            "per-robot TF trees are not in one shared frame yet."
                        ))
                    ),
                    # VSCode-snap shells inject XDG_DATA_HOME, GSETTINGS_SCHEMA_DIR,
                    # GTK_PATH, LOCPATH that point into /snap/code/*/. That path
                    # has a gio-modules cache whose symlinks carry RPATHs into
                    # /snap/core20/.../libpthread.so.0, which crashes rviz2 at
                    # runtime with "undefined symbol: __libc_pthread_init".
                    # Strip those vars before exec'ing rviz2.
                    ExecuteProcess(
                        cmd=[
                            "bash", "-c",
                            "unset XDG_DATA_HOME GSETTINGS_SCHEMA_DIR GTK_PATH LOCPATH "
                            "SNAP SNAP_NAME SNAP_INSTANCE_NAME SNAP_REVISION "
                            "SNAP_LIBRARY_PATH SNAP_USER_DATA SNAP_USER_COMMON; "
                            # --log-level WARN silences rviz2's per-display
                            # INFO spam ("Map received", "Using fixed frame",
                            # TF-lookup INFO retries, Ogre mesh-loader info).
                            # Warnings + errors still print so you notice real
                            # problems (missing frames, topic QoS mismatches).
                            "exec rviz2 -d \"$1\" "
                            "--ros-args -p use_sim_time:=true "
                            "--log-level rviz2:=WARN "
                            "--log-level rviz_common:=WARN "
                            "--log-level rviz_default_plugins:=WARN",
                            "--",
                            # nav_test_mixed.rviz points displays at
                            # /robot_a/* (primary map) and /robot_b/map as a
                            # secondary overlay — the stock nav_test.rviz
                            # uses /robot/* which is single-robot only.
                            os.path.join(go2_gazebo_pkg, "rviz", "nav_test_mixed.rviz"),
                        ],
                        name="rviz2_nav_test_mixed",
                        # output="log" routes rviz2 stdout/stderr to the per-
                        # launch log file under ~/.ros/log/<session>/rviz2*.log
                        # instead of the shared terminal, keeping it readable.
                        # Tail that file if you want to see rviz2 output live.
                        output="log",
                    ),
                ],
            )
        )

    # `debug:=true` — focus the terminal on path-planner / nav / controller
    # output. Every other process is routed to ~/.ros/log/<session>/<name>*.log
    # (still inspectable via `tail -f` if needed) so the operator can read
    # astar_nav_node / far_planner / pathFollower / hybrid_cmd_router output
    # without it being interleaved with mujoco, fast_lio, champ_base, octomap,
    # terrain_analysis, ekf, sensor bridges, rviz, map_merge, etc.
    if debug:
        _filter_actions_to_nav_only(actions)
        actions.insert(0, LogInfo(msg=(
            "[nav_test_mujoco_fastlio_mixed] debug:=true — non-nav nodes "
            "routed to log file; only path-planner / controller output on "
            "stdout. Visible: " + ", ".join(_NAV_DEBUG_KEEP_EXECUTABLES)
        )))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("explore", default_value="true"),
        DeclareLaunchArgument(
            "slam_only", default_value="false",
            description=(
                "Sensor/SLAM validation mode: run MuJoCo, per-robot RSP, "
                "sensor bridges, Fast-LIO and octomap, but skip CHAMP "
                "controllers, stand-up, nav planners and CFPA2."
            ),
        ),
        DeclareLaunchArgument("cleanup_stale", default_value="true"),
        DeclareLaunchArgument(
            "mujoco_cameras", default_value="false",
            description=(
                "Start MJCF RGB-D camera publishers. Default false keeps this "
                "LiDAR/odometry SLAM launch headless-safe; set true for "
                "VLM/camera tests."
            ),
        ),
        DeclareLaunchArgument(
            "map_merge", default_value="true",
            description=(
                "Run map merge when the selected relative_pose_source can provide "
                "an inter-robot transform. Disabled automatically for "
                "relative_pose_source:=none."
            ),
        ),
        DeclareLaunchArgument(
            "relative_pose_source", default_value="none",
            description=(
                "none | gt | discovered. none assumes robots do not know their "
                "initial relative pose and disables GT bootstrap/map_merge; gt "
                "keeps legacy MuJoCo/bootstrap behavior; discovered waits for "
                "team_loop_closure to estimate robot_a/map -> robot_b/map."
            ),
        ),
        DeclareLaunchArgument(
            "inter_robot_loop_closure", default_value="false",
            description=(
                "Enable LiDAR-only cross-robot place recognition and "
                "alignment status topics. Does not modify ROS1 SC-PGO graphs."
            ),
        ),
        DeclareLaunchArgument(
            "team_alignment_min_matches", default_value="2",
            description="Consistent accepted cross-robot matches required before status:=aligned.",
        ),
        DeclareLaunchArgument(
            "team_alignment_max_translation_disagreement", default_value="1.0",
            description="Max translation disagreement among accepted cross-robot transforms before rejecting alignment.",
        ),
        DeclareLaunchArgument(
            "team_alignment_max_yaw_disagreement_deg", default_value="10.0",
            description="Max yaw disagreement among accepted cross-robot transforms before rejecting alignment.",
        ),
        DeclareLaunchArgument(
            "team_alignment_single_match_debug", default_value="false",
            description="Debug only: allow one high-confidence match to publish status:=aligned.",
        ),
        DeclareLaunchArgument(
            "team_alignment_enable_map_merge", default_value="true",
            description="When relative_pose_source:=discovered, start map_merge only after team alignment.",
        ),
        DeclareLaunchArgument(
            "enable_gt_drift_metrics", default_value="false",
            description="Subscribe to sim /odom/ground_truth only for eval-only drift metrics; keep false for no-GT runtime.",
        ),
        DeclareLaunchArgument("scan_context_num_rings", default_value="20"),
        DeclareLaunchArgument("scan_context_num_sectors", default_value="60"),
        DeclareLaunchArgument("scan_context_max_radius", default_value="80.0"),
        DeclareLaunchArgument(
            "registration_backend", default_value="icp_2d",
            description="Cross-robot registration backend interface. v2 keeps self-contained icp_2d only.",
        ),
        DeclareLaunchArgument(
            "deployment_mode", default_value="sim_ros2",
            description="sim_ros2 | sim_hybrid_ros1_slam_ros2_nav | real_hybrid_ros1_slam_ros2_nav | real_ros1_only_experimental.",
        ),
        DeclareLaunchArgument(
            "slam_backend", default_value="fast_lio_scpgo",
            description="fast_lio_scpgo | swarm_lio2_shadow | swarm_lio2_primary.",
        ),
        DeclareLaunchArgument(
            "dynamic_filter_backend", default_value="none",
            description="dynamic_lio_port | dynamic_lio_wrapper | temporal_voxel_fallback | none.",
        ),
        DeclareLaunchArgument(
            "static_map_cleanup_backend", default_value="none",
            description="erasor_wrapper | temporal_voxel_fallback | none.",
        ),
        DeclareLaunchArgument(
            "erasor_trigger_mode", default_value="manual",
            description="manual | periodic | benchmark.",
        ),
        DeclareLaunchArgument(
            "require_swarm_loop_agreement", default_value="true",
            description="Require Swarm-LIO2 mutual transform to agree with robust loop closure in swarm_lio2_primary mode.",
        ),
        DeclareLaunchArgument("swarm_loop_agreement_max_translation", default_value="0.5"),
        DeclareLaunchArgument("swarm_loop_agreement_max_yaw_deg", default_value="5.0"),
        DeclareLaunchArgument(
            "robust_min_inliers", default_value="7",
            description="Robust inter-robot loop inlier set size required before accepted alignment.",
        ),
        DeclareLaunchArgument(
            "robust_min_inlier_ratio", default_value="0.25",
            description="Minimum robust inlier ratio inside the eligible pairwise-consistent component.",
        ),
        DeclareLaunchArgument(
            "robust_max_median_rmse", default_value="0.45",
            description="Maximum median ICP RMSE for the robust inter-robot inlier set.",
        ),
        DeclareLaunchArgument(
            "robust_max_translation_spread_m", default_value="1.0",
            description="Maximum translation spread of the selected robust transform estimates.",
        ),
        DeclareLaunchArgument(
            "robust_max_yaw_spread_deg", default_value="12.0",
            description="Maximum yaw spread of the selected robust transform estimates.",
        ),
        DeclareLaunchArgument(
            "robust_prefilter_max_rmse", default_value="0.45",
            description="Maximum ICP RMSE before a verified match is eligible for robust consensus.",
        ),
        DeclareLaunchArgument(
            "robust_prefilter_min_inlier_ratio", default_value="0.45",
            description="Minimum ICP inlier ratio before a verified match is eligible for robust consensus.",
        ),
        DeclareLaunchArgument(
            "robust_prefilter_min_correspondences", default_value="0",
            description="Minimum ICP correspondence count before a verified match is eligible for robust consensus.",
        ),
        DeclareLaunchArgument(
            "robust_prefilter_max_descriptor_distance", default_value="0.45",
            description="Maximum Scan Context descriptor distance before a verified match is eligible.",
        ),
        DeclareLaunchArgument(
            "robust_deduplicate_by_query_keyframe", default_value="false",
            description="Optionally keep only the best verified match per query keyframe before robust PCM selection.",
        ),
        DeclareLaunchArgument(
            "robust_deduplicate_by_match_keyframe", default_value="false",
            description="Optionally keep only the best verified match per matched keyframe before robust PCM selection.",
        ),
        DeclareLaunchArgument(
            "robust_deduplicate_transform_bin_translation_m", default_value="0.0",
            description="Optional translation bin size for transform-cluster de-duplication; 0 disables.",
        ),
        DeclareLaunchArgument(
            "robust_deduplicate_transform_bin_yaw_deg", default_value="0.0",
            description="Optional yaw bin size for transform-cluster de-duplication; 0 disables.",
        ),
        DeclareLaunchArgument(
            "team_pose_graph_backend", default_value="auto",
            description="auto | gtsam_python | gtsam_cpp | g2o_export_only. auto uses available GTSAM, otherwise explicit export-only mode.",
        ),
        DeclareLaunchArgument(
            "team_alignment_allow_export_only_gate", default_value="false",
            description="Debug/eval only: allow map-merge alignment gate when pose graph is export-only rather than optimized.",
        ),
        DeclareLaunchArgument(
            "alignment_reject_timeout_sec", default_value="60.0",
            description="After this many seconds without robust acceptance, publish rejected status for no-overlap diagnostics.",
        ),
        DeclareLaunchArgument(
            "alignment_reject_min_verified_matches", default_value="7",
            description="Verified-match evidence threshold for publishing rejected status before robust acceptance.",
        ),
        DeclareLaunchArgument("use_dynamic_filter", default_value="false"),
        DeclareLaunchArgument("dynamic_filter_enabled", default_value="false"),
        DeclareLaunchArgument("decentralized_mode", default_value="false"),
        DeclareLaunchArgument("robot_id", default_value="robot_a"),
        DeclareLaunchArgument("peer_robot_id", default_value="robot_b"),
        DeclareLaunchArgument("team_comm_mode", default_value="dds"),
        DeclareLaunchArgument(
            "mujoco_model_path", default_value="",
            description="Path to MJCF scene. Defaults to demo3_mixed.xml (Go2W robot_a + Go2 robot_b).",
        ),
        DeclareLaunchArgument("session_duration_sec", default_value="0.0"),
        DeclareLaunchArgument("session_output_dir", default_value=""),
        DeclareLaunchArgument("scene_area_m2", default_value="384.0"),
        DeclareLaunchArgument(
            "collision_output_path",
            default_value="/tmp/dual_robot_collision_report.json",
        ),
        DeclareLaunchArgument(
            "nav_backend_a", default_value="nav2_mppi",
            description=(
                "Nav backend for robot_a (Go2W). Default 'nav2_mppi' "
                "(SmacPlannerHybrid + MPPI + behavior_server, polygon "
                "footprint, stuck_watchdog) — production stack since "
                "2026-04-29. Other options: 'far' | 'astar' | 'none'."
            ),
        ),
        DeclareLaunchArgument(
            "nav_backend_b", default_value="nav2_mppi",
            description=(
                "Nav backend for robot_b (Go2). Default 'nav2_mppi' "
                "(uses nav2_go2_full_stack.yaml: 0.65×0.30 m footprint, "
                "vx_max 0.30, in-place pivot via min_turning_radius=0.05). "
                "Other options: 'far' | 'astar' | 'none'."
            ),
        ),
        DeclareLaunchArgument(
            "loop_closure", default_value="false",
            description=(
                "Enable Fast-LIO2 loop-closure post-processor (per-namespace). "
                "When true, backend selection is controlled by "
                "loop_closure_backend. The ROS1 bridge backend expects "
                "Docker to run native ROS1 fast_lio_sam_node and bridge "
                "/<ns>/sc_pgo/pose_stamped back into ROS2; this launch then "
                "converts it to /<ns>/corrected_odom for fast_lio_tf_adapter."
            ),
        ),
        DeclareLaunchArgument(
            "loop_closure_backend", default_value="auto",
            description=(
                "auto | ros2_sc_pgo | ros1_bridge. auto first tries a native "
                "ROS2 sc_pgo package; if unavailable, starts the ROS2 "
                "PoseStamped->Odometry adapter for the Docker ROS1 bridge. "
                "Use ros1_bridge to force the Docker path."
            ),
        ),
        DeclareLaunchArgument(
            "sensor_trust_robot_a", default_value="1.00",
            description="Static sensor trust for robot_a / Go2W. Default is 1.00 because both robots now use Livox MID-360; lower values are only for old L1 ablations.",
        ),
        DeclareLaunchArgument(
            "sensor_trust_robot_b", default_value="1.00",
            description="Static sensor trust for robot_b / Go2 MID-360.",
        ),
        DeclareLaunchArgument(
            "frontier_trust_enabled", default_value="true",
            description="Enable Stage-2 frontier trust annotation and Stage-3 utility shaping.",
        ),
        DeclareLaunchArgument(
            "frontier_validation_confidence_threshold", default_value="0.40",
            description="Frontier confidence below this value requests validator bias.",
        ),
        DeclareLaunchArgument(
            "frontier_trust_info_gain_penalty_weight", default_value="0.20",
            description="Information-gain penalty weight for uncertain frontiers.",
        ),
        DeclareLaunchArgument(
            "frontier_validation_trusted_bonus", default_value="0.25",
            description="Utility bonus for the validator on validation-required frontiers.",
        ),
        DeclareLaunchArgument(
            "frontier_validation_untrusted_penalty", default_value="0.25",
            description="Utility penalty for non-validator robots on validation-required frontiers.",
        ),
        DeclareLaunchArgument(
            "frontier_validation_robot_namespace", default_value="robot_b",
            description="Robot assigned soft validation bias for low-confidence frontiers.",
        ),
        DeclareLaunchArgument(
            "role_awareness_enabled", default_value="false",
            description="Enable CFPA2 Loop+Risk role-aware scoring inputs.",
        ),
        DeclareLaunchArgument(
            "loop_candidates_enabled", default_value="false",
            description="Run pose_graph_health + loop_closure_candidate nodes and allow loop_close goals.",
        ),
        DeclareLaunchArgument(
            "morphology_risk_enabled", default_value="false",
            description="Run morphology_risk_node and feed mobility risk into CFPA2 utility.",
        ),
        DeclareLaunchArgument(
            "peer_obstacle_enabled", default_value="false",
            description="Publish synthetic peer LaserScan obstacles for Nav2 local costmaps.",
        ),
        DeclareLaunchArgument(
            "reconstruction_quality_enabled", default_value="false",
            description=(
                "Run reconstruction_quality_node (geometry-first voxel "
                "density + view diversity) and allow CFPA2 to assign "
                "role=reconstruct goals to under-reconstructed regions."
            ),
        ),
        DeclareLaunchArgument(
            "scene_graph_enabled", default_value="false",
            description=(
                "Run scene_graph_builder_node — geometry-only 3D scene "
                "graph (rooms, corridors, doorways, obstacles, loop / "
                "reconstruction candidate nodes; connected_to / inside / "
                "near / needs_revisit edges). No VLM, no GPU."
            ),
        ),
        DeclareLaunchArgument(
            "loop_risk_output_dir", default_value="",
            description="Per-trial directory for pose_graph_health / loop_candidates / morphology_risk JSON artifacts.",
        ),
        DeclareLaunchArgument(
            "debug", default_value="false",
            description=(
                "Self-contained nav diagnostic terminal. When true, every "
                "node not on the keep-list (mujoco, fast_lio, octomap, "
                "champ, ekf, sensor bridges, terrain_analysis, rviz, "
                "map_merge, session monitors, RSP, controller spawners) "
                "is routed to the log file, leaving stdout for the path "
                "planner / controller / goal source / safety monitor: "
                "astar_nav_node, far_planner, localPlanner, pathFollower, "
                "twist_bridge, go2w_hybrid_cmd_router, far_status_adapter, "
                "cfpa2_coordinator, dual_robot_collision_monitor. The "
                "monitor emits WALL CONTACT / TIP-OVER / PLANNER STUCK "
                "warnings with a 2-second context snapshot of pose / "
                "yaw / tilt / commanded v,ω / nav_state / dist-to-goal "
                "around each event, plus a 10 s STATE+SAFETY summary "
                "line per robot including coverage % vs scene_area_m2. "
                "Logs of silenced nodes remain under ~/.ros/log/<session>/."
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
