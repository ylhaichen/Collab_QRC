#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
LOGS = ROOT / "logs"


def run_json(cmd: list[str], deployment_mode: str) -> dict:
    env = os.environ.copy()
    env["DEPLOYMENT_MODE"] = deployment_mode
    proc = subprocess.run(cmd, cwd=ROOT, env=env, text=True, capture_output=True)
    if proc.returncode not in (0, 1):
        return {
            "available": False,
            "buildable": False,
            "runtime_ready": False,
            "blocker": (proc.stderr or proc.stdout or f"exit={proc.returncode}").strip(),
        }
    return json.loads(proc.stdout)


def read_json(path: Path, default: dict) -> dict:
    if not path.exists():
        return dict(default)
    return json.loads(path.read_text())


def write_json(path: Path, payload: dict) -> None:
    LOGS.mkdir(exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def write_md(path: Path, title: str, payload: dict) -> None:
    lines = [f"# {title}", ""]
    for key, value in payload.items():
        if isinstance(value, (str, int, float, bool)) or value is None:
            lines.append(f"- {key}: `{value}`")
    path.write_text("\n".join(lines) + "\n")


def backend_check_arg(deployment_mode: str) -> str:
    if deployment_mode in {"sim_ros2", "sim_hybrid_ros1_slam_ros2_nav"}:
        return "--docker"
    return "--host"


def backend_checks(deployment_mode: str) -> tuple[dict, dict, dict]:
    mode_arg = backend_check_arg(deployment_mode)
    return (
        run_json(["bash", "scripts/setup/check_swarm_lio2.sh", mode_arg], deployment_mode),
        run_json(["bash", "scripts/setup/check_dynamic_lio.sh", mode_arg], deployment_mode),
        run_json(["bash", "scripts/setup/check_erasor.sh", mode_arg], deployment_mode),
    )


def baseline() -> dict:
    data = read_json(LOGS / "cross_loop_closure_final_eval.json", {})
    return {
        "overlap_pass": bool(data.get("overlap_pass", False)),
        "no_overlap_pass": bool(data.get("no_overlap_pass", False)),
        "gt_used_runtime": bool(
            data.get("gt_used_runtime", False)
            or data.get("overlap", {}).get("gt_used_runtime", False)
            or data.get("no_overlap", {}).get("gt_used_runtime", False)
        ),
    }


def docker_backend_status() -> dict:
    return read_json(LOGS / "ros1_hybrid_docker_build_status.json", {})


def baseline_rerun_status() -> dict:
    return read_json(LOGS / "baseline_regression_rerun_status.json", {})


def shadow(deployment_mode: str) -> dict:
    swarm, _, _ = backend_checks(deployment_mode)
    runtime_ready = bool(swarm.get("runtime_ready", False))
    payload = {
        "schema": "swarm_lio2_shadow_validation/v2",
        "deployment_mode": deployment_mode,
        "slam_backend": "swarm_lio2_shadow",
        "swarm_lio2_source_available": bool(swarm.get("available", False)),
        "swarm_lio2_buildable": bool(swarm.get("buildable", False)),
        "swarm_lio2_runtime_ready": runtime_ready,
        "swarm_lio2_started": False,
        "swarm_lio2_odometry_valid": False,
        "swarm_lio2_relative_state_valid": False,
        "ros2_receives_shadow_odometry": False,
        "fast_lio_baseline_still_runs": baseline()["overlap_pass"] and baseline()["no_overlap_pass"],
        "production_downstream_depends_on_swarm": False,
        "metrics_recorded": True,
        "gt_used_runtime": False,
        "blocker": "" if runtime_ready else str(swarm.get("blocker", "swarm_lio2_runtime_not_ready")),
    }
    write_json(LOGS / "swarm_lio2_shadow_validation.json", payload)
    return payload


def primary(deployment_mode: str) -> dict:
    swarm, _, _ = backend_checks(deployment_mode)
    runtime_ready = bool(swarm.get("runtime_ready", False))
    payload = {
        "schema": "swarm_lio2_primary_validation/v2",
        "deployment_mode": deployment_mode,
        "slam_backend": "swarm_lio2_primary",
        "swarm_lio2_source_available": bool(swarm.get("available", False)),
        "swarm_lio2_buildable": bool(swarm.get("buildable", False)),
        "swarm_lio2_runtime_ready": runtime_ready,
        "adapter_contract_configured": True,
        "real_nav2_odom_contract_configured": True,
        "odometry_valid": False,
        "corrected_odom_valid": False,
        "cloud_static_or_registered_valid": False,
        "nav2_runtime_valid": False,
        "team_loop_closure_keyframes_valid": False,
        "overlap_pass": False,
        "no_overlap_pass": False,
        "dynamic_object_pass": False,
        "erasor_cleanup_pass": False,
        "loop_closure_agreement_gate_pass": False,
        "gt_used_runtime": False,
        "merged_map_agreement_gated": True,
        "blocker": "" if runtime_ready else str(swarm.get("blocker", "swarm_lio2_runtime_not_ready")),
    }
    write_json(LOGS / "swarm_lio2_primary_validation.json", payload)
    return payload


def dynamic(deployment_mode: str) -> dict:
    _, dyn, _ = backend_checks(deployment_mode)
    dyn_prev = read_json(LOGS / "dynamic_filter_validation.json", {})
    runtime_ready = bool(dyn.get("runtime_ready", False))
    payload = {
        "schema": "dynamic_lio_filter_integration/v2",
        "deployment_mode": deployment_mode,
        "dynamic_lio_source_available": bool(dyn.get("available", False)),
        "dynamic_lio_buildable": bool(dyn.get("buildable", False)),
        "dynamic_lio_runtime_ready": runtime_ready,
        "dynamic_filter_backend": "dynamic_lio_wrapper" if runtime_ready else "temporal_voxel_fallback",
        "dynamic_points_filtered": int(dyn_prev.get("dynamic_points_filtered", 0) or 0),
        "static_points_kept": int(dyn_prev.get("static_points_kept", 0) or 0),
        "dynamic_filter_ratio": float(dyn_prev.get("dynamic_filter_ratio", 0.0) or 0.0),
        "stale_obstacle_decay_time_sec": dyn_prev.get("stale_obstacle_decay_time_sec"),
        "fallback_used": not runtime_ready,
        "gt_used_runtime": False,
        "blocker": "" if runtime_ready else str(dyn.get("blocker", "dynamic_lio_runtime_not_ready")),
    }
    write_json(LOGS / "dynamic_lio_filter_integration.json", payload)
    return payload


def erasor(deployment_mode: str) -> dict:
    _, _, er = backend_checks(deployment_mode)
    runtime_ready = bool(er.get("runtime_ready", False))
    payload = {
        "schema": "erasor_map_cleanup_validation/v2",
        "deployment_mode": deployment_mode,
        "static_map_cleanup_backend": "erasor_wrapper" if runtime_ready else "temporal_voxel_fallback",
        "erasor_source_available": bool(er.get("available", False)),
        "erasor_buildable": bool(er.get("buildable", False)),
        "erasor_runtime_ready": runtime_ready,
        "naive_map_contains_dynamic_trace": False,
        "cleaned_map_removes_dynamic_trace": False,
        "static_walls_preserved": False,
        "cleaned_map_published": False,
        "control_loop_blocked": False,
        "fallback_used": not runtime_ready,
        "gt_used_runtime": False,
        "blocker": "" if runtime_ready else str(er.get("blocker", "erasor_runtime_not_ready")),
    }
    write_json(LOGS / "erasor_map_cleanup_validation.json", payload)
    return payload


def sim() -> dict:
    mode = "sim_hybrid_ros1_slam_ros2_nav"
    sh = shadow(mode)
    pr = primary(mode)
    dy = dynamic(mode)
    er = erasor(mode)
    base = baseline()
    docker_status = docker_backend_status()
    docker_blocker = str(docker_status.get("blocker", "") or "")
    payload = {
        "schema": "sim_hybrid_ros1_slam_ros2_nav_validation/v1",
        "deployment_mode": mode,
        "baseline_fast_lio": base,
        "baseline_regression_rerun_status": baseline_rerun_status(),
        "ros1_hybrid_docker_build_status": docker_status,
        "shadow": sh,
        "primary": pr,
        "dynamic_filter": dy,
        "erasor_cleanup": er,
        "pass": False,
        "final_status": "Status D \u2014 External Blocker",
        "blocker": docker_blocker or sh.get("blocker") or pr.get("blocker") or dy.get("blocker") or er.get("blocker"),
        "docker_backend_build_blocker": docker_blocker,
        "claim": "Fast-LIO remains production backend; sim hybrid Swarm-LIO2 primary is not validated.",
        "implemented_scaffolding": True,
        "docker_image_build_passed": bool(
            docker_status.get("docker_image_build_passed", False)
            or docker_status.get("docker_image_build_success", False)
            or docker_status.get("pass", False)
        ),
        "docker_run_blocked": bool(docker_blocker),
        "real_robot_available": False,
        "next_manual_commands": [
            "bash scripts/manual/run_swarm_lio2_docker_build_and_test.sh",
            "bash scripts/manual/run_dynamic_lio_docker_build_and_test.sh",
            "bash scripts/manual/run_erasor_docker_build_and_test.sh",
            "bash scripts/manual/run_sim_hybrid_full_validation.sh",
        ],
    }
    write_json(LOGS / "sim_hybrid_ros1_slam_ros2_nav_validation.json", payload)
    write_md(LOGS / "sim_hybrid_ros1_slam_ros2_nav_validation.md", "Sim Hybrid ROS1 SLAM / ROS2 Nav Validation", payload)
    return payload


def real() -> dict:
    check_path = LOGS / "real_hybrid_ros1_slam_ros2_nav_check.json"
    check = read_json(check_path, {"pass": False, "blocker": "real preflight has not been run"})
    sh = shadow("real_hybrid_ros1_slam_ros2_nav")
    pr = primary("real_hybrid_ros1_slam_ros2_nav")
    payload = {
        "schema": "real_hybrid_ros1_slam_ros2_nav_validation/v1",
        "deployment_mode": "real_hybrid_ros1_slam_ros2_nav",
        "preflight": check,
        "shadow": sh,
        "primary": pr,
        "pass": False,
        "final_status": "Status D \u2014 External Blocker",
        "blocker": check.get("blocker") or sh.get("blocker") or pr.get("blocker"),
        "claim": "Fast-LIO remains production backend on real robot; real hybrid Swarm-LIO2 primary is not validated.",
        "implemented_scaffolding": True,
        "real_robot_available": bool(check.get("pass", False)),
        "docker_run_blocked": False,
        "next_manual_commands": [
            "CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_shadow_validation.sh",
            "CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_primary_validation.sh",
        ],
    }
    write_json(LOGS / "real_hybrid_ros1_slam_ros2_nav_validation.json", payload)
    write_md(LOGS / "real_hybrid_ros1_slam_ros2_nav_validation.md", "Real Hybrid ROS1 SLAM / ROS2 Nav Validation", payload)
    return payload


def comparison() -> dict:
    sim_payload = read_json(LOGS / "sim_hybrid_ros1_slam_ros2_nav_validation.json", {})
    real_payload = read_json(LOGS / "real_hybrid_ros1_slam_ros2_nav_validation.json", {})
    payload = {
        "schema": "slam_backend_comparison/v2",
        "default_deployment_mode": "sim_ros2",
        "default_slam_backend": "fast_lio_scpgo",
        "fast_lio_scpgo": baseline(),
        "baseline_regression_rerun_status": baseline_rerun_status(),
        "ros1_hybrid_docker_build_status": docker_backend_status(),
        "sim_hybrid_ros1_slam_ros2_nav": sim_payload,
        "real_hybrid_ros1_slam_ros2_nav": real_payload,
        "final_status": "Status D \u2014 External Blocker",
        "claim": "Migration interface and hybrid deployment scaffolding are implemented; Fast-LIO remains production backend.",
        "origin_push_policy": "origin push intentionally skipped when origin points to HanshangZhu/Collab_QRC; fork-only push policy is active.",
        "next_manual_commands": [
            "bash scripts/manual/run_sim_hybrid_full_validation.sh",
            "CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_shadow_validation.sh",
            "CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_primary_validation.sh",
        ],
    }
    write_json(LOGS / "slam_backend_comparison.json", payload)
    write_md(LOGS / "slam_backend_comparison.md", "SLAM Backend Comparison", payload)
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["sim", "real", "shadow", "primary", "dynamic", "erasor", "comparison"])
    parser.add_argument("--deployment-mode", default="sim_hybrid_ros1_slam_ros2_nav")
    args = parser.parse_args()
    if args.mode == "sim":
        payload = sim()
    elif args.mode == "real":
        payload = real()
    elif args.mode == "shadow":
        payload = shadow(args.deployment_mode)
    elif args.mode == "primary":
        payload = primary(args.deployment_mode)
    elif args.mode == "dynamic":
        payload = dynamic(args.deployment_mode)
    elif args.mode == "erasor":
        payload = erasor(args.deployment_mode)
    else:
        payload = comparison()
    print(json.dumps(payload, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
