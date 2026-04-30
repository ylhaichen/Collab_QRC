#!/usr/bin/env bash
# Equal-Livox morphology baseline (2026-04-29).
#
# Both Go2W (robot_a, wheeled-legged) and Go2 (robot_b, legged-only) mount
# Livox MID-360 with identical SLAM stack (Fast-LIO + pointlio_gazebo_mid360
# config). sensor_trust is 1.00 for both robots and frontier_trust is OFF
# by default, so any per-robot performance gap is attributable to embodiment
# alone — exactly the substrate the Loop_Closure_Reconstruction proposal
# (docs/Loop_Closure_Reconstruction_Grounded_Scene_Reports_Proposal.md §17,
# Month 1 deliverable) needs before pose-graph and reconstruction terms
# are added to CFPA2.
#
# Output per trial:
#   ${OUT_DIR}/trial_${i}/session/robot_a.json        (session_reporter)
#   ${OUT_DIR}/trial_${i}/session/robot_b.json        (session_reporter)
#   ${OUT_DIR}/trial_${i}/collision.json              (dual_robot_collision_monitor)
#   ${OUT_DIR}/trial_${i}/launch.log                  (launch stdout/stderr)
#   ${OUT_DIR}/trial_${i}/ros_logs/                   (per-node ROS logs)
# Aggregate across trials:
#   ${OUT_DIR}/summary.tsv                            (per-trial × per-robot)
#   ${OUT_DIR}/morphology_summary.json                (Go2W vs Go2 means)
#
# Env-overridable:
#   NUM_TRIALS         (default 3)
#   DURATION_SEC       (default 240; demo3_mixed = 384 m², ≥ 4 min recommended)
#   OUT_DIR            (default results/equal_livox_morphology/<ts>)
#   NAV_A / NAV_B      (default far — works without nav2_common; production
#                       stack is nav2_mppi but that requires
#                       `ros-humble-nav2-common` + bringup deps which are
#                       not installed on every dev machine. For the Month 1
#                       proposal deliverable any planner is fine as long as
#                       both robots use the same one — what matters is the
#                       morphology asymmetry surfacing under equal sensing
#                       and equal stack. Override with NAV_A=nav2_mppi
#                       NAV_B=nav2_mppi once nav2_common is installed.)
#   FRONTIER_TRUST     (default false — neutral baseline; set to true to keep
#                       trust-allocator validation hooks wired but with both
#                       sensor_trust=1.00 the term is a no-op on info_gain)
#   FRONTIER_VALIDATOR (default robot_b — only consulted if FRONTIER_TRUST=true)
#   DEBUG              (default false; matches debug:= flag of the launch)
#   SCENE_AREA_M2      (default 384.0 — demo3_mixed)
#
# Usage:
#   ./scripts/bench/benchmark_equal_livox_morphology.sh
#   NUM_TRIALS=5 DURATION_SEC=300 ./scripts/bench/benchmark_equal_livox_morphology.sh
#   NAV_A=far NAV_B=far ./scripts/bench/benchmark_equal_livox_morphology.sh
set -u -o pipefail
# do NOT use -e — individual trials may crash; we want to keep going.

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"

NUM_TRIALS="${NUM_TRIALS:-3}"
DURATION_SEC="${DURATION_SEC:-240}"
SCENE_AREA_M2="${SCENE_AREA_M2:-384.0}"
DEBUG="${DEBUG:-false}"
NAV_A="${NAV_A:-far}"
NAV_B="${NAV_B:-far}"
FRONTIER_TRUST="${FRONTIER_TRUST:-false}"
FRONTIER_VALIDATOR="${FRONTIER_VALIDATOR:-robot_b}"
# GUI / RViz on by default — the user prefers to see the sim while a trial
# runs so failures (robot wedged in a wall, map drift, RViz markers off,
# CHAMP standup glitches) are visible. For unattended CI-style runs flip
# both to false.
GUI="${GUI:-true}"
RVIZ="${RVIZ:-true}"
DEFAULT_OUT_DIR="${WS_DIR}/results/equal_livox_morphology/$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_DIR:-${DEFAULT_OUT_DIR}}"

mkdir -p "${OUT_DIR}"

echo "================================================================"
echo "  EQUAL-LIVOX MORPHOLOGY BASELINE"
echo "  workspace      : ${WS_DIR}"
echo "  trials         : ${NUM_TRIALS}"
echo "  duration/run   : ${DURATION_SEC} s"
echo "  scene area     : ${SCENE_AREA_M2} m²"
echo "  out dir        : ${OUT_DIR}"
echo "  nav_backend_a  : ${NAV_A}    (Go2W / robot_a)"
echo "  nav_backend_b  : ${NAV_B}    (Go2  / robot_b)"
echo "  sensor_trust   : robot_a=1.00, robot_b=1.00"
echo "  frontier_trust : ${FRONTIER_TRUST}  (validator=${FRONTIER_VALIDATOR})"
echo "  gui / rviz     : ${GUI} / ${RVIZ}"
echo "  debug          : ${DEBUG}"
echo "================================================================"

cleanup_procs() {
  pkill -f 'mujoco_ros2_control/mujoco_ros2_control' 2>/dev/null || true
  pkill -f 'ros2 launch go2_gazebo_sim nav_test_mujoco' 2>/dev/null || true
  pkill -f 'fast_lio'        2>/dev/null || true
  pkill -f 'far_planner'     2>/dev/null || true
  pkill -f 'session_reporter.py' 2>/dev/null || true
  pkill -f 'dual_robot_collision_monitor.py' 2>/dev/null || true
  sleep 1
  pkill -9 -f 'mujoco_ros2_control/mujoco_ros2_control' 2>/dev/null || true
  pkill -9 -f 'ros2 launch go2_gazebo_sim nav_test_mujoco' 2>/dev/null || true
  pkill -9 -f 'fast_lio'        2>/dev/null || true
  pkill -9 -f 'far_planner'     2>/dev/null || true
  sleep 1
}

trap 'echo "[benchmark] INT — cleaning up"; cleanup_procs; exit 130' INT TERM

run_trial() {
  local trial="$1"
  local trial_dir="${OUT_DIR}/trial_${trial}"
  mkdir -p "${trial_dir}/session" "${trial_dir}/ros_logs"
  local launch_log="${trial_dir}/launch.log"
  local collision_json="${trial_dir}/collision.json"

  echo
  echo "---------- trial ${trial}/${NUM_TRIALS} ----------"
  echo "  session : ${trial_dir}/session"
  echo "  log     : ${launch_log}"

  cleanup_procs

  # Outer timeout in case the launch hangs past session_duration_sec.
  local outer_timeout=$((DURATION_SEC + 60))

  (
    if [[ -f "${WS_DIR}/env.sh" ]]; then
      set +u
      source "${WS_DIR}/env.sh"
      set -u
    fi
    export ROS_LOG_DIR="${trial_dir}/ros_logs"
    timeout --signal=SIGTERM --kill-after=10 "${outer_timeout}" \
      "${WS_DIR}/scripts/launch/nav_test_demo3_mixed.sh" \
        gui:="${GUI}" \
        rviz:="${RVIZ}" \
        debug:="${DEBUG}" \
        nav_backend_a:="${NAV_A}" \
        nav_backend_b:="${NAV_B}" \
        scene_area_m2:="${SCENE_AREA_M2}" \
        session_duration_sec:="${DURATION_SEC}" \
        session_output_dir:="${trial_dir}/session" \
        collision_output_path:="${collision_json}" \
        sensor_trust_robot_a:="1.00" \
        sensor_trust_robot_b:="1.00" \
        frontier_trust_enabled:="${FRONTIER_TRUST}" \
        frontier_validation_robot_namespace:="${FRONTIER_VALIDATOR}"
  ) >"${launch_log}" 2>&1
  local rc=$?
  echo "  exit    : ${rc}"
  if [[ ! -f "${trial_dir}/session/robot_a.json" ]]; then
    echo "  WARN    : robot_a.json missing — see ${launch_log}"
  fi
  if [[ ! -f "${trial_dir}/session/robot_b.json" ]]; then
    echo "  WARN    : robot_b.json missing — see ${launch_log}"
  fi
}

for i in $(seq 1 "${NUM_TRIALS}"); do
  run_trial "${i}"
done

cleanup_procs

echo
echo "================================================================"
echo "  AGGREGATE: Go2W (robot_a) vs Go2 (robot_b) — equal Livox MID-360"
echo "================================================================"

OUT_DIR="${OUT_DIR}" SCENE_AREA_M2="${SCENE_AREA_M2}" \
/usr/bin/python3 - "${OUT_DIR}" <<'PY'
from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from statistics import mean, pstdev

root = Path(sys.argv[1])
scene_area = float(os.environ.get("SCENE_AREA_M2", "384.0"))

trial_dirs = sorted(p for p in root.iterdir() if p.is_dir() and p.name.startswith("trial_"))
if not trial_dirs:
    print(f"NO trial_* directories under {root}")
    sys.exit(1)

# Per-trial × per-robot rows.
rows: list[dict] = []
for tdir in trial_dirs:
    sess = tdir / "session"
    coll_path = tdir / "collision.json"
    coll = json.loads(coll_path.read_text()) if coll_path.exists() else {}
    coll_robots = coll.get("robots", {})
    for ns in ("robot_a", "robot_b"):
        sess_path = sess / f"{ns}.json"
        if not sess_path.exists():
            continue
        d = json.loads(sess_path.read_text())
        cov = d.get("coverage", {}) or {}
        prog = d.get("progress", {}) or {}
        slam = d.get("slam", {}) or {}
        # Prefer collision_monitor for safety (per-robot ownership; session
        # reporter sometimes mirrors the shared contact stream into both
        # rows). Fall back to session_reporter if collision.json is absent.
        robot_coll = coll_robots.get(ns, {})
        touched = robot_coll.get("ever_touched", {})
        tipped = robot_coll.get("tipped_over", {})
        degraded = robot_coll.get("degraded_tilt", {})
        if touched:
            wall_hits = touched.get("wall_hits_total", 0)
            obs_hits = touched.get("obstacle_hits_total", 0)
            tipped_flag = bool(tipped.get("tripped", False))
            peak_tilt = float(tipped.get("peak_tilt_deg", 0.0))
            degraded_flag = bool(degraded.get("currently_degraded", False))
            degraded_count = int(degraded.get("count", 0))
        else:
            saf = d.get("safety", {}) or {}
            wall_hits = saf.get("wall_contact_count", 0)
            obs_hits = saf.get("obstacle_contact_count", 0)
            tipped_flag = bool(prog.get("tipped_over", False))
            peak_tilt = float(prog.get("peak_tilt_deg", 0.0))
            deg_block = prog.get("degraded_tilt", {})
            if isinstance(deg_block, dict):
                degraded_flag = bool(deg_block.get("currently_degraded", False))
                degraded_count = int(deg_block.get("count", 0))
            else:
                degraded_flag = bool(deg_block)
                degraded_count = 0
        rows.append({
            "trial": tdir.name,
            "robot": ns,
            "morphology": "Go2W" if ns == "robot_a" else "Go2",
            "outcome": d.get("outcome", "?"),
            "elapsed_sec": float(d.get("elapsed_sec", 0.0)),
            "coverage_ratio": float(cov.get("coverage_ratio_of_scene", 0.0)),
            "explored_area_m2": float(cov.get("explored_area_m2", 0.0)),
            "distance_m": float(prog.get("distance_travelled_m", 0.0)),
            "wall_contacts": int(wall_hits),
            "obstacle_contacts": int(obs_hits),
            "total_contacts": int(wall_hits) + int(obs_hits),
            "tipped": tipped_flag,
            "peak_tilt_deg": peak_tilt,
            "degraded_tilt": degraded_flag,
            "degraded_tilt_count": degraded_count,
            "slam_trans_drift_peak_m": slam.get("trans_error_peak_m"),
            "slam_yaw_drift_peak_deg": slam.get("yaw_error_peak_deg"),
            "slam_trans_drift_final_m": slam.get("trans_error_final_m"),
            "slam_yaw_drift_final_deg": slam.get("yaw_error_final_deg"),
        })

# Pretty per-trial table.
hdr = (f"{'trial':<8} {'robot':<7} {'morph':<5} {'outcome':<10} "
       f"{'cov%':>6} {'dist m':>7} {'walls':>5} {'obs':>4} "
       f"{'tilt°':>6} {'tip':>4} {'deg':>4} {'drift m':>8} {'drift °':>8}")
print(hdr)
print("-" * len(hdr))
for r in rows:
    drift_m = r["slam_trans_drift_peak_m"]
    drift_d = r["slam_yaw_drift_peak_deg"]
    drift_m_s = f"{drift_m:.2f}" if drift_m is not None else "n/a"
    drift_d_s = f"{drift_d:.1f}" if drift_d is not None else "n/a"
    print(
        f"{r['trial']:<8} {r['robot']:<7} {r['morphology']:<5} "
        f"{r['outcome']:<10} {r['coverage_ratio']*100:>5.1f}% "
        f"{r['distance_m']:>7.2f} {r['wall_contacts']:>5d} "
        f"{r['obstacle_contacts']:>4d} {r['peak_tilt_deg']:>6.1f} "
        f"{('Y' if r['tipped'] else '-'):>4} "
        f"{r['degraded_tilt_count']:>4d} "
        f"{drift_m_s:>8} {drift_d_s:>8}"
    )

# Per-morphology aggregate.
def agg(label, sel):
    sub = [r for r in rows if sel(r)]
    if not sub:
        return None
    def m(key):
        vals = [r[key] for r in sub if r[key] is not None]
        return mean(vals) if vals else None
    def s(key):
        vals = [r[key] for r in sub if r[key] is not None]
        return pstdev(vals) if len(vals) > 1 else 0.0
    return {
        "n_trials": len(sub),
        "outcome_completed": sum(1 for r in sub if r["outcome"] == "completed"),
        "coverage_ratio_mean": m("coverage_ratio"),
        "coverage_ratio_std": s("coverage_ratio"),
        "explored_area_m2_mean": m("explored_area_m2"),
        "distance_m_mean": m("distance_m"),
        "distance_m_std": s("distance_m"),
        "total_contacts_mean": m("total_contacts"),
        "total_contacts_sum": sum(r["total_contacts"] for r in sub),
        "n_zero_contacts": sum(1 for r in sub if r["total_contacts"] == 0),
        "n_tipped": sum(1 for r in sub if r["tipped"]),
        "peak_tilt_max": max((r["peak_tilt_deg"] for r in sub), default=0.0),
        "peak_tilt_mean": m("peak_tilt_deg"),
        "degraded_tilt_count_mean": m("degraded_tilt_count"),
        "n_degraded_currently": sum(1 for r in sub if r["degraded_tilt"]),
        "slam_trans_drift_peak_mean_m": m("slam_trans_drift_peak_m"),
        "slam_yaw_drift_peak_mean_deg": m("slam_yaw_drift_peak_deg"),
    }

go2w_stats = agg("Go2W", lambda r: r["robot"] == "robot_a") or {}
go2_stats  = agg("Go2",  lambda r: r["robot"] == "robot_b") or {}

print("\n--- per-morphology means ---")
print(f"{'metric':<32} {'Go2W (robot_a)':>18} {'Go2 (robot_b)':>18}")
print("-" * 70)

def fmt(v, pct=False):
    if v is None:
        return "n/a"
    if pct:
        return f"{v*100:.1f}%"
    if isinstance(v, float):
        return f"{v:.3f}"
    return str(v)

key_pairs = [
    ("trials",                 "n_trials",                False),
    ("outcome=completed",      "outcome_completed",       False),
    ("coverage mean",          "coverage_ratio_mean",     True),
    ("coverage std",           "coverage_ratio_std",      True),
    ("explored area mean (m²)", "explored_area_m2_mean",   False),
    ("distance mean (m)",      "distance_m_mean",         False),
    ("distance std (m)",       "distance_m_std",          False),
    ("contacts mean",          "total_contacts_mean",     False),
    ("contacts sum",           "total_contacts_sum",      False),
    ("zero-contact trials",    "n_zero_contacts",         False),
    ("tipped trials",          "n_tipped",                False),
    ("peak tilt max (°)",      "peak_tilt_max",           False),
    ("peak tilt mean (°)",     "peak_tilt_mean",          False),
    ("degraded count mean",    "degraded_tilt_count_mean", False),
    ("trans drift peak (m)",   "slam_trans_drift_peak_mean_m",  False),
    ("yaw drift peak (°)",     "slam_yaw_drift_peak_mean_deg",  False),
]
for label, key, pct in key_pairs:
    a = fmt(go2w_stats.get(key), pct)
    b = fmt(go2_stats.get(key),  pct)
    print(f"{label:<32} {a:>18} {b:>18}")

# Plain TSV for downstream analysis.
tsv = root / "summary.tsv"
cols = [
    "trial", "robot", "morphology", "outcome", "elapsed_sec",
    "coverage_ratio", "explored_area_m2", "distance_m",
    "wall_contacts", "obstacle_contacts", "total_contacts",
    "tipped", "peak_tilt_deg", "degraded_tilt", "degraded_tilt_count",
    "slam_trans_drift_peak_m", "slam_yaw_drift_peak_deg",
    "slam_trans_drift_final_m", "slam_yaw_drift_final_deg",
]
with tsv.open("w") as f:
    f.write("\t".join(cols) + "\n")
    for r in rows:
        f.write("\t".join("" if r.get(c) is None else str(r.get(c, "")) for c in cols) + "\n")
print(f"\nsummary.tsv               : {tsv}")

morph_json = root / "morphology_summary.json"
morph_json.write_text(json.dumps({
    "scene_area_m2": scene_area,
    "n_trials": len(trial_dirs),
    "go2w_robot_a": go2w_stats,
    "go2_robot_b": go2_stats,
    "rows": rows,
}, indent=2))
print(f"morphology_summary.json   : {morph_json}")
PY

echo
echo "done. all artefacts under: ${OUT_DIR}"
