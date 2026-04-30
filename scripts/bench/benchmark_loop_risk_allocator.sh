#!/usr/bin/env bash
# Loop-Closure + Mobility-Risk allocator benchmark.
#
# Modes:
#   coverage_only_mppi      — MPPI + CFPA2 frontier allocation only
#   loop_only_mppi          — add pose_graph_health + loop candidates + CFPA2 loop role
#   loop_risk_mppi          — add mobility risk + peer obstacle scan
#   loop_risk_recon_mppi    — full Loop+Risk + reconstruction_quality_node
#                             (geometry-first voxel density + view diversity
#                             → CFPA2 role=reconstruct candidates)
#   loop_risk_recon_graph_mppi
#                           — adds geometry-only scene_graph_builder_node
#                             (rooms / corridors / doorways / obstacles +
#                             loop / reconstruct candidate nodes wired
#                             into mission_summary topology section).
set -u -o pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"

MODE="${1:-loop_risk_mppi}"
NUM_TRIALS="${NUM_TRIALS:-3}"
DURATION_SEC="${DURATION_SEC:-240}"
SCENE_AREA_M2="${SCENE_AREA_M2:-384.0}"
DEBUG="${DEBUG:-false}"
GUI="${GUI:-true}"
RVIZ="${RVIZ:-true}"
NAV_A="${NAV_A:-nav2_mppi}"
NAV_B="${NAV_B:-nav2_mppi}"
FRONTIER_TRUST="${FRONTIER_TRUST:-false}"

ROLE_AWARENESS=false
LOOP_CANDIDATES=false
MORPHOLOGY_RISK=false
PEER_OBSTACLE=false
RECON_QUALITY=false
SCENE_GRAPH=false
case "${MODE}" in
  coverage_only_mppi)
    ;;
  loop_only_mppi)
    ROLE_AWARENESS=true
    LOOP_CANDIDATES=true
    ;;
  loop_risk_mppi)
    ROLE_AWARENESS=true
    LOOP_CANDIDATES=true
    MORPHOLOGY_RISK=true
    PEER_OBSTACLE=true
    ;;
  loop_risk_recon_mppi)
    ROLE_AWARENESS=true
    LOOP_CANDIDATES=true
    MORPHOLOGY_RISK=true
    PEER_OBSTACLE=true
    RECON_QUALITY=true
    ;;
  loop_risk_recon_graph_mppi)
    ROLE_AWARENESS=true
    LOOP_CANDIDATES=true
    MORPHOLOGY_RISK=true
    PEER_OBSTACLE=true
    RECON_QUALITY=true
    SCENE_GRAPH=true
    ;;
  *)
    echo "ERROR: unknown mode '${MODE}' (coverage_only_mppi | loop_only_mppi | loop_risk_mppi | loop_risk_recon_mppi | loop_risk_recon_graph_mppi)" >&2
    exit 2
    ;;
esac

DEFAULT_OUT_DIR="${WS_DIR}/results/loop_risk_allocator/${MODE}_$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_DIR:-${DEFAULT_OUT_DIR}}"
mkdir -p "${OUT_DIR}"

echo "================================================================"
echo "  LOOP + RISK ALLOCATOR BENCHMARK"
echo "  mode           : ${MODE}"
echo "  trials         : ${NUM_TRIALS}"
echo "  duration/run   : ${DURATION_SEC} s"
echo "  nav backends   : ${NAV_A} / ${NAV_B}"
echo "  role/loop/risk : ${ROLE_AWARENESS} / ${LOOP_CANDIDATES} / ${MORPHOLOGY_RISK}"
echo "  peer obstacle  : ${PEER_OBSTACLE}"
echo "  recon quality  : ${RECON_QUALITY}"
echo "  scene graph    : ${SCENE_GRAPH}"
echo "  gui / rviz     : ${GUI} / ${RVIZ}"
echo "  out dir        : ${OUT_DIR}"
echo "================================================================"

cleanup_procs() {
  pkill -f 'ros2 launch go2_gazebo_sim nav_test_mujoco' 2>/dev/null || true
  pkill -f 'mujoco_ros2_control/mujoco_ros2_control' 2>/dev/null || true
  pkill -f 'pose_graph_health_node' 2>/dev/null || true
  pkill -f 'loop_closure_candidate_node' 2>/dev/null || true
  pkill -f 'morphology_risk_node' 2>/dev/null || true
  pkill -f 'peer_obstacle_scan_node' 2>/dev/null || true
  pkill -f 'session_reporter.py' 2>/dev/null || true
  pkill -f 'dual_robot_collision_monitor.py' 2>/dev/null || true
  sleep 1
  pkill -9 -f 'ros2 launch go2_gazebo_sim nav_test_mujoco' 2>/dev/null || true
  pkill -9 -f 'mujoco_ros2_control/mujoco_ros2_control' 2>/dev/null || true
}

trap 'echo "[benchmark] interrupted — cleaning up"; cleanup_procs; exit 130' INT TERM

run_trial() {
  local trial="$1"
  local trial_dir="${OUT_DIR}/trial_${trial}"
  mkdir -p "${trial_dir}/session" "${trial_dir}/ros_logs"
  local launch_log="${trial_dir}/launch.log"
  local collision_json="${trial_dir}/collision.json"
  local outer_timeout=$((DURATION_SEC + 90))

  echo
  echo "---------- trial ${trial}/${NUM_TRIALS} (${MODE}) ----------"
  echo "  dir     : ${trial_dir}"
  echo "  log     : ${launch_log}"
  cleanup_procs

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
        role_awareness_enabled:="${ROLE_AWARENESS}" \
        loop_candidates_enabled:="${LOOP_CANDIDATES}" \
        morphology_risk_enabled:="${MORPHOLOGY_RISK}" \
        peer_obstacle_enabled:="${PEER_OBSTACLE}" \
        reconstruction_quality_enabled:="${RECON_QUALITY}" \
        scene_graph_enabled:="${SCENE_GRAPH}" \
        loop_risk_output_dir:="${trial_dir}"
  ) >"${launch_log}" 2>&1
  local rc=$?
  echo "  exit    : ${rc}"
  for f in session/robot_a.json session/robot_b.json collision.json pose_graph_health.json loop_candidates.json morphology_risk.json reconstruction_quality.json scene_graph.json; do
    if [[ ! -f "${trial_dir}/${f}" ]]; then
      echo "  WARN    : missing ${f}"
    fi
  done
  # Stage 5: offline mission_summary.{md,json} + evidence_index.json
  # generated from existing artefacts. Pure metric → narrative; no VLM.
  if [[ -f "${trial_dir}/session/robot_a.json" ]]; then
    python3 "${WS_DIR}/scripts/runtime/mission_summary_generator.py" \
      "${trial_dir}" 2>&1 | sed 's/^/  /'
  fi
}

for i in $(seq 1 "${NUM_TRIALS}"); do
  run_trial "${i}"
done

cleanup_procs

OUT_DIR="${OUT_DIR}" MODE="${MODE}" SCENE_AREA_M2="${SCENE_AREA_M2}" /usr/bin/python3 - <<'PY'
from __future__ import annotations

import json
import os
import re
from pathlib import Path
from statistics import mean, pstdev

root = Path(os.environ["OUT_DIR"])
mode = os.environ["MODE"]
rows = []
loop_counts = []
health_states = []
risk_scores = {"robot_a": [], "robot_b": []}
assignment_role_counts = {
    "robot_a": {"explore": 0, "loop_close": 0},
    "robot_b": {"explore": 0, "loop_close": 0},
}
first_loop_close_assignment = None
recovery_counts = {
    "watchdog_frontier_replan": 0,
    "cfpa2_frontier_replan_blacklist": 0,
    "cfpa2_fast_blacklist": 0,
    "cfpa2_stuck_recovery": 0,
}
first_recovery_event = None
role_line_re = re.compile(r"\b(robot_[ab]): .*?\brole=([A-Za-z0-9_]+)")

for tdir in sorted(root.glob("trial_*")):
    coll = json.loads((tdir / "collision.json").read_text()) if (tdir / "collision.json").exists() else {}
    coll_robots = coll.get("robots", {})
    loop_payload = json.loads((tdir / "loop_candidates.json").read_text()) if (tdir / "loop_candidates.json").exists() else {}
    health_payload = json.loads((tdir / "pose_graph_health.json").read_text()) if (tdir / "pose_graph_health.json").exists() else {}
    risk_payload = json.loads((tdir / "morphology_risk.json").read_text()) if (tdir / "morphology_risk.json").exists() else {}
    loop_counts.append(int(loop_payload.get("candidate_count", 0) or 0))
    launch_log = tdir / "launch.log"
    if launch_log.exists():
        for line in launch_log.read_text(errors="replace").splitlines():
            if "Published frontier_replan on" in line:
                recovery_counts["watchdog_frontier_replan"] += 1
                if first_recovery_event is None:
                    first_recovery_event = {"trial": tdir.name, "line": line.strip()}
            if "frontier_replan received — blacklisting current goal" in line:
                recovery_counts["cfpa2_frontier_replan_blacklist"] += 1
                if first_recovery_event is None:
                    first_recovery_event = {"trial": tdir.name, "line": line.strip()}
            if "FAST-BL goal=" in line:
                recovery_counts["cfpa2_fast_blacklist"] += 1
            if "CFPA2 stuck-recovery triggered" in line:
                recovery_counts["cfpa2_stuck_recovery"] += 1
            match = role_line_re.search(line)
            if not match:
                continue
            ns, role = match.groups()
            ns_counts = assignment_role_counts.setdefault(ns, {})
            ns_counts[role] = ns_counts.get(role, 0) + 1
            if role == "loop_close" and first_loop_close_assignment is None:
                first_loop_close_assignment = {
                    "trial": tdir.name,
                    "robot": ns,
                    "line": line.strip(),
                }
    for ns, state in (health_payload.get("robots", {}) or {}).items():
        if isinstance(state, dict):
            health_states.append(str(state.get("state", "unknown")))
    for ns, risk in (risk_payload.get("robots", {}) or {}).items():
        if ns in risk_scores and isinstance(risk, dict):
            risk_scores[ns].append(float(risk.get("risk_score", 0.0) or 0.0))
    for ns in ("robot_a", "robot_b"):
        sp = tdir / "session" / f"{ns}.json"
        if not sp.exists():
            continue
        sess = json.loads(sp.read_text())
        cov = sess.get("coverage", {}) or {}
        prog = sess.get("progress", {}) or {}
        slam = sess.get("slam", {}) or {}
        touched = (coll_robots.get(ns, {}) or {}).get("ever_touched", {}) or {}
        tipped = (coll_robots.get(ns, {}) or {}).get("tipped_over", {}) or {}
        rows.append({
            "trial": tdir.name,
            "robot": ns,
            "morph": "Go2W" if ns == "robot_a" else "Go2",
            "coverage": float(cov.get("coverage_ratio_of_scene", 0.0) or 0.0),
            "distance": float(prog.get("distance_travelled_m", 0.0) or 0.0),
            "wall_contacts": int(touched.get("wall_hits_total", 0) or 0),
            "obstacle_contacts": int(touched.get("obstacle_hits_total", 0) or 0),
            "tipped": bool(tipped.get("tripped", prog.get("tipped_over", False))),
            "yaw_drift": float(slam.get("yaw_error_peak_deg", 0.0) or 0.0),
        })

summary_tsv = root / "summary.tsv"
with summary_tsv.open("w", encoding="utf-8") as f:
    f.write("mode\ttrial\trobot\tmorph\tcoverage_pct\tdistance_m\twall_contacts\tobstacle_contacts\ttipped\tyaw_drift_deg\n")
    for r in rows:
        f.write(
            f"{mode}\t{r['trial']}\t{r['robot']}\t{r['morph']}\t"
            f"{100.0*r['coverage']:.2f}\t{r['distance']:.3f}\t"
            f"{r['wall_contacts']}\t{r['obstacle_contacts']}\t"
            f"{int(r['tipped'])}\t{r['yaw_drift']:.3f}\n"
        )

by_robot = {}
for ns in ("robot_a", "robot_b"):
    rs = [r for r in rows if r["robot"] == ns]
    by_robot[ns] = {
        "trials": len(rs),
        "coverage_mean": mean([r["coverage"] for r in rs]) if rs else None,
        "distance_mean_m": mean([r["distance"] for r in rs]) if rs else None,
        "distance_std_m": pstdev([r["distance"] for r in rs]) if len(rs) > 1 else 0.0,
        "obstacle_contacts_sum": sum(r["obstacle_contacts"] for r in rs),
        "wall_contacts_sum": sum(r["wall_contacts"] for r in rs),
        "tipped_trials": sum(1 for r in rs if r["tipped"]),
        "yaw_drift_peak_deg": max([r["yaw_drift"] for r in rs], default=0.0),
        "risk_score_mean": mean(risk_scores[ns]) if risk_scores[ns] else None,
    }

summary = {
    "schema": "loop_risk_summary/v1",
    "mode": mode,
    "trial_count": len({r["trial"] for r in rows}),
    "robots": by_robot,
    "loop_candidate_count_mean": mean(loop_counts) if loop_counts else 0.0,
    "loop_candidate_count_max": max(loop_counts, default=0),
    "pose_health_states_seen": sorted(set(health_states)),
    "assignment_role_counts": assignment_role_counts,
    "loop_close_assignment_count": sum(
        counts.get("loop_close", 0) for counts in assignment_role_counts.values()
    ),
    "loop_close_assignment_seen": any(
        counts.get("loop_close", 0) > 0 for counts in assignment_role_counts.values()
    ),
    "first_loop_close_assignment": first_loop_close_assignment,
    "recovery_counts": recovery_counts,
    "frontier_replan_recovery_seen": recovery_counts["watchdog_frontier_replan"] > 0
    and recovery_counts["cfpa2_frontier_replan_blacklist"] > 0,
    "first_recovery_event": first_recovery_event,
}
(root / "loop_risk_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")

print("\n================================================================")
print(f"  LOOP+RISK SUMMARY ({mode})")
print("================================================================")
for ns, s in by_robot.items():
    cov = "n/a" if s["coverage_mean"] is None else f"{100*s['coverage_mean']:.1f}%"
    dist = "n/a" if s["distance_mean_m"] is None else f"{s['distance_mean_m']:.2f}m"
    print(
        f"{ns:7s} cov={cov:>6s} dist={dist:>8s} "
        f"obs={s['obstacle_contacts_sum']:5d} wall={s['wall_contacts_sum']:3d} "
        f"tip={s['tipped_trials']} yaw_peak={s['yaw_drift_peak_deg']:.1f} "
        f"risk_mean={s['risk_score_mean']}"
    )
print(f"loop candidates mean/max: {summary['loop_candidate_count_mean']:.1f}/{summary['loop_candidate_count_max']}")
role_parts = [
    f"{ns}:loop={assignment_role_counts.get(ns, {}).get('loop_close', 0)}"
    for ns in ("robot_a", "robot_b")
]
print(
    "loop_close assignments : "
    f"{summary['loop_close_assignment_count']} "
    f"({'seen' if summary['loop_close_assignment_seen'] else 'not seen'}) "
    + " ".join(role_parts)
)
print(
    "recovery events        : "
    f"watchdog_replan={recovery_counts['watchdog_frontier_replan']} "
    f"cfpa2_replan_bl={recovery_counts['cfpa2_frontier_replan_blacklist']} "
    f"fast_bl={recovery_counts['cfpa2_fast_blacklist']} "
    f"cfpa2_stuck={recovery_counts['cfpa2_stuck_recovery']}"
)
print(f"summary.tsv            : {summary_tsv}")
print(f"loop_risk_summary.json : {root / 'loop_risk_summary.json'}")
PY

echo
echo "done. artifacts under: ${OUT_DIR}"
