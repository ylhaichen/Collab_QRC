#!/usr/bin/env python3
"""Offline mission summary generator.

Implements proposal §12 (Grounded Report) scaffold step from §22 step 9:
build evidence-grounded mission_summary.{md,json} + evidence_index.json
from a finished trial directory's existing artefacts. No VLM, no scene
graph, no rendered keyframes — pure metric → narrative transformation.

This sets up the contract that the future VLM-grounded report will
honour:

- every claim has an `evidence` list pointing into evidence_index.json
- every evidence ID resolves to a real file path + key path
- the verifier flags any claim whose evidence list is empty as
  `unverified` per §12.4
- spatial statements quote actual map coordinates (m), not VLM-invented
  positions

Inputs (per-trial directory layout produced by benchmark_loop_risk
_allocator.sh + nav_test_mujoco_fastlio_mixed launch):

    <trial>/
        session/robot_a.json
        session/robot_b.json
        collision.json
        pose_graph_health.json        (if loop_candidates_enabled)
        loop_candidates.json          (if loop_candidates_enabled)
        morphology_risk.json          (if morphology_risk_enabled)
        reconstruction_quality.json   (if reconstruction_quality_enabled)
        launch.log                    (CFPA2 ASSIGN entries — optional)

Outputs (written to the same directory):

    <trial>/mission_summary.md
    <trial>/mission_summary.json
    <trial>/evidence_index.json

Usage:
    python3 mission_summary_generator.py <trial_dir>
"""
from __future__ import annotations

import json
import math
import os
import re
import sys
from pathlib import Path
from typing import Any


# ──────────────────────────────────────────────────────────────────────
# Evidence index — every fact we touch ends up here.
# ──────────────────────────────────────────────────────────────────────


class EvidenceIndex:
    """A monotonic id → source-pointer map. Each evidence record names
    the source file path (relative to the trial dir) and the key path
    inside that file (dot-separated). Future VLM extensions can append
    keyframe / scene-graph evidence; the contract stays the same."""

    def __init__(self, trial_dir: Path) -> None:
        self.trial_dir = trial_dir
        self.records: dict[str, dict[str, Any]] = {}
        self._counter = 0

    def add(self, source_file: str, key_path: str, value: Any, kind: str) -> str:
        self._counter += 1
        eid = f"ev_{self._counter:04d}"
        self.records[eid] = {
            "source_file": source_file,
            "key_path": key_path,
            "value": value,
            "kind": kind,
        }
        return eid

    def to_json(self) -> dict[str, Any]:
        return {
            "trial_dir": str(self.trial_dir),
            "evidence_count": len(self.records),
            "evidence": self.records,
        }


# ──────────────────────────────────────────────────────────────────────
# Loaders. All return None if the file is missing — generator stays
# robust against e.g. a trial run with reconstruction_quality_enabled
# false (no recon json).
# ──────────────────────────────────────────────────────────────────────


def _load_json(path: Path) -> Any | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None


def _coalesce(x: Any, default: Any = 0) -> Any:
    return default if x is None else x


# ──────────────────────────────────────────────────────────────────────
# Claim builders. Each returns a list of structured claims; each claim
# has `evidence` pointing into the EvidenceIndex.
# ──────────────────────────────────────────────────────────────────────


def _claims_executive(
    sess_a: dict[str, Any] | None,
    sess_b: dict[str, Any] | None,
    coll: dict[str, Any] | None,
    ev: EvidenceIndex,
) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    if sess_a is None or sess_b is None:
        return out
    cov_a = float(_coalesce(sess_a.get("coverage", {}).get("coverage_ratio_of_scene"), 0.0))
    cov_b = float(_coalesce(sess_b.get("coverage", {}).get("coverage_ratio_of_scene"), 0.0))
    cov_mean = (cov_a + cov_b) / 2.0
    elapsed = float(_coalesce(sess_a.get("elapsed_sec"), 0.0))
    eid_cov_a = ev.add("session/robot_a.json", "coverage.coverage_ratio_of_scene", cov_a, "metric")
    eid_cov_b = ev.add("session/robot_b.json", "coverage.coverage_ratio_of_scene", cov_b, "metric")
    eid_elap = ev.add("session/robot_a.json", "elapsed_sec", elapsed, "metric")
    out.append({
        "claim": (
            f"The team explored {cov_mean*100:.1f}% of the {sess_a.get('coverage',{}).get('scene_area_m2',0.0):.0f} m² scene "
            f"in {elapsed:.0f} s of trial time."
        ),
        "type": "coverage_summary",
        "confidence": 0.95,  # direct metric, not inferred
        "evidence": [eid_cov_a, eid_cov_b, eid_elap],
        "uncertainty_reason": "Coverage measured against MuJoCo ground-truth scene area; SLAM drift can mis-label cells.",
        "recommended_action": (
            "Continue exploration." if cov_mean < 0.85
            else "Coverage near-complete; switch focus to reconstruction quality and loop-closure validation."
        ),
    })
    if coll is not None:
        ra = coll.get("robots", {}).get("robot_a", {})
        rb = coll.get("robots", {}).get("robot_b", {})
        wall_a = int(_coalesce(ra.get("wall_contacts", {}).get("count"), 0))
        wall_b = int(_coalesce(rb.get("wall_contacts", {}).get("count"), 0))
        obs_a = int(_coalesce(ra.get("obstacle_contacts", {}).get("count"), 0))
        obs_b = int(_coalesce(rb.get("obstacle_contacts", {}).get("count"), 0))
        eids = [
            ev.add("collision.json", "robots.robot_a.wall_contacts.count", wall_a, "metric"),
            ev.add("collision.json", "robots.robot_b.wall_contacts.count", wall_b, "metric"),
            ev.add("collision.json", "robots.robot_a.obstacle_contacts.count", obs_a, "metric"),
            ev.add("collision.json", "robots.robot_b.obstacle_contacts.count", obs_b, "metric"),
        ]
        total = wall_a + wall_b + obs_a + obs_b
        out.append({
            "claim": (
                f"Total physical contacts during the trial: {total} "
                f"(Go2W wall={wall_a} obstacle={obs_a}; Go2 wall={wall_b} obstacle={obs_b})."
            ),
            "type": "safety_summary",
            "confidence": 0.97,
            "evidence": eids,
            "uncertainty_reason": (
                "Counts include CHAMP gait stamping when MPPI is corner-rejected; "
                "not every contact represents a true wall scuff."
            ),
            "recommended_action": (
                "No safety changes required." if total <= 30
                else "Tighten cfpa2_frontier_obstacle_clearance_m or enable peer_obstacle for the offending robot."
            ),
        })
    return out


def _claims_localization(
    sess_a: dict[str, Any] | None,
    sess_b: dict[str, Any] | None,
    pgh: dict[str, Any] | None,
    loop_cands: dict[str, Any] | None,
    ev: EvidenceIndex,
) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    if sess_a is None or sess_b is None:
        return out
    yaw_a = float(_coalesce(sess_a.get("slam", {}).get("yaw_error_peak_deg"), 0.0))
    yaw_b = float(_coalesce(sess_b.get("slam", {}).get("yaw_error_peak_deg"), 0.0))
    trans_a = float(_coalesce(sess_a.get("slam", {}).get("trans_error_peak_m"), 0.0))
    trans_b = float(_coalesce(sess_b.get("slam", {}).get("trans_error_peak_m"), 0.0))
    eids = [
        ev.add("session/robot_a.json", "slam.yaw_error_peak_deg", yaw_a, "metric"),
        ev.add("session/robot_b.json", "slam.yaw_error_peak_deg", yaw_b, "metric"),
        ev.add("session/robot_a.json", "slam.trans_error_peak_m", trans_a, "metric"),
        ev.add("session/robot_b.json", "slam.trans_error_peak_m", trans_b, "metric"),
    ]
    yaw_max = max(yaw_a, yaw_b)
    if yaw_max > 60.0:
        verdict = "catastrophic divergence (>60° yaw drift)"
        action = "Investigate Fast-LIO IMU covariance + Go2W wheel-skid noise; restart trial."
        conf = 0.90
    elif yaw_max > 15.0:
        verdict = "elevated SLAM drift"
        action = "Schedule loop-closure revisit on the high-drift robot."
        conf = 0.80
    else:
        verdict = "SLAM stable"
        action = "No localization action required."
        conf = 0.85
    out.append({
        "claim": (
            f"SLAM localization is {verdict}: peak yaw drift Go2W={yaw_a:.2f}° / Go2={yaw_b:.2f}°, "
            f"peak translation drift Go2W={trans_a:.3f} m / Go2={trans_b:.3f} m."
        ),
        "type": "localization_quality",
        "confidence": conf,
        "evidence": eids,
        "uncertainty_reason": (
            "Drift measured against MuJoCo ground-truth pose; on real robots only relative drift is observable."
        ),
        "recommended_action": action,
    })
    if pgh is not None:
        states_seen: set[str] = set()
        robots = pgh.get("robots", {})
        eid_states = []
        for ns, info in robots.items():
            st = str(info.get("state", ""))
            if st:
                states_seen.add(st)
                eid_states.append(ev.add("pose_graph_health.json", f"robots.{ns}.state", st, "state"))
        if states_seen:
            out.append({
                "claim": (
                    f"pose_graph_health observed states: "
                    f"{', '.join(sorted(states_seen))}."
                ),
                "type": "pose_graph_health_state",
                "confidence": 0.85,
                "evidence": eid_states,
                "uncertainty_reason": "State is heuristic — distance since last revisit + drift proxy, not true PGO chi².",
                "recommended_action": (
                    "If 'loop_needed' or 'drift_risk' was observed, continue running loop_close role allocation."
                    if states_seen & {"loop_needed", "drift_risk"} else "Maintain current allocation."
                ),
            })
    if loop_cands is not None:
        n_cands = int(_coalesce(loop_cands.get("candidate_count"), len(loop_cands.get("candidates", []) or [])))
        eid = ev.add("loop_candidates.json", "candidate_count", n_cands, "metric")
        out.append({
            "claim": f"{n_cands} loop-closure candidate viewpoints were available at trial end.",
            "type": "loop_candidate_inventory",
            "confidence": 0.95,
            "evidence": [eid],
            "uncertainty_reason": "Candidates derived from trajectory crossings + revisit gap heuristic, not true scan-context overlap.",
            "recommended_action": (
                "If candidate_count is low and yaw drift is high, "
                "extend trial duration or relax loop_close hysteresis."
            ),
        })
    return out


def _claims_reconstruction(
    rq: dict[str, Any] | None,
    ev: EvidenceIndex,
) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    if rq is None:
        return out
    voxels = int(_coalesce(rq.get("voxel_count"), 0))
    points = int(_coalesce(rq.get("point_count"), 0))
    cands = rq.get("candidates", []) or []
    n_cand = len(cands)
    target = int(_coalesce(rq.get("density_target_pts"), 30))
    view_t = int(_coalesce(rq.get("view_diversity_target"), 3))
    eids_meta = [
        ev.add("reconstruction_quality.json", "voxel_count", voxels, "metric"),
        ev.add("reconstruction_quality.json", "point_count", points, "metric"),
        ev.add("reconstruction_quality.json", "density_target_pts", target, "param"),
        ev.add("reconstruction_quality.json", "view_diversity_target", view_t, "param"),
    ]
    out.append({
        "claim": (
            f"Geometry-first reconstruction map covers {voxels} voxels at "
            f"{rq.get('voxel_size_m', 0.30)} m resolution, "
            f"{points/1000:.0f}k accumulated points across "
            f"{int(_coalesce(rq.get('cloud_count'), 0))} scan integrations. "
            f"Quality target: ≥ {target} pts and ≥ {view_t} viewpoint bins per voxel."
        ),
        "type": "reconstruction_summary",
        "confidence": 0.95,
        "evidence": eids_meta,
        "uncertainty_reason": "2D voxelisation (XY only); ceiling/floor variation invisible at this layer.",
        "recommended_action": "Run reconstruction_quality_node at finer voxel size (0.10 m) for a Level-2 evaluation.",
    })
    if n_cand > 0:
        eid_cands = ev.add("reconstruction_quality.json", "candidates[]", n_cand, "list")
        # Identify the worst-quality candidate as a concrete example.
        worst = max(cands, key=lambda c: float(_coalesce(c.get("recon_gain"), 0.0)))
        eid_worst = ev.add(
            "reconstruction_quality.json",
            f"candidates[{cands.index(worst)}]",
            {k: worst.get(k) for k in ("x", "y", "recon_gain", "view_diversity", "voxel_count", "target_robot")},
            "candidate",
        )
        out.append({
            "claim": (
                f"{n_cand} under-reconstructed regions are flagged for revisit. "
                f"Worst candidate at ({worst.get('x', 0):.2f}, {worst.get('y', 0):.2f}) "
                f"with recon_gain={float(worst.get('recon_gain', 0)):.2f} "
                f"(views={worst.get('view_diversity', 0)}, points={worst.get('voxel_count', 0)}); "
                f"routed to {worst.get('target_robot', '?')}."
            ),
            "type": "reconstruction_low_quality_region",
            "confidence": 0.80,
            "evidence": [eid_cands, eid_worst],
            "uncertainty_reason": (
                "recon_gain uses a geometric-mean proxy (density × view diversity); "
                "it does not measure true surface completeness or rendered-view quality."
            ),
            "recommended_action": (
                f"Send {worst.get('target_robot', 'a robot')} to ({worst.get('x', 0):.2f}, {worst.get('y', 0):.2f}) "
                f"for a multi-view revisit before generating the final map deliverable."
            ),
        })
    return out


_ROLE_RX = re.compile(r"role=(?P<role>\w+)")


def _scrape_assignment_log(launch_log: Path, ev: EvidenceIndex) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    if not launch_log.exists():
        return out
    role_counts: dict[tuple[str, str], int] = {}
    try:
        with launch_log.open("r", encoding="utf-8", errors="replace") as fh:
            for line in fh:
                if "cfpa2_coordinator" not in line or "role=" not in line:
                    continue
                m = _ROLE_RX.search(line)
                if not m:
                    continue
                role = m.group("role")
                ns = "robot_a" if "robot_a:" in line else ("robot_b" if "robot_b:" in line else "?")
                role_counts[(ns, role)] = role_counts.get((ns, role), 0) + 1
    except OSError:
        return out
    if not role_counts:
        return out
    # JSON-friendly version: tuple keys → "<ns>:<role>".
    role_counts_serializable = {f"{ns}:{role}": cnt for (ns, role), cnt in role_counts.items()}
    parts = []
    eid_log = ev.add(
        "launch.log",
        "cfpa2_coordinator_node ASSIGN events",
        role_counts_serializable,
        "log_aggregate",
    )
    for (ns, role), count in sorted(role_counts.items()):
        parts.append(f"{ns} {role}={count}")
    out.append({
        "claim": "CFPA2 role assignment distribution: " + "; ".join(parts) + ".",
        "type": "role_assignment_distribution",
        "confidence": 0.90,
        "evidence": [eid_log],
        "uncertainty_reason": (
            "Counts derived from grep over CFPA2 ASSIGN log lines; reflects per-tick decision frequency, not goal-completion frequency."
        ),
        "recommended_action": (
            "If reconstruct count is zero but reconstruction_quality_node published candidates, "
            "lower role_recon_min_coverage_ratio or raise role_w_recon."
        ),
    })
    return out


def _claims_failures(
    sess_a: dict[str, Any] | None,
    sess_b: dict[str, Any] | None,
    coll: dict[str, Any] | None,
    ev: EvidenceIndex,
) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for ns, sess in (("robot_a", sess_a), ("robot_b", sess_b)):
        if sess is None:
            continue
        tip = bool(sess.get("progress", {}).get("tipped_over"))
        peak_tilt = float(_coalesce(sess.get("progress", {}).get("peak_tilt_deg"), 0.0))
        if tip:
            eids = [
                ev.add(f"session/{ns}.json", "progress.tipped_over", tip, "flag"),
                ev.add(f"session/{ns}.json", "progress.peak_tilt_deg", peak_tilt, "metric"),
            ]
            out.append({
                "claim": f"Robot {ns} tipped over (peak tilt {peak_tilt:.1f}°) during the trial.",
                "type": "robot_failure_event",
                "confidence": 0.99,
                "evidence": eids,
                "uncertainty_reason": "Tip flag from session_reporter is a hard threshold; momentary near-tip events not flagged.",
                "recommended_action": "Reduce cmd_vel acceleration cap or stop loop_close in tight corner geometry.",
            })
    if coll is not None:
        for ns in ("robot_a", "robot_b"):
            data = coll.get("robots", {}).get(ns, {})
            tipped = bool(data.get("tipped_over", {}).get("tripped", False))
            wall = data.get("wall_contacts", {}).get("count", 0) or 0
            if not tipped and wall > 30:
                eids = [
                    ev.add("collision.json", f"robots.{ns}.wall_contacts.count", wall, "metric"),
                ]
                out.append({
                    "claim": f"{ns} accumulated {wall} wall contacts without tipping; corner-stamping pathology likely.",
                    "type": "near_failure_pattern",
                    "confidence": 0.75,
                    "evidence": eids,
                    "uncertainty_reason": (
                        "Without per-event location clustering this could be one prolonged scuff or many distinct corner events."
                    ),
                    "recommended_action": (
                        "Run trial with reconstruction_quality_enabled and inspect "
                        "low-density voxels — they often co-locate with corner stamping zones."
                    ),
                })
    return out


def _claims_unexplored(
    sess_a: dict[str, Any] | None,
    rq: dict[str, Any] | None,
    ev: EvidenceIndex,
) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    if sess_a is None:
        return out
    cov = float(_coalesce(sess_a.get("coverage", {}).get("coverage_ratio_of_scene"), 0.0))
    area = float(_coalesce(sess_a.get("coverage", {}).get("scene_area_m2"), 0.0))
    if cov >= 0.95 or area <= 1.0:
        return out
    unex_m2 = (1.0 - cov) * area
    eids = [
        ev.add("session/robot_a.json", "coverage.coverage_ratio_of_scene", cov, "metric"),
        ev.add("session/robot_a.json", "coverage.scene_area_m2", area, "metric"),
    ]
    out.append({
        "claim": (
            f"Approximately {unex_m2:.1f} m² of the {area:.0f} m² scene "
            f"({(1.0-cov)*100:.1f} %) remains unexplored at trial end."
        ),
        "type": "unexplored_region",
        "confidence": 0.85,
        "evidence": eids,
        "uncertainty_reason": (
            "Coverage gauges /merged_map known area; small unmapped fragments behind walls "
            "may register as 'unknown' even when geometrically inaccessible."
        ),
        "recommended_action": (
            "Extend trial duration or schedule a fresh coverage_only_mppi run; "
            "if loop_close is dominating ASSIGN, lower role_w_loop."
        ),
    })
    return out


# ──────────────────────────────────────────────────────────────────────
# Verifier (proposal §12.4 lite). Walks the claims list; any claim
# missing an evidence entry, or referencing an unknown evidence_id,
# gets `verified=false` so downstream consumers can filter.
# ──────────────────────────────────────────────────────────────────────


def _verify_claims(claims: list[dict[str, Any]], evidence: dict[str, Any]) -> list[dict[str, Any]]:
    valid_ids = set(evidence.keys())
    out: list[dict[str, Any]] = []
    for c in claims:
        eids = list(c.get("evidence", []))
        unknown = [e for e in eids if e not in valid_ids]
        verified = bool(eids) and not unknown
        c2 = dict(c)
        c2["verified"] = verified
        c2["unverified_reason"] = (
            None
            if verified
            else (
                "no evidence ids attached" if not eids
                else f"evidence ids not in evidence_index: {unknown}"
            )
        )
        out.append(c2)
    return out


# ──────────────────────────────────────────────────────────────────────
# Markdown rendering. Stays close to proposal §12.2 section ordering.
# Claims that share a `type` get grouped under their section heading.
# ──────────────────────────────────────────────────────────────────────


_SECTION_ORDER: list[tuple[str, set[str]]] = [
    ("Executive Summary", {"coverage_summary", "safety_summary"}),
    ("Localization Quality", {"localization_quality", "pose_graph_health_state", "loop_candidate_inventory"}),
    ("Reconstruction Quality", {"reconstruction_summary", "reconstruction_low_quality_region"}),
    ("Hazards and Robot Failure Events", {"robot_failure_event", "near_failure_pattern"}),
    ("Unexplored or Uncertain Regions", {"unexplored_region"}),
    ("Allocation Distribution", {"role_assignment_distribution"}),
]


def _render_markdown(payload: dict[str, Any]) -> str:
    claims = payload["claims"]
    by_type: dict[str, list[dict[str, Any]]] = {}
    for c in claims:
        by_type.setdefault(c["type"], []).append(c)
    lines: list[str] = []
    lines.append(f"# Mission Summary — {payload['trial_dir']}")
    lines.append("")
    lines.append(
        f"_Generated by `mission_summary_generator.py` from {payload['source_file_count']} evidence files; "
        f"{payload['claim_count']} claims, {payload['evidence_count']} evidence ids._"
    )
    lines.append("")
    rendered_types: set[str] = set()
    for section_title, types in _SECTION_ORDER:
        section_claims: list[dict[str, Any]] = []
        for t in types:
            section_claims.extend(by_type.get(t, []))
        if not section_claims:
            continue
        lines.append(f"## {section_title}")
        lines.append("")
        for c in section_claims:
            badge = "✓" if c.get("verified") else "⚠ unverified"
            lines.append(f"- **{c['claim']}**  ")
            lines.append(
                f"  _confidence={c['confidence']:.2f} · {badge} · evidence={c['evidence']}_"
            )
            if c.get("uncertainty_reason"):
                lines.append(f"  - _uncertainty:_ {c['uncertainty_reason']}")
            if c.get("recommended_action"):
                lines.append(f"  - _next:_ {c['recommended_action']}")
            lines.append("")
            rendered_types.add(c["type"])
    leftover = [c for c in claims if c["type"] not in rendered_types]
    if leftover:
        lines.append("## Other")
        lines.append("")
        for c in leftover:
            lines.append(f"- {c['claim']} · evidence={c['evidence']}")
        lines.append("")
    return "\n".join(lines)


# ──────────────────────────────────────────────────────────────────────
# Driver.
# ──────────────────────────────────────────────────────────────────────


def generate(trial_dir: Path) -> dict[str, Any]:
    sess_a = _load_json(trial_dir / "session" / "robot_a.json")
    sess_b = _load_json(trial_dir / "session" / "robot_b.json")
    coll = _load_json(trial_dir / "collision.json")
    pgh = _load_json(trial_dir / "pose_graph_health.json")
    loop_cands = _load_json(trial_dir / "loop_candidates.json")
    rq = _load_json(trial_dir / "reconstruction_quality.json")

    ev = EvidenceIndex(trial_dir)
    claims: list[dict[str, Any]] = []
    claims.extend(_claims_executive(sess_a, sess_b, coll, ev))
    claims.extend(_claims_localization(sess_a, sess_b, pgh, loop_cands, ev))
    claims.extend(_claims_reconstruction(rq, ev))
    claims.extend(_claims_failures(sess_a, sess_b, coll, ev))
    claims.extend(_claims_unexplored(sess_a, rq, ev))
    claims.extend(_scrape_assignment_log(trial_dir / "launch.log", ev))

    evidence_payload = ev.to_json()
    claims = _verify_claims(claims, evidence_payload["evidence"])
    sources = [
        p for p in [
            trial_dir / "session" / "robot_a.json",
            trial_dir / "session" / "robot_b.json",
            trial_dir / "collision.json",
            trial_dir / "pose_graph_health.json",
            trial_dir / "loop_candidates.json",
            trial_dir / "morphology_risk.json",
            trial_dir / "reconstruction_quality.json",
            trial_dir / "launch.log",
        ] if p.exists()
    ]
    summary = {
        "trial_dir": str(trial_dir),
        "claim_count": len(claims),
        "evidence_count": evidence_payload["evidence_count"],
        "source_file_count": len(sources),
        "verified_count": sum(1 for c in claims if c.get("verified")),
        "claims": claims,
    }

    out_md = trial_dir / "mission_summary.md"
    out_json = trial_dir / "mission_summary.json"
    out_evidence = trial_dir / "evidence_index.json"
    out_md.write_text(_render_markdown(summary), encoding="utf-8")
    out_json.write_text(json.dumps(summary, indent=2, sort_keys=False), encoding="utf-8")
    out_evidence.write_text(
        json.dumps(evidence_payload, indent=2, sort_keys=False), encoding="utf-8"
    )
    return summary


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: mission_summary_generator.py <trial_dir>", file=sys.stderr)
        return 2
    trial_dir = Path(sys.argv[1]).resolve()
    if not trial_dir.is_dir():
        print(f"not a directory: {trial_dir}", file=sys.stderr)
        return 2
    summary = generate(trial_dir)
    print(
        f"mission_summary: {trial_dir}/mission_summary.md "
        f"({summary['claim_count']} claims, {summary['verified_count']} verified, "
        f"{summary['evidence_count']} evidence ids)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
