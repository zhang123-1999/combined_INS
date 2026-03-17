from __future__ import annotations

import argparse
import copy
import datetime as dt
import itertools
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.generate_dataset_report_cases import compute_case_metrics, run_process, solver_cmd
from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml

MULTIPLIER_GRID = [0.25, 0.5, 1.0, 2.0, 4.0]
REFERENCE_CHECK_KEYS = ["rmse_3d_m", "p95_3d_m", "final_err_3d_m", "tail70_rmse_3d_m"]
REFERENCE_CASE_IDS = [
    "data2_gnss30_eskf_best",
    "data2_gnss30_true_iekf_best",
    "data4_gnss30_eskf_best",
    "data4_gnss30_true_iekf_best",
]

FAMILY_KEYS: dict[str, list[tuple[str, ...]]] = {
    "Q_imu": [
        ("fusion", "noise", "sigma_acc"),
        ("fusion", "noise", "sigma_gyro"),
    ],
    "Q_bias": [
        ("fusion", "noise", "sigma_ba"),
        ("fusion", "noise", "sigma_bg"),
    ],
    "Q_calib": [
        ("fusion", "noise", "sigma_sg"),
        ("fusion", "noise", "sigma_sa"),
        ("fusion", "noise", "sigma_odo_scale"),
        ("fusion", "noise", "sigma_mounting"),
        ("fusion", "noise", "sigma_mounting_roll"),
        ("fusion", "noise", "sigma_mounting_pitch"),
        ("fusion", "noise", "sigma_mounting_yaw"),
        ("fusion", "noise", "sigma_lever_arm"),
        ("fusion", "noise", "sigma_gnss_lever_arm"),
    ],
    "R_gnss_pos": [
        ("fusion", "noise", "sigma_gnss_pos"),
    ],
    "R_odo": [
        ("fusion", "constraints", "sigma_odo"),
    ],
    "R_nhc": [
        ("fusion", "constraints", "sigma_nhc_y"),
        ("fusion", "constraints", "sigma_nhc_z"),
    ],
}

ALLOWED_MUTATION_PATHS = {("fusion", "output_path")}
for path_list in FAMILY_KEYS.values():
    ALLOWED_MUTATION_PATHS.update(path_list)


@dataclass(frozen=True)
class CaseTarget:
    case_id: str
    dataset_id: str
    method: str
    label: str
    official_config_path: Path
    official_row: dict[str, Any]


def get_in(mapping: dict[str, Any], path: tuple[str, ...]) -> Any:
    cur: Any = mapping
    for key in path:
        cur = cur[key]
    return cur


def set_in(mapping: dict[str, Any], path: tuple[str, ...], value: Any) -> None:
    cur: Any = mapping
    for key in path[:-1]:
        cur = cur[key]
    cur[path[-1]] = value


def multiplier_label(value: float) -> str:
    text = f"{value:.6g}"
    return text.replace("-", "m").replace(".", "p")


def collect_leaf_paths(value: Any, prefix: tuple[str, ...] = ()) -> dict[tuple[str, ...], Any]:
    out: dict[tuple[str, ...], Any] = {}
    if isinstance(value, dict):
        for key, child in value.items():
            out.update(collect_leaf_paths(child, prefix + (str(key),)))
        return out
    if isinstance(value, list):
        for idx, child in enumerate(value):
            out.update(collect_leaf_paths(child, prefix + (str(idx),)))
        return out
    out[prefix] = value
    return out


def normalize_for_lock_check(cfg: dict[str, Any], base_cfg: dict[str, Any]) -> dict[str, Any]:
    normalized = copy.deepcopy(cfg)
    for path in ALLOWED_MUTATION_PATHS:
        set_in(normalized, path, get_in(base_cfg, path))
    return normalized


def validate_locked_sections(base_cfg: dict[str, Any], mutated_cfg: dict[str, Any]) -> None:
    normalized = normalize_for_lock_check(mutated_cfg, base_cfg)
    if normalized != base_cfg:
        base_leaves = collect_leaf_paths(base_cfg)
        new_leaves = collect_leaf_paths(normalized)
        changed: list[str] = []
        for path in sorted(set(base_leaves) | set(new_leaves)):
            if base_leaves.get(path) != new_leaves.get(path):
                changed.append(".".join(path))
        raise RuntimeError(f"unexpected config mutation outside allowed Q/R set: {changed}")


def write_csv_flexible(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    import csv

    with path.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def noise_signature(cfg: dict[str, Any]) -> tuple[tuple[str, float], ...]:
    signature: list[tuple[str, float]] = []
    for family_paths in FAMILY_KEYS.values():
        for path in family_paths:
            signature.append((".".join(path), float(get_in(cfg, path))))
    signature.sort()
    return tuple(signature)


def family_neighbors(multiplier: float) -> list[float]:
    if multiplier not in MULTIPLIER_GRID:
        raise RuntimeError(f"multiplier {multiplier} not in grid")
    idx = MULTIPLIER_GRID.index(multiplier)
    lo = max(0, idx - 1)
    hi = min(len(MULTIPLIER_GRID) - 1, idx + 1)
    return MULTIPLIER_GRID[lo : hi + 1]


def gnss_vel_audit(gnss_path: Path) -> dict[str, Any]:
    raw = pd.read_csv(gnss_path, sep=r"\s+", comment="#")
    cols = [str(c) for c in raw.columns]
    if "timestamp" not in cols:
        fallback = pd.read_csv(gnss_path, sep=r"\s+", header=None, comment="#")
        if fallback.shape[1] == 7:
            fallback.columns = ["timestamp", "lat", "lon", "h", "sigma_n", "sigma_e", "sigma_d"]
        elif fallback.shape[1] == 13:
            fallback.columns = [
                "timestamp",
                "lat",
                "lon",
                "h",
                "sigma_n",
                "sigma_e",
                "sigma_d",
                "vn",
                "ve",
                "vd",
                "sigma_vn",
                "sigma_ve",
                "sigma_vd",
            ]
        raw = fallback
        cols = [str(c) for c in raw.columns]
    out: dict[str, Any] = {
        "gnss_path": rel_from_root(gnss_path, REPO_ROOT),
        "rows": int(raw.shape[0]),
        "columns": cols,
        "has_velocity_columns": False,
        "gnss_vel_update_possible": False,
        "sigma_vel_fallback_value_mps": 0.5,
        "sigma_vel_floor_mps": 1e-4,
    }
    required = {"vn", "ve", "vd", "sigma_vn", "sigma_ve", "sigma_vd"}
    if not required.issubset(cols):
        out["reason"] = "GNSS file has no velocity/std columns; GNSS_VEL update is not executed."
        return out

    out["has_velocity_columns"] = True
    out["gnss_vel_update_possible"] = True
    for axis in ("vn", "ve", "vd"):
        vals = raw[axis].to_numpy(dtype=float)
        out[f"{axis}_mean_mps"] = float(np.mean(vals))
        out[f"{axis}_std_mps"] = float(np.std(vals))
        out[f"{axis}_abs_p95_mps"] = float(np.percentile(np.abs(vals), 95))

    for axis in ("sigma_vn", "sigma_ve", "sigma_vd"):
        vals = raw[axis].to_numpy(dtype=float)
        finite = np.isfinite(vals)
        positive = finite & (vals > 0.0)
        out[f"{axis}_mean_mps"] = float(np.mean(vals[positive])) if np.any(positive) else float("nan")
        out[f"{axis}_median_mps"] = float(np.median(vals[positive])) if np.any(positive) else float("nan")
        out[f"{axis}_p95_mps"] = float(np.percentile(vals[positive], 95)) if np.any(positive) else float("nan")
        out[f"{axis}_invalid_or_nonpositive_count"] = int(np.count_nonzero(~positive))
        out[f"{axis}_floor_clamp_count"] = int(np.count_nonzero(positive & (vals < 1e-4)))
    return out


def is_eligible(row: dict[str, Any]) -> bool:
    if int(row.get("return_code", 1)) != 0:
        return False
    if not bool(row.get("sol_exists", False)) or not bool(row.get("log_exists", False)):
        return False
    if float(row.get("odo_accept_ratio", float("nan"))) < 0.95:
        return False
    if float(row.get("nhc_accept_ratio", float("nan"))) < 0.95:
        return False
    if float(row.get("odo_reject_numeric", float("nan"))) != 0.0:
        return False
    if float(row.get("nhc_reject_numeric", float("nan"))) != 0.0:
        return False
    return True


def better_candidate(a: dict[str, Any], b: dict[str, Any] | None) -> bool:
    if b is None:
        return True
    if not a["eligible"] and b["eligible"]:
        return False
    if a["eligible"] and not b["eligible"]:
        return True
    a_rmse = float(a["rmse_3d_m"])
    b_rmse = float(b["rmse_3d_m"])
    if min(a_rmse, b_rmse) <= 0.0:
        rmse_close = abs(a_rmse - b_rmse) <= 1e-12
    else:
        rmse_close = abs(a_rmse - b_rmse) / min(a_rmse, b_rmse) < 0.01
    if not rmse_close:
        return a_rmse < b_rmse
    a_p95 = float(a["p95_3d_m"])
    b_p95 = float(b["p95_3d_m"])
    if a_p95 != b_p95:
        return a_p95 < b_p95
    return float(a["final_err_3d_m"]) < float(b["final_err_3d_m"])


def best_row(rows: list[dict[str, Any]]) -> dict[str, Any]:
    best: dict[str, Any] | None = None
    for row in rows:
        if better_candidate(row, best):
            best = row
    if best is None:
        raise RuntimeError("best_row called with empty rows")
    return best


def select_family_best(rows: list[dict[str, Any]], family: str) -> dict[str, Any] | None:
    candidates = [row for row in rows if row["search_stage"] == "rough" and row["family"] == family and row["eligible"]]
    if not candidates:
        return None
    return best_row(candidates)


def row_sort_key(row: dict[str, Any]) -> tuple[Any, ...]:
    return (
        0 if row["eligible"] else 1,
        float(row["rmse_3d_m"]),
        float(row["p95_3d_m"]),
        float(row["final_err_3d_m"]),
        row["case_name"],
    )


def reference_status(row: dict[str, Any], official_row: dict[str, Any], atol: float) -> dict[str, Any]:
    deltas: dict[str, float] = {}
    mismatches: dict[str, dict[str, float]] = {}
    ok = True
    for key in REFERENCE_CHECK_KEYS:
        got = float(row[key])
        ref = float(official_row[key])
        delta = got - ref
        deltas[key] = delta
        if abs(delta) > atol:
            ok = False
            mismatches[key] = {"expected": ref, "got": got, "delta": delta}
    return {
        "passed": ok,
        "atol": atol,
        "deltas": deltas,
        "mismatches": mismatches,
    }


class CaseRunner:
    def __init__(self, target: CaseTarget, outdir: Path, exe: Path, reference_tol: float) -> None:
        self.target = target
        self.outdir = outdir
        self.case_dir = outdir / target.case_id
        self.exe = exe
        self.reference_tol = reference_tol
        self.base_cfg = load_yaml(target.official_config_path)
        self.reference_row: dict[str, Any] | None = None
        self.rows: list[dict[str, Any]] = []
        self.by_signature: dict[tuple[tuple[str, float], ...], dict[str, Any]] = {}
        ensure_dir(self.case_dir)

    def _build_cfg(self, scales: dict[str, float], sol_path: Path) -> dict[str, Any]:
        cfg = copy.deepcopy(self.base_cfg)
        fusion = cfg.setdefault("fusion", {})
        fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
        for family, multiplier in scales.items():
            for path in FAMILY_KEYS[family]:
                base_value = float(get_in(self.base_cfg, path))
                set_in(cfg, path, base_value * multiplier)
        validate_locked_sections(self.base_cfg, cfg)
        return cfg

    def run_candidate(
        self,
        *,
        search_stage: str,
        family: str,
        multiplier: float,
        family_b: str = "",
        multiplier_b: float = 1.0,
    ) -> dict[str, Any]:
        scales: dict[str, float] = {}
        if family and family != "reference" and multiplier != 1.0:
            scales[family] = multiplier
        if family_b and multiplier_b != 1.0:
            scales[family_b] = multiplier_b

        candidate_suffix = family or "reference"
        if family == "reference":
            case_name = "reference"
        elif family_b:
            case_name = (
                f"{search_stage}_{family}_{multiplier_label(multiplier)}__"
                f"{family_b}_{multiplier_label(multiplier_b)}"
            )
        else:
            case_name = f"{search_stage}_{family}_{multiplier_label(multiplier)}"

        sol_path = self.case_dir / f"SOL_{self.target.case_id}_{case_name}.txt"
        cfg = self._build_cfg(scales, sol_path)
        signature = noise_signature(cfg)
        if signature in self.by_signature:
            return self.by_signature[signature]

        cfg_path = self.case_dir / f"cfg_{self.target.case_id}_{case_name}.yaml"
        log_path = self.case_dir / f"{self.target.case_id}_{case_name}.log"

        return_code = 0
        merged = ""
        reused_existing = False
        if cfg_path.exists() and log_path.exists():
            try:
                existing_cfg = load_yaml(cfg_path)
            except Exception:
                existing_cfg = None
            if existing_cfg == cfg:
                merged = log_path.read_text(encoding="utf-8", errors="ignore")
                if sol_path.exists():
                    reused_existing = True
                elif "command failed" in merged or "Traceback" in merged:
                    reused_existing = True
                    return_code = 1
        if not reused_existing:
            save_yaml(cfg, cfg_path)
            try:
                merged = run_process(solver_cmd(self.exe, cfg_path), cwd=REPO_ROOT)
            except RuntimeError as exc:
                return_code = 1
                merged = str(exc)
            log_path.write_text(merged, encoding="utf-8")

        metrics: dict[str, Any]
        if return_code == 0 and sol_path.exists():
            truth_path = (REPO_ROOT / str(cfg["fusion"]["pos_path"])).resolve()
            metrics = compute_case_metrics(sol_path, truth_path, merged)
        else:
            metrics = {
                "rmse_x_m": float("nan"),
                "rmse_y_m": float("nan"),
                "rmse_z_m": float("nan"),
                "rmse_3d_m": float("inf"),
                "p95_3d_m": float("inf"),
                "tail70_rmse_3d_m": float("inf"),
                "final_err_3d_m": float("inf"),
                "samples": 0,
                "gnss_stop_s": float("nan"),
                "nav_duration_s": 0.0,
                "odo_seen": float("nan"),
                "odo_accepted": float("nan"),
                "odo_accept_ratio": float("nan"),
                "odo_reject_nis": float("nan"),
                "odo_reject_numeric": float("nan"),
                "odo_nis_mean": float("nan"),
                "odo_nis_max": float("nan"),
                "odo_accepted_hz": float("nan"),
                "nhc_seen": float("nan"),
                "nhc_accepted": float("nan"),
                "nhc_accept_ratio": float("nan"),
                "nhc_reject_nis": float("nan"),
                "nhc_reject_numeric": float("nan"),
                "nhc_nis_mean": float("nan"),
                "nhc_nis_max": float("nan"),
                "nhc_accepted_hz": float("nan"),
            }

        row: dict[str, Any] = {
            "case_id": self.target.case_id,
            "dataset_id": self.target.dataset_id,
            "method": self.target.method,
            "label": self.target.label,
            "search_stage": search_stage,
            "family": family,
            "multiplier": float(multiplier),
            "family_b": family_b,
            "multiplier_b": float(multiplier_b),
            "case_name": case_name,
            "base_config_path": rel_from_root(self.target.official_config_path, REPO_ROOT),
            "config_path": rel_from_root(cfg_path, REPO_ROOT),
            "sol_path": rel_from_root(sol_path, REPO_ROOT),
            "log_path": rel_from_root(log_path, REPO_ROOT),
            "return_code": return_code,
            "sol_exists": sol_path.exists(),
            "log_exists": log_path.exists(),
            "artifact_mtime_cfg": dt.datetime.fromtimestamp(cfg_path.stat().st_mtime).isoformat(timespec="seconds"),
            "artifact_mtime_log": dt.datetime.fromtimestamp(log_path.stat().st_mtime).isoformat(timespec="seconds"),
            "applied_scales": json.dumps(scales, ensure_ascii=False, sort_keys=True),
            "reused_existing": reused_existing,
        }
        if sol_path.exists():
            row["artifact_mtime_sol"] = dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds")
        else:
            row["artifact_mtime_sol"] = ""
        row.update(metrics)

        if self.reference_row is None and family == "reference":
            row["delta_rmse_pct_vs_ref"] = 0.0
            row["delta_p95_pct_vs_ref"] = 0.0
            row["delta_final_pct_vs_ref"] = 0.0
        elif self.reference_row is not None:
            ref_rmse = float(self.reference_row["rmse_3d_m"])
            ref_p95 = float(self.reference_row["p95_3d_m"])
            ref_final = float(self.reference_row["final_err_3d_m"])
            row["delta_rmse_pct_vs_ref"] = 100.0 * (float(row["rmse_3d_m"]) - ref_rmse) / ref_rmse
            row["delta_p95_pct_vs_ref"] = 100.0 * (float(row["p95_3d_m"]) - ref_p95) / ref_p95
            row["delta_final_pct_vs_ref"] = 100.0 * (float(row["final_err_3d_m"]) - ref_final) / ref_final
        else:
            row["delta_rmse_pct_vs_ref"] = float("nan")
            row["delta_p95_pct_vs_ref"] = float("nan")
            row["delta_final_pct_vs_ref"] = float("nan")
        row["eligible"] = is_eligible(row)

        self.rows.append(row)
        self.by_signature[signature] = row
        return row

    def run_reference(self) -> dict[str, Any]:
        ref = self.run_candidate(search_stage="reference", family="reference", multiplier=1.0)
        self.reference_row = ref
        ref_check = reference_status(ref, self.target.official_row, self.reference_tol)
        ref["reference_match"] = ref_check["passed"]
        ref["reference_check_json"] = json.dumps(ref_check, ensure_ascii=False, sort_keys=True)
        return ref


def load_targets(manifest_path: Path) -> tuple[dict[str, Any], list[CaseTarget]]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    case_map = {case["case_id"]: case for case in manifest.get("cases", [])}
    targets: list[CaseTarget] = []
    for case_id in REFERENCE_CASE_IDS:
        row = case_map.get(case_id)
        if row is None:
            raise RuntimeError(f"missing case in manifest: {case_id}")
        targets.append(
            CaseTarget(
                case_id=case_id,
                dataset_id=str(row["dataset_id"]),
                method=str(row["method"]),
                label=str(row["label"]),
                official_config_path=(REPO_ROOT / str(row["config_path"])).resolve(),
                official_row=row,
            )
        )
    return manifest, targets


def assign_case_ranks(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    ranked: list[dict[str, Any]] = []
    by_case: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        by_case.setdefault(str(row["case_id"]), []).append(row)
    for case_id, case_rows in by_case.items():
        ordered = sorted(case_rows, key=row_sort_key)
        for idx, row in enumerate(ordered, start=1):
            new_row = dict(row)
            new_row["case_rank"] = idx
            ranked.append(new_row)
    ranked.sort(key=lambda row: (row["case_id"], row["case_rank"]))
    return ranked


def write_case_outputs(case_runner: CaseRunner, case_rows: list[dict[str, Any]], best: dict[str, Any], judgment: str) -> None:
    reference = case_runner.reference_row
    if reference is None:
        raise RuntimeError("reference row missing when writing case outputs")
    case_dir = case_runner.case_dir
    best_cfg = load_yaml(REPO_ROOT / str(best["config_path"]))
    best_cfg_path = case_dir / "best_config.yaml"
    save_yaml(best_cfg, best_cfg_path)

    top5 = sorted(case_rows, key=row_sort_key)[:5]
    summary_lines = [
        f"# {case_runner.target.case_id} noise sweep summary",
        "",
        f"- judgment: {judgment}",
        f"- reference RMSE3D={reference['rmse_3d_m']:.6f} m, P95={reference['p95_3d_m']:.6f} m, final={reference['final_err_3d_m']:.6f} m",
        f"- best case: `{best['case_name']}`",
        f"- best family/multiplier: `{best['family']}` x `{best['multiplier']}`",
    ]
    if best.get("family_b"):
        summary_lines.append(
            f"- refine pair: `{best['family']}` x `{best['multiplier']}` + `{best['family_b']}` x `{best['multiplier_b']}`"
        )
    summary_lines.extend(
        [
            f"- best RMSE3D={best['rmse_3d_m']:.6f} m ({best['delta_rmse_pct_vs_ref']:+.3f}% vs ref)",
            f"- best P95={best['p95_3d_m']:.6f} m ({best['delta_p95_pct_vs_ref']:+.3f}% vs ref)",
            f"- best final={best['final_err_3d_m']:.6f} m ({best['delta_final_pct_vs_ref']:+.3f}% vs ref)",
            "",
            "## top5",
        ]
    )
    for row in top5:
        summary_lines.append(
            f"- {row['case_rank']}. `{row['case_name']}` eligible={row['eligible']} "
            f"RMSE3D={row['rmse_3d_m']:.6f} m, P95={row['p95_3d_m']:.6f} m, final={row['final_err_3d_m']:.6f} m"
        )
    (case_dir / "best_summary.md").write_text("\n".join(summary_lines), encoding="utf-8")

    best_vs_ref = {
        "case_id": case_runner.target.case_id,
        "judgment": judgment,
        "reference": {
            key: float(reference[key]) for key in ("rmse_3d_m", "p95_3d_m", "tail70_rmse_3d_m", "final_err_3d_m")
        },
        "best": {
            "case_name": best["case_name"],
            "family": best["family"],
            "multiplier": float(best["multiplier"]),
            "family_b": best["family_b"],
            "multiplier_b": float(best["multiplier_b"]),
            "config_path": best["config_path"],
            "rmse_3d_m": float(best["rmse_3d_m"]),
            "p95_3d_m": float(best["p95_3d_m"]),
            "tail70_rmse_3d_m": float(best["tail70_rmse_3d_m"]),
            "final_err_3d_m": float(best["final_err_3d_m"]),
            "eligible": bool(best["eligible"]),
        },
        "delta_vs_ref": {
            "rmse_pct": float(best["delta_rmse_pct_vs_ref"]),
            "p95_pct": float(best["delta_p95_pct_vs_ref"]),
            "final_pct": float(best["delta_final_pct_vs_ref"]),
        },
    }
    (case_dir / "best_vs_ref.json").write_text(json.dumps(best_vs_ref, indent=2, ensure_ascii=False), encoding="utf-8")


def write_top_level_summary(
    outdir: Path,
    results: list[dict[str, Any]],
    case_summaries: list[dict[str, Any]],
    reference_failures: list[dict[str, Any]],
    gnss_audit_path: Path,
) -> None:
    lines = [
        "# GNSS30 Q/R noise sweep",
        "",
        f"- generated_at: {dt.datetime.now().isoformat(timespec='seconds')}",
        f"- gnss_vel_audit: `{rel_from_root(gnss_audit_path, REPO_ROOT)}`",
    ]
    if reference_failures:
        lines.extend(["- status: stale_or_conflict", "", "## reference mismatch"])
        for item in reference_failures:
            lines.append(f"- {item['case_id']}: {json.dumps(item['mismatches'], ensure_ascii=False)}")
    else:
        lines.extend(["- status: completed", "", "## case outcomes"])
        for item in case_summaries:
            lines.append(
                f"- {item['case_id']}: {item['judgment']}; best=`{item['best_case_name']}`; "
                f"family=`{item['best_family']}`; RMSE3D `{item['reference_rmse_3d_m']:.6f} -> {item['best_rmse_3d_m']:.6f}` m"
            )
    lines.append("")
    lines.append("## reference values")
    for row in sorted([r for r in results if r["search_stage"] == "reference"], key=lambda x: x["case_id"]):
        lines.append(
            f"- {row['case_id']}: RMSE3D={row['rmse_3d_m']:.6f} m, P95={row['p95_3d_m']:.6f} m, final={row['final_err_3d_m']:.6f} m"
        )
    (outdir / "summary.md").write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    today = dt.datetime.now().strftime("%Y%m%d")
    parser = argparse.ArgumentParser(description="Sweep explicit Q/R scalars for the four GNSS30 canonical cases.")
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("output/review/EXP-20260313-dataset-report-cases-r4/manifest.json"),
        help="Canonical case manifest used as official reference.",
    )
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path(f"output/review/EXP-{today}-noise-qr-gnss30-r1"),
        help="Output directory for the Q/R sweep.",
    )
    parser.add_argument(
        "--exe",
        type=Path,
        default=Path("build/Release/eskf_fusion.exe"),
        help="Solver executable relative to repo root.",
    )
    parser.add_argument(
        "--reference-tol",
        type=float,
        default=1e-4,
        help="Absolute tolerance for reference reproduction checks.",
    )
    args = parser.parse_args()
    args.manifest = (REPO_ROOT / args.manifest).resolve()
    args.outdir = (REPO_ROOT / args.outdir).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    return args


def main() -> int:
    args = parse_args()
    ensure_dir(args.outdir)
    if not args.manifest.exists():
        raise FileNotFoundError(f"missing manifest: {args.manifest}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing solver executable: {args.exe}")

    source_manifest, targets = load_targets(args.manifest)
    gnss_audits: dict[str, Any] = {}
    runners = [CaseRunner(target, args.outdir, args.exe, args.reference_tol) for target in targets]

    all_rows: list[dict[str, Any]] = []
    reference_failures: list[dict[str, Any]] = []

    for runner in runners:
        gnss_path = (REPO_ROOT / str(runner.base_cfg["fusion"]["gnss_path"])).resolve()
        gnss_audits[runner.target.case_id] = gnss_vel_audit(gnss_path)
        ref_row = runner.run_reference()
        all_rows.extend(runner.rows[-1:])
        ref_check = json.loads(ref_row["reference_check_json"])
        if not ref_check["passed"]:
            reference_failures.append(
                {
                    "case_id": runner.target.case_id,
                    "mismatches": ref_check["mismatches"],
                }
            )

    gnss_audit_path = args.outdir / "gnss_vel_audit.json"
    gnss_audit_path.write_text(json.dumps(gnss_audits, indent=2, ensure_ascii=False), encoding="utf-8")

    if reference_failures:
        ranked_rows = assign_case_ranks(all_rows)
        write_csv_flexible(args.outdir / "metrics.csv", ranked_rows)
        write_csv_flexible(args.outdir / "case_ranking.csv", ranked_rows)
        write_top_level_summary(args.outdir, ranked_rows, [], reference_failures, gnss_audit_path)
        manifest = {
            "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
            "base_manifest": rel_from_root(args.manifest, REPO_ROOT),
            "status": "stale_or_conflict",
            "reference_failures": reference_failures,
            "gnss_vel_audit_path": rel_from_root(gnss_audit_path, REPO_ROOT),
            "cases": [],
        }
        (args.outdir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
        print(args.outdir.resolve())
        return 2

    case_summaries: list[dict[str, Any]] = []
    for runner in runners:
        for family in FAMILY_KEYS:
            for multiplier in MULTIPLIER_GRID:
                row = runner.run_candidate(search_stage="rough", family=family, multiplier=multiplier)
                if row not in all_rows:
                    all_rows.append(row)

        family_best_rows: list[dict[str, Any]] = []
        for family in FAMILY_KEYS:
            best = select_family_best(runner.rows, family)
            if best is not None:
                family_best_rows.append(best)
        family_best_rows.sort(key=lambda row: (-1.0 * ((float(runner.reference_row["rmse_3d_m"]) - float(row["rmse_3d_m"])) / float(runner.reference_row["rmse_3d_m"])) if row["eligible"] else float("inf"), row_sort_key(row)))
        top2 = family_best_rows[:2]
        if len(top2) == 2:
            fam_a, fam_b = top2[0], top2[1]
            for mult_a, mult_b in itertools.product(
                family_neighbors(float(fam_a["multiplier"])),
                family_neighbors(float(fam_b["multiplier"])),
            ):
                row = runner.run_candidate(
                    search_stage="refine",
                    family=str(fam_a["family"]),
                    multiplier=float(mult_a),
                    family_b=str(fam_b["family"]),
                    multiplier_b=float(mult_b),
                )
                if row not in all_rows:
                    all_rows.append(row)

        case_rows = [row for row in runner.rows]
        best = best_row(case_rows)
        has_clear_room = any(
            row["case_name"] != "reference"
            and row["eligible"]
            and float(row["delta_rmse_pct_vs_ref"]) <= -5.0
            and float(row["delta_p95_pct_vs_ref"]) <= 5.0
            for row in case_rows
        )
        judgment = "当前 Q/R 有明确优化空间" if has_clear_room else "当前 Q/R 基本合理"
        ranked_case_rows = assign_case_ranks(case_rows)
        write_case_outputs(runner, ranked_case_rows, best, judgment)

        case_summaries.append(
            {
                "case_id": runner.target.case_id,
                "judgment": judgment,
                "best_case_name": str(best["case_name"]),
                "best_family": str(best["family"]),
                "best_multiplier": float(best["multiplier"]),
                "best_family_b": str(best["family_b"]),
                "best_multiplier_b": float(best["multiplier_b"]),
                "reference_rmse_3d_m": float(runner.reference_row["rmse_3d_m"]),
                "best_rmse_3d_m": float(best["rmse_3d_m"]),
                "reference_p95_3d_m": float(runner.reference_row["p95_3d_m"]),
                "best_p95_3d_m": float(best["p95_3d_m"]),
                "reference_final_err_3d_m": float(runner.reference_row["final_err_3d_m"]),
                "best_final_err_3d_m": float(best["final_err_3d_m"]),
                "best_config_path": rel_from_root(runner.case_dir / "best_config.yaml", REPO_ROOT),
                "best_summary_path": rel_from_root(runner.case_dir / "best_summary.md", REPO_ROOT),
                "best_vs_ref_path": rel_from_root(runner.case_dir / "best_vs_ref.json", REPO_ROOT),
            }
        )

    ranked_rows = assign_case_ranks(all_rows)
    write_csv_flexible(args.outdir / "metrics.csv", ranked_rows)
    write_csv_flexible(args.outdir / "case_ranking.csv", ranked_rows)
    write_top_level_summary(args.outdir, ranked_rows, case_summaries, [], gnss_audit_path)
    manifest = {
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_manifest": rel_from_root(args.manifest, REPO_ROOT),
        "status": "completed",
        "reference_tol": float(args.reference_tol),
        "reference_case_ids": REFERENCE_CASE_IDS,
        "multiplier_grid": MULTIPLIER_GRID,
        "families": {name: [".".join(path) for path in paths] for name, paths in FAMILY_KEYS.items()},
        "official_case_ids": REFERENCE_CASE_IDS,
        "base_manifest_generated_at": source_manifest.get("generated_at"),
        "gnss_vel_audit_path": rel_from_root(gnss_audit_path, REPO_ROOT),
        "cases": case_summaries,
    }
    (args.outdir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    print(args.outdir.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
