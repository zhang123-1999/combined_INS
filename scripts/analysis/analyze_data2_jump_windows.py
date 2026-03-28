from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import pandas as pd
import yaml


KEY_STATES = [
    "ba_x_mgal",
    "ba_y_mgal",
    "bg_z_degh",
    "sg_z_ppm",
    "sa_x_ppm",
    "sa_y_ppm",
    "sa_z_ppm",
    "odo_scale",
    "mounting_yaw_deg",
    "total_mounting_yaw_deg",
    "odo_lever_y_m",
    "gnss_lever_y_m",
]

EVENT_MARKERS = (
    "[GNSS_POS]",
    "[Warn] NHC skipped",
    "[Consistency] ODO reject",
    "[Diag] First divergence start",
    "[Diag] Div GNSS_POS update",
    "[Diag] Div NHC update",
    "[Diag] Div ODO update",
    "[Runtime] phases",
)

T_PATTERN = re.compile(r"t=([0-9]+\.[0-9]+)")
H_ATT_PATTERN = re.compile(r"\|\|H_att\|\|_F=([0-9]+\.[0-9]+)")


@dataclass
class JumpRecord:
    timestamp: float
    score: float
    phase_name: str
    near_gnss_boundary: str
    v_fwd: float | None
    omega_z: float | None
    odo_speed: float | None
    top_deltas: list[tuple[str, float]]
    event_lines: list[str]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit major state jump windows for a solver case.")
    parser.add_argument("--case-dir", type=Path, required=True, help="Case directory containing config/state_series/stdout.")
    parser.add_argument(
        "--output-md",
        type=Path,
        default=None,
        help="Optional markdown report path. Defaults to <case-dir>/jump_window_audit.md",
    )
    parser.add_argument("--top-k", type=int, default=6, help="Number of clustered jump windows to keep.")
    parser.add_argument("--cluster-gap", type=float, default=5.0, help="Cluster nearby jump seconds within this gap.")
    parser.add_argument(
        "--min-time",
        type=float,
        default=None,
        help="Optional lower time bound to ignore early initialization transients.",
    )
    return parser.parse_args()


def find_single(case_dir: Path, pattern: str) -> Path:
    matches = sorted(case_dir.glob(pattern))
    if len(matches) != 1:
        raise FileNotFoundError(f"Expected exactly one match for {pattern} under {case_dir}, got {len(matches)}")
    return matches[0]


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_state_series(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def sample_per_second(df: pd.DataFrame) -> pd.DataFrame:
    sampled = df.copy()
    sampled["sec"] = sampled["timestamp"].astype(int)
    idx = sampled.groupby("sec")["timestamp"].idxmin()
    sampled = sampled.loc[idx].sort_values("timestamp").reset_index(drop=True)
    return sampled


def robust_scales(diff_df: pd.DataFrame) -> pd.Series:
    scales = diff_df.median()
    p90 = diff_df.quantile(0.90)
    scales = scales.where(scales > 0.0, p90)
    scales = scales.where(scales > 0.0, 1.0)
    return scales


def cluster_candidate_times(times: Iterable[tuple[float, float]], gap: float) -> list[list[tuple[float, float]]]:
    ordered = sorted(times, key=lambda item: item[0])
    clusters: list[list[tuple[float, float]]] = []
    for t, score in ordered:
        if not clusters or t - clusters[-1][-1][0] > gap:
            clusters.append([(t, score)])
        else:
            clusters[-1].append((t, score))
    return clusters


def phase_name_for_time(t: float, phases: list[dict]) -> str:
    for phase in phases:
        start_time = float(phase.get("start_time", float("-inf")))
        end_time = float(phase.get("end_time", float("inf")))
        if start_time <= t <= end_time:
            return str(phase.get("name", "unnamed_phase"))
    return "no_runtime_phase"


def gnss_boundary_note(t: float, windows: list[tuple[float, float]], tol: float = 2.0) -> str:
    for start, end in windows:
        if abs(t - start) <= tol:
            return f"near_gnss_on_start({start:.3f})"
        if abs(t - end) <= tol:
            return f"near_gnss_on_end({end:.3f})"
    return "none"


def load_diag(path: Path) -> pd.DataFrame:
    diag = pd.read_csv(path, sep=r"\s+")
    for col in diag.columns:
        diag[col] = pd.to_numeric(diag[col], errors="coerce")
    return diag


def nearest_diag_row(diag_df: pd.DataFrame, t: float) -> pd.Series | None:
    if diag_df.empty:
        return None
    idx = (diag_df["t"] - t).abs().idxmin()
    return diag_df.loc[idx]


def extract_event_time(line: str) -> float | None:
    match = T_PATTERN.search(line)
    if match:
        return float(match.group(1))
    return None


def select_event_lines(stdout_path: Path, t: float, radius: float = 1.5) -> list[str]:
    lines: list[str] = []
    with stdout_path.open("r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            if not any(marker in raw for marker in EVENT_MARKERS):
                continue
            event_t = extract_event_time(raw)
            if event_t is None:
                continue
            if abs(event_t - t) <= radius:
                lines.append(raw.rstrip())
    return lines[:20]


def detect_major_jumps(
    sampled: pd.DataFrame,
    phases: list[dict],
    gnss_windows: list[tuple[float, float]],
    diag_df: pd.DataFrame,
    stdout_path: Path,
    start_time: float,
    top_k: int,
    cluster_gap: float,
    min_time: float | None,
) -> list[JumpRecord]:
    work = sampled.copy()
    if min_time is not None:
        work = work[work["timestamp"] >= min_time].reset_index(drop=True)

    diff_df = work[KEY_STATES].diff().abs()
    scales = robust_scales(diff_df.iloc[1:])
    norm_df = diff_df.divide(scales, axis=1).clip(upper=100.0)
    work["jump_score"] = norm_df.sum(axis=1)
    candidates = work.iloc[1:][["timestamp", "jump_score"]].sort_values("jump_score", ascending=False).head(top_k * 6)
    clusters = cluster_candidate_times(candidates.itertuples(index=False, name=None), cluster_gap)

    records: list[JumpRecord] = []
    for cluster in clusters:
        best_t, best_score = max(cluster, key=lambda item: item[1])
        row_idx = (work["timestamp"] - best_t).abs().idxmin()
        raw_delta = diff_df.loc[row_idx, KEY_STATES].sort_values(ascending=False).head(6)
        diag_row = nearest_diag_row(diag_df, best_t - start_time)
        records.append(
            JumpRecord(
                timestamp=float(best_t),
                score=float(best_score),
                phase_name=phase_name_for_time(best_t, phases),
                near_gnss_boundary=gnss_boundary_note(best_t, gnss_windows),
                v_fwd=None if diag_row is None else float(diag_row.get("v_fwd", math.nan)),
                omega_z=None if diag_row is None else float(diag_row.get("omega_z", math.nan)),
                odo_speed=None if diag_row is None else float(diag_row.get("odo_speed", math.nan)),
                top_deltas=[(str(k), float(v)) for k, v in raw_delta.items()],
                event_lines=select_event_lines(stdout_path, best_t),
            )
        )

    records.sort(key=lambda item: item.score, reverse=True)
    return records[:top_k]


def format_float(value: float | None, digits: int = 3) -> str:
    if value is None or not math.isfinite(value):
        return "NA"
    return f"{value:.{digits}f}"


def render_report(case_name: str, config: dict, records: list[JumpRecord]) -> str:
    fusion = config.get("fusion", {})
    windows = fusion.get("gnss_schedule", {}).get("enabled_windows", []) or []
    phases = fusion.get("runtime_phases", []) or []
    lines: list[str] = []
    lines.append(f"# Jump Window Audit: `{case_name}`")
    lines.append("")
    lines.append(f"- start/end: `{fusion.get('starttime')}` -> `{fusion.get('finaltime')}`")
    lines.append(f"- gnss_windows: `{len(windows)}`")
    lines.append(f"- runtime_phases: `{len(phases)}`")
    lines.append("")
    lines.append("## Major Jumps")
    for idx, record in enumerate(records, start=1):
        lines.append(f"### Jump {idx}")
        lines.append(
            f"- timestamp: `{record.timestamp:.6f}` | score=`{record.score:.2f}` | phase=`{record.phase_name}` | boundary=`{record.near_gnss_boundary}`"
        )
        lines.append(
            f"- diag_nearest: `v_fwd={format_float(record.v_fwd)}` `omega_z={format_float(record.omega_z)}` `odo_speed={format_float(record.odo_speed)}`"
        )
        lines.append(
            "- top_state_deltas: "
            + ", ".join(f"`{name}={value:.6f}`" for name, value in record.top_deltas)
        )
        if record.event_lines:
            lines.append("- nearby_events:")
            for line in record.event_lines:
                lines.append(f"  - `{line}`")
        else:
            lines.append("- nearby_events: none")
        lines.append("")
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    case_dir = args.case_dir.resolve()
    config_path = find_single(case_dir, "config_*.yaml")
    state_series_path = find_single(case_dir, "state_series_*.csv")
    stdout_path = find_single(case_dir, "solver_stdout_*.txt")
    diag_path = find_single(case_dir, "DIAG_*.txt")
    output_md = args.output_md.resolve() if args.output_md else case_dir / "jump_window_audit.md"

    config = load_yaml(config_path)
    sampled = sample_per_second(load_state_series(state_series_path))
    diag_df = load_diag(diag_path)

    fusion = config.get("fusion", {})
    phases = fusion.get("runtime_phases", []) or []
    windows = [
        (float(item["start_time"]), float(item["end_time"]))
        for item in (fusion.get("gnss_schedule", {}).get("enabled_windows", []) or [])
        if "start_time" in item and "end_time" in item
    ]

    records = detect_major_jumps(
        sampled=sampled,
        phases=phases,
        gnss_windows=windows,
        diag_df=diag_df,
        stdout_path=stdout_path,
        start_time=float(fusion.get("starttime", 0.0)),
        top_k=args.top_k,
        cluster_gap=args.cluster_gap,
        min_time=args.min_time,
    )
    report = render_report(case_dir.name, config, records)
    output_md.write_text(report, encoding="utf-8")
    print(report)
    print(f"\n[written] {output_md}")


if __name__ == "__main__":
    main()
