from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]


def detect_has_header(path: Path) -> bool:
    with path.open("r", encoding="utf-8") as f:
        first = f.readline().strip()
    if not first:
        return False
    token = first.split()[0]
    return not token.lstrip("-").replace(".", "", 1).isdigit()


def load_gnss(path: Path) -> tuple[np.ndarray, list[str]]:
    has_header = detect_has_header(path)
    header: list[str] = []
    if has_header:
        with path.open("r", encoding="utf-8") as f:
            header = [f.readline().rstrip("\n")]
    data = np.loadtxt(path, comments="#")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data, header


def on_window_mask(
    t: np.ndarray,
    t0: float,
    first_on: float,
    on_dur: float,
    off_dur: float,
) -> np.ndarray:
    rel = t - t0
    if on_dur <= 0 or off_dur < 0:
        return np.ones_like(rel, dtype=bool)
    mask_first = (rel >= 0.0) & (rel < first_on)
    if first_on <= 0.0:
        first_on = 0.0
    rel2 = rel - first_on
    period = on_dur + off_dur
    if period <= 0.0:
        return mask_first
    in_cycle = rel2 >= 0.0
    cycle_phase = np.mod(rel2, period)
    mask_cycle = in_cycle & (cycle_phase < on_dur)
    return mask_first | mask_cycle


def filter_gnss(
    input_path: Path,
    output_path: Path,
    start_time: float,
    final_time: float,
    first_on: float,
    on_dur: float,
    off_dur: float,
) -> dict[str, float]:
    data, header = load_gnss(input_path)
    t = data[:, 0].astype(float)
    base_mask = (t >= start_time) & (t <= final_time)
    data = data[base_mask]
    t = t[base_mask]
    mask = on_window_mask(t, start_time, first_on, on_dur, off_dur)
    filtered = data[mask]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        for line in header:
            f.write(line + "\n")
        np.savetxt(f, filtered, fmt="%.9f")
    return {
        "rows_raw": int(data.shape[0]),
        "rows_kept": int(filtered.shape[0]),
        "ratio_kept": float(filtered.shape[0] / max(1, data.shape[0])),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Filter GNSS data with periodic outage windows.")
    parser.add_argument("--input", type=str, default=str(REPO_ROOT / "dataset/data2_converted/GNSS_converted.txt"))
    parser.add_argument("--output", type=str, default=str(REPO_ROOT / "output/review/EXP-20260310-data2-gnss-outage-cycle-r1/GNSS_outage_cycle.txt"))
    parser.add_argument("--start-time", type=float, default=528076.0)
    parser.add_argument("--final-time", type=float, default=530488.9)
    parser.add_argument("--first-on", type=float, default=900.0)
    parser.add_argument("--on", type=float, default=300.0)
    parser.add_argument("--off", type=float, default=120.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    stats = filter_gnss(
        Path(args.input),
        Path(args.output),
        args.start_time,
        args.final_time,
        args.first_on,
        args.on,
        args.off,
    )
    print(f"[filter_gnss_outage] rows kept: {stats['rows_kept']} / {stats['rows_raw']} ({stats['ratio_kept']:.3f})")
    print(f"[filter_gnss_outage] output: {Path(args.output).resolve()}")


if __name__ == "__main__":
    main()
