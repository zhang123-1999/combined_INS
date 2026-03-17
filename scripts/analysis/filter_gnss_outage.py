from __future__ import annotations

import argparse
import datetime as dt
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
    data = np.loadtxt(path, comments="#", skiprows=1 if has_header else 0)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data, header


def on_window_mask(
    t: np.ndarray,
    t0: float,
    initial_on: float,
    on_dur: float,
    off_dur: float,
    cycle_starts_with: str = "off",
) -> np.ndarray:
    rel = t - t0
    if on_dur <= 0 or off_dur < 0:
        return np.ones_like(rel, dtype=bool)
    initial_on = max(initial_on, 0.0)
    mask_initial = (rel >= 0.0) & (rel < initial_on)
    rel2 = rel - initial_on
    period = on_dur + off_dur
    if period <= 0.0:
        return mask_initial
    in_cycle = rel2 >= 0.0
    cycle_phase = np.mod(rel2, period)
    if cycle_starts_with == "off":
        mask_cycle = in_cycle & (cycle_phase >= off_dur)
    else:
        mask_cycle = in_cycle & (cycle_phase < on_dur)
    return mask_initial | mask_cycle


def describe_schedule(
    start_time: float,
    initial_on: float,
    on_dur: float,
    off_dur: float,
    cycle_starts_with: str,
) -> dict[str, float | str]:
    initial_on = max(initial_on, 0.0)
    first_cycle_start = start_time + initial_on
    if cycle_starts_with == "off":
        first_off_start = first_cycle_start
        first_off_end = first_off_start + off_dur
        first_on_start = first_off_end
        first_on_end = first_on_start + on_dur
    else:
        first_on_start = first_cycle_start
        first_on_end = first_on_start + on_dur
        first_off_start = first_on_end
        first_off_end = first_off_start + off_dur
    return {
        "initial_on_s": float(initial_on),
        "cycle_starts_with": cycle_starts_with,
        "on_s": float(on_dur),
        "off_s": float(off_dur),
        "first_cycle_start_time": float(first_cycle_start),
        "first_off_start_time": float(first_off_start),
        "first_off_end_time": float(first_off_end),
        "first_on_start_time": float(first_on_start),
        "first_on_end_time": float(first_on_end),
    }


def filter_gnss(
    input_path: Path,
    output_path: Path,
    start_time: float,
    final_time: float,
    initial_on: float = 0.0,
    on_dur: float = 0.0,
    off_dur: float = 0.0,
    cycle_starts_with: str = "off",
    first_on: float | None = None,
) -> dict[str, float | str]:
    if first_on is not None:
        if abs(initial_on) > 1.0e-9 and abs(initial_on - first_on) > 1.0e-9:
            raise ValueError("initial_on and first_on must match when both are provided")
        initial_on = first_on
    data, header = load_gnss(input_path)
    rows_total = int(data.shape[0])
    t = data[:, 0].astype(float)
    base_mask = (t >= start_time) & (t <= final_time)
    data = data[base_mask]
    t = t[base_mask]
    mask = on_window_mask(t, start_time, initial_on, on_dur, off_dur, cycle_starts_with)
    filtered = data[mask]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        for line in header:
            f.write(line + "\n")
        np.savetxt(f, filtered, fmt="%.9f")
    stats: dict[str, float | str] = {
        "input_path": str(input_path.resolve()),
        "output_path": str(output_path.resolve()),
        "input_rows_total": rows_total,
        "rows_raw": int(data.shape[0]),
        "rows_kept": int(filtered.shape[0]),
        "ratio_kept": float(filtered.shape[0] / max(1, data.shape[0])),
        "start_time": float(start_time),
        "final_time": float(final_time),
    }
    stats.update(describe_schedule(start_time, initial_on, on_dur, off_dur, cycle_starts_with))
    return stats


def parse_args() -> argparse.Namespace:
    today = dt.date.today().strftime("%Y%m%d")
    default_out = (
        REPO_ROOT
        / "output"
        / "review"
        / f"EXP-{today}-data2-rtk-outage-eskf-r1"
        / "GNSS_outage_cycle_rtk_300on_100off_150on.txt"
    )
    parser = argparse.ArgumentParser(description="Filter GNSS data with periodic outage windows.")
    parser.add_argument("--input", type=str, default=str(REPO_ROOT / "dataset/data2/rtk.txt"))
    parser.add_argument("--output", type=str, default=str(default_out))
    parser.add_argument("--start-time", type=float, default=528076.0)
    parser.add_argument("--final-time", type=float, default=530488.9)
    parser.add_argument("--initial-on", "--first-on", dest="initial_on", type=float, default=300.0)
    parser.add_argument("--on", type=float, default=150.0)
    parser.add_argument("--off", type=float, default=100.0)
    parser.add_argument("--cycle-start", choices=("off", "on"), default="off")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    stats = filter_gnss(
        Path(args.input),
        Path(args.output),
        args.start_time,
        args.final_time,
        args.initial_on,
        args.on,
        args.off,
        args.cycle_start,
    )
    print(f"[filter_gnss_outage] rows kept: {stats['rows_kept']} / {stats['rows_raw']} ({stats['ratio_kept']:.3f})")
    print(
        "[filter_gnss_outage] schedule: "
        f"initial_on={stats['initial_on_s']:.3f}s, "
        f"cycle_start={stats['cycle_starts_with']}, "
        f"off={stats['off_s']:.3f}s, on={stats['on_s']:.3f}s"
    )
    print(
        "[filter_gnss_outage] first cycle window: "
        f"off=[{stats['first_off_start_time']:.3f}, {stats['first_off_end_time']:.3f}), "
        f"on=[{stats['first_on_start_time']:.3f}, {stats['first_on_end_time']:.3f})"
    )
    print(f"[filter_gnss_outage] output: {Path(args.output).resolve()}")


if __name__ == "__main__":
    main()
