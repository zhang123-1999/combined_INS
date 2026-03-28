from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root  # noqa: E402
from scripts.analysis.run_mounting_yaw_nhc_bgz_compare import mtime_text  # noqa: E402


BASE_CONFIG_DEFAULT = Path(
    "output/debug_mounting_yaw_first_update_geometry_r1_20260323/"
    "cases/baseline/config_baseline.yaml"
)
MECHANISM_DEFAULT = Path(
    "output/debug_mounting_yaw_first_update_geometry_r1_20260323/"
    "cases/baseline/SOL_baseline_mechanism.csv"
)
POS_DEFAULT = Path("dataset/data2_converted/POS_converted.txt")
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_observability_svd_r1_20260323")

STRAIGHT_SPEED_MIN_M_S = 3.0
TURN_YAW_RATE_MIN_DEG_S = 8.0

STATE_LABELS = ["att_z", "bg_z", "mounting_yaw"]


def parse_vec(text: Any) -> np.ndarray:
    stripped = str(text).strip()
    if not stripped or stripped == "[]":
        return np.zeros(0, dtype=float)
    if stripped[0] == "[" and stripped[-1] == "]":
        stripped = stripped[1:-1]
    if not stripped:
        return np.zeros(0, dtype=float)
    return np.asarray([float(part) for part in stripped.split(";") if part], dtype=float)


def load_pos_dataframe(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=r"\s+",
        header=None,
        names=["t", "lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"],
        engine="python",
    )


def find_first_turn_time(pos_df: pd.DataFrame, start_t: float) -> float:
    t = pos_df["t"].to_numpy(dtype=float)
    speed = np.hypot(pos_df["vn"].to_numpy(dtype=float), pos_df["ve"].to_numpy(dtype=float))
    yaw = np.unwrap(np.deg2rad(pos_df["yaw"].to_numpy(dtype=float)))
    yaw_rate_deg_s = np.rad2deg(np.gradient(yaw, t))
    mask = (t >= start_t) & (speed >= STRAIGHT_SPEED_MIN_M_S) & (
        np.abs(yaw_rate_deg_s) >= TURN_YAW_RATE_MIN_DEG_S
    )
    hits = np.flatnonzero(mask)
    if hits.size == 0:
        raise RuntimeError("failed to detect first turn time from POS_converted.txt")
    return float(t[hits[0]])


def interpolate_rpy_deg(pos_df: pd.DataFrame, t_query: float) -> tuple[float, float, float]:
    t = pos_df["t"].to_numpy(dtype=float)
    roll = np.interp(t_query, t, pos_df["roll"].to_numpy(dtype=float))
    pitch = np.interp(t_query, t, pos_df["pitch"].to_numpy(dtype=float))
    yaw = np.interp(t_query, t, pos_df["yaw"].to_numpy(dtype=float))
    return float(roll), float(pitch), float(yaw)


def bgz_to_attz_phi_coeff(roll_deg: float, pitch_deg: float, dt: float) -> float:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    c_bn_zz = math.cos(pitch) * math.cos(roll)
    return -c_bn_zz * dt


def build_heading_phi(*, dt: float, roll_deg: float, pitch_deg: float, markov_corr_time: float) -> np.ndarray:
    phi = np.eye(3, dtype=float)
    phi[0, 1] = bgz_to_attz_phi_coeff(roll_deg, pitch_deg, dt)
    if markov_corr_time > 0.0:
        phi[1, 1] = 1.0 - dt / markov_corr_time
    return phi


def build_heading_h(row: pd.Series) -> np.ndarray:
    h_att = parse_vec(row["h_att_z_vec"])
    h_bg = parse_vec(row["h_bg_z_vec"])
    h_mount = parse_vec(row["h_mount_yaw_vec"])
    if h_att.size == 0 or h_bg.size == 0 or h_mount.size == 0:
        raise RuntimeError("mechanism row contains empty heading Jacobian vector")
    if not (h_att.size == h_bg.size == h_mount.size):
        raise RuntimeError("heading Jacobian vectors have inconsistent dimensions")
    return np.column_stack([h_att, h_bg, h_mount])


def build_window_observability(
    mech_df: pd.DataFrame,
    pos_df: pd.DataFrame,
    *,
    end_t: float,
    markov_corr_time: float,
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    window = mech_df[mech_df["t_state"] <= end_t + 1.0e-9].reset_index(drop=True)
    if window.empty:
        raise RuntimeError(f"window end_t={end_t} produced empty mechanism slice")

    transition = np.eye(3, dtype=float)
    obs_blocks: list[np.ndarray] = []
    row_meta: list[dict[str, Any]] = []
    prev_t = float(window.iloc[0]["t_state"])
    for idx, row in window.iterrows():
        t_state = float(row["t_state"])
        if idx > 0:
            dt = max(0.0, t_state - prev_t)
            roll_deg, pitch_deg, _ = interpolate_rpy_deg(pos_df, prev_t)
            phi = build_heading_phi(
                dt=dt,
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
                markov_corr_time=markov_corr_time,
            )
            transition = phi @ transition
        h_heading = build_heading_h(row)
        obs_blocks.append(h_heading @ transition)
        row_meta.append(
            {
                "seq_idx": int(idx + 1),
                "tag": str(row["tag"]),
                "t_state": t_state,
                "y_dim": int(h_heading.shape[0]),
                "transition_01": float(transition[0, 1]),
                "transition_11": float(transition[1, 1]),
            }
        )
        prev_t = t_state
    return np.vstack(obs_blocks), row_meta


def vector_projection_report(v: np.ndarray) -> dict[str, float]:
    abs_v = np.abs(v)
    denom = float(abs_v.sum())
    if denom <= 0.0:
        return {f"proj_{label}": float("nan") for label in STATE_LABELS}
    return {f"proj_{label}": float(abs_v[idx] / denom) for idx, label in enumerate(STATE_LABELS)}


def plot_singular_values(results_df: pd.DataFrame, output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 4.5))
    x = np.arange(len(results_df))
    for idx in range(3):
        ax.semilogy(
            x,
            results_df[f"singular_value_{idx + 1}"].to_numpy(dtype=float),
            marker="o",
            linewidth=1.2,
            label=f"s{idx + 1}",
        )
    ax.set_xticks(x)
    ax.set_xticklabels(results_df["window_id"].tolist(), rotation=20)
    ax.set_ylabel("singular value")
    ax.set_title("Reduced heading-space observability singular values")
    ax.grid(alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=170)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compute reduced-order heading observability SVD for bg_z and mounting_yaw."
    )
    parser.add_argument("--config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--mechanism", type=Path, default=MECHANISM_DEFAULT)
    parser.add_argument("--pos", type=Path, default=POS_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    args = parser.parse_args()

    config_path = REPO_ROOT / args.config
    mechanism_path = REPO_ROOT / args.mechanism
    pos_path = REPO_ROOT / args.pos
    outdir = REPO_ROOT / args.output_dir
    ensure_dir(outdir)

    cfg = load_yaml(config_path)
    markov_corr_time = float(cfg["fusion"]["noise"]["markov_corr_time"])
    mech_df = pd.read_csv(mechanism_path)
    pos_df = load_pos_dataframe(pos_path)

    if mech_df.empty:
        raise RuntimeError("mechanism csv is empty")

    t0 = float(mech_df["t_state"].iloc[0])
    t_last = float(mech_df["t_state"].iloc[-1])
    first_turn_t = find_first_turn_time(pos_df, t0)
    window_defs = [
        ("first_1s", min(t_last, t0 + 1.0)),
        ("first_3s", min(t_last, t0 + 3.0)),
        ("pre_first_turn", min(t_last, first_turn_t)),
        ("first_turn_plus_10s", min(t_last, first_turn_t + 10.0)),
    ]

    rows: list[dict[str, Any]] = []
    vectors_payload: dict[str, Any] = {
        "config_path": rel_from_root(config_path, REPO_ROOT),
        "mechanism_path": rel_from_root(mechanism_path, REPO_ROOT),
        "pos_path": rel_from_root(pos_path, REPO_ROOT),
        "config_mtime": mtime_text(config_path),
        "mechanism_mtime": mtime_text(mechanism_path),
        "pos_mtime": mtime_text(pos_path),
        "t0": t0,
        "first_turn_t": first_turn_t,
        "t_last": t_last,
        "markov_corr_time": markov_corr_time,
        "windows": {},
    }

    for window_id, end_t in window_defs:
        obs, row_meta = build_window_observability(
            mech_df,
            pos_df,
            end_t=end_t,
            markov_corr_time=markov_corr_time,
        )
        _, singular_values, vt = np.linalg.svd(obs, full_matrices=False)
        smallest_vec = vt[-1, :]
        proj = vector_projection_report(smallest_vec)
        row = {
            "window_id": window_id,
            "window_end_t": float(end_t),
            "window_duration_s": float(end_t - t0),
            "n_update_rows": int(len(row_meta)),
            "obs_rows": int(obs.shape[0]),
            "singular_value_1": float(singular_values[0]),
            "singular_value_2": float(singular_values[1]),
            "singular_value_3": float(singular_values[2]),
            "condition_number": float(singular_values[0] / singular_values[-1]),
            "smallest_vec_att_z": float(smallest_vec[0]),
            "smallest_vec_bg_z": float(smallest_vec[1]),
            "smallest_vec_mounting_yaw": float(smallest_vec[2]),
        }
        row.update(proj)
        rows.append(row)
        vectors_payload["windows"][window_id] = {
            "end_t": float(end_t),
            "row_meta": row_meta,
            "singular_values": [float(v) for v in singular_values],
            "smallest_right_singular_vector": [float(v) for v in smallest_vec],
            "projection_ratio": proj,
        }

    results_df = pd.DataFrame(rows)
    results_path = outdir / "window_metrics.csv"
    results_df.to_csv(results_path, index=False)

    vectors_path = outdir / "window_vectors.json"
    vectors_path.write_text(json.dumps(vectors_payload, indent=2, ensure_ascii=False), encoding="utf-8")

    plot_path = outdir / "singular_values.png"
    plot_singular_values(results_df, plot_path)

    lines = [
        "# Reduced Heading Observability SVD",
        "",
        "目标：基于 `NHC/ODO` accepted update 的 heading 子空间 Jacobian，构造 reduced-order local observability matrix，",
        "检查 `bg_z` 与 `mounting_yaw` 是否在冷启动阶段共同占据最弱可观方向。",
        "",
        "## Inputs",
        f"- config: `{rel_from_root(config_path, REPO_ROOT)}`",
        f"- mechanism: `{rel_from_root(mechanism_path, REPO_ROOT)}`",
        f"- pos: `{rel_from_root(pos_path, REPO_ROOT)}`",
        f"- first mechanism t0: `{t0:.6f}` s",
        f"- first turn t: `{first_turn_t:.6f}` s",
        f"- markov_corr_time: `{markov_corr_time:.6f}` s",
        "",
        "## Window Summary",
        "",
        "| window_id | duration_s | n_update_rows | s1 | s2 | s3 | proj_att_z | proj_bg_z | proj_mounting_yaw |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for _, row in results_df.iterrows():
        lines.append(
            "| "
            + " | ".join(
                [
                    str(row["window_id"]),
                    f"{float(row['window_duration_s']):.6f}",
                    str(int(row["n_update_rows"])),
                    f"{float(row['singular_value_1']):.6e}",
                    f"{float(row['singular_value_2']):.6e}",
                    f"{float(row['singular_value_3']):.6e}",
                    f"{float(row['proj_att_z']):.6f}",
                    f"{float(row['proj_bg_z']):.6f}",
                    f"{float(row['proj_mounting_yaw']):.6f}",
                ]
            )
            + " |"
        )
    strongest_pre_turn = results_df.loc[results_df["window_id"] == "pre_first_turn"].iloc[0]
    lines.extend(
        [
            "",
            "## Key Reading",
            (
                f"- `pre_first_turn` 的最小右奇异向量投影为 "
                f"`att_z/bg_z/mounting_yaw = "
                f"{strongest_pre_turn['proj_att_z']:.6f}/"
                f"{strongest_pre_turn['proj_bg_z']:.6f}/"
                f"{strongest_pre_turn['proj_mounting_yaw']:.6f}`。"
            ),
            (
                "- 若 `bg_z` 与 `mounting_yaw` 两个投影在最弱方向上持续占主导，则说明冷启动直行段内，"
                "该二者在 heading 子空间里呈现结构性混叠，而非单纯数值偶然。"
            ),
            "",
            "## Artifacts",
            f"- metrics: `{rel_from_root(results_path, REPO_ROOT)}`",
            f"- vectors: `{rel_from_root(vectors_path, REPO_ROOT)}`",
            f"- plot: `{rel_from_root(plot_path, REPO_ROOT)}`",
        ]
    )
    (outdir / "summary.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"[done] metrics: {results_path}")
    print(f"[done] vectors: {vectors_path}")
    print(f"[done] plot: {plot_path}")


if __name__ == "__main__":
    main()
