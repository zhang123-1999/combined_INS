import csv
import math
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
EXE = ROOT / "build" / "Release" / "eskf_fusion.exe"
BASE_CONFIG = ROOT / "config_data2_gnss.yaml"
OUT_DIR = ROOT / "output" / "tuning_motion_adaptive"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def _scalar_to_text(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return f"{value:.10g}"
    return str(value)


def set_yaml_scalar(text: str, key: str, value) -> str:
    value_s = _scalar_to_text(value)
    pattern = rf"(^\s*{re.escape(key)}:\s*).*$"
    repl = rf"\g<1>{value_s}"
    new_text, n = re.subn(pattern, repl, text, flags=re.MULTILINE)
    if n == 0:
        raise RuntimeError(f"Key not found in config: {key}")
    return new_text


def upsert_constraint_scalar(text: str, key: str, value) -> str:
    try:
        return set_yaml_scalar(text, key, value)
    except RuntimeError:
        pass

    value_s = _scalar_to_text(value)
    m = re.search(r"^(\s*)constraints:\s*$", text, flags=re.MULTILINE)
    if not m:
        raise RuntimeError("constraints section not found for key insertion")
    indent = m.group(1) + "  "
    insert_line = f"{indent}{key}: {value_s}\n"
    insert_pos = m.end()
    return text[:insert_pos] + "\n" + insert_line + text[insert_pos:]


def upsert_noise_scalar(text: str, key: str, value) -> str:
    try:
        return set_yaml_scalar(text, key, value)
    except RuntimeError:
        pass

    value_s = _scalar_to_text(value)
    m = re.search(r"^(\s*)noise:\s*$", text, flags=re.MULTILINE)
    if not m:
        raise RuntimeError("noise section not found for key insertion")
    indent = m.group(1) + "  "
    insert_line = f"{indent}{key}: {value_s}\n"
    insert_pos = m.end()
    return text[:insert_pos] + "\n" + insert_line + text[insert_pos:]


def upsert_yaml_scalar(text: str, key: str, value) -> str:
    noise_keys = {"sigma_mounting_pitch", "sigma_mounting_yaw"}
    if key in noise_keys:
        return upsert_noise_scalar(text, key, value)
    return upsert_constraint_scalar(text, key, value)


def parse_rmse(stdout: str):
    m = re.search(r"RMSE \(融合\) \[m\]:\s*([\-0-9.eE]+)\s+([\-0-9.eE]+)\s+([\-0-9.eE]+)", stdout)
    if not m:
        return None
    x, y, z = map(float, m.groups())
    return x, y, z


def parse_accept_ratio(stdout: str, tag: str):
    m = re.search(rf"\[Consistency\]\s+{tag}\s+.*?accept_ratio=([0-9.]+)", stdout)
    if not m:
        return float("nan")
    return float(m.group(1))


def build_candidates():
    common = {
        "enable_consistency_log": False,
        "enable_motion_adaptive_strategy": True,
        "enable_covariance_floor": True,
        "adaptive_statewise_update": True,
        "motion_eps_omega": 0.05,
        "motion_eps_ax": 0.3,
        "straight_constraint_hz": 10.0,
        "dynamic_constraint_hz": 50.0,
        "adaptive_q_min_ratio": 0.01,
        "p_floor_pos_var": 0.01,
        "p_floor_vel_var": 0.001,
        "p_floor_att_deg": 0.01,
        "p_floor_mounting_deg": 0.1,
        "p_floor_bg_var": 1.0e-8,
    }

    return [
        ("legacy_whole_update", {
            **common,
            "enable_motion_adaptive_strategy": False,
            "enable_covariance_floor": False,
        }),
        ("adaptive_default", common),
        ("adaptive_conservative", {
            **common,
            "motion_eps_omega": 0.04,
            "motion_eps_ax": 0.25,
            "straight_constraint_hz": 8.0,
            "dynamic_constraint_hz": 40.0,
            "adaptive_q_min_ratio": 0.005,
        }),
        ("adaptive_strong_freeze", {
            **common,
            "motion_eps_omega": 0.03,
            "motion_eps_ax": 0.2,
            "straight_constraint_hz": 6.0,
            "dynamic_constraint_hz": 40.0,
            "adaptive_q_min_ratio": 0.003,
        }),
        ("adaptive_loose_freeze", {
            **common,
            "motion_eps_omega": 0.07,
            "motion_eps_ax": 0.45,
            "straight_constraint_hz": 12.0,
            "dynamic_constraint_hz": 60.0,
            "adaptive_q_min_ratio": 0.02,
        }),
        ("adaptive_high_dynamic_rate", {
            **common,
            "dynamic_constraint_hz": 80.0,
        }),
        ("adaptive_low_dynamic_rate", {
            **common,
            "dynamic_constraint_hz": 30.0,
        }),
        ("adaptive_more_downsample_straight", {
            **common,
            "straight_constraint_hz": 5.0,
        }),
        ("adaptive_less_downsample_straight", {
            **common,
            "straight_constraint_hz": 15.0,
        }),
        ("adaptive_alt_floor_loose", {
            **common,
            "p_floor_pos_var": 0.005,
            "p_floor_vel_var": 0.0005,
            "p_floor_att_deg": 0.005,
            "p_floor_mounting_deg": 0.05,
        }),
        ("adaptive_alt_floor_strict", {
            **common,
            "p_floor_pos_var": 0.02,
            "p_floor_vel_var": 0.002,
            "p_floor_att_deg": 0.02,
            "p_floor_mounting_deg": 0.2,
        }),
        ("adaptive_candidate_mix", {
            **common,
            "motion_eps_omega": 0.06,
            "motion_eps_ax": 0.35,
            "straight_constraint_hz": 8.0,
            "dynamic_constraint_hz": 60.0,
            "adaptive_q_min_ratio": 0.008,
        }),
    ]


def main():
    if not EXE.exists():
        print(f"Missing executable: {EXE}", file=sys.stderr)
        return 1
    if not BASE_CONFIG.exists():
        print(f"Missing base config: {BASE_CONFIG}", file=sys.stderr)
        return 1

    base_text = BASE_CONFIG.read_text(encoding="utf-8")
    candidates = build_candidates()
    rows = []

    for idx, (name, params) in enumerate(candidates, start=1):
        cfg_text = base_text
        for k, v in params.items():
            cfg_text = upsert_yaml_scalar(cfg_text, k, v)
        cfg_path = OUT_DIR / f"{idx:02d}_{name}.yaml"
        cfg_path.write_text(cfg_text, encoding="utf-8")

        cmd = [str(EXE), "--config", str(cfg_path)]
        proc = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True)
        out = (proc.stdout or "") + "\n" + (proc.stderr or "")
        rmse = parse_rmse(out)
        if rmse is None:
            rmse = (float("inf"), float("inf"), float("inf"))
        rx, ry, rz = rmse
        norm3d = math.sqrt(rx * rx + ry * ry + rz * rz)
        mean_xyz = (rx + ry + rz) / 3.0
        nhc_acc = parse_accept_ratio(out, "NHC")
        odo_acc = parse_accept_ratio(out, "ODO")

        log_path = OUT_DIR / f"{idx:02d}_{name}.log"
        log_path.write_text(out, encoding="utf-8", errors="ignore")

        rows.append({
            "name": name,
            "config_path": str(cfg_path),
            "return_code": proc.returncode,
            "rmse_x": rx,
            "rmse_y": ry,
            "rmse_z": rz,
            "rmse_norm3d": norm3d,
            "rmse_mean_xyz": mean_xyz,
            "nhc_accept_ratio": nhc_acc,
            "odo_accept_ratio": odo_acc,
            **params,
        })
        print(f"[{idx}/{len(candidates)}] {name}: RMSE=({rx:.6f}, {ry:.6f}, {rz:.6f}), norm={norm3d:.6f}")

    rows.sort(key=lambda r: r["rmse_norm3d"])
    csv_path = OUT_DIR / "results.csv"
    fieldnames = list(rows[0].keys())
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    best = rows[0]
    best_txt = OUT_DIR / "best.txt"
    best_txt.write_text(
        "\n".join([
            f"name={best['name']}",
            f"rmse_x={best['rmse_x']}",
            f"rmse_y={best['rmse_y']}",
            f"rmse_z={best['rmse_z']}",
            f"rmse_norm3d={best['rmse_norm3d']}",
            f"config_path={best['config_path']}",
        ]),
        encoding="utf-8",
    )
    print(f"Best: {best['name']} norm3d={best['rmse_norm3d']:.6f}")
    print(f"Results: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
