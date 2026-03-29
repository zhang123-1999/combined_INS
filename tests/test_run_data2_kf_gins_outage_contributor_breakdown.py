import importlib.util
from pathlib import Path
import sys

import numpy as np
import pandas as pd


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_kf_gins_outage_contributor_breakdown.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing outage contributor breakdown runner: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_kf_gins_outage_contributor_breakdown", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_load_current_sol_exposes_unified_attitude_columns(tmp_path):
    module = load_module()
    sol_path = tmp_path / "solver.sol"
    sol_path.write_text(
        "timestamp fused_x fused_y fused_z fused_vx fused_vy fused_vz "
        "fused_roll fused_pitch fused_yaw mounting_pitch mounting_yaw odo_scale "
        "sg_x sg_y sg_z sa_x sa_y sa_z ba_x ba_y ba_z bg_x bg_y bg_z "
        "lever_x lever_y lever_z gnss_lever_x gnss_lever_y gnss_lever_z\n"
        + " ".join(
            [
                "528270.000000",
                "1", "2", "3",
                "4", "5", "6",
                "7.1", "8.2", "9.3",
                "10", "11", "12",
                "13", "14", "15",
                "16", "17", "18",
                "19", "20", "21",
                "22", "23", "24",
                "25", "26", "27",
                "28", "29", "30",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    frame = module.load_current_sol(sol_path)

    assert frame.loc[0, "fused_roll"] == 7.1
    assert frame.loc[0, "fused_pitch"] == 8.2
    assert frame.loc[0, "fused_yaw"] == 9.3
    assert frame.loc[0, "roll_deg"] == 7.1
    assert frame.loc[0, "pitch_deg"] == 8.2
    assert frame.loc[0, "yaw_deg"] == 9.3


def test_analyse_window_from_shared_rows_uses_shared_contract(monkeypatch):
    module = load_module()

    monkeypatch.setattr(
        module,
        "along_cross_basis",
        lambda truth, start_t, end_t: (np.array([1.0, 0.0]), np.array([0.0, 1.0])),
    )
    monkeypatch.setattr(
        module,
        "contributors_current",
        lambda p_ecef_prev, v_ecef_prev, v_ecef_curr, dt: {
            "dv_total": v_ecef_curr - v_ecef_prev,
            "dv_g": np.zeros(3),
            "dv_c": np.zeros(3),
            "dv_sf": v_ecef_curr - v_ecef_prev,
        },
    )
    monkeypatch.setattr(
        module,
        "contributors_kf",
        lambda lat_prev_deg, lon_prev_deg, h_prev_m, v_ned_prev, v_ned_curr, dt: {
            "dv_total": v_ned_curr - v_ned_prev,
            "dv_g": np.zeros(3),
            "dv_c": np.zeros(3),
            "dv_sf": v_ned_curr - v_ned_prev,
        },
    )

    timestamps = [0.0, 0.1, 0.2, 1.0, 1.1, 1.2]
    current_sol = pd.DataFrame(
        {
            "timestamp": timestamps,
            "fused_x": [0.0] * 6,
            "fused_y": [0.0] * 6,
            "fused_z": [0.0] * 6,
            "fused_vx": [0.0, 10.0, 11.0, 0.0, 20.0, 21.0],
            "fused_vy": [0.0] * 6,
            "fused_vz": [0.0] * 6,
        }
    )
    kf_nav = pd.DataFrame(
        {
            "timestamp": timestamps,
            "lat_deg": [30.0] * 6,
            "lon_deg": [114.0] * 6,
            "h_m": [10.0] * 6,
            "vn_mps": [0.0, 8.0, 9.0, 0.0, 18.0, 19.0],
            "ve_mps": [0.0] * 6,
            "vd_mps": [0.0] * 6,
        }
    )
    truth = pd.DataFrame(
        {
            "timestamp": [0.0, 2.0],
            "vn_mps": [1.0, 1.0],
            "ve_mps": [0.0, 0.0],
        }
    )
    shared_rows = pd.DataFrame(
        {
            "window_name": ["w", "w", "w", "w"],
            "interval_start_t": [0.0, 0.0, 1.0, 1.0],
            "interval_end_t": [1.0, 1.0, 2.0, 2.0],
            "step_idx": [1, 2, 1, 2],
            "current_t": [0.1, 0.2, 1.1, 1.2],
            "kf_t": [0.1, 0.2, 1.1, 1.2],
        }
    )
    current_gnss_debug = pd.DataFrame(
        {
            "gnss_t": [0.0, 1.0],
            "state_t": [0.0, 1.0],
            "state_p_after_x": [0.0, 0.0],
            "state_p_after_y": [0.0, 0.0],
            "state_p_after_z": [0.0, 0.0],
            "state_v_after_x": [9.0, 19.0],
            "state_v_after_y": [0.0, 0.0],
            "state_v_after_z": [0.0, 0.0],
        }
    )
    kf_gnss_debug = pd.DataFrame(
        {
            "gnss_t": [0.0, 1.0],
            "state_t": [0.0, 1.0],
            "state_pos_after_x": [np.deg2rad(30.0), np.deg2rad(30.0)],
            "state_pos_after_y": [np.deg2rad(114.0), np.deg2rad(114.0)],
            "state_pos_after_z": [10.0, 10.0],
            "state_vel_after_x": [7.0, 17.0],
            "state_vel_after_y": [0.0, 0.0],
            "state_vel_after_z": [0.0, 0.0],
        }
    )

    step_df, summary_df = module.analyse_window_from_shared_rows(
        "w", current_sol, kf_nav, truth, shared_rows, current_gnss_debug, kf_gnss_debug
    )

    assert len(step_df) == 4
    assert step_df.loc[0, "delta_dv_total_along"] == 0.0
    delta_total = summary_df[
        (summary_df["solver"] == "delta") & (summary_df["contributor"] == "dv_total")
    ].iloc[0]
    assert delta_total["cum_along"] == 0.0
    assert delta_total["cum_cross"] == 0.0
    assert delta_total["cum_down"] == 0.0
    assert delta_total["cum_norm"] == 0.0
    assert delta_total["n_steps_per_interval"] == 2
    assert delta_total["n_intervals"] == 2
