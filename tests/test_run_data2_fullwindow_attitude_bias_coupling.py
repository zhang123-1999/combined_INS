from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml
from scripts.analysis.run_data2_state_sanity_matrix import build_truth_reference

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_fullwindow_attitude_bias_coupling.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing experiment script: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_fullwindow_attitude_bias_coupling", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_case_configs_match_fullwindow_plan(tmp_path):
    module = load_module()
    assert module.BASE_CONFIG_DEFAULT == Path("config_data2_research_seed_eskf.yaml")
    base_cfg = load_yaml(REPO_ROOT / "config_data2_research_seed_eskf.yaml")
    truth_reference = build_truth_reference(base_cfg)
    case_root = REPO_ROOT / "output" / "_tmp_case_config_tests"

    assert [spec.case_id for spec in module.CASE_SPECS] == [
        "group1_ins_gnss_truth_gnss_lever_fixed",
        "group2_ins_gnss_odo_nhc_truth_extrinsics_fixed",
        "group3_ins_gnss_odo_nhc_truth_extrinsics_soft",
        "group4_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise",
        "group5_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise_no_imu_scale",
    ]

    built = {}
    for spec in module.CASE_SPECS:
        case_dir = case_root / spec.case_id
        cfg, _ = module.build_case_config(base_cfg, truth_reference, case_dir, spec)
        built[spec.case_id] = cfg
        assert "runtime_phases" not in cfg["fusion"]
        assert not cfg["fusion"].get("gnss_schedule", {}).get("enabled", False)
        assert "post_gnss_ablation" not in cfg["fusion"]

    g1 = built["group1_ins_gnss_truth_gnss_lever_fixed"]
    assert g1["fusion"]["constraints"]["enable_odo"] is False
    assert g1["fusion"]["constraints"]["enable_nhc"] is False
    assert g1["fusion"]["ablation"]["disable_gnss_lever_arm"] is True
    assert g1["fusion"]["init"]["gnss_lever_arm0"] == [
        truth_reference["states"]["gnss_lever_x"]["reference_value"],
        truth_reference["states"]["gnss_lever_y"]["reference_value"],
        truth_reference["states"]["gnss_lever_z"]["reference_value"],
    ]

    g2 = built["group2_ins_gnss_odo_nhc_truth_extrinsics_fixed"]
    assert g2["fusion"]["constraints"]["enable_odo"] is True
    assert g2["fusion"]["constraints"]["enable_nhc"] is True
    assert g2["fusion"]["ablation"]["disable_mounting"] is True
    assert g2["fusion"]["ablation"]["disable_mounting_roll"] is False
    assert g2["fusion"]["ablation"]["disable_odo_lever_arm"] is True
    assert g2["fusion"]["ablation"]["disable_gnss_lever_arm"] is True
    assert g2["fusion"]["ablation"]["disable_odo_scale"] is True
    assert g2["fusion"]["init"]["odo_scale"] == 1.0

    g3 = built["group3_ins_gnss_odo_nhc_truth_extrinsics_soft"]
    base_noise = base_cfg["fusion"]["noise"]
    assert g3["fusion"]["ablation"]["disable_mounting"] is False
    assert g3["fusion"]["ablation"]["disable_mounting_roll"] is False
    assert g3["fusion"]["ablation"]["disable_odo_lever_arm"] is False
    assert g3["fusion"]["ablation"]["disable_gnss_lever_arm"] is False
    assert g3["fusion"]["ablation"]["disable_odo_scale"] is True
    assert g3["fusion"]["noise"]["sigma_mounting"] == base_noise["sigma_mounting"] * 0.01
    assert g3["fusion"]["noise"]["sigma_mounting_pitch"] == base_noise["sigma_mounting_pitch"] * 0.01
    assert g3["fusion"]["noise"]["sigma_mounting_yaw"] == base_noise["sigma_mounting_yaw"] * 0.01
    assert g3["fusion"]["noise"]["sigma_lever_arm"] == base_noise["sigma_lever_arm"] * 0.01
    assert g3["fusion"]["noise"]["sigma_gnss_lever_arm"] == base_noise["sigma_gnss_lever_arm"] * 0.01

    g4 = built["group4_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise"]
    base_constraints = base_cfg["fusion"]["constraints"]
    assert g4["fusion"]["constraints"]["sigma_odo"] == base_constraints["sigma_odo"] * 5.0
    assert g4["fusion"]["constraints"]["sigma_nhc_y"] == base_constraints["sigma_nhc_y"] * 5.0
    assert g4["fusion"]["constraints"]["sigma_nhc_z"] == base_constraints["sigma_nhc_z"] * 5.0

    g5 = built["group5_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise_no_imu_scale"]
    assert g5["fusion"]["constraints"]["sigma_odo"] == base_constraints["sigma_odo"] * 5.0
    assert g5["fusion"]["constraints"]["sigma_nhc_y"] == base_constraints["sigma_nhc_y"] * 5.0
    assert g5["fusion"]["constraints"]["sigma_nhc_z"] == base_constraints["sigma_nhc_z"] * 5.0
    assert g5["fusion"]["ablation"]["disable_gyro_scale"] is True
    assert g5["fusion"]["ablation"]["disable_accel_scale"] is True


def test_all_state_specs_cover_31_states_with_truth_overlay():
    module = load_module()

    assert len(module.ALL_STATE_SPECS) == 31
    assert all(spec.truth_state_key is not None for spec in module.ALL_STATE_SPECS)
    assert [group.group_id for group in module.GROUP_SPECS] == [
        "position",
        "velocity",
        "attitude",
        "ba",
        "bg",
        "sg",
        "sa",
        "odo_scale",
        "mounting",
        "odo_lever",
        "gnss_lever",
    ]


def test_build_state_frame_uses_total_mounting_semantics_and_preserves_state_delta():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_research_seed_eskf.yaml")
    truth_reference = build_truth_reference(base_cfg)

    merged_df = pd.DataFrame(
        {
            "timestamp": [528276.0],
            "fused_x": [0.0],
            "fused_y": [0.0],
            "fused_z": [0.0],
            "fused_vx": [0.0],
            "fused_vy": [0.0],
            "fused_vz": [0.0],
            "fused_roll": [0.1],
            "fused_pitch": [0.2],
            "fused_yaw": [0.3],
            "ba_x_mgal": [1.0],
            "ba_y_mgal": [2.0],
            "ba_z_mgal": [3.0],
            "bg_x_degh": [4.0],
            "bg_y_degh": [5.0],
            "bg_z_degh": [6.0],
            "sg_x_ppm": [7.0],
            "sg_y_ppm": [8.0],
            "sg_z_ppm": [9.0],
            "sa_x_ppm": [10.0],
            "sa_y_ppm": [11.0],
            "sa_z_ppm": [12.0],
            "odo_scale_state": [1.0],
            "mounting_roll_deg": [0.0],
            "mounting_pitch_deg": [0.12],
            "mounting_yaw_deg": [-0.25],
            "total_mounting_roll_deg": [0.0],
            "total_mounting_pitch_deg": [0.12],
            "total_mounting_yaw_deg": [1.13],
            "odo_lever_x_m": [0.2],
            "odo_lever_y_m": [-1.0],
            "odo_lever_z_m": [0.6],
            "gnss_lever_x_m": [0.15],
            "gnss_lever_y_m": [-0.22],
            "gnss_lever_z_m": [-1.15],
        }
    )
    truth_interp_df = pd.DataFrame(
        {
            "timestamp": [528276.0],
            "lat": [0.0],
            "lon": [0.0],
            "h": [0.0],
            "vn": [0.0],
            "ve": [0.0],
            "vd": [0.0],
            "roll": [0.0],
            "pitch": [0.0],
            "yaw": [0.0],
        }
    )

    frame = module.build_state_frame(merged_df, truth_interp_df, truth_reference)

    assert frame.loc[0, "mounting_roll_deg"] == 0.0
    assert frame.loc[0, "mounting_pitch_deg"] == 0.12
    assert frame.loc[0, "mounting_yaw_deg"] == 1.13
    assert frame.loc[0, "truth_mounting_roll_deg"] == 0.0
    assert frame.loc[0, "truth_mounting_pitch_deg"] == 0.36
    assert frame.loc[0, "truth_mounting_yaw_deg"] == 1.37

    assert frame.loc[0, "mounting_state_roll_deg"] == 0.0
    assert frame.loc[0, "mounting_state_pitch_deg"] == 0.12
    assert frame.loc[0, "mounting_state_yaw_deg"] == -0.25
    assert frame.loc[0, "truth_mounting_state_roll_deg"] == 0.0
    assert frame.loc[0, "truth_mounting_state_pitch_deg"] == 0.36
    assert math.isclose(frame.loc[0, "truth_mounting_state_yaw_deg"], -0.01, abs_tol=1.0e-12)
