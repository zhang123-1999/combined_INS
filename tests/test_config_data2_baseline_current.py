from pathlib import Path
import sys

import pytest
import math


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml


LEGACY_BASELINE_SIGMA_GYRO = 0.005
LEGACY_BASELINE_SIGMA_ACC = 0.05
LEGACY_BASELINE_SIGMA_BG = 0.000233
LEGACY_BASELINE_SIGMA_BA = 0.0005
LEGACY_BASELINE_SIGMA_SCALE = 0.0005
LEGACY_BASELINE_TAU_S = 3600.0


def test_official_baseline_config_matches_legacy_best_baseline_contract():
    cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    fusion = cfg["fusion"]
    noise = fusion["noise"]
    constraints = fusion["constraints"]
    init_cfg = fusion["init"]
    ablation = fusion["ablation"]
    runtime_phases = fusion["runtime_phases"]
    gnss_schedule = fusion["gnss_schedule"]

    assert fusion["output_path"] == "output/data2_baseline_current/SOL_data2_baseline_current.txt"
    assert (
        fusion["state_series_output_path"]
        == "output/data2_baseline_current/state_series_data2_baseline_current.csv"
    )
    assert noise["sigma_acc"] == pytest.approx(LEGACY_BASELINE_SIGMA_ACC)
    assert noise["sigma_gyro"] == pytest.approx(LEGACY_BASELINE_SIGMA_GYRO)
    assert noise["sigma_ba"] == pytest.approx(LEGACY_BASELINE_SIGMA_BA)
    assert noise["sigma_bg"] == pytest.approx(LEGACY_BASELINE_SIGMA_BG)
    assert noise["sigma_sg"] == pytest.approx(LEGACY_BASELINE_SIGMA_SCALE)
    assert noise["sigma_sa"] == pytest.approx(LEGACY_BASELINE_SIGMA_SCALE)
    assert noise["markov_corr_time"] == pytest.approx(LEGACY_BASELINE_TAU_S)
    assert noise["sigma_mounting"] == 1.0e-06
    assert noise["sigma_mounting_roll"] == 1.0e-06
    assert noise["sigma_mounting_pitch"] == 1.0e-06
    assert noise["sigma_mounting_yaw"] == 1.0e-06
    assert noise["sigma_lever_arm"] == 1.0e-05
    assert noise["sigma_gnss_lever_arm"] == 1.0e-06
    assert constraints["sigma_odo"] == 3.0
    assert constraints["sigma_nhc_y"] == 3.0
    assert constraints["sigma_nhc_z"] == 3.0
    assert constraints["enable_diagnostics"] is True
    assert constraints["enable_consistency_log"] is True
    assert ablation["disable_mounting_roll"] is False
    assert ablation["disable_gnss_lever_z"] is False
    assert ablation["disable_gyro_scale"] is True
    assert ablation["disable_accel_scale"] is True
    assert init_cfg["odo_scale"] == 1.0
    assert init_cfg["lever_arm0"] == [0.0, 0.0, 0.0]
    assert init_cfg["gnss_lever_arm0"] == [0.15, -0.22, -1.15]
    assert init_cfg["std_mounting_roll"] == 3.0
    assert init_cfg["std_mounting_pitch"] == 3.0
    assert init_cfg["std_mounting_yaw"] == 3.0
    assert gnss_schedule["enabled"] is True
    assert len(gnss_schedule["enabled_windows"]) == 10
    assert [phase["name"] for phase in runtime_phases] == [
        "phase1_ins_gnss_freeze_mounting_odo_states",
        "phase2_cov_seed",
        "phase2_ins_gnss_odo_nhc_freeze_gnss_lever",
        "phase3_periodic_gnss_outage_freeze_calibrated_extrinsics",
    ]
    assert runtime_phases[1]["end_time"] - runtime_phases[1]["start_time"] == pytest.approx(0.02)
    assert runtime_phases[1]["constraints"]["p_floor_odo_scale_var"] == 1.0e-04
    assert runtime_phases[1]["constraints"]["p_floor_lever_arm_vec"] == [0.16, 0.16, 0.16]


def test_research_seed_config_preserves_legacy_baseline_noise_reference():
    cfg = load_yaml(REPO_ROOT / "config_data2_research_seed_eskf.yaml")
    fusion = cfg["fusion"]
    noise = fusion["noise"]
    constraints = fusion["constraints"]
    init_cfg = fusion["init"]
    ablation = fusion["ablation"]

    assert fusion["output_path"] == "SOL_data2_baseline_eskf.txt"
    assert "state_series_output_path" not in fusion
    assert noise["sigma_acc"] == pytest.approx(LEGACY_BASELINE_SIGMA_ACC)
    assert noise["sigma_gyro"] == pytest.approx(LEGACY_BASELINE_SIGMA_GYRO)
    assert noise["sigma_ba"] == pytest.approx(LEGACY_BASELINE_SIGMA_BA)
    assert noise["sigma_bg"] == pytest.approx(LEGACY_BASELINE_SIGMA_BG)
    assert noise["sigma_sg"] == pytest.approx(LEGACY_BASELINE_SIGMA_SCALE)
    assert noise["sigma_sa"] == pytest.approx(LEGACY_BASELINE_SIGMA_SCALE)
    assert noise["markov_corr_time"] == pytest.approx(LEGACY_BASELINE_TAU_S)
    assert noise["sigma_mounting"] == 1.0e-04
    assert noise["sigma_mounting_roll"] == 1.0e-04
    assert noise["sigma_mounting_pitch"] == 1.0e-04
    assert noise["sigma_mounting_yaw"] == 1.0e-04
    assert noise["sigma_lever_arm"] == 1.0e-03
    assert noise["sigma_gnss_lever_arm"] == 1.0e-04
    assert constraints["sigma_odo"] == 0.1
    assert constraints["sigma_nhc_y"] == 0.1
    assert constraints["sigma_nhc_z"] == 0.1
    assert constraints["enable_diagnostics"] is False
    assert constraints["enable_consistency_log"] is False
    assert ablation["disable_mounting_roll"] is True
    assert ablation["disable_gnss_lever_z"] is True
    assert ablation.get("disable_gyro_scale", False) is False
    assert ablation.get("disable_accel_scale", False) is False
    assert init_cfg["odo_scale"] == 0.0
    assert init_cfg["gnss_lever_arm0"] == [0.0, 0.0, 0.0]
    assert "runtime_phases" not in fusion
    assert fusion.get("gnss_schedule", {}).get("enabled", False) is False
