import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml
from scripts.analysis.run_data2_state_sanity_matrix import build_truth_reference

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_staged_g5_odo_lever_process_q_sweep.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing experiment script: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_staged_g5_odo_lever_process_q_sweep", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def build_args(module):
    return SimpleNamespace(
        phase1_end_offset=module.PHASE1_END_OFFSET_DEFAULT,
        phase2_end_offset=module.PHASE2_END_OFFSET_DEFAULT,
        phase3_gnss_on=module.PHASE3_GNSS_ON_DEFAULT,
        phase3_gnss_off=module.PHASE3_GNSS_OFF_DEFAULT,
        phase2_cov_seed_duration=module.PHASE2_COV_SEED_DURATION_DEFAULT,
    )


def test_case_configs_match_odo_lever_process_q_sweep_plan():
    module = load_module()
    assert module.BASE_CONFIG_DEFAULT == Path("config_data2_research_seed_eskf.yaml")
    base_cfg = load_yaml(REPO_ROOT / "config_data2_research_seed_eskf.yaml")
    truth_reference = build_truth_reference(base_cfg)
    case_root = REPO_ROOT / "output" / "_tmp_staged_g5_odo_lever_process_q_sweep_tests"
    args = build_args(module)

    assert module.EXP_ID_DEFAULT == "EXP-20260326-data2-staged-g5-odo-lever-process-q-sweep-r1"
    assert module.OUTPUT_DIR_DEFAULT == Path("output/data2_staged_g5_odo_lever_process_q_sweep_r1_20260326")
    assert module.PHASE2_COV_SEED_DURATION_DEFAULT == 0.02
    assert module.MOUNTING_INIT_STD_DEG == 3.0
    assert module.ODO_SCALE_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED == 2.0
    assert module.ODO_LEVER_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED == 4.0
    assert [spec.case_id for spec in module.CASE_SPECS] == [
        "staged_g5_odo_lever_q_0p25x",
        "staged_g5_odo_lever_q_0p5x",
        "staged_g5_odo_lever_q_1x",
        "staged_g5_odo_lever_q_2x",
        "staged_g5_odo_lever_q_4x",
    ]

    built = {}
    metadata_map = {}
    for spec in module.CASE_SPECS:
        case_dir = case_root / spec.case_id
        cfg, metadata = module.build_case_config(base_cfg, truth_reference, case_dir, spec, args)
        built[spec.case_id] = cfg
        metadata_map[spec.case_id] = metadata
        assert cfg["fusion"]["gnss_schedule"]["enabled"] is True
        assert cfg["fusion"]["runtime_phases"]
        assert "post_gnss_ablation" not in cfg["fusion"]

    q025 = built["staged_g5_odo_lever_q_0p25x"]
    q050 = built["staged_g5_odo_lever_q_0p5x"]
    q100 = built["staged_g5_odo_lever_q_1x"]
    q200 = built["staged_g5_odo_lever_q_2x"]
    q400 = built["staged_g5_odo_lever_q_4x"]
    base_constraints = base_cfg["fusion"]["constraints"]
    base_noise = base_cfg["fusion"]["noise"]
    base_init = base_cfg["fusion"]["init"]
    base_p0_diag = [float(x) for x in base_init["P0_diag"]]
    base_lever_var = [float(x) ** 2 for x in base_init["std_lever_arm"]]
    mounting_var = math.radians(module.MOUNTING_INIT_STD_DEG) ** 2
    staged_mount_q = base_noise["sigma_mounting"] * module.G5_EXTRINSIC_NOISE_SCALE
    staged_gnss_lever_q = base_noise["sigma_gnss_lever_arm"] * module.G5_EXTRINSIC_NOISE_SCALE
    staged_odo_lever_q = base_noise["sigma_lever_arm"] * module.G5_EXTRINSIC_NOISE_SCALE
    fixed_odo_scale_seed_var = float(base_init["std_odo_scale"]) ** 2 * 4.0
    fixed_lever_seed_var = [value * 16.0 for value in base_lever_var]

    for cfg in (q025, q050, q100, q200, q400):
        assert cfg["fusion"]["ablation"]["disable_gyro_scale"] is True
        assert cfg["fusion"]["ablation"]["disable_accel_scale"] is True
        assert cfg["fusion"]["ablation"]["disable_mounting_roll"] is False
        assert cfg["fusion"]["ablation"]["disable_gnss_lever_z"] is False
        assert cfg["fusion"]["ablation"]["disable_odo_scale"] is False
        assert cfg["fusion"]["init"]["odo_scale"] == 1.0
        assert cfg["fusion"]["init"]["lever_arm0"] == [0.0, 0.0, 0.0]
        assert cfg["fusion"]["constraints"]["odo_lever_arm"] == [0.0, 0.0, 0.0]
        assert cfg["fusion"]["init"]["gnss_lever_arm0"] == [
            truth_reference["states"]["gnss_lever_x"]["reference_value"],
            truth_reference["states"]["gnss_lever_y"]["reference_value"],
            truth_reference["states"]["gnss_lever_z"]["reference_value"],
        ]
        assert cfg["fusion"]["constraints"]["sigma_odo"] == (
            base_constraints["sigma_odo"] * module.G5_ODO_NHC_MEASUREMENT_NOISE_SCALE
        )
        assert cfg["fusion"]["constraints"]["sigma_nhc_y"] == (
            base_constraints["sigma_nhc_y"] * module.G5_ODO_NHC_MEASUREMENT_NOISE_SCALE
        )
        assert cfg["fusion"]["constraints"]["sigma_nhc_z"] == (
            base_constraints["sigma_nhc_z"] * module.G5_ODO_NHC_MEASUREMENT_NOISE_SCALE
        )

        p0_diag = cfg["fusion"]["init"]["P0_diag"]
        assert len(p0_diag) == len(base_p0_diag)
        assert p0_diag[21] == base_p0_diag[21]
        assert p0_diag[22] == mounting_var
        assert p0_diag[23] == mounting_var
        assert p0_diag[24] == mounting_var
        assert p0_diag[25] == base_p0_diag[25]
        assert p0_diag[26] == base_p0_diag[26]
        assert p0_diag[27] == base_p0_diag[27]

    assert q100["fusion"]["noise"]["sigma_mounting"] == staged_mount_q
    assert q100["fusion"]["noise"]["sigma_mounting_roll"] == base_noise["sigma_mounting_roll"] * module.G5_EXTRINSIC_NOISE_SCALE
    assert q100["fusion"]["noise"]["sigma_mounting_pitch"] == base_noise["sigma_mounting_pitch"] * module.G5_EXTRINSIC_NOISE_SCALE
    assert q100["fusion"]["noise"]["sigma_mounting_yaw"] == base_noise["sigma_mounting_yaw"] * module.G5_EXTRINSIC_NOISE_SCALE
    assert q100["fusion"]["noise"]["sigma_gnss_lever_arm"] == staged_gnss_lever_q

    assert q025["fusion"]["noise"]["sigma_lever_arm"] == staged_odo_lever_q * 0.25
    assert q050["fusion"]["noise"]["sigma_lever_arm"] == staged_odo_lever_q * 0.5
    assert q100["fusion"]["noise"]["sigma_lever_arm"] == staged_odo_lever_q
    assert q200["fusion"]["noise"]["sigma_lever_arm"] == staged_odo_lever_q * 2.0
    assert q400["fusion"]["noise"]["sigma_lever_arm"] == staged_odo_lever_q * 4.0

    phase1, phase2_seed, phase2, phase3 = q100["fusion"]["runtime_phases"]
    assert phase1["ablation"]["disable_odo_scale"] is True
    assert phase1["ablation"]["disable_mounting"] is True
    assert phase1["ablation"]["disable_odo_lever_arm"] is True
    assert phase1["constraints"]["enable_odo"] is False
    assert phase1["constraints"]["enable_nhc"] is False

    assert phase2_seed["name"] == "phase2_cov_seed"
    assert phase2_seed["constraints"]["enable_odo"] is True
    assert phase2_seed["constraints"]["enable_nhc"] is True
    assert phase2_seed["constraints"]["enable_covariance_floor"] is True
    assert phase2_seed["constraints"]["p_floor_mounting_deg"] == module.MOUNTING_INIT_STD_DEG
    assert phase2_seed["constraints"]["p_floor_odo_scale_var"] == fixed_odo_scale_seed_var
    assert phase2_seed["constraints"]["p_floor_lever_arm_vec"] == fixed_lever_seed_var
    assert phase2_seed["ablation"]["disable_gnss_lever_arm"] is True
    assert "noise" not in phase2_seed

    assert phase2["constraints"]["enable_odo"] is True
    assert phase2["constraints"]["enable_nhc"] is True
    assert phase2["ablation"]["disable_gnss_lever_arm"] is True
    assert "noise" not in phase2

    assert phase3["ablation"]["disable_gnss_lever_arm"] is True
    assert phase3["ablation"]["disable_odo_scale"] is True
    assert phase3["ablation"]["disable_mounting"] is True
    assert phase3["ablation"]["disable_odo_lever_arm"] is True

    metadata = metadata_map["staged_g5_odo_lever_q_1x"]
    assert metadata["phase1_end_time"] < metadata["phase2_end_time"]
    assert metadata["phase2_main_start_time"] == metadata["phase1_end_time"] + module.PHASE2_COV_SEED_DURATION_DEFAULT
    assert metadata["odo_scale_phase2_seed"]["std"] == float(base_init["std_odo_scale"]) * 2.0
    assert metadata["odo_scale_phase2_seed"]["var"] == fixed_odo_scale_seed_var
    assert metadata["odo_lever_phase2_seed"]["std_vec_m"] == [float(x) * 4.0 for x in base_init["std_lever_arm"]]
    assert metadata["odo_lever_phase2_seed"]["var_vec_m2"] == fixed_lever_seed_var
    assert metadata["noise_scales"]["lever_q_scale_rel_to_staged_g5"] == 1.0
    assert metadata["noise_scales"]["lever_q_scale_rel_to_base_q"] == module.G5_EXTRINSIC_NOISE_SCALE
    assert metadata["noise_scales"]["lever_q_sigma_m_sqrt_hz"] == staged_odo_lever_q


def test_best_rerun_plot_config_matches_requested_layout():
    module = load_module()

    plot_config = module.build_best_rerun_plot_config()
    overview_keys = [spec.key for spec in plot_config.overview_states]
    group_ids = [group.group_id for group in plot_config.group_specs]

    assert overview_keys[:9] == [
        "p_n_m",
        "p_e_m",
        "p_u_m",
        "v_n_mps",
        "v_e_mps",
        "v_u_mps",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
    ]
    assert "sg_x_ppm" not in overview_keys
    assert "sa_x_ppm" not in overview_keys
    assert plot_config.plot_modes_by_key["p_n_m"] == "error"
    assert plot_config.plot_modes_by_key["v_n_mps"] == "error"
    assert plot_config.plot_modes_by_key["roll_deg"] == "error"
    assert group_ids == [
        "position",
        "velocity",
        "attitude",
        "ba",
        "bg",
        "odo_scale",
        "mounting",
        "odo_lever",
        "gnss_lever",
    ]
    assert plot_config.truth_keys_to_hide == {
        "ba_x_mgal",
        "ba_y_mgal",
        "ba_z_mgal",
        "bg_x_degh",
        "bg_y_degh",
        "bg_z_degh",
    }
    assert [spec.key for spec in plot_config.vehicle_velocity_error_states] == [
        "vv_x_err_mps",
        "vv_y_err_mps",
        "vv_z_err_mps",
    ]
    assert [spec.key for spec in plot_config.vehicle_heading_error_states] == ["vehicle_heading_err_deg"]


def test_build_vehicle_error_frame_adds_vehicle_speed_and_heading_errors(monkeypatch):
    module = load_module()

    merged_df = pd.DataFrame(
        {
            "timestamp": [10.0, 11.0],
            "fused_roll": [0.0, 0.0],
            "fused_pitch": [0.0, 0.0],
            "fused_yaw": [0.0, 0.0],
            "total_mounting_roll_deg": [0.0, 0.0],
            "total_mounting_pitch_deg": [0.0, 0.0],
            "total_mounting_yaw_deg": [0.0, 0.0],
        }
    )
    truth_interp_df = pd.DataFrame(
        {
            "timestamp": [10.0, 11.0],
            "roll": [0.0, 0.0],
            "pitch": [0.0, 0.0],
            "yaw": [0.0, 0.0],
        }
    )
    imu_df = pd.DataFrame({"timestamp": [10.0, 11.0]})
    truth_reference = {"sources": {}}
    motion_df = pd.DataFrame(
        {
            "v_v_x_mps": [1.5, 1.8],
            "v_v_y_mps": [-0.2, 0.4],
            "v_v_z_mps": [0.1, -0.3],
            "truth_v_v_x_mps": [1.0, 1.2],
            "truth_v_v_y_mps": [-0.1, 0.1],
            "truth_v_v_z_mps": [0.0, -0.1],
        }
    )

    monkeypatch.setattr(module, "build_motion_frame", lambda *args, **kwargs: motion_df)
    monkeypatch.setattr(
        module,
        "relative_euler_error_deg",
        lambda *args, **kwargs: (np.zeros(2), np.zeros(2), np.array([0.25, -0.75], dtype=float)),
    )

    vehicle_error_df = module.build_vehicle_error_frame(merged_df, truth_interp_df, imu_df, truth_reference)

    assert list(vehicle_error_df.columns) == [
        "timestamp",
        "vv_x_err_mps",
        "vv_y_err_mps",
        "vv_z_err_mps",
        "vehicle_heading_err_deg",
    ]
    assert vehicle_error_df["timestamp"].tolist() == [10.0, 11.0]
    assert np.allclose(vehicle_error_df["vv_x_err_mps"].to_numpy(dtype=float), [0.5, 0.6])
    assert np.allclose(vehicle_error_df["vv_y_err_mps"].to_numpy(dtype=float), [-0.1, 0.3])
    assert np.allclose(vehicle_error_df["vv_z_err_mps"].to_numpy(dtype=float), [0.1, -0.2])
    assert np.allclose(vehicle_error_df["vehicle_heading_err_deg"].to_numpy(dtype=float), [0.25, -0.75])
