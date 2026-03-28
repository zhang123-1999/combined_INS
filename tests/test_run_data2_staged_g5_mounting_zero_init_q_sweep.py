import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml
from scripts.analysis.run_data2_state_sanity_matrix import build_truth_reference

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_staged_g5_mounting_zero_init_q_sweep.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing experiment script: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_staged_g5_mounting_zero_init_q_sweep", MODULE_PATH)
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
        phase2_mounting_cov_seed_duration=module.PHASE2_MOUNTING_COV_SEED_DURATION_DEFAULT,
    )


def test_case_configs_match_mounting_zero_init_q_sweep_plan():
    module = load_module()
    assert module.BASE_CONFIG_DEFAULT == Path("config_data2_research_seed_eskf.yaml")
    base_cfg = load_yaml(REPO_ROOT / "config_data2_research_seed_eskf.yaml")
    truth_reference = build_truth_reference(base_cfg)
    case_root = REPO_ROOT / "output" / "_tmp_staged_g5_mounting_zero_init_q_sweep_tests"
    args = build_args(module)

    assert module.EXP_ID_DEFAULT == "EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2"
    assert module.OUTPUT_DIR_DEFAULT == Path("output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325")
    assert module.PHASE3_GNSS_ON_DEFAULT == 90.0
    assert module.PHASE3_GNSS_OFF_DEFAULT == 90.0
    assert module.PHASE2_MOUNTING_COV_SEED_DURATION_DEFAULT == 0.02
    assert module.MOUNTING_INIT_STD_DEG == 3.0
    assert [spec.case_id for spec in module.CASE_SPECS] == [
        "staged_g5_mounting_zero_init_q_0p5x",
        "staged_g5_mounting_zero_init_q_1x",
        "staged_g5_mounting_zero_init_q_2x",
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

    q05 = built["staged_g5_mounting_zero_init_q_0p5x"]
    q10 = built["staged_g5_mounting_zero_init_q_1x"]
    q20 = built["staged_g5_mounting_zero_init_q_2x"]
    base_constraints = base_cfg["fusion"]["constraints"]
    base_noise = base_cfg["fusion"]["noise"]
    base_p0_diag = [float(x) for x in base_cfg["fusion"]["init"]["P0_diag"]]
    large_mount_var = math.radians(module.MOUNTING_INIT_STD_DEG) ** 2
    staged_mount_q = base_noise["sigma_mounting"] * module.G5_EXTRINSIC_NOISE_SCALE

    for cfg in (q05, q10, q20):
        assert cfg["fusion"]["ablation"]["disable_gyro_scale"] is True
        assert cfg["fusion"]["ablation"]["disable_accel_scale"] is True
        assert cfg["fusion"]["ablation"]["disable_mounting_roll"] is False
        assert cfg["fusion"]["ablation"]["disable_gnss_lever_z"] is False
        assert cfg["fusion"]["init"]["odo_scale"] == 1.0
        assert cfg["fusion"]["init"]["mounting_roll0"] == 0.0
        assert cfg["fusion"]["init"]["mounting_pitch0"] == 0.0
        assert cfg["fusion"]["init"]["mounting_yaw0"] == 0.0
        assert cfg["fusion"]["constraints"]["imu_mounting_angle"] == [0.0, 0.0, 0.0]
        assert cfg["fusion"]["init"]["lever_arm0"] == truth_reference["sources"]["odo_lever_truth"]["value_m"]
        assert cfg["fusion"]["init"]["gnss_lever_arm0"] == [
            truth_reference["states"]["gnss_lever_x"]["reference_value"],
            truth_reference["states"]["gnss_lever_y"]["reference_value"],
            truth_reference["states"]["gnss_lever_z"]["reference_value"],
        ]
        assert cfg["fusion"]["constraints"]["sigma_odo"] == base_constraints["sigma_odo"] * 30.0
        assert cfg["fusion"]["constraints"]["sigma_nhc_y"] == base_constraints["sigma_nhc_y"] * 30.0
        assert cfg["fusion"]["constraints"]["sigma_nhc_z"] == base_constraints["sigma_nhc_z"] * 30.0
        assert cfg["fusion"]["constraints"]["odo_lever_arm"] == truth_reference["sources"]["odo_lever_truth"]["value_m"]

        p0_diag = cfg["fusion"]["init"]["P0_diag"]
        assert len(p0_diag) == len(base_p0_diag)
        assert p0_diag[22] == large_mount_var
        assert p0_diag[23] == large_mount_var
        assert p0_diag[24] == large_mount_var
        for idx, (actual, expected) in enumerate(zip(p0_diag, base_p0_diag)):
            if idx in {22, 23, 24}:
                continue
            assert actual == expected

    assert q05["fusion"]["noise"]["sigma_mounting"] == staged_mount_q * 0.5
    assert q05["fusion"]["noise"]["sigma_mounting_roll"] == base_noise["sigma_mounting_roll"] * module.G5_EXTRINSIC_NOISE_SCALE * 0.5
    assert q05["fusion"]["noise"]["sigma_mounting_pitch"] == base_noise["sigma_mounting_pitch"] * module.G5_EXTRINSIC_NOISE_SCALE * 0.5
    assert q05["fusion"]["noise"]["sigma_mounting_yaw"] == base_noise["sigma_mounting_yaw"] * module.G5_EXTRINSIC_NOISE_SCALE * 0.5

    assert q10["fusion"]["noise"]["sigma_mounting"] == staged_mount_q
    assert q10["fusion"]["noise"]["sigma_mounting_roll"] == base_noise["sigma_mounting_roll"] * module.G5_EXTRINSIC_NOISE_SCALE
    assert q10["fusion"]["noise"]["sigma_mounting_pitch"] == base_noise["sigma_mounting_pitch"] * module.G5_EXTRINSIC_NOISE_SCALE
    assert q10["fusion"]["noise"]["sigma_mounting_yaw"] == base_noise["sigma_mounting_yaw"] * module.G5_EXTRINSIC_NOISE_SCALE

    assert q20["fusion"]["noise"]["sigma_mounting"] == staged_mount_q * 2.0
    assert q20["fusion"]["noise"]["sigma_mounting_roll"] == base_noise["sigma_mounting_roll"] * module.G5_EXTRINSIC_NOISE_SCALE * 2.0
    assert q20["fusion"]["noise"]["sigma_mounting_pitch"] == base_noise["sigma_mounting_pitch"] * module.G5_EXTRINSIC_NOISE_SCALE * 2.0
    assert q20["fusion"]["noise"]["sigma_mounting_yaw"] == base_noise["sigma_mounting_yaw"] * module.G5_EXTRINSIC_NOISE_SCALE * 2.0

    assert q10["fusion"]["noise"]["sigma_lever_arm"] == base_noise["sigma_lever_arm"] * module.G5_EXTRINSIC_NOISE_SCALE
    assert q10["fusion"]["noise"]["sigma_gnss_lever_arm"] == base_noise["sigma_gnss_lever_arm"] * module.G5_EXTRINSIC_NOISE_SCALE

    phase1, phase2_seed, phase2, phase3 = q10["fusion"]["runtime_phases"]
    assert phase1["constraints"]["enable_odo"] is False
    assert phase1["constraints"]["enable_nhc"] is False
    assert phase1["ablation"]["disable_odo_scale"] is True
    assert phase1["ablation"]["disable_mounting"] is True
    assert phase1["ablation"]["disable_odo_lever_arm"] is True
    assert "noise" not in phase1

    assert phase2_seed["name"] == "phase2_mounting_cov_seed"
    assert phase2_seed["constraints"]["enable_odo"] is True
    assert phase2_seed["constraints"]["enable_nhc"] is True
    assert phase2_seed["constraints"]["enable_covariance_floor"] is True
    assert phase2_seed["constraints"]["p_floor_mounting_deg"] == module.MOUNTING_INIT_STD_DEG
    assert phase2_seed["ablation"]["disable_gnss_lever_arm"] is True
    assert "noise" not in phase2_seed

    assert phase2["constraints"]["enable_odo"] is True
    assert phase2["constraints"]["enable_nhc"] is True
    assert phase2["ablation"]["disable_gnss_lever_arm"] is True
    assert "noise" not in phase2

    assert phase3["constraints"]["enable_odo"] is True
    assert phase3["constraints"]["enable_nhc"] is True
    assert phase3["ablation"]["disable_gnss_lever_arm"] is True
    assert phase3["ablation"]["disable_odo_scale"] is True
    assert phase3["ablation"]["disable_mounting"] is True
    assert phase3["ablation"]["disable_odo_lever_arm"] is True
    assert "noise" not in phase3

    metadata = metadata_map["staged_g5_mounting_zero_init_q_1x"]
    assert metadata["phase1_end_time"] < metadata["phase2_end_time"]
    assert metadata["phase2_main_start_time"] == metadata["phase1_end_time"] + module.PHASE2_MOUNTING_COV_SEED_DURATION_DEFAULT
    assert metadata["gnss_on_windows"]
    assert metadata["gnss_off_windows"]
    off_start, off_end = metadata["gnss_off_windows"][0]
    assert off_end - off_start == 90.0
    assert off_start == metadata["phase2_end_time"] + 90.0
    assert metadata["mounting_init_mode"] == "zero_init_large_p0"
    assert metadata["noise_scales"]["odo_nhc_r_scale"] == 30.0
    assert metadata["noise_scales"]["odo_nhc_r_scale_rel_to_g5"] == 6.0
    assert metadata["noise_scales"]["mounting_q_scale_rel_to_staged_6x"] == 1.0
    assert metadata["noise_scales"]["mounting_q_scale_rel_to_base_q"] == module.G5_EXTRINSIC_NOISE_SCALE
    assert metadata["large_initial_covariance"]["mounting_std_deg"] == module.MOUNTING_INIT_STD_DEG
    assert metadata["phase2_mounting_cov_seed_duration_s"] == module.PHASE2_MOUNTING_COV_SEED_DURATION_DEFAULT
    assert metadata["phase2_mounting_cov_seed_floor_deg"] == module.MOUNTING_INIT_STD_DEG
    assert metadata["mounting_zero_init_semantics"] == "total_and_state_zero"
    assert metadata["truth_initialized"] == [
        "lever_odo(25-27)",
        "lever_gnss(28-30)",
        "odo_scale(21)=1.0",
    ]

    q10_truth_reference = build_truth_reference(q10)
    assert q10_truth_reference["sources"]["mounting_total_truth"]["value_deg"] == {
        "roll": 0.0,
        "pitch": 0.36,
        "yaw": 1.37,
    }
    assert q10_truth_reference["states"]["mounting_roll"]["reference_value"] == 0.0
    assert q10_truth_reference["states"]["mounting_pitch"]["reference_value"] == 0.36
    assert q10_truth_reference["states"]["mounting_yaw"]["reference_value"] == 1.37


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


def test_pva_group_plots_use_error_specs():
    module = load_module()

    assert [group.group_id for group in module.PVA_ERROR_GROUP_SPECS] == ["position", "velocity", "attitude"]
    assert [state.key for state in module.PVA_ERROR_GROUP_SPECS[0].states] == ["p_n_m", "p_e_m", "p_u_m"]
    assert [state.truth_column for state in module.PVA_ERROR_GROUP_SPECS[0].states] == [
        "truth_p_n_m",
        "truth_p_e_m",
        "truth_p_u_m",
    ]
    assert [state.key for state in module.PVA_ERROR_GROUP_SPECS[1].states] == ["v_n_mps", "v_e_mps", "v_u_mps"]
    assert [state.key for state in module.PVA_ERROR_GROUP_SPECS[2].states] == ["roll_deg", "pitch_deg", "yaw_deg"]
