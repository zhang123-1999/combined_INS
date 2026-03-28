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

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_staged_g5_odo_lever_phase2_seed_sweep.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing experiment script: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_staged_g5_odo_lever_phase2_seed_sweep", MODULE_PATH)
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


def test_case_configs_match_odo_lever_phase2_seed_sweep_plan():
    module = load_module()
    assert module.BASE_CONFIG_DEFAULT == Path("config_data2_research_seed_eskf.yaml")
    base_cfg = load_yaml(REPO_ROOT / "config_data2_research_seed_eskf.yaml")
    truth_reference = build_truth_reference(base_cfg)
    case_root = REPO_ROOT / "output" / "_tmp_staged_g5_odo_lever_phase2_seed_sweep_tests"
    args = build_args(module)

    assert module.EXP_ID_DEFAULT == "EXP-20260326-data2-staged-g5-odo-lever-phase2-seed-sweep-r2"
    assert module.OUTPUT_DIR_DEFAULT == Path("output/data2_staged_g5_odo_lever_phase2_seed_sweep_r2_20260326")
    assert module.PHASE2_COV_SEED_DURATION_DEFAULT == 0.02
    assert module.MOUNTING_INIT_STD_DEG == 3.0
    assert [spec.case_id for spec in module.CASE_SPECS] == [
        "staged_g5_odo_lever_phase2_seed_0p5x",
        "staged_g5_odo_lever_phase2_seed_1x",
        "staged_g5_odo_lever_phase2_seed_2x",
        "staged_g5_odo_lever_phase2_seed_4x",
        "staged_g5_odo_lever_phase2_seed_8x",
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

    s05 = built["staged_g5_odo_lever_phase2_seed_0p5x"]
    s10 = built["staged_g5_odo_lever_phase2_seed_1x"]
    s20 = built["staged_g5_odo_lever_phase2_seed_2x"]
    s40 = built["staged_g5_odo_lever_phase2_seed_4x"]
    s80 = built["staged_g5_odo_lever_phase2_seed_8x"]
    base_constraints = base_cfg["fusion"]["constraints"]
    base_noise = base_cfg["fusion"]["noise"]
    base_init = base_cfg["fusion"]["init"]
    base_p0_diag = [float(x) for x in base_init["P0_diag"]]
    mounting_var = math.radians(module.MOUNTING_INIT_STD_DEG) ** 2
    staged_mount_q = base_noise["sigma_mounting"] * module.G5_EXTRINSIC_NOISE_SCALE
    base_odo_scale_var = float(base_init["std_odo_scale"]) ** 2
    base_lever_var = [float(x) ** 2 for x in base_init["std_lever_arm"]]

    for cfg in (s05, s10, s20, s40, s80):
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
        assert cfg["fusion"]["constraints"]["sigma_odo"] == base_constraints["sigma_odo"] * 30.0
        assert cfg["fusion"]["constraints"]["sigma_nhc_y"] == base_constraints["sigma_nhc_y"] * 30.0
        assert cfg["fusion"]["constraints"]["sigma_nhc_z"] == base_constraints["sigma_nhc_z"] * 30.0

        p0_diag = cfg["fusion"]["init"]["P0_diag"]
        assert len(p0_diag) == len(base_p0_diag)
        assert p0_diag[21] == base_p0_diag[21]
        assert p0_diag[22] == mounting_var
        assert p0_diag[23] == mounting_var
        assert p0_diag[24] == mounting_var
        assert p0_diag[25] == base_p0_diag[25]
        assert p0_diag[26] == base_p0_diag[26]
        assert p0_diag[27] == base_p0_diag[27]

    assert s10["fusion"]["noise"]["sigma_mounting"] == staged_mount_q
    assert s10["fusion"]["noise"]["sigma_mounting_roll"] == base_noise["sigma_mounting_roll"] * module.G5_EXTRINSIC_NOISE_SCALE
    assert s10["fusion"]["noise"]["sigma_mounting_pitch"] == base_noise["sigma_mounting_pitch"] * module.G5_EXTRINSIC_NOISE_SCALE
    assert s10["fusion"]["noise"]["sigma_mounting_yaw"] == base_noise["sigma_mounting_yaw"] * module.G5_EXTRINSIC_NOISE_SCALE
    assert s10["fusion"]["noise"]["sigma_lever_arm"] == base_noise["sigma_lever_arm"] * module.G5_EXTRINSIC_NOISE_SCALE

    phase1, phase2_seed, phase2, phase3 = s10["fusion"]["runtime_phases"]
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
    assert phase2_seed["constraints"]["p_floor_odo_scale_var"] == base_odo_scale_var * 4.0
    assert phase2_seed["constraints"]["p_floor_lever_arm_vec"] == base_lever_var
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

    assert s05["fusion"]["runtime_phases"][1]["constraints"]["p_floor_lever_arm_vec"] == [x * 0.25 for x in base_lever_var]
    assert s20["fusion"]["runtime_phases"][1]["constraints"]["p_floor_lever_arm_vec"] == [x * 4.0 for x in base_lever_var]
    assert s40["fusion"]["runtime_phases"][1]["constraints"]["p_floor_lever_arm_vec"] == [x * 16.0 for x in base_lever_var]
    assert s80["fusion"]["runtime_phases"][1]["constraints"]["p_floor_lever_arm_vec"] == [x * 64.0 for x in base_lever_var]

    metadata = metadata_map["staged_g5_odo_lever_phase2_seed_1x"]
    assert metadata["phase1_end_time"] < metadata["phase2_end_time"]
    assert metadata["phase2_main_start_time"] == metadata["phase1_end_time"] + module.PHASE2_COV_SEED_DURATION_DEFAULT
    assert metadata["odo_scale_phase2_seed"]["std"] == float(base_init["std_odo_scale"]) * 2.0
    assert metadata["odo_scale_phase2_seed"]["var"] == base_odo_scale_var * 4.0
    assert metadata["odo_lever_zero_init_semantics"] == "total_and_state_zero"
    assert metadata["odo_lever_phase2_seed"]["std_vec_m"] == [float(x) for x in base_init["std_lever_arm"]]
    assert metadata["odo_lever_phase2_seed"]["var_vec_m2"] == base_lever_var
