import importlib.util
import math
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_baseline_ins_gnss_outage_no_odo_nhc.py"
POS320_SIGMA_GYRO = 0.05 * math.pi / 180.0 / math.sqrt(3600.0)
POS320_SIGMA_ACC = 0.1 / math.sqrt(3600.0)
POS320_SIGMA_BG = math.radians(0.5) / 3600.0
POS320_SIGMA_BA = 25.0e-5
POS320_SIGMA_SCALE = 300.0e-6
POS320_TAU_S = 4.0 * 3600.0
OVERRIDE_SIGMA_BG_DEGH = 3.0
OVERRIDE_SIGMA_BA_MGAL = 8.0
OVERRIDE_SIGMA_BG = math.radians(OVERRIDE_SIGMA_BG_DEGH) / 3600.0
OVERRIDE_SIGMA_BA = OVERRIDE_SIGMA_BA_MGAL * 1.0e-5
PROCESS_SIGMA_BG_DEGH = 0.25
PROCESS_SIGMA_BA_MGAL = 12.5
PROCESS_SIGMA_BG = math.radians(PROCESS_SIGMA_BG_DEGH) / 3600.0
PROCESS_SIGMA_BA = PROCESS_SIGMA_BA_MGAL * 1.0e-5
INIT_STD_BG_DEGH = 1.0
INIT_STD_BA_MGAL = 50.0
INIT_STD_BG = math.radians(INIT_STD_BG_DEGH) / 3600.0
INIT_STD_BA = INIT_STD_BA_MGAL * 1.0e-5
PHASE3_PROCESS_SIGMA_BG_DEGH = 0.125
PHASE3_PROCESS_SIGMA_BA_MGAL = 6.25
PHASE3_PROCESS_SIGMA_BG = math.radians(PHASE3_PROCESS_SIGMA_BG_DEGH) / 3600.0
PHASE3_PROCESS_SIGMA_BA = PHASE3_PROCESS_SIGMA_BA_MGAL * 1.0e-5
PROCESS_SIGMA_SG_PPM = 150.0
PROCESS_SIGMA_SA_PPM = 120.0
PROCESS_SIGMA_SG = PROCESS_SIGMA_SG_PPM * 1.0e-6
PROCESS_SIGMA_SA = PROCESS_SIGMA_SA_PPM * 1.0e-6
INIT_STD_SG_PPM = 600.0
INIT_STD_SA_PPM = 900.0
INIT_STD_SG = INIT_STD_SG_PPM * 1.0e-6
INIT_STD_SA = INIT_STD_SA_PPM * 1.0e-6
PHASE12_PROCESS_SIGMA_BG_DEGH = 0.125
PHASE12_PROCESS_SIGMA_BA_MGAL = 6.25
PHASE12_PROCESS_SIGMA_SG_PPM = 150.0
PHASE12_PROCESS_SIGMA_SA_PPM = 150.0
PHASE12_PROCESS_SIGMA_BG = math.radians(PHASE12_PROCESS_SIGMA_BG_DEGH) / 3600.0
PHASE12_PROCESS_SIGMA_BA = PHASE12_PROCESS_SIGMA_BA_MGAL * 1.0e-5
PHASE12_PROCESS_SIGMA_SG = PHASE12_PROCESS_SIGMA_SG_PPM * 1.0e-6
PHASE12_PROCESS_SIGMA_SA = PHASE12_PROCESS_SIGMA_SA_PPM * 1.0e-6
PHASE3_SIGMA_ACC_MPS_SQRT_HR = 0.2
PHASE3_SIGMA_GYRO_DEG_SQRT_HR = 0.1
PHASE3_SIGMA_ACC = PHASE3_SIGMA_ACC_MPS_SQRT_HR / math.sqrt(3600.0)
PHASE3_SIGMA_GYRO = math.radians(PHASE3_SIGMA_GYRO_DEG_SQRT_HR) / math.sqrt(3600.0)


def load_module():
    assert MODULE_PATH.exists(), f"missing INS/GNSS outage runner: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_baseline_ins_gnss_outage_no_odo_nhc", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_defaults_match_ins_gnss_outage_contract():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_ins_gnss_outage_no_odo_nhc_contract")

    assert module.EXP_ID_DEFAULT == "EXP-20260326-data2-baseline-ins-gnss-outage-no-odo-nhc-r1"
    assert module.OUTPUT_DIR_DEFAULT == Path("output/data2_baseline_ins_gnss_outage_no_odo_nhc")
    assert module.BASE_CONFIG_DEFAULT == Path("config_data2_baseline_eskf.yaml")
    assert module.CASE_ID_DEFAULT == "data2_baseline_ins_gnss_outage_no_odo_nhc"

    cfg, metadata = module.build_run_config(base_cfg, output_dir)
    fusion = cfg["fusion"]
    constraints = fusion["constraints"]
    ablation = fusion["ablation"]
    noise = fusion["noise"]
    init_cfg = fusion["init"]
    runtime_phases = fusion["runtime_phases"]

    assert (
        fusion["output_path"]
        == "output/_tmp_baseline_ins_gnss_outage_no_odo_nhc_contract/SOL_data2_baseline_ins_gnss_outage_no_odo_nhc.txt"
    )
    assert (
        fusion["state_series_output_path"]
        == "output/_tmp_baseline_ins_gnss_outage_no_odo_nhc_contract/state_series_data2_baseline_ins_gnss_outage_no_odo_nhc.csv"
    )
    assert constraints["enable_odo"] is False
    assert constraints["enable_nhc"] is False
    assert ablation["disable_odo_scale"] is True
    assert ablation["disable_mounting"] is True
    assert ablation["disable_odo_lever_arm"] is True
    assert ablation["disable_gnss_lever_arm"] is False
    assert ablation["disable_gyro_scale"] is False
    assert ablation["disable_accel_scale"] is False
    assert noise["sigma_acc"] == POS320_SIGMA_ACC
    assert noise["sigma_gyro"] == POS320_SIGMA_GYRO
    assert noise["sigma_ba"] == POS320_SIGMA_BA
    assert noise["sigma_bg"] == POS320_SIGMA_BG
    assert noise["sigma_sg"] == POS320_SIGMA_SCALE
    assert noise["sigma_sa"] == POS320_SIGMA_SCALE
    assert noise["sigma_ba_vec"] == [POS320_SIGMA_BA] * 3
    assert noise["sigma_bg_vec"] == [POS320_SIGMA_BG] * 3
    assert noise["sigma_sg_vec"] == [POS320_SIGMA_SCALE] * 3
    assert noise["sigma_sa_vec"] == [POS320_SIGMA_SCALE] * 3
    assert noise["markov_corr_time"] == POS320_TAU_S

    for field in ["ba0", "bg0", "sg0", "sa0", "std_ba", "std_bg", "std_sg", "std_sa"]:
        assert field not in init_cfg
    assert init_cfg["P0_diag"][9:12] == [POS320_SIGMA_BA**2] * 3
    assert init_cfg["P0_diag"][12:15] == [POS320_SIGMA_BG**2] * 3
    assert init_cfg["P0_diag"][15:18] == [POS320_SIGMA_SCALE**2] * 3
    assert init_cfg["P0_diag"][18:21] == [POS320_SIGMA_SCALE**2] * 3

    assert [phase["name"] for phase in runtime_phases] == [
        "phase1_ins_gnss_freeze_odo_states",
        "phase2_ins_gnss_freeze_odo_states",
        "phase3_periodic_gnss_outage_freeze_gnss_lever",
    ]
    assert runtime_phases[0]["constraints"]["enable_odo"] is False
    assert runtime_phases[0]["constraints"]["enable_nhc"] is False
    assert runtime_phases[1]["constraints"]["enable_odo"] is False
    assert runtime_phases[1]["constraints"]["enable_nhc"] is False
    assert runtime_phases[0]["ablation"]["disable_gnss_lever_arm"] is False
    assert runtime_phases[1]["ablation"]["disable_gnss_lever_arm"] is False
    assert runtime_phases[2]["ablation"]["disable_gnss_lever_arm"] is True
    assert runtime_phases[2]["constraints"]["enable_odo"] is False
    assert runtime_phases[2]["constraints"]["enable_nhc"] is False

    assert metadata["case_id"] == "data2_baseline_ins_gnss_outage_no_odo_nhc"
    assert metadata["phase1_window"] == [528076.0, 528276.0]
    assert metadata["phase2_window"] == [528276.0, 528776.0]
    assert metadata["phase3_window"] == [528776.0, 530488.9]
    assert metadata["gnss_off_windows"][0] == [528866.0, 528956.0]


def test_ins_gnss_plot_config_keeps_scale_factor_groups():
    module = load_module()
    plot_config = module.build_ins_gnss_plot_config()
    group_ids = [group.group_id for group in plot_config.group_specs]

    assert "sg" in group_ids
    assert "sa" in group_ids
    assert "ba" in group_ids
    assert "bg" in group_ids
    assert any(state.truth_column == "truth_sg_x_ppm" for state in plot_config.overview_states)
    assert any(state.truth_column == "truth_sa_x_ppm" for state in plot_config.overview_states)
    assert "bg_x_degh" in plot_config.truth_keys_to_hide
    assert "ba_x_mgal" in plot_config.truth_keys_to_hide
    assert "sg_x_ppm" in plot_config.truth_keys_to_hide
    assert "sa_x_ppm" in plot_config.truth_keys_to_hide


def test_fixed_gnss_lever_truth_contract():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_ins_gnss_outage_no_odo_nhc_lever_truth")

    cfg, metadata = module.build_run_config(base_cfg, output_dir, fix_gnss_lever_truth=True)
    fusion = cfg["fusion"]
    noise = fusion["noise"]
    init_cfg = fusion["init"]
    ablation = fusion["ablation"]
    runtime_phases = fusion["runtime_phases"]

    assert metadata["case_id"] == "data2_baseline_ins_gnss_outage_no_odo_nhc_gnss_lever_truth_fixed"
    assert metadata["fix_gnss_lever_truth"] is True
    assert ablation["disable_gnss_lever_arm"] is True
    assert noise["sigma_gnss_lever_arm"] == 0.0
    assert noise["sigma_gnss_lever_arm_vec"] == [0.0, 0.0, 0.0]
    assert init_cfg["std_gnss_lever_arm"] == [0.0, 0.0, 0.0]
    assert init_cfg["P0_diag"][28:31] == [0.0, 0.0, 0.0]
    assert all(phase["ablation"]["disable_gnss_lever_arm"] is True for phase in runtime_phases)


def test_custom_phase3_outage_duration_updates_schedule_for_fixed_truth_mode():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_ins_gnss_outage_no_odo_nhc_lever_truth_off180")

    cfg, metadata = module.build_run_config(
        base_cfg,
        output_dir,
        fix_gnss_lever_truth=True,
        phase3_gnss_on=90.0,
        phase3_gnss_off=180.0,
    )
    enabled_windows = cfg["fusion"]["gnss_schedule"]["enabled_windows"]

    assert metadata["fix_gnss_lever_truth"] is True
    assert metadata["phase3_gnss_on_s"] == 90.0
    assert metadata["phase3_gnss_off_s"] == 180.0
    assert enabled_windows[0] == {"start_time": 528076.0, "end_time": 528866.0}
    assert enabled_windows[1] == {"start_time": 529046.0, "end_time": 529136.0}
    assert metadata["gnss_off_windows"][0] == [528866.0, 529046.0]
    assert metadata["gnss_off_windows"][1] == [529136.0, 529316.0]


def test_bias_override_contract_for_fixed_gnss_lever_mode():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_ins_gnss_outage_no_odo_nhc_bias_override")

    cfg, metadata = module.build_run_config(
        base_cfg,
        output_dir,
        fix_gnss_lever_truth=True,
        sigma_bg_degh_override=OVERRIDE_SIGMA_BG_DEGH,
        sigma_ba_mgal_override=OVERRIDE_SIGMA_BA_MGAL,
    )
    fusion = cfg["fusion"]
    noise = fusion["noise"]
    init_cfg = fusion["init"]

    assert metadata["fix_gnss_lever_truth"] is True
    assert metadata["sigma_bg_degh_override"] == OVERRIDE_SIGMA_BG_DEGH
    assert metadata["sigma_ba_mgal_override"] == OVERRIDE_SIGMA_BA_MGAL
    assert noise["sigma_bg"] == OVERRIDE_SIGMA_BG
    assert noise["sigma_ba"] == OVERRIDE_SIGMA_BA
    assert noise["sigma_bg_vec"] == [OVERRIDE_SIGMA_BG] * 3
    assert noise["sigma_ba_vec"] == [OVERRIDE_SIGMA_BA] * 3
    assert init_cfg["P0_diag"][12:15] == [OVERRIDE_SIGMA_BG**2] * 3
    assert init_cfg["P0_diag"][9:12] == [OVERRIDE_SIGMA_BA**2] * 3


def test_decoupled_bias_process_and_init_override_contract_for_fixed_gnss_lever_mode():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_ins_gnss_outage_no_odo_nhc_bias_process_init_override")

    cfg, metadata = module.build_run_config(
        base_cfg,
        output_dir,
        fix_gnss_lever_truth=True,
        process_sigma_bg_degh_override=PROCESS_SIGMA_BG_DEGH,
        process_sigma_ba_mgal_override=PROCESS_SIGMA_BA_MGAL,
        init_std_bg_degh_override=INIT_STD_BG_DEGH,
        init_std_ba_mgal_override=INIT_STD_BA_MGAL,
    )
    fusion = cfg["fusion"]
    noise = fusion["noise"]
    init_cfg = fusion["init"]

    assert metadata["fix_gnss_lever_truth"] is True
    assert metadata["process_sigma_bg_degh_override"] == PROCESS_SIGMA_BG_DEGH
    assert metadata["process_sigma_ba_mgal_override"] == PROCESS_SIGMA_BA_MGAL
    assert metadata["init_std_bg_degh_override"] == INIT_STD_BG_DEGH
    assert metadata["init_std_ba_mgal_override"] == INIT_STD_BA_MGAL
    assert noise["sigma_bg"] == PROCESS_SIGMA_BG
    assert noise["sigma_ba"] == PROCESS_SIGMA_BA
    assert noise["sigma_bg_vec"] == [PROCESS_SIGMA_BG] * 3
    assert noise["sigma_ba_vec"] == [PROCESS_SIGMA_BA] * 3
    assert init_cfg["P0_diag"][12:15] == [INIT_STD_BG**2] * 3
    assert init_cfg["P0_diag"][9:12] == [INIT_STD_BA**2] * 3


def test_phase3_only_bias_process_override_contract_for_fixed_gnss_lever_mode():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_ins_gnss_outage_no_odo_nhc_phase3_bias_process_override")

    cfg, metadata = module.build_run_config(
        base_cfg,
        output_dir,
        fix_gnss_lever_truth=True,
        process_sigma_bg_degh_override=PROCESS_SIGMA_BG_DEGH,
        process_sigma_ba_mgal_override=PROCESS_SIGMA_BA_MGAL,
        init_std_bg_degh_override=2.0,
        init_std_ba_mgal_override=100.0,
        phase3_process_sigma_bg_degh_override=PHASE3_PROCESS_SIGMA_BG_DEGH,
        phase3_process_sigma_ba_mgal_override=PHASE3_PROCESS_SIGMA_BA_MGAL,
    )
    fusion = cfg["fusion"]
    noise = fusion["noise"]
    runtime_phases = fusion["runtime_phases"]

    assert metadata["fix_gnss_lever_truth"] is True
    assert metadata["phase3_process_sigma_bg_degh_override"] == PHASE3_PROCESS_SIGMA_BG_DEGH
    assert metadata["phase3_process_sigma_ba_mgal_override"] == PHASE3_PROCESS_SIGMA_BA_MGAL
    assert noise["sigma_bg"] == PROCESS_SIGMA_BG
    assert noise["sigma_ba"] == PROCESS_SIGMA_BA
    assert "noise" not in runtime_phases[0]
    assert "noise" not in runtime_phases[1]
    assert runtime_phases[2]["noise"]["sigma_bg"] == PHASE3_PROCESS_SIGMA_BG
    assert runtime_phases[2]["noise"]["sigma_ba"] == PHASE3_PROCESS_SIGMA_BA
    assert runtime_phases[2]["noise"]["sigma_bg_vec"] == [PHASE3_PROCESS_SIGMA_BG] * 3
    assert runtime_phases[2]["noise"]["sigma_ba_vec"] == [PHASE3_PROCESS_SIGMA_BA] * 3


def test_decoupled_scale_factor_process_and_init_override_contract_for_fixed_gnss_lever_mode():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_ins_gnss_outage_no_odo_nhc_scale_process_init_override")

    cfg, metadata = module.build_run_config(
        base_cfg,
        output_dir,
        fix_gnss_lever_truth=True,
        process_sigma_sg_ppm_override=PROCESS_SIGMA_SG_PPM,
        process_sigma_sa_ppm_override=PROCESS_SIGMA_SA_PPM,
        init_std_sg_ppm_override=INIT_STD_SG_PPM,
        init_std_sa_ppm_override=INIT_STD_SA_PPM,
    )
    fusion = cfg["fusion"]
    noise = fusion["noise"]
    init_cfg = fusion["init"]

    assert metadata["fix_gnss_lever_truth"] is True
    assert metadata["process_sigma_sg_ppm_override"] == PROCESS_SIGMA_SG_PPM
    assert metadata["process_sigma_sa_ppm_override"] == PROCESS_SIGMA_SA_PPM
    assert metadata["init_std_sg_ppm_override"] == INIT_STD_SG_PPM
    assert metadata["init_std_sa_ppm_override"] == INIT_STD_SA_PPM
    assert noise["sigma_sg"] == PROCESS_SIGMA_SG
    assert noise["sigma_sa"] == PROCESS_SIGMA_SA
    assert noise["sigma_sg_vec"] == [PROCESS_SIGMA_SG] * 3
    assert noise["sigma_sa_vec"] == [PROCESS_SIGMA_SA] * 3
    assert init_cfg["P0_diag"][15:18] == [INIT_STD_SG**2] * 3
    assert init_cfg["P0_diag"][18:21] == [INIT_STD_SA**2] * 3


def test_phase12_bias_scale_process_override_contract_for_fixed_gnss_lever_mode():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_ins_gnss_outage_no_odo_nhc_phase12_noise_override")

    cfg, metadata = module.build_run_config(
        base_cfg,
        output_dir,
        fix_gnss_lever_truth=True,
        phase12_process_sigma_bg_degh_override=PHASE12_PROCESS_SIGMA_BG_DEGH,
        phase12_process_sigma_ba_mgal_override=PHASE12_PROCESS_SIGMA_BA_MGAL,
        phase12_process_sigma_sg_ppm_override=PHASE12_PROCESS_SIGMA_SG_PPM,
        phase12_process_sigma_sa_ppm_override=PHASE12_PROCESS_SIGMA_SA_PPM,
    )
    runtime_phases = cfg["fusion"]["runtime_phases"]

    assert metadata["phase12_process_sigma_bg_degh_override"] == PHASE12_PROCESS_SIGMA_BG_DEGH
    assert metadata["phase12_process_sigma_ba_mgal_override"] == PHASE12_PROCESS_SIGMA_BA_MGAL
    assert metadata["phase12_process_sigma_sg_ppm_override"] == PHASE12_PROCESS_SIGMA_SG_PPM
    assert metadata["phase12_process_sigma_sa_ppm_override"] == PHASE12_PROCESS_SIGMA_SA_PPM
    for phase_idx in (0, 1):
        assert runtime_phases[phase_idx]["noise"]["sigma_bg"] == PHASE12_PROCESS_SIGMA_BG
        assert runtime_phases[phase_idx]["noise"]["sigma_ba"] == PHASE12_PROCESS_SIGMA_BA
        assert runtime_phases[phase_idx]["noise"]["sigma_sg"] == PHASE12_PROCESS_SIGMA_SG
        assert runtime_phases[phase_idx]["noise"]["sigma_sa"] == PHASE12_PROCESS_SIGMA_SA
        assert runtime_phases[phase_idx]["noise"]["sigma_bg_vec"] == [PHASE12_PROCESS_SIGMA_BG] * 3
        assert runtime_phases[phase_idx]["noise"]["sigma_ba_vec"] == [PHASE12_PROCESS_SIGMA_BA] * 3
        assert runtime_phases[phase_idx]["noise"]["sigma_sg_vec"] == [PHASE12_PROCESS_SIGMA_SG] * 3
        assert runtime_phases[phase_idx]["noise"]["sigma_sa_vec"] == [PHASE12_PROCESS_SIGMA_SA] * 3
    assert "noise" not in runtime_phases[2]


def test_phase3_navigation_white_noise_override_contract_for_fixed_gnss_lever_mode():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_ins_gnss_outage_no_odo_nhc_phase3_nav_noise_override")

    cfg, metadata = module.build_run_config(
        base_cfg,
        output_dir,
        fix_gnss_lever_truth=True,
        phase3_sigma_acc_mps_sqrt_hr_override=PHASE3_SIGMA_ACC_MPS_SQRT_HR,
        phase3_sigma_gyro_deg_sqrt_hr_override=PHASE3_SIGMA_GYRO_DEG_SQRT_HR,
    )
    runtime_phases = cfg["fusion"]["runtime_phases"]

    assert metadata["phase3_sigma_acc_mps_sqrt_hr_override"] == PHASE3_SIGMA_ACC_MPS_SQRT_HR
    assert metadata["phase3_sigma_gyro_deg_sqrt_hr_override"] == PHASE3_SIGMA_GYRO_DEG_SQRT_HR
    assert "noise" not in runtime_phases[0]
    assert "noise" not in runtime_phases[1]
    assert runtime_phases[2]["noise"]["sigma_acc"] == PHASE3_SIGMA_ACC
    assert runtime_phases[2]["noise"]["sigma_gyro"] == PHASE3_SIGMA_GYRO


def test_enable_gnss_update_debug_sets_case_relative_output_path():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_ins_gnss_outage_no_odo_nhc_debug")

    cfg, metadata = module.build_run_config(
        base_cfg,
        output_dir,
        fix_gnss_lever_truth=True,
        enable_gnss_update_debug=True,
    )
    fusion = cfg["fusion"]

    expected_rel = (
        "output/_tmp_baseline_ins_gnss_outage_no_odo_nhc_debug/artifacts/cases/"
        "data2_baseline_ins_gnss_outage_no_odo_nhc_gnss_lever_truth_fixed/"
        "gnss_updates_data2_baseline_ins_gnss_outage_no_odo_nhc_gnss_lever_truth_fixed.csv"
    )
    assert metadata["enable_gnss_update_debug"] is True
    assert metadata["gnss_update_debug_output_path"] == expected_rel
    assert fusion["gnss_update_debug_output_path"] == expected_rel
