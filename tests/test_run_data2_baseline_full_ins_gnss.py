import importlib.util
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml


MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_baseline_full_ins_gnss.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing full INS/GNSS baseline runner: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_baseline_full_ins_gnss", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_defaults_match_full_ins_gnss_reset_contract():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_full_ins_gnss_contract")

    assert module.EXP_ID_DEFAULT == "EXP-20260327-data2-baseline-full-ins-gnss-r1"
    assert module.OUTPUT_DIR_DEFAULT == Path("output/data2_baseline_full_ins_gnss")
    assert module.BASE_CONFIG_DEFAULT == Path("config_data2_baseline_eskf.yaml")
    assert module.CASE_ID_DEFAULT == "data2_baseline_full_ins_gnss"

    cfg, metadata = module.build_run_config(base_cfg, output_dir)

    assert cfg["fusion"]["output_path"] == "output/_tmp_baseline_full_ins_gnss_contract/SOL_data2_baseline_full_ins_gnss.txt"
    assert (
        cfg["fusion"]["state_series_output_path"]
        == "output/_tmp_baseline_full_ins_gnss_contract/state_series_data2_baseline_full_ins_gnss.csv"
    )
    assert cfg["fusion"]["constraints"]["enable_odo"] is False
    assert cfg["fusion"]["constraints"]["enable_nhc"] is False
    assert cfg["fusion"]["runtime_phases"] == []
    assert cfg["fusion"]["gnss_schedule"]["enabled"] is True
    assert cfg["fusion"]["gnss_schedule"]["enabled_windows"] == [
        {"start_time": 528076.0, "end_time": 530488.9}
    ]
    assert metadata["case_id"] == "data2_baseline_full_ins_gnss"
    assert metadata["full_window"] == [528076.0, 530488.9]
    assert metadata["gnss_on_windows"] == [[528076.0, 530488.9]]
    assert metadata["gnss_off_windows"] == []


def test_process_noise_scale_contract_for_bias_and_scale_states():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_full_ins_gnss_process_scale_contract")

    cfg, metadata = module.build_run_config(
        base_cfg,
        output_dir,
        case_id="data2_baseline_full_ins_gnss_q4x",
        case_label="data2 baseline full INS/GNSS q4x",
        process_noise_scale_rel=4.0,
    )

    base_noise = base_cfg["fusion"]["noise"]
    noise = cfg["fusion"]["noise"]

    assert cfg["fusion"]["output_path"] == "output/_tmp_baseline_full_ins_gnss_process_scale_contract/SOL_data2_baseline_full_ins_gnss_q4x.txt"
    assert (
        cfg["fusion"]["state_series_output_path"]
        == "output/_tmp_baseline_full_ins_gnss_process_scale_contract/state_series_data2_baseline_full_ins_gnss_q4x.csv"
    )
    assert metadata["case_id"] == "data2_baseline_full_ins_gnss_q4x"
    assert metadata["case_label"] == "data2 baseline full INS/GNSS q4x"
    assert metadata["process_noise_scale_rel"] == 4.0
    assert noise["sigma_ba"] == base_noise["sigma_ba"] * 4.0
    assert noise["sigma_bg"] == base_noise["sigma_bg"] * 4.0
    assert noise["sigma_sg"] == base_noise["sigma_sg"] * 4.0
    assert noise["sigma_sa"] == base_noise["sigma_sa"] * 4.0
    assert noise["sigma_ba_vec"] == [base_noise["sigma_ba"] * 4.0] * 3
    assert noise["sigma_bg_vec"] == [base_noise["sigma_bg"] * 4.0] * 3
    assert noise["sigma_sg_vec"] == [base_noise["sigma_sg"] * 4.0] * 3
    assert noise["sigma_sa_vec"] == [base_noise["sigma_sa"] * 4.0] * 3


def test_fixed_gnss_lever_truth_contract():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_full_ins_gnss_fix_gnss_lever_truth")

    cfg, metadata = module.build_run_config(
        base_cfg,
        output_dir,
        case_id="data2_baseline_full_ins_gnss_fix_gnss_lever_truth",
        case_label="data2 baseline full INS/GNSS with GNSS lever fixed to truth",
        fix_gnss_lever_truth=True,
    )

    fusion = cfg["fusion"]
    noise = fusion["noise"]
    init_cfg = fusion["init"]
    ablation = fusion["ablation"]

    assert metadata["fix_gnss_lever_truth"] is True
    assert ablation["disable_gnss_lever_arm"] is True
    assert noise["sigma_gnss_lever_arm"] == 0.0
    assert noise["sigma_gnss_lever_arm_vec"] == [0.0, 0.0, 0.0]
    assert init_cfg["std_gnss_lever_arm"] == [0.0, 0.0, 0.0]
    assert init_cfg["P0_diag"][28:31] == [0.0, 0.0, 0.0]
    assert init_cfg["gnss_lever_arm0"] == [0.15, -0.22, -1.15]
