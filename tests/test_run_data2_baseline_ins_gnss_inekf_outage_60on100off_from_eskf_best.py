import importlib.util
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml


MODULE_PATH = (
    REPO_ROOT
    / "scripts"
    / "analysis"
    / "run_data2_baseline_ins_gnss_inekf_outage_60on100off_from_eskf_best.py"
)


def load_module():
    assert MODULE_PATH.exists(), f"missing pure INS/GNSS InEKF outage runner: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location(
        "run_data2_baseline_ins_gnss_inekf_outage_60on100off_from_eskf_best",
        MODULE_PATH,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_defaults_match_pure_ins_gnss_inekf_60on100off_replay_contract():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_ins_gnss_eskf_outage_60on100off_best.yaml")
    output_dir = Path("output/_tmp_data2_baseline_ins_gnss_inekf_outage_60on100off_from_eskf_best_contract")

    assert module.EXP_ID_DEFAULT == "EXP-20260407-data2-baseline-ins-gnss-inekf-outage-60on100off-from-eskf-best-r1"
    assert module.OUTPUT_DIR_DEFAULT == Path("output/data2_baseline_ins_gnss_inekf_outage_60on100off_from_eskf_best_r1")
    assert module.BASE_CONFIG_DEFAULT == Path("config_data2_baseline_ins_gnss_eskf_outage_60on100off_best.yaml")
    assert module.BASELINE_CASE_ID == "data2_baseline_ins_gnss_inekf_outage_60on100off_from_eskf_best"

    cfg, metadata = module.build_run_config(base_cfg, output_dir)

    assert (
        cfg["fusion"]["output_path"]
        == "output/_tmp_data2_baseline_ins_gnss_inekf_outage_60on100off_from_eskf_best_contract/SOL_data2_baseline_ins_gnss_inekf_outage_60on100off_from_eskf_best.txt"
    )
    assert (
        cfg["fusion"]["state_series_output_path"]
        == "output/_tmp_data2_baseline_ins_gnss_inekf_outage_60on100off_from_eskf_best_contract/state_series_data2_baseline_ins_gnss_inekf_outage_60on100off_from_eskf_best.csv"
    )
    assert cfg["fusion"]["constraints"]["enable_odo"] is False
    assert cfg["fusion"]["constraints"]["enable_nhc"] is False
    assert cfg["fusion"]["ablation"]["disable_gnss_lever_arm"] is True
    assert cfg["fusion"]["inekf"]["enable"] is True
    assert metadata["case_id"] == "data2_baseline_ins_gnss_inekf_outage_60on100off_from_eskf_best"
    assert metadata["phase1_window"] == [528076.0, 528276.0]
    assert metadata["phase2_window"] == [528276.0, 528776.0]
    assert metadata["phase3_window"] == [528776.0, 530488.9]
    assert metadata["gnss_off_windows"][0] == [528836.0, 528936.0]
    assert metadata["gnss_off_windows"][1] == [528996.0, 529096.0]
