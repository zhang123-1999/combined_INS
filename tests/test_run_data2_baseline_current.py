import importlib.util
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_baseline_current.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing canonical baseline runner: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_baseline_current", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_defaults_match_canonical_baseline_contract():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_baseline_current_contract")

    assert module.EXP_ID_DEFAULT == "EXP-20260326-data2-baseline-current-r1"
    assert module.OUTPUT_DIR_DEFAULT == Path("output/data2_baseline_current")
    assert module.BASE_CONFIG_DEFAULT == Path("config_data2_baseline_eskf.yaml")
    assert module.BASELINE_CASE_ID == "data2_baseline_current"

    cfg, metadata = module.build_run_config(base_cfg, output_dir)

    assert cfg["fusion"]["output_path"] == "output/_tmp_baseline_current_contract/SOL_data2_baseline_current.txt"
    assert (
        cfg["fusion"]["state_series_output_path"]
        == "output/_tmp_baseline_current_contract/state_series_data2_baseline_current.csv"
    )
    assert metadata["case_id"] == "data2_baseline_current"
    assert metadata["phase1_window"] == [528076.0, 528276.0]
    assert metadata["phase2_seed_window"] == [528276.0, 528276.02]
    assert metadata["phase2_main_window"] == [528276.02, 528776.0]
    assert metadata["phase3_window"] == [528776.0, 530488.9]
    assert metadata["gnss_off_windows"][0] == [528866.0, 528956.0]
