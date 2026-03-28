import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml
from scripts.analysis.run_data2_state_sanity_matrix import build_truth_reference

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_staged_g5_phase3_entry_pos_only.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing phase3 entry position-only runner: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location(
        "run_data2_staged_g5_phase3_entry_pos_only",
        MODULE_PATH,
    )
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
    )


def test_phase3_entry_position_only_case_contract():
    module = load_module()
    assert module.BASE_CONFIG_DEFAULT == Path("config_data2_research_seed_eskf.yaml")
    assert module.EXP_ID_DEFAULT == "EXP-20260326-data2-staged-g5-phase3-entry-pos-only-r1"
    assert module.OUTPUT_DIR_DEFAULT == Path("output/data2_staged_g5_phase3_entry_pos_only_r1_20260326")
    assert module.CASE_ID_DEFAULT == "staged_g5_odo_nhc_noise_6x_phase3_entry_pos_only"
    assert module.REFERENCE_CASE_ID == "staged_g5_odo_nhc_noise_6x_phase3_ins_only"

    base_cfg = load_yaml(REPO_ROOT / "config_data2_research_seed_eskf.yaml")
    truth_reference = build_truth_reference(base_cfg)
    args = build_args(module)
    case_dir = REPO_ROOT / "output" / "_tmp_phase3_entry_pos_only"

    cfg, metadata = module.build_case_config(base_cfg, truth_reference, case_dir, args)
    fusion = cfg["fusion"]
    runtime_phases = fusion["runtime_phases"]

    assert fusion["output_path"].endswith("SOL_staged_g5_odo_nhc_noise_6x_phase3_entry_pos_only.txt")
    assert fusion["state_series_output_path"].endswith(
        "state_series_staged_g5_odo_nhc_noise_6x_phase3_entry_pos_only.csv"
    )
    assert len(runtime_phases) == 4

    phase1, phase2, phase3, phase3_entry = runtime_phases
    assert phase1["constraints"]["enable_odo"] is False
    assert phase1["constraints"]["enable_nhc"] is False
    assert phase2["constraints"]["enable_odo"] is True
    assert phase2["constraints"]["enable_nhc"] is True
    assert phase3["constraints"]["enable_odo"] is False
    assert phase3["constraints"]["enable_nhc"] is False
    assert phase3["ablation"]["disable_gnss_lever_arm"] is True
    assert phase3["ablation"]["disable_odo_scale"] is True
    assert phase3["ablation"]["disable_mounting"] is True
    assert phase3["ablation"]["disable_odo_lever_arm"] is True

    assert phase3_entry["name"] == "phase3_entry_gnss_pos_position_only"
    assert phase3_entry["start_time"] == metadata["phase2_end_time"]
    assert phase3_entry["end_time"] == metadata["gnss_off_windows"][0][0]
    assert phase3_entry["constraints"]["gnss_pos_update_mode"] == "position_only"
    assert "ablation" not in phase3_entry
    assert "noise" not in phase3_entry

    assert metadata["case_id"] == "staged_g5_odo_nhc_noise_6x_phase3_entry_pos_only"
    assert metadata["reference_case_id"] == "staged_g5_odo_nhc_noise_6x_phase3_ins_only"
    assert metadata["phase3_constraint_mode"] == "ins_gnss_outage_control_phase3_entry_pos_only"
    assert metadata["phase3_entry_update_mode"] == "position_only"
    assert metadata["phase3_entry_window"] == (528776.0, 528866.0)
    assert metadata["phase1_end_time"] == 528276.0
    assert metadata["phase2_end_time"] == 528776.0
    assert metadata["gnss_off_windows"][0] == (528866.0, 528956.0)
