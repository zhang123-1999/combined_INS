import importlib.util
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

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_staged_g5_no_imu_scale.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing experiment script: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_staged_g5_no_imu_scale", MODULE_PATH)
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


def test_case_configs_match_staged_g5_plan():
    module = load_module()
    assert module.BASE_CONFIG_DEFAULT == Path("config_data2_research_seed_eskf.yaml")
    base_cfg = load_yaml(REPO_ROOT / "config_data2_research_seed_eskf.yaml")
    truth_reference = build_truth_reference(base_cfg)
    case_root = REPO_ROOT / "output" / "_tmp_staged_g5_case_config_tests"
    args = build_args(module)

    assert module.EXP_ID_DEFAULT == "EXP-20260325-data2-staged-g5-no-imu-scale-r2"
    assert module.OUTPUT_DIR_DEFAULT == Path("output/data2_staged_g5_no_imu_scale_r2_20260325")
    assert module.PHASE3_GNSS_ON_DEFAULT == 90.0
    assert module.PHASE3_GNSS_OFF_DEFAULT == 90.0

    assert [spec.case_id for spec in module.CASE_SPECS] == [
        "staged_g5_odo_nhc_noise_4x",
        "staged_g5_odo_nhc_noise_5x",
        "staged_g5_odo_nhc_noise_6x",
        "staged_g5_odo_nhc_noise_6x_phase3_ins_only",
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

    g4x = built["staged_g5_odo_nhc_noise_4x"]
    g5x = built["staged_g5_odo_nhc_noise_5x"]
    g6x = built["staged_g5_odo_nhc_noise_6x"]
    g6x_phase3_ins_only = built["staged_g5_odo_nhc_noise_6x_phase3_ins_only"]
    base_constraints = base_cfg["fusion"]["constraints"]

    assert g4x["fusion"]["ablation"]["disable_gyro_scale"] is True
    assert g4x["fusion"]["ablation"]["disable_accel_scale"] is True
    assert g4x["fusion"]["ablation"]["disable_mounting_roll"] is False
    assert g4x["fusion"]["ablation"]["disable_gnss_lever_z"] is False
    assert g4x["fusion"]["init"]["odo_scale"] == 1.0
    assert g4x["fusion"]["init"]["gnss_lever_arm0"] == [
        truth_reference["states"]["gnss_lever_x"]["reference_value"],
        truth_reference["states"]["gnss_lever_y"]["reference_value"],
        truth_reference["states"]["gnss_lever_z"]["reference_value"],
    ]
    assert g4x["fusion"]["constraints"]["sigma_odo"] == base_constraints["sigma_odo"] * 20.0
    assert g4x["fusion"]["constraints"]["sigma_nhc_y"] == base_constraints["sigma_nhc_y"] * 20.0
    assert g4x["fusion"]["constraints"]["sigma_nhc_z"] == base_constraints["sigma_nhc_z"] * 20.0
    assert g5x["fusion"]["constraints"]["sigma_odo"] == base_constraints["sigma_odo"] * 25.0
    assert g5x["fusion"]["constraints"]["sigma_nhc_y"] == base_constraints["sigma_nhc_y"] * 25.0
    assert g5x["fusion"]["constraints"]["sigma_nhc_z"] == base_constraints["sigma_nhc_z"] * 25.0
    assert g6x["fusion"]["constraints"]["sigma_odo"] == base_constraints["sigma_odo"] * 30.0
    assert g6x["fusion"]["constraints"]["sigma_nhc_y"] == base_constraints["sigma_nhc_y"] * 30.0
    assert g6x["fusion"]["constraints"]["sigma_nhc_z"] == base_constraints["sigma_nhc_z"] * 30.0
    assert g6x_phase3_ins_only["fusion"]["constraints"]["sigma_odo"] == base_constraints["sigma_odo"] * 30.0
    assert g6x_phase3_ins_only["fusion"]["constraints"]["sigma_nhc_y"] == base_constraints["sigma_nhc_y"] * 30.0
    assert g6x_phase3_ins_only["fusion"]["constraints"]["sigma_nhc_z"] == base_constraints["sigma_nhc_z"] * 30.0
    assert g4x["fusion"]["init"]["lever_arm0"] == truth_reference["sources"]["odo_lever_truth"]["value_m"]
    assert g6x_phase3_ins_only["fusion"]["init"]["lever_arm0"] == truth_reference["sources"]["odo_lever_truth"]["value_m"]

    phase1, phase2, phase3 = g4x["fusion"]["runtime_phases"]
    assert phase1["constraints"]["enable_odo"] is False
    assert phase1["constraints"]["enable_nhc"] is False
    assert phase1["ablation"]["disable_odo_scale"] is True
    assert phase1["ablation"]["disable_mounting"] is True
    assert phase1["ablation"]["disable_odo_lever_arm"] is True
    assert "noise" not in phase1

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

    _, _, phase3_ins_only = g6x_phase3_ins_only["fusion"]["runtime_phases"]
    assert phase3_ins_only["constraints"]["enable_odo"] is False
    assert phase3_ins_only["constraints"]["enable_nhc"] is False
    assert phase3_ins_only["ablation"]["disable_gnss_lever_arm"] is True
    assert phase3_ins_only["ablation"]["disable_odo_scale"] is True
    assert phase3_ins_only["ablation"]["disable_mounting"] is True
    assert phase3_ins_only["ablation"]["disable_odo_lever_arm"] is True
    assert "noise" not in phase3_ins_only

    assert metadata_map["staged_g5_odo_nhc_noise_4x"]["phase1_end_time"] < metadata_map["staged_g5_odo_nhc_noise_4x"][
        "phase2_end_time"
    ]
    assert metadata_map["staged_g5_odo_nhc_noise_4x"]["gnss_on_windows"]
    assert metadata_map["staged_g5_odo_nhc_noise_4x"]["gnss_off_windows"]
    off_start, off_end = metadata_map["staged_g5_odo_nhc_noise_4x"]["gnss_off_windows"][0]
    assert off_end - off_start == 90.0
    assert off_start == metadata_map["staged_g5_odo_nhc_noise_4x"]["phase2_end_time"] + 90.0
    phase3_on_windows = metadata_map["staged_g5_odo_nhc_noise_4x"]["gnss_on_windows"][1:]
    assert phase3_on_windows
    on_start, on_end = phase3_on_windows[0]
    assert on_end - on_start == 90.0
    assert metadata_map["staged_g5_odo_nhc_noise_6x"]["phase3_constraint_mode"] == "odo_nhc_enabled"
    assert (
        metadata_map["staged_g5_odo_nhc_noise_6x_phase3_ins_only"]["phase3_constraint_mode"]
        == "ins_gnss_outage_control"
    )


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


def test_mainline_plot_config_removes_sg_sa_and_hides_bias_truth():
    module = load_module()

    plot_config = module.build_mainline_plot_config()
    overview_keys = [state.key for state in plot_config.overview_states]
    group_ids = [group.group_id for group in plot_config.group_specs]

    assert "sg_x_ppm" not in overview_keys
    assert "sa_x_ppm" not in overview_keys
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


def test_remove_obsolete_mainline_plot_files_deletes_stale_sg_sa_only(tmp_path):
    module = load_module()
    sg_path = tmp_path / "sg.png"
    sa_path = tmp_path / "sa.png"
    keep_path = tmp_path / "ba.png"
    sg_path.write_text("stale", encoding="utf-8")
    sa_path.write_text("stale", encoding="utf-8")
    keep_path.write_text("keep", encoding="utf-8")

    module.remove_obsolete_mainline_plot_files(tmp_path)

    assert not sg_path.exists()
    assert not sa_path.exists()
    assert keep_path.exists()


def test_plot_state_grid_can_hide_truth_overlay_for_selected_states(tmp_path, monkeypatch):
    module = load_module()
    case_spec = SimpleNamespace(case_id="case", label="case", color="#123456")
    state_spec = module.GROUP_SPECS[3].states[0]
    case_frames = {
        "case": pd.DataFrame(
            {
                "timestamp": [0.0, 1.0],
                state_spec.key: [1.0, 2.0],
                state_spec.truth_column: [0.5, 0.5],
            }
        )
    }
    captured: dict[str, object] = {}
    original_subplots = module.plt.subplots

    def capture_subplots(*args, **kwargs):
        fig, axes = original_subplots(*args, **kwargs)
        captured["axes"] = axes
        return fig, axes

    monkeypatch.setattr(module.plt, "subplots", capture_subplots)
    monkeypatch.setattr(module.plt, "close", lambda fig: None)

    with_truth_path = tmp_path / "with_truth.png"
    module.plot_state_grid(
        case_frames,
        [case_spec],
        (state_spec,),
        with_truth_path,
        "with truth",
        0.25,
        0.75,
        [],
    )
    axes = np.atleast_1d(captured["axes"]).reshape(-1)
    with_truth_labels = [line.get_label() for line in axes[0].lines]
    assert "truth" in with_truth_labels

    hidden_truth_path = tmp_path / "hidden_truth.png"
    module.plot_state_grid(
        case_frames,
        [case_spec],
        (state_spec,),
        hidden_truth_path,
        "hidden truth",
        0.25,
        0.75,
        [],
        truth_keys_to_hide={state_spec.key},
    )
    axes = np.atleast_1d(captured["axes"]).reshape(-1)
    hidden_truth_labels = [line.get_label() for line in axes[0].lines]
    assert "truth" not in hidden_truth_labels
