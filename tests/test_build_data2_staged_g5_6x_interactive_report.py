import importlib.util
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "build_data2_staged_g5_6x_interactive_report.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing report script: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("build_data2_staged_g5_6x_interactive_report", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_defaults_match_staged_g5_6x_report_contract():
    module = load_module()

    assert module.EXP_ID_DEFAULT == "EXP-20260326-data2-staged-g5-6x-interactive-report-r1"
    assert module.INPUT_DIR_DEFAULT == Path("output/data2_staged_g5_no_imu_scale_r2_20260325")
    assert module.OUTPUT_HTML_DEFAULT == (
        Path("output/data2_staged_g5_no_imu_scale_r2_20260325") / "interactive" / "staged_g5_6x_report.html"
    )
    assert module.OUTPUT_MANIFEST_DEFAULT == (
        Path("output/data2_staged_g5_no_imu_scale_r2_20260325")
        / "interactive"
        / "staged_g5_6x_report_manifest.json"
    )
    assert module.MAIN_CASE_ID_DEFAULT == "staged_g5_odo_nhc_noise_6x"
    assert module.CONTROL_CASE_ID_DEFAULT == "staged_g5_odo_nhc_noise_6x_phase3_ins_only"
    assert module.GRID_COLUMNS == 3


def test_non_pva_state_specs_keep_only_bias_and_calibration_change_signals():
    module = load_module()

    assert len(module.NON_PVA_STATE_SPECS) == 16
    assert [spec.key for spec in module.NON_PVA_STATE_SPECS] == [
        "ba_x_mgal",
        "ba_y_mgal",
        "ba_z_mgal",
        "bg_x_degh",
        "bg_y_degh",
        "bg_z_degh",
        "odo_scale_state",
        "mounting_roll_deg",
        "mounting_pitch_deg",
        "mounting_yaw_deg",
        "odo_lever_x_m",
        "odo_lever_y_m",
        "odo_lever_z_m",
        "gnss_lever_x_m",
        "gnss_lever_y_m",
        "gnss_lever_z_m",
    ]


def test_non_pva_state_specs_render_state_change_curves_with_consistent_semantics():
    module = load_module()

    spec_by_key = {spec.key: spec for spec in module.NON_PVA_STATE_SPECS}

    assert spec_by_key["ba_x_mgal"].plot_mode == "value"
    assert spec_by_key["ba_x_mgal"].show_truth is False
    assert spec_by_key["bg_z_degh"].plot_mode == "value"
    assert spec_by_key["bg_z_degh"].show_truth is False
    assert "sg_x_ppm" not in spec_by_key
    assert "sa_x_ppm" not in spec_by_key
    assert spec_by_key["odo_scale_state"].plot_mode == "delta"
    assert spec_by_key["odo_scale_state"].show_truth is False
    assert spec_by_key["mounting_yaw_deg"].plot_mode == "delta"
    assert spec_by_key["mounting_yaw_deg"].show_truth is False
    assert spec_by_key["odo_lever_y_m"].plot_mode == "delta"
    assert spec_by_key["odo_lever_y_m"].show_truth is False
    assert spec_by_key["gnss_lever_y_m"].plot_mode == "delta"
    assert spec_by_key["gnss_lever_y_m"].show_truth is False


def test_pva_and_control_sections_keep_only_three_error_groups():
    module = load_module()

    assert [group.group_id for group in module.PVA_ERROR_GROUP_SPECS] == ["position", "velocity", "attitude"]
    assert [group.group_id for group in module.CONTROL_PVA_GROUP_SPECS] == ["position", "velocity", "attitude"]
    assert [state.key for state in module.PVA_ERROR_GROUP_SPECS[0].states] == ["p_n_m", "p_e_m", "p_u_m"]
    assert [state.key for state in module.PVA_ERROR_GROUP_SPECS[1].states] == ["v_n_mps", "v_e_mps", "v_u_mps"]
    assert [state.key for state in module.PVA_ERROR_GROUP_SPECS[2].states] == ["roll_deg", "pitch_deg", "yaw_deg"]


def test_vehicle_truth_override_uses_working_yaw_0p84():
    module = load_module()

    source = {
        "sources": {
            "mounting_total_truth": {
                "value_deg": {"roll": 0.0, "pitch": 0.36, "yaw": 1.37},
            }
        }
    }
    overridden = module.build_vehicle_truth_reference(source)

    assert overridden is not source
    assert overridden["sources"]["mounting_total_truth"]["value_deg"] == {"roll": 0.0, "pitch": 0.36, "yaw": 0.84}
    assert source["sources"]["mounting_total_truth"]["value_deg"]["yaw"] == 1.37


def test_context_loader_reads_manifest_and_case_artifacts():
    module = load_module()

    context = module.load_experiment_context(
        REPO_ROOT / module.INPUT_DIR_DEFAULT,
        module.MAIN_CASE_ID_DEFAULT,
        module.CONTROL_CASE_ID_DEFAULT,
    )

    assert context.phase3_start_time == 528776.0
    assert context.phase2_end_time == 528776.0
    assert context.main_case.case_id == module.MAIN_CASE_ID_DEFAULT
    assert context.control_case.case_id == module.CONTROL_CASE_ID_DEFAULT
    assert context.main_case.all_states_path.exists()
    assert context.control_case.all_states_path.exists()
    assert context.main_case.sol_path.exists()
    assert context.main_case.state_series_path.exists()
    assert context.control_case.sol_path.exists()
    assert context.control_case.state_series_path.exists()
    manifest = json.loads(context.manifest_path.read_text(encoding="utf-8"))
    assert manifest["cases"][-1] == module.CONTROL_CASE_ID_DEFAULT


def test_state_signal_figure_uses_state_value_titles_for_non_pva_signals():
    module = load_module()
    context = module.load_experiment_context(
        REPO_ROOT / module.INPUT_DIR_DEFAULT,
        module.MAIN_CASE_ID_DEFAULT,
        module.CONTROL_CASE_ID_DEFAULT,
    )
    states = module.load_all_states(context.main_case, context.phase1_start_time)
    spec_by_key = {spec.key: spec for spec in module.NON_PVA_STATE_SPECS}

    bias_fig = module.build_state_signal_figure(states, spec_by_key["ba_x_mgal"], context, context.main_case.color, 300)
    odo_fig = module.build_state_signal_figure(
        states,
        spec_by_key["odo_scale_state"],
        context,
        context.main_case.color,
        300,
    )

    assert [trace.name for trace in bias_fig.data] == ["estimate"]
    assert [trace.name for trace in odo_fig.data] == ["delta"]
    assert "Δ" in odo_fig.layout.title.text
    mount_fig = module.build_state_signal_figure(
        states,
        spec_by_key["mounting_yaw_deg"],
        context,
        context.main_case.color,
        300,
    )
    assert "Δ" in mount_fig.layout.title.text
    assert abs(float(odo_fig.data[0].y[0])) < 1e-12
    assert abs(float(mount_fig.data[0].y[0])) < 1e-12


def test_state_signal_figure_avoids_si_prefixes_and_uses_scientific_notation_contract():
    module = load_module()
    context = module.load_experiment_context(
        REPO_ROOT / module.INPUT_DIR_DEFAULT,
        module.MAIN_CASE_ID_DEFAULT,
        module.CONTROL_CASE_ID_DEFAULT,
    )
    states = module.load_all_states(context.main_case, context.phase1_start_time)
    spec_by_key = {spec.key: spec for spec in module.NON_PVA_STATE_SPECS}

    fig = module.build_state_signal_figure(
        states,
        spec_by_key["mounting_yaw_deg"],
        context,
        context.main_case.color,
        300,
    )

    assert fig.layout.yaxis.tickformat == ".6g"
    assert fig.layout.yaxis.exponentformat == "e"
    assert fig.layout.yaxis.showexponent == "all"
    assert ":%{y:.6f}" not in fig.data[0].hovertemplate
    assert "%{y:.6g}" in fig.data[0].hovertemplate


def test_delta_plots_keep_raw_non_pva_initial_values_and_only_normalize_in_rendering():
    module = load_module()
    context = module.load_experiment_context(
        REPO_ROOT / module.INPUT_DIR_DEFAULT,
        module.MAIN_CASE_ID_DEFAULT,
        module.CONTROL_CASE_ID_DEFAULT,
    )
    states = module.load_all_states(context.main_case, context.phase1_start_time)
    spec_by_key = {spec.key: spec for spec in module.NON_PVA_STATE_SPECS}

    assert float(states["odo_scale_state"].iloc[0]) == 1.0
    assert float(states["mounting_pitch_deg"].iloc[0]) == 0.36
    assert float(states["mounting_yaw_deg"].iloc[0]) == 1.37
    assert float(states["odo_lever_x_m"].iloc[0]) == 0.2
    assert float(states["odo_lever_y_m"].iloc[0]) == -1.0
    assert float(states["odo_lever_z_m"].iloc[0]) == 0.6
    assert float(states["gnss_lever_x_m"].iloc[0]) == 0.15
    assert float(states["gnss_lever_y_m"].iloc[0]) == -0.22
    assert float(states["gnss_lever_z_m"].iloc[0]) == -1.15
    assert float(states["mounting_state_pitch_deg"].iloc[0]) == 0.36
    assert float(states["mounting_state_yaw_deg"].iloc[0]) == -0.01

    odo_fig = module.build_state_signal_figure(
        states,
        spec_by_key["odo_scale_state"],
        context,
        context.main_case.color,
        300,
    )
    mount_fig = module.build_state_signal_figure(
        states,
        spec_by_key["mounting_yaw_deg"],
        context,
        context.main_case.color,
        300,
    )

    assert abs(float(odo_fig.data[0].y[0])) < 1e-12
    assert abs(float(mount_fig.data[0].y[0])) < 1e-12


def test_rendered_html_uses_compact_ieee_latex_style_and_removes_mount_state_section_copy():
    module = load_module()
    context = module.load_experiment_context(
        REPO_ROOT / module.INPUT_DIR_DEFAULT,
        module.MAIN_CASE_ID_DEFAULT,
        module.CONTROL_CASE_ID_DEFAULT,
    )
    main_states = module.decimate_df(module.load_all_states(context.main_case, context.phase1_start_time), 160)
    main_vehicle = module.build_vehicle_error_frame(context, context.main_case, 160)
    control_states = module.load_all_states(context.control_case, context.phase1_start_time)
    control_states_phase3 = module.decimate_df(module.select_window(control_states, context.phase3_start_time), 160)

    html = module.render_html_page(context, main_states, main_vehicle, control_states_phase3, 160)

    assert "Times New Roman" in html
    assert "--ieee-blue" in html
    assert "gap: 6px;" in html
    assert "box-shadow: none;" in html
    assert "16 维" in html
    assert "mount_state" not in html
    assert "非 PVA 状态变化" in html
    assert "ODO/NHC 标定状态统一按相对初值变化绘制" in html
    assert "sg/sa 已移除" in html
