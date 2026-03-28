import importlib.util
from pathlib import Path
import sys

import math
import pandas as pd


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml


MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_kf_gins_gnss_update_feedback_compare.py"
BASE_CONFIG_PATH = REPO_ROOT / "config_data2_baseline_eskf.yaml"
SOURCE_CFG_PATH = (
    REPO_ROOT
    / "output"
    / "data2_baseline_ins_gnss_outage_no_odo_nhc_r2"
    / "artifacts"
    / "cases"
    / "data2_baseline_ins_gnss_outage_no_odo_nhc"
    / "config_data2_baseline_ins_gnss_outage_no_odo_nhc.yaml"
)


def load_module():
    assert MODULE_PATH.exists(), f"missing GNSS feedback compare runner: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_kf_gins_gnss_update_feedback_compare", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_build_current_compare_config_enforces_full_window_fixed_lever_and_debug_contract():
    module = load_module()
    base_cfg = load_yaml(BASE_CONFIG_PATH)
    output_dir = Path("output/_tmp_kf_gins_gnss_feedback_compare")

    cfg, metadata = module.build_current_compare_config(base_cfg, output_dir)
    fusion = cfg["fusion"]

    assert metadata["compare_limit_updates"] == 500
    assert metadata["fix_gnss_lever_truth"] is True
    assert fusion["constraints"]["enable_odo"] is False
    assert fusion["constraints"]["enable_nhc"] is False
    assert fusion["runtime_phases"] == []
    assert fusion["gnss_schedule"]["enabled_windows"] == [{"start_time": 528076.0, "end_time": 530488.9}]
    assert fusion["ablation"]["disable_gnss_lever_arm"] is True
    assert fusion["gnss_update_debug_output_path"] == (
        "output/_tmp_kf_gins_gnss_feedback_compare/artifacts/current_solver/"
        "gnss_updates_current_solver.csv"
    )
    assert metadata["current_gnss_update_debug_output_path"] == fusion["gnss_update_debug_output_path"]


def test_build_kf_gins_feedback_config_sets_fixed_lever_and_debug_output_path():
    module = load_module()
    source_cfg = load_yaml(SOURCE_CFG_PATH)

    cfg = module.build_kf_gins_feedback_config(
        source_cfg,
        imupath=Path(r"C:\temp\uwb_kf_gins_compare\inputs\IMU_converted.txt"),
        gnsspath=Path(r"C:\temp\uwb_kf_gins_compare\inputs\rtk_full_window.txt"),
        outputpath=Path(r"C:\temp\uwb_kf_gins_compare\kf_gins_output"),
        gnss_update_debug_path=Path(r"C:\temp\uwb_kf_gins_compare\kf_gins_output\gnss_updates_kf_gins.csv"),
    )

    assert cfg["imupath"] == r"C:\temp\uwb_kf_gins_compare\inputs\IMU_converted.txt"
    assert cfg["gnsspath"] == r"C:\temp\uwb_kf_gins_compare\inputs\rtk_full_window.txt"
    assert cfg["outputpath"] == r"C:\temp\uwb_kf_gins_compare\kf_gins_output"
    assert cfg["antlever"] == [0.15, -0.22, -1.15]
    assert cfg["gnss_update_debug_path"] == r"C:\temp\uwb_kf_gins_compare\kf_gins_output\gnss_updates_kf_gins.csv"


def test_build_kf_gins_feedback_config_honors_scale_ablation_and_axiswise_init_std():
    module = load_module()
    source_cfg = load_yaml(BASE_CONFIG_PATH)

    cfg = module.build_kf_gins_feedback_config(
        source_cfg,
        imupath=Path(r"C:\temp\uwb_kf_gins_compare\inputs\IMU_converted.txt"),
        gnsspath=Path(r"C:\temp\uwb_kf_gins_compare\inputs\rtk_full_window.txt"),
        outputpath=Path(r"C:\temp\uwb_kf_gins_compare\kf_gins_output"),
    )

    assert all(math.isclose(a, b, rel_tol=0.0, abs_tol=1.0e-9) for a, b in zip(cfg["initbastd"], [2250.0, 225.0, 112.5]))
    assert all(math.isclose(a, b, rel_tol=0.0, abs_tol=1.0e-9) for a, b in zip(cfg["initbgstd"], [150.0, 112.5, 150.0]))
    assert cfg["initsgstd"] == [0.0, 0.0, 0.0]
    assert cfg["initsastd"] == [0.0, 0.0, 0.0]
    assert cfg["imunoise"]["gsstd"] == [0.0, 0.0, 0.0]
    assert cfg["imunoise"]["asstd"] == [0.0, 0.0, 0.0]


def test_normalize_current_gnss_debug_extracts_shared_state_blocks():
    module = load_module()
    raw = pd.DataFrame(
        [
            {
                "tag": "GNSS_POS",
                "gnss_t": 528076.0,
                "y_x": 0.1,
                "y_y": -0.2,
                "y_z": 0.3,
                "dx_pos_x": 1.0,
                "dx_pos_y": 2.0,
                "dx_pos_z": 2.0,
                "dx_vel_x": 3.0,
                "dx_vel_y": 4.0,
                "dx_vel_z": 0.0,
                "dx_att_x": 0.01,
                "dx_att_y": 0.02,
                "dx_att_z": 0.02,
                "dx_ba_x": 5.0,
                "dx_ba_y": 0.0,
                "dx_ba_z": 0.0,
                "dx_bg_x": 0.5,
                "dx_bg_y": 0.4,
                "dx_bg_z": 0.3,
                "dx_sg_x": 6.0,
                "dx_sg_y": 7.0,
                "dx_sg_z": 8.0,
                "dx_sa_x": 9.0,
                "dx_sa_y": 10.0,
                "dx_sa_z": 11.0,
            }
        ]
    )

    normalized = module.normalize_current_gnss_update_df(raw)
    row = normalized.iloc[0]

    assert row["update_index"] == 0
    assert row["gnss_t"] == 528076.0
    assert math.isclose(row["innovation_norm"], math.sqrt(0.14), rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["dx_pos_norm"], 3.0, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["dx_vel_norm"], 5.0, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["dx_att_norm"], 0.03, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["dx_sa_norm"], math.sqrt(302.0), rel_tol=0.0, abs_tol=1.0e-12)
    assert {"dx_vel_x", "dx_vel_y", "dx_vel_z", "dx_sa_x", "dx_sa_y", "dx_sa_z"}.issubset(normalized.columns)


def test_normalize_kf_gins_gnss_debug_extracts_shared_state_blocks():
    module = load_module()
    raw = pd.DataFrame(
        [
            {
                "update_index": 7,
                "gnss_t": 528080.0,
                "y_x": 0.4,
                "y_y": 0.5,
                "y_z": 0.6,
                "dx_pos_x": 0.1,
                "dx_pos_y": 0.2,
                "dx_pos_z": 0.2,
                "dx_vel_x": 0.3,
                "dx_vel_y": 0.4,
                "dx_vel_z": 0.0,
                "dx_att_x": 0.01,
                "dx_att_y": 0.01,
                "dx_att_z": 0.02,
                "dx_ba_x": 0.7,
                "dx_ba_y": 0.8,
                "dx_ba_z": 0.9,
                "dx_bg_x": 1.0,
                "dx_bg_y": 1.1,
                "dx_bg_z": 1.2,
                "dx_sg_x": 1.3,
                "dx_sg_y": 1.4,
                "dx_sg_z": 1.5,
                "dx_sa_x": 1.6,
                "dx_sa_y": 1.7,
                "dx_sa_z": 1.8,
            }
        ]
    )

    normalized = module.normalize_kf_gins_gnss_update_df(raw)
    row = normalized.iloc[0]

    assert row["update_index"] == 7
    assert row["gnss_t"] == 528080.0
    assert math.isclose(row["innovation_norm"], math.sqrt(0.77), rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["dx_pos_norm"], 0.3, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["dx_vel_norm"], 0.5, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["dx_att_norm"], math.sqrt(0.0006), rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["dx_sa_norm"], math.sqrt(8.69), rel_tol=0.0, abs_tol=1.0e-12)


def test_align_first_n_updates_trims_to_first_500_epochs_and_computes_diffs():
    module = load_module()

    current = pd.DataFrame(
        {
            "update_index": list(range(600)),
            "gnss_t": [528076.0 + 0.5 * idx for idx in range(600)],
            "innovation_x": [1.0] * 600,
            "innovation_y": [2.0] * 600,
            "innovation_z": [3.0] * 600,
            "dx_pos_x": [10.0] * 600,
            "dx_pos_y": [20.0] * 600,
            "dx_pos_z": [30.0] * 600,
        }
    )
    kf = pd.DataFrame(
        {
            "update_index": list(range(600)),
            "gnss_t": [528076.01 + 0.5 * idx for idx in range(600)],
            "innovation_x": [1.5] * 600,
            "innovation_y": [1.0] * 600,
            "innovation_z": [2.5] * 600,
            "dx_pos_x": [9.0] * 600,
            "dx_pos_y": [21.0] * 600,
            "dx_pos_z": [31.0] * 600,
        }
    )

    aligned = module.align_first_n_updates(current, kf, limit=500)

    assert len(aligned) == 500
    assert list(aligned["update_index"][:3]) == [0, 1, 2]
    assert math.isclose(aligned.iloc[0]["gnss_t_diff_s"], -0.01, rel_tol=0.0, abs_tol=1.0e-9)
    assert aligned.iloc[0]["diff_innovation_x"] == -0.5
    assert aligned.iloc[0]["diff_dx_pos_x"] == 1.0
    assert aligned.iloc[0]["diff_dx_pos_y"] == -1.0
    assert aligned.iloc[0]["diff_dx_pos_z"] == -1.0


def test_align_first_n_updates_adds_sign_aligned_diff_columns():
    module = load_module()

    current = pd.DataFrame(
        {
            "update_index": [0],
            "gnss_t": [528076.0],
            "innovation_x": [0.1],
            "innovation_y": [0.2],
            "innovation_z": [0.3],
            "dx_pos_x": [1.0],
            "dx_pos_y": [2.0],
            "dx_pos_z": [3.0],
        }
    )
    kf = pd.DataFrame(
        {
            "update_index": [0],
            "gnss_t": [528076.0],
            "innovation_x": [-0.1],
            "innovation_y": [-0.2],
            "innovation_z": [-0.3],
            "dx_pos_x": [-1.0],
            "dx_pos_y": [-2.0],
            "dx_pos_z": [-3.0],
        }
    )

    aligned = module.align_first_n_updates(current, kf, limit=1)

    assert aligned.iloc[0]["sign_aligned_diff_innovation_x"] == 0.0
    assert aligned.iloc[0]["sign_aligned_diff_innovation_y"] == 0.0
    assert aligned.iloc[0]["sign_aligned_diff_innovation_z"] == 0.0
    assert aligned.iloc[0]["sign_aligned_diff_dx_pos_x"] == 0.0
    assert aligned.iloc[0]["sign_aligned_diff_dx_pos_y"] == 0.0
    assert aligned.iloc[0]["sign_aligned_diff_dx_pos_z"] == 0.0


def test_summarize_feedback_deltas_reports_blockwise_abs_and_vector_stats():
    module = load_module()
    aligned = pd.DataFrame(
        {
            "update_index": [0, 1],
            "diff_innovation_x": [1.0, -1.0],
            "diff_innovation_y": [0.0, 2.0],
            "diff_innovation_z": [0.0, 2.0],
            "diff_dx_pos_x": [3.0, 0.0],
            "diff_dx_pos_y": [4.0, 0.0],
            "diff_dx_pos_z": [0.0, 12.0],
            "diff_dx_vel_x": [0.0, 0.0],
            "diff_dx_vel_y": [0.0, 0.0],
            "diff_dx_vel_z": [0.0, 0.0],
            "diff_dx_att_x": [0.0, 0.0],
            "diff_dx_att_y": [0.0, 0.0],
            "diff_dx_att_z": [0.0, 0.0],
            "diff_dx_ba_x": [0.0, 0.0],
            "diff_dx_ba_y": [0.0, 0.0],
            "diff_dx_ba_z": [0.0, 0.0],
            "diff_dx_bg_x": [0.0, 0.0],
            "diff_dx_bg_y": [0.0, 0.0],
            "diff_dx_bg_z": [0.0, 0.0],
            "diff_dx_sg_x": [0.0, 0.0],
            "diff_dx_sg_y": [0.0, 0.0],
            "diff_dx_sg_z": [0.0, 0.0],
            "diff_dx_sa_x": [0.0, 0.0],
            "diff_dx_sa_y": [0.0, 0.0],
            "diff_dx_sa_z": [0.0, 0.0],
        }
    )

    summary = module.summarize_feedback_deltas(aligned)
    rows = {row["block"]: row for _, row in summary.iterrows()}

    assert rows["innovation"]["samples"] == 2
    assert math.isclose(rows["innovation"]["mean_abs_component_diff"], 1.0, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(rows["innovation"]["max_abs_component_diff"], 2.0, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(rows["innovation"]["mean_vector_diff_norm"], 2.0, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(rows["pos"]["mean_vector_diff_norm"], 8.5, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(rows["pos"]["max_vector_diff_norm"], 12.0, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(rows["vel"]["mean_vector_diff_norm"], 0.0, rel_tol=0.0, abs_tol=1.0e-12)


def test_build_current_ba_x_decomposition_df_reconstructs_signed_numerator_s_and_gain():
    module = load_module()
    raw = pd.DataFrame(
        [
            {
                "update_index": 3,
                "gnss_t": 528078.0,
                "y_x": 1.0,
                "y_y": -1.0,
                "y_z": 2.0,
                "dx_ba_x": 6.0,
                "prior_cov_ba_pos_mat": "[1;2;3;0;0;0;0;0;0]",
                "prior_cov_ba_att_mat": "[4;5;6;0;0;0;0;0;0]",
                "prior_cov_ba_gnss_lever_mat": "[7;8;9;0;0;0;0;0;0]",
                "s_mat": "[2;0;0;0;3;0;0;0;6]",
                "h_pos_x_vec": "[1;0;0]",
                "h_pos_y_vec": "[0;1;0]",
                "h_pos_z_vec": "[0;0;1]",
                "h_att_x_vec": "[1;0;0]",
                "h_att_y_vec": "[0;1;0]",
                "h_att_z_vec": "[0;0;1]",
                "h_gnss_lever_x_vec": "[1;0;0]",
                "h_gnss_lever_y_vec": "[0;1;0]",
                "h_gnss_lever_z_vec": "[0;0;1]",
                "k_ba_x_vec": "[6;5;3]",
            }
        ]
    )

    decomposition = module.build_current_ba_x_decomposition_df(raw)
    row = decomposition.iloc[0]

    assert row["update_index"] == 3
    assert row["num_ba_x_from_pos_x"] == 1.0
    assert row["num_ba_x_from_pos_y"] == 2.0
    assert row["num_ba_x_from_pos_z"] == 3.0
    assert row["num_ba_x_from_att_x"] == 4.0
    assert row["num_ba_x_from_gnss_lever_z"] == 9.0
    assert row["num_ba_x_total_x"] == 12.0
    assert row["num_ba_x_total_y"] == 15.0
    assert row["num_ba_x_total_z"] == 18.0
    assert row["k_ba_x_x"] == 6.0
    assert row["k_ba_x_y"] == 5.0
    assert row["k_ba_x_z"] == 3.0
    assert row["k_ba_x_reconstructed_x"] == 6.0
    assert row["k_ba_x_reconstructed_y"] == 5.0
    assert row["k_ba_x_reconstructed_z"] == 3.0
    assert row["dx_ba_x_from_meas_x"] == 6.0
    assert row["dx_ba_x_from_meas_y"] == -5.0
    assert row["dx_ba_x_from_meas_z"] == 6.0
    assert row["s_00"] == 2.0
    assert row["s_11"] == 3.0
    assert row["s_22"] == 6.0
    assert math.isclose(row["k_ba_x_reconstruction_error_norm"], 0.0, rel_tol=0.0, abs_tol=1.0e-12)


def test_align_ba_x_decomposition_uses_sign_aware_for_signed_blocks_and_plain_diff_for_s():
    module = load_module()

    current_row = {"update_index": 1, "gnss_t": 528078.0, "k_ba_x_reconstruction_error_norm": 0.0}
    kf_row = {"update_index": 1, "gnss_t": 528078.0, "k_ba_x_reconstruction_error_norm": 0.0}

    for columns in module.BA_X_DECOMP_SIGNED_BLOCKS.values():
        for name in columns:
            current_row[name] = 1.0
            kf_row[name] = -0.75
    for columns in module.BA_X_DECOMP_INVARIANT_BLOCKS.values():
        for name in columns:
            current_row[name] = 10.0
            kf_row[name] = 7.0

    aligned = module.align_ba_x_decomposition(pd.DataFrame([current_row]), pd.DataFrame([kf_row]), limit=1)
    row = aligned.iloc[0]

    assert math.isclose(row["sign_aligned_diff_num_ba_x_from_pos_x"], 0.25, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["sign_aligned_diff_num_ba_x_total_z"], 0.25, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["sign_aligned_diff_k_ba_x_y"], 0.25, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["sign_aligned_diff_dx_ba_x"], 0.25, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["diff_num_ba_x_from_att_x"], 1.75, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["diff_s_00"], 3.0, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["diff_s_22"], 3.0, rel_tol=0.0, abs_tol=1.0e-12)


def test_render_summary_markdown_includes_sign_aware_ba_x_decomposition_section():
    module = load_module()
    raw_summary = pd.DataFrame(
        [
            {
                "block": "innovation",
                "samples": 1,
                "mean_abs_component_diff": 1.0,
                "max_abs_component_diff": 1.0,
                "mean_vector_diff_norm": 1.0,
                "max_vector_diff_norm": 1.0,
            }
        ]
    )
    sign_aligned_summary = pd.DataFrame(
        [
            {
                "block": "innovation",
                "samples": 1,
                "mean_abs_component_diff": 0.1,
                "max_abs_component_diff": 0.1,
                "mean_vector_diff_norm": 0.1,
                "max_vector_diff_norm": 0.1,
            }
        ]
    )
    aligned = pd.DataFrame({"update_index": [1], "gnss_t_diff_s": [0.0]})
    ba_x_summary = pd.DataFrame(
        [
            {
                "block": "num_ba_x_total",
                "samples": 1,
                "mean_abs_component_diff": 0.02,
                "max_abs_component_diff": 0.03,
                "mean_vector_diff_norm": 0.04,
                "max_vector_diff_norm": 0.05,
            }
        ]
    )
    ba_x_focus = pd.DataFrame(
        [
            {
                "update_index": 1,
                "gnss_t": 528078.0,
                "sign_aligned_diff_num_ba_x_total_norm": 0.04,
                "diff_s_mat_norm": 0.01,
                "sign_aligned_diff_k_ba_x_norm": 0.02,
                "sign_aligned_diff_dx_ba_x_abs": 0.001,
                "current_k_ba_x_reconstruction_error_norm": 0.0,
                "kf_k_ba_x_reconstruction_error_norm": 0.0,
            }
        ]
    )

    markdown = module.render_summary_markdown(
        raw_summary,
        sign_aligned_summary,
        aligned,
        ba_x_summary,
        ba_x_focus,
    )

    assert "## Sign-Aware PH^T Numerator / S / K_ba_x Decomposition" in markdown
    assert "num_ba_x_total" in markdown
    assert "sign_aligned_diff_k_ba_x_norm" in markdown


def test_build_current_s_decomposition_df_reconstructs_s_from_pos_att_and_r_blocks():
    module = load_module()
    raw = pd.DataFrame(
        [
            {
                "update_index": 1,
                "gnss_t": 528078.0,
                "prior_cov_pos_mat": "[10;1;2;1;20;3;2;3;30]",
                "prior_cov_att_mat": "[4;0;0;0;5;0;0;0;6]",
                "prior_cov_pos_att_mat": "[1;2;3;4;5;6;7;8;9]",
                "prior_cov_pos_gnss_lever_mat": "[0;0;0;0;0;0;0;0;0]",
                "prior_cov_att_gnss_lever_mat": "[0;0;0;0;0;0;0;0;0]",
                "prior_cov_gnss_lever_mat": "[0;0;0;0;0;0;0;0;0]",
                "r_mat": "[0.5;0;0;0;1.0;0;0;0;1.5]",
                "s_mat": "[16.5;7;12;7;36;17;12;17;55.5]",
                "h_pos_x_vec": "[1;0;0]",
                "h_pos_y_vec": "[0;1;0]",
                "h_pos_z_vec": "[0;0;1]",
                "h_att_x_vec": "[1;0;0]",
                "h_att_y_vec": "[0;1;0]",
                "h_att_z_vec": "[0;0;1]",
                "h_gnss_lever_x_vec": "[0;0;0]",
                "h_gnss_lever_y_vec": "[0;0;0]",
                "h_gnss_lever_z_vec": "[0;0;0]",
            }
        ]
    )

    decomposition = module.build_current_s_decomposition_df(raw)
    row = decomposition.iloc[0]

    assert row["update_index"] == 1
    assert row["s_from_pos_pos_00"] == 10.0
    assert row["s_from_pos_pos_12"] == 3.0
    assert row["s_from_pos_att_cross_00"] == 2.0
    assert row["s_from_pos_att_cross_01"] == 6.0
    assert row["s_from_pos_att_cross_22"] == 18.0
    assert row["s_from_att_att_11"] == 5.0
    assert row["r_22"] == 1.5
    assert row["s_reconstructed_00"] == 16.5
    assert row["s_reconstructed_11"] == 36.0
    assert row["s_reconstructed_22"] == 55.5
    assert math.isclose(row["s_reconstruction_error_norm"], 0.0, rel_tol=0.0, abs_tol=1.0e-12)


def test_align_s_decomposition_uses_plain_diff_for_all_s_blocks():
    module = load_module()

    current_row = {"update_index": 1, "gnss_t": 528078.0, "s_reconstruction_error_norm": 0.0}
    kf_row = {"update_index": 1, "gnss_t": 528078.0, "s_reconstruction_error_norm": 0.0}

    for columns in module.S_DECOMP_BLOCKS.values():
        for name in columns:
            current_row[name] = 5.0
            kf_row[name] = 3.5

    aligned = module.align_s_decomposition(pd.DataFrame([current_row]), pd.DataFrame([kf_row]), limit=1)
    row = aligned.iloc[0]

    assert math.isclose(row["diff_s_from_pos_pos_00"], 1.5, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["diff_s_from_pos_att_cross_12"], 1.5, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["diff_r_22"], 1.5, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["diff_s_reconstructed_21"], 1.5, rel_tol=0.0, abs_tol=1.0e-12)


def test_render_summary_markdown_includes_s_decomposition_section():
    module = load_module()
    raw_summary = pd.DataFrame(
        [
            {
                "block": "innovation",
                "samples": 1,
                "mean_abs_component_diff": 1.0,
                "max_abs_component_diff": 1.0,
                "mean_vector_diff_norm": 1.0,
                "max_vector_diff_norm": 1.0,
            }
        ]
    )
    sign_aligned_summary = raw_summary.copy()
    aligned = pd.DataFrame({"update_index": [1], "gnss_t_diff_s": [0.0]})
    ba_x_summary = pd.DataFrame(
        [
            {
                "block": "num_ba_x_total",
                "comparison_mode": "sign_aligned",
                "samples": 1,
                "mean_abs_component_diff": 0.02,
                "max_abs_component_diff": 0.03,
                "mean_vector_diff_norm": 0.04,
                "max_vector_diff_norm": 0.05,
            }
        ]
    )
    ba_x_focus = pd.DataFrame(
        [
            {
                "update_index": 1,
                "gnss_t": 528078.0,
                "sign_aligned_diff_num_ba_x_from_pos_norm": 0.01,
                "sign_aligned_diff_num_ba_x_from_att_norm": 0.01,
                "sign_aligned_diff_num_ba_x_from_gnss_lever_norm": 0.0,
                "sign_aligned_diff_num_ba_x_total_norm": 0.04,
                "diff_s_mat_norm": 0.01,
                "sign_aligned_diff_k_ba_x_norm": 0.02,
                "sign_aligned_diff_dx_ba_x_abs": 0.001,
                "current_k_ba_x_reconstruction_error_norm": 0.0,
                "kf_k_ba_x_reconstruction_error_norm": 0.0,
            }
        ]
    )
    s_summary = pd.DataFrame(
        [
            {
                "block": "s_from_pos_pos",
                "comparison_mode": "plain_diff",
                "samples": 1,
                "mean_abs_component_diff": 0.1,
                "max_abs_component_diff": 0.2,
                "mean_vector_diff_norm": 0.3,
                "max_vector_diff_norm": 0.4,
            }
        ]
    )
    s_focus = pd.DataFrame(
        [
            {
                "update_index": 1,
                "gnss_t": 528078.0,
                "diff_s_from_pos_pos_norm": 0.01,
                "diff_s_from_pos_att_cross_norm": 0.02,
                "diff_s_from_att_att_norm": 0.03,
                "diff_s_from_pos_gnss_lever_cross_norm": 0.0,
                "diff_s_from_att_gnss_lever_cross_norm": 0.0,
                "diff_s_from_gnss_lever_gnss_lever_norm": 0.0,
                "diff_r_mat_norm": 0.04,
                "diff_s_mat_norm": 0.05,
                "current_s_reconstruction_error_norm": 0.0,
                "kf_s_reconstruction_error_norm": 0.0,
            }
        ]
    )

    markdown = module.render_summary_markdown(
        raw_summary,
        sign_aligned_summary,
        aligned,
        ba_x_summary,
        ba_x_focus,
        s_summary,
        s_focus,
    )

    assert "## S = HPH^T + R Decomposition" in markdown
    assert "s_from_pos_pos" in markdown
    assert "diff_r_mat_norm" in markdown


def test_build_current_posterior_covariance_df_reconstructs_post_pos_and_att_blocks():
    module = load_module()
    raw = pd.DataFrame(
        [
            {
                "update_index": 0,
                "gnss_t": 528077.0,
                "prior_cov_pos_mat": "[2;0;0;0;2;0;0;0;2]",
                "prior_cov_att_mat": "[3;0;0;0;3;0;0;0;3]",
                "prior_cov_pos_att_mat": "[0;0;0;0;0;0;0;0;0]",
                "prior_cov_pos_gnss_lever_mat": "[0;0;0;0;0;0;0;0;0]",
                "prior_cov_att_gnss_lever_mat": "[0;0;0;0;0;0;0;0;0]",
                "prior_cov_gnss_lever_mat": "[0;0;0;0;0;0;0;0;0]",
                "r_mat": "[2;0;0;0;2;0;0;0;2]",
                "h_pos_x_vec": "[1;0;0]",
                "h_pos_y_vec": "[0;1;0]",
                "h_pos_z_vec": "[0;0;1]",
                "h_att_x_vec": "[0;0;0]",
                "h_att_y_vec": "[0;0;0]",
                "h_att_z_vec": "[0;0;0]",
                "h_gnss_lever_x_vec": "[0;0;0]",
                "h_gnss_lever_y_vec": "[0;0;0]",
                "h_gnss_lever_z_vec": "[0;0;0]",
                "k_pos_x_vec": "[0.5;0;0]",
                "k_pos_y_vec": "[0;0.5;0]",
                "k_pos_z_vec": "[0;0;0.5]",
                "k_att_x_vec": "[0;0;0]",
                "k_att_y_vec": "[0;0;0]",
                "k_att_z_vec": "[0;0;0]",
            }
        ]
    )

    posterior = module.build_current_posterior_covariance_df(raw)
    row = posterior.iloc[0]

    assert row["update_index"] == 0
    assert row["post_cov_pos_00"] == 1.0
    assert row["post_cov_pos_11"] == 1.0
    assert row["post_cov_pos_22"] == 1.0
    assert row["post_cov_att_00"] == 3.0
    assert row["post_cov_att_11"] == 3.0
    assert row["post_cov_pos_att_00"] == 0.0
    assert math.isclose(row["post_cov_reconstruction_error_norm"], 0.0, rel_tol=0.0, abs_tol=1.0e-12)


def test_build_kf_gins_posterior_covariance_df_reconstructs_post_pos_and_att_blocks():
    module = load_module()
    raw = pd.DataFrame(
        [
            {
                "update_index": 0,
                "gnss_t": 528077.0,
                "prior_cov_pos_mat": "[2;0;0;0;2;0;0;0;2]",
                "prior_cov_att_mat": "[3;0;0;0;3;0;0;0;3]",
                "prior_cov_pos_att_mat": "[0;0;0;0;0;0;0;0;0]",
                "r_mat": "[2;0;0;0;2;0;0;0;2]",
                "h_att_x_vec": "[0;0;0]",
                "h_att_y_vec": "[0;0;0]",
                "h_att_z_vec": "[0;0;0]",
                "k_pos_x_vec": "[0.5;0;0]",
                "k_pos_y_vec": "[0;0.5;0]",
                "k_pos_z_vec": "[0;0;0.5]",
                "k_att_x_vec": "[0;0;0]",
                "k_att_y_vec": "[0;0;0]",
                "k_att_z_vec": "[0;0;0]",
            }
        ]
    )

    posterior = module.build_kf_gins_posterior_covariance_df(raw)
    row = posterior.iloc[0]

    assert row["update_index"] == 0
    assert row["post_cov_pos_00"] == 1.0
    assert row["post_cov_pos_11"] == 1.0
    assert row["post_cov_pos_22"] == 1.0
    assert row["post_cov_att_00"] == 3.0
    assert row["post_cov_att_11"] == 3.0
    assert row["post_cov_pos_att_00"] == 0.0
    assert math.isclose(row["post_cov_reconstruction_error_norm"], 0.0, rel_tol=0.0, abs_tol=1.0e-12)


def test_build_first_step_transition_focus_table_reports_post_and_predict_deltas():
    module = load_module()

    current_prior = pd.DataFrame(
        [
            {"update_index": 0, "gnss_t": 528077.0, "prior_cov_pos_00": 2.0, "prior_cov_pos_att_00": 0.1, "prior_cov_att_00": 3.0},
            {"update_index": 1, "gnss_t": 528078.0, "prior_cov_pos_00": 1.5, "prior_cov_pos_att_00": 0.3, "prior_cov_att_00": 3.4},
        ]
    )
    current_post = pd.DataFrame(
        [
            {"update_index": 0, "gnss_t": 528077.0, "post_cov_pos_00": 1.0, "post_cov_pos_att_00": 0.05, "post_cov_att_00": 3.0}
        ]
    )
    kf_prior = pd.DataFrame(
        [
            {"update_index": 0, "gnss_t": 528077.0, "prior_cov_pos_00": 1.8, "prior_cov_pos_att_00": 0.1, "prior_cov_att_00": 2.8},
            {"update_index": 1, "gnss_t": 528078.0, "prior_cov_pos_00": 1.2, "prior_cov_pos_att_00": 0.15, "prior_cov_att_00": 3.0},
        ]
    )
    kf_post = pd.DataFrame(
        [
            {"update_index": 0, "gnss_t": 528077.0, "post_cov_pos_00": 0.9, "post_cov_pos_att_00": 0.04, "post_cov_att_00": 2.7}
        ]
    )

    focus = module.build_first_step_transition_focus_table(current_prior, current_post, kf_prior, kf_post)
    row = focus.iloc[0]

    assert row["from_update_index"] == 0
    assert row["to_update_index"] == 1
    assert math.isclose(row["diff_post_cov_pos_norm"], 0.1, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["diff_post_cov_pos_att_norm"], 0.01, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["current_predict_delta_cov_pos_norm"], 0.5, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["kf_predict_delta_cov_pos_norm"], 0.3, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["diff_predict_delta_cov_pos_norm"], 0.2, rel_tol=0.0, abs_tol=1.0e-12)


def test_infer_first_predict_window_from_updates_uses_first_two_gnss_updates():
    module = load_module()
    updates = pd.DataFrame(
        [
            {"update_index": 0, "gnss_t": 528077.0},
            {"update_index": 1, "gnss_t": 528078.0},
            {"update_index": 2, "gnss_t": 528079.0},
        ]
    )

    window = module.infer_first_predict_window_from_updates(updates)

    assert window == (528077.0, 528078.0)


def test_build_predict_debug_alignment_and_summary_separate_phi_p_and_qd():
    module = load_module()

    def mat_csv(values: list[list[float]]) -> str:
        flat = [str(item) for row in values for item in row]
        return "[" + ";".join(flat) + "]"

    zero_21 = [[0.0] * 21 for _ in range(21)]
    eye_21 = [[0.0] * 21 for _ in range(21)]
    for idx in range(21):
        eye_21[idx][idx] = 1.0

    current_p_before = [row[:] for row in zero_21]
    current_p_before[0][0] = 1.0
    current_phi_p = [row[:] for row in current_p_before]
    current_qd = [row[:] for row in zero_21]
    current_qd[0][0] = 0.1
    current_after = [row[:] for row in current_p_before]
    current_after[0][0] = 1.1

    kf_p_before = [row[:] for row in zero_21]
    kf_p_before[0][0] = 0.9
    kf_phi_p = [row[:] for row in kf_p_before]
    kf_qd = [row[:] for row in zero_21]
    kf_qd[0][0] = 0.05
    kf_after = [row[:] for row in kf_p_before]
    kf_after[0][0] = 0.95

    current_raw = pd.DataFrame(
        [
            {
                "step_index": 0,
                "tag": "tail",
                "t_prev": 528077.0,
                "t_curr": 528077.005,
                "dt": 0.005,
                "p_before_common_mat": mat_csv(current_p_before),
                "phi_common_mat": mat_csv(eye_21),
                "qd_common_mat": mat_csv(current_qd),
                "phi_p_common_mat": mat_csv(current_phi_p),
                "p_after_raw_common_mat": mat_csv(current_after),
                "p_after_final_common_mat": mat_csv(current_after),
            }
        ]
    )
    kf_raw = pd.DataFrame(
        [
            {
                "step_index": 0,
                "tag": "ins",
                "t_prev": 528077.0,
                "t_curr": 528077.005,
                "dt": 0.005,
                "p_before_common_mat": mat_csv(kf_p_before),
                "phi_common_mat": mat_csv(eye_21),
                "qd_common_mat": mat_csv(kf_qd),
                "phi_p_common_mat": mat_csv(kf_phi_p),
                "p_after_common_mat": mat_csv(kf_after),
            }
        ]
    )

    current_predict = module.build_current_predict_debug_df(current_raw)
    kf_predict = module.build_kf_predict_debug_df(kf_raw)
    aligned = module.align_predict_steps(current_predict, kf_predict)
    summary = module.summarize_predict_step_deltas(aligned)
    focus = module.build_predict_step_focus_table(aligned)

    row = aligned.iloc[0]
    assert math.isclose(row["diff_p_before_cov_pos_00"], 0.1, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["diff_phi_p_cov_pos_norm"], 0.1, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["diff_qd_cov_pos_norm"], 0.05, rel_tol=0.0, abs_tol=1.0e-12)
    assert math.isclose(row["diff_p_after_final_cov_pos_norm"], 0.15, rel_tol=0.0, abs_tol=1.0e-12)

    phi_p_summary = summary[(summary["term"] == "phi_p") & (summary["block"] == "cov_pos")].iloc[0]
    assert math.isclose(phi_p_summary["max_vector_diff_norm"], 0.1, rel_tol=0.0, abs_tol=1.0e-12)
    qd_summary = summary[(summary["term"] == "qd") & (summary["block"] == "cov_pos")].iloc[0]
    assert math.isclose(qd_summary["max_vector_diff_norm"], 0.05, rel_tol=0.0, abs_tol=1.0e-12)
    assert focus.iloc[0]["current_tag"] == "tail"


def test_render_summary_markdown_includes_first_step_transition_section():
    module = load_module()
    raw_summary = pd.DataFrame(
        [
            {
                "block": "innovation",
                "samples": 1,
                "mean_abs_component_diff": 1.0,
                "max_abs_component_diff": 1.0,
                "mean_vector_diff_norm": 1.0,
                "max_vector_diff_norm": 1.0,
            }
        ]
    )
    sign_aligned_summary = raw_summary.copy()
    aligned = pd.DataFrame({"update_index": [1], "gnss_t_diff_s": [0.0]})
    ba_x_summary = pd.DataFrame(
        [
            {
                "block": "num_ba_x_total",
                "comparison_mode": "sign_aligned",
                "samples": 1,
                "mean_abs_component_diff": 0.02,
                "max_abs_component_diff": 0.03,
                "mean_vector_diff_norm": 0.04,
                "max_vector_diff_norm": 0.05,
            }
        ]
    )
    ba_x_focus = pd.DataFrame(
        [
            {
                "update_index": 1,
                "gnss_t": 528078.0,
                "sign_aligned_diff_num_ba_x_from_pos_norm": 0.01,
                "sign_aligned_diff_num_ba_x_from_att_norm": 0.01,
                "sign_aligned_diff_num_ba_x_from_gnss_lever_norm": 0.0,
                "sign_aligned_diff_num_ba_x_total_norm": 0.04,
                "diff_s_mat_norm": 0.01,
                "sign_aligned_diff_k_ba_x_norm": 0.02,
                "sign_aligned_diff_dx_ba_x_abs": 0.001,
                "current_k_ba_x_reconstruction_error_norm": 0.0,
                "kf_k_ba_x_reconstruction_error_norm": 0.0,
            }
        ]
    )
    s_summary = pd.DataFrame(
        [
            {
                "block": "s_from_pos_pos",
                "comparison_mode": "plain_diff",
                "samples": 1,
                "mean_abs_component_diff": 0.1,
                "max_abs_component_diff": 0.2,
                "mean_vector_diff_norm": 0.3,
                "max_vector_diff_norm": 0.4,
            }
        ]
    )
    s_focus = pd.DataFrame(
        [
            {
                "update_index": 1,
                "gnss_t": 528078.0,
                "diff_s_from_pos_pos_norm": 0.01,
                "diff_s_from_pos_att_cross_norm": 0.02,
                "diff_s_from_att_att_norm": 0.03,
                "diff_s_from_pos_gnss_lever_cross_norm": 0.0,
                "diff_s_from_att_gnss_lever_cross_norm": 0.0,
                "diff_s_from_gnss_lever_gnss_lever_norm": 0.0,
                "diff_r_mat_norm": 0.04,
                "diff_s_mat_norm": 0.05,
                "current_s_reconstruction_error_norm": 0.0,
                "kf_s_reconstruction_error_norm": 0.0,
            }
        ]
    )
    first_step_focus = pd.DataFrame(
        [
            {
                "from_update_index": 0,
                "to_update_index": 1,
                "diff_post_cov_pos_norm": 0.1,
                "diff_post_cov_pos_att_norm": 0.01,
                "diff_post_cov_att_norm": 0.3,
                "diff_prior_cov_pos_norm": 0.2,
                "diff_prior_cov_pos_att_norm": 0.15,
                "diff_prior_cov_att_norm": 0.4,
                "current_predict_delta_cov_pos_norm": 0.5,
                "kf_predict_delta_cov_pos_norm": 0.3,
                "diff_predict_delta_cov_pos_norm": 0.2,
                "current_predict_delta_cov_pos_att_norm": 0.25,
                "kf_predict_delta_cov_pos_att_norm": 0.11,
                "diff_predict_delta_cov_pos_att_norm": 0.14,
                "current_predict_delta_cov_att_norm": 0.4,
                "kf_predict_delta_cov_att_norm": 0.3,
                "diff_predict_delta_cov_att_norm": 0.1,
            }
        ]
    )
    predict_summary = pd.DataFrame(
        [
            {
                "term": "phi_p",
                "block": "cov_pos",
                "samples": 1,
                "mean_abs_component_diff": 0.1,
                "max_abs_component_diff": 0.1,
                "mean_vector_diff_norm": 0.1,
                "max_vector_diff_norm": 0.1,
            }
        ]
    )
    predict_focus = pd.DataFrame(
        [
            {
                "current_tag": "tail",
                "current_step_index": 0,
                "current_t_prev": 528077.0,
                "current_t_curr": 528077.005,
                "current_dt": 0.005,
                "diff_p_before_cov_pos_norm": 0.1,
                "diff_phi_p_cov_pos_norm": 0.1,
                "diff_qd_cov_pos_norm": 0.05,
                "diff_p_after_raw_cov_pos_norm": 0.15,
                "diff_p_after_final_cov_pos_norm": 0.15,
                "diff_phi_p_cov_pos_att_norm": 0.01,
                "diff_qd_cov_pos_att_norm": 0.0,
                "diff_p_after_final_cov_pos_att_norm": 0.01,
                "diff_phi_p_cov_pos_vel_norm": 0.02,
                "diff_p_after_final_cov_pos_vel_norm": 0.02,
                "diff_phi_p_cov_vel_vel_norm": 0.03,
                "diff_p_after_final_cov_vel_vel_norm": 0.03,
                "current_raw_reconstruction_error_norm": 0.0,
                "kf_raw_reconstruction_error_norm": 0.0,
                "current_final_minus_raw_norm": 0.0,
                "kf_final_minus_raw_norm": 0.0,
            }
        ]
    )

    markdown = module.render_summary_markdown(
        raw_summary,
        sign_aligned_summary,
        aligned,
        ba_x_summary,
        ba_x_focus,
        s_summary,
        s_focus,
        first_step_focus,
        predict_summary,
        predict_focus,
    )

    assert "## First-Step Posterior vs Predict Delta" in markdown
    assert "diff_post_cov_pos_norm" in markdown
    assert "diff_predict_delta_cov_pos_norm" in markdown
    assert "## Predict Decomposition Inside First GNSS Window" in markdown
    assert "diff_phi_p_cov_pos_norm" in markdown
