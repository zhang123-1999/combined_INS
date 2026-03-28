import importlib.util
from pathlib import Path
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml


MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_full_ins_gnss_bias_scale_process_sweep.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing sweep runner: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_full_ins_gnss_bias_scale_process_sweep", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_case_specs_and_configs_match_full_ins_gnss_process_sweep_plan():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_full_ins_gnss_bias_scale_process_sweep_contract")

    assert module.EXP_ID_DEFAULT == "EXP-20260327-data2-full-ins-gnss-bias-scale-process-sweep-r1"
    assert module.OUTPUT_DIR_DEFAULT == Path("output/data2_full_ins_gnss_bias_scale_process_sweep")
    assert module.Q_SCALES_DEFAULT == (1.0, 2.0, 4.0, 8.0)

    specs = module.build_case_specs()
    assert [spec.case_id for spec in specs] == [
        "data2_full_ins_gnss_bias_scale_q1x",
        "data2_full_ins_gnss_bias_scale_q2x",
        "data2_full_ins_gnss_bias_scale_q4x",
        "data2_full_ins_gnss_bias_scale_q8x",
    ]

    base_noise = base_cfg["fusion"]["noise"]
    for spec in specs:
        cfg, metadata = module.build_case_config(base_cfg, output_dir, spec)
        noise = cfg["fusion"]["noise"]
        ablation = cfg["fusion"]["ablation"]
        assert metadata["case_id"] == spec.case_id
        assert metadata["case_label"] == spec.label
        assert metadata["process_noise_scale_rel"] == spec.process_noise_scale_rel
        assert ablation["disable_gyro_scale"] is False
        assert ablation["disable_accel_scale"] is False
        assert noise["sigma_ba"] == base_noise["sigma_ba"] * spec.process_noise_scale_rel
        assert noise["sigma_bg"] == base_noise["sigma_bg"] * spec.process_noise_scale_rel
        assert noise["sigma_sg"] == base_noise["sigma_sg"] * spec.process_noise_scale_rel
        assert noise["sigma_sa"] == base_noise["sigma_sa"] * spec.process_noise_scale_rel
        assert noise["sigma_ba_vec"] == [noise["sigma_ba"]] * 3
        assert noise["sigma_bg_vec"] == [noise["sigma_bg"]] * 3
        assert noise["sigma_sg_vec"] == [noise["sigma_sg"]] * 3
        assert noise["sigma_sa_vec"] == [noise["sigma_sa"]] * 3


def test_compute_series_oscillation_metrics_ranks_oscillatory_trace_above_monotonic():
    module = load_module()

    monotonic = np.array([0.0, 1.0, 2.0, 3.0, 4.0], dtype=float)
    oscillatory = np.array([0.0, 2.0, -2.0, 2.0, -2.0, 0.0], dtype=float)

    monotonic_metrics = module.compute_series_oscillation_metrics(monotonic)
    oscillatory_metrics = module.compute_series_oscillation_metrics(oscillatory)

    assert monotonic_metrics["sign_flip_count"] == 0
    assert monotonic_metrics["oscillation_index"] == 1.0
    assert oscillatory_metrics["sign_flip_count"] > monotonic_metrics["sign_flip_count"]
    assert oscillatory_metrics["total_variation"] > monotonic_metrics["total_variation"]
    assert oscillatory_metrics["oscillation_index"] > monotonic_metrics["oscillation_index"]


def test_plot_config_focuses_on_bias_and_scale_groups():
    module = load_module()

    plot_config = module.build_bias_scale_plot_config()

    assert [group.group_id for group in plot_config.group_specs] == ["ba", "bg", "sg", "sa"]
    assert [state.key for state in plot_config.overview_states] == [
        "ba_x_mgal",
        "ba_y_mgal",
        "ba_z_mgal",
        "bg_x_degh",
        "bg_y_degh",
        "bg_z_degh",
        "sg_x_ppm",
        "sg_y_ppm",
        "sg_z_ppm",
        "sa_x_ppm",
        "sa_y_ppm",
        "sa_z_ppm",
    ]


def test_build_case_config_supports_fixed_gnss_lever_truth_mode():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_full_ins_gnss_bias_scale_process_sweep_fix_gnss_lever_truth")

    spec = module.build_case_specs([2.0])[0]
    cfg, metadata = module.build_case_config(
        base_cfg,
        output_dir,
        spec,
        fix_gnss_lever_truth=True,
    )

    fusion = cfg["fusion"]
    noise = fusion["noise"]
    init_cfg = fusion["init"]
    ablation = fusion["ablation"]

    assert metadata["fix_gnss_lever_truth"] is True
    assert ablation["disable_gnss_lever_arm"] is True
    assert ablation["disable_gyro_scale"] is False
    assert ablation["disable_accel_scale"] is False
    assert noise["sigma_gnss_lever_arm"] == 0.0
    assert noise["sigma_gnss_lever_arm_vec"] == [0.0, 0.0, 0.0]
    assert init_cfg["std_gnss_lever_arm"] == [0.0, 0.0, 0.0]
    assert init_cfg["P0_diag"][28:31] == [0.0, 0.0, 0.0]
