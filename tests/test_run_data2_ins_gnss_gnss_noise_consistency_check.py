import importlib.util
import math
from pathlib import Path
import sys

import pandas as pd
import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_ins_gnss_gnss_noise_consistency_check.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing GNSS noise consistency runner: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_ins_gnss_gnss_noise_consistency_check", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_build_case_config_enables_gnss_update_debug_output():
    module = load_module()
    base_cfg = load_yaml(REPO_ROOT / "config_data2_baseline_eskf.yaml")
    output_dir = Path("output/_tmp_gnss_noise_consistency")
    gnss_path = Path("dataset/data2/rtk.txt")

    cfg, metadata = module.build_case_config(
        base_cfg=base_cfg,
        output_dir=output_dir,
        gnss_path=gnss_path,
        case_id="consistency_r1p0",
        case_label="consistency r=1.0x",
    )
    fusion = cfg["fusion"]

    assert fusion["gnss_path"] == "dataset/data2/rtk.txt"
    assert fusion["output_path"] == "output/_tmp_gnss_noise_consistency/SOL_consistency_r1p0.txt"
    assert (
        fusion["state_series_output_path"]
        == "output/_tmp_gnss_noise_consistency/state_series_consistency_r1p0.csv"
    )
    assert (
        fusion["gnss_update_debug_output_path"]
        == "output/_tmp_gnss_noise_consistency/artifacts/cases/consistency_r1p0/gnss_updates_consistency_r1p0.csv"
    )
    assert metadata["case_id"] == "consistency_r1p0"
    assert metadata["case_label"] == "consistency r=1.0x"


def test_compute_window_consistency_metrics_uses_s_diagonal():
    module = load_module()
    df = pd.DataFrame(
        [
            {
                "tag": "GNSS_POS",
                "gnss_t": 10.0,
                "y_x": 2.0,
                "y_y": 1.0,
                "y_z": 0.5,
                "s_mat": "[1;0;0;0;4;0;0;0;0.25]",
            },
            {
                "tag": "GNSS_POS",
                "gnss_t": 11.0,
                "y_x": -2.0,
                "y_y": -1.0,
                "y_z": -0.5,
                "s_mat": "[1;0;0;0;4;0;0;0;0.25]",
            },
            {
                "tag": "GNSS_POS",
                "gnss_t": 12.0,
                "y_x": 2.0,
                "y_y": 1.0,
                "y_z": 0.5,
                "s_mat": "[1;0;0;0;4;0;0;0;0.25]",
            },
            {
                "tag": "GNSS_POS",
                "gnss_t": 13.0,
                "y_x": -2.0,
                "y_y": -1.0,
                "y_z": -0.5,
                "s_mat": "[1;0;0;0;4;0;0;0;0.25]",
            },
        ]
    )

    metrics = module.compute_window_consistency_metrics(df, window_name="phase12", start_time=9.5, end_time=13.5)

    assert metrics["accepted_updates"] == 4
    assert metrics["actual_std_x_m"] == 2.0
    assert metrics["actual_std_y_m"] == 1.0
    assert metrics["actual_std_z_m"] == 0.5
    assert metrics["predicted_std_x_m"] == 1.0
    assert metrics["predicted_std_y_m"] == 2.0
    assert metrics["predicted_std_z_m"] == 0.5
    assert metrics["ratio_x"] == 2.0
    assert metrics["ratio_y"] == 0.5
    assert metrics["ratio_z"] == 1.0
    assert metrics["normalized_std_x"] == 2.0
    assert metrics["normalized_std_y"] == 0.5
    assert metrics["normalized_std_z"] == 1.0


def test_select_recommended_r_scale_prefers_ratio_closest_to_one():
    module = load_module()
    summary_df = pd.DataFrame(
        [
            {
                "case_id": "r1",
                "r_scale": 1.0,
                "window_name": "phase12",
                "accepted_updates": 100,
                "ratio_x": 3.0,
                "ratio_y": 2.6,
                "ratio_z": 2.2,
            },
            {
                "case_id": "r4",
                "r_scale": 4.0,
                "window_name": "phase12",
                "accepted_updates": 100,
                "ratio_x": 1.05,
                "ratio_y": 0.98,
                "ratio_z": 1.10,
            },
            {
                "case_id": "r8",
                "r_scale": 8.0,
                "window_name": "phase12",
                "accepted_updates": 100,
                "ratio_x": 0.70,
                "ratio_y": 0.66,
                "ratio_z": 0.61,
            },
        ]
    )

    recommended = module.select_recommended_r_scale(summary_df, target_window="phase12")

    assert recommended["case_id"] == "r4"
    assert recommended["r_scale"] == 4.0
    assert recommended["accepted_updates"] == 100
    assert math.isclose(recommended["consistency_score"], 0.05666666666666672, rel_tol=1.0e-6)


def test_assess_r_inflation_need_distinguishes_small_mixed_and_not_small():
    module = load_module()

    assert module.assess_r_inflation_need({"ratio_x": 2.1, "ratio_y": 1.9, "ratio_z": 1.7}) == "too_small"
    assert module.assess_r_inflation_need({"ratio_x": 0.6, "ratio_y": 0.4, "ratio_z": 0.2}) == "not_too_small"
    assert module.assess_r_inflation_need({"ratio_x": 1.2, "ratio_y": 1.3, "ratio_z": 0.4}) == "mixed"


def test_write_axis_scaled_gnss_variant_scales_xy_and_z_separately(tmp_path: Path):
    module = load_module()
    src_path = tmp_path / "rtk_src.txt"
    src_path.write_text(
        "\n".join(
            [
                "1.00 31.0 121.0 10.0 0.010000 0.020000 0.030000",
                "2.00 31.1 121.1 11.0 0.040000 0.050000 0.060000",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    dst_path = tmp_path / "rtk_scaled.txt"

    row = module.write_axis_scaled_gnss_variant(src_path, dst_path, r_xy_scale=4.0, r_z_scale=0.5)
    scaled = module.read_rtk_file(dst_path)

    assert dst_path.exists()
    assert list(scaled["sigma_n"]) == pytest.approx([0.04, 0.16])
    assert list(scaled["sigma_e"]) == pytest.approx([0.08, 0.20])
    assert list(scaled["sigma_d"]) == pytest.approx([0.015, 0.03])
    assert row["r_xy_scale"] == 4.0
    assert row["r_z_scale"] == 0.5


def test_build_r_scale_specs_supports_xy_z_grid():
    module = load_module()

    specs = module.build_r_scale_specs(r_scales=None, r_xy_scales=[1.0, 4.0], r_z_scales=[0.25, 0.5])

    assert [(spec.r_xy_scale, spec.r_z_scale, spec.slug) for spec in specs] == [
        (1.0, 0.25, "xy1_z0p25"),
        (1.0, 0.5, "xy1_z0p5"),
        (4.0, 0.25, "xy4_z0p25"),
        (4.0, 0.5, "xy4_z0p5"),
    ]


def test_build_summary_lines_reports_separate_xy_and_z_recommendation():
    module = load_module()
    target_rows = pd.DataFrame(
        [
            {
                "case_id": "xy1_z1",
                "r_xy_scale": 1.0,
                "r_z_scale": 1.0,
                "accepted_updates": 100,
                "actual_std_x_m": 0.02,
                "predicted_std_x_m": 0.017,
                "ratio_x": 1.18,
                "actual_std_y_m": 0.021,
                "predicted_std_y_m": 0.017,
                "ratio_y": 1.24,
                "actual_std_z_m": 0.013,
                "predicted_std_z_m": 0.037,
                "ratio_z": 0.35,
                "consistency_score": 0.356,
                "overall_rmse_3d_m_aux": 77.7,
            },
            {
                "case_id": "xy4_z0p25",
                "r_xy_scale": 4.0,
                "r_z_scale": 0.25,
                "accepted_updates": 100,
                "actual_std_x_m": 0.060,
                "predicted_std_x_m": 0.060,
                "ratio_x": 1.0,
                "actual_std_y_m": 0.068,
                "predicted_std_y_m": 0.061,
                "ratio_y": 1.11,
                "actual_std_z_m": 0.006,
                "predicted_std_z_m": 0.007,
                "ratio_z": 0.86,
                "consistency_score": 0.083,
                "overall_rmse_3d_m_aux": 78.0,
            },
        ]
    )
    manifest = {
        "exp_id": "EXP-TEST",
        "base_config": "config.yaml",
        "source_runner": "runner.py",
        "output_dir": "output/test",
        "target_window": "phase2",
        "generated_at": "2026-03-26T23:59:00",
    }
    recommended = target_rows.iloc[1].to_dict()

    lines = module.build_summary_lines(manifest, target_rows, recommended)
    text = "\n".join(lines)

    assert "r_xy_scale" in text
    assert "r_z_scale" in text
    assert "recommended_r_scales: `xy=4.000000`, `z=0.250000`" in text
