import importlib.util
import json
import math
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_kf_gins_pure_ins_compare.py"
SOURCE_MANIFEST_PATH = REPO_ROOT / "output" / "d2_baseline_pure_ins_90s_r2" / "manifest.json"
SOURCE_CONFIG_PATH = (
    REPO_ROOT
    / "output"
    / "d2_baseline_pure_ins_90s_r2"
    / "artifacts"
    / "cases"
    / "d2_baseline_pure_ins_90s"
    / "config_d2_baseline_pure_ins_90s.yaml"
)


def load_module():
    assert MODULE_PATH.exists(), f"missing KF-GINS pure INS compare runner: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_kf_gins_pure_ins_compare", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_build_kf_gins_pure_ins_config_maps_current_pure_ins_contract():
    module = load_module()
    source_cfg = load_yaml(SOURCE_CONFIG_PATH)

    cfg = module.build_kf_gins_pure_ins_config(
        source_cfg,
        imupath=Path(r"C:\temp\uwb_kf_gins_compare\inputs\IMU_converted.txt"),
        gnsspath=Path(r"C:\temp\uwb_kf_gins_compare\inputs\dummy_gnss.txt"),
        outputpath=Path(r"C:\temp\uwb_kf_gins_compare\kf_gins_output"),
    )

    assert cfg["imupath"] == r"C:\temp\uwb_kf_gins_compare\inputs\IMU_converted.txt"
    assert cfg["gnsspath"] == r"C:\temp\uwb_kf_gins_compare\inputs\dummy_gnss.txt"
    assert cfg["outputpath"] == r"C:\temp\uwb_kf_gins_compare\kf_gins_output"
    assert cfg["imudatalen"] == 7
    assert cfg["imudatarate"] == 200
    assert cfg["starttime"] == 528076.0
    assert cfg["endtime"] == 528166.0
    assert cfg["initpos"] == [30.3957018325, 114.2754286202, 28.9047]
    assert cfg["initvel"] == [-0.2748, -11.4603, -0.1198]
    assert cfg["initatt"] == [-0.10782, 0.62269, 268.28679]
    assert cfg["antlever"] == [0.15, -0.22, -1.15]

    assert cfg["initbgstd"] == [150.0, 112.5, 150.0]
    assert cfg["initbastd"] == [2250.0, 225.0, 112.5]
    assert cfg["initsgstd"] == [1500.0, 2250.0, 6000.0]
    assert cfg["initsastd"] == [3900.0, 3150.0, 1350.0]

    imunoise = cfg["imunoise"]
    expected_arw = 0.005 * 180.0 / math.pi * math.sqrt(3600.0)
    expected_vrw = 0.05 * math.sqrt(3600.0)
    expected_gbstd = 0.000233 * 180.0 / math.pi * 3600.0
    assert all(math.isclose(value, expected_arw, rel_tol=0.0, abs_tol=1.0e-12) for value in imunoise["arw"])
    assert all(math.isclose(value, expected_vrw, rel_tol=0.0, abs_tol=1.0e-12) for value in imunoise["vrw"])
    assert all(math.isclose(value, expected_gbstd, rel_tol=0.0, abs_tol=1.0e-12) for value in imunoise["gbstd"])
    assert imunoise["abstd"] == [50.0] * 3
    assert imunoise["gsstd"] == [500.0] * 3
    assert imunoise["asstd"] == [500.0] * 3
    assert imunoise["corrtime"] == 1.0


def test_write_dummy_gnss_file_emits_single_future_measurement(tmp_path):
    module = load_module()
    gnss_path = tmp_path / "dummy_gnss.txt"

    metadata = module.write_dummy_gnss_file(
        gnss_path,
        timestamp=528176.0,
        lat_deg=30.3957018325,
        lon_deg=114.2754286202,
        h_m=28.9047,
        std_xyz=(1.5, 2.5, 3.5),
    )

    rows = gnss_path.read_text(encoding="utf-8").strip().splitlines()
    assert len(rows) == 1
    assert rows[0].split() == [
        "528176.000000000",
        "30.395701833",
        "114.275428620",
        "28.904700000",
        "1.500000000",
        "2.500000000",
        "3.500000000",
    ]
    assert metadata["rows_written"] == 1
    assert metadata["timestamp"] == 528176.0


def test_compute_system_window_metrics_matches_current_pure_ins_baseline():
    module = load_module()
    manifest = json.loads(SOURCE_MANIFEST_PATH.read_text(encoding="utf-8"))
    err_frame = module.load_current_error_frame(REPO_ROOT / manifest["all_states_path"])

    rows = module.compute_system_window_metrics("current_solver", err_frame)
    metrics = {row["window"]: row for row in rows}

    assert set(metrics) == {"30s", "60s", "90s", "full"}
    assert math.isclose(metrics["30s"]["pos_final_3d_m"], 2.167589180379149, rel_tol=0.0, abs_tol=1.0e-6)
    assert math.isclose(metrics["60s"]["pos_final_3d_m"], 10.125879048722295, rel_tol=0.0, abs_tol=1.0e-6)
    assert math.isclose(metrics["90s"]["pos_final_3d_m"], 29.177695446955774, rel_tol=0.0, abs_tol=1.0e-6)
    assert math.isclose(metrics["30s"]["pos_rmse_3d_m"], 0.9365337160731082, rel_tol=0.0, abs_tol=1.0e-6)
    assert math.isclose(metrics["60s"]["pos_rmse_3d_m"], 4.23432309498754, rel_tol=0.0, abs_tol=1.0e-6)
    assert math.isclose(metrics["90s"]["pos_rmse_3d_m"], 11.710067937604634, rel_tol=0.0, abs_tol=1.0e-6)
