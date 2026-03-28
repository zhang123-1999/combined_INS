import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pandas as pd


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_kf_gins_outage_compare.py"
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
    assert MODULE_PATH.exists(), f"missing KF-GINS outage compare runner: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_kf_gins_outage_compare", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_build_kf_gins_config_maps_current_outage_reference_contract():
    module = load_module()
    source_cfg = load_yaml(SOURCE_CFG_PATH)

    cfg = module.build_kf_gins_config(
        source_cfg,
        imupath=Path(r"C:\temp\uwb_kf_gins_compare\inputs\IMU_converted.txt"),
        gnsspath=Path(r"C:\temp\uwb_kf_gins_compare\inputs\rtk_outage.txt"),
        outputpath=Path(r"C:\temp\uwb_kf_gins_compare\kf_gins_output"),
    )

    assert cfg["imupath"] == r"C:\temp\uwb_kf_gins_compare\inputs\IMU_converted.txt"
    assert cfg["gnsspath"] == r"C:\temp\uwb_kf_gins_compare\inputs\rtk_outage.txt"
    assert cfg["outputpath"] == r"C:\temp\uwb_kf_gins_compare\kf_gins_output"
    assert cfg["imudatalen"] == 7
    assert cfg["imudatarate"] == 200
    assert cfg["starttime"] == 528076.0
    assert cfg["endtime"] == 530488.9
    assert cfg["initpos"] == [30.3957018325, 114.2754286202, 28.9047]
    assert cfg["initvel"] == [-0.2748, -11.4603, -0.1198]
    assert cfg["initatt"] == [-0.10782, 0.62269, 268.28679]
    assert cfg["antlever"] == [0.15, -0.22, -1.15]

    imunoise = cfg["imunoise"]
    expected_arw = 1.454441043328608e-05 * 180.0 / math.pi * math.sqrt(3600.0)
    expected_vrw = 0.0016666666666666668 * math.sqrt(3600.0)
    assert imunoise["arw"] == [expected_arw] * 3
    assert imunoise["vrw"] == [expected_vrw] * 3
    assert imunoise["gbstd"] == [0.5] * 3
    assert imunoise["abstd"] == [25.0] * 3
    assert imunoise["gsstd"] == [300.0] * 3
    assert imunoise["asstd"] == [300.0] * 3
    assert imunoise["corrtime"] == 4.0

    assert cfg["initbgstd"] == [0.5] * 3
    assert cfg["initbastd"] == [25.0] * 3
    assert cfg["initsgstd"] == [300.0] * 3
    assert cfg["initsastd"] == [300.0] * 3


def test_filter_gnss_by_windows_keeps_only_enabled_samples(tmp_path):
    module = load_module()
    input_path = tmp_path / "rtk.txt"
    output_path = tmp_path / "rtk_filtered.txt"
    input_path.write_text(
        "\n".join(
            [
                "9 30.0 114.0 10.0 1 1 1",
                "10 30.0 114.0 10.0 1 1 1",
                "11 30.0 114.0 10.0 1 1 1",
                "12 30.0 114.0 10.0 1 1 1",
                "13 30.0 114.0 10.0 1 1 1",
                "20 30.0 114.0 10.0 1 1 1",
                "21 30.0 114.0 10.0 1 1 1",
                "22 30.0 114.0 10.0 1 1 1",
                "23 30.0 114.0 10.0 1 1 1",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    stats = module.filter_gnss_by_windows(
        input_path,
        output_path,
        enabled_windows=[(10.0, 12.0), (21.0, 22.0)],
    )

    kept = np.loadtxt(output_path)
    if kept.ndim == 1:
        kept = kept.reshape(1, -1)
    kept_t = kept[:, 0].tolist()

    assert kept_t == [10.0, 11.0, 12.0, 21.0, 22.0]
    assert stats["rows_raw"] == 9
    assert stats["rows_kept"] == 5


def test_load_kf_gins_navresult_reads_time_and_ecef_columns(tmp_path):
    module = load_module()
    nav_path = tmp_path / "KF_GINS_Navresult.nav"
    nav_path.write_text(
        "\n".join(
            [
                "2234 528076.0 30.3957018325 114.2754286202 28.9047 -0.2748 -11.4603 -0.1198 -0.10782 0.62269 268.28679",
                "2234 528077.0 30.3957019000 114.2754287000 28.9050 -0.2000 -11.4000 -0.1000 -0.10000 0.62000 268.30000",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    frame = module.load_kf_gins_navresult(nav_path)

    assert list(frame["timestamp"]) == [528076.0, 528077.0]
    assert list(frame["lat_deg"]) == [30.3957018325, 30.3957019]
    assert list(frame["lon_deg"]) == [114.2754286202, 114.2754287]
    assert {"ecef_x", "ecef_y", "ecef_z", "vn_mps", "ve_mps", "vd_mps", "roll_deg", "pitch_deg", "yaw_deg"}.issubset(
        frame.columns
    )
    assert frame["ecef_x"].notna().all()
    assert frame["ecef_y"].notna().all()
    assert frame["ecef_z"].notna().all()


def test_resolve_runtime_root_falls_back_to_ascii_when_probe_fails():
    module = load_module()

    chosen, mode = module.resolve_runtime_root(
        preferred_root=REPO_ROOT,
        fallback_root=Path(r"C:\temp\uwb_kf_gins_compare"),
        probe=lambda _path: False,
    )

    assert chosen == Path(r"C:\temp\uwb_kf_gins_compare")
    assert mode == "fallback_ascii"


def test_resolve_runtime_root_keeps_preferred_root_when_probe_succeeds():
    module = load_module()

    chosen, mode = module.resolve_runtime_root(
        preferred_root=REPO_ROOT,
        fallback_root=Path(r"C:\temp\uwb_kf_gins_compare"),
        probe=lambda _path: True,
    )

    assert chosen == REPO_ROOT
    assert mode == "preferred_root"

