import importlib.util
from pathlib import Path
import sys

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

MODULE_PATH = REPO_ROOT / "plot_navresult.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing plot script: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("plot_navresult", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def build_minimal_new_format_frame():
    return pd.DataFrame(
        {
            "timestamp": [0.0, 1.0, 2.0],
            "time_sec": [0.0, 1.0, 2.0],
            "fused_x": [1.0, 2.0, 3.0],
            "fused_y": [2.0, 3.0, 4.0],
            "fused_z": [3.0, 4.0, 5.0],
            "fused_vx": [0.1, 0.2, 0.3],
            "fused_vy": [0.0, 0.1, 0.2],
            "fused_vz": [-0.1, -0.2, -0.3],
            "fused_roll": [0.1, 0.2, 0.3],
            "fused_pitch": [0.0, 0.1, 0.2],
            "fused_yaw": [1.0, 1.1, 1.2],
            "mounting_pitch": [0.0, 0.0, 0.0],
            "mounting_yaw": [0.0, 0.0, 0.0],
            "odo_scale": [1.0, 1.0, 1.0],
            "sg_x": [0.0, 0.0, 0.0],
            "sg_y": [0.0, 0.0, 0.0],
            "sg_z": [0.0, 0.0, 0.0],
            "sa_x": [0.0, 0.0, 0.0],
            "sa_y": [0.0, 0.0, 0.0],
            "sa_z": [0.0, 0.0, 0.0],
            "ba_x": [0.0, 0.0, 0.0],
            "ba_y": [0.0, 0.0, 0.0],
            "ba_z": [0.0, 0.0, 0.0],
            "bg_x": [0.0, 0.0, 0.0],
            "bg_y": [0.0, 0.0, 0.0],
            "bg_z": [0.0, 0.0, 0.0],
            "lever_x": [0.2, 0.2, 0.2],
            "lever_y": [-1.0, -1.0, -1.0],
            "lever_z": [0.6, 0.6, 0.6],
            "gnss_lever_x": [0.15, 0.15, 0.15],
            "gnss_lever_y": [-0.22, -0.22, -0.22],
            "gnss_lever_z": [-1.15, -1.15, -1.15],
            "lat": [30.0, 30.0001, 30.0002],
            "lon": [114.0, 114.0001, 114.0002],
            "alt": [10.0, 10.1, 10.2],
            "vn": [1.0, 1.1, 1.2],
            "ve": [0.1, 0.2, 0.3],
            "vd": [-0.1, -0.1, -0.1],
            "_format": ["new", "new", "new"],
        }
    )


def test_standard_state_files_drop_scale_factor_figures():
    module = load_module()

    assert module.STANDARD_STATE_FILES == [
        "state_gyro_bias.png",
        "state_accel_bias.png",
        "state_mounting_odo_scale.png",
        "state_odo_lever_arm.png",
        "state_gnss_lever_arm.png",
    ]


def test_legacy_mode_no_longer_writes_imu_scale_figures(tmp_path, monkeypatch):
    module = load_module()
    data = build_minimal_new_format_frame()
    stale_gyro_scale = tmp_path / "06_gyro_scale_factor.png"
    stale_accel_scale = tmp_path / "07_accel_scale_factor.png"
    stale_gyro_scale.write_text("stale", encoding="utf-8")
    stale_accel_scale.write_text("stale", encoding="utf-8")

    monkeypatch.setattr(module, "find_ref_file", lambda _: None)
    monkeypatch.setattr(module, "maybe_add_vehicle_velocity", lambda *_args, **_kwargs: None)

    module.run_legacy_mode(
        data,
        filepath="SOL_data2_baseline_current.txt",
        config_path=None,
        explicit_output_dir=str(tmp_path),
    )

    assert (tmp_path / "04_gyro_bias.png").exists()
    assert (tmp_path / "05_accel_bias.png").exists()
    assert not stale_gyro_scale.exists()
    assert not stale_accel_scale.exists()
