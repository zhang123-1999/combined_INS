import importlib.util
from pathlib import Path
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_kf_gins_outage_roll_pitch_stepwise_audit.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing roll/pitch stepwise audit runner: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_kf_gins_outage_roll_pitch_stepwise_audit", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_predict_step_delta_acd_zero_when_roll_pitch_match():
    module = load_module()
    delta = module.predict_step_delta_acd(
        cur_roll_deg=1.0,
        cur_pitch_deg=-0.5,
        kf_roll_deg=1.0,
        kf_pitch_deg=-0.5,
        kf_yaw_deg=120.0,
        kf_step_acd=np.array([0.3, -0.2, -9.7], dtype=float),
        along_2d=np.array([1.0, 0.0], dtype=float),
        cross_2d=np.array([0.0, 1.0], dtype=float),
    )
    assert np.allclose(delta, np.zeros(3))


def test_accumulate_stepwise_prediction_only_counts_steps_with_local_attitude_gap():
    module = load_module()
    step_with_gap = {
        "cur_roll_deg": 0.0,
        "cur_pitch_deg": 1.0,
        "kf_roll_deg": 0.0,
        "kf_pitch_deg": 0.0,
        "kf_yaw_deg": 0.0,
        "kf_step_acd": np.array([0.0, 0.0, -1.0], dtype=float),
        "along_2d": np.array([1.0, 0.0], dtype=float),
        "cross_2d": np.array([0.0, 1.0], dtype=float),
    }
    step_without_gap = {
        "cur_roll_deg": 0.0,
        "cur_pitch_deg": 0.0,
        "kf_roll_deg": 0.0,
        "kf_pitch_deg": 0.0,
        "kf_yaw_deg": 0.0,
        "kf_step_acd": np.array([0.0, 0.0, -1.0], dtype=float),
        "along_2d": np.array([1.0, 0.0], dtype=float),
        "cross_2d": np.array([0.0, 1.0], dtype=float),
    }

    expected = module.predict_step_delta_acd(**step_with_gap)
    total = module.accumulate_stepwise_prediction([step_with_gap, step_without_gap])

    assert np.allclose(total, expected)
    assert total[0] < 0.0
    assert abs(total[1]) < 1e-12
