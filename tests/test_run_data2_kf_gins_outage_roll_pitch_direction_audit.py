import importlib.util
from pathlib import Path
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "run_data2_kf_gins_outage_roll_pitch_direction_audit.py"


def load_module():
    assert MODULE_PATH.exists(), f"missing roll/pitch direction audit runner: {MODULE_PATH}"
    spec = importlib.util.spec_from_file_location("run_data2_kf_gins_outage_roll_pitch_direction_audit", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_predict_roll_pitch_only_delta_ned_zero_when_roll_pitch_match():
    module = load_module()
    vec = np.array([0.3, -0.2, -9.7], dtype=float)
    delta = module.predict_roll_pitch_only_delta_ned(
        cur_roll_deg=1.0,
        cur_pitch_deg=-0.5,
        kf_roll_deg=1.0,
        kf_pitch_deg=-0.5,
        kf_yaw_deg=120.0,
        kf_vec_ned=vec,
    )
    assert np.allclose(delta, np.zeros(3))


def test_predict_roll_pitch_only_delta_ned_positive_pitch_pushes_south_for_down_vector():
    module = load_module()
    delta = module.predict_roll_pitch_only_delta_ned(
        cur_roll_deg=0.0,
        cur_pitch_deg=1.0,
        kf_roll_deg=0.0,
        kf_pitch_deg=0.0,
        kf_yaw_deg=0.0,
        kf_vec_ned=np.array([0.0, 0.0, -1.0], dtype=float),
    )
    assert delta[0] < 0.0
    assert abs(delta[1]) < 1e-12
