from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_current_gnss_update_debug_header_includes_velocity_covariance_and_gain_fields():
    source = (REPO_ROOT / "src" / "app" / "diagnostics.cpp").read_text(encoding="utf-8")

    required_tokens = [
        "prior_cov_vel_mat",
        "prior_cov_pos_vel_mat",
        "prior_cov_vel_att_mat",
        "k_vel_x_vec",
        "k_vel_y_vec",
        "k_vel_z_vec",
    ]

    for token in required_tokens:
        assert token in source


def test_kf_gins_gnss_update_debug_header_includes_velocity_covariance_and_gain_fields():
    source = (REPO_ROOT / "KF-GINS" / "src" / "kf-gins" / "gi_engine.cpp").read_text(
        encoding="utf-8"
    )
    header = (REPO_ROOT / "KF-GINS" / "src" / "kf-gins" / "gi_engine.h").read_text(
        encoding="utf-8"
    )

    required_cpp_tokens = [
        "prior_cov_vel_mat",
        "prior_cov_pos_vel_mat",
        "prior_cov_vel_att_mat",
        "k_vel_x_vec",
        "k_vel_y_vec",
        "k_vel_z_vec",
    ]
    required_header_tokens = [
        "prior_cov_vel",
        "prior_cov_pos_vel",
        "prior_cov_vel_att",
        "k_vel",
    ]

    for token in required_cpp_tokens:
        assert token in source
    for token in required_header_tokens:
        assert token in header
