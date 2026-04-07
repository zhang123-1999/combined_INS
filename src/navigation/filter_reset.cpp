#include "navigation/filter_engine.h"

#include <algorithm>
#include <cmath>

#include "utils/math_utils.h"

using namespace std;
using namespace Eigen;

void NavigationFilterEngine::InjectErrorState(const VectorXd &dx,
                                              const StateMask *update_mask) {
  Vector3d dr_ned = dx.segment<3>(StateIdx::kPos);
  Vector3d dv_ned = dx.segment<3>(StateIdx::kVel);
  Vector3d dphi_ned = dx.segment<3>(StateIdx::kAtt);
  Vector3d dba = dx.segment<3>(StateIdx::kBa);
  Vector3d dbg = dx.segment<3>(StateIdx::kBg);
  Vector3d dsg = dx.segment<3>(StateIdx::kSg);
  Vector3d dsa = dx.segment<3>(StateIdx::kSa);
  double dscale = dx(StateIdx::kOdoScale);
  double d_mounting_roll = dx(StateIdx::kMountRoll);
  double d_mounting_pitch = dx(StateIdx::kMountPitch);
  double d_mounting_yaw = dx(StateIdx::kMountYaw);
  Vector3d dlever = dx.segment<3>(StateIdx::kLever);

  if (correction_guard_.enabled) {
    dscale = clamp(dscale, -correction_guard_.max_odo_scale_step,
                   correction_guard_.max_odo_scale_step);
    d_mounting_roll =
        clamp(d_mounting_roll, -correction_guard_.max_mounting_step,
              correction_guard_.max_mounting_step);
    d_mounting_pitch =
        clamp(d_mounting_pitch, -correction_guard_.max_mounting_step,
              correction_guard_.max_mounting_step);
    d_mounting_yaw =
        clamp(d_mounting_yaw, -correction_guard_.max_mounting_step,
              correction_guard_.max_mounting_step);

    const double dlever_norm = dlever.norm();
    if (dlever_norm > correction_guard_.max_lever_arm_step &&
        dlever_norm > 1.0e-12) {
      dlever *= correction_guard_.max_lever_arm_step / dlever_norm;
    }
  }

  const FilterSemantics effective_semantics = ResolveEffectiveSemantics();
  const bool use_inekf =
      effective_semantics.flavor != FilterFlavor::kStandardEskf;

  const Llh llh = EcefToLlh(state_.p);
  const Matrix3d R_ne = RotNedToEcef(llh);
  const Matrix3d C_bn = R_ne.transpose() * QuatToRot(state_.q);

  if (use_inekf) {
    const Vector3d rho_p_body = dr_ned;
    const Vector3d rho_v_body = dv_ned;
    const Vector3d phi_body = dphi_ned;

    dr_ned = C_bn * rho_p_body;
    dv_ned = C_bn * rho_v_body;

    state_.p += R_ne * dr_ned;
    state_.v += R_ne * dv_ned;

    const Vector4d dq = QuatFromSmallAngle(phi_body);
    state_.q = NormalizeQuat(QuatMultiply(state_.q, dq));
  } else {
    state_.p += R_ne * dr_ned;
    state_.v += R_ne * dv_ned;

    const Vector3d dphi_ecef = R_ne * dphi_ned;
    // Preserve the historical standard-ESKF quaternion injection contract
    // used by the accepted data2/data5 baselines.
    const Vector4d dq = QuatFromSmallAngle(-dphi_ecef);
    state_.q = NormalizeQuat(QuatMultiply(dq, state_.q));
  }

  state_.ba += dba;
  state_.bg += dbg;
  state_.sg += dsg;
  state_.sa += dsa;
  state_.odo_scale += dscale;
  state_.mounting_roll += d_mounting_roll;
  state_.mounting_pitch += d_mounting_pitch;
  state_.mounting_yaw += d_mounting_yaw;
  state_.lever_arm += dlever;
  state_.gnss_lever_arm += dx.segment<3>(StateIdx::kGnssLever);

  if (!correction_guard_.enabled) {
    return;
  }

  if (IsStateEnabledByMasks(StateIdx::kOdoScale, update_mask) &&
      std::abs(dscale) > 0.0) {
    state_.odo_scale =
        clamp(state_.odo_scale, correction_guard_.odo_scale_min,
              correction_guard_.odo_scale_max);
  }
  if (IsStateEnabledByMasks(StateIdx::kMountRoll, update_mask) &&
      std::abs(d_mounting_roll) > 0.0) {
    state_.mounting_roll =
        clamp(state_.mounting_roll, -correction_guard_.max_mounting_roll,
              correction_guard_.max_mounting_roll);
  }
  if (IsStateEnabledByMasks(StateIdx::kMountPitch, update_mask) &&
      std::abs(d_mounting_pitch) > 0.0) {
    state_.mounting_pitch =
        clamp(state_.mounting_pitch, -correction_guard_.max_mounting_pitch,
              correction_guard_.max_mounting_pitch);
  }
  if (IsStateEnabledByMasks(StateIdx::kMountYaw, update_mask) &&
      std::abs(d_mounting_yaw) > 0.0) {
    state_.mounting_yaw =
        clamp(state_.mounting_yaw, -correction_guard_.max_mounting_yaw,
              correction_guard_.max_mounting_yaw);
  }

  bool lever_enabled = false;
  for (int axis = 0; axis < 3; ++axis) {
    lever_enabled =
        lever_enabled || IsStateEnabledByMasks(StateIdx::kLever + axis, update_mask);
  }
  const double lever_norm = state_.lever_arm.norm();
  if (lever_enabled && dlever.norm() > 0.0 &&
      lever_norm > correction_guard_.max_lever_arm_norm &&
      lever_norm > 1.0e-12) {
    state_.lever_arm *= correction_guard_.max_lever_arm_norm / lever_norm;
  }
}

Matrix<double, kStateDim, kStateDim>
NavigationFilterEngine::BuildInEkfResetGamma(const VectorXd &dx) const {
  Matrix<double, kStateDim, kStateDim> Gamma =
      Matrix<double, kStateDim, kStateDim>::Identity();
  if (inekf_ != nullptr && inekf_->debug_disable_true_reset_gamma) {
    return Gamma;
  }

  const Vector3d rho_p_body = dx.segment<3>(StateIdx::kPos);
  const Vector3d rho_v_body = dx.segment<3>(StateIdx::kVel);
  const Vector3d phi_body = dx.segment<3>(StateIdx::kAtt);
  const Matrix3d core_reset = QuatToRot(QuatFromSmallAngle(-phi_body));
  Gamma.block<3, 3>(StateIdx::kPos, StateIdx::kPos) = core_reset;
  Gamma.block<3, 3>(StateIdx::kVel, StateIdx::kVel) = core_reset;
  Gamma.block<3, 3>(StateIdx::kAtt, StateIdx::kAtt) = core_reset;
  Gamma.block<3, 3>(StateIdx::kPos, StateIdx::kAtt) =
      -Skew(rho_p_body) * core_reset;
  Gamma.block<3, 3>(StateIdx::kVel, StateIdx::kAtt) =
      -Skew(rho_v_body) * core_reset;
  return Gamma;
}

void NavigationFilterEngine::ApplyInEkfReset(const VectorXd &dx) {
  const Matrix<double, kStateDim, kStateDim> Gamma =
      BuildInEkfResetGamma(dx);
  P_ = Gamma * P_ * Gamma.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
  ApplyStateMaskToCov();
}

Matrix<double, kStateDim, kStateDim>
NavigationFilterEngine::BuildStandardEskfResetGamma(const VectorXd &dx) const {
  Matrix<double, kStateDim, kStateDim> Gamma =
      Matrix<double, kStateDim, kStateDim>::Identity();
  const Vector3d dphi_ned = dx.segment<3>(StateIdx::kAtt);
  Gamma.block<3, 3>(StateIdx::kAtt, StateIdx::kAtt) =
      Matrix3d::Identity() - 0.5 * Skew(dphi_ned);
  return Gamma;
}

void NavigationFilterEngine::ApplyStandardEskfReset(const VectorXd &dx) {
  const Matrix<double, kStateDim, kStateDim> Gamma =
      BuildStandardEskfResetGamma(dx);
  P_ = Gamma * P_ * Gamma.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
  ApplyStateMaskToCov();
}

bool NavigationFilterEngine::IsStateEnabledByMasks(
    int idx, const StateMask *update_mask) const {
  bool enabled = state_mask_[idx];
  if (update_mask != nullptr) {
    enabled = enabled && (*update_mask)[idx];
  }
  return enabled;
}

void NavigationFilterEngine::ApplyStateMaskToDx(
    VectorXd &dx, const StateMask *update_mask) const {
  for (int i = 0; i < kStateDim; ++i) {
    if (!IsStateEnabledByMasks(i, update_mask)) {
      dx(i) = 0.0;
    }
  }
}

void NavigationFilterEngine::ApplyUpdateMaskToKalmanGain(
    MatrixXd &K, const StateMask *update_mask) const {
  if (K.rows() != kStateDim) {
    return;
  }
  for (int i = 0; i < kStateDim; ++i) {
    if (!IsStateEnabledByMasks(i, update_mask)) {
      K.row(i).setZero();
    }
  }
}

void NavigationFilterEngine::ApplyGainScaleToKalmanGain(
    MatrixXd &K, const StateGainScale *gain_scale) const {
  if (gain_scale == nullptr || K.rows() != kStateDim) {
    return;
  }
  for (int i = 0; i < kStateDim; ++i) {
    double scale = (*gain_scale)[i];
    if (!std::isfinite(scale)) {
      scale = 1.0;
    }
    K.row(i) *= scale;
  }
}

void NavigationFilterEngine::ApplyElementGainScaleToKalmanGain(
    MatrixXd &K, const StateMeasurementGainScale *gain_element_scale) const {
  if (gain_element_scale == nullptr) {
    return;
  }
  if (gain_element_scale->rows() != K.rows() ||
      gain_element_scale->cols() != K.cols()) {
    return;
  }
  for (int i = 0; i < K.rows(); ++i) {
    for (int j = 0; j < K.cols(); ++j) {
      double scale = (*gain_element_scale)(i, j);
      if (!std::isfinite(scale)) {
        scale = 1.0;
      }
      K(i, j) *= scale;
    }
  }
}

void NavigationFilterEngine::ApplyStateMaskToCov() {
  for (int i = 0; i < kStateDim; ++i) {
    if (!state_mask_[i]) {
      P_.row(i).setZero();
      P_.col(i).setZero();
    }
  }
}

void NavigationFilterEngine::ApplyCovarianceFloor() {
  if (!covariance_floor_.enabled) {
    return;
  }

  const auto apply_floor = [&](int idx, double floor_var) {
    if (!state_mask_[idx]) {
      return;
    }
    if (!std::isfinite(P_(idx, idx)) || P_(idx, idx) < floor_var) {
      P_(idx, idx) = floor_var;
    }
  };

  const double pos_floor = max(0.0, covariance_floor_.pos_var);
  const double vel_floor = max(0.0, covariance_floor_.vel_var);
  const double att_floor = max(0.0, covariance_floor_.att_var);
  const double odo_scale_floor = max(0.0, covariance_floor_.odo_scale_var);
  const Vector3d lever_floor = covariance_floor_.lever_var.cwiseMax(0.0);
  const double mounting_floor = max(0.0, covariance_floor_.mounting_var);
  const double bg_floor = max(0.0, covariance_floor_.bg_var);

  apply_floor(StateIdx::kOdoScale, odo_scale_floor);
  for (int i = 0; i < 3; ++i) {
    apply_floor(StateIdx::kPos + i, pos_floor);
    apply_floor(StateIdx::kVel + i, vel_floor);
    apply_floor(StateIdx::kAtt + i, att_floor);
    apply_floor(StateIdx::kBg + i, bg_floor);
    apply_floor(StateIdx::kLever + i, lever_floor(i));
    apply_floor(StateIdx::kMountRoll + i, mounting_floor);
  }

  P_ = 0.5 * (P_ + P_.transpose());
}
