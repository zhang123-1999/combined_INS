#include "core/eskf.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace std;
using namespace Eigen;

/**
 * 构造 ESKF 引擎并保存噪声参数。
 * @param noise 过程与量测噪声配置
 */
EskfEngine::EskfEngine(const NoiseParams &noise) : noise_(noise) {
  state_mask_.fill(true);
}

/**
 * 设置初始名义状态与协方差。
 * @param state 初始状态（p/v/q/ba/bg）
 * @param P0 初始 15x15 协方差矩阵
 */
void EskfEngine::Initialize(const State &state,
                            const Matrix<double, kStateDim, kStateDim> &P0) {
  // 设定初值并标记引擎已就绪
  state_ = state;
  P_ = P0;
  if (fej_ != nullptr && fej_->enabled) {
    fej_->p_init_ecef = state.p;
  }
  ApplyStateMaskToCov();
  initialized_ = true;
}

/**
 * 缓存一帧 IMU 数据，维护 prev/curr 状态机。
 * @param imu 当前帧 IMU 增量
 */
void EskfEngine::AddImu(const ImuData &imu) {
  switch (imu_state_) {
    case ImuCacheState::Empty:
      curr_imu_ = imu;
      imu_state_ = ImuCacheState::HasCurr;
      break;
    case ImuCacheState::HasCurr:
    case ImuCacheState::Ready:
      prev_imu_ = curr_imu_;
      curr_imu_ = imu;
      if (curr_imu_.dt <= 0.0) {
        curr_imu_.dt = curr_imu_.t - prev_imu_.t;
      }
      imu_state_ = ImuCacheState::Ready;
      break;
  }
}

/**
 * 使用 prev/curr IMU 完成一次预测传播。
 * @return 传播是否成功
 */
bool EskfEngine::Predict() {
  if (!initialized_ || imu_state_ != ImuCacheState::Ready) {
    return false;
  }
  if (curr_imu_.dt <= 1e-9) {
    // 异常 dt 不参与传播
    return false;
  }

  // 惯导传播
  PropagationResult res = InsMech::Propagate(state_, prev_imu_, curr_imu_);

  // 计算NED速度和地理参数（用于NED系下的误差状态传播）
  Llh llh = EcefToLlh(res.state.p);
  Matrix3d R_ne = RotNedToEcef(llh);
  Vector3d v_ned = R_ne.transpose() * res.state.v;  // ECEF速度转NED速度

  // 惯导传播 + 过程噪声离散化（NED系下的完整误差模型）
  Matrix<double, kStateDim, kStateDim> Phi, Qd;

  // 过程模型中的 ω_ib^b 需要使用“零偏/比例因子修正后的陀螺角速度”
  Vector3d omega_ib_b_corr = Vector3d::Zero();
  if (curr_imu_.dt > 1e-9) {
    Vector3d omega_ib_b_raw = curr_imu_.dtheta / curr_imu_.dt;
    Vector3d omega_ib_unbiased = omega_ib_b_raw - state_.bg;
    Vector3d sf_g = Vector3d::Ones() - state_.sg;
    omega_ib_b_corr = sf_g.cwiseProduct(omega_ib_unbiased);
  }
  InsMech::BuildProcessModel(R_ne.transpose() * res.Cbn,  // C_b^n = R_e^n * C_b^e
                             res.f_b, omega_ib_b_corr, v_ned,
                             llh.lat, llh.h, curr_imu_.dt, noise_, Phi, Qd,
                             fej_);
  state_ = res.state;
  P_ = Phi * P_ * Phi.transpose() + Qd;
  P_ = 0.5 * (P_ + P_.transpose());
  ApplyStateMaskToCov();
  ApplyCovarianceFloor();
  return true;
}

/**
 * 通用量测更新。
 * @param y 残差向量
 * @param H 观测矩阵
 * @param R 量测噪声矩阵
 * @return 是否执行更新
 */
bool EskfEngine::Correct(const VectorXd &y, const MatrixXd &H,
                         const MatrixXd &R, VectorXd *dx_out,
                         const StateMask *update_mask) {
  last_true_iekf_correction_.valid = false;
  last_correction_debug_.valid = false;
  if (!initialized_ || y.size() == 0) {
    return false;
  }
  if (H.cols() != kStateDim || H.rows() != y.size() || R.rows() != y.size() ||
      R.cols() != y.size()) {
    throw invalid_argument("measurement dimension mismatch");
  }

  // K = P H^T (H P H^T + R)^{-1}
  Matrix<double, kStateDim, kStateDim> P_prior = P_;
  MatrixXd S = H * P_ * H.transpose() + R;
  MatrixXd K = ComputeKalmanGain(H, R);
  ApplyUpdateMaskToKalmanGain(K, update_mask);
  if (K.rows() != kStateDim || K.cols() != y.size() || !K.allFinite()) {
    return false;
  }
  VectorXd dx = K * y;
  ApplyStateMaskToDx(dx, update_mask);

  // NaN/发散保护：如果修正量含 NaN 或姿态/位置修正异常，跳过本次更新
  if (!dx.allFinite()) {
    return false;
  }
  if (dx.segment<3>(StateIdx::kPos).norm() > 1e6) {  // >1000km
    return false;
  }
  if (dx.segment<3>(StateIdx::kVel).norm() > 1e3) {  // >1000m/s
    return false;
  }
  if (dx.segment<3>(StateIdx::kAtt).norm() > EIGEN_PI) {  // >180deg
    return false;
  }

  if (dx_out) {
    *dx_out = dx;
  }
  bool use_true_iekf = (fej_ != nullptr && fej_->UseTrueInEkfMode());
  last_correction_debug_.valid = true;
  last_correction_debug_.used_true_iekf = use_true_iekf;
  last_correction_debug_.t_state = curr_imu_.t;
  last_correction_debug_.P_prior = P_prior;
  last_correction_debug_.y = y;
  last_correction_debug_.H = H;
  last_correction_debug_.R = R;
  last_correction_debug_.S = S;
  last_correction_debug_.K = K;
  last_correction_debug_.dx = dx;
  if (use_true_iekf) {
    UpdateCovarianceJoseph(K, H, R);
    last_true_iekf_correction_.t_state = curr_imu_.t;
    last_true_iekf_correction_.P_tilde = P_;
    last_true_iekf_correction_.dx = dx;
    last_true_iekf_correction_.covariance_floor_applied = false;
    InjectErrorState(dx);
    ApplyTrueInEkfReset(dx);
    last_true_iekf_correction_.P_after_reset = P_;
    if (fej_ != nullptr && fej_->apply_covariance_floor_after_reset) {
      ApplyCovarianceFloor();
      last_true_iekf_correction_.covariance_floor_applied = true;
    }
    last_true_iekf_correction_.P_after_all = P_;
    last_true_iekf_correction_.valid = true;
  } else {
    bool apply_standard_reset =
        (fej_ != nullptr && fej_->debug_enable_standard_reset_gamma);
    if (apply_standard_reset) {
      UpdateCovarianceJoseph(K, H, R);
      InjectErrorState(dx);
      ApplyStandardEskfReset(dx);
    } else {
      InjectErrorState(dx);
      UpdateCovarianceJoseph(K, H, R);
    }
  }
  return true;
}

void EskfEngine::SetStateMask(const StateMask &mask) {
  state_mask_ = mask;
  if (initialized_) {
    ApplyStateMaskToCov();
  }
}

/**
 * 计算卡尔曼增益 K。
 * @param H 观测矩阵
 * @param R 量测噪声矩阵
 * @return 卡尔曼增益矩阵
 */
MatrixXd EskfEngine::ComputeKalmanGain(const MatrixXd &H,
                                       const MatrixXd &R) const {
  MatrixXd S = H * P_ * H.transpose() + R;
  MatrixXd PHt = P_ * H.transpose();
  LDLT<MatrixXd> ldlt(S);
  if (ldlt.info() != Success) {
    return MatrixXd::Constant(kStateDim, H.rows(),
                              std::numeric_limits<double>::quiet_NaN());
  }
  // Solve S * X = (PHt)^T, then K = X^T
  MatrixXd X = ldlt.solve(PHt.transpose());
  if (ldlt.info() != Success || !X.allFinite()) {
    return MatrixXd::Constant(kStateDim, H.rows(),
                              std::numeric_limits<double>::quiet_NaN());
  }
  return X.transpose();
}

/**
 * 注入误差状态到名义状态。
 * 注意：dx 中的位置误差 δr^n 和速度误差 δv^n 是NED系下的，
 *       需要转换为ECEF系后再修正名义状态。
 *
 * @param dx 误差状态增量 [δr^n, δv^n, φ, dba, dbg, dsg, dsa, ...]
 */
void EskfEngine::InjectErrorState(const VectorXd &dx) {
  Vector3d dr_ned = dx.segment<3>(StateIdx::kPos);  // NED位置误差
  Vector3d dv_ned = dx.segment<3>(StateIdx::kVel);  // NED速度误差
  Vector3d dphi_ned = dx.segment<3>(StateIdx::kAtt);  // 姿态误差（NED系）
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
    dscale = std::clamp(dscale, -correction_guard_.max_odo_scale_step,
                        correction_guard_.max_odo_scale_step);
    d_mounting_roll =
        std::clamp(d_mounting_roll, -correction_guard_.max_mounting_step,
                   correction_guard_.max_mounting_step);
    d_mounting_pitch =
        std::clamp(d_mounting_pitch, -correction_guard_.max_mounting_step,
                   correction_guard_.max_mounting_step);
    d_mounting_yaw =
        std::clamp(d_mounting_yaw, -correction_guard_.max_mounting_step,
                   correction_guard_.max_mounting_step);

    double dlever_norm = dlever.norm();
    if (dlever_norm > correction_guard_.max_lever_arm_step && dlever_norm > 1e-12) {
      dlever *= (correction_guard_.max_lever_arm_step / dlever_norm);
    }
  }

  // 将NED误差转换为ECEF误差
  Llh llh = EcefToLlh(state_.p);
  Matrix3d R_ne = RotNedToEcef(llh);
  Matrix3d C_bn = R_ne.transpose() * QuatToRot(state_.q);
  bool use_inekf = (fej_ != nullptr && fej_->enabled);
  bool use_true_iekf = (fej_ != nullptr && fej_->UseTrueInEkfMode());
  if (use_true_iekf) {
    Vector3d rho_p_body = dr_ned;
    Vector3d rho_v_body = dv_ned;
    Vector3d phi_body = dphi_ned;

    dr_ned = C_bn * rho_p_body;
    dv_ned = C_bn * rho_v_body;

    Vector3d dr_ecef = R_ne * dr_ned;
    Vector3d dv_ecef = R_ne * dv_ned;
    state_.p += dr_ecef;
    state_.v += dv_ecef;

    Vector4d dq = QuatFromSmallAngle(phi_body);
    state_.q = NormalizeQuat(QuatMultiply(state_.q, dq));
  } else if (use_inekf) {
    // RI 误差坐标 -> 加法误差坐标：
    // δv = ξ_v - (v^n ×)ξ_φ, δp = ξ_p - (p^n_local ×)ξ_φ
    Vector3d v_ned_nom = R_ne.transpose() * state_.v;
    dv_ned -= Skew(v_ned_nom) * dphi_ned;
    if (fej_->ri_inject_pos_inverse) {
      Vector3d p_ned_local = R_ne.transpose() * (state_.p - fej_->p_init_ecef);
      dr_ned -= Skew(p_ned_local) * dphi_ned;
    }
  }

  if (!use_true_iekf) {
    // δr^e = R_n^e * δr^n
    Vector3d dr_ecef = R_ne * dr_ned;
    // δv^e = R_n^e * δv^n
    Vector3d dv_ecef = R_ne * dv_ned;

    // 误差注入（标准ESKF约定：δx = x_true - x̂，x_new = x̂ + δx）
    state_.p += dr_ecef;
    state_.v += dv_ecef;

    // InEKF Right-Invariant 姿态注入（左乘）：
    // q_new = Exp(dphi_ecef) ⊗ q
    // 标准 ESKF（右乘等效写法）：
    // q_new = q ⊗ Exp(-dphi_ecef)
    // 误差状态 φ 在 NED 系定义，先转到 ECEF 后注入。
    Vector3d dphi_ecef = R_ne * dphi_ned;
    if (use_inekf) {
      Vector4d dq = QuatFromSmallAngle(dphi_ecef);
      state_.q = NormalizeQuat(QuatMultiply(dq, state_.q));
    } else {
      Vector4d dq = QuatFromSmallAngle(-dphi_ecef);
      state_.q = NormalizeQuat(QuatMultiply(dq, state_.q));
    }
  }

  // IMU误差反馈（零偏和比例因子误差定义为：b̂ = b + δb）
  state_.ba += dba;
  state_.bg += dbg;
  state_.sg += dsg;
  state_.sa += dsa;
  state_.odo_scale += dscale;
  state_.mounting_roll += d_mounting_roll;
  state_.mounting_pitch += d_mounting_pitch;
  state_.mounting_yaw += d_mounting_yaw;
  state_.lever_arm += dlever;

  // GNSS杆臂误差注入
  Vector3d dgnss_lever = dx.segment<3>(StateIdx::kGnssLever);
  state_.gnss_lever_arm += dgnss_lever;

  if (correction_guard_.enabled) {
    state_.odo_scale = std::clamp(state_.odo_scale, correction_guard_.odo_scale_min,
                                  correction_guard_.odo_scale_max);
    state_.mounting_roll =
        std::clamp(state_.mounting_roll, -correction_guard_.max_mounting_roll,
                   correction_guard_.max_mounting_roll);
    state_.mounting_pitch =
        std::clamp(state_.mounting_pitch, -correction_guard_.max_mounting_pitch,
                   correction_guard_.max_mounting_pitch);
    state_.mounting_yaw =
        std::clamp(state_.mounting_yaw, -correction_guard_.max_mounting_yaw,
                   correction_guard_.max_mounting_yaw);

    double lever_norm = state_.lever_arm.norm();
    if (lever_norm > correction_guard_.max_lever_arm_norm && lever_norm > 1e-12) {
      state_.lever_arm *= (correction_guard_.max_lever_arm_norm / lever_norm);
    }
  }

}

/**
 * Joseph 形式更新协方差，保证数值稳定性。
 * @param K 卡尔曼增益
 * @param H 观测矩阵
 * @param R 量测噪声矩阵
 */
void EskfEngine::UpdateCovarianceJoseph(const MatrixXd &K, const MatrixXd &H,
                                        const MatrixXd &R) {
  Matrix<double, kStateDim, kStateDim> I = Matrix<double, kStateDim, kStateDim>::Identity();
  // Joseph 形式保证协方差对称正定
  Matrix<double, kStateDim, kStateDim> A = I - K * H;
  P_ = A * P_ * A.transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
  ApplyStateMaskToCov();
}

Matrix<double, kStateDim, kStateDim> EskfEngine::BuildTrueInEkfResetGamma(
    const VectorXd &dx) const {
  Matrix<double, kStateDim, kStateDim> Gamma =
      Matrix<double, kStateDim, kStateDim>::Identity();
  if (fej_ != nullptr && fej_->debug_disable_true_reset_gamma) {
    return Gamma;
  }
  Vector3d rho_p_body = dx.segment<3>(StateIdx::kPos);
  Vector3d rho_v_body = dx.segment<3>(StateIdx::kVel);
  Vector3d phi_body = dx.segment<3>(StateIdx::kAtt);
  Matrix3d core_reset = QuatToRot(QuatFromSmallAngle(-phi_body));
  Gamma.block<3, 3>(StateIdx::kPos, StateIdx::kPos) = core_reset;
  Gamma.block<3, 3>(StateIdx::kVel, StateIdx::kVel) = core_reset;
  Gamma.block<3, 3>(StateIdx::kAtt, StateIdx::kAtt) = core_reset;
  Gamma.block<3, 3>(StateIdx::kPos, StateIdx::kAtt) =
      -Skew(rho_p_body) * core_reset;
  Gamma.block<3, 3>(StateIdx::kVel, StateIdx::kAtt) =
      -Skew(rho_v_body) * core_reset;
  return Gamma;
}

void EskfEngine::ApplyTrueInEkfReset(const VectorXd &dx) {
  Matrix<double, kStateDim, kStateDim> Gamma = BuildTrueInEkfResetGamma(dx);
  P_ = Gamma * P_ * Gamma.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
  ApplyStateMaskToCov();
}

Matrix<double, kStateDim, kStateDim> EskfEngine::BuildStandardEskfResetGamma(
    const VectorXd &dx) const {
  Matrix<double, kStateDim, kStateDim> Gamma =
      Matrix<double, kStateDim, kStateDim>::Identity();
  const Vector3d dphi_ned = dx.segment<3>(StateIdx::kAtt);
  Gamma.block<3, 3>(StateIdx::kAtt, StateIdx::kAtt) =
      Matrix3d::Identity() - 0.5 * Skew(dphi_ned);
  return Gamma;
}

void EskfEngine::ApplyStandardEskfReset(const VectorXd &dx) {
  Matrix<double, kStateDim, kStateDim> Gamma = BuildStandardEskfResetGamma(dx);
  P_ = Gamma * P_ * Gamma.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
  ApplyStateMaskToCov();
}

bool EskfEngine::IsStateEnabledByMasks(int idx, const StateMask *update_mask) const {
  bool enabled = state_mask_[idx];
  if (update_mask != nullptr) {
    enabled = enabled && (*update_mask)[idx];
  }
  return enabled;
}

void EskfEngine::ApplyStateMaskToDx(VectorXd &dx, const StateMask *update_mask) const {
  for (int i = 0; i < kStateDim; ++i) {
    if (!IsStateEnabledByMasks(i, update_mask)) {
      dx(i) = 0.0;
    }
  }
}

void EskfEngine::ApplyUpdateMaskToKalmanGain(MatrixXd &K,
                                             const StateMask *update_mask) const {
  if (K.rows() != kStateDim) {
    return;
  }
  for (int i = 0; i < kStateDim; ++i) {
    if (!IsStateEnabledByMasks(i, update_mask)) {
      K.row(i).setZero();
    }
  }
}

void EskfEngine::ApplyStateMaskToCov() {
  for (int i = 0; i < kStateDim; ++i) {
    if (!state_mask_[i]) {
      P_.row(i).setZero();
      P_.col(i).setZero();
    }
  }
}

void EskfEngine::ApplyCovarianceFloor() {
  if (!covariance_floor_.enabled) {
    return;
  }

  auto apply_floor = [&](int idx, double floor_var) {
    if (!state_mask_[idx]) {
      return;
    }
    if (!std::isfinite(P_(idx, idx)) || P_(idx, idx) < floor_var) {
      P_(idx, idx) = floor_var;
    }
  };

  const double pos_floor = std::max(0.0, covariance_floor_.pos_var);
  const double vel_floor = std::max(0.0, covariance_floor_.vel_var);
  const double att_floor = std::max(0.0, covariance_floor_.att_var);
  const double mounting_floor = std::max(0.0, covariance_floor_.mounting_var);
  const double bg_floor = std::max(0.0, covariance_floor_.bg_var);

  for (int i = 0; i < 3; ++i) {
    apply_floor(StateIdx::kPos + i, pos_floor);        // position
    apply_floor(StateIdx::kVel + i, vel_floor);        // velocity
    apply_floor(StateIdx::kAtt + i, att_floor);        // attitude
    apply_floor(StateIdx::kBg + i, bg_floor);          // gyro bias
    apply_floor(StateIdx::kMountRoll + i, mounting_floor);  // mounting
  }

  P_ = 0.5 * (P_ + P_.transpose());
}
