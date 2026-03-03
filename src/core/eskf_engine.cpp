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
  Llh llh = EcefToLlh(state_.p);
  Matrix3d R_ne = RotNedToEcef(llh);
  Vector3d v_ned = R_ne.transpose() * state_.v;  // ECEF速度转NED速度

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

  // FEJ 传播雅可比中的比力近似：按参考 bias/scale 重新构造（仅用于雅可比）
  Vector3d f_b_fej = res.f_b;
  if (fej_ != nullptr && fej_->enabled && fej_->initialized && curr_imu_.dt > 1e-9) {
    Vector3d ba_ref = state_.ba;
    Vector3d sa_ref = state_.sa;
    if (fej_->use_layer2) {
      ba_ref = fej_->b_a_fej;
      sa_ref = fej_->s_a_fej;
    }
    Vector3d sf_a_ref = Vector3d::Ones() - sa_ref;
    Vector3d dvel_bc_ref = curr_imu_.dvel - ba_ref * curr_imu_.dt;
    f_b_fej = sf_a_ref.cwiseProduct(dvel_bc_ref) / curr_imu_.dt;
  }

  InsMech::BuildProcessModel(R_ne.transpose() * res.Cbn,  // C_b^n = R_e^n * C_b^e
                             res.f_b, omega_ib_b_corr, v_ned,
                             llh.lat, llh.h, curr_imu_.dt, noise_, Phi, Qd,
                             fej_, f_b_fej);
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
  if (!initialized_ || y.size() == 0) {
    return false;
  }
  if (H.cols() != kStateDim || H.rows() != y.size() || R.rows() != y.size() ||
      R.cols() != y.size()) {
    throw invalid_argument("measurement dimension mismatch");
  }

  // K = P H^T (H P H^T + R)^{-1}
  MatrixXd K = ComputeKalmanGain(H, R);
  ApplyUpdateMaskToKalmanGain(K, update_mask);
  if (K.rows() != kStateDim || K.cols() != y.size() || !K.allFinite()) {
    return false;
  }
  VectorXd dx = K * y;
  ApplyStateMaskToDx(dx, update_mask);

  // NaN/发散保护：如果修正量含 NaN 或姿态/位置修正异常，跳过本次更新
  if (!dx.allFinite() || dx.segment<3>(0).norm() > 1e6) {
    return false;
  }

  if (dx_out) {
    *dx_out = dx;
  }
  InjectErrorState(dx);
  UpdateCovarianceJoseph(K, H, R);
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
  Vector3d dr_ned = dx.segment<3>(0);  // NED位置误差
  Vector3d dv_ned = dx.segment<3>(3);  // NED速度误差
  Vector3d dphi_ned = dx.segment<3>(6);  // 姿态误差（NED系）
  Vector3d dba = dx.segment<3>(9);
  Vector3d dbg = dx.segment<3>(12);
  Vector3d dsg = dx.segment<3>(15);
  Vector3d dsa = dx.segment<3>(18);
  double dscale = dx(21);
  double d_mounting_roll = dx(22);
  double d_mounting_pitch = dx(23);
  double d_mounting_yaw = dx(24);
  Vector3d dlever = dx.segment<3>(25);

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

  // δr^e = R_n^e * δr^n
  Vector3d dr_ecef = R_ne * dr_ned;
  // δv^e = R_n^e * δv^n
  Vector3d dv_ecef = R_ne * dv_ned;

  // 误差注入（标准ESKF约定：δx = x_true - x̂，x_new = x̂ + δx）
  state_.p += dr_ecef;
  state_.v += dv_ecef;

  // 姿态修正：
  // 误差状态 φ 在 NED 系定义，四元数 state_.q 表示 body->ECEF。
  // 先将 φ 转到 ECEF 系，再进行左乘注入：
  //   C_be,new ≈ (I - (δφ_e×)) C_be
  Vector3d dphi_ecef = R_ne * dphi_ned;
  Vector4d dq = QuatFromSmallAngle(-dphi_ecef);
  state_.q = NormalizeQuat(QuatMultiply(dq, state_.q));

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
  Vector3d dgnss_lever = dx.segment<3>(28);
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
    apply_floor(i, pos_floor);        // position
    apply_floor(3 + i, vel_floor);    // velocity
    apply_floor(6 + i, att_floor);    // attitude
    apply_floor(12 + i, bg_floor);    // gyro bias
    apply_floor(22 + i, mounting_floor);  // mounting
  }

  P_ = 0.5 * (P_ + P_.transpose());
}
