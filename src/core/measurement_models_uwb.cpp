#include "core/uwb.h"
#include "utils/math_utils.h"
#include <iostream>

using namespace std;
using namespace Eigen;

namespace MeasModels {

namespace {

bool UseTrueInEkf(const FejManager *fej) {
  return fej != nullptr && fej->UseTrueInEkfMode();
}

bool UseHybridInEkf(const FejManager *fej) {
  return fej != nullptr && fej->enabled && !fej->true_iekf_mode;
}

void TransformAdditiveCoreJacobianToTrueInEkf(MatrixXd &H,
                                              const Matrix3d &C_bn) {
  if (H.cols() < StateIdx::kAtt + 3) {
    return;
  }
  MatrixXd H_pos = H.block(0, StateIdx::kPos, H.rows(), 3).eval();
  MatrixXd H_vel = H.block(0, StateIdx::kVel, H.rows(), 3).eval();
  MatrixXd H_att = H.block(0, StateIdx::kAtt, H.rows(), 3).eval();

  H.block(0, StateIdx::kPos, H.rows(), 3) = H_pos * C_bn;
  H.block(0, StateIdx::kVel, H.rows(), 3) = H_vel * C_bn;
  H.block(0, StateIdx::kAtt, H.rows(), 3) = H_att * (-C_bn);
}

}  // namespace

// === InEKF / Right-Invariant 误差约定说明 ===
// 误差定义：η = X_tilde * X^{-1}（右不变误差）。
// 姿态注入见 EskfEngine::InjectErrorState：
//   InEKF: q_new = Exp(dphi_ecef) ⊗ q
//   ESKF : q_new = q ⊗ Exp(-dphi_ecef)
//
// 但 InEKF 并非“仅姿态列翻符号”：
// - ODO/NHC 速度约束在 RI 坐标下姿态列解耦（置零）
// - GNSS 位置/速度模型需补充 RI 坐标变换带来的耦合项

/**
 * 计算 UWB 多基站量测模型。
 * 标准ESKF约定：y = z - h，H = ∂h/∂x
 */
UwbModel ComputeUwbModel(const State &state, const VectorXd &z,
                         const MatrixXd &anchors,
                         double sigma_uwb) {
  int num_anchors = anchors.rows();
  if (z.size() != num_anchors) {
  }

  UwbModel model;
  model.H.setZero(num_anchors, kStateDim);
  VectorXd h(num_anchors);

  for (int i = 0; i < num_anchors; ++i) {
    Vector3d r = state.p - anchors.row(i).transpose();
    double dist = r.norm();
    h[i] = dist;
    if (dist > 1e-9) {
      model.H.block<1, 3>(i, StateIdx::kPos) = r.transpose() / dist;
    }
  }

  // 标准残差 y = z - h
  model.y = z - h;
  model.R = MatrixXd::Identity(num_anchors, num_anchors) * (sigma_uwb * sigma_uwb);
  return model;
}

/**
 * 零速更新（ZUPT）量测模型。
 * 标准ESKF约定：y = z - h = 0 - v^n = -v^n
 */
VelConstraintModel ComputeZuptModel(const State &state, double sigma_zupt) {
  VelConstraintModel model;

  Llh llh = EcefToLlh(state.p);
  Matrix3d R_ne = RotNedToEcef(llh);
  Vector3d v_ned = R_ne.transpose() * state.v;

  // y = z - h = 0 - v_ned = -v_ned
  model.y = -v_ned;

  model.H.setZero(3, kStateDim);
  model.H.block<3, 3>(0, StateIdx::kVel) = Matrix3d::Identity();

  model.R = Matrix3d::Identity() * (sigma_zupt * sigma_zupt);
  return model;
}

/**
 * 非完整约束（NHC）量测模型。
 * 标准ESKF约定：y = z - h = 0 - v_v = -v_v
 * H 使用标准雅可比 ∂h/∂x
 */
VelConstraintModel ComputeNhcModel(const State &state, const Matrix3d &C_b_v,
                                   const Vector3d &omega_ib_b_raw,
                                   double sigma_nhc_y, double sigma_nhc_z,
                                   const FejManager *fej) {
  bool use_inekf = (fej != nullptr && fej->enabled);
  bool use_true_iekf = UseTrueInEkf(fej);
  bool use_hybrid_inekf = UseHybridInEkf(fej);
  VelConstraintModel model;

  Llh llh = EcefToLlh(state.p);
  Matrix3d R_ne = RotNedToEcef(llh);
  Matrix3d C_bn_nom = R_ne.transpose() * QuatToRot(state.q);
  Matrix3d C_bn = C_bn_nom;

  Vector3d v_ned = R_ne.transpose() * state.v;
  Vector3d v_b = C_bn.transpose() * v_ned;
  Vector3d omega_ie_n = OmegaIeNed(llh.lat);
  Vector3d omega_en_n = OmegaEnNed(v_ned, llh.lat, llh.h);
  Vector3d omega_in_n = omega_ie_n + omega_en_n;
  Vector3d omega_ib_unbiased = omega_ib_b_raw - state.bg;
  Vector3d sf_g = Vector3d::Ones() - state.sg;
  Vector3d omega_ib_corr = sf_g.cwiseProduct(omega_ib_unbiased);
  Vector3d omega_nb_b = omega_ib_corr - C_bn.transpose() * omega_in_n;

  const Vector3d &lever_arm = state.lever_arm;
  Vector3d v_wheel_b = v_b + omega_nb_b.cross(lever_arm);
  Vector3d v_v = C_b_v * v_wheel_b;

  // y = z - h = 0 - (v_v.y, v_v.z)
  model.y.resize(2);
  model.y << -v_v.y(), -v_v.z();

  model.H.setZero(2, kStateDim);

  // H_v = C_b^v * C_n^b
  Matrix3d H_v = C_b_v * C_bn.transpose();

  Matrix3d H_theta = Matrix3d::Zero();
  if (use_hybrid_inekf) {
    // RI 坐标下速度类约束与姿态误差解耦
    H_theta.setZero();
  } else {
    // 标准 ESKF
    H_theta = -C_b_v * Skew(v_b) * C_bn.transpose();
  }

  // H_bg = C_b^v * Skew(l) * diag(1-sg)
  Matrix3d H_bg = C_b_v * Skew(lever_arm) * sf_g.asDiagonal();
  // H_sg = C_b^v * Skew(l) * diag(ω_ib^b-bg)
  Matrix3d H_sg = C_b_v * Skew(lever_arm) * omega_ib_unbiased.asDiagonal();

  model.H.block<2, 3>(0, StateIdx::kVel) = H_v.block<2, 3>(1, 0);
  model.H.block<2, 3>(0, StateIdx::kAtt) = H_theta.block<2, 3>(1, 0);
  model.H.block<2, 3>(0, StateIdx::kBg) = H_bg.block<2, 3>(1, 0);
  model.H.block<2, 3>(0, StateIdx::kSg) = H_sg.block<2, 3>(1, 0);

  // H_alpha: 安装角微小旋转对 v_v 的影响
  // 安装角 pitch 对应绕 vehicle-Y 轴旋转，mounting_yaw 对应绕 vehicle-Z 轴旋转
  // ∂v_v / ∂mounting_pitch = e_Y × v_v,  e_Y = [0,1,0] 在 vehicle 系
  // ∂v_v / ∂mounting_yaw  = e_Z × v_v,  e_Z = [0,0,1] 在 vehicle 系
  Vector3d e_pitch_v(0.0, 1.0, 0.0);  // vehicle 系 Y 轴
  Vector3d e_yaw_v(0.0, 0.0, 1.0);   // vehicle 系 Z 轴
  Vector3d dv_dpitch = e_pitch_v.cross(v_v);  // ∂v_v/∂mounting_pitch
  Vector3d dv_dyaw = e_yaw_v.cross(v_v);      // ∂v_v/∂mounting_yaw
  // NHC 取 y 分量（行0）和 z 分量（行1）
  model.H(0, StateIdx::kMountPitch) = dv_dpitch(1);
  model.H(0, StateIdx::kMountYaw) = dv_dyaw(1);
  model.H(1, StateIdx::kMountPitch) = dv_dpitch(2);
  model.H(1, StateIdx::kMountYaw) = dv_dyaw(2);

  // H_lever = C_b^v * Skew(ω_nb^b)
  Matrix3d H_lever = C_b_v * Skew(omega_nb_b);
  model.H.block<2, 3>(0, StateIdx::kLever) = H_lever.block<2, 3>(1, 0);

  // 补充：杆臂通过 omega_nb_b 对姿态误差的间接耦合
  // δ(C_n^b * omega_in^n) / δφ ≈ C_n^b * Skew(omega_in^n) 的贡献经杆臂传播
  // InEKF (Right-Invariant): 姿态误差列相对加法误差模型取反
  // 注：仅当 lever_arm.norm() > 0.05m 时该项才有显著影响
  if (!use_hybrid_inekf && lever_arm.norm() > 1e-3) {
    Matrix3d H_theta_lever =
        -C_b_v * Skew(lever_arm) * C_bn.transpose() * Skew(omega_in_n);
    model.H.block<2, 3>(0, StateIdx::kAtt) += H_theta_lever.block<2, 3>(1, 0);
  }

  if (use_true_iekf) {
    TransformAdditiveCoreJacobianToTrueInEkf(model.H, C_bn);
  }

  model.R = Matrix2d::Zero();
  model.R(0, 0) = sigma_nhc_y * sigma_nhc_y;
  model.R(1, 1) = sigma_nhc_z * sigma_nhc_z;
  return model;
}

/**
 * 里程计（ODO）前向速度约束。
 * 标准ESKF约定：y = z - h = odo_speed - pred_reading
 */
VelConstraintModel ComputeOdoModel(const State &state, double odo_speed,
                                   const Matrix3d &C_b_v,
                                   const Vector3d &omega_ib_b_raw,
                                   double sigma_odo,
                                   const FejManager *fej) {
  bool use_inekf = (fej != nullptr && fej->enabled);
  bool use_true_iekf = UseTrueInEkf(fej);
  bool use_hybrid_inekf = UseHybridInEkf(fej);
  VelConstraintModel model;

  Llh llh = EcefToLlh(state.p);
  Matrix3d R_ne = RotNedToEcef(llh);
  Matrix3d C_bn_nom = R_ne.transpose() * QuatToRot(state.q);
  Matrix3d C_bn = C_bn_nom;

  Vector3d v_ned = R_ne.transpose() * state.v;
  Vector3d v_b = C_bn.transpose() * v_ned;
  Vector3d omega_ie_n = OmegaIeNed(llh.lat);
  Vector3d omega_en_n = OmegaEnNed(v_ned, llh.lat, llh.h);
  Vector3d omega_in_n = omega_ie_n + omega_en_n;
  Vector3d omega_ib_unbiased = omega_ib_b_raw - state.bg;
  Vector3d sf_g = Vector3d::Ones() - state.sg;
  Vector3d omega_ib_corr = sf_g.cwiseProduct(omega_ib_unbiased);
  Vector3d omega_nb_b = omega_ib_corr - C_bn.transpose() * omega_in_n;

  const Vector3d &lever_arm = state.lever_arm;
  Vector3d v_wheel_b = v_b + omega_nb_b.cross(lever_arm);
  Vector3d v_phys_v = C_b_v * v_wheel_b;

  double pred_reading = state.odo_scale * v_phys_v.x();

  // y = z - h = odo_speed - pred_reading
  model.y.resize(1);
  model.y(0) = odo_speed - pred_reading;

  model.H.setZero(1, kStateDim);

  double s = state.odo_scale;

  // 1. H_v
  RowVector3d H_v_phys = (C_b_v * C_bn.transpose()).row(0);
  model.H.block<1, 3>(0, StateIdx::kVel) = s * H_v_phys;

  Matrix3d H_theta_full = Matrix3d::Zero();
  if (use_hybrid_inekf) {
    // RI 坐标下，速度类约束的姿态列解耦
    H_theta_full.setZero();
  } else {
    // 标准 ESKF
    H_theta_full = -C_b_v * Skew(v_b) * C_bn.transpose();
  }
  model.H.block<1, 3>(0, StateIdx::kAtt) = s * H_theta_full.row(0);

  // 3. H_bg = C_b^v * Skew(l) * diag(1-sg)
  RowVector3d H_bg_phys =
      C_b_v.row(0) * Skew(lever_arm) * sf_g.asDiagonal();
  model.H.block<1, 3>(0, StateIdx::kBg) = s * H_bg_phys;

  // 3b. H_sg = C_b^v * Skew(l) * diag(ω_ib^b-bg)
  RowVector3d H_sg_phys =
      C_b_v.row(0) * Skew(lever_arm) * omega_ib_unbiased.asDiagonal();
  model.H.block<1, 3>(0, StateIdx::kSg) = s * H_sg_phys;

  // 4. H_scale = v_phys_x（标准雅可比）
  model.H(0, StateIdx::kOdoScale) = v_phys_v.x();

  // 5. H_alpha: 安装角微小旋转对 v_phys_v 前向分量的影响
  // ∂v_phys_v.x() / ∂mounting_pitch = (e_Y × v_phys_v).x()
  // ∂v_phys_v.x() / ∂mounting_yaw   = (e_Z × v_phys_v).x()
  Vector3d e_pitch_v(0.0, 1.0, 0.0);
  Vector3d e_yaw_v(0.0, 0.0, 1.0);
  Vector3d dv_dpitch = e_pitch_v.cross(v_phys_v);
  Vector3d dv_dyaw = e_yaw_v.cross(v_phys_v);
  model.H(0, StateIdx::kMountPitch) = s * dv_dpitch(0);
  model.H(0, StateIdx::kMountYaw) = s * dv_dyaw(0);

  // 6. H_lever = C_b^v * Skew(ω_nb^b)
  RowVector3d H_lever_phys = C_b_v.row(0) * Skew(omega_nb_b);
  model.H.block<1, 3>(0, StateIdx::kLever) = s * H_lever_phys;

  model.R.resize(1, 1);
  model.R(0, 0) = sigma_odo * sigma_odo;

  if (use_true_iekf) {
    TransformAdditiveCoreJacobianToTrueInEkf(model.H, C_bn);
  }

  return model;
}

/**
 * GNSS位置量测模型（NED系下）。
 * 标准ESKF约定：y = z - h，H = ∂h/∂x
 */
UwbModel ComputeGnssPositionModel(const State &state, const Vector3d &z_ecef,
                                  const Vector3d &sigma_gnss,
                                  const FejManager *fej) {
  bool use_inekf = (fej != nullptr && fej->enabled);
  bool use_true_iekf = UseTrueInEkf(fej);
  UwbModel model;

  Llh llh = EcefToLlh(state.p);
  Matrix3d R_ne = RotNedToEcef(llh);
  Matrix3d C_bn = R_ne.transpose() * QuatToRot(state.q);

  if (use_true_iekf) {
    Vector3d d_ecef = z_ecef - state.p;
    Vector3d d_ned = R_ne.transpose() * d_ecef;
    Vector3d d_body = C_bn.transpose() * d_ned;

    model.y = d_body - state.gnss_lever_arm;
    model.H.setZero(3, kStateDim);
    model.H.block<3, 3>(0, StateIdx::kPos) = Matrix3d::Identity();
    model.H.block<3, 3>(0, StateIdx::kAtt) = -Skew(state.gnss_lever_arm);
    model.H.block<3, 3>(0, StateIdx::kGnssLever) = Matrix3d::Identity();

    Matrix3d R_ecef = (sigma_gnss.cwiseProduct(sigma_gnss)).asDiagonal();
    model.R = C_bn.transpose() * R_ne.transpose() * R_ecef * R_ne * C_bn;
    return model;
  }

  Vector3d lever_ned = C_bn * state.gnss_lever_arm;
  Vector3d p_pred_ecef = state.p + R_ne * lever_ned;

  Vector3d dr_ecef = p_pred_ecef - z_ecef;
  Vector3d dr_ned = R_ne.transpose() * dr_ecef;

  // y = z - h = -(h - z) = -dr_ned
  model.y = -dr_ned;

  model.H.setZero(3, kStateDim);
  model.H.block<3, 3>(0, StateIdx::kPos) = Matrix3d::Identity();
  if (use_inekf) {
    if (fej->ri_gnss_pos_use_p_ned_local) {
      // RI 坐标: H_phi = -(Skew(p_ned_local) + Skew(lever_ned))
      Vector3d p_ned_local = R_ne.transpose() * (state.p - fej->p_init_ecef);
      model.H.block<3, 3>(0, StateIdx::kAtt) =
          -(Skew(p_ned_local) + Skew(lever_ned));
    } else {
      // A/B 开关：退化为仅杠杆臂耦合。
      model.H.block<3, 3>(0, StateIdx::kAtt) = -Skew(lever_ned);
    }
  } else {
    // 标准 ESKF
    model.H.block<3, 3>(0, StateIdx::kAtt) = Skew(lever_ned);
  }
  model.H.block<3, 3>(0, StateIdx::kGnssLever) = C_bn;

  model.R = (sigma_gnss.cwiseProduct(sigma_gnss)).asDiagonal();

  return model;
}

/**
 * GNSS速度量测模型（NED系下）。
 * 标准ESKF约定：y = z - h，H = ∂h/∂x
 */
UwbModel ComputeGnssVelocityModel(const State &state, const Vector3d &z_gnss_vel_ecef,
                                  const Vector3d &omega_ib_b_raw,
                                  const Vector3d &sigma_gnss_vel,
                                  const FejManager *fej) {
  bool use_inekf = (fej != nullptr && fej->enabled);
  bool use_true_iekf = UseTrueInEkf(fej);
  bool use_hybrid_inekf = UseHybridInEkf(fej);
  UwbModel model;

  Llh llh = EcefToLlh(state.p);
  Matrix3d R_ne = RotNedToEcef(llh);
  Matrix3d C_bn = R_ne.transpose() * QuatToRot(state.q);

  Vector3d v_ned = R_ne.transpose() * state.v;

  Vector3d omega_ie_n = OmegaIeNed(llh.lat);
  Vector3d omega_en_n = OmegaEnNed(v_ned, llh.lat, llh.h);
  Vector3d omega_in_n = omega_ie_n + omega_en_n;
  Vector3d omega_ib_unbiased = omega_ib_b_raw - state.bg;
  Vector3d sf_g = Vector3d::Ones() - state.sg;
  Vector3d omega_ib_corr = sf_g.cwiseProduct(omega_ib_unbiased);
  Vector3d omega_nb_b = omega_ib_corr - C_bn.transpose() * omega_in_n;

  Vector3d lever_vel_b = Skew(omega_nb_b) * state.gnss_lever_arm;
  Vector3d lever_vel_n = C_bn * lever_vel_b;

  Vector3d v_pred_ned = v_ned + lever_vel_n;
  Vector3d z_gnss_vel_ned = R_ne.transpose() * z_gnss_vel_ecef;

  // y = z - h = obs - pred
  model.y = z_gnss_vel_ned - v_pred_ned;

  model.H.setZero(3, kStateDim);

  // H_v = I_3
  model.H.block<3, 3>(0, StateIdx::kVel) = Matrix3d::Identity();

  // H_φ 基础项（加法误差坐标下）
  Matrix3d H_phi_1 = -Skew(omega_in_n) * Skew(C_bn * state.gnss_lever_arm);
  Vector3d Cb_l_cross_omega_H = C_bn * Skew(state.gnss_lever_arm) * omega_ib_corr;
  Matrix3d H_phi_2 = -Skew(Cb_l_cross_omega_H);
  Matrix3d H_phi = H_phi_1 + H_phi_2;
  if (use_hybrid_inekf) {
    model.H.block<3, 3>(0, StateIdx::kAtt) = -H_phi - Skew(v_ned);
  } else {
    model.H.block<3, 3>(0, StateIdx::kAtt) = H_phi;
  }

  // H_bg = C_b^n * Skew(l_gnss) * diag(1-sg)
  model.H.block<3, 3>(0, StateIdx::kBg) =
      C_bn * Skew(state.gnss_lever_arm) * sf_g.asDiagonal();

  // H_sg = C_b^n * Skew(l_gnss) * diag(ω_ib^b-bg)
  model.H.block<3, 3>(0, StateIdx::kSg) =
      C_bn * Skew(state.gnss_lever_arm) * omega_ib_unbiased.asDiagonal();

  // H_gnss_lever = C_b^n * Skew(ω_nb^b)
  model.H.block<3, 3>(0, StateIdx::kGnssLever) = C_bn * Skew(omega_nb_b);

  if (use_true_iekf) {
    TransformAdditiveCoreJacobianToTrueInEkf(model.H, C_bn);
  }

  model.R = sigma_gnss_vel.cwiseProduct(sigma_gnss_vel).asDiagonal();

  return model;
}

}  // namespace MeasModels
