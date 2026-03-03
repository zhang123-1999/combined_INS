#include "core/eskf.h"

#include <iostream>

using namespace std;
using namespace Eigen;

namespace {
// 地球自转角速度（ECEF）
constexpr double kOmegaEarth = 7.292115e-5;
const Vector3d kOmegaIeE(0.0, 0.0, kOmegaEarth);
constexpr double kMinDt = 1e-9;
// WGS84 椭球参数
constexpr double kA = 6378137.0;
constexpr double kE2 = 6.69437999014e-3;

// 计算子午圈半径 R_M 和卯酉圈半径 R_N
std::pair<double, double> ComputeRN(double lat) {
  double sin_lat = sin(lat);
  double sin2_lat = sin_lat * sin_lat;
  double R_M = kA * (1.0 - kE2) / pow(1.0 - kE2 * sin2_lat, 1.5);
  double R_N = kA / sqrt(1.0 - kE2 * sin2_lat);
  return {R_M, R_N};
}

// 计算 NED 系下的地球自转角速度 omega_ie^n
Vector3d OmegaIeNedLocal(double lat) {
  double cos_lat = cos(lat);
  double sin_lat = sin(lat);
  return Vector3d(kOmegaEarth * cos_lat, 0.0, -kOmegaEarth * sin_lat);
}

// 计算 NED 系下的导航系旋转角速度 omega_en^n
Vector3d OmegaEnNedLocal(const Vector3d &v_ned, double lat, double h, double R_M, double R_N) {
  double v_N = v_ned.x();
  double v_E = v_ned.y();

  double omega_en_N = v_E / (R_N + h);
  double omega_en_E = -v_N / (R_M + h);
  double omega_en_D = -v_E * tan(lat) / (R_N + h);

  return Vector3d(omega_en_N, omega_en_E, omega_en_D);
}

// 计算局部重力（NED系下，指向下为正）
double LocalGravityLocal(double lat, double h) {
  double sin_lat = sin(lat);
  double sin2_lat = sin_lat * sin_lat;
  double sin4_lat = sin2_lat * sin2_lat;
  double gamma_a = 9.7803267715;
  double g0 = gamma_a * (1.0 + 0.0052790414 * sin2_lat + 0.0000232718 * sin4_lat);
  return g0 - (3.0877e-6 - 4.3e-9 * sin2_lat) * h + 0.72e-12 * h * h;
}
}  // namespace

/**
 * 基于相邻两帧 IMU 增量的惯导传播。
 * 输出传播后的名义状态、姿态矩阵与中间量（比力、角速度）。
 */
PropagationResult InsMech::Propagate(const State &state, const ImuData &imu_prev,
                                     const ImuData &imu_curr) {
  PropagationResult out;
  out.state = state;
  out.Cbn = QuatToRot(state.q);
  out.f_b.setZero();
  out.omega_b.setZero();

  const double dt = imu_curr.dt;
  if (dt <= kMinDt) {
    // 极小 dt 直接返回原状态
    return out;
  }

  // 零偏 + 比例因子 + 地球自转补偿后的角增量/速度增量
  Vector3d omega_ie_b = out.Cbn.transpose() * kOmegaIeE;
  // 1) 去零偏
  Vector3d dtheta_bc_prev = imu_prev.dtheta - state.bg * imu_prev.dt;
  Vector3d dtheta_bc_curr = imu_curr.dtheta - state.bg * dt;
  Vector3d dvel_bc_prev = imu_prev.dvel - state.ba * imu_prev.dt;
  Vector3d dvel_bc_curr = imu_curr.dvel - state.ba * dt;
  // 2) 补偿比例因子: (I - diag(s)) * bias_corrected = (1-s) .* bc
  Vector3d sf_g = Vector3d::Ones() - state.sg;  // 陀螺比例因子修正系数
  Vector3d sf_a = Vector3d::Ones() - state.sa;  // 加速度计比例因子修正系数
  Vector3d dtheta_prev = sf_g.cwiseProduct(dtheta_bc_prev) - omega_ie_b * imu_prev.dt;
  Vector3d dtheta_curr = sf_g.cwiseProduct(dtheta_bc_curr) - omega_ie_b * dt;
  Vector3d dvel_prev = sf_a.cwiseProduct(dvel_bc_prev);
  Vector3d dvel_curr = sf_a.cwiseProduct(dvel_bc_curr);

  // 双子样圆锥/划桨补偿，提升高频旋转/运动精度
  Vector3d coning = dtheta_curr;
  Vector3d sculling = dvel_curr + 0.5 * dtheta_curr.cross(dvel_curr);
  if (imu_prev.dt > kMinDt) {
    coning += dtheta_prev.cross(dtheta_curr) / 12.0;
    sculling += (dtheta_prev.cross(dvel_curr) + dvel_prev.cross(dtheta_curr)) / 12.0;
  }

  // 姿态用中值四元数更新，速度使用中值姿态旋转的比力
  Vector4d dq_mid = QuatFromSmallAngle(coning * 0.5);
  Vector4d dq = QuatFromSmallAngle(coning);
  Vector4d q_mid = NormalizeQuat(QuatMultiply(state.q, dq_mid));
  Matrix3d R_mid = QuatToRot(q_mid);

  // 位置、速度、姿态递推（ECEF 导航系）
  State next = state;
  next.q = NormalizeQuat(QuatMultiply(state.q, dq));
  Matrix3d Cbn_next = QuatToRot(next.q);
  Vector3d dv_nav = R_mid * sculling;
  Vector3d gravity_e = GravityEcef(state.p);
  Vector3d coriolis = -2.0 * kOmegaIeE.cross(state.v);
  Vector3d v_next = state.v + dv_nav + (gravity_e + coriolis) * dt;
  next.v = v_next;
  next.p = state.p + 0.5 * (state.v + v_next) * dt;

  out.state = next;
  out.Cbn = Cbn_next;
  out.f_b = sculling / dt;
  out.omega_b = coning / dt;  // 近似为 ω_nb^b
  return out;
}

/**
 * 构建离散化过程模型 Phi 与 Qd。
 * 使用NED系下的完整误差状态微分方程（参考kf-gins-docs.tex）。
 *
 * 状态顺序: [δr^n, δv^n, φ, ba, bg, sg, sa, δk_odo, α, δℓ_odo, δℓ_gnss]
 *
 * 误差微分方程（NED系）：
 *   δṙ^n = -(ω_en^n ×) δr^n + δv^n - (v^n ×) δθ
 *   δv̇^n = C_b^n δf^b + (f^n ×) φ - (2ω_ie^n + ω_en^n) × δv^n + δg^n
 *   φ̇ = -(ω_in^n ×) φ - C_b^n δω_ib^b + δω_in^n
 *
 * @param C_bn 姿态矩阵 C_b^n (body -> NED)
 * @param f_b 机体系比力
 * @param omega_ib_b 机体系角速度
 * @param v_ned NED速度
 * @param lat 纬度（弧度）
 * @param h 高度（米）
 * @param dt 时间步长
 * @param np 噪声参数
 * @param Phi 输出状态转移矩阵
 * @param Qd 输出离散过程噪声
 */
void InsMech::BuildProcessModel(const Matrix3d &C_bn,
                                const Vector3d &f_b,
                                const Vector3d &omega_ib_b,
                                const Vector3d &v_ned,
                                double lat, double h, double dt,
                                const NoiseParams &np,
                                Matrix<double, kStateDim, kStateDim> &Phi,
                                Matrix<double, kStateDim, kStateDim> &Qd,
                                const FejManager *fej,
                                const Vector3d &f_b_fej) {
  // 检查输入参数有效性
  if (std::abs(lat) > 1.57079632679 + 0.1) {
    std::cerr << "[BuildProcessModel] WARNING: lat out of range: " << lat << "\n";
  }
  if (std::abs(h) > 1e7) {
    std::cerr << "[BuildProcessModel] WARNING: h out of range: " << h << "\n";
  }
  if (v_ned.norm() > 1e5) {
    std::cerr << "[BuildProcessModel] WARNING: v_ned too large: " << v_ned.transpose() << "\n";
  }

  Phi.setIdentity();

  // 计算地理参数
  auto [R_M, R_N] = ComputeRN(lat);
  Vector3d omega_ie_n = OmegaIeNedLocal(lat);
  Vector3d omega_en_n = OmegaEnNedLocal(v_ned, lat, h, R_M, R_N);
  Vector3d omega_in_n = omega_ie_n + omega_en_n;

  // 导航系下的比力
  Vector3d f_n = C_bn * f_b;

  // 连续时间误差模型 F（NED系）
  Matrix<double, kStateDim, kStateDim> F = Matrix<double, kStateDim, kStateDim>::Zero();

  // === 位置误差 δṙ^n ===
  // F_rr = -(ω_en^n ×)
  F.block<3, 3>(0, 0) = -Skew(omega_en_n);
  // F_rv = I
  F.block<3, 3>(0, 3) = Matrix3d::Identity();
  // F_rφ = 0（按 kf-gins-docs 状态转移矩阵）

  // === 速度误差 δv̇^n ===
  // F_vr: 地球曲率和重力梯度项
  // δg^n ≈ [0, 0, 2g/(R_M+R_N) * δr_D]^T
  double g = LocalGravityLocal(lat, h);
  Matrix3d F_vr = Matrix3d::Zero();
  F_vr(2, 2) = 2.0 * g / (sqrt(R_M * R_N) + h);  // 垂向重力梯度
  // ω_ie^n 对位置的依赖
  Vector3d F_vr_col0;
  F_vr_col0 << 2.0 * (-omega_ie_n.z()) / (R_M + h),
              0.0,
              2.0 * (-omega_ie_n.x()) / (R_M + h);
  F_vr.col(0) = F_vr_col0;
  F.block<3, 3>(3, 0) = F_vr;

  // F_vv = -(2ω_ie^n + ω_en^n) ×
  F.block<3, 3>(3, 3) = -Skew(2.0 * omega_ie_n + omega_en_n);

  // F_vφ = (f^n ×)
  F.block<3, 3>(3, 6) = Skew(f_n);

  // F_vba = -C_b^n（标准ESKF约定）
  F.block<3, 3>(3, 9) = -C_bn;

  // F_vsa = -C_b^n * diag(f^b)（标准ESKF约定）
  Matrix3d diag_fb = f_b.asDiagonal();
  F.block<3, 3>(3, 18) = -C_bn * diag_fb;

  // === 姿态误差 φ̇ ===
  // F_φr = ∂ω_in^n/∂r
  Matrix3d F_phir = Matrix3d::Zero();
  // ω_ie^n 对纬度的依赖
  F_phir(0, 0) = -omega_ie_n.z() / (R_M + h);
  F_phir(2, 0) = -omega_ie_n.x() / (R_M + h);
  // ω_en^n 对位置的依赖
  F_phir(0, 2) = v_ned.y() / ((R_N + h) * (R_N + h));
  F_phir(1, 2) = -v_ned.x() / ((R_M + h) * (R_M + h));
  F_phir(2, 0) += -v_ned.y() / ((R_M + h) * (R_N + h) * cos(lat) * cos(lat));
  F_phir(2, 2) = -v_ned.y() * tan(lat) / ((R_N + h) * (R_N + h));
  F.block<3, 3>(6, 0) = F_phir;

  // F_φv = ∂ω_in^n/∂v
  Matrix3d F_phiv = Matrix3d::Zero();
  F_phiv(0, 1) = 1.0 / (R_N + h);
  F_phiv(1, 0) = -1.0 / (R_M + h);
  F_phiv(2, 1) = -tan(lat) / (R_N + h);
  F.block<3, 3>(6, 3) = F_phiv;

  // F_φφ = -(ω_in^n ×)
  F.block<3, 3>(6, 6) = -Skew(omega_in_n);

  // F_φbg = -C_b^n（标准ESKF约定）
  F.block<3, 3>(6, 12) = -C_bn;

  // F_φsg = -C_b^n * diag(ω_ib^b)（标准ESKF约定）
  Matrix3d diag_wib = omega_ib_b.asDiagonal();
  F.block<3, 3>(6, 15) = -C_bn * diag_wib;

  // === IMU误差（马尔可夫模型）===
  double T = np.markov_corr_time;
  if (T > 0.0) {
    double invT = 1.0 / T;
    Matrix3d neg_invT_I = -invT * Matrix3d::Identity();
    F.block<3, 3>(9,  9)  = neg_invT_I;   // ba
    F.block<3, 3>(12, 12) = neg_invT_I;   // bg
    F.block<3, 3>(15, 15) = neg_invT_I;   // sg
    F.block<3, 3>(18, 18) = neg_invT_I;   // sa
  }
  // odo_scale, mounting, lever_arm, gnss_lever_arm 保持随机常数

  // FEJ：仅替换弱可观敏感块（其余块保持标准 ESKF）
  if (fej != nullptr && fej->enabled && fej->initialized) {
    Vector3d f_n_fej = fej->C_bn_fej * f_b_fej;
    F.block<3, 3>(3, 6) = Skew(f_n_fej);       // F_vphi
    F.block<3, 3>(6, 12) = -fej->C_bn_fej;     // F_phibg
  }

  // 检查F矩阵是否有异常大的值
  double max_F = F.cwiseAbs().maxCoeff();
  if (max_F > 1e6) {
    std::cerr << "[BuildProcessModel] WARNING: max |F| = " << max_F << " at lat=" << lat << " h=" << h << "\n";
    // 找出最大值的位置
    int max_row, max_col;
    F.cwiseAbs().maxCoeff(&max_row, &max_col);
    std::cerr << "[BuildProcessModel] Max F at (" << max_row << "," << max_col << "): " << F(max_row, max_col) << "\n";
    // 打印相关值
    std::cerr << "[BuildProcessModel] tan(lat)=" << tan(lat) << " cos(lat)=" << cos(lat) << " v_ned=" << v_ned.transpose() << "\n";
  }

  Phi += F * dt;

  // 噪声驱动矩阵 G（28 维噪声：acc(3), gyro(3), ba(3), bg(3), sg(3), sa(3), odo_scale(1), mounting_roll(1), mounting_pitch(1), mounting_yaw(1), lever(3), gnss_lever(3) = 28）
  constexpr int kNoiseDim = 28;
  Matrix<double, kStateDim, kNoiseDim> G = Matrix<double, kStateDim, kNoiseDim>::Zero();
  G.block<3, 3>(3, 0) = -C_bn;                    // acc noise → velocity
  // 姿态误差 φ 在 NED 系，陀螺白噪声在 body 系，需经 C_b^n 映射。
  G.block<3, 3>(6, 3) = -C_bn;                   // gyro noise → attitude
  G.block<3, 3>(9, 6) = Matrix3d::Identity();    // ba process noise
  G.block<3, 3>(12, 9) = Matrix3d::Identity();   // bg process noise
  G.block<3, 3>(15, 12) = Matrix3d::Identity();  // sg process noise
  G.block<3, 3>(18, 15) = Matrix3d::Identity();  // sa process noise
  G(21, 18) = 1.0;   // odo_scale
  G(22, 19) = 1.0;   // mounting_roll
  G(23, 20) = 1.0;   // mounting_pitch
  G(24, 21) = 1.0;   // mounting_yaw
  G.block<3, 3>(25, 22) = Matrix3d::Identity();  // lever_arm (state indices 25-27, noise indices 22-24)
  G.block<3, 3>(28, 25) = Matrix3d::Identity();  // gnss_lever_arm (state indices 28-30, noise indices 25-27)

  // 连续时间噪声协方差 Qc
  // 马尔可夫模型: sigma 为不稳定性（稳态标准差），驱动噪声 σ_w = σ_ss * √(2/T)
  // 随机游走: sigma 直接作为驱动噪声密度
  double ba_w, bg_w, sg_w, sa_w;
  if (T > 0.0) {
    double scale = sqrt(2.0 / T);
    ba_w = np.sigma_ba * scale;
    bg_w = np.sigma_bg * scale;
    sg_w = np.sigma_sg * scale;
    sa_w = np.sigma_sa * scale;
  } else {
    ba_w = np.sigma_ba;
    bg_w = np.sigma_bg;
    sg_w = np.sigma_sg;
    sa_w = np.sigma_sa;
  }

  Matrix<double, kNoiseDim, kNoiseDim> Qc = Matrix<double, kNoiseDim, kNoiseDim>::Zero();
  double sigma_mounting_roll =
      (np.sigma_mounting_roll >= 0.0) ? np.sigma_mounting_roll : np.sigma_mounting;
  double sigma_mounting_pitch =
      (np.sigma_mounting_pitch >= 0.0) ? np.sigma_mounting_pitch : np.sigma_mounting;
  double sigma_mounting_yaw =
      (np.sigma_mounting_yaw >= 0.0) ? np.sigma_mounting_yaw : np.sigma_mounting;
  double sa2 = np.sigma_acc * np.sigma_acc;     // acc white noise (不受马尔可夫影响)
  double sg2 = np.sigma_gyro * np.sigma_gyro;   // gyro white noise
  Qc.diagonal() << sa2, sa2, sa2,                // acc white noise (0-2)
      sg2, sg2, sg2,                              // gyro white noise (3-5)
      ba_w*ba_w, ba_w*ba_w, ba_w*ba_w,           // ba driving noise (6-8)
      bg_w*bg_w, bg_w*bg_w, bg_w*bg_w,           // bg driving noise (9-11)
      sg_w*sg_w, sg_w*sg_w, sg_w*sg_w,           // sg driving noise (12-14)
      sa_w*sa_w, sa_w*sa_w, sa_w*sa_w,           // sa driving noise (15-17)
      np.sigma_odo_scale * np.sigma_odo_scale,   // odo_scale (18)
      sigma_mounting_roll * sigma_mounting_roll,       // mounting_roll (19)
      sigma_mounting_pitch * sigma_mounting_pitch,     // mounting_pitch (20)
      sigma_mounting_yaw * sigma_mounting_yaw,         // mounting_yaw (21)
      np.sigma_lever_arm * np.sigma_lever_arm,   // lever_arm x (22)
      np.sigma_lever_arm * np.sigma_lever_arm,   // lever_arm y (23)
      np.sigma_lever_arm * np.sigma_lever_arm,   // lever_arm z (24)
      np.sigma_gnss_lever_arm * np.sigma_gnss_lever_arm,   // gnss_lever_arm x (25)
      np.sigma_gnss_lever_arm * np.sigma_gnss_lever_arm,   // gnss_lever_arm y (26)
      np.sigma_gnss_lever_arm * np.sigma_gnss_lever_arm;   // gnss_lever_arm z (27)

  // 梯形近似离散噪声（与文档式(569)一致的简化实现）：
  // Qd ≈ 0.5 * [Phi * (GQcG^T) * Phi^T + (GQcG^T)] * dt
  Matrix<double, kStateDim, kStateDim> Qc_cont = G * Qc * G.transpose();
  Qd = 0.5 * (Phi * Qc_cont * Phi.transpose() + Qc_cont) * dt;
  Qd = 0.5 * (Qd + Qd.transpose());
}
