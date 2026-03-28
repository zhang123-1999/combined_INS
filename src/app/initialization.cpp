#include "app/fusion.h"

#include <algorithm>
#include <iostream>
#include <iomanip>

#include "utils/math_utils.h"

using namespace std;
using namespace Eigen;

namespace {

Vector4d InterpolateQuaternionNlerp(const Vector4d &q0_in,
                                    const Vector4d &q1_in,
                                    double alpha) {
  Vector4d q0 = NormalizeQuat(q0_in);
  Vector4d q1 = NormalizeQuat(q1_in);
  if (q0.dot(q1) < 0.0) {
    q1 = -q1;
  }
  return NormalizeQuat((1.0 - alpha) * q0 + alpha * q1);
}

}  // namespace

bool InterpolateTruthPva(const TruthData &truth, double t, int &cursor,
                         Vector3d &p_out, Vector3d &v_out, Vector4d &q_out) {
  const int n = static_cast<int>(truth.timestamps.size());
  if (n <= 0 || truth.positions.rows() != n || truth.velocities.rows() != n ||
      truth.quaternions.rows() != n) {
    return false;
  }

  if (n == 1 || t <= truth.timestamps(0)) {
    p_out = truth.positions.row(0).transpose();
    v_out = truth.velocities.row(0).transpose();
    q_out = NormalizeQuat(truth.quaternions.row(0).transpose());
    cursor = 0;
    return true;
  }
  if (t >= truth.timestamps(n - 1)) {
    p_out = truth.positions.row(n - 1).transpose();
    v_out = truth.velocities.row(n - 1).transpose();
    q_out = NormalizeQuat(truth.quaternions.row(n - 1).transpose());
    cursor = std::max(0, n - 2);
    return true;
  }

  cursor = std::clamp(cursor, 0, std::max(0, n - 2));
  while (cursor + 1 < n - 1 && truth.timestamps(cursor + 1) < t) {
    ++cursor;
  }
  while (cursor > 0 && truth.timestamps(cursor) > t) {
    --cursor;
  }

  const int i0 = cursor;
  const int i1 = std::min(i0 + 1, n - 1);
  const double t0 = truth.timestamps(i0);
  const double t1 = truth.timestamps(i1);
  const double alpha =
      (t1 > t0 + 1.0e-12) ? std::clamp((t - t0) / (t1 - t0), 0.0, 1.0) : 0.0;

  p_out = (1.0 - alpha) * truth.positions.row(i0).transpose() +
          alpha * truth.positions.row(i1).transpose();
  v_out = (1.0 - alpha) * truth.velocities.row(i0).transpose() +
          alpha * truth.velocities.row(i1).transpose();
  q_out = InterpolateQuaternionNlerp(truth.quaternions.row(i0).transpose(),
                                     truth.quaternions.row(i1).transpose(),
                                     alpha);
  return true;
}

/**
 * 初始化名义状态与协方差矩阵。
 *
 * 名义状态（ECEF系）：
 *   - p: ECEF位置
 *   - v: ECEF速度
 *   - q: ECEF姿态四元数
 *
 * 误差状态协方差 P0（NED系）：
 *   - δr^n: NED位置误差（北东地，单位米）
 *   - δv^n: NED速度误差（m/s）
 *   - φ: 姿态误差（NED系，弧度）
 *   - ...
 *
 * 注意：P0对角线上的姿态标准差单位是度，需要转换为弧度。
 */
bool InitializeState(const FusionOptions &options, const vector<ImuData> &imu,
                     const TruthData &truth, State &x0,
                     Matrix<double, kStateDim, kStateDim> &P0) {
  if (imu.empty()) {
    cout << "error: 初始化失败，IMU 数据为空\n";
    return false;
  }
  if (options.init.use_truth_pva && truth.positions.rows() == 0) {
    cout << "error: 初始化失败，真值数据为空\n";
    return false;
  }

  if (options.init.use_truth_pva) {
    if (truth.positions.rows() == 0) {
      cout << "error: 初始化失败，真值数据为空\n";
      return false;
    }
    int truth_cursor = 0;
    if (!InterpolateTruthPva(truth, imu.front().t, truth_cursor, x0.p, x0.v,
                             x0.q)) {
      cout << "error: 初始化失败，真值插值失败\n";
      return false;
    }

    cout << "[Init] Mode: Truth\n";
    cout << "[Init] Truth aligned to imu.front().t=" << fixed
         << setprecision(6) << imu.front().t << "\n";
    cout << "[Init] Truth P0: " << x0.p.transpose() << "\n";
  } else {
    // 手动初始化 (LLA -> ECEF, NED -> ECEF, RPY -> Quaternion)
    Llh llh;
    llh.lat = options.init.init_pos_lla.x() * EIGEN_PI / 180.0;
    llh.lon = options.init.init_pos_lla.y() * EIGEN_PI / 180.0;
    llh.h = options.init.init_pos_lla.z();

    x0.p = LlhToEcef(llh);

    // 速度: NED -> ECEF
    Matrix3d R_n_e = RotNedToEcef(llh);
    x0.v = R_n_e * options.init.init_vel_ned;

    // 姿态: RPY (deg) -> Body-End Quaternion
    // q_b_e = q_n_e * q_b_n
    Vector3d rpy_rad = options.init.init_att_rpy * EIGEN_PI / 180.0;
    Vector4d q_b_n = RpyToQuat(rpy_rad);

    Quaterniond Q_ne(R_n_e); // Eigen 构造函数接受旋转矩阵
    Q_ne.normalize();
    Vector4d q_n_e(Q_ne.w(), Q_ne.x(), Q_ne.y(), Q_ne.z());

    x0.q = QuatMultiply(q_n_e, q_b_n);

    cout << "[Init] Mode: Manual\n";
    cout << "[Init] Manual P0: " << x0.p.transpose() << "\n";
  }

  x0.ba = options.init.ba0;
  x0.bg = options.init.bg0;
  x0.sg = options.init.sg0;  // 陀螺比例因子初值
  x0.sa = options.init.sa0;  // 加速度计比例因子初值
  x0.odo_scale = options.init.odo_scale;
  if (options.constraints.enable_odo && x0.odo_scale <= 0.0) {
    // ODO比例因子应为正数；若配置为0表示“未知”，回退到1.0便于在线估计收敛。
    cout << "[Init] WARNING: init.odo_scale <= 0，已回退为 1.0\n";
    x0.odo_scale = 1.0;
  }
  // 安装角初始值（deg → rad）
  x0.mounting_roll = options.init.mounting_roll0 * EIGEN_PI / 180.0;  // 横滚安装角
  x0.mounting_pitch = options.init.mounting_pitch0 * EIGEN_PI / 180.0;
  x0.mounting_yaw = options.init.mounting_yaw0 * EIGEN_PI / 180.0;

  bool lever_conflict =
      (options.init.lever_arm0 - options.constraints.odo_lever_arm).norm() > 1e-9;
  if (lever_conflict && options.init.strict_extrinsic_conflict) {
    cout << "error: init.lever_arm0 与 constraints.odo_lever_arm 冲突，"
         << "且 strict_extrinsic_conflict=true\n";
    return false;
  }
  if (options.init.lever_arm_source == "init") {
    x0.lever_arm = options.init.lever_arm0;
  } else {
    x0.lever_arm = options.constraints.odo_lever_arm;
  }
  if (lever_conflict) {
    cout << "[Init] WARNING: init.lever_arm0 与 constraints.odo_lever_arm 不一致，"
         << "当前按 lever_arm_source=" << options.init.lever_arm_source
         << " 取值\n";
  }
  x0.gnss_lever_arm = options.init.gnss_lever_arm0;

  P0 = Matrix<double, kStateDim, kStateDim>::Zero();

  // 若显式提供 P0_diag，则其优先于 std_* 字段，避免“配置已写但未生效”。
  if (options.init.has_custom_P0_diag) {
      for (int i = 0; i < kStateDim; ++i) {
          P0(i, i) = options.init.P0_diag(i);
      }
      cout << "[Init] P0 source: custom P0_diag\n";
  } else if (options.init.std_pos.norm() > 1e-9) {
      Vector3d var_pos = options.init.std_pos.array().square();
      Vector3d var_vel = options.init.std_vel.array().square();
      // 姿态 STD 单位是 degree，需转为 radian
      Vector3d var_att = (options.init.std_att * EIGEN_PI / 180.0).array().square();
      Vector3d var_ba = options.init.std_ba.array().square();
      Vector3d var_bg = options.init.std_bg.array().square();
      Vector3d var_sg = options.init.std_sg.array().square();  // 陀螺比例因子
      Vector3d var_sa = options.init.std_sa.array().square();  // 加速度计比例因子
      double var_odo_scale = options.init.std_odo_scale * options.init.std_odo_scale;
      double var_mounting_roll = (options.init.std_mounting_roll * EIGEN_PI / 180.0) *
                                 (options.init.std_mounting_roll * EIGEN_PI / 180.0);  // 横滚安装角
      double var_mounting_pitch = (options.init.std_mounting_pitch * EIGEN_PI / 180.0) *
                                  (options.init.std_mounting_pitch * EIGEN_PI / 180.0);
      double var_mounting_yaw = (options.init.std_mounting_yaw * EIGEN_PI / 180.0) *
                                (options.init.std_mounting_yaw * EIGEN_PI / 180.0);

      P0.diagonal().segment<3>(StateIdx::kPos) = var_pos;
      P0.diagonal().segment<3>(StateIdx::kVel) = var_vel;
      P0.diagonal().segment<3>(StateIdx::kAtt) = var_att;
      P0.diagonal().segment<3>(StateIdx::kBa) = var_ba;       // [修复] 之前遗漏了 ba 的 P0 赋值
      P0.diagonal().segment<3>(StateIdx::kBg) = var_bg;
      P0.diagonal().segment<3>(StateIdx::kSg) = var_sg;      // [新增] 陀螺比例因子
      P0.diagonal().segment<3>(StateIdx::kSa) = var_sa;      // [新增] 加速度计比例因子
      P0(StateIdx::kOdoScale, StateIdx::kOdoScale) = var_odo_scale;
      P0(StateIdx::kMountRoll, StateIdx::kMountRoll) = var_mounting_roll;   // [修复] 添加缺失的 mounting_roll
      P0(StateIdx::kMountPitch, StateIdx::kMountPitch) = var_mounting_pitch;
      P0(StateIdx::kMountYaw, StateIdx::kMountYaw) = var_mounting_yaw;
      Vector3d var_lever = options.init.std_lever_arm.array().square();
      P0.diagonal().segment<3>(StateIdx::kLever) = var_lever;  // [修复] lever_arm 从 25 开始
      // GNSS天线杆臂 (28-30)：初值未知，需要足够大的初始方差
      Vector3d var_gnss_lever = options.init.std_gnss_lever_arm.array().square();
      P0.diagonal().segment<3>(StateIdx::kGnssLever) = var_gnss_lever;  // [修复] gnss_lever_arm 从 28 开始
      cout << "[Init] P0 source: std_* fields\n";
  } else {
      // 兼容旧逻辑：若 std_* 被显式清零，则回退到默认 P0_diag。
      for(int i = 0; i < kStateDim; ++i) {
          P0(i, i) = options.init.P0_diag(i);
      }
      cout << "[Init] P0 source: fallback default P0_diag\n";
  }


  return true;
}
