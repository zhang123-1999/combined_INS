#include "utils/math_utils.h"

#include <algorithm>
#include <cmath>

using namespace std;
using namespace Eigen;

namespace {
// WGS84 椭球参数
constexpr double kA = 6378137.0;
constexpr double kE2 = 6.69437999014e-3;
}  // namespace

/**
 * 生成向量的反对称矩阵（叉乘矩阵）。
 * @param v 输入向量
 * @return 叉乘等价矩阵
 */
Matrix3d Skew(const Vector3d &v) {
  Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

/**
 * 四元数归一化，防止模长漂移。
 * @param q 输入四元数 (w,x,y,z)
 * @return 归一化后的四元数
 */
// 防止长时间积分导致四元数模长漂移
Vector4d NormalizeQuat(const Vector4d &q) { return q / q.norm(); }

/**
 * 四元数哈密顿乘法（w,x,y,z）。
 * @param q1 左四元数
 * @param q2 右四元数
 * @return 乘积四元数
 */
// 采用 (w,x,y,z) 顺序的哈密顿乘法
Vector4d QuatMultiply(const Vector4d &q1, const Vector4d &q2) {
  Vector4d r;
  double w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
  double w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
  r << w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
      w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
      w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
  return r;
}

/**
 * 小角度扰动向量转增量四元数。
 * @param dtheta 小姿态扰动向量
 * @return 对应的增量四元数
 */
// 将小角度扰动向量映射为增量四元数，用于误差注入
Vector4d QuatFromSmallAngle(const Vector3d &dtheta) {
  double angle = dtheta.norm();
  if (angle < 1e-12) {
    return Vector4d(1.0, 0.0, 0.0, 0.0);
  }
  Vector3d axis = dtheta / angle;
  double half = 0.5 * angle;
  double s = sin(half);
  return Vector4d(cos(half), axis.x() * s, axis.y() * s, axis.z() * s);
}

/**
 * 四元数转旋转矩阵（导航系 <- 机体系）。
 * @param q_in 输入四元数 (w,x,y,z)
 * @return 旋转矩阵
 */
Matrix3d QuatToRot(const Vector4d &q_in) {
  Vector4d q = NormalizeQuat(q_in);
  double w = q[0], x = q[1], y = q[2], z = q[3];
  Matrix3d R;
  R << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),      //
      2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),        //
      2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y);
  return R;
}

/**
 * 用平均比力对准重力方向，估计初始姿态（无偏航）。
 * @param avg_specific_force 平均比力（机体系）
 * @param gravity_nav 重力方向（导航系）
 * @return 估计的四元数 (w,x,y,z)
 */
Vector4d AlignWithGravity(const Vector3d &avg_specific_force,
                          const Vector3d &gravity_nav) {
  Vector3d body_z = avg_specific_force.normalized();
  Vector3d target_z = (-gravity_nav).normalized();
  if (body_z.norm() < 1e-12 || target_z.norm() < 1e-12) {
    return Vector4d(1.0, 0.0, 0.0, 0.0);
  }

  // 利用叉积求解旋转轴，点积求解旋转角
  Vector3d v = body_z.cross(target_z);
  double s = v.norm();
  double c = body_z.dot(target_z);
  if (s < 1e-8) {
    return Vector4d(1.0, 0.0, 0.0, 0.0);
  }
  Vector3d axis = v / s;
  double angle = atan2(s, c);
  return NormalizeQuat(QuatFromSmallAngle(axis * angle));
}

/**
 * ECEF 坐标转大地坐标（rad, rad, m）。
 * @param ecef 输入 ECEF 坐标
 * @return 纬度/经度/高程
 */
Llh EcefToLlh(const Vector3d &ecef) {
  double x = ecef.x();
  double y = ecef.y();
  double z = ecef.z();

  double lon = atan2(y, x);
  double p = sqrt(x * x + y * y);
  double lat = atan2(z, p * (1.0 - kE2));
  double h = 0.0;

  // 简短迭代改进 lat/h
  for (int i = 0; i < 5; ++i) {
    double sin_lat = sin(lat);
    double N = kA / sqrt(1.0 - kE2 * sin_lat * sin_lat);
    h = p / cos(lat) - N;
    double ratio = 1.0 - kE2 * N / (N + h);
    lat = atan2(z, p * ratio);
  }

  double sin_lat = sin(lat);
  double N = kA / sqrt(1.0 - kE2 * sin_lat * sin_lat);
  h = p / cos(lat) - N;
  return {lat, lon, h};
}

Vector3d LlhToEcef(const Llh &llh) {
  double sin_lat = sin(llh.lat);
  double cos_lat = cos(llh.lat);
  double sin_lon = sin(llh.lon);
  double cos_lon = cos(llh.lon);

  double N = kA / sqrt(1.0 - kE2 * sin_lat * sin_lat);

  double x = (N + llh.h) * cos_lat * cos_lon;
  double y = (N + llh.h) * cos_lat * sin_lon;
  double z = (N * (1.0 - kE2) + llh.h) * sin_lat;

  return Vector3d(x, y, z);
}

Matrix3d RotNedToEcef(const Llh &llh) {
  double sin_lat = sin(llh.lat);
  double cos_lat = cos(llh.lat);
  double sin_lon = sin(llh.lon);
  double cos_lon = cos(llh.lon);

  Matrix3d R;
  R(0, 0) = -sin_lat * cos_lon; R(0, 1) = -sin_lon; R(0, 2) = -cos_lat * cos_lon;
  R(1, 0) = -sin_lat * sin_lon; R(1, 1) =  cos_lon; R(1, 2) = -cos_lat * sin_lon;
  R(2, 0) =  cos_lat;           R(2, 1) =  0.0;     R(2, 2) = -sin_lat;

  return R;
}

Vector4d RpyToQuat(const Vector3d &rpy) {
  double phi = rpy[0] / 2.0;    // Roll
  double theta = rpy[1] / 2.0;  // Pitch
  double psi = rpy[2] / 2.0;    // Yaw

  double s_phi = sin(phi); double c_phi = cos(phi);
  double s_theta = sin(theta); double c_theta = cos(theta);
  double s_psi = sin(psi); double c_psi = cos(psi);

  double w = c_psi * c_theta * c_phi + s_psi * s_theta * s_phi;
  double x = c_psi * c_theta * s_phi - s_psi * s_theta * c_phi;
  double y = c_psi * s_theta * c_phi + s_psi * c_theta * s_phi;
  double z = s_psi * c_theta * c_phi - c_psi * s_theta * s_phi;

  return Vector4d(w, x, y, z);
}

/**
 * 计算 ECEF 中的重力向量（指向地心）。
 * @param ecef 输入 ECEF 坐标
 * @return 重力向量
 */
Vector3d GravityEcef(const Vector3d &ecef) {
  Llh llh = EcefToLlh(ecef);
  double sin_lat = sin(llh.lat);
  double cos_lat = cos(llh.lat);

  // Somigliana 模型 + 简单高度改正
  double g0 =
      9.7803253359 * (1.0 + 0.00193185265241 * sin_lat * sin_lat) /
      sqrt(1.0 - kE2 * sin_lat * sin_lat);
  double g = g0 - 3.086e-6 * llh.h;

  Vector3d n(cos_lat * cos(llh.lon), cos_lat * sin(llh.lon),
                    sin_lat);
  return -g * n;
}

/**
 * 单维线性插值（超界时返回端点值）。
 * @param t 时间序列
 * @param v 数值序列
 * @param query 查询时间
 * @return 插值结果
 */
// 简单双点线性插值（超界时返回端点值）
double LinearInterp(const VectorXd &t, const VectorXd &v, double query) {
  if (query <= t[0]) return v[0];
  if (query >= t[t.size() - 1]) return v[v.size() - 1];
  auto it = lower_bound(t.data(), t.data() + t.size(), query);
  int idx = static_cast<int>(it - t.data());
  int i0 = idx - 1;
  int i1 = idx;
  double t0 = t[i0], t1 = t[i1];
  double ratio = (query - t0) / (t1 - t0 + 1e-12);
  return v[i0] + ratio * (v[i1] - v[i0]);
}

/**
 * 计算轨迹 RMSE。
 * @param est 估计轨迹矩阵
 * @param truth 真值轨迹矩阵
 * @return 3D RMSE 向量
 */
Vector3d RMSE(const MatrixXd &est, const MatrixXd &truth) {
  MatrixXd diff = est - truth;
  Vector3d mse = diff.array().square().colwise().mean();
  return mse.array().sqrt();
}

double Clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

Vector4d QuatConjugate(const Vector4d &q) {
  return Vector4d(q[0], -q[1], -q[2], -q[3]);
}

double QuatDeltaAngleRad(const Vector4d &q0, const Vector4d &q1) {
  Vector4d dq = NormalizeQuat(QuatMultiply(QuatConjugate(q0), q1));
  double w = Clamp(dq[0], -1.0, 1.0);
  return 2.0 * acos(w);
}

void EcefToLatLon(const Vector3d &ecef, double &lat, double &lon) {
  Llh llh = EcefToLlh(ecef);
  lat = llh.lat;
  lon = llh.lon;
}

Matrix3d RotNedToEcef(double lat, double lon) {
  return RotNedToEcef(Llh{lat, lon, 0.0});
}

Vector3d LlhToEcef(double lat_rad, double lon_rad, double h) {
  return LlhToEcef(Llh{lat_rad, lon_rad, h});
}

Matrix3d EulerToRotation(double roll_rad, double pitch_rad, double yaw_rad) {
  AngleAxisd rollAngle(roll_rad, Vector3d::UnitX());
  AngleAxisd pitchAngle(pitch_rad, Vector3d::UnitY());
  AngleAxisd yawAngle(yaw_rad, Vector3d::UnitZ());
  return (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
}

Vector3d RotToEuler(const Matrix3d &R) {
  double pitch = asin(-Clamp(R(2, 0), -1.0, 1.0));
  double roll = atan2(R(2, 1), R(2, 2));
  double yaw = atan2(R(1, 0), R(0, 0));
  return Vector3d(roll, pitch, yaw);
}

std::pair<double, double> ComputeEarthRadius(double lat) {
  double sin_lat = sin(lat);
  double sin2_lat = sin_lat * sin_lat;
  double R_M = kA * (1.0 - kE2) / pow(1.0 - kE2 * sin2_lat, 1.5);  // 子午圈半径
  double R_N = kA / sqrt(1.0 - kE2 * sin2_lat);                     // 卯酉圈半径
  return {R_M, R_N};
}

Vector3d OmegaIeNed(double lat) {
  double omega_ie = 7.292115e-5;  // 地球自转角速度
  double cos_lat = cos(lat);
  double sin_lat = sin(lat);
  return Vector3d(omega_ie * cos_lat, 0.0, -omega_ie * sin_lat);
}

Vector3d OmegaEnNed(const Vector3d &v_ned, double lat, double h) {
  auto [R_M, R_N] = ComputeEarthRadius(lat);
  double v_N = v_ned.x();
  double v_E = v_ned.y();
  double cos_lat = cos(lat);

  double omega_en_N = v_E / (R_N + h);
  double omega_en_E = -v_N / (R_M + h);
  double omega_en_D = -v_E * tan(lat) / (R_N + h);

  return Vector3d(omega_en_N, omega_en_E, omega_en_D);
}

Vector3d OmegaInNed(const Vector3d &v_ned, double lat, double h) {
  return OmegaIeNed(lat) + OmegaEnNed(v_ned, lat, h);
}

double LocalGravity(double lat, double h) {
  double sin_lat = sin(lat);
  double sin2_lat = sin_lat * sin_lat;
  double sin4_lat = sin2_lat * sin2_lat;

  // 参考椭球面上的正常重力
  double gamma_a = 9.7803267715;
  double g0 = gamma_a * (1.0 + 0.0052790414 * sin2_lat + 0.0000232718 * sin4_lat);

  // 高度改正
  double g = g0 - (3.0877e-6 - 4.3e-9 * sin2_lat) * h + 0.72e-12 * h * h;

  return g;
}

Vector3d EcefVelToNed(const Vector3d &v_ecef, double lat, double lon) {
  Matrix3d R_ne = RotNedToEcef(lat, lon);
  return R_ne.transpose() * v_ecef;
}

Vector3d NedVelToEcef(const Vector3d &v_ned, double lat, double lon) {
  Matrix3d R_ne = RotNedToEcef(lat, lon);
  return R_ne * v_ned;
}

Vector3d NedPosErrorToEcef(const Vector3d &dr_ned, double lat, double lon, double h) {
  // dr_ned = [delta_N, delta_E, delta_D]
  // 需要转换为ECEF坐标修正
  // delta_r^e = R_n^e * D_R * delta_p
  // 其中 D_R = diag(R_M + h, (R_N + h)*cos(lat), -1)
  // 简化为：delta_r^e ≈ R_n^e * dr_ned (小误差近似)

  Matrix3d R_ne = RotNedToEcef(lat, lon);
  return R_ne * dr_ned;
}

Vector3d NedVelErrorToEcef(const Vector3d &dv_ned, double lat, double lon) {
  // dv_ned 是NED系下的速度误差
  // 转换为ECEF系：dv^e = R_n^e * dv^n
  Matrix3d R_ne = RotNedToEcef(lat, lon);
  return R_ne * dv_ned;
}
