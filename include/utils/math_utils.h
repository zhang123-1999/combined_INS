// 数学工具函数：四元数运算、插值与误差评估
#pragma once

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

/**
 * 大地坐标结构体。
 * lat/lon 为弧度制，h 为高程（米）。
 */
struct Llh {
  double lat;
  double lon;
  double h;
};

/**
 * 反对称矩阵（叉乘矩阵）生成。
 * @param v 输入向量
 * @return 叉乘等价矩阵
 */
Matrix3d Skew(const Vector3d &v);

/**
 * 四元数归一化，抑制数值漂移。
 * @param q 输入四元数 (w,x,y,z)
 * @return 归一化后的四元数
 */
Vector4d NormalizeQuat(const Vector4d &q);

/**
 * 四元数哈密顿积。
 * @param q1 左四元数
 * @param q2 右四元数
 * @return 乘积四元数
 */
// 四元数哈密顿积
Vector4d QuatMultiply(const Vector4d &q1, const Vector4d &q2);

/**
 * 小姿态扰动向量转增量四元数。
 * @param dtheta 小角度扰动向量
 * @return 对应的四元数增量
 */
Vector4d QuatFromSmallAngle(const Vector3d &dtheta);

/**
 * 四元数转旋转矩阵（导航系 <- 机体系）。
 * @param q_in 输入四元数 (w,x,y,z)
 * @return 旋转矩阵
 */
Matrix3d QuatToRot(const Vector4d &q_in);

/**
 * 用平均比力对准重力方向，估计初始姿态（无偏航）。
 * @param avg_specific_force 平均比力（机体系）
 * @param gravity_nav 重力方向（导航系）
 * @return 估计的四元数 (w,x,y,z)
 */
Vector4d AlignWithGravity(const Vector3d &avg_specific_force,
                          const Vector3d &gravity_nav);

/**
 * ECEF -> 大地坐标 (rad, rad, m) 转换。
 * @param ecef 输入 ECEF 坐标
 * @return 纬度/经度/高程
 */
Llh EcefToLlh(const Vector3d &ecef);

/**
 * 大地坐标 (rad, rad, m) -> ECEF 转换。
 * @param llh 输入 LLH 坐标
 * @return ECEF 坐标
 */
Vector3d LlhToEcef(const Llh &llh);

/**
 * 计算 NED 到 ECEF 的旋转矩阵。
 * @param llh 参考点大地坐标
 * @return R_n^e
 */
Matrix3d RotNedToEcef(const Llh &llh);

/**
 * 欧拉角 (Roll, Pitch, Yaw) 转四元数 (Body -> Nav)。
 * @param rpy 欧拉角 (rad)
 * @return 四元数 (w,x,y,z)
 */
Vector4d RpyToQuat(const Vector3d &rpy);

/**
 * 计算 ECEF 中的重力向量（指向地心，单位 m/s^2）。
 * @param ecef 输入 ECEF 坐标
 * @return 重力向量
 */
Vector3d GravityEcef(const Vector3d &ecef);

/**
 * 单维线性插值（超界时返回端点值）。
 * @param t 时间序列
 * @param v 数值序列
 * @param query 查询时间
 * @return 插值结果
 */
double LinearInterp(const VectorXd &t, const VectorXd &v, double query);

/**
 * 轨迹 RMSE 计算。
 * @param est 估计轨迹矩阵
 * @param truth 真值轨迹矩阵
 * @return 3D RMSE 向量
 */
Vector3d RMSE(const MatrixXd &est, const MatrixXd &truth);

/**
 * 数值截断到 [lo, hi] 范围。
 */
double Clamp(double v, double lo, double hi);

/**
 * 四元数共轭（取逆）。
 */
Vector4d QuatConjugate(const Vector4d &q);

/**
 * 计算两个四元数之间的旋转角（弧度）。
 */
double QuatDeltaAngleRad(const Vector4d &q0, const Vector4d &q1);

/**
 * ECEF → 大地坐标便利接口：仅输出 lat/lon（弧度），无高程。
 * @param ecef 输入 ECEF 坐标
 * @param lat 输出纬度（弧度）
 * @param lon 输出经度（弧度）
 */
void EcefToLatLon(const Vector3d &ecef, double &lat, double &lon);

/**
 * NED→ECEF 旋转矩阵便利接口：直接传入 lat/lon（弧度）。
 * @param lat 参考点纬度（弧度）
 * @param lon 参考点经度（弧度）
 * @return R_n^e
 */
Matrix3d RotNedToEcef(double lat, double lon);

/**
 * LLH→ECEF 便利接口：直接传入 lat/lon/h。
 * @param lat_rad 纬度（弧度）
 * @param lon_rad 经度（弧度）
 * @param h 高程（米）
 * @return ECEF 坐标
 */
Vector3d LlhToEcef(double lat_rad, double lon_rad, double h);

/**
 * 欧拉角 (Roll, Pitch, Yaw) 转旋转矩阵 (Body → Nav, ZYX 顺序)。
 * @param roll_rad 滚转角（弧度）
 * @param pitch_rad 俯仰角（弧度）
 * @param yaw_rad 航向角（弧度）
 */
Matrix3d EulerToRotation(double roll_rad, double pitch_rad, double yaw_rad);

/**
 * 旋转矩阵转欧拉角 (Roll, Pitch, Yaw)，单位弧度。
 */
Vector3d RotToEuler(const Matrix3d &R);

/**
 * 计算子午圈半径 R_M 和卯酉圈半径 R_N。
 * @param lat 纬度（弧度）
 * @return (R_M, R_N) 对
 */
std::pair<double, double> ComputeEarthRadius(double lat);

/**
 * 计算NED系下的地球自转角速度 ω_ie^n。
 * @param lat 纬度（弧度）
 * @return ω_ie^n 向量
 */
Vector3d OmegaIeNed(double lat);

/**
 * 计算NED系下的导航系旋转角速度 ω_en^n。
 * @param v_ned NED速度
 * @param lat 纬度（弧度）
 * @param h 高度（米）
 * @return ω_en^n 向量
 */
Vector3d OmegaEnNed(const Vector3d &v_ned, double lat, double h);

/**
 * 计算 ω_in^n = ω_ie^n + ω_en^n。
 */
Vector3d OmegaInNed(const Vector3d &v_ned, double lat, double h);

/**
 * 计算局部重力（NED系下，指向下方为正）。
 * @param lat 纬度（弧度）
 * @param h 高度（米）
 * @return 重力大小（m/s^2）
 */
double LocalGravity(double lat, double h);

/**
 * ECEF速度转换为NED速度。
 * @param v_ecef ECEF速度
 * @param lat 纬度（弧度）
 * @param lon 经度（弧度）
 * @return NED速度
 */
Vector3d EcefVelToNed(const Vector3d &v_ecef, double lat, double lon);

/**
 * NED速度转换为ECEF速度。
 * @param v_ned NED速度
 * @param lat 纬度（弧度）
 * @param lon 经度（弧度）
 * @return ECEF速度
 */
Vector3d NedVelToEcef(const Vector3d &v_ned, double lat, double lon);

/**
 * 将NED位置误差转换为ECEF坐标修正。
 * @param dr_ned NED位置误差（北东地，单位米）
 * @param lat 纬度（弧度）
 * @param lon 经度（弧度）
 * @param h 高度（米）
 * @return ECEF坐标修正
 */
Vector3d NedPosErrorToEcef(const Vector3d &dr_ned, double lat, double lon, double h);

/**
 * 将NED速度误差转换为ECEF速度修正。
 * @param dv_ned NED速度误差（m/s）
 * @param lat 纬度（弧度）
 * @param lon 经度（弧度）
 * @return ECEF速度修正
 */
Vector3d NedVelErrorToEcef(const Vector3d &dv_ned, double lat, double lon);
