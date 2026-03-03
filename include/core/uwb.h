// UWB 相关：基站工具与量测模型声明
#pragma once

#include <Eigen/Dense>
#include <string>

#include "core/eskf.h"

using namespace std;
using namespace Eigen;

/**
 * 四基站坐标容器。
 * positions 每行对应一个基站的 (x, y, z) 坐标。
 */
struct Anchors {
  // 基站坐标，每行一个 (x, y, z)
  MatrixXd positions;
};

/**
 * 从字符串解析基站坐标。
 * 输入格式为 "x,y,z;..."，共 4 个基站。
 */
Anchors ParseAnchors(const string &anchor_str);

/**
 * 根据真值轨迹自动布设四角基站。
 * pos_xyz 为轨迹位置矩阵，margin 为外扩边距。
 */
Anchors AutoPlaceAnchors(const MatrixXd &pos_xyz, double margin = 1.0);

namespace MeasModels {

/**
 * UWB 量测模型输出。
 * y 为残差向量，H 为观测矩阵，R 为量测噪声。
 */
struct UwbModel {
  // 残差 y，观测矩阵 H，量测噪声 R
  VectorXd y;
  MatrixXd H;
  MatrixXd R;
};

/**
 * 速度约束伪量测模型输出。
 * y 为残差向量，H 为观测矩阵，R 为量测噪声。
 */
struct VelConstraintModel {
  VectorXd y;
  MatrixXd H;
  MatrixXd R;
};

/**
 * 计算 UWB 多基站观测模型。
 * @param state 当前名义状态
 * @param z 实测多基站距离
 * @param anchors 基站坐标矩阵
 * @param sigma_uwb 量测标准差
 * @return 残差/雅可比/噪声矩阵
 */
UwbModel ComputeUwbModel(const State &state, const VectorXd &z,
                         const MatrixXd &anchors,
                         double sigma_uwb);

/**
 * 零速更新（ZUPT）模型：速度约束为 0。
 * @param state 当前名义状态
 * @param sigma_zupt 速度观测标准差
 * @return 残差/雅可比/噪声矩阵
 */
VelConstraintModel ComputeZuptModel(const State &state, double sigma_zupt);

/**
 * 非完整约束（NHC）：车体侧向/垂向速度为 0。
 * @param state 当前名义状态
 * @param sigma_nhc_y 侧向速度观测标准差
 * @param sigma_nhc_z 垂向速度观测标准差
 * @return 残差/雅可比/噪声矩阵
 */
VelConstraintModel ComputeNhcModel(const State &state, const Matrix3d &C_b_v,
                                   const Vector3d &omega_ib_b_raw,
                                   double sigma_nhc_y, double sigma_nhc_z,
                                   const FejManager *fej = nullptr);

/**
 * 里程计（ODO）前向速度约束：车体前向速度为量测值。
 * 考虑杆臂效应。
 * @param state 当前名义状态
 * @param odo_speed 里程计速度读数
 * @param lever_arm 里程计在 IMU 系下的位置 (FRD)
 * @param C_b_v IMU 到车体坐标系的旋转矩阵
 * @param omega_ib_b_raw 陀螺原始角速度测量值 (rad/s, b系)
 * @param sigma_odo 观测标准差
 * @return 残差/雅可比/噪声矩阵
 */
VelConstraintModel ComputeOdoModel(const State &state, double odo_speed,
                                   const Matrix3d &C_b_v,
                                   const Vector3d &omega_ib_b_raw,
                                   double sigma_odo,
                                   const FejManager *fej = nullptr);

/**
 * GNSS位置量测模型。
 * 将IMU中心位置通过GNSS杆臂转换到GNSS天线相位中心。
 * @param state 当前名义状态
 * @param z GNSS量测位置 (ECEF, m)
 * @param sigma_gnss GNSS位置量测标准差各向 (m)  [sigma_x, sigma_y, sigma_z]
 * @return 残差/雅可比/噪声矩阵
 */
UwbModel ComputeGnssPositionModel(const State &state, const Vector3d &z,
                                  const Vector3d &sigma_gnss,
                                  const FejManager *fej = nullptr);

/**
 * GNSS速度量测模型。
 * 考虑GNSS杆臂效应：v_gnss = v_imu + C_bn * (omega_b × l_gnss)
 * @param state 当前名义状态
 * @param z_gnss_vel GNSS量测速度 (ECEF, m/s)
 * @param omega_ib_b_raw 陀螺原始角速度测量值 (rad/s, b系)
 * @param sigma_gnss_vel GNSS速度量测标准差各向 (m/s)  [sigma_vx, sigma_vy, sigma_vz]
 * @return 残差/雅可比/噪声矩阵
 */
UwbModel ComputeGnssVelocityModel(const State &state, const Vector3d &z_gnss_vel,
                                  const Vector3d &omega_ib_b_raw, const Vector3d &sigma_gnss_vel,
                                  const FejManager *fej = nullptr);

}  // namespace MeasModels
