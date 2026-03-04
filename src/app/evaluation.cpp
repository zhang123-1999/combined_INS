#include "app/fusion.h"

#include <iostream>
#include <limits>

#include "io/data_io.h"
#include "utils/math_utils.h"

using namespace std;
using namespace Eigen;

namespace {
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
// RotToEuler 已统一到 math_utils.h
}

/**
 * 评估融合结果并计算 RMSE。
 * 将真值插值到融合时间轴，输出 RMSE 与保存用矩阵。
 */
EvaluationSummary EvaluateFusion(const FusionResult &result, const Dataset &dataset) {
  EvaluationSummary summary;
  const int n = static_cast<int>(result.fused_positions.size());
  if (n == 0 || result.fused_velocities.size() != static_cast<size_t>(n) ||
      result.fused_quaternions.size() != static_cast<size_t>(n)) {
    // 长度异常直接返回
    cout << "error: 结果长度异常，无法评估\n";
    return summary;
  }

  MatrixXd fused_mat(n, 3), fused_vel(n, 3), fused_quat(n, 4);
  for (int i = 0; i < n; ++i) {
    fused_mat.row(i) = result.fused_positions[i];
    fused_vel.row(i) = result.fused_velocities[i];
    fused_quat.row(i) = result.fused_quaternions[i];
  }

  // 将真值时间插值到融合时间轴并计算 RMSE（真值缺失时仅输出轨迹，不计算 RMSE）。
  bool has_truth = dataset.truth.timestamps.size() >= 2 &&
                   dataset.truth.positions.rows() == dataset.truth.timestamps.size() &&
                   dataset.truth.positions.cols() >= 3;
  if (has_truth) {
    MatrixXd truth_interp = MatrixXd::Zero(n, 3);
    for (int j = 0; j < 3; ++j) {
      VectorXd col = dataset.truth.positions.col(j);
      const VectorXd &t_truth = dataset.truth.timestamps;
      for (int i = 0; i < n; ++i) {
        truth_interp(i, j) = LinearInterp(t_truth, col, result.time_axis[i]);
      }
    }
    summary.rmse_fused = RMSE(fused_mat, truth_interp);
  } else {
    cout << "[Warn] 真值不可用，跳过 RMSE 计算（仅输出解算结果）\n";
    double nan_v = std::numeric_limits<double>::quiet_NaN();
    summary.rmse_fused = Vector3d(nan_v, nan_v, nan_v);
  }

  // 组装输出矩阵：时间、融合 XYZ、融合速度 XYZ、融合姿态欧拉角、安装角（deg）、
  // ODO比例、sg(ppm)、sa(ppm)、ba、bg、ODO杆臂、GNSS杆臂
  summary.output_matrix.resize(n, 31);
  for (int i = 0; i < n; ++i) {
    summary.output_matrix(i, 0) = result.time_axis[i];
    summary.output_matrix.block<1, 3>(i, 1) = fused_mat.row(i);
    summary.output_matrix.block<1, 3>(i, 4) = fused_vel.row(i);
    
    // Convert ECEF Quaternion to NED Euler
    Vector3d p_i = fused_mat.row(i);
    Vector4d q_i = fused_quat.row(i); // w, x, y, z
    Quaterniond q_eb(q_i(0), q_i(1), q_i(2), q_i(3));
    
    double lat, lon;
    EcefToLatLon(p_i, lat, lon);
    Matrix3d R_ne = RotNedToEcef(lat, lon);
    Matrix3d R_eb = q_eb.toRotationMatrix();  // body→ECEF
    Matrix3d R_bn = R_ne.transpose() * R_eb;  // body→NED
    Vector3d rpy = RotToEuler(R_bn);
    
    // Store in Degrees
    summary.output_matrix(i, 7) = rpy(0) * kRadToDeg;
    summary.output_matrix(i, 8) = rpy(1) * kRadToDeg;
    summary.output_matrix(i, 9) = rpy(2) * kRadToDeg;
    // 安装角（rad→deg）和 ODO 比例因子
    summary.output_matrix(i, 10) = result.mounting_pitch[i] * kRadToDeg;
    summary.output_matrix(i, 11) = result.mounting_yaw[i] * kRadToDeg;
    summary.output_matrix(i, 12) = result.odo_scale[i];
    // 比例因子（无量纲 → ppm）
    summary.output_matrix(i, 13) = result.sg[i].x() * 1e6;
    summary.output_matrix(i, 14) = result.sg[i].y() * 1e6;
    summary.output_matrix(i, 15) = result.sg[i].z() * 1e6;
    summary.output_matrix(i, 16) = result.sa[i].x() * 1e6;
    summary.output_matrix(i, 17) = result.sa[i].y() * 1e6;
    summary.output_matrix(i, 18) = result.sa[i].z() * 1e6;
    // 零偏
    summary.output_matrix(i, 19) = result.ba[i].x();
    summary.output_matrix(i, 20) = result.ba[i].y();
    summary.output_matrix(i, 21) = result.ba[i].z();
    summary.output_matrix(i, 22) = result.bg[i].x();
    summary.output_matrix(i, 23) = result.bg[i].y();
    summary.output_matrix(i, 24) = result.bg[i].z();
    // 杆臂
    summary.output_matrix(i, 25) = result.lever_arm[i].x();
    summary.output_matrix(i, 26) = result.lever_arm[i].y();
    summary.output_matrix(i, 27) = result.lever_arm[i].z();
    // GNSS杆臂
    summary.output_matrix(i, 28) = result.gnss_lever_arm[i].x();
    summary.output_matrix(i, 29) = result.gnss_lever_arm[i].y();
    summary.output_matrix(i, 30) = result.gnss_lever_arm[i].z();
  }
  return summary;
}

/**
 * 保存融合评估结果矩阵到文件。
 * @param path 输出路径
 * @param summary 评估摘要（含 output_matrix）
 */
void SaveFusionResult(const string &path, const EvaluationSummary &summary) {
  io::SaveMatrix(path, summary.output_matrix,
                 "timestamp fused_x fused_y fused_z fused_vx fused_vy fused_vz "
                 "fused_roll fused_pitch fused_yaw mounting_pitch mounting_yaw odo_scale "
                 "sg_x sg_y sg_z sa_x sa_y sa_z "
                 "ba_x ba_y ba_z bg_x bg_y bg_z "
                 "lever_x lever_y lever_z "
                 "gnss_lever_x gnss_lever_y gnss_lever_z");
}
