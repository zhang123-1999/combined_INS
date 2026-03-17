#include "app/fusion.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "io/data_io.h"
#include "utils/math_utils.h"

using namespace std;
using namespace Eigen;
namespace fs = std::filesystem;

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kRadPerSecToDegPerHour = kRadToDeg * 3600.0;
constexpr double kMs2ToMgal = 1.0e5;

Vector3d ResolveMountingBaseRpyRad(const FusionOptions &options) {
  Vector3d cfg_mounting_rpy = options.constraints.imu_mounting_angle * kDegToRad;
  if (!options.init.use_legacy_mounting_base_logic) {
    return cfg_mounting_rpy;
  }

  constexpr double kMountInitEps = 1e-12;
  bool init_pitch_nonzero = std::abs(options.init.mounting_pitch0) > kMountInitEps;
  bool init_yaw_nonzero = std::abs(options.init.mounting_yaw0) > kMountInitEps;
  return Vector3d(cfg_mounting_rpy.x(),
                  init_pitch_nonzero ? 0.0 : cfg_mounting_rpy.y(),
                  init_yaw_nonzero ? 0.0 : cfg_mounting_rpy.z());
}

void EnsureParentDir(const string &path) {
  fs::path out_path(path);
  if (!out_path.parent_path().empty()) {
    fs::create_directories(out_path.parent_path());
  }
}

void EnsureSeriesSize(size_t expected, size_t actual, const string &name) {
  if (actual != expected) {
    throw invalid_argument("state series size mismatch: " + name);
  }
}
}  // namespace

/**
 * 评估融合结果并计算 RMSE。
 * 将真值插值到融合时间轴，输出 RMSE 与保存用矩阵。
 */
EvaluationSummary EvaluateFusion(const FusionResult &result, const Dataset &dataset) {
  EvaluationSummary summary;
  const int n = static_cast<int>(result.fused_positions.size());
  if (n == 0 || result.fused_velocities.size() != static_cast<size_t>(n) ||
      result.fused_quaternions.size() != static_cast<size_t>(n)) {
    cout << "error: 结果长度异常，无法评估\n";
    return summary;
  }

  MatrixXd fused_mat(n, 3), fused_vel(n, 3), fused_quat(n, 4);
  for (int i = 0; i < n; ++i) {
    fused_mat.row(i) = result.fused_positions[i];
    fused_vel.row(i) = result.fused_velocities[i];
    fused_quat.row(i) = result.fused_quaternions[i];
  }

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

  summary.output_matrix.resize(n, 31);
  for (int i = 0; i < n; ++i) {
    summary.output_matrix(i, 0) = result.time_axis[i];
    summary.output_matrix.block<1, 3>(i, 1) = fused_mat.row(i);
    summary.output_matrix.block<1, 3>(i, 4) = fused_vel.row(i);

    Vector3d p_i = fused_mat.row(i);
    Vector4d q_i = fused_quat.row(i);
    Quaterniond q_eb(q_i(0), q_i(1), q_i(2), q_i(3));

    double lat = 0.0;
    double lon = 0.0;
    EcefToLatLon(p_i, lat, lon);
    Matrix3d R_ne = RotNedToEcef(lat, lon);
    Matrix3d R_eb = q_eb.toRotationMatrix();
    Matrix3d R_bn = R_ne.transpose() * R_eb;
    Vector3d rpy = RotToEuler(R_bn);

    summary.output_matrix(i, 7) = rpy(0) * kRadToDeg;
    summary.output_matrix(i, 8) = rpy(1) * kRadToDeg;
    summary.output_matrix(i, 9) = rpy(2) * kRadToDeg;
    summary.output_matrix(i, 10) = result.mounting_pitch[i] * kRadToDeg;
    summary.output_matrix(i, 11) = result.mounting_yaw[i] * kRadToDeg;
    summary.output_matrix(i, 12) = result.odo_scale[i];
    summary.output_matrix(i, 13) = result.sg[i].x() * 1e6;
    summary.output_matrix(i, 14) = result.sg[i].y() * 1e6;
    summary.output_matrix(i, 15) = result.sg[i].z() * 1e6;
    summary.output_matrix(i, 16) = result.sa[i].x() * 1e6;
    summary.output_matrix(i, 17) = result.sa[i].y() * 1e6;
    summary.output_matrix(i, 18) = result.sa[i].z() * 1e6;
    summary.output_matrix(i, 19) = result.ba[i].x();
    summary.output_matrix(i, 20) = result.ba[i].y();
    summary.output_matrix(i, 21) = result.ba[i].z();
    summary.output_matrix(i, 22) = result.bg[i].x();
    summary.output_matrix(i, 23) = result.bg[i].y();
    summary.output_matrix(i, 24) = result.bg[i].z();
    summary.output_matrix(i, 25) = result.lever_arm[i].x();
    summary.output_matrix(i, 26) = result.lever_arm[i].y();
    summary.output_matrix(i, 27) = result.lever_arm[i].z();
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

/**
 * 导出实验专用状态序列文件。
 */
void SaveStateSeries(const string &path, const FusionResult &result,
                     const FusionOptions &options) {
  const size_t n = result.time_axis.size();
  EnsureSeriesSize(n, result.ba.size(), "ba");
  EnsureSeriesSize(n, result.bg.size(), "bg");
  EnsureSeriesSize(n, result.sg.size(), "sg");
  EnsureSeriesSize(n, result.sa.size(), "sa");
  EnsureSeriesSize(n, result.odo_scale.size(), "odo_scale");
  EnsureSeriesSize(n, result.mounting_roll.size(), "mounting_roll");
  EnsureSeriesSize(n, result.mounting_pitch.size(), "mounting_pitch");
  EnsureSeriesSize(n, result.mounting_yaw.size(), "mounting_yaw");
  EnsureSeriesSize(n, result.lever_arm.size(), "lever_arm");
  EnsureSeriesSize(n, result.gnss_lever_arm.size(), "gnss_lever_arm");

  EnsureParentDir(path);
  ofstream fout(path);
  if (!fout.is_open()) {
    throw runtime_error("failed to open state series output: " + path);
  }

  Vector3d mounting_base_rpy = ResolveMountingBaseRpyRad(options);
  fout << fixed << setprecision(9);
  fout << "timestamp,"
       << "ba_x_mgal,ba_y_mgal,ba_z_mgal,"
       << "bg_x_degh,bg_y_degh,bg_z_degh,"
       << "sg_x_ppm,sg_y_ppm,sg_z_ppm,"
       << "sa_x_ppm,sa_y_ppm,sa_z_ppm,"
       << "odo_scale,"
       << "mounting_roll_deg,mounting_pitch_deg,mounting_yaw_deg,"
       << "total_mounting_roll_deg,total_mounting_pitch_deg,total_mounting_yaw_deg,"
       << "odo_lever_x_m,odo_lever_y_m,odo_lever_z_m,"
       << "gnss_lever_x_m,gnss_lever_y_m,gnss_lever_z_m\n";

  for (size_t i = 0; i < n; ++i) {
    double mounting_roll = result.mounting_roll[i];
    double mounting_pitch = result.mounting_pitch[i];
    double mounting_yaw = result.mounting_yaw[i];
    Vector3d total_mounting = mounting_base_rpy +
                              Vector3d(mounting_roll, mounting_pitch, mounting_yaw);

    fout << result.time_axis[i] << ","
         << result.ba[i].x() * kMs2ToMgal << ","
         << result.ba[i].y() * kMs2ToMgal << ","
         << result.ba[i].z() * kMs2ToMgal << ","
         << result.bg[i].x() * kRadPerSecToDegPerHour << ","
         << result.bg[i].y() * kRadPerSecToDegPerHour << ","
         << result.bg[i].z() * kRadPerSecToDegPerHour << ","
         << result.sg[i].x() * 1e6 << ","
         << result.sg[i].y() * 1e6 << ","
         << result.sg[i].z() * 1e6 << ","
         << result.sa[i].x() * 1e6 << ","
         << result.sa[i].y() * 1e6 << ","
         << result.sa[i].z() * 1e6 << ","
         << result.odo_scale[i] << ","
         << mounting_roll * kRadToDeg << ","
         << mounting_pitch * kRadToDeg << ","
         << mounting_yaw * kRadToDeg << ","
         << total_mounting.x() * kRadToDeg << ","
         << total_mounting.y() * kRadToDeg << ","
         << total_mounting.z() * kRadToDeg << ","
         << result.lever_arm[i].x() << ","
         << result.lever_arm[i].y() << ","
         << result.lever_arm[i].z() << ","
         << result.gnss_lever_arm[i].x() << ","
         << result.gnss_lever_arm[i].y() << ","
         << result.gnss_lever_arm[i].z() << "\n";
  }
}
