#include "app/fusion.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "io/data_io.h"
#include "utils/math_utils.h"

using namespace std;
using namespace Eigen;

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

vector<ImuData> BuildImuSequence(const MatrixXd &imu_mat) {
  vector<ImuData> seq;
  seq.reserve(imu_mat.rows());
  for (int i = 0; i < imu_mat.rows(); ++i) {
    ImuData d;
    d.t = imu_mat(i, 0);
    d.dtheta = imu_mat.block<1, 3>(i, 1).transpose();
    d.dvel = imu_mat.block<1, 3>(i, 4).transpose();
    d.dt = (i == 0) ? 0.0 : max(0.0, imu_mat(i, 0) - imu_mat(i - 1, 0));
    seq.push_back(d);
  }
  return seq;
}

vector<ImuData> CropImu(const vector<ImuData> &in, double t0, double t1,
                        double tol, double max_dt) {
  vector<ImuData> out;
  for (const auto &d : in) {
    if (d.t + tol < t0 || d.t - tol > t1) continue;
    out.push_back(d);
  }
  for (size_t i = 0; i < out.size(); ++i) {
    out[i].dt = (i == 0) ? 0.0 : max(0.0, out[i].t - out[i - 1].t);
    if (max_dt > 0.0 && out[i].dt > max_dt) out[i].dt = 0.0;
  }
  return out;
}

MatrixXd CropMatrixRows(const MatrixXd &mat, double t0, double t1, double tol) {
  if (mat.rows() == 0) return mat;
  vector<int> keep;
  keep.reserve(mat.rows());
  for (int i = 0; i < mat.rows(); ++i) {
    double t = mat(i, 0);
    if (t + tol >= t0 && t - tol <= t1) keep.push_back(i);
  }
  MatrixXd out(keep.size(), mat.cols());
  for (size_t i = 0; i < keep.size(); ++i) out.row(i) = mat.row(keep[i]);
  return out;
}

int DetectNumericColumnCount(const string &path) {
  ifstream fin(path);
  if (!fin) return 0;

  string line;
  while (getline(fin, line)) {
    if (line.empty()) continue;
    replace(line.begin(), line.end(), ',', ' ');
    stringstream ss(line);
    vector<double> row_vals;
    double v = 0.0;
    while (ss >> v) row_vals.push_back(v);
    if (!row_vals.empty()) return static_cast<int>(row_vals.size());
  }
  return 0;
}

}  // namespace

Dataset LoadDataset(const FusionOptions &options) {
  Dataset data;

  MatrixXd imu_mat = io::LoadMatrix(options.imu_path, 7);

  MatrixXd uwb_mat;
  if (options.anchors.mode == "fixed" && options.anchors.positions.empty()) {
    uwb_mat.resize(0, 0);
  } else {
    int uwb_cols = 5;
    if (options.anchors.mode == "fixed" && !options.anchors.positions.empty()) {
      uwb_cols = 1 + static_cast<int>(options.anchors.positions.size());
    }
    uwb_mat = io::LoadMatrix(options.uwb_path, uwb_cols);
  }

  MatrixXd truth_raw = io::LoadMatrix(options.pos_path, 10);

  MatrixXd odo_mat;
  if (!options.odo_path.empty()) {
    odo_mat = io::LoadMatrix(options.odo_path, 2);
  }

  data.imu = BuildImuSequence(imu_mat);
  data.uwb = uwb_mat;
  data.odo = odo_mat;

  int n_truth = truth_raw.rows();
  data.truth.timestamps = truth_raw.col(0);
  data.truth.positions = truth_raw.block(0, 1, n_truth, 3);
  data.truth.velocities = truth_raw.block(0, 4, n_truth, 3);
  data.truth.quaternions.resize(n_truth, 4);

  bool truth_is_lla = (n_truth > 0 &&
      data.truth.positions.row(0).head<2>().cwiseAbs().maxCoeff() < 200.0);
  if (truth_is_lla) {
    cout << "[Load] Detected LLA/NED format in truth data, converting to ECEF...\n";
  }

  for (int i = 0; i < n_truth; ++i) {
    double lat = 0.0;
    double lon = 0.0;
    if (truth_is_lla) {
      lat = data.truth.positions(i, 0) * kDegToRad;
      lon = data.truth.positions(i, 1) * kDegToRad;
      double h = data.truth.positions(i, 2);
      Llh llh{lat, lon, h};
      Vector3d ecef = LlhToEcef(llh);
      data.truth.positions.row(i) = ecef.transpose();

      Matrix3d R_ne = RotNedToEcef(lat, lon);
      Vector3d v_ned = data.truth.velocities.row(i).transpose();
      data.truth.velocities.row(i) = (R_ne * v_ned).transpose();
    } else {
      Vector3d p = data.truth.positions.row(i);
      EcefToLatLon(p, lat, lon);
    }
    Matrix3d R_ne = RotNedToEcef(lat, lon);
    Matrix3d R_nb = EulerToRotation(truth_raw(i, 7) * kDegToRad,
                                    truth_raw(i, 8) * kDegToRad,
                                    truth_raw(i, 9) * kDegToRad);
    Quaterniond q(R_ne * R_nb);
    q.normalize();
    data.truth.quaternions.row(i) << q.w(), q.x(), q.y(), q.z();
  }

  data.anchors = BuildAnchors(options.anchors, data.truth.positions);

  double t0 = options.start_time;
  double t1 = options.final_time;
  if (options.init.use_truth_pva && data.truth.timestamps.size() > 0) {
    double truth_start = data.truth.timestamps(0);
    if (t0 < truth_start) {
      cout << "[Load] WARNING: start_time (" << fixed << t0
           << ") < truth start (" << truth_start
           << "), adjusting to truth start\n";
      t0 = truth_start;
    }
  }

  data.imu = CropImu(data.imu, t0, t1, options.gating.time_tolerance,
                     options.gating.max_dt);
  data.uwb = CropMatrixRows(data.uwb, t0, t1, options.gating.time_tolerance);
  data.odo = CropMatrixRows(data.odo, t0, t1, options.gating.time_tolerance);

  MatrixXd truth_all(data.truth.timestamps.size(), 11);
  truth_all.col(0) = data.truth.timestamps;
  truth_all.block(0, 1, data.truth.positions.rows(), 3) = data.truth.positions;
  truth_all.block(0, 4, data.truth.velocities.rows(), 3) = data.truth.velocities;
  truth_all.block(0, 7, data.truth.quaternions.rows(), 4) = data.truth.quaternions;
  truth_all = CropMatrixRows(truth_all, t0, t1, options.gating.time_tolerance);
  data.truth.timestamps = truth_all.col(0);
  data.truth.positions = truth_all.block(0, 1, truth_all.rows(), 3);
  data.truth.velocities = truth_all.block(0, 4, truth_all.rows(), 3);
  data.truth.quaternions = truth_all.block(0, 7, truth_all.rows(), 4);

  cout << "[Load] Time range: " << fixed << setprecision(6) << t0 << " to " << t1 << "\n";
  cout << "[Load] Truth rows raw: " << n_truth
       << ", filtered: " << data.truth.timestamps.size() << "\n";
  if (data.truth.timestamps.size() > 0) {
    cout << "[Load] Truth start t=" << data.truth.timestamps(0) << "\n";
    cout << "[Load] Truth start P=" << data.truth.positions.row(0) << "\n";
  } else {
    cout << "[Load] Truth empty after filtering!\n";
  }

  cout << "[Load] IMU pre-rotation: SKIPPED (IMU_converted already flipped y/z)\n";

  if (!options.gnss_path.empty()) {
    int gnss_cols = DetectNumericColumnCount(options.gnss_path);
    if (gnss_cols <= 0) {
      throw invalid_argument("error: GNSS 文件为空或无有效数值行: " + options.gnss_path);
    }
    if (gnss_cols < 7) {
      throw invalid_argument("error: GNSS 文件列数不足（至少需要7列）: " + options.gnss_path);
    }
    bool has_velocity = (gnss_cols >= 13);

    MatrixXd gnss_mat = io::LoadMatrix(options.gnss_path, has_velocity ? 13 : 7);
    if (gnss_mat.rows() <= 0) {
      throw invalid_argument("error: GNSS 数据加载失败: " + options.gnss_path);
    }

    data.gnss.timestamps = gnss_mat.col(0);
    data.gnss.std = gnss_mat.block(0, 4, gnss_mat.rows(), 3);

    bool has_velocity_data = false;
    if (has_velocity && gnss_mat.cols() >= 13) {
      data.gnss.velocities = gnss_mat.block(0, 7, gnss_mat.rows(), 3);
      data.gnss.vel_std = gnss_mat.block(0, 10, gnss_mat.rows(), 3);
      has_velocity_data =
          (data.gnss.velocities.rows() == gnss_mat.rows() &&
           data.gnss.vel_std.rows() == gnss_mat.rows());
    }

    bool gnss_is_lla = (gnss_mat.row(0).segment(1, 2).cwiseAbs().maxCoeff() < 200.0);
    if (gnss_is_lla) {
      MatrixXd gnss_ecef(gnss_mat.rows(), 3);
      MatrixXd gnss_vel_ecef(gnss_mat.rows(), 3);

      for (int i = 0; i < gnss_mat.rows(); ++i) {
        double lat_rad = gnss_mat(i, 1) * kDegToRad;
        double lon_rad = gnss_mat(i, 2) * kDegToRad;
        double h = gnss_mat(i, 3);
        Llh llh{lat_rad, lon_rad, h};
        gnss_ecef.row(i) = LlhToEcef(llh).transpose();

        if (has_velocity_data) {
          double vn = data.gnss.velocities(i, 0);
          double ve = data.gnss.velocities(i, 1);
          double vd = data.gnss.velocities(i, 2);
          Matrix3d R_ne = RotNedToEcef(lat_rad, lon_rad);
          Vector3d v_ned(vn, ve, vd);
          gnss_vel_ecef.row(i) = (R_ne * v_ned).transpose();
        }
      }
      data.gnss.positions = gnss_ecef;
      if (has_velocity_data) {
        data.gnss.velocities = gnss_vel_ecef;
      }
      cout << "[Load] GNSS data: " << gnss_mat.rows() << " records (converted LLA->ECEF)";
      if (has_velocity_data) {
        cout << " with velocity";
      }
      cout << "\n";
    } else {
      data.gnss.positions = gnss_mat.block(0, 1, gnss_mat.rows(), 3);
      cout << "[Load] GNSS data: " << gnss_mat.rows() << " records (already ECEF)";
      if (has_velocity_data) {
        cout << " with velocity";
      }
      cout << "\n";
    }

    double t_imu_start = data.imu.empty() ? t0 : data.imu.front().t;
    double t_imu_end = data.imu.empty() ? t1 : data.imu.back().t;
    double tol = options.gating.time_tolerance;
    vector<int> gnss_keep;
    gnss_keep.reserve(data.gnss.timestamps.size());
    for (int i = 0; i < data.gnss.timestamps.size(); ++i) {
      double tg = data.gnss.timestamps(i);
      if (tg + tol >= t_imu_start && tg - tol <= t_imu_end) {
        gnss_keep.push_back(i);
      }
    }
    if (static_cast<int>(gnss_keep.size()) < data.gnss.timestamps.size()) {
      int n_gnss = static_cast<int>(gnss_keep.size());
      VectorXd new_ts(n_gnss);
      MatrixXd new_pos(n_gnss, 3), new_std(n_gnss, 3);
      bool has_gnss_vel =
          (data.gnss.velocities.rows() == data.gnss.timestamps.size() &&
           data.gnss.vel_std.rows() == data.gnss.timestamps.size());
      MatrixXd new_vel, new_vel_std;
      if (has_gnss_vel) {
        new_vel.resize(n_gnss, 3);
        new_vel_std.resize(n_gnss, 3);
      }
      for (int i = 0; i < n_gnss; ++i) {
        new_ts(i) = data.gnss.timestamps(gnss_keep[i]);
        new_pos.row(i) = data.gnss.positions.row(gnss_keep[i]);
        new_std.row(i) = data.gnss.std.row(gnss_keep[i]);
        if (has_gnss_vel) {
          new_vel.row(i) = data.gnss.velocities.row(gnss_keep[i]);
          new_vel_std.row(i) = data.gnss.vel_std.row(gnss_keep[i]);
        }
      }
      cout << "[Load] GNSS cropped: " << data.gnss.timestamps.size()
           << " -> " << n_gnss << " (IMU time range)\n";
      data.gnss.timestamps = new_ts;
      data.gnss.positions = new_pos;
      data.gnss.std = new_std;
      if (has_gnss_vel) {
        data.gnss.velocities = new_vel;
        data.gnss.vel_std = new_vel_std;
      }
    }
  }

  return data;
}
