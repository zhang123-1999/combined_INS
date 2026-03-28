#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>

#include "app/fusion.h"
#include "core/eskf.h"
#include "core/uwb.h"
#include "io/data_io.h"
#include "utils/math_utils.h"

using namespace std;
using namespace Eigen;
namespace fs = std::filesystem;

namespace {

constexpr double kDegToRad = EIGEN_PI / 180.0;
constexpr double kRadToDeg = 180.0 / EIGEN_PI;

struct SolRecord {
  double t = 0.0;
  State state;
  Vector3d fused_rpy_rad{Vector3d::Zero()};
  Vector3d v_ned{Vector3d::Zero()};
  double speed_h = 0.0;
  double yaw_rate_rad_s = 0.0;
  double turn_score = 0.0;
};

struct AuditSample {
  string measurement;
  int record_idx = -1;
  int meas_idx = -1;
  double t_meas = 0.0;
  double t_state = 0.0;
  double speed_h = 0.0;
  double yaw_rate_rad_s = 0.0;
  double turn_score = 0.0;
};

struct AuditModel {
  VectorXd y;
  MatrixXd H;
};

struct BlockSpec {
  string name;
  int start = 0;
  int size = 0;
};

struct BlockError {
  string dataset;
  string measurement;
  double t = 0.0;
  double speed_h = 0.0;
  double yaw_rate_deg_s = 0.0;
  string block_name;
  double analytic_norm = 0.0;
  double numeric_norm = 0.0;
  double diff_norm = 0.0;
  double rel_fro = 0.0;
  double max_abs = 0.0;
  int worst_row = -1;
  int worst_col = -1;
  string worst_state;
};

struct ColumnError {
  string dataset;
  string measurement;
  double t = 0.0;
  double speed_h = 0.0;
  double yaw_rate_deg_s = 0.0;
  int col = -1;
  string state_name;
  double analytic_norm = 0.0;
  double numeric_norm = 0.0;
  double diff_norm = 0.0;
  double rel = 0.0;
  double max_abs = 0.0;
};

struct AuditSummary {
  string dataset;
  string config_path;
  string sol_path;
  string sol_mtime;
  double split_t = 0.0;
  bool split_cov_valid = false;
  string split_cov_tag;
  double split_cov_t_meas = 0.0;
  double split_cov_t_state = 0.0;
  Vector3d split_cov_att_bgz{Vector3d::Zero()};
  Vector3d split_corr_att_bgz{Vector3d::Zero()};
  Vector3d split_att_var{Vector3d::Zero()};
  double split_bgz_var = 0.0;
  bool reset_consistency_valid = false;
  string reset_tag;
  double reset_t_meas = 0.0;
  double reset_t_state = 0.0;
  bool reset_floor_applied = false;
  double reset_expected_norm = 0.0;
  double reset_actual_norm = 0.0;
  double reset_diff_norm = 0.0;
  double reset_rel_fro = 0.0;
  double reset_max_abs = 0.0;
  int reset_worst_row = -1;
  int reset_worst_col = -1;
  string reset_p_tilde_path;
  string reset_p_expected_path;
  string reset_p_after_reset_path;
  string reset_dx_path;
  vector<AuditSample> samples;
  vector<BlockError> block_errors;
  vector<ColumnError> column_errors;
};

string ToStringPrec(double v, int prec = 6) {
  ostringstream oss;
  oss << fixed << setprecision(prec) << v;
  return oss.str();
}

string FormatFileTime(const fs::file_time_type &ft) {
  using namespace std::chrono;
  const auto sctp = time_point_cast<system_clock::duration>(
      ft - fs::file_time_type::clock::now() + system_clock::now());
  time_t tt = system_clock::to_time_t(sctp);
  tm tm_local{};
#ifdef _WIN32
  localtime_s(&tm_local, &tt);
#else
  localtime_r(&tt, &tm_local);
#endif
  ostringstream oss;
  oss << put_time(&tm_local, "%Y-%m-%d %H:%M:%S");
  return oss.str();
}

string DatasetTagFromPath(const string &path) {
  string lower = path;
  transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
    return static_cast<char>(tolower(c));
  });
  if (lower.find("data2") != string::npos) return "data2";
  if (lower.find("data4") != string::npos) return "data4";
  fs::path p(path);
  return p.stem().string();
}

Matrix<double, kStateDim, kStateDim> BuildTrueInEkfResetGammaFromDx(
    const Matrix<double, kStateDim, 1> &dx) {
  Matrix<double, kStateDim, kStateDim> Gamma =
      Matrix<double, kStateDim, kStateDim>::Identity();
  Vector3d rho_p_body = dx.segment<3>(StateIdx::kPos);
  Vector3d rho_v_body = dx.segment<3>(StateIdx::kVel);
  Vector3d phi_body = dx.segment<3>(StateIdx::kAtt);
  Matrix3d core_reset = QuatToRot(QuatFromSmallAngle(-phi_body));
  Gamma.block<3, 3>(StateIdx::kPos, StateIdx::kPos) = core_reset;
  Gamma.block<3, 3>(StateIdx::kVel, StateIdx::kVel) = core_reset;
  Gamma.block<3, 3>(StateIdx::kAtt, StateIdx::kAtt) = core_reset;
  Gamma.block<3, 3>(StateIdx::kPos, StateIdx::kAtt) =
      -Skew(rho_p_body) * core_reset;
  Gamma.block<3, 3>(StateIdx::kVel, StateIdx::kAtt) =
      -Skew(rho_v_body) * core_reset;
  return Gamma;
}

vector<string> BuildStateNames() {
  vector<string> names(kStateDim);
  names[0] = "pos_x";
  names[1] = "pos_y";
  names[2] = "pos_z";
  names[3] = "vel_x";
  names[4] = "vel_y";
  names[5] = "vel_z";
  names[6] = "att_x";
  names[7] = "att_y";
  names[8] = "att_z";
  names[9] = "ba_x";
  names[10] = "ba_y";
  names[11] = "ba_z";
  names[12] = "bg_x";
  names[13] = "bg_y";
  names[14] = "bg_z";
  names[15] = "sg_x";
  names[16] = "sg_y";
  names[17] = "sg_z";
  names[18] = "sa_x";
  names[19] = "sa_y";
  names[20] = "sa_z";
  names[21] = "odo_scale";
  names[22] = "mount_roll";
  names[23] = "mount_pitch";
  names[24] = "mount_yaw";
  names[25] = "lever_x";
  names[26] = "lever_y";
  names[27] = "lever_z";
  names[28] = "gnss_lever_x";
  names[29] = "gnss_lever_y";
  names[30] = "gnss_lever_z";
  return names;
}

vector<BlockSpec> BuildFocusBlocks(const string &measurement) {
  vector<BlockSpec> blocks;
  if (measurement == "GNSS_POS") {
    blocks.push_back({"pos", StateIdx::kPos, 3});
    blocks.push_back({"att", StateIdx::kAtt, 3});
    blocks.push_back({"gnss_lever", StateIdx::kGnssLever, 3});
    return blocks;
  }
  if (measurement == "GNSS_VEL") {
    blocks.push_back({"vel", StateIdx::kVel, 3});
    blocks.push_back({"att", StateIdx::kAtt, 3});
    blocks.push_back({"bg", StateIdx::kBg, 3});
    blocks.push_back({"sg", StateIdx::kSg, 3});
    blocks.push_back({"gnss_lever", StateIdx::kGnssLever, 3});
    return blocks;
  }

  blocks.push_back({"vel", StateIdx::kVel, 3});
  blocks.push_back({"att", StateIdx::kAtt, 3});
  blocks.push_back({"bg", StateIdx::kBg, 3});
  blocks.push_back({"sg", StateIdx::kSg, 3});
  if (measurement == "ODO") {
    blocks.push_back({"odo_scale", StateIdx::kOdoScale, 1});
  }
  blocks.push_back({"mount_roll", StateIdx::kMountRoll, 1});
  blocks.push_back({"mount_pitch", StateIdx::kMountPitch, 1});
  blocks.push_back({"mount_yaw", StateIdx::kMountYaw, 1});
  blocks.push_back({"lever", StateIdx::kLever, 3});
  return blocks;
}

double WrapToPi(double angle_rad) {
  while (angle_rad > EIGEN_PI) angle_rad -= 2.0 * EIGEN_PI;
  while (angle_rad < -EIGEN_PI) angle_rad += 2.0 * EIGEN_PI;
  return angle_rad;
}

Vector3d ComputeMountingBaseRpyRad(const FusionOptions &options) {
  Vector3d cfg_mounting_rpy = options.constraints.imu_mounting_angle * kDegToRad;
  constexpr double kMountInitEps = 1e-12;
  bool init_pitch_nonzero = std::abs(options.init.mounting_pitch0) > kMountInitEps;
  bool init_yaw_nonzero = std::abs(options.init.mounting_yaw0) > kMountInitEps;
  Vector3d mounting_base_rpy = cfg_mounting_rpy;
  if (options.init.use_legacy_mounting_base_logic) {
    bool use_cfg_pitch_base = !init_pitch_nonzero;
    bool use_cfg_yaw_base = !init_yaw_nonzero;
    mounting_base_rpy = Vector3d(cfg_mounting_rpy.x(),
                                 use_cfg_pitch_base ? cfg_mounting_rpy.y() : 0.0,
                                 use_cfg_yaw_base ? cfg_mounting_rpy.z() : 0.0);
  }
  return mounting_base_rpy;
}

Matrix3d BuildVehicleRotation(const Vector3d &mounting_base_rpy,
                              const State &state) {
  Vector3d rpy(mounting_base_rpy.x(),
               mounting_base_rpy.y() + state.mounting_pitch,
               mounting_base_rpy.z() + state.mounting_yaw);
  return QuatToRot(RpyToQuat(rpy)).transpose();
}

State ParseStateFromSolRow(const RowVectorXd &row) {
  State state;
  state.p = row.segment<3>(1).transpose();
  state.v = row.segment<3>(4).transpose();

  Vector3d rpy_rad = row.segment<3>(7).transpose() * kDegToRad;
  double lat = 0.0;
  double lon = 0.0;
  EcefToLatLon(state.p, lat, lon);
  Matrix3d R_ne = RotNedToEcef(lat, lon);
  Matrix3d R_bn = EulerToRotation(rpy_rad(0), rpy_rad(1), rpy_rad(2));
  Matrix3d R_eb = R_ne * R_bn;
  Quaterniond q_eb(R_eb);
  q_eb.normalize();
  state.q << q_eb.w(), q_eb.x(), q_eb.y(), q_eb.z();

  state.mounting_roll = 0.0;
  state.mounting_pitch = row(10) * kDegToRad;
  state.mounting_yaw = row(11) * kDegToRad;
  state.odo_scale = row(12);
  state.sg = row.segment<3>(13).transpose() / 1e6;
  state.sa = row.segment<3>(16).transpose() / 1e6;
  state.ba = row.segment<3>(19).transpose();
  state.bg = row.segment<3>(22).transpose();
  state.lever_arm = row.segment<3>(25).transpose();
  state.gnss_lever_arm = row.segment<3>(28).transpose();
  return state;
}

vector<SolRecord> LoadSolutionRecords(const string &sol_path) {
  MatrixXd sol = io::LoadMatrix(sol_path, 31);
  if (sol.rows() == 0) {
    throw runtime_error("solution file is empty or unreadable: " + sol_path);
  }

  vector<SolRecord> records(sol.rows());
  vector<double> yaw_raw(sol.rows(), 0.0);
  vector<double> yaw_unwrapped(sol.rows(), 0.0);

  for (int i = 0; i < sol.rows(); ++i) {
    const RowVectorXd row = sol.row(i);
    records[i].t = row(0);
    records[i].state = ParseStateFromSolRow(row);
    records[i].fused_rpy_rad = row.segment<3>(7).transpose() * kDegToRad;
    double lat = 0.0;
    double lon = 0.0;
    EcefToLatLon(records[i].state.p, lat, lon);
    records[i].v_ned = EcefVelToNed(records[i].state.v, lat, lon);
    records[i].speed_h = records[i].v_ned.head<2>().norm();
    yaw_raw[i] = records[i].fused_rpy_rad.z();
  }

  yaw_unwrapped[0] = yaw_raw[0];
  for (int i = 1; i < static_cast<int>(yaw_raw.size()); ++i) {
    double dy = WrapToPi(yaw_raw[i] - yaw_raw[i - 1]);
    yaw_unwrapped[i] = yaw_unwrapped[i - 1] + dy;
  }

  for (int i = 0; i < static_cast<int>(records.size()); ++i) {
    double yaw_rate = 0.0;
    if (records.size() >= 3) {
      if (i == 0) {
        double dt = max(1e-9, records[1].t - records[0].t);
        yaw_rate = (yaw_unwrapped[1] - yaw_unwrapped[0]) / dt;
      } else if (i == static_cast<int>(records.size()) - 1) {
        double dt = max(1e-9, records[i].t - records[i - 1].t);
        yaw_rate = (yaw_unwrapped[i] - yaw_unwrapped[i - 1]) / dt;
      } else {
        double dt = max(1e-9, records[i + 1].t - records[i - 1].t);
        yaw_rate = (yaw_unwrapped[i + 1] - yaw_unwrapped[i - 1]) / dt;
      }
    }
    records[i].yaw_rate_rad_s = yaw_rate;
    records[i].turn_score = records[i].speed_h * std::abs(yaw_rate);
  }
  return records;
}

double ComputeSplitTime(const FusionOptions &options, const Dataset &dataset) {
  if (!options.gnss_schedule.enabled || dataset.imu.empty()) {
    return numeric_limits<double>::infinity();
  }
  double t0_nav = dataset.imu.front().t;
  double t1_nav = dataset.imu.back().t;
  return t0_nav + options.gnss_schedule.head_ratio * (t1_nav - t0_nav);
}

vector<AuditSample> SelectTurningSamples(const vector<SolRecord> &records,
                                         double split_t,
                                         int max_samples,
                                         double min_separation_s) {
  vector<AuditSample> candidates;
  for (int i = 0; i < static_cast<int>(records.size()); ++i) {
    if (!(records[i].t > split_t)) continue;
    if (!std::isfinite(records[i].turn_score)) continue;
    AuditSample sample;
    sample.measurement = "ROAD";
    sample.record_idx = i;
    sample.meas_idx = i;
    sample.t_meas = records[i].t;
    sample.t_state = records[i].t;
    sample.speed_h = records[i].speed_h;
    sample.yaw_rate_rad_s = records[i].yaw_rate_rad_s;
    sample.turn_score = records[i].turn_score;
    candidates.push_back(sample);
  }

  sort(candidates.begin(), candidates.end(), [](const AuditSample &a, const AuditSample &b) {
    return a.turn_score > b.turn_score;
  });

  vector<AuditSample> selected;
  for (const auto &candidate : candidates) {
    bool separated = true;
    for (const auto &chosen : selected) {
      if (std::abs(candidate.t_state - chosen.t_state) < min_separation_s) {
        separated = false;
        break;
      }
    }
    if (!separated) continue;
    selected.push_back(candidate);
    if (static_cast<int>(selected.size()) >= max_samples) break;
  }

  sort(selected.begin(), selected.end(), [](const AuditSample &a, const AuditSample &b) {
    return a.t_state < b.t_state;
  });
  return selected;
}

int FindClosestImuIndex(const vector<ImuData> &imu, double t_query) {
  if (imu.empty()) return -1;
  auto it = lower_bound(imu.begin(), imu.end(), t_query,
                        [](const ImuData &sample, double t) { return sample.t < t; });
  if (it == imu.begin()) return 0;
  if (it == imu.end()) return static_cast<int>(imu.size()) - 1;
  int idx_hi = static_cast<int>(distance(imu.begin(), it));
  int idx_lo = idx_hi - 1;
  if (std::abs(imu[idx_hi].t - t_query) < std::abs(imu[idx_lo].t - t_query)) {
    return idx_hi;
  }
  return idx_lo;
}

int FindClosestRecordIndex(const vector<SolRecord> &records, double t_query) {
  if (records.empty()) return -1;
  auto it = lower_bound(records.begin(), records.end(), t_query,
                        [](const SolRecord &record, double t) { return record.t < t; });
  if (it == records.begin()) return 0;
  if (it == records.end()) return static_cast<int>(records.size()) - 1;
  int idx_hi = static_cast<int>(distance(records.begin(), it));
  int idx_lo = idx_hi - 1;
  if (std::abs(records[idx_hi].t - t_query) < std::abs(records[idx_lo].t - t_query)) {
    return idx_hi;
  }
  return idx_lo;
}


bool ComputeOdoMeasurementAtTime(const Dataset &dataset,
                                 const ConstraintConfig &cfg,
                                 const GatingConfig &gating,
                                 double t_query,
                                 double &odo_speed_out) {
  if (dataset.odo.rows() == 0) {
    return false;
  }

  int best_idx = -1;
  for (int i = 0; i < dataset.odo.rows(); ++i) {
    double t_odo = dataset.odo(i, 0) + cfg.odo_time_offset;
    if (t_odo <= t_query + gating.time_tolerance) {
      best_idx = i;
    } else {
      break;
    }
  }
  if (best_idx < 0) {
    return false;
  }

  double t_odo = dataset.odo(best_idx, 0) + cfg.odo_time_offset;
  double odo_vel = dataset.odo(best_idx, 1);
  int next = best_idx + 1;
  if (next < dataset.odo.rows()) {
    double t_next = dataset.odo(next, 0) + cfg.odo_time_offset;
    double dt_odo = t_next - t_odo;
    if (dt_odo > 1e-6 && t_query >= t_odo && t_query <= t_next) {
      double alpha = (t_query - t_odo) / dt_odo;
      odo_vel += alpha * (dataset.odo(next, 1) - odo_vel);
    }
  }
  odo_speed_out = odo_vel;
  return true;
}

bool ComputeGnssVelocityMeasurementAtIndex(const Dataset &dataset,
                                          int gnss_idx,
                                          double t_curr,
                                          Vector3d &gnss_vel_ecef,
                                          Vector3d &gnss_vel_std_out) {
  if (!HasGnssVelocityData(dataset)) {
    return false;
  }
  if (gnss_idx < 0 || gnss_idx >= dataset.gnss.timestamps.size()) {
    return false;
  }

  constexpr double kSigmaVelMin = 1e-4;
  double sigma_vel_fallback = 0.5;
  double t_gnss = dataset.gnss.timestamps(gnss_idx);
  double dt_align = t_curr - t_gnss;

  gnss_vel_ecef = dataset.gnss.velocities.row(gnss_idx).transpose();
  gnss_vel_std_out = dataset.gnss.vel_std.row(gnss_idx).transpose();

  int next_gnss = gnss_idx + 1;
  if (dt_align > 1e-9 && next_gnss < dataset.gnss.timestamps.size()) {
    double t_next = dataset.gnss.timestamps(next_gnss);
    double dt_gnss = t_next - t_gnss;
    if (dt_gnss > 1e-6 && dt_align < dt_gnss) {
      double alpha = dt_align / dt_gnss;
      Vector3d vel_next = dataset.gnss.velocities.row(next_gnss).transpose();
      gnss_vel_ecef += alpha * (vel_next - gnss_vel_ecef);
    }
  }

  for (int k = 0; k < 3; ++k) {
    if (!std::isfinite(gnss_vel_std_out(k)) || gnss_vel_std_out(k) <= 0.0) {
      gnss_vel_std_out(k) = sigma_vel_fallback;
    }
    if (gnss_vel_std_out(k) < kSigmaVelMin) {
      gnss_vel_std_out(k) = kSigmaVelMin;
    }
  }
  return true;
}

bool ComputeGnssPositionMeasurementAtIndex(const Dataset &dataset,
                                          const NoiseParams &noise,
                                          int gnss_idx,
                                          double t_curr,
                                          Vector3d &gnss_pos_ecef,
                                          Vector3d &gnss_std_out) {
  return ComputeAlignedGnssPositionMeasurement(dataset, noise, gnss_idx,
                                               t_curr, gnss_pos_ecef,
                                               gnss_std_out);
}

vector<AuditSample> SelectGnssPositionSamples(const Dataset &dataset,
                                              const vector<SolRecord> &records,
                                              double tol,
                                              int max_samples,
                                              double min_separation_s) {
  vector<AuditSample> selected;
  double last_t_state = -numeric_limits<double>::infinity();
  for (int gnss_idx = 0; gnss_idx < dataset.gnss.timestamps.size(); ++gnss_idx) {
    double t_gnss = dataset.gnss.timestamps(gnss_idx);
    auto imu_it = lower_bound(dataset.imu.begin(), dataset.imu.end(), t_gnss - tol,
                              [](const ImuData &sample, double t) { return sample.t < t; });
    if (imu_it == dataset.imu.end()) {
      continue;
    }
    int imu_idx = static_cast<int>(distance(dataset.imu.begin(), imu_it));
    double t_state = dataset.imu[imu_idx].t;
    if (std::abs(t_state - last_t_state) < min_separation_s) {
      continue;
    }
    int record_idx = FindClosestRecordIndex(records, t_state);
    if (record_idx < 0) {
      continue;
    }
    const SolRecord &record = records[record_idx];
    AuditSample sample;
    sample.measurement = "GNSS_POS";
    sample.record_idx = record_idx;
    sample.meas_idx = gnss_idx;
    sample.t_meas = t_gnss;
    sample.t_state = t_state;
    sample.speed_h = record.speed_h;
    sample.yaw_rate_rad_s = record.yaw_rate_rad_s;
    sample.turn_score = record.turn_score;
    selected.push_back(sample);
    last_t_state = t_state;
    if (static_cast<int>(selected.size()) >= max_samples) {
      break;
    }
  }
  return selected;
}

vector<AuditSample> SelectGnssVelocitySamples(const Dataset &dataset,
                                              const vector<SolRecord> &records,
                                              double split_t,
                                              double tol,
                                              int max_samples,
                                              double min_separation_s) {
  vector<AuditSample> candidates;
  if (!HasGnssVelocityData(dataset)) {
    return candidates;
  }

  for (int gnss_idx = 0; gnss_idx < dataset.gnss.timestamps.size(); ++gnss_idx) {
    double t_gnss = dataset.gnss.timestamps(gnss_idx);
    if (t_gnss > split_t + tol) {
      break;
    }

    auto imu_it = lower_bound(dataset.imu.begin(), dataset.imu.end(), t_gnss - tol,
                              [](const ImuData &sample, double t) { return sample.t < t; });
    if (imu_it == dataset.imu.end()) {
      continue;
    }
    int imu_idx = static_cast<int>(distance(dataset.imu.begin(), imu_it));
    double t_state = dataset.imu[imu_idx].t;
    int record_idx = FindClosestRecordIndex(records, t_state);
    if (record_idx < 0) {
      continue;
    }

    const SolRecord &record = records[record_idx];
    if (!std::isfinite(record.turn_score)) {
      continue;
    }

    AuditSample sample;
    sample.measurement = "GNSS_VEL";
    sample.record_idx = record_idx;
    sample.meas_idx = gnss_idx;
    sample.t_meas = t_gnss;
    sample.t_state = t_state;
    sample.speed_h = record.speed_h;
    sample.yaw_rate_rad_s = record.yaw_rate_rad_s;
    sample.turn_score = record.turn_score;
    candidates.push_back(sample);
  }

  sort(candidates.begin(), candidates.end(), [](const AuditSample &a, const AuditSample &b) {
    return a.turn_score > b.turn_score;
  });

  vector<AuditSample> selected;
  for (const auto &candidate : candidates) {
    bool separated = true;
    for (const auto &chosen : selected) {
      if (std::abs(candidate.t_state - chosen.t_state) < min_separation_s) {
        separated = false;
        break;
      }
    }
    if (!separated) continue;
    selected.push_back(candidate);
    if (static_cast<int>(selected.size()) >= max_samples) break;
  }

  sort(selected.begin(), selected.end(), [](const AuditSample &a, const AuditSample &b) {
    return a.t_state < b.t_state;
  });
  return selected;
}


bool UseTrueInEkf(const FejManager *fej) {
  return fej != nullptr && fej->UseTrueInEkfMode();
}

bool UseHybridInEkf(const FejManager *fej) {
  return fej != nullptr && fej->enabled && !fej->true_iekf_mode;
}

State ApplyErrorState(const State &nominal,
                      const VectorXd &dx,
                      const FejManager *fej) {
  State out = nominal;
  Vector3d dr_ned = dx.segment<3>(StateIdx::kPos);
  Vector3d dv_ned = dx.segment<3>(StateIdx::kVel);
  Vector3d dphi_ned = dx.segment<3>(StateIdx::kAtt);

  Llh llh = EcefToLlh(nominal.p);
  Matrix3d R_ne = RotNedToEcef(llh);
  Matrix3d C_bn = R_ne.transpose() * QuatToRot(nominal.q);
  bool use_true_iekf = UseTrueInEkf(fej);
  bool use_hybrid_inekf = UseHybridInEkf(fej);

  if (use_true_iekf) {
    Vector3d rho_p_body = dr_ned;
    Vector3d rho_v_body = dv_ned;
    Vector3d phi_body = dphi_ned;

    dr_ned = C_bn * rho_p_body;
    dv_ned = C_bn * rho_v_body;

    out.p += R_ne * dr_ned;
    out.v += R_ne * dv_ned;
    Vector4d dq = QuatFromSmallAngle(phi_body);
    out.q = NormalizeQuat(QuatMultiply(out.q, dq));
  } else {
    if (use_hybrid_inekf) {
      Vector3d v_ned_nom = R_ne.transpose() * nominal.v;
      dv_ned -= Skew(v_ned_nom) * dphi_ned;
      if (fej->ri_inject_pos_inverse) {
        Vector3d p_ned_local = R_ne.transpose() * (nominal.p - fej->p_init_ecef);
        dr_ned -= Skew(p_ned_local) * dphi_ned;
      }
    }

    out.p += R_ne * dr_ned;
    out.v += R_ne * dv_ned;

    Vector3d dphi_ecef = R_ne * dphi_ned;
    if (use_hybrid_inekf) {
      Vector4d dq = QuatFromSmallAngle(dphi_ecef);
      out.q = NormalizeQuat(QuatMultiply(dq, out.q));
    } else {
      Vector4d dq = QuatFromSmallAngle(-dphi_ecef);
      out.q = NormalizeQuat(QuatMultiply(dq, out.q));
    }
  }

  out.ba += dx.segment<3>(StateIdx::kBa);
  out.bg += dx.segment<3>(StateIdx::kBg);
  out.sg += dx.segment<3>(StateIdx::kSg);
  out.sa += dx.segment<3>(StateIdx::kSa);
  out.odo_scale += dx(StateIdx::kOdoScale);
  out.mounting_roll += dx(StateIdx::kMountRoll);
  out.mounting_pitch += dx(StateIdx::kMountPitch);
  out.mounting_yaw += dx(StateIdx::kMountYaw);
  out.lever_arm += dx.segment<3>(StateIdx::kLever);
  out.gnss_lever_arm += dx.segment<3>(StateIdx::kGnssLever);
  return out;
}

VectorXd BuildFiniteDifferenceSteps() {
  VectorXd eps = VectorXd::Constant(kStateDim, 1e-6);
  eps.segment<3>(StateIdx::kPos).setConstant(1e-3);
  eps.segment<3>(StateIdx::kVel).setConstant(1e-4);
  eps.segment<3>(StateIdx::kAtt).setConstant(1e-7);
  eps.segment<3>(StateIdx::kBa).setConstant(1e-5);
  eps.segment<3>(StateIdx::kBg).setConstant(1e-6);
  eps.segment<3>(StateIdx::kSg).setConstant(1e-6);
  eps.segment<3>(StateIdx::kSa).setConstant(1e-6);
  eps(StateIdx::kOdoScale) = 1e-6;
  eps(StateIdx::kMountRoll) = 1e-7;
  eps(StateIdx::kMountPitch) = 1e-7;
  eps(StateIdx::kMountYaw) = 1e-7;
  eps.segment<3>(StateIdx::kLever).setConstant(1e-5);
  eps.segment<3>(StateIdx::kGnssLever).setConstant(1e-5);
  return eps;
}

template <typename ModelBuilder>
AuditModel EvaluateModel(const State &state, ModelBuilder &&builder) {
  return builder(state);
}

template <typename ModelBuilder>
MatrixXd ComputeNumericalJacobian(const State &nominal,
                                  const FejManager *fej,
                                  const VectorXd &eps,
                                  ModelBuilder &&builder) {
  AuditModel nominal_model = builder(nominal);
  MatrixXd H_num = MatrixXd::Zero(nominal_model.y.size(), kStateDim);

  for (int j = 0; j < kStateDim; ++j) {
    VectorXd dx = VectorXd::Zero(kStateDim);
    dx(j) = eps(j);
    State plus = ApplyErrorState(nominal, dx, fej);
    State minus = ApplyErrorState(nominal, -dx, fej);
    AuditModel model_plus = builder(plus);
    AuditModel model_minus = builder(minus);
    H_num.col(j) = -(model_plus.y - model_minus.y) / (2.0 * eps(j));
  }
  return H_num;
}

vector<BlockError> ComputeBlockErrors(const string &dataset,
                                      const string &measurement,
                                      double t,
                                      double speed_h,
                                      double yaw_rate_deg_s,
                                      const MatrixXd &H_analytic,
                                      const MatrixXd &H_numeric,
                                      const vector<BlockSpec> &blocks,
                                      const vector<string> &state_names) {
  vector<BlockError> out;
  for (const auto &block : blocks) {
    MatrixXd Ha = H_analytic.block(0, block.start, H_analytic.rows(), block.size);
    MatrixXd Hn = H_numeric.block(0, block.start, H_numeric.rows(), block.size);
    MatrixXd Hd = Ha - Hn;

    double analytic_norm = Ha.norm();
    double numeric_norm = Hn.norm();
    double diff_norm = Hd.norm();
    double denom = max({1e-12, analytic_norm, numeric_norm});
    double rel_fro = diff_norm / denom;

    Eigen::Index row_idx = 0;
    Eigen::Index col_idx = 0;
    double max_abs = Hd.cwiseAbs().maxCoeff(&row_idx, &col_idx);

    BlockError err;
    err.dataset = dataset;
    err.measurement = measurement;
    err.t = t;
    err.speed_h = speed_h;
    err.yaw_rate_deg_s = yaw_rate_deg_s;
    err.block_name = block.name;
    err.analytic_norm = analytic_norm;
    err.numeric_norm = numeric_norm;
    err.diff_norm = diff_norm;
    err.rel_fro = rel_fro;
    err.max_abs = max_abs;
    err.worst_row = static_cast<int>(row_idx);
    err.worst_col = block.start + static_cast<int>(col_idx);
    err.worst_state = state_names[err.worst_col];
    out.push_back(err);
  }
  return out;
}

vector<ColumnError> ComputeColumnErrors(const string &dataset,
                                        const string &measurement,
                                        double t,
                                        double speed_h,
                                        double yaw_rate_deg_s,
                                        const MatrixXd &H_analytic,
                                        const MatrixXd &H_numeric,
                                        const vector<string> &state_names) {
  vector<ColumnError> out;
  for (int j = 0; j < kStateDim; ++j) {
    VectorXd ha = H_analytic.col(j);
    VectorXd hn = H_numeric.col(j);
    VectorXd hd = ha - hn;
    double analytic_norm = ha.norm();
    double numeric_norm = hn.norm();
    double diff_norm = hd.norm();
    double denom = max({1e-12, analytic_norm, numeric_norm});
    double rel = diff_norm / denom;
    double max_abs = hd.cwiseAbs().maxCoeff();

    ColumnError err;
    err.dataset = dataset;
    err.measurement = measurement;
    err.t = t;
    err.speed_h = speed_h;
    err.yaw_rate_deg_s = yaw_rate_deg_s;
    err.col = j;
    err.state_name = state_names[j];
    err.analytic_norm = analytic_norm;
    err.numeric_norm = numeric_norm;
    err.diff_norm = diff_norm;
    err.rel = rel;
    err.max_abs = max_abs;
    out.push_back(err);
  }
  return out;
}

void SaveSelectedSamplesCsv(const fs::path &path,
                            const vector<AuditSummary> &summaries) {
  ofstream fout(path);
  fout << "dataset,config,measurement,t_meas,t_state,split_t,speed_h,yaw_rate_deg_s,turn_score,record_idx,meas_idx\n";
  for (const auto &summary : summaries) {
    for (const auto &sample : summary.samples) {
      fout << summary.dataset << ','
           << '"' << summary.config_path << '"' << ','
           << sample.measurement << ','
           << fixed << setprecision(6) << sample.t_meas << ','
           << sample.t_state << ','
           << summary.split_t << ','
           << sample.speed_h << ','
           << sample.yaw_rate_rad_s * kRadToDeg << ','
           << sample.turn_score << ','
           << sample.record_idx << ','
           << sample.meas_idx << '\n';
    }
  }
}

void SaveBlockErrorsCsv(const fs::path &path,
                        const vector<AuditSummary> &summaries) {
  ofstream fout(path);
  fout << "dataset,measurement,t,speed_h,yaw_rate_deg_s,block,analytic_norm,numeric_norm,diff_norm,rel_fro,max_abs,worst_row,worst_col,worst_state\n";
  for (const auto &summary : summaries) {
    for (const auto &err : summary.block_errors) {
      fout << err.dataset << ','
           << err.measurement << ','
           << fixed << setprecision(6) << err.t << ','
           << err.speed_h << ','
           << err.yaw_rate_deg_s << ','
           << err.block_name << ','
           << err.analytic_norm << ','
           << err.numeric_norm << ','
           << err.diff_norm << ','
           << err.rel_fro << ','
           << err.max_abs << ','
           << err.worst_row << ','
           << err.worst_col << ','
           << err.worst_state << '\n';
    }
  }
}

void SaveColumnErrorsCsv(const fs::path &path,
                         const vector<AuditSummary> &summaries) {
  ofstream fout(path);
  fout << "dataset,measurement,t,speed_h,yaw_rate_deg_s,col,state_name,analytic_norm,numeric_norm,diff_norm,rel,max_abs\n";
  for (const auto &summary : summaries) {
    for (const auto &err : summary.column_errors) {
      fout << err.dataset << ','
           << err.measurement << ','
           << fixed << setprecision(6) << err.t << ','
           << err.speed_h << ','
           << err.yaw_rate_deg_s << ','
           << err.col << ','
           << err.state_name << ','
           << err.analytic_norm << ','
           << err.numeric_norm << ','
           << err.diff_norm << ','
           << err.rel << ','
           << err.max_abs << '\n';
    }
  }
}

void SaveSplitCovarianceCsv(const fs::path &path,
                            const vector<AuditSummary> &summaries) {
  ofstream fout(path);
  fout << "dataset,config,tag,split_t,t_meas,t_state,"
          "P_attx_bgz,P_atty_bgz,P_attz_bgz,"
          "corr_attx_bgz,corr_atty_bgz,corr_attz_bgz,"
          "var_attx,var_atty,var_attz,var_bgz\n";
  for (const auto &summary : summaries) {
    if (!summary.split_cov_valid) continue;
    fout << summary.dataset << ','
         << '"' << summary.config_path << '"' << ','
         << summary.split_cov_tag << ','
         << fixed << setprecision(9)
         << summary.split_t << ','
         << summary.split_cov_t_meas << ','
         << summary.split_cov_t_state << ','
         << summary.split_cov_att_bgz.x() << ','
         << summary.split_cov_att_bgz.y() << ','
         << summary.split_cov_att_bgz.z() << ','
         << summary.split_corr_att_bgz.x() << ','
         << summary.split_corr_att_bgz.y() << ','
         << summary.split_corr_att_bgz.z() << ','
         << summary.split_att_var.x() << ','
         << summary.split_att_var.y() << ','
         << summary.split_att_var.z() << ','
         << summary.split_bgz_var << '\n';
  }
}

void SaveResetConsistencyCsv(const fs::path &path,
                             const vector<AuditSummary> &summaries) {
  ofstream fout(path);
  fout << "dataset,config,tag,split_t,t_meas,t_state,"
          "floor_applied,expected_norm,actual_norm,diff_norm,rel_fro,max_abs,"
          "worst_row,worst_col,p_tilde_path,p_expected_path,p_after_reset_path,dx_path\n";
  for (const auto &summary : summaries) {
    if (!summary.reset_consistency_valid) continue;
    fout << summary.dataset << ','
         << '"' << summary.config_path << '"' << ','
         << summary.reset_tag << ','
         << fixed << setprecision(9)
         << summary.split_t << ','
         << summary.reset_t_meas << ','
         << summary.reset_t_state << ','
         << (summary.reset_floor_applied ? 1 : 0) << ','
         << summary.reset_expected_norm << ','
         << summary.reset_actual_norm << ','
         << summary.reset_diff_norm << ','
         << summary.reset_rel_fro << ','
         << summary.reset_max_abs << ','
         << summary.reset_worst_row << ','
         << summary.reset_worst_col << ','
         << '"' << summary.reset_p_tilde_path << '"' << ','
         << '"' << summary.reset_p_expected_path << '"' << ','
         << '"' << summary.reset_p_after_reset_path << '"' << ','
         << '"' << summary.reset_dx_path << '"' << '\n';
  }
}

void SaveResetAuditMatrix(const fs::path &path, const MatrixXd &mat,
                          const string &header = "") {
  io::SaveMatrix(path.string(), mat, header);
}

map<pair<string, string>, BlockError> CollectWorstBlockPerMeasurement(
    const vector<AuditSummary> &summaries) {
  map<pair<string, string>, BlockError> worst;
  for (const auto &summary : summaries) {
    for (const auto &err : summary.block_errors) {
      auto key = make_pair(err.dataset, err.measurement + ":" + err.block_name);
      auto it = worst.find(key);
      if (it == worst.end() ||
          tie(err.rel_fro, err.max_abs) > tie(it->second.rel_fro, it->second.max_abs)) {
        worst[key] = err;
      }
    }
  }
  return worst;
}

vector<ColumnError> CollectWorstColumns(const vector<AuditSummary> &summaries,
                                        const string &dataset,
                                        const string &measurement,
                                        int top_k) {
  vector<ColumnError> cols;
  for (const auto &summary : summaries) {
    for (const auto &err : summary.column_errors) {
      if (err.dataset == dataset && err.measurement == measurement) {
        cols.push_back(err);
      }
    }
  }
  sort(cols.begin(), cols.end(), [](const ColumnError &a, const ColumnError &b) {
    return tie(a.rel, a.max_abs) > tie(b.rel, b.max_abs);
  });
  if (static_cast<int>(cols.size()) > top_k) {
    cols.resize(top_k);
  }
  return cols;
}

void SaveSummaryMarkdown(const fs::path &path,
                         const vector<AuditSummary> &summaries,
                         const VectorXd &eps) {
  ofstream fout(path);
  fout << "# Jacobian Audit Summary\n\n";
  fout << "- Target: `GNSS_POS/GNSS_VEL/ODO/NHC` numeric-Jacobian audit + optional GNSS split covariance + update-reset-covariance consistency\n";
  fout << "- Modes: audit the mode actually specified by each config (current run uses `true_iekf`)\n";
  fout << "- Finite difference: central difference on filter-consistent error injection\n";
  fout << "- Residual convention: compare analytic `H` against `-d(y)/d(dx)` because code stores `y = z - h`\n";
  fout << "- Step sizes: `pos=1e-3 m`, `vel=1e-4 m/s`, `att/mount=1e-7 rad`, `bg=1e-6 rad/s`, `sg/odo=1e-6`, `lever=1e-5 m`\n\n";

  for (const auto &summary : summaries) {
    fout << "## " << summary.dataset << "\n\n";
    fout << "- Config: `" << summary.config_path << "`\n";
    fout << "- Solution: `" << summary.sol_path << "`\n";
    fout << "- Solution mtime: `" << summary.sol_mtime << "`\n";
    fout << "- GNSS split_t: `" << ToStringPrec(summary.split_t, 6) << "`\n";
    if (summary.split_cov_valid) {
      fout << "- GNSS split covariance capture: tag=`" << summary.split_cov_tag
           << "`, t_meas=`" << ToStringPrec(summary.split_cov_t_meas, 6)
           << "`, t_state=`" << ToStringPrec(summary.split_cov_t_state, 6) << "`\n";
      fout << "- P[att,bg_z]: `" << ToStringPrec(summary.split_cov_att_bgz.x(), 9)
           << " " << ToStringPrec(summary.split_cov_att_bgz.y(), 9)
           << " " << ToStringPrec(summary.split_cov_att_bgz.z(), 9) << "`\n";
      fout << "- corr(att,bg_z): `" << ToStringPrec(summary.split_corr_att_bgz.x(), 6)
           << " " << ToStringPrec(summary.split_corr_att_bgz.y(), 6)
           << " " << ToStringPrec(summary.split_corr_att_bgz.z(), 6) << "`\n";
    }
    if (summary.reset_consistency_valid) {
      fout << "- Reset consistency: tag=`" << summary.reset_tag
           << "`, floor_after_reset=`"
           << (summary.reset_floor_applied ? "ON" : "OFF")
           << "`, rel_fro=`" << ToStringPrec(summary.reset_rel_fro, 12)
           << "`, max_abs=`" << ToStringPrec(summary.reset_max_abs, 12)
           << "`, worst=(row `" << summary.reset_worst_row
           << "`, col `" << summary.reset_worst_col << "`)\n";
      fout << "- Raw matrices: `" << summary.reset_p_tilde_path << "`, `"
           << summary.reset_p_expected_path << "`, `"
           << summary.reset_p_after_reset_path << "`, `"
           << summary.reset_dx_path << "`\n";
    }
    fout << "- Selected audit samples:\n";
    for (const auto &sample : summary.samples) {
      fout << "  - meas=`" << sample.measurement
           << "`, t_meas=`" << ToStringPrec(sample.t_meas, 6)
           << "`, t_state=`" << ToStringPrec(sample.t_state, 6)
           << "`, speed_h=`" << ToStringPrec(sample.speed_h, 3)
           << " m/s`, yaw_rate=`" << ToStringPrec(sample.yaw_rate_rad_s * kRadToDeg, 3)
           << " deg/s`, score=`" << ToStringPrec(sample.turn_score, 3) << "`\n";
    }
    fout << "\n";

    for (const string measurement : {string("GNSS_POS"), string("GNSS_VEL"), string("NHC"), string("ODO")}) {
      vector<BlockError> errs;
      for (const auto &err : summary.block_errors) {
        if (err.measurement == measurement) errs.push_back(err);
      }
      sort(errs.begin(), errs.end(), [](const BlockError &a, const BlockError &b) {
        return tie(a.rel_fro, a.max_abs) > tie(b.rel_fro, b.max_abs);
      });
      fout << "### " << measurement << " worst blocks\n\n";
      if (errs.empty()) {
        fout << "- none\n\n";
        continue;
      }
      int keep = min<int>(errs.size(), 6);
      for (int i = 0; i < keep; ++i) {
        const auto &err = errs[i];
        fout << "- block=`" << err.block_name
             << "`, t=`" << ToStringPrec(err.t, 6)
             << "`, rel_fro=`" << ToStringPrec(err.rel_fro, 6)
             << "`, max_abs=`" << ToStringPrec(err.max_abs, 6)
             << "`, worst_state=`" << err.worst_state << "`\n";
      }
      fout << "\n";
    }
  }

  fout << "## Cross-Dataset Worst Blocks\n\n";
  auto worst = CollectWorstBlockPerMeasurement(summaries);
  for (const auto &[key, err] : worst) {
    fout << "- `" << key.first << ' ' << key.second
         << "`: rel_fro=`" << ToStringPrec(err.rel_fro, 6)
         << "`, max_abs=`" << ToStringPrec(err.max_abs, 6)
         << "`, t=`" << ToStringPrec(err.t, 6)
         << "`, worst_state=`" << err.worst_state << "`\n";
  }
  fout << "\n";

  fout << "## Top Columns\n\n";
  for (const string dataset : {string("data2"), string("data4")}) {
    for (const string measurement : {string("GNSS_POS"), string("GNSS_VEL"), string("NHC"), string("ODO")}) {
      auto top_cols = CollectWorstColumns(summaries, dataset, measurement, 5);
      fout << "### " << dataset << ' ' << measurement << "\n\n";
      if (top_cols.empty()) {
        fout << "- none\n\n";
        continue;
      }
      for (const auto &col : top_cols) {
        fout << "- state=`" << col.state_name
             << "`, t=`" << ToStringPrec(col.t, 6)
             << "`, rel=`" << ToStringPrec(col.rel, 6)
             << "`, max_abs=`" << ToStringPrec(col.max_abs, 6) << "`\n";
      }
      fout << "\n";
    }
  }
}

AuditSummary RunAuditForConfig(const string &config_path,
                               const fs::path &outdir,
                               const VectorXd &eps,
                               const vector<string> &state_names) {
  FusionOptions options = LoadFusionOptions(config_path);
  Dataset dataset = LoadDataset(options);
  if (dataset.imu.size() < 3) {
    throw runtime_error("IMU data is insufficient for audit: " + config_path);
  }

  fs::path sol_path = options.output_path;
  if (!fs::exists(sol_path)) {
    throw runtime_error("solution file missing, run fusion first: " + sol_path.string());
  }

  AuditSummary summary;
  summary.dataset = DatasetTagFromPath(config_path + " " + options.output_path);
  summary.config_path = config_path;
  summary.sol_path = sol_path.string();
  summary.sol_mtime = FormatFileTime(fs::last_write_time(sol_path));
  summary.split_t = ComputeSplitTime(options, dataset);

  vector<SolRecord> records = LoadSolutionRecords(sol_path.string());
  vector<AuditSample> road_samples;
  if (options.constraints.enable_nhc || options.constraints.enable_odo) {
    road_samples = SelectTurningSamples(records, summary.split_t, 5, 20.0);
  }
  summary.samples = road_samples;

  FejManager fej;
  fej.Enable(options.fej.enable);
  fej.true_iekf_mode = options.fej.true_iekf_mode;
  fej.apply_covariance_floor_after_reset =
      options.fej.apply_covariance_floor_after_reset;
  fej.ri_gnss_pos_use_p_ned_local = options.fej.ri_gnss_pos_use_p_ned_local;
  fej.ri_vel_gyro_noise_mode = options.fej.ri_vel_gyro_noise_mode;
  fej.ri_inject_pos_inverse = options.fej.ri_inject_pos_inverse;
  fej.p_init_ecef = records.front().state.p;
  FejManager *fej_ptr = fej.enabled ? &fej : nullptr;

  Vector3d mounting_base_rpy = ComputeMountingBaseRpyRad(options);

  for (const auto &sample : road_samples) {
    const SolRecord &record = records.at(sample.record_idx);
    int imu_idx = FindClosestImuIndex(dataset.imu, sample.t_state);
    if (imu_idx < 0) {
      throw runtime_error("failed to match IMU sample for t=" + ToStringPrec(sample.t_state, 6));
    }
    const ImuData &imu = dataset.imu[imu_idx];
    Vector3d omega_ib_b_raw = Vector3d::Zero();
    if (imu.dt > 1e-9) {
      omega_ib_b_raw = imu.dtheta / imu.dt;
    }

    if (options.constraints.enable_nhc) {
      auto nhc_builder = [&](const State &state) -> AuditModel {
        Matrix3d C_b_v = BuildVehicleRotation(mounting_base_rpy, state);
        auto model = MeasModels::ComputeNhcModel(state, C_b_v, omega_ib_b_raw,
                                                 options.constraints.sigma_nhc_y,
                                                 options.constraints.sigma_nhc_z,
                                                 fej_ptr);
        return {model.y, model.H};
      };
      AuditModel nhc_analytic = EvaluateModel(record.state, nhc_builder);
      MatrixXd nhc_numeric = ComputeNumericalJacobian(record.state, fej_ptr, eps, nhc_builder);
      auto nhc_block_errors = ComputeBlockErrors(summary.dataset, "NHC", sample.t_state,
                                                 sample.speed_h,
                                                 sample.yaw_rate_rad_s * kRadToDeg,
                                                 nhc_analytic.H, nhc_numeric,
                                                 BuildFocusBlocks("NHC"),
                                                 state_names);
      auto nhc_column_errors = ComputeColumnErrors(summary.dataset, "NHC", sample.t_state,
                                                   sample.speed_h,
                                                   sample.yaw_rate_rad_s * kRadToDeg,
                                                   nhc_analytic.H, nhc_numeric,
                                                   state_names);
      summary.block_errors.insert(summary.block_errors.end(),
                                  nhc_block_errors.begin(), nhc_block_errors.end());
      summary.column_errors.insert(summary.column_errors.end(),
                                   nhc_column_errors.begin(), nhc_column_errors.end());
    }

    if (options.constraints.enable_odo) {
      double odo_speed = 0.0;
      if (!ComputeOdoMeasurementAtTime(dataset, options.constraints, options.gating,
                                       sample.t_state, odo_speed)) {
        throw runtime_error("failed to interpolate ODO measurement for t=" + ToStringPrec(sample.t_state, 6));
      }
      auto odo_builder = [&](const State &state) -> AuditModel {
        Matrix3d C_b_v = BuildVehicleRotation(mounting_base_rpy, state);
        auto model = MeasModels::ComputeOdoModel(state, odo_speed, C_b_v,
                                                 omega_ib_b_raw,
                                                 options.constraints.sigma_odo,
                                                 fej_ptr);
        return {model.y, model.H};
      };
      AuditModel odo_analytic = EvaluateModel(record.state, odo_builder);
      MatrixXd odo_numeric = ComputeNumericalJacobian(record.state, fej_ptr, eps, odo_builder);
      auto odo_block_errors = ComputeBlockErrors(summary.dataset, "ODO", sample.t_state,
                                                 sample.speed_h,
                                                 sample.yaw_rate_rad_s * kRadToDeg,
                                                 odo_analytic.H, odo_numeric,
                                                 BuildFocusBlocks("ODO"),
                                                 state_names);
      auto odo_column_errors = ComputeColumnErrors(summary.dataset, "ODO", sample.t_state,
                                                   sample.speed_h,
                                                   sample.yaw_rate_rad_s * kRadToDeg,
                                                   odo_analytic.H, odo_numeric,
                                                   state_names);
      summary.block_errors.insert(summary.block_errors.end(),
                                  odo_block_errors.begin(), odo_block_errors.end());
      summary.column_errors.insert(summary.column_errors.end(),
                                   odo_column_errors.begin(), odo_column_errors.end());
    }
  }

  vector<AuditSample> gnss_pos_samples = SelectGnssPositionSamples(
      dataset, records, options.gating.time_tolerance, 5, 20.0);
  summary.samples.insert(summary.samples.end(), gnss_pos_samples.begin(), gnss_pos_samples.end());

  for (const auto &sample : gnss_pos_samples) {
    const SolRecord &record = records.at(sample.record_idx);
    Vector3d gnss_pos_ecef = Vector3d::Zero();
    Vector3d gnss_pos_std = Vector3d::Zero();
    if (!ComputeGnssPositionMeasurementAtIndex(dataset, options.noise, sample.meas_idx,
                                               sample.t_state, gnss_pos_ecef,
                                               gnss_pos_std)) {
      throw runtime_error("failed to reconstruct GNSS_POS measurement for idx=" +
                          to_string(sample.meas_idx));
    }

    auto gnss_pos_builder = [&](const State &state) -> AuditModel {
      auto model = MeasModels::ComputeGnssPositionModel(state, gnss_pos_ecef,
                                                        gnss_pos_std, fej_ptr);
      return {model.y, model.H};
    };

    AuditModel gnss_pos_analytic = EvaluateModel(record.state, gnss_pos_builder);
    MatrixXd gnss_pos_numeric = ComputeNumericalJacobian(record.state, fej_ptr, eps,
                                                         gnss_pos_builder);
    auto gnss_pos_block_errors = ComputeBlockErrors(summary.dataset, "GNSS_POS",
                                                    sample.t_state, sample.speed_h,
                                                    sample.yaw_rate_rad_s * kRadToDeg,
                                                    gnss_pos_analytic.H,
                                                    gnss_pos_numeric,
                                                    BuildFocusBlocks("GNSS_POS"),
                                                    state_names);
    auto gnss_pos_column_errors = ComputeColumnErrors(summary.dataset, "GNSS_POS",
                                                      sample.t_state, sample.speed_h,
                                                      sample.yaw_rate_rad_s * kRadToDeg,
                                                      gnss_pos_analytic.H,
                                                      gnss_pos_numeric,
                                                      state_names);
    summary.block_errors.insert(summary.block_errors.end(),
                                gnss_pos_block_errors.begin(),
                                gnss_pos_block_errors.end());
    summary.column_errors.insert(summary.column_errors.end(),
                                 gnss_pos_column_errors.begin(),
                                 gnss_pos_column_errors.end());
  }

  vector<AuditSample> gnss_vel_samples = SelectGnssVelocitySamples(
      dataset, records, summary.split_t, options.gating.time_tolerance, 5, 20.0);
  summary.samples.insert(summary.samples.end(), gnss_vel_samples.begin(), gnss_vel_samples.end());

  for (const auto &sample : gnss_vel_samples) {
    const SolRecord &record = records.at(sample.record_idx);
    int imu_idx = FindClosestImuIndex(dataset.imu, sample.t_state);
    if (imu_idx < 0) {
      throw runtime_error("failed to match IMU sample for GNSS_VEL t=" + ToStringPrec(sample.t_state, 6));
    }
    const ImuData &imu = dataset.imu[imu_idx];
    Vector3d omega_ib_b_raw = Vector3d::Zero();
    if (imu.dt > 1e-9) {
      omega_ib_b_raw = imu.dtheta / imu.dt;
    }

    Vector3d gnss_vel_ecef = Vector3d::Zero();
    Vector3d gnss_vel_std = Vector3d::Zero();
    if (!ComputeGnssVelocityMeasurementAtIndex(dataset, sample.meas_idx, sample.t_state,
                                               gnss_vel_ecef, gnss_vel_std)) {
      throw runtime_error("failed to reconstruct GNSS_VEL measurement for idx=" + to_string(sample.meas_idx));
    }

    auto gnss_vel_builder = [&](const State &state) -> AuditModel {
      auto model = MeasModels::ComputeGnssVelocityModel(state, gnss_vel_ecef,
                                                        omega_ib_b_raw,
                                                        gnss_vel_std,
                                                        fej_ptr);
      return {model.y, model.H};
    };

    AuditModel gnss_vel_analytic = EvaluateModel(record.state, gnss_vel_builder);
    MatrixXd gnss_vel_numeric = ComputeNumericalJacobian(record.state, fej_ptr, eps, gnss_vel_builder);
    auto gnss_vel_block_errors = ComputeBlockErrors(summary.dataset, "GNSS_VEL", sample.t_state,
                                                    sample.speed_h,
                                                    sample.yaw_rate_rad_s * kRadToDeg,
                                                    gnss_vel_analytic.H, gnss_vel_numeric,
                                                    BuildFocusBlocks("GNSS_VEL"),
                                                    state_names);
    auto gnss_vel_column_errors = ComputeColumnErrors(summary.dataset, "GNSS_VEL", sample.t_state,
                                                      sample.speed_h,
                                                      sample.yaw_rate_rad_s * kRadToDeg,
                                                      gnss_vel_analytic.H, gnss_vel_numeric,
                                                      state_names);
    summary.block_errors.insert(summary.block_errors.end(),
                                gnss_vel_block_errors.begin(), gnss_vel_block_errors.end());
    summary.column_errors.insert(summary.column_errors.end(),
                                 gnss_vel_column_errors.begin(), gnss_vel_column_errors.end());
  }

  State x0;
  Matrix<double, kStateDim, kStateDim> P0;
  if (!InitializeState(options, dataset.imu, dataset.truth, x0, P0)) {
    throw runtime_error("failed to initialize state for reset-consistency replay: " +
                        config_path);
  }

  if (options.gnss_schedule.enabled) {
    FusionDebugCapture debug_capture;
    debug_capture.capture_last_gnss_before_split = true;
    RunFusion(options, dataset, x0, P0, &debug_capture);

    if (!debug_capture.gnss_split_cov.valid) {
      throw runtime_error("failed to capture GNSS split covariance for " + config_path);
    }
    summary.split_cov_valid = true;
    summary.split_cov_tag = debug_capture.gnss_split_cov.tag;
    summary.split_cov_t_meas = debug_capture.gnss_split_cov.t_meas;
    summary.split_cov_t_state = debug_capture.gnss_split_cov.t_state;
    summary.split_cov_att_bgz = debug_capture.gnss_split_cov.P_att_bgz;
    summary.split_corr_att_bgz = debug_capture.gnss_split_cov.corr_att_bgz;
    summary.split_att_var = debug_capture.gnss_split_cov.att_var;
    summary.split_bgz_var = debug_capture.gnss_split_cov.bgz_var;

    if (!debug_capture.reset_consistency.valid) {
      throw runtime_error("failed to capture reset consistency snapshot for " + config_path);
    }
    summary.reset_consistency_valid = true;
    summary.reset_tag = debug_capture.reset_consistency.tag;
    summary.reset_t_meas = debug_capture.reset_consistency.t_meas;
    summary.reset_t_state = debug_capture.reset_consistency.t_state;
    summary.reset_floor_applied =
        debug_capture.reset_consistency.covariance_floor_applied;

    Matrix<double, kStateDim, 1> dx_reset = debug_capture.reset_consistency.dx;
    Matrix<double, kStateDim, kStateDim> P_tilde =
        debug_capture.reset_consistency.P_tilde;
    Matrix<double, kStateDim, kStateDim> P_after_reset =
        debug_capture.reset_consistency.P_after_reset;
    Matrix<double, kStateDim, kStateDim> Gamma =
        BuildTrueInEkfResetGammaFromDx(dx_reset);
    Matrix<double, kStateDim, kStateDim> P_expected =
        Gamma * P_tilde * Gamma.transpose();
    P_expected = 0.5 * (P_expected + P_expected.transpose());

    Matrix<double, kStateDim, kStateDim> diff = P_after_reset - P_expected;
    summary.reset_expected_norm = P_expected.norm();
    summary.reset_actual_norm = P_after_reset.norm();
    summary.reset_diff_norm = diff.norm();
    double denom = max({1e-12, summary.reset_expected_norm, summary.reset_actual_norm});
    summary.reset_rel_fro = summary.reset_diff_norm / denom;
    Eigen::Index worst_row = -1;
    Eigen::Index worst_col = -1;
    summary.reset_max_abs = diff.cwiseAbs().maxCoeff(&worst_row, &worst_col);
    summary.reset_worst_row = static_cast<int>(worst_row);
    summary.reset_worst_col = static_cast<int>(worst_col);

    fs::path p_tilde_path = outdir / (summary.dataset + "_reset_p_tilde.txt");
    fs::path p_expected_path = outdir / (summary.dataset + "_reset_p_expected.txt");
    fs::path p_after_reset_path = outdir / (summary.dataset + "_reset_p_after_reset.txt");
    fs::path dx_path = outdir / (summary.dataset + "_reset_dx.txt");
    SaveResetAuditMatrix(p_tilde_path, P_tilde, "P_tilde");
    SaveResetAuditMatrix(p_expected_path, P_expected, "P_expected");
    SaveResetAuditMatrix(p_after_reset_path, P_after_reset, "P_after_reset");
    SaveResetAuditMatrix(dx_path, dx_reset, "dx_reset");
    summary.reset_p_tilde_path = p_tilde_path.string();
    summary.reset_p_expected_path = p_expected_path.string();
    summary.reset_p_after_reset_path = p_after_reset_path.string();
    summary.reset_dx_path = dx_path.string();

    cout << "[AuditSplitCov] dataset=" << summary.dataset
         << " t_meas=" << summary.split_cov_t_meas
         << " P_att_bgz=" << summary.split_cov_att_bgz.transpose()
         << " corr_att_bgz=" << summary.split_corr_att_bgz.transpose() << "\n";
    cout << "[AuditReset] dataset=" << summary.dataset
         << " t_meas=" << summary.reset_t_meas
         << " rel_fro=" << summary.reset_rel_fro
         << " max_abs=" << summary.reset_max_abs
         << " floor_after_reset="
         << (summary.reset_floor_applied ? "ON" : "OFF") << "\n";
  }

  return summary;
}

vector<string> ParseConfigPaths(int argc, char **argv) {
  vector<string> configs;
  for (int i = 1; i < argc; ++i) {
    string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      configs.push_back(argv[++i]);
    }
  }
  if (configs.empty()) {
    configs = {
        "config_data2_gnss30_true_iekf.yaml",
        "config_data4_gnss30_true_iekf.yaml",
    };
  }
  return configs;
}

fs::path ParseOutputDir(int argc, char **argv) {
  for (int i = 1; i < argc; ++i) {
    string arg = argv[i];
    if (arg == "--outdir" && i + 1 < argc) {
      return fs::path(argv[++i]);
    }
  }
  return fs::path("output/review/20260307-update-reset-consistency-r1");
}

}  // namespace

int main(int argc, char **argv) {
  try {
    vector<string> config_paths = ParseConfigPaths(argc, argv);
    fs::path outdir = ParseOutputDir(argc, argv);
    fs::create_directories(outdir);

    VectorXd eps = BuildFiniteDifferenceSteps();
    vector<string> state_names = BuildStateNames();
    vector<AuditSummary> summaries;
    summaries.reserve(config_paths.size());

    for (const auto &config_path : config_paths) {
      cout << "[Audit] Running config: " << config_path << "\n";
      summaries.push_back(RunAuditForConfig(config_path, outdir, eps, state_names));
    }

    SaveSelectedSamplesCsv(outdir / "selected_samples.csv", summaries);
    SaveBlockErrorsCsv(outdir / "block_errors.csv", summaries);
    SaveColumnErrorsCsv(outdir / "column_errors.csv", summaries);
    SaveSplitCovarianceCsv(outdir / "split_covariance.csv", summaries);
    SaveResetConsistencyCsv(outdir / "reset_consistency.csv", summaries);
    SaveSummaryMarkdown(outdir / "summary.md", summaries, eps);

    cout << "[Audit] Done. Artifacts:\n";
    cout << "  - " << (outdir / "summary.md").string() << "\n";
    cout << "  - " << (outdir / "selected_samples.csv").string() << "\n";
    cout << "  - " << (outdir / "block_errors.csv").string() << "\n";
    cout << "  - " << (outdir / "column_errors.csv").string() << "\n";
    cout << "  - " << (outdir / "split_covariance.csv").string() << "\n";
    cout << "  - " << (outdir / "reset_consistency.csv").string() << "\n";
    return 0;
  } catch (const exception &e) {
    cerr << "jacobian_audit failed: " << e.what() << "\n";
    return 1;
  }
}
