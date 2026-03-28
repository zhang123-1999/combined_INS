#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
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
  double speed_h = 0.0;
  double yaw_rate_rad_s = 0.0;
};

struct CaseMetaRow {
  string case_id;
  string config_path;
  string sol_path;
  string sol_mtime;
};

struct AuditEntryRow {
  string case_id;
  string config_path;
  string measurement;
  string sample_label;
  double target_t = 0.0;
  double matched_t = 0.0;
  double dt_target = 0.0;
  double imu_t = 0.0;
  double dt_imu = 0.0;
  int row_idx = 0;
  int state_idx = 0;
  string state_name;
  double analytic = 0.0;
  double numeric = 0.0;
  double abs_diff = 0.0;
  double rel_diff = 0.0;
  double eps = 0.0;
  double speed_h = 0.0;
  double yaw_rate_deg_s = 0.0;
  double odo_speed = numeric_limits<double>::quiet_NaN();
};

struct SampleSummaryRow {
  string case_id;
  string config_path;
  string measurement;
  string sample_label;
  double target_t = 0.0;
  double matched_t = 0.0;
  double dt_target = 0.0;
  double imu_t = 0.0;
  double dt_imu = 0.0;
  double speed_h = 0.0;
  double yaw_rate_deg_s = 0.0;
  double odo_speed = numeric_limits<double>::quiet_NaN();
  double y_norm = 0.0;
  double analytic_norm = 0.0;
  double numeric_norm = 0.0;
  double diff_norm = 0.0;
  double rel_fro = 0.0;
  double max_abs = 0.0;
  int worst_row = -1;
  int worst_col = -1;
  string worst_state;
};

struct OdoComponentRow {
  string case_id;
  string config_path;
  string sample_label;
  double target_t = 0.0;
  double matched_t = 0.0;
  double dt_target = 0.0;
  double imu_t = 0.0;
  double dt_imu = 0.0;
  double speed_h = 0.0;
  double yaw_rate_deg_s = 0.0;
  double odo_speed = 0.0;
  double pred_reading = 0.0;
  double odo_scale = 0.0;
  double c_b_v_row0_x = 0.0;
  double c_b_v_row0_y = 0.0;
  double c_b_v_row0_z = 0.0;
  double lever_x = 0.0;
  double lever_y = 0.0;
  double lever_z = 0.0;
  double bg_x = 0.0;
  double bg_y = 0.0;
  double bg_z = 0.0;
  double sg_x = 0.0;
  double sg_y = 0.0;
  double sg_z = 0.0;
  double sf_g_x = 0.0;
  double sf_g_y = 0.0;
  double sf_g_z = 0.0;
  double omega_ib_unbiased_x = 0.0;
  double omega_ib_unbiased_y = 0.0;
  double omega_ib_unbiased_z = 0.0;
  double omega_in_b_x = 0.0;
  double omega_in_b_y = 0.0;
  double omega_in_b_z = 0.0;
  double omega_nb_b_x = 0.0;
  double omega_nb_b_y = 0.0;
  double omega_nb_b_z = 0.0;
  double v_b_x = 0.0;
  double v_b_y = 0.0;
  double v_b_z = 0.0;
  double v_wheel_b_x = 0.0;
  double v_wheel_b_y = 0.0;
  double v_wheel_b_z = 0.0;
  double v_phys_x = 0.0;
  double v_phys_y = 0.0;
  double v_phys_z = 0.0;
  double h_v_x = 0.0;
  double h_v_y = 0.0;
  double h_v_z = 0.0;
  double h_att_base_x = 0.0;
  double h_att_base_y = 0.0;
  double h_att_base_z = 0.0;
  double h_att_lever_x = 0.0;
  double h_att_lever_y = 0.0;
  double h_att_lever_z = 0.0;
  double h_att_total_x = 0.0;
  double h_att_total_y = 0.0;
  double h_att_total_z = 0.0;
  double h_bg_x = 0.0;
  double h_bg_y = 0.0;
  double h_bg_z = 0.0;
  double h_sg_x = 0.0;
  double h_sg_y = 0.0;
  double h_sg_z = 0.0;
  double h_lever_x = 0.0;
  double h_lever_y = 0.0;
  double h_lever_z = 0.0;
  double h_mount_pitch = 0.0;
  double h_mount_yaw = 0.0;
};

struct ParsedArgs {
  vector<string> config_paths;
  vector<double> timestamps;
  fs::path outdir;
};

struct AuditBundle {
  CaseMetaRow meta;
  vector<AuditEntryRow> entries;
  vector<SampleSummaryRow> summaries;
  vector<OdoComponentRow> odo_components;
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
  localtime_r(&tm_local, &tt);
#endif
  ostringstream oss;
  oss << put_time(&tm_local, "%Y-%m-%d %H:%M:%S");
  return oss.str();
}

string CsvEscape(const string &value) {
  string out = "\"";
  for (char c : value) {
    if (c == '"') {
      out += "\"\"";
    } else {
      out.push_back(c);
    }
  }
  out.push_back('"');
  return out;
}

double SafeRelativeScalar(double analytic, double numeric) {
  double denom = max({1e-12, std::abs(analytic), std::abs(numeric)});
  return std::abs(analytic - numeric) / denom;
}

double SafeRelativeNorm(const MatrixXd &analytic, const MatrixXd &numeric) {
  double denom = max({1e-12, analytic.norm(), numeric.norm()});
  return (analytic - numeric).norm() / denom;
}

string CaseIdFromConfigPath(const string &config_path) {
  string stem = fs::path(config_path).stem().string();
  const string prefix = "config_";
  if (stem.rfind(prefix, 0) == 0) {
    return stem.substr(prefix.size());
  }
  return stem;
}

string MakeSampleLabel(double target_t) {
  string label = "t" + ToStringPrec(target_t, 6);
  for (char &c : label) {
    if (c == '.') {
      c = '_';
    }
  }
  return label;
}

vector<string> DefaultConfigPaths() {
  return {
      "output/data2_bgz_route_decomposition_r1_20260325/artifacts/cases/"
      "control_strict_gate_freeze_gnss_lever/"
      "config_control_strict_gate_freeze_gnss_lever.yaml",
      "output/data2_bgz_route_decomposition_r1_20260325/artifacts/cases/"
      "fix_odo_lever_truth/config_fix_odo_lever_truth.yaml",
      "output/data2_bgz_route_decomposition_r1_20260325/artifacts/cases/"
      "fix_mounting_and_odo_lever_truth/"
      "config_fix_mounting_and_odo_lever_truth.yaml",
  };
}

vector<double> DefaultTimestamps() {
  return {528276.004906, 528314.000008, 528315.655012, 528867.106000};
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
    double lat = 0.0;
    double lon = 0.0;
    EcefToLatLon(records[i].state.p, lat, lon);
    Vector3d v_ned = EcefVelToNed(records[i].state.v, lat, lon);
    records[i].speed_h = v_ned.head<2>().norm();
    yaw_raw[i] = row(9) * kDegToRad;
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
  }
  return records;
}

int FindClosestImuIndex(const vector<ImuData> &imu, double t_query) {
  if (imu.empty()) return -1;
  auto it = lower_bound(
      imu.begin(), imu.end(), t_query,
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
  auto it = lower_bound(
      records.begin(), records.end(), t_query,
      [](const SolRecord &record, double t) { return record.t < t; });
  if (it == records.begin()) return 0;
  if (it == records.end()) return static_cast<int>(records.size()) - 1;
  int idx_hi = static_cast<int>(distance(records.begin(), it));
  int idx_lo = idx_hi - 1;
  if (std::abs(records[idx_hi].t - t_query) <
      std::abs(records[idx_lo].t - t_query)) {
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
MatrixXd ComputeNumericalJacobian(const State &nominal,
                                  const FejManager *fej,
                                  const VectorXd &eps,
                                  const VectorXd &nominal_y,
                                  ModelBuilder &&builder) {
  MatrixXd H_num = MatrixXd::Zero(nominal_y.size(), kStateDim);
  for (int j = 0; j < kStateDim; ++j) {
    VectorXd dx = VectorXd::Zero(kStateDim);
    dx(j) = eps(j);
    State plus = ApplyErrorState(nominal, dx, fej);
    State minus = ApplyErrorState(nominal, -dx, fej);
    VectorXd y_plus = builder(plus);
    VectorXd y_minus = builder(minus);
    H_num.col(j) = -(y_plus - y_minus) / (2.0 * eps(j));
  }
  return H_num;
}

ParsedArgs ParseArgs(int argc, char **argv) {
  ParsedArgs args;
  args.config_paths = DefaultConfigPaths();
  args.timestamps = DefaultTimestamps();
  args.outdir = "output/data2_bgz_jacobian_fd_audit_r1_20260325";

  bool custom_config = false;
  bool custom_timestamps = false;

  for (int i = 1; i < argc; ++i) {
    string arg = argv[i];
    if (arg == "--config") {
      if (i + 1 >= argc) {
        throw runtime_error("--config requires a path");
      }
      if (!custom_config) {
        args.config_paths.clear();
        custom_config = true;
      }
      args.config_paths.push_back(argv[++i]);
      continue;
    }
    if (arg == "--timestamp") {
      if (i + 1 >= argc) {
        throw runtime_error("--timestamp requires a numeric value");
      }
      if (!custom_timestamps) {
        args.timestamps.clear();
        custom_timestamps = true;
      }
      args.timestamps.push_back(stod(argv[++i]));
      continue;
    }
    if (arg == "--outdir") {
      if (i + 1 >= argc) {
        throw runtime_error("--outdir requires a path");
      }
      args.outdir = argv[++i];
      continue;
    }
    throw runtime_error("unknown argument: " + arg);
  }

  if (args.config_paths.empty()) {
    throw runtime_error("no configs provided");
  }
  if (args.timestamps.empty()) {
    throw runtime_error("no timestamps provided");
  }
  return args;
}

FejManager BuildFejManager(const FusionOptions &options,
                           const State &reference_state) {
  FejManager fej;
  fej.Enable(options.fej.enable);
  fej.true_iekf_mode = options.fej.true_iekf_mode;
  fej.apply_covariance_floor_after_reset =
      options.fej.apply_covariance_floor_after_reset;
  fej.ri_gnss_pos_use_p_ned_local = options.fej.ri_gnss_pos_use_p_ned_local;
  fej.ri_vel_gyro_noise_mode = options.fej.ri_vel_gyro_noise_mode;
  fej.ri_inject_pos_inverse = options.fej.ri_inject_pos_inverse;
  fej.debug_force_process_model = options.fej.debug_force_process_model;
  fej.debug_force_vel_jacobian = options.fej.debug_force_vel_jacobian;
  fej.debug_disable_true_reset_gamma = options.fej.debug_disable_true_reset_gamma;
  fej.debug_enable_standard_reset_gamma =
      options.fej.debug_enable_standard_reset_gamma;
  fej.p_init_ecef = reference_state.p;
  return fej;
}

SampleSummaryRow BuildSampleSummary(const CaseMetaRow &meta,
                                    const string &measurement,
                                    const string &sample_label,
                                    double target_t,
                                    const SolRecord &record,
                                    double imu_t,
                                    const VectorXd &y,
                                    const MatrixXd &analytic,
                                    const MatrixXd &numeric,
                                    double odo_speed) {
  SampleSummaryRow row;
  row.case_id = meta.case_id;
  row.config_path = meta.config_path;
  row.measurement = measurement;
  row.sample_label = sample_label;
  row.target_t = target_t;
  row.matched_t = record.t;
  row.dt_target = record.t - target_t;
  row.imu_t = imu_t;
  row.dt_imu = imu_t - record.t;
  row.speed_h = record.speed_h;
  row.yaw_rate_deg_s = record.yaw_rate_rad_s * kRadToDeg;
  row.odo_speed = odo_speed;
  row.y_norm = y.norm();
  row.analytic_norm = analytic.norm();
  row.numeric_norm = numeric.norm();
  MatrixXd diff = analytic - numeric;
  row.diff_norm = diff.norm();
  row.rel_fro = SafeRelativeNorm(analytic, numeric);
  Eigen::Index worst_row = -1;
  Eigen::Index worst_col = -1;
  row.max_abs = diff.cwiseAbs().maxCoeff(&worst_row, &worst_col);
  row.worst_row = static_cast<int>(worst_row);
  row.worst_col = static_cast<int>(worst_col);
  auto state_names = BuildStateNames();
  if (row.worst_col >= 0 && row.worst_col < kStateDim) {
    row.worst_state = state_names[row.worst_col];
  }
  return row;
}

void AppendAuditEntries(const CaseMetaRow &meta,
                        const string &measurement,
                        const string &sample_label,
                        double target_t,
                        const SolRecord &record,
                        double imu_t,
                        const MatrixXd &analytic,
                        const MatrixXd &numeric,
                        const VectorXd &eps,
                        const vector<string> &state_names,
                        double odo_speed,
                        vector<AuditEntryRow> &rows) {
  for (int r = 0; r < analytic.rows(); ++r) {
    for (int c = 0; c < analytic.cols(); ++c) {
      AuditEntryRow row;
      row.case_id = meta.case_id;
      row.config_path = meta.config_path;
      row.measurement = measurement;
      row.sample_label = sample_label;
      row.target_t = target_t;
      row.matched_t = record.t;
      row.dt_target = record.t - target_t;
      row.imu_t = imu_t;
      row.dt_imu = imu_t - record.t;
      row.row_idx = r;
      row.state_idx = c;
      row.state_name = state_names.at(c);
      row.analytic = analytic(r, c);
      row.numeric = numeric(r, c);
      row.abs_diff = std::abs(row.analytic - row.numeric);
      row.rel_diff = SafeRelativeScalar(row.analytic, row.numeric);
      row.eps = eps(c);
      row.speed_h = record.speed_h;
      row.yaw_rate_deg_s = record.yaw_rate_rad_s * kRadToDeg;
      row.odo_speed = odo_speed;
      rows.push_back(row);
    }
  }
}

OdoComponentRow BuildOdoComponentRow(const CaseMetaRow &meta,
                                     const string &sample_label,
                                     double target_t,
                                     const SolRecord &record,
                                     double imu_t,
                                     double odo_speed,
                                     const State &state,
                                     const Matrix3d &C_b_v,
                                     const Vector3d &omega_ib_b_raw,
                                     const FejManager *fej) {
  OdoComponentRow row;
  row.case_id = meta.case_id;
  row.config_path = meta.config_path;
  row.sample_label = sample_label;
  row.target_t = target_t;
  row.matched_t = record.t;
  row.dt_target = record.t - target_t;
  row.imu_t = imu_t;
  row.dt_imu = imu_t - record.t;
  row.speed_h = record.speed_h;
  row.yaw_rate_deg_s = record.yaw_rate_rad_s * kRadToDeg;
  row.odo_speed = odo_speed;
  row.odo_scale = state.odo_scale;

  Llh llh = EcefToLlh(state.p);
  Matrix3d R_ne = RotNedToEcef(llh);
  Matrix3d C_bn = R_ne.transpose() * QuatToRot(state.q);
  Vector3d v_ned = R_ne.transpose() * state.v;
  Vector3d v_b = C_bn.transpose() * v_ned;
  Vector3d omega_ie_n = OmegaIeNed(llh.lat);
  Vector3d omega_en_n = OmegaEnNed(v_ned, llh.lat, llh.h);
  Vector3d omega_in_n = omega_ie_n + omega_en_n;
  Vector3d omega_in_b = C_bn.transpose() * omega_in_n;
  Vector3d omega_ib_unbiased = omega_ib_b_raw - state.bg;
  Vector3d sf_g = Vector3d::Ones() - state.sg;
  Vector3d omega_ib_corr = sf_g.cwiseProduct(omega_ib_unbiased);
  Vector3d omega_nb_b = omega_ib_corr - omega_in_b;
  Vector3d v_wheel_b = v_b + omega_nb_b.cross(state.lever_arm);
  Vector3d v_phys_v = C_b_v * v_wheel_b;

  bool use_true_iekf = UseTrueInEkf(fej);
  bool use_hybrid_inekf = UseHybridInEkf(fej);
  bool hybrid_zero = use_hybrid_inekf;

  RowVector3d H_v_phys = RowVector3d::Zero();
  if (use_true_iekf) {
    H_v_phys = C_b_v.row(0);
  } else {
    H_v_phys = (C_b_v * C_bn.transpose()).row(0);
  }

  Matrix3d H_theta_base = Matrix3d::Zero();
  if (use_true_iekf) {
    H_theta_base = C_b_v * Skew(v_b);
  } else if (hybrid_zero) {
    H_theta_base.setZero();
  } else {
    H_theta_base = -C_b_v * Skew(v_b) * C_bn.transpose();
  }

  Matrix3d H_theta_lever = Matrix3d::Zero();
  if (!hybrid_zero && state.lever_arm.norm() > 1e-3) {
    if (use_true_iekf) {
      H_theta_lever = C_b_v * Skew(state.lever_arm) * Skew(omega_in_b);
    } else {
      H_theta_lever =
          -C_b_v * Skew(state.lever_arm) * C_bn.transpose() * Skew(omega_in_n);
    }
  }

  RowVector3d H_bg_phys =
      C_b_v.row(0) * Skew(state.lever_arm) * sf_g.asDiagonal();
  RowVector3d H_sg_phys =
      C_b_v.row(0) * Skew(state.lever_arm) * omega_ib_unbiased.asDiagonal();
  RowVector3d H_lever_phys = C_b_v.row(0) * Skew(omega_nb_b);

  Vector3d e_pitch_v(0.0, 1.0, 0.0);
  Vector3d e_yaw_v(0.0, 0.0, 1.0);
  Vector3d dv_dpitch = -e_pitch_v.cross(v_phys_v);
  Vector3d dv_dyaw = -e_yaw_v.cross(v_phys_v);

  row.pred_reading = state.odo_scale * v_phys_v.x();
  row.c_b_v_row0_x = C_b_v(0, 0);
  row.c_b_v_row0_y = C_b_v(0, 1);
  row.c_b_v_row0_z = C_b_v(0, 2);
  row.lever_x = state.lever_arm.x();
  row.lever_y = state.lever_arm.y();
  row.lever_z = state.lever_arm.z();
  row.bg_x = state.bg.x();
  row.bg_y = state.bg.y();
  row.bg_z = state.bg.z();
  row.sg_x = state.sg.x();
  row.sg_y = state.sg.y();
  row.sg_z = state.sg.z();
  row.sf_g_x = sf_g.x();
  row.sf_g_y = sf_g.y();
  row.sf_g_z = sf_g.z();
  row.omega_ib_unbiased_x = omega_ib_unbiased.x();
  row.omega_ib_unbiased_y = omega_ib_unbiased.y();
  row.omega_ib_unbiased_z = omega_ib_unbiased.z();
  row.omega_in_b_x = omega_in_b.x();
  row.omega_in_b_y = omega_in_b.y();
  row.omega_in_b_z = omega_in_b.z();
  row.omega_nb_b_x = omega_nb_b.x();
  row.omega_nb_b_y = omega_nb_b.y();
  row.omega_nb_b_z = omega_nb_b.z();
  row.v_b_x = v_b.x();
  row.v_b_y = v_b.y();
  row.v_b_z = v_b.z();
  row.v_wheel_b_x = v_wheel_b.x();
  row.v_wheel_b_y = v_wheel_b.y();
  row.v_wheel_b_z = v_wheel_b.z();
  row.v_phys_x = v_phys_v.x();
  row.v_phys_y = v_phys_v.y();
  row.v_phys_z = v_phys_v.z();
  row.h_v_x = state.odo_scale * H_v_phys(0);
  row.h_v_y = state.odo_scale * H_v_phys(1);
  row.h_v_z = state.odo_scale * H_v_phys(2);
  row.h_att_base_x = state.odo_scale * H_theta_base(0, 0);
  row.h_att_base_y = state.odo_scale * H_theta_base(0, 1);
  row.h_att_base_z = state.odo_scale * H_theta_base(0, 2);
  row.h_att_lever_x = state.odo_scale * H_theta_lever(0, 0);
  row.h_att_lever_y = state.odo_scale * H_theta_lever(0, 1);
  row.h_att_lever_z = state.odo_scale * H_theta_lever(0, 2);
  row.h_att_total_x = row.h_att_base_x + row.h_att_lever_x;
  row.h_att_total_y = row.h_att_base_y + row.h_att_lever_y;
  row.h_att_total_z = row.h_att_base_z + row.h_att_lever_z;
  row.h_bg_x = state.odo_scale * H_bg_phys(0);
  row.h_bg_y = state.odo_scale * H_bg_phys(1);
  row.h_bg_z = state.odo_scale * H_bg_phys(2);
  row.h_sg_x = state.odo_scale * H_sg_phys(0);
  row.h_sg_y = state.odo_scale * H_sg_phys(1);
  row.h_sg_z = state.odo_scale * H_sg_phys(2);
  row.h_lever_x = state.odo_scale * H_lever_phys(0);
  row.h_lever_y = state.odo_scale * H_lever_phys(1);
  row.h_lever_z = state.odo_scale * H_lever_phys(2);
  row.h_mount_pitch = state.odo_scale * dv_dpitch(0);
  row.h_mount_yaw = state.odo_scale * dv_dyaw(0);
  return row;
}

void WriteCaseMetaCsv(const fs::path &path, const vector<CaseMetaRow> &rows) {
  ofstream fout(path);
  if (!fout) {
    throw runtime_error("failed to open CSV for write: " + path.string());
  }
  fout << "case_id,config_path,sol_path,sol_mtime\n";
  for (const auto &row : rows) {
    fout << CsvEscape(row.case_id) << ','
         << CsvEscape(row.config_path) << ','
         << CsvEscape(row.sol_path) << ','
         << CsvEscape(row.sol_mtime) << '\n';
  }
}

void WriteAuditEntriesCsv(const fs::path &path,
                          const vector<AuditEntryRow> &rows) {
  ofstream fout(path);
  if (!fout) {
    throw runtime_error("failed to open CSV for write: " + path.string());
  }
  fout << setprecision(12);
  fout << "case_id,config_path,measurement,sample_label,target_t,matched_t,"
          "dt_target,imu_t,dt_imu,row_idx,state_idx,state_name,analytic,"
          "numeric,abs_diff,rel_diff,eps,speed_h,yaw_rate_deg_s,odo_speed\n";
  for (const auto &row : rows) {
    fout << CsvEscape(row.case_id) << ','
         << CsvEscape(row.config_path) << ','
         << CsvEscape(row.measurement) << ','
         << CsvEscape(row.sample_label) << ','
         << row.target_t << ','
         << row.matched_t << ','
         << row.dt_target << ','
         << row.imu_t << ','
         << row.dt_imu << ','
         << row.row_idx << ','
         << row.state_idx << ','
         << CsvEscape(row.state_name) << ','
         << row.analytic << ','
         << row.numeric << ','
         << row.abs_diff << ','
         << row.rel_diff << ','
         << row.eps << ','
         << row.speed_h << ','
         << row.yaw_rate_deg_s << ','
         << row.odo_speed << '\n';
  }
}

void WriteSampleSummaryCsv(const fs::path &path,
                           const vector<SampleSummaryRow> &rows) {
  ofstream fout(path);
  if (!fout) {
    throw runtime_error("failed to open CSV for write: " + path.string());
  }
  fout << setprecision(12);
  fout << "case_id,config_path,measurement,sample_label,target_t,matched_t,"
          "dt_target,imu_t,dt_imu,speed_h,yaw_rate_deg_s,odo_speed,y_norm,"
          "analytic_norm,numeric_norm,diff_norm,rel_fro,max_abs,worst_row,"
          "worst_col,worst_state\n";
  for (const auto &row : rows) {
    fout << CsvEscape(row.case_id) << ','
         << CsvEscape(row.config_path) << ','
         << CsvEscape(row.measurement) << ','
         << CsvEscape(row.sample_label) << ','
         << row.target_t << ','
         << row.matched_t << ','
         << row.dt_target << ','
         << row.imu_t << ','
         << row.dt_imu << ','
         << row.speed_h << ','
         << row.yaw_rate_deg_s << ','
         << row.odo_speed << ','
         << row.y_norm << ','
         << row.analytic_norm << ','
         << row.numeric_norm << ','
         << row.diff_norm << ','
         << row.rel_fro << ','
         << row.max_abs << ','
         << row.worst_row << ','
         << row.worst_col << ','
         << CsvEscape(row.worst_state) << '\n';
  }
}

void WriteOdoComponentCsv(const fs::path &path,
                          const vector<OdoComponentRow> &rows) {
  ofstream fout(path);
  if (!fout) {
    throw runtime_error("failed to open CSV for write: " + path.string());
  }
  fout << setprecision(12);
  fout
      << "case_id,config_path,sample_label,target_t,matched_t,dt_target,imu_t,"
         "dt_imu,speed_h,yaw_rate_deg_s,odo_speed,pred_reading,odo_scale,"
         "c_b_v_row0_x,c_b_v_row0_y,c_b_v_row0_z,lever_x,lever_y,lever_z,"
         "bg_x,bg_y,bg_z,sg_x,sg_y,sg_z,sf_g_x,sf_g_y,sf_g_z,"
         "omega_ib_unbiased_x,omega_ib_unbiased_y,omega_ib_unbiased_z,"
         "omega_in_b_x,omega_in_b_y,omega_in_b_z,"
         "omega_nb_b_x,omega_nb_b_y,omega_nb_b_z,"
         "v_b_x,v_b_y,v_b_z,v_wheel_b_x,v_wheel_b_y,v_wheel_b_z,"
         "v_phys_x,v_phys_y,v_phys_z,h_v_x,h_v_y,h_v_z,"
         "h_att_base_x,h_att_base_y,h_att_base_z,"
         "h_att_lever_x,h_att_lever_y,h_att_lever_z,"
         "h_att_total_x,h_att_total_y,h_att_total_z,"
         "h_bg_x,h_bg_y,h_bg_z,h_sg_x,h_sg_y,h_sg_z,"
         "h_lever_x,h_lever_y,h_lever_z,h_mount_pitch,h_mount_yaw\n";
  for (const auto &row : rows) {
    fout << CsvEscape(row.case_id) << ','
         << CsvEscape(row.config_path) << ','
         << CsvEscape(row.sample_label) << ','
         << row.target_t << ','
         << row.matched_t << ','
         << row.dt_target << ','
         << row.imu_t << ','
         << row.dt_imu << ','
         << row.speed_h << ','
         << row.yaw_rate_deg_s << ','
         << row.odo_speed << ','
         << row.pred_reading << ','
         << row.odo_scale << ','
         << row.c_b_v_row0_x << ','
         << row.c_b_v_row0_y << ','
         << row.c_b_v_row0_z << ','
         << row.lever_x << ','
         << row.lever_y << ','
         << row.lever_z << ','
         << row.bg_x << ','
         << row.bg_y << ','
         << row.bg_z << ','
         << row.sg_x << ','
         << row.sg_y << ','
         << row.sg_z << ','
         << row.sf_g_x << ','
         << row.sf_g_y << ','
         << row.sf_g_z << ','
         << row.omega_ib_unbiased_x << ','
         << row.omega_ib_unbiased_y << ','
         << row.omega_ib_unbiased_z << ','
         << row.omega_in_b_x << ','
         << row.omega_in_b_y << ','
         << row.omega_in_b_z << ','
         << row.omega_nb_b_x << ','
         << row.omega_nb_b_y << ','
         << row.omega_nb_b_z << ','
         << row.v_b_x << ','
         << row.v_b_y << ','
         << row.v_b_z << ','
         << row.v_wheel_b_x << ','
         << row.v_wheel_b_y << ','
         << row.v_wheel_b_z << ','
         << row.v_phys_x << ','
         << row.v_phys_y << ','
         << row.v_phys_z << ','
         << row.h_v_x << ','
         << row.h_v_y << ','
         << row.h_v_z << ','
         << row.h_att_base_x << ','
         << row.h_att_base_y << ','
         << row.h_att_base_z << ','
         << row.h_att_lever_x << ','
         << row.h_att_lever_y << ','
         << row.h_att_lever_z << ','
         << row.h_att_total_x << ','
         << row.h_att_total_y << ','
         << row.h_att_total_z << ','
         << row.h_bg_x << ','
         << row.h_bg_y << ','
         << row.h_bg_z << ','
         << row.h_sg_x << ','
         << row.h_sg_y << ','
         << row.h_sg_z << ','
         << row.h_lever_x << ','
         << row.h_lever_y << ','
         << row.h_lever_z << ','
         << row.h_mount_pitch << ','
         << row.h_mount_yaw << '\n';
  }
}

vector<AuditEntryRow> FilterEntries(const vector<AuditEntryRow> &entries,
                                    const string &measurement,
                                    const vector<string> &states) {
  vector<AuditEntryRow> filtered;
  for (const auto &entry : entries) {
    if (entry.measurement != measurement) {
      continue;
    }
    if (find(states.begin(), states.end(), entry.state_name) == states.end()) {
      continue;
    }
    filtered.push_back(entry);
  }
  sort(filtered.begin(), filtered.end(),
       [](const AuditEntryRow &a, const AuditEntryRow &b) {
         if (a.case_id != b.case_id) return a.case_id < b.case_id;
         if (a.sample_label != b.sample_label) return a.sample_label < b.sample_label;
         if (a.row_idx != b.row_idx) return a.row_idx < b.row_idx;
         return a.state_idx < b.state_idx;
       });
  return filtered;
}

void WriteSummaryMarkdown(const fs::path &path,
                          const vector<CaseMetaRow> &cases,
                          const vector<double> &timestamps,
                          const vector<SampleSummaryRow> &summaries,
                          const vector<AuditEntryRow> &entries,
                          const vector<OdoComponentRow> &odo_components) {
  ofstream fout(path);
  if (!fout) {
    throw runtime_error("failed to open summary for write: " + path.string());
  }

  fout << "# EXP-20260325-data2-bgz-jacobian-fd-audit-r1\n\n";
  fout << "## Setup\n\n";
  fout << "- output dir: `" << path.parent_path().generic_string() << "`\n";
  fout << "- timestamps:";
  for (size_t i = 0; i < timestamps.size(); ++i) {
    fout << (i == 0 ? " " : ", ") << '`' << ToStringPrec(timestamps[i], 6) << '`';
  }
  fout << "\n";
  fout << "- configs:\n";
  for (const auto &meta : cases) {
    fout << "  - `" << meta.case_id << "`: `" << meta.config_path
         << "` (`SOL mtime=" << meta.sol_mtime << "`)\n";
  }
  fout << "\n";

  fout << "## Sample Summary\n\n";
  fout << "| case_id | measurement | sample | matched_t | imu_t | speed_h | yaw_rate_deg_s | rel_fro | max_abs | worst_state |\n";
  fout << "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n";
  for (const auto &row : summaries) {
    fout << "| " << row.case_id
         << " | " << row.measurement
         << " | " << row.sample_label
         << " | " << ToStringPrec(row.matched_t, 6)
         << " | " << ToStringPrec(row.imu_t, 6)
         << " | " << ToStringPrec(row.speed_h, 6)
         << " | " << ToStringPrec(row.yaw_rate_deg_s, 6)
         << " | " << ToStringPrec(row.rel_fro, 6)
         << " | " << ToStringPrec(row.max_abs, 6)
         << " | " << row.worst_state << " |\n";
  }
  fout << "\n";

  vector<string> odo_focus_states = {
      "att_z", "bg_z", "sg_z", "lever_y", "mount_pitch", "mount_yaw"};
  auto odo_focus_rows = FilterEntries(entries, "ODO", odo_focus_states);
  fout << "## ODO Key Columns\n\n";
  fout << "| case_id | sample | state | analytic | numeric | abs_diff | rel_diff |\n";
  fout << "| --- | --- | --- | --- | --- | --- | --- |\n";
  for (const auto &row : odo_focus_rows) {
    fout << "| " << row.case_id
         << " | " << row.sample_label
         << " | " << row.state_name
         << " | " << ToStringPrec(row.analytic, 9)
         << " | " << ToStringPrec(row.numeric, 9)
         << " | " << ToStringPrec(row.abs_diff, 9)
         << " | " << ToStringPrec(row.rel_diff, 9) << " |\n";
  }
  fout << "\n";

  vector<string> nhc_focus_states = {"att_z", "bg_z", "sg_z"};
  auto nhc_focus_rows = FilterEntries(entries, "NHC", nhc_focus_states);
  fout << "## NHC Contrast Columns\n\n";
  fout << "| case_id | sample | row_idx | state | analytic | numeric | abs_diff | rel_diff |\n";
  fout << "| --- | --- | --- | --- | --- | --- | --- | --- |\n";
  for (const auto &row : nhc_focus_rows) {
    fout << "| " << row.case_id
         << " | " << row.sample_label
         << " | " << row.row_idx
         << " | " << row.state_name
         << " | " << ToStringPrec(row.analytic, 9)
         << " | " << ToStringPrec(row.numeric, 9)
         << " | " << ToStringPrec(row.abs_diff, 9)
         << " | " << ToStringPrec(row.rel_diff, 9) << " |\n";
  }
  fout << "\n";

  fout << "## ODO Component Breakdown\n\n";
  fout << "| case_id | sample | pred_reading | h_att_base_z | h_att_lever_z | h_att_total_z | h_bg_z | h_sg_z | h_lever_y | h_mount_pitch | h_mount_yaw |\n";
  fout << "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n";
  for (const auto &row : odo_components) {
    fout << "| " << row.case_id
         << " | " << row.sample_label
         << " | " << ToStringPrec(row.pred_reading, 9)
         << " | " << ToStringPrec(row.h_att_base_z, 9)
         << " | " << ToStringPrec(row.h_att_lever_z, 9)
         << " | " << ToStringPrec(row.h_att_total_z, 9)
         << " | " << ToStringPrec(row.h_bg_z, 9)
         << " | " << ToStringPrec(row.h_sg_z, 9)
         << " | " << ToStringPrec(row.h_lever_y, 9)
         << " | " << ToStringPrec(row.h_mount_pitch, 9)
         << " | " << ToStringPrec(row.h_mount_yaw, 9) << " |\n";
  }
  fout << "\n";

  fout << "## Artifacts\n\n";
  fout << "- `case_meta.csv`\n";
  fout << "- `fd_vs_analytic.csv`\n";
  fout << "- `sample_summary.csv`\n";
  fout << "- `odo_component_breakdown.csv`\n";
}

AuditBundle RunAuditForConfig(const string &config_path,
                              const vector<double> &target_timestamps,
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

  vector<SolRecord> records = LoadSolutionRecords(sol_path.string());
  if (records.empty()) {
    throw runtime_error("failed to parse solution records: " + sol_path.string());
  }

  AuditBundle bundle;
  bundle.meta.case_id = CaseIdFromConfigPath(config_path);
  bundle.meta.config_path = config_path;
  bundle.meta.sol_path = sol_path.generic_string();
  bundle.meta.sol_mtime = FormatFileTime(fs::last_write_time(sol_path));

  Vector3d mounting_base_rpy = ComputeMountingBaseRpyRad(options);
  FejManager fej = BuildFejManager(options, records.front().state);
  FejManager *fej_ptr = fej.enabled ? &fej : nullptr;

  for (double target_t : target_timestamps) {
    int record_idx = FindClosestRecordIndex(records, target_t);
    if (record_idx < 0) {
      throw runtime_error("failed to match solution row for target t=" +
                          ToStringPrec(target_t, 6));
    }
    const SolRecord &record = records.at(record_idx);
    int imu_idx = FindClosestImuIndex(dataset.imu, record.t);
    if (imu_idx < 0) {
      throw runtime_error("failed to match IMU row for state t=" +
                          ToStringPrec(record.t, 6));
    }
    const ImuData &imu = dataset.imu.at(imu_idx);
    Vector3d omega_ib_b_raw = Vector3d::Zero();
    if (imu.dt > 1e-9) {
      omega_ib_b_raw = imu.dtheta / imu.dt;
    }
    Matrix3d C_b_v = BuildVehicleRotation(mounting_base_rpy, record.state);
    string sample_label = MakeSampleLabel(target_t);

    if (options.constraints.enable_nhc) {
      auto nhc_analytic = MeasModels::ComputeNhcModel(
          record.state, C_b_v, omega_ib_b_raw, options.constraints.sigma_nhc_y,
          options.constraints.sigma_nhc_z, fej_ptr);
      auto nhc_builder = [&](const State &state) -> VectorXd {
        Matrix3d C_b_v_local = BuildVehicleRotation(mounting_base_rpy, state);
        auto model = MeasModels::ComputeNhcModel(
            state, C_b_v_local, omega_ib_b_raw, options.constraints.sigma_nhc_y,
            options.constraints.sigma_nhc_z, fej_ptr);
        return model.y;
      };
      MatrixXd nhc_numeric = ComputeNumericalJacobian(
          record.state, fej_ptr, eps, nhc_analytic.y, nhc_builder);
      bundle.summaries.push_back(BuildSampleSummary(
          bundle.meta, "NHC", sample_label, target_t, record, imu.t,
          nhc_analytic.y, nhc_analytic.H, nhc_numeric,
          numeric_limits<double>::quiet_NaN()));
      AppendAuditEntries(bundle.meta, "NHC", sample_label, target_t, record,
                         imu.t, nhc_analytic.H, nhc_numeric, eps, state_names,
                         numeric_limits<double>::quiet_NaN(), bundle.entries);
    }

    if (options.constraints.enable_odo) {
      double odo_speed = 0.0;
      if (!ComputeOdoMeasurementAtTime(dataset, options.constraints, options.gating,
                                       record.t, odo_speed)) {
        throw runtime_error("failed to reconstruct ODO measurement for state t=" +
                            ToStringPrec(record.t, 6));
      }
      auto odo_analytic = MeasModels::ComputeOdoModel(
          record.state, odo_speed, C_b_v, omega_ib_b_raw,
          options.constraints.sigma_odo, fej_ptr);
      auto odo_builder = [&](const State &state) -> VectorXd {
        Matrix3d C_b_v_local = BuildVehicleRotation(mounting_base_rpy, state);
        auto model = MeasModels::ComputeOdoModel(
            state, odo_speed, C_b_v_local, omega_ib_b_raw,
            options.constraints.sigma_odo, fej_ptr);
        return model.y;
      };
      MatrixXd odo_numeric = ComputeNumericalJacobian(
          record.state, fej_ptr, eps, odo_analytic.y, odo_builder);
      bundle.summaries.push_back(BuildSampleSummary(
          bundle.meta, "ODO", sample_label, target_t, record, imu.t,
          odo_analytic.y, odo_analytic.H, odo_numeric, odo_speed));
      AppendAuditEntries(bundle.meta, "ODO", sample_label, target_t, record,
                         imu.t, odo_analytic.H, odo_numeric, eps, state_names,
                         odo_speed, bundle.entries);
      bundle.odo_components.push_back(BuildOdoComponentRow(
          bundle.meta, sample_label, target_t, record, imu.t, odo_speed,
          record.state, C_b_v, omega_ib_b_raw, fej_ptr));
    }
  }

  return bundle;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    ParsedArgs args = ParseArgs(argc, argv);
    fs::create_directories(args.outdir);

    VectorXd eps = BuildFiniteDifferenceSteps();
    vector<string> state_names = BuildStateNames();

    vector<CaseMetaRow> case_rows;
    vector<AuditEntryRow> audit_rows;
    vector<SampleSummaryRow> summary_rows;
    vector<OdoComponentRow> odo_rows;

    for (const auto &config_path : args.config_paths) {
      cout << "[JacobianFDAudit] config=" << config_path << "\n";
      AuditBundle bundle =
          RunAuditForConfig(config_path, args.timestamps, eps, state_names);
      case_rows.push_back(bundle.meta);
      audit_rows.insert(audit_rows.end(),
                        bundle.entries.begin(), bundle.entries.end());
      summary_rows.insert(summary_rows.end(),
                          bundle.summaries.begin(), bundle.summaries.end());
      odo_rows.insert(odo_rows.end(),
                      bundle.odo_components.begin(), bundle.odo_components.end());
    }

    WriteCaseMetaCsv(args.outdir / "case_meta.csv", case_rows);
    WriteAuditEntriesCsv(args.outdir / "fd_vs_analytic.csv", audit_rows);
    WriteSampleSummaryCsv(args.outdir / "sample_summary.csv", summary_rows);
    WriteOdoComponentCsv(args.outdir / "odo_component_breakdown.csv", odo_rows);
    WriteSummaryMarkdown(args.outdir / "summary.md", case_rows, args.timestamps,
                         summary_rows, audit_rows, odo_rows);

    cout << "[JacobianFDAudit] Done. Artifacts:\n";
    cout << "  - " << (args.outdir / "summary.md").generic_string() << "\n";
    cout << "  - " << (args.outdir / "case_meta.csv").generic_string() << "\n";
    cout << "  - " << (args.outdir / "fd_vs_analytic.csv").generic_string() << "\n";
    cout << "  - " << (args.outdir / "sample_summary.csv").generic_string() << "\n";
    cout << "  - " << (args.outdir / "odo_component_breakdown.csv").generic_string()
         << "\n";
    return 0;
  } catch (const exception &e) {
    cerr << "odo_nhc_bgz_jacobian_fd failed: " << e.what() << "\n";
    return 1;
  }
}
