// 融合主流程：数据加载、ESKF 初始化、量测更新、结果记录
#include "app/fusion.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "app/diagnostics.h"
#include "utils/math_utils.h"

using namespace std;
using namespace Eigen;

// ============================================================
// 内部辅助函数
// ============================================================
namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kTruthAnchorPosVar = 1.0e-12;
constexpr double kTruthAnchorVelVar = 1.0e-12;
constexpr double kTruthAnchorAttVar = 1.0e-14;
using SteadyClock = std::chrono::steady_clock;

double DurationSeconds(const SteadyClock::duration &duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

bool IsPerfDebugEnabledFromEnv() {
  const char *env = std::getenv("UWB_PERF_DEBUG");
  if (env == nullptr) {
    return false;
  }
  string value(env);
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  return !(value.empty() || value == "0" || value == "false" ||
           value == "off" || value == "no");
}

struct FusionPerfStats {
  bool enabled = false;
  size_t progress_stride = 2000;
  size_t imu_steps = 0;
  size_t gnss_exact_calls = 0;
  size_t gnss_update_calls = 0;
  size_t gnss_samples = 0;
  size_t split_predict_count = 0;
  size_t align_curr_predict_count = 0;
  size_t tail_predict_count = 0;
  size_t direct_predict_count = 0;
  double predict_s = 0.0;
  double gnss_exact_s = 0.0;
  double gnss_update_s = 0.0;
  double zupt_s = 0.0;
  double gravity_diag_s = 0.0;
  double uwb_s = 0.0;
  double step_diag_s = 0.0;
  double diag_write_s = 0.0;
  SteadyClock::time_point wall_start = SteadyClock::time_point{};
};

struct ConstraintUpdateStats {
  int seen = 0;
  int accepted = 0;
  int rejected_nis = 0;
  int rejected_numeric = 0;
  double nis_sum = 0.0;
  double nis_max = 0.0;
  double robust_weight_sum = 0.0;
  double noise_scale_sum = 0.0;
};

enum class NhcAdmissionVelocitySource {
  kBody,
  kWheelBody,
  kVehicle,
};

struct NhcAdmissionKinematics {
  Vector3d v_b = Vector3d::Zero();
  Vector3d v_wheel_b = Vector3d::Zero();
  Vector3d v_v = Vector3d::Zero();
};

struct NhcAdmissionDecision {
  bool accept = true;
  bool below_forward_speed = false;
  bool exceed_lateral_vertical_limit = false;
};

NhcAdmissionVelocitySource ResolveNhcAdmissionVelocitySource(
    const ConstraintConfig &cfg) {
  if (cfg.nhc_admission_velocity_source == "v_wheel_b") {
    return NhcAdmissionVelocitySource::kWheelBody;
  }
  if (cfg.nhc_admission_velocity_source == "v_v") {
    return NhcAdmissionVelocitySource::kVehicle;
  }
  return NhcAdmissionVelocitySource::kBody;
}

const char *NhcAdmissionVelocitySourceName(
    NhcAdmissionVelocitySource source) {
  switch (source) {
    case NhcAdmissionVelocitySource::kWheelBody:
      return "v_wheel_b";
    case NhcAdmissionVelocitySource::kVehicle:
      return "v_v";
    case NhcAdmissionVelocitySource::kBody:
    default:
      return "v_b";
  }
}

NhcAdmissionKinematics ComputeNhcAdmissionKinematics(
    const State &state, const Matrix3d &C_b_v,
    const Vector3d &omega_ib_b_raw) {
  NhcAdmissionKinematics kin;
  Llh llh = EcefToLlh(state.p);
  Matrix3d R_ne = RotNedToEcef(llh);
  Matrix3d C_bn = R_ne.transpose() * QuatToRot(state.q);
  Vector3d v_ned = R_ne.transpose() * state.v;
  kin.v_b = C_bn.transpose() * v_ned;

  Vector3d omega_ie_n = OmegaIeNed(llh.lat);
  Vector3d omega_en_n = OmegaEnNed(v_ned, llh.lat, llh.h);
  Vector3d omega_in_n = omega_ie_n + omega_en_n;
  Vector3d omega_in_b = C_bn.transpose() * omega_in_n;
  Vector3d omega_ib_unbiased = omega_ib_b_raw - state.bg;
  Vector3d sf_g = (Vector3d::Ones() + state.sg).cwiseInverse();
  Vector3d omega_ib_corr = sf_g.cwiseProduct(omega_ib_unbiased);
  Vector3d omega_nb_b = omega_ib_corr - omega_in_b;

  kin.v_wheel_b = kin.v_b + omega_nb_b.cross(state.lever_arm);
  kin.v_v = C_b_v * kin.v_wheel_b;
  return kin;
}

NhcAdmissionDecision EvaluateNhcAdmissionDecision(const Vector3d &v_ref,
                                                  const ConstraintConfig &cfg) {
  NhcAdmissionDecision decision;
  if (cfg.nhc_disable_below_forward_speed > 0.0) {
    decision.below_forward_speed =
        std::abs(v_ref.x()) < cfg.nhc_disable_below_forward_speed;
  }
  if (cfg.nhc_max_abs_v > 0.0) {
    decision.exceed_lateral_vertical_limit =
        std::abs(v_ref.y()) > cfg.nhc_max_abs_v ||
        std::abs(v_ref.z()) > cfg.nhc_max_abs_v;
  }
  decision.accept = !decision.below_forward_speed &&
                    !decision.exceed_lateral_vertical_limit;
  return decision;
}

Vector3d SelectNhcAdmissionVelocity(const NhcAdmissionKinematics &kin,
                                    NhcAdmissionVelocitySource source) {
  switch (source) {
    case NhcAdmissionVelocitySource::kWheelBody:
      return kin.v_wheel_b;
    case NhcAdmissionVelocitySource::kVehicle:
      return kin.v_v;
    case NhcAdmissionVelocitySource::kBody:
    default:
      return kin.v_b;
  }
}

void LogNhcAdmissionSample(std::ofstream *file, double t,
                           NhcAdmissionVelocitySource selected_source,
                           const NhcAdmissionKinematics &kin,
                           const ConstraintConfig &cfg,
                           const NhcAdmissionDecision &decision_v_b,
                           const NhcAdmissionDecision &decision_v_wheel_b,
                           const NhcAdmissionDecision &decision_v_v) {
  if (file == nullptr || !file->is_open()) {
    return;
  }
  const NhcAdmissionDecision *selected_decision = &decision_v_b;
  switch (selected_source) {
    case NhcAdmissionVelocitySource::kWheelBody:
      selected_decision = &decision_v_wheel_b;
      break;
    case NhcAdmissionVelocitySource::kVehicle:
      selected_decision = &decision_v_v;
      break;
    case NhcAdmissionVelocitySource::kBody:
    default:
      break;
  }
  (*file) << t << ","
          << NhcAdmissionVelocitySourceName(selected_source) << ","
          << (selected_decision->accept ? 1 : 0) << ","
          << (decision_v_b.accept ? 1 : 0) << ","
          << (decision_v_wheel_b.accept ? 1 : 0) << ","
          << (decision_v_v.accept ? 1 : 0) << ","
          << (decision_v_b.below_forward_speed ? 1 : 0) << ","
          << (decision_v_wheel_b.below_forward_speed ? 1 : 0) << ","
          << (decision_v_v.below_forward_speed ? 1 : 0) << ","
          << (decision_v_b.exceed_lateral_vertical_limit ? 1 : 0) << ","
          << (decision_v_wheel_b.exceed_lateral_vertical_limit ? 1 : 0) << ","
          << (decision_v_v.exceed_lateral_vertical_limit ? 1 : 0) << ","
          << cfg.nhc_disable_below_forward_speed << ","
          << cfg.nhc_max_abs_v << ","
          << kin.v_b.x() << "," << kin.v_b.y() << "," << kin.v_b.z() << ","
          << kin.v_wheel_b.x() << "," << kin.v_wheel_b.y() << ","
          << kin.v_wheel_b.z() << ","
          << kin.v_v.x() << "," << kin.v_v.y() << "," << kin.v_v.z()
          << "\n";
}

void ApplyStateGainScaleToKalmanGain(MatrixXd &K,
                                     const StateGainScale *gain_scale) {
  if (gain_scale == nullptr || K.rows() != kStateDim) {
    return;
  }
  for (int i = 0; i < kStateDim; ++i) {
    K.row(i) *= (*gain_scale)[i];
  }
}

void ApplyStateMeasurementGainScaleToKalmanGain(
    MatrixXd &K, const StateMeasurementGainScale *gain_element_scale) {
  if (gain_element_scale == nullptr ||
      gain_element_scale->rows() != K.rows() ||
      gain_element_scale->cols() != K.cols()) {
    return;
  }
  K.array() *= gain_element_scale->array();
}

double InvNormCdf(double p) {
  // Peter J. Acklam's approximation (sufficient for gating quantiles)
  if (p <= 0.0 || p >= 1.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  static const double a1 = -3.969683028665376e+01;
  static const double a2 = 2.209460984245205e+02;
  static const double a3 = -2.759285104469687e+02;
  static const double a4 = 1.383577518672690e+02;
  static const double a5 = -3.066479806614716e+01;
  static const double a6 = 2.506628277459239e+00;
  static const double b1 = -5.447609879822406e+01;
  static const double b2 = 1.615858368580409e+02;
  static const double b3 = -1.556989798598866e+02;
  static const double b4 = 6.680131188771972e+01;
  static const double b5 = -1.328068155288572e+01;
  static const double c1 = -7.784894002430293e-03;
  static const double c2 = -3.223964580411365e-01;
  static const double c3 = -2.400758277161838e+00;
  static const double c4 = -2.549732539343734e+00;
  static const double c5 = 4.374664141464968e+00;
  static const double c6 = 2.938163982698783e+00;
  static const double d1 = 7.784695709041462e-03;
  static const double d2 = 3.224671290700398e-01;
  static const double d3 = 2.445134137142996e+00;
  static const double d4 = 3.754408661907416e+00;
  static const double plow = 0.02425;
  static const double phigh = 1.0 - plow;

  if (p < plow) {
    double q = std::sqrt(-2.0 * std::log(p));
    return (((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
           ((((d1 * q + d2) * q + d3) * q + d4) * q + 1.0);
  }
  if (p > phigh) {
    double q = std::sqrt(-2.0 * std::log(1.0 - p));
    return -(((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
            ((((d1 * q + d2) * q + d3) * q + d4) * q + 1.0);
  }

  double q = p - 0.5;
  double r = q * q;
  return (((((a1 * r + a2) * r + a3) * r + a4) * r + a5) * r + a6) * q /
         (((((b1 * r + b2) * r + b3) * r + b4) * r + b5) * r + 1.0);
}

double ChiSquareQuantile(int dof, double prob) {
  double p = std::clamp(prob, 1e-6, 1.0 - 1e-6);
  if (dof <= 0) return 0.0;
  if (dof == 1) {
    double z = InvNormCdf(0.5 * (1.0 + p));
    return z * z;
  }
  if (dof == 2) {
    return -2.0 * std::log(1.0 - p);
  }
  double z = InvNormCdf(p);
  double a = 2.0 / (9.0 * static_cast<double>(dof));
  double term = 1.0 - a + z * std::sqrt(a);
  if (term < 1e-6) term = 1e-6;
  return static_cast<double>(dof) * term * term * term;
}

bool ComputeNis(const EskfEngine &engine, const MatrixXd &H, const MatrixXd &R,
                const VectorXd &y, double &nis_out) {
  MatrixXd S = H * engine.cov() * H.transpose() + R;
  LDLT<MatrixXd> ldlt(S);
  if (ldlt.info() != Success) {
    return false;
  }
  VectorXd w = ldlt.solve(y);
  if (ldlt.info() != Success || !w.allFinite()) {
    return false;
  }
  double nis = y.dot(w);
  if (!std::isfinite(nis) || nis < 0.0) {
    return false;
  }
  nis_out = nis;
  return true;
}

bool AreOppositeJacobianBlocks(const MatrixXd &eskf_block,
                               const MatrixXd &inekf_block,
                               double tol,
                               double &rel_err_out) {
  double scale = std::max(1.0, std::max(eskf_block.norm(), inekf_block.norm()));
  double rel_err = (eskf_block + inekf_block).norm() / scale;
  rel_err_out = rel_err;
  return std::isfinite(rel_err) && rel_err <= tol;
}

double ComputeRobustWeight(const ConstraintConfig &cfg, double whitened_norm) {
  if (!cfg.enable_robust_weighting) return 1.0;
  double k = std::max(1e-6, cfg.robust_tuning);
  double w = 1.0;
  if (cfg.robust_kernel == "cauchy") {
    double r = whitened_norm / k;
    w = 1.0 / (1.0 + r * r);
  } else {  // huber
    if (whitened_norm > k) {
      w = k / whitened_norm;
    }
  }
  return std::clamp(w, cfg.robust_min_weight, 1.0);
}

bool ApplyRuntimeTruthAnchor(EskfEngine &engine, const TruthData &truth,
                             const InitConfig &init, double t, int &cursor) {
  Vector3d p_truth = Vector3d::Zero();
  Vector3d v_truth = Vector3d::Zero();
  Vector4d q_truth = Vector4d(1.0, 0.0, 0.0, 0.0);
  if (!InterpolateTruthPva(truth, t, cursor, p_truth, v_truth, q_truth)) {
    return false;
  }

  State anchored_state = engine.state();
  Matrix<double, kStateDim, kStateDim> anchored_cov = engine.cov();
  auto anchor_block = [&](int start_idx, double variance) {
    for (int idx = start_idx; idx < start_idx + 3; ++idx) {
      anchored_cov.row(idx).setZero();
      anchored_cov.col(idx).setZero();
      anchored_cov(idx, idx) = variance;
    }
  };
  if (init.runtime_truth_anchor_position) {
    anchored_state.p = p_truth;
    anchor_block(StateIdx::kPos, kTruthAnchorPosVar);
  }
  if (init.runtime_truth_anchor_velocity) {
    anchored_state.v = v_truth;
    anchor_block(StateIdx::kVel, kTruthAnchorVelVar);
  }
  if (init.runtime_truth_anchor_attitude) {
    anchored_state.q = q_truth;
    anchor_block(StateIdx::kAtt, kTruthAnchorAttVar);
  }

  engine.OverrideStateAndCov(anchored_state, anchored_cov);
  return true;
}

bool ApplyDebugSeedBeforeFirstNhc(EskfEngine &engine,
                                  const ConstraintConfig &cfg,
                                  bool &already_applied) {
  if (already_applied) {
    return false;
  }

  const double seed_mount_yaw_bgz_cov =
      cfg.debug_seed_mount_yaw_bgz_cov_before_first_nhc;
  const double seed_bg_z = cfg.debug_seed_bg_z_before_first_nhc;
  const bool seed_bg_z_att_cov =
      cfg.debug_seed_bg_z_att_cov_before_first_nhc.allFinite();
  const bool seed_mount_yaw_bgz_cov_enabled =
      std::isfinite(seed_mount_yaw_bgz_cov);
  const bool seed_bg_z_enabled = std::isfinite(seed_bg_z);
  if (!seed_mount_yaw_bgz_cov_enabled && !seed_bg_z_enabled &&
      !seed_bg_z_att_cov) {
    return false;
  }

  State seeded_state = engine.state();
  Matrix<double, kStateDim, kStateDim> seeded_cov = engine.cov();
  if (seed_mount_yaw_bgz_cov_enabled) {
    seeded_cov(StateIdx::kMountYaw, StateIdx::kBg + 2) = seed_mount_yaw_bgz_cov;
    seeded_cov(StateIdx::kBg + 2, StateIdx::kMountYaw) = seed_mount_yaw_bgz_cov;
  }
  if (seed_bg_z_att_cov) {
    for (int axis = 0; axis < 3; ++axis) {
      seeded_cov(StateIdx::kBg + 2, StateIdx::kAtt + axis) =
          cfg.debug_seed_bg_z_att_cov_before_first_nhc(axis);
      seeded_cov(StateIdx::kAtt + axis, StateIdx::kBg + 2) =
          cfg.debug_seed_bg_z_att_cov_before_first_nhc(axis);
    }
  }
  if (seed_bg_z_enabled) {
    seeded_state.bg.z() = seed_bg_z;
  }
  engine.OverrideStateAndCov(seeded_state, seeded_cov);
  already_applied = true;

  const double mount_var = std::max(0.0, seeded_cov(StateIdx::kMountYaw, StateIdx::kMountYaw));
  const double bg_var = std::max(0.0, seeded_cov(StateIdx::kBg + 2, StateIdx::kBg + 2));
  double corr = 0.0;
  if (seed_mount_yaw_bgz_cov_enabled && mount_var > 0.0 && bg_var > 0.0) {
    corr = seed_mount_yaw_bgz_cov / std::sqrt(mount_var * bg_var);
  }
  cout << "[Debug] Seed before first NHC: "
       << "P(mount_yaw,bg_z)=" << seed_mount_yaw_bgz_cov
       << " corr=" << corr
       << " bg_z=" << seed_bg_z
       << " P(bg_z,att_xyz)="
       << cfg.debug_seed_bg_z_att_cov_before_first_nhc.transpose() << "\n";
  return true;
}

bool ApplyDebugResetBgzStateAndCov(EskfEngine &engine,
                                   const ConstraintConfig &cfg,
                                   double t,
                                   double time_tolerance,
                                   bool &already_applied) {
  if (already_applied) {
    return false;
  }
  if (!std::isfinite(cfg.debug_reset_bg_z_state_and_cov_after_time) ||
      !std::isfinite(cfg.debug_reset_bg_z_value) ||
      t + time_tolerance < cfg.debug_reset_bg_z_state_and_cov_after_time) {
    return false;
  }

  State reset_state = engine.state();
  Matrix<double, kStateDim, kStateDim> reset_cov = engine.cov();
  const int bg_z_idx = StateIdx::kBg + 2;
  const double bg_z_before = reset_state.bg.z();
  const double bg_var_before = reset_cov(bg_z_idx, bg_z_idx);
  double offdiag_energy = 0.0;
  for (int idx = 0; idx < kStateDim; ++idx) {
    if (idx == bg_z_idx) continue;
    offdiag_energy += reset_cov(bg_z_idx, idx) * reset_cov(bg_z_idx, idx);
    reset_cov(bg_z_idx, idx) = 0.0;
    reset_cov(idx, bg_z_idx) = 0.0;
  }
  reset_cov(bg_z_idx, bg_z_idx) =
      std::isfinite(bg_var_before) ? std::max(0.0, bg_var_before) : 0.0;
  reset_state.bg.z() = cfg.debug_reset_bg_z_value;
  engine.OverrideStateAndCov(reset_state, reset_cov);
  already_applied = true;

  cout << "[Debug] Mid-run bg_z reset applied at t=" << fixed
       << setprecision(6) << t
       << " target_t=" << cfg.debug_reset_bg_z_state_and_cov_after_time
       << " bg_z_before=" << bg_z_before
       << " bg_z_after=" << cfg.debug_reset_bg_z_value
       << " bg_var=" << reset_cov(bg_z_idx, bg_z_idx)
       << " cleared_cross_cov_norm=" << std::sqrt(offdiag_energy) << "\n";
  return true;
}

void ValidateInEkfHSignConsistencyOrThrow(const FusionOptions &options,
                                          const State &seed_state,
                                          const Vector3d &mounting_base_rpy,
                                          const ImuData &imu_for_check) {
  State probe = seed_state;
  probe.v = Vector3d(8.0, -2.5, 0.4);
  probe.q = NormalizeQuat(RpyToQuat(Vector3d(0.35, -0.22, 0.18)));
  probe.bg = Vector3d(0.01, -0.02, 0.03);
  probe.sg = Vector3d(150e-6, -80e-6, 60e-6);
  probe.mounting_pitch = 1.5 * kDegToRad;
  probe.mounting_yaw = -2.0 * kDegToRad;
  probe.lever_arm = Vector3d(0.8, 0.2, -0.3);
  probe.gnss_lever_arm = Vector3d(1.1, -0.25, 0.35);

  Vector3d omega_ib_b_raw = Vector3d::Zero();
  if (imu_for_check.dt > 1e-9) {
    omega_ib_b_raw = imu_for_check.dtheta / imu_for_check.dt;
  }

  Vector3d rpy(mounting_base_rpy.x(),
               mounting_base_rpy.y() + probe.mounting_pitch,
               mounting_base_rpy.z() + probe.mounting_yaw);
  Matrix3d C_b_v = QuatToRot(RpyToQuat(rpy)).transpose();

  double sigma_gnss = (options.noise.sigma_gnss_pos > 0.0)
                          ? options.noise.sigma_gnss_pos
                          : 1.0;
  Vector3d sigma_gnss_vec = Vector3d::Constant(std::max(1e-3, sigma_gnss));
  Vector3d z_gnss_probe = probe.p + Vector3d(3.0, -1.5, 2.0);

  double sigma_odo = std::max(1e-3, options.constraints.sigma_odo);
  double sigma_nhc_y = std::max(1e-3, options.constraints.sigma_nhc_y);
  double sigma_nhc_z = std::max(1e-3, options.constraints.sigma_nhc_z);

  FejManager fej_probe;
  fej_probe.Enable(true);

  auto gnss_eskf = MeasModels::ComputeGnssPositionModel(
      probe, z_gnss_probe, sigma_gnss_vec, nullptr);
  auto gnss_inekf = MeasModels::ComputeGnssPositionModel(
      probe, z_gnss_probe, sigma_gnss_vec, &fej_probe);
  auto odo_eskf = MeasModels::ComputeOdoModel(
      probe, 7.5, C_b_v, omega_ib_b_raw, sigma_odo, nullptr);
  auto odo_inekf = MeasModels::ComputeOdoModel(
      probe, 7.5, C_b_v, omega_ib_b_raw, sigma_odo, &fej_probe);
  auto nhc_eskf = MeasModels::ComputeNhcModel(
      probe, C_b_v, omega_ib_b_raw, sigma_nhc_y, sigma_nhc_z, nullptr);
  auto nhc_inekf = MeasModels::ComputeNhcModel(
      probe, C_b_v, omega_ib_b_raw, sigma_nhc_y, sigma_nhc_z, &fej_probe);

  const double tol = 1e-8;
  double rel_gnss = 0.0;
  double rel_odo = 0.0;
  double rel_nhc = 0.0;
  bool gnss_ok = AreOppositeJacobianBlocks(
      gnss_eskf.H.block<3, 3>(0, StateIdx::kAtt),
      gnss_inekf.H.block<3, 3>(0, StateIdx::kAtt),
      tol, rel_gnss);
  bool odo_ok = AreOppositeJacobianBlocks(
      odo_eskf.H.block<1, 3>(0, StateIdx::kAtt),
      odo_inekf.H.block<1, 3>(0, StateIdx::kAtt),
      tol, rel_odo);
  bool nhc_ok = AreOppositeJacobianBlocks(
      nhc_eskf.H.block<2, 3>(0, StateIdx::kAtt),
      nhc_inekf.H.block<2, 3>(0, StateIdx::kAtt),
      tol, rel_nhc);

  if (!(gnss_ok && odo_ok && nhc_ok)) {
    cout << "[ERROR] InEKF H-matrix sign inconsistency detected"
         << " (gnss_rel=" << rel_gnss
         << ", odo_rel=" << rel_odo
         << ", nhc_rel=" << rel_nhc << ")\n";
    throw runtime_error("[ERROR] InEKF H-matrix sign inconsistency detected");
  }
  cout << "[Init] InEKF H-matrix sign consistency check: PASS\n";
}

bool IsWeakExcitation(const State &state, const ImuData &imu,
                      const ConstraintConfig &cfg) {
  if (!cfg.freeze_extrinsics_when_weak_excitation) {
    return false;
  }
  if (imu.dt <= 1e-9) {
    return false;
  }
  Matrix3d Cbn = QuatToRot(state.q);
  Vector3d v_b = Cbn.transpose() * state.v;
  Vector3d omega = imu.dtheta / imu.dt;
  Vector3d acc = imu.dvel / imu.dt;

  return std::abs(v_b.x()) < cfg.excitation_min_speed &&
         std::abs(omega.z()) < cfg.excitation_min_yaw_rate &&
         std::abs(acc.y()) < cfg.excitation_min_lateral_acc;
}

bool MeetsWeakExcitationThresholds(const State &state, const ImuData &imu,
                                  const ConstraintConfig &cfg) {
  if (imu.dt <= 1e-9) {
    return false;
  }
  Matrix3d Cbn = QuatToRot(state.q);
  Vector3d v_b = Cbn.transpose() * state.v;
  Vector3d omega = imu.dtheta / imu.dt;
  Vector3d acc = imu.dvel / imu.dt;

  return std::abs(v_b.x()) < cfg.excitation_min_speed &&
         std::abs(omega.z()) < cfg.excitation_min_yaw_rate &&
         std::abs(acc.y()) < cfg.excitation_min_lateral_acc;
}

bool IsTimeInWindow(double t, double start_time, double end_time,
                    double tol = 1.0e-6) {
  return std::isfinite(start_time) && std::isfinite(end_time) &&
         t >= start_time - tol && t <= end_time + tol;
}

struct BgzObservabilityGateInfo {
  double forward_speed_abs = 0.0;
  double yaw_rate_abs = 0.0;
  double lateral_acc_abs = 0.0;
  double gate_scale = 1.0;
};

BgzObservabilityGateInfo ComputeBgzObservabilityGateInfo(
    const State &state, const ImuData &imu, const ConstraintConfig &cfg) {
  BgzObservabilityGateInfo info;
  if (imu.dt <= 1e-9) {
    return info;
  }

  Matrix3d Cbn = QuatToRot(state.q);
  Vector3d v_b = Cbn.transpose() * state.v;
  Vector3d omega = imu.dtheta / imu.dt;
  Vector3d acc = imu.dvel / imu.dt;
  const double yaw_rate_min = cfg.bgz_gate_yaw_rate_min_deg_s * kDegToRad;

  info.forward_speed_abs = std::abs(v_b.x());
  info.yaw_rate_abs = std::abs(omega.z());
  info.lateral_acc_abs = std::abs(acc.y());

  const double speed_score = std::clamp(
      info.forward_speed_abs / std::max(1e-12, cfg.bgz_gate_forward_speed_min),
      0.0, 1.0);
  const double yaw_score = std::clamp(
      info.yaw_rate_abs / std::max(1e-12, yaw_rate_min), 0.0, 1.0);
  const double lat_score = std::clamp(
      info.lateral_acc_abs /
          std::max(1e-12, cfg.bgz_gate_lateral_acc_min),
      0.0, 1.0);
  const double turn_score = std::max(yaw_score, lat_score);
  const double raw_gate = speed_score * turn_score;
  info.gate_scale =
      std::clamp(std::max(cfg.bgz_gate_min_scale, raw_gate), 0.0, 1.0);
  return info;
}

bool ApplyBgzCovarianceForgettingIfNeeded(EskfEngine &engine, const ImuData &imu,
                                          const ConstraintConfig &cfg) {
  if (!cfg.enable_bgz_covariance_forgetting || imu.dt <= 1e-9) {
    return false;
  }
  const BgzObservabilityGateInfo gate =
      ComputeBgzObservabilityGateInfo(engine.state(), imu, cfg);
  if (gate.gate_scale >= 1.0 - 1e-12) {
    return false;
  }

  const double tau = std::max(1e-12, cfg.bgz_cov_forgetting_tau_s);
  const double decay = std::exp(-(1.0 - gate.gate_scale) * imu.dt / tau);
  if (decay >= 1.0 - 1e-12) {
    return false;
  }

  State state = engine.state();
  Matrix<double, kStateDim, kStateDim> P = engine.cov();
  const int bg_z_idx = StateIdx::kBg + 2;
  for (int idx = 0; idx < kStateDim; ++idx) {
    if (idx == bg_z_idx) continue;
    P(bg_z_idx, idx) *= decay;
    P(idx, bg_z_idx) *= decay;
  }
  engine.OverrideStateAndCov(state, P);
  return true;
}

void ZeroExtrinsicSensitivity(MatrixXd &H, bool freeze_scale, bool freeze_mounting,
                              bool freeze_lever) {
  if (freeze_scale && H.cols() > StateIdx::kOdoScale) {
    H.col(StateIdx::kOdoScale).setZero();
  }
  if (freeze_mounting && H.cols() > StateIdx::kMountYaw) {
    H.block(0, StateIdx::kMountRoll, H.rows(), 3).setZero();
  }
  if (freeze_lever && H.cols() > StateIdx::kLever + 2) {
    H.block(0, StateIdx::kLever, H.rows(), 3).setZero();
  }
}

bool CorrectConstraintWithRobustness(EskfEngine &engine, const string &tag, double t,
                                     const ImuData &imu, const ConstraintConfig &cfg,
                                     const VectorXd &y, MatrixXd H, MatrixXd R,
                                     bool freeze_scale, bool freeze_mounting,
                                     bool freeze_lever, double nis_prob,
                                     bool apply_bgz_observability_gate,
                                     const StateMask *update_mask,
                                     DiagnosticsEngine &diag,
                                     ConstraintUpdateStats &stats) {
  ++stats.seen;

  if (IsWeakExcitation(engine.state(), imu, cfg)) {
    ZeroExtrinsicSensitivity(H, freeze_scale, freeze_mounting, freeze_lever);
  }

  StateGainScale bgz_gate_scale;
  const StateGainScale *bgz_gate_scale_ptr = nullptr;
  const BgzObservabilityGateInfo bgz_gate =
      ComputeBgzObservabilityGateInfo(engine.state(), imu, cfg);
  if (cfg.enable_bgz_observability_gate &&
      apply_bgz_observability_gate &&
      bgz_gate.gate_scale < 1.0 - 1e-12) {
    bgz_gate_scale.fill(1.0);
    bgz_gate_scale[StateIdx::kBg + 2] = bgz_gate.gate_scale;
    bgz_gate_scale_ptr = &bgz_gate_scale;
  }

  // 先使用原始 R 做 NIS 门控，避免“先鲁棒放大 R 再门控”导致门控被削弱。
  double nis_gate = 0.0;
  if (!ComputeNis(engine, H, R, y, nis_gate)) {
    ++stats.rejected_numeric;
    return false;
  }

  int dof = std::max(1, static_cast<int>(y.size()));
  double gate = ChiSquareQuantile(dof, nis_prob);
  if (cfg.enable_nis_gating && nis_gate > gate) {
    ++stats.rejected_nis;
    if (cfg.enable_consistency_log &&
        (stats.rejected_nis <= 20 || (stats.rejected_nis % 5000) == 0)) {
      cout << "[Consistency] " << tag << " reject t=" << t
           << " NIS=" << nis_gate << " gate=" << gate << "\n";
    }
    return false;
  }

  double robust_w = ComputeRobustWeight(cfg, std::sqrt(std::max(0.0, nis_gate)));
  MatrixXd R_eff = R;
  if (cfg.enable_robust_weighting) {
    R_eff /= (robust_w * robust_w);
  }

  // 仅用于调试统计，不参与是否更新的决策。
  double nis_update = 0.0;
  (void)ComputeNis(engine, H, R_eff, y, nis_update);

  bool updated = diag.Correct(engine, tag, t, y, H, R_eff, update_mask,
                              bgz_gate_scale_ptr, nullptr);
  if (!updated) {
    ++stats.rejected_numeric;
    return false;
  }
  ++stats.accepted;
  stats.nis_sum += nis_gate;
  stats.nis_max = std::max(stats.nis_max, nis_gate);
  stats.robust_weight_sum += robust_w;
  stats.noise_scale_sum += 1.0;
  return true;
}

void PrintConstraintStats(const string &tag, const ConstraintUpdateStats &s) {
  if (s.seen <= 0) return;
  double accept_ratio = static_cast<double>(s.accepted) / static_cast<double>(s.seen);
  // nis_mean/nis_max 记录的是门控阶段的原始 NIS（未做鲁棒加权前）。
  double nis_mean = (s.accepted > 0) ? s.nis_sum / static_cast<double>(s.accepted) : 0.0;
  double w_mean =
      (s.accepted > 0) ? s.robust_weight_sum / static_cast<double>(s.accepted) : 0.0;
  double scale_mean =
      (s.accepted > 0) ? s.noise_scale_sum / static_cast<double>(s.accepted) : 0.0;
  cout << "[Consistency] " << tag
       << " seen=" << s.seen
       << " accepted=" << s.accepted
       << " accept_ratio=" << accept_ratio
       << " reject_nis=" << s.rejected_nis
       << " reject_numeric=" << s.rejected_numeric
       << " nis_mean=" << nis_mean
       << " nis_max=" << s.nis_max
       << " robust_w_mean=" << w_mean
       << " noise_scale_mean=" << scale_mean << "\n";
}

double SafeCorrFromCov(const Matrix<double, kStateDim, kStateDim> &P,
                       int a, int b) {
  double pa = P(a, a);
  double pb = P(b, b);
  if (pa < 1e-30 || pb < 1e-30) {
    return 0.0;
  }
  return P(a, b) / std::sqrt(pa * pb);
}

void MaybeCaptureGnssSplitDebug(FusionDebugCapture *debug_capture,
                                const EskfEngine &engine,
                                const string &tag,
                                double t_meas,
                                double split_t,
                                double tol) {
  if (debug_capture == nullptr || !debug_capture->capture_last_gnss_before_split) {
    return;
  }
  if (!(t_meas <= split_t + tol)) {
    return;
  }
  if (debug_capture->gnss_split_cov.valid &&
      t_meas + tol < debug_capture->gnss_split_cov.t_meas) {
    return;
  }

  const auto &P = engine.cov();
  GnssSplitCovarianceCapture cov_capture;
  cov_capture.valid = true;
  cov_capture.tag = tag;
  cov_capture.split_t = split_t;
  cov_capture.t_meas = t_meas;
  cov_capture.t_state = engine.timestamp();
  cov_capture.P_att_bgz = P.block<3, 1>(StateIdx::kAtt, StateIdx::kBg + 2);
  cov_capture.att_var = P.diagonal().segment<3>(StateIdx::kAtt);
  cov_capture.bgz_var = P(StateIdx::kBg + 2, StateIdx::kBg + 2);
  for (int i = 0; i < 3; ++i) {
    cov_capture.corr_att_bgz(i) =
        SafeCorrFromCov(P, StateIdx::kAtt + i, StateIdx::kBg + 2);
  }
  debug_capture->gnss_split_cov = cov_capture;

  const auto &reset_snapshot = engine.last_true_iekf_correction();
  if (reset_snapshot.valid) {
    ResetConsistencyCapture reset_capture;
    reset_capture.valid = true;
    reset_capture.tag = tag;
    reset_capture.split_t = split_t;
    reset_capture.t_meas = t_meas;
    reset_capture.t_state = reset_snapshot.t_state;
    reset_capture.P_tilde = reset_snapshot.P_tilde;
    reset_capture.P_after_reset = reset_snapshot.P_after_reset;
    reset_capture.P_after_all = reset_snapshot.P_after_all;
    reset_capture.dx = reset_snapshot.dx;
    reset_capture.covariance_floor_applied =
        reset_snapshot.covariance_floor_applied;
    debug_capture->reset_consistency = reset_capture;
  }
}

StateMask BuildStateMask(const StateAblationConfig &cfg) {
  StateMask mask;
  mask.fill(true);
  auto disable_range = [&](int start, int len) {
    for (int i = 0; i < len; ++i) {
      mask[start + i] = false;
    }
  };
  if (cfg.disable_accel_bias) {
    disable_range(StateIdx::kBa, 3);
  }
  if (cfg.disable_gyro_bias) {
    disable_range(StateIdx::kBg, 3);
  }
  if (cfg.disable_gyro_scale) {
    disable_range(StateIdx::kSg, 3);
  }
  if (cfg.disable_accel_scale) {
    disable_range(StateIdx::kSa, 3);
  }
  if (cfg.disable_odo_scale) {
    disable_range(StateIdx::kOdoScale, 1);
  }
  if (cfg.disable_mounting) {
    disable_range(StateIdx::kMountRoll, 3);
  }
  if (cfg.disable_mounting_roll) {
    disable_range(StateIdx::kMountRoll, 1);
  }
  if (cfg.disable_mounting_pitch) {
    disable_range(StateIdx::kMountPitch, 1);
  }
  if (cfg.disable_mounting_yaw) {
    disable_range(StateIdx::kMountYaw, 1);
  }
  if (cfg.disable_odo_lever_arm) {
    disable_range(StateIdx::kLever, 3);
  }
  if (cfg.disable_gnss_lever_arm) {
    disable_range(StateIdx::kGnssLever, 3);
  }
  if (cfg.disable_gnss_lever_z) {
    disable_range(StateIdx::kGnssLever + 2, 1);
  }
  return mask;
}

StateMask BuildGnssPosNonPositionMask() {
  StateMask mask;
  mask.fill(true);
  for (int axis = 0; axis < 3; ++axis) {
    mask[StateIdx::kPos + axis] = false;
  }
  return mask;
}

StateMask BuildGnssPosPositionOnlyMask() {
  StateMask mask;
  mask.fill(false);
  for (int axis = 0; axis < 3; ++axis) {
    mask[StateIdx::kPos + axis] = true;
  }
  return mask;
}

StateAblationConfig MergeAblationConfig(const StateAblationConfig &base,
                                        const StateAblationConfig &extra) {
  StateAblationConfig out = base;
  out.disable_gnss_lever_arm =
      out.disable_gnss_lever_arm || extra.disable_gnss_lever_arm;
  out.disable_gnss_lever_z =
      out.disable_gnss_lever_z || extra.disable_gnss_lever_z;
  out.disable_odo_lever_arm =
      out.disable_odo_lever_arm || extra.disable_odo_lever_arm;
  out.disable_odo_scale =
      out.disable_odo_scale || extra.disable_odo_scale;
  out.disable_accel_bias =
      out.disable_accel_bias || extra.disable_accel_bias;
  out.disable_gyro_bias =
      out.disable_gyro_bias || extra.disable_gyro_bias;
  out.disable_gyro_scale =
      out.disable_gyro_scale || extra.disable_gyro_scale;
  out.disable_accel_scale =
      out.disable_accel_scale || extra.disable_accel_scale;
  out.disable_mounting =
      out.disable_mounting || extra.disable_mounting;
  out.disable_mounting_roll =
      out.disable_mounting_roll || extra.disable_mounting_roll;
  out.disable_mounting_pitch =
      out.disable_mounting_pitch || extra.disable_mounting_pitch;
  out.disable_mounting_yaw =
      out.disable_mounting_yaw || extra.disable_mounting_yaw;
  return out;
}

void ApplyAblationToNoise(NoiseParams &noise, const StateAblationConfig &cfg) {
  if (cfg.disable_accel_bias) {
    noise.sigma_ba = 0.0;
    noise.sigma_ba_vec.setZero();
  }
  if (cfg.disable_gyro_bias) {
    noise.sigma_bg = 0.0;
    noise.sigma_bg_vec.setZero();
  }
  if (cfg.disable_gyro_scale) {
    noise.sigma_sg = 0.0;
    noise.sigma_sg_vec.setZero();
  }
  if (cfg.disable_accel_scale) {
    noise.sigma_sa = 0.0;
    noise.sigma_sa_vec.setZero();
  }
  if (cfg.disable_odo_scale) {
    noise.sigma_odo_scale = 0.0;
  }
  if (cfg.disable_mounting) {
    noise.sigma_mounting = 0.0;
    noise.sigma_mounting_roll = 0.0;
    noise.sigma_mounting_pitch = 0.0;
    noise.sigma_mounting_yaw = 0.0;
  } else {
    if (cfg.disable_mounting_roll) {
      noise.sigma_mounting_roll = 0.0;
    }
    if (cfg.disable_mounting_pitch) {
      noise.sigma_mounting_pitch = 0.0;
    }
    if (cfg.disable_mounting_yaw) {
      noise.sigma_mounting_yaw = 0.0;
    }
  }
  if (cfg.disable_odo_lever_arm) {
    noise.sigma_lever_arm = 0.0;
    noise.sigma_lever_arm_vec.setZero();
  }
  if (cfg.disable_gnss_lever_arm) {
    noise.sigma_gnss_lever_arm = 0.0;
    noise.sigma_gnss_lever_arm_vec.setZero();
  } else if (cfg.disable_gnss_lever_z) {
    if ((noise.sigma_gnss_lever_arm_vec.array() >= 0.0).all()) {
      noise.sigma_gnss_lever_arm_vec.z() = 0.0;
    }
  }
}

void ApplyRuntimeNoiseOverride(NoiseParams &noise,
                               const RuntimeNoiseOverride &override) {
  if (std::isfinite(override.sigma_acc)) {
    noise.sigma_acc = override.sigma_acc;
  }
  if (std::isfinite(override.sigma_gyro)) {
    noise.sigma_gyro = override.sigma_gyro;
  }
  if (std::isfinite(override.sigma_ba)) {
    noise.sigma_ba = override.sigma_ba;
  }
  if (std::isfinite(override.sigma_bg)) {
    noise.sigma_bg = override.sigma_bg;
  }
  if (std::isfinite(override.sigma_sg)) {
    noise.sigma_sg = override.sigma_sg;
  }
  if (std::isfinite(override.sigma_sa)) {
    noise.sigma_sa = override.sigma_sa;
  }
  if (override.sigma_ba_vec.allFinite()) {
    noise.sigma_ba_vec = override.sigma_ba_vec;
  }
  if (override.sigma_bg_vec.allFinite()) {
    noise.sigma_bg_vec = override.sigma_bg_vec;
  }
  if (override.sigma_sg_vec.allFinite()) {
    noise.sigma_sg_vec = override.sigma_sg_vec;
  }
  if (override.sigma_sa_vec.allFinite()) {
    noise.sigma_sa_vec = override.sigma_sa_vec;
  }
  if (std::isfinite(override.sigma_odo_scale)) {
    noise.sigma_odo_scale = override.sigma_odo_scale;
  }
  if (std::isfinite(override.sigma_mounting)) {
    noise.sigma_mounting = override.sigma_mounting;
  }
  if (std::isfinite(override.sigma_mounting_roll)) {
    noise.sigma_mounting_roll = override.sigma_mounting_roll;
  }
  if (std::isfinite(override.sigma_mounting_pitch)) {
    noise.sigma_mounting_pitch = override.sigma_mounting_pitch;
  }
  if (std::isfinite(override.sigma_mounting_yaw)) {
    noise.sigma_mounting_yaw = override.sigma_mounting_yaw;
  }
  if (std::isfinite(override.sigma_lever_arm)) {
    noise.sigma_lever_arm = override.sigma_lever_arm;
  }
  if (std::isfinite(override.sigma_gnss_lever_arm)) {
    noise.sigma_gnss_lever_arm = override.sigma_gnss_lever_arm;
  }
  if (override.sigma_lever_arm_vec.allFinite()) {
    noise.sigma_lever_arm_vec = override.sigma_lever_arm_vec;
  }
  if (override.sigma_gnss_lever_arm_vec.allFinite()) {
    noise.sigma_gnss_lever_arm_vec = override.sigma_gnss_lever_arm_vec;
  }
  if (std::isfinite(override.sigma_uwb)) {
    noise.sigma_uwb = override.sigma_uwb;
  }
  if (std::isfinite(override.sigma_gnss_pos)) {
    noise.sigma_gnss_pos = override.sigma_gnss_pos;
  }
  if (std::isfinite(override.markov_corr_time)) {
    noise.markov_corr_time = override.markov_corr_time;
  }
  if (override.has_disable_nominal_ba_bg_decay) {
    noise.disable_nominal_ba_bg_decay = override.disable_nominal_ba_bg_decay;
  }
}

ConstraintConfig ApplyRuntimeConstraintOverride(
    const ConstraintConfig &base,
    const RuntimeConstraintOverride &override) {
  ConstraintConfig out = base;
  if (override.has_enable_nhc) {
    out.enable_nhc = override.enable_nhc;
  }
  if (override.has_enable_odo) {
    out.enable_odo = override.enable_odo;
  }
  if (override.has_enable_covariance_floor) {
    out.enable_covariance_floor = override.enable_covariance_floor;
  }
  if (override.has_enable_nis_gating) {
    out.enable_nis_gating = override.enable_nis_gating;
  }
  if (override.has_odo_nis_gate_prob) {
    out.odo_nis_gate_prob = override.odo_nis_gate_prob;
  }
  if (override.has_nhc_nis_gate_prob) {
    out.nhc_nis_gate_prob = override.nhc_nis_gate_prob;
  }
  if (override.has_p_floor_odo_scale_var) {
    out.p_floor_odo_scale_var = override.p_floor_odo_scale_var;
  }
  if (override.has_p_floor_lever_arm_vec) {
    out.p_floor_lever_arm_vec = override.p_floor_lever_arm_vec;
  }
  if (override.has_p_floor_mounting_deg) {
    out.p_floor_mounting_deg = override.p_floor_mounting_deg;
  }
  return out;
}

string ComputeEffectiveGnssPosUpdateMode(const FusionOptions &options,
                                         double t_now) {
  string effective = options.gnss_pos_update_mode;
  for (const auto &phase : options.runtime_phases) {
    if (!phase.enabled ||
        !IsTimeInWindow(t_now, phase.start_time, phase.end_time,
                        options.gating.time_tolerance)) {
      continue;
    }
    if (phase.constraints.has_gnss_pos_update_mode) {
      effective = phase.constraints.gnss_pos_update_mode;
    }
  }
  return effective;
}

bool IsTimeInAnyWindow(double t, const vector<pair<double, double>> &windows,
                       double tol) {
  for (const auto &window : windows) {
    if (t >= window.first - tol && t <= window.second + tol) {
      return true;
    }
  }
  return false;
}

int AdvanceTimestampIndexToTime(const VectorXd &timestamps, int &idx,
                                double t_curr, double tol) {
  const int n = static_cast<int>(timestamps.size());
  const int start_idx = std::clamp(idx, 0, n);
  idx = start_idx;
  while (idx < n && timestamps(idx) <= t_curr + tol) {
    ++idx;
  }
  return idx - start_idx;
}

int AdvanceMatrixTimeIndexToTime(const MatrixXd &data, int &idx, int time_col,
                                 double time_offset, double t_curr, double tol) {
  const int n = data.rows();
  const int start_idx = std::clamp(idx, 0, n);
  idx = start_idx;
  while (idx < n && data(idx, time_col) + time_offset <= t_curr + tol) {
    ++idx;
  }
  return idx - start_idx;
}

// ---------- UWB 调度辅助 ----------
bool BuildUniqueAnchorIndices(const vector<int> &indices, int n_anchors,
                              const string &label, vector<int> &out) {
  out.clear();
  for (int idx : indices) {
    if (idx < 0 || idx >= n_anchors) {
      cout << "error: " << label << " 索引超出基站数量范围\n";
      return false;
    }
    bool exists = false;
    for (int v : out) { if (v == idx) { exists = true; break; } }
    if (!exists) out.push_back(idx);
  }
  return true;
}

bool ComputeUwbTimeRange(const MatrixXd &uwb, double &t0, double &t1) {
  if (uwb.rows() == 0) return false;
  t0 = t1 = uwb(0, 0);
  for (int i = 1; i < uwb.rows(); ++i) {
    double t = uwb(i, 0);
    if (t < t0) t0 = t;
    if (t > t1) t1 = t;
  }
  return true;
}

// ---------- ZUPT 静止检测 ----------
bool DetectZupt(const ImuData &imu, const State &state,
                const ConstraintConfig &config) {
  if (imu.dt <= 1e-9) return false;
  Vector3d omega = imu.dtheta / imu.dt;
  Vector3d f_b = imu.dvel / imu.dt;
  double acc_diff = std::abs(f_b.norm() - 9.81);
  if (state.v.norm() > config.zupt_max_speed) return false;
  return (omega.norm() < config.zupt_max_gyro && acc_diff < config.zupt_max_acc);
}

// ---------- 量测更新辅助函数 ----------

/**
 * 执行 ZUPT 更新，返回是否触发了零速约束。
 */
bool RunZuptUpdate(EskfEngine &engine, const ImuData &imu,
                   const ConstraintConfig &cfg, double &static_duration,
                   DiagnosticsEngine &diag, double t) {
  if (!cfg.enable_zupt) {
    static_duration = 0.0;
    return false;
  }

  bool is_static = DetectZupt(imu, engine.state(), cfg);
  double dt = imu.dt;
  if (is_static && dt > 0.0) {
    static_duration += dt;
  } else if (!is_static) {
    static_duration = 0.0;
  }

  if (is_static && static_duration + 1e-12 >= cfg.zupt_min_duration) {
    auto model = MeasModels::ComputeZuptModel(engine.state(), cfg.sigma_zupt);
    return diag.Correct(engine, "ZUPT", t, model.y, model.H, model.R);
  }
  return false;
}

/**
 * 执行 NHC 更新。
 */
void RunNhcUpdate(EskfEngine &engine, const ImuData &imu,
                  const ConstraintConfig &cfg, const Vector3d &mounting_base_rpy,
                  bool zupt_ready, DiagnosticsEngine &diag, double t,
                  const FejManager *fej, ConstraintUpdateStats &stats,
                  double &last_nhc_update_t, double nhc_min_interval,
                  std::ofstream *nhc_admission_log_file) {
  if (!cfg.enable_nhc || (cfg.enable_zupt && zupt_ready)) return;
  if (IsTimeInWindow(t, cfg.debug_nhc_disable_start_time,
                     cfg.debug_nhc_disable_end_time)) {
    return;
  }
  if (nhc_min_interval > 0.0 &&
      last_nhc_update_t > -1e17 &&
      (t - last_nhc_update_t) < nhc_min_interval) {
    return;
  }

  double dt = imu.dt;
  if (dt <= 1e-9) return;
  const State &state = engine.state();
  if (cfg.disable_nhc_when_weak_excitation &&
      MeetsWeakExcitationThresholds(state, imu, cfg)) {
    return;
  }

  Vector3d mounting_rpy(mounting_base_rpy.x(),
                        mounting_base_rpy.y() + state.mounting_pitch,
                        mounting_base_rpy.z() + state.mounting_yaw);
  Matrix3d C_b_v = QuatToRot(RpyToQuat(mounting_rpy)).transpose();
  Vector3d omega_ib_b_raw = imu.dtheta / dt;
  const NhcAdmissionKinematics admission_kin =
      ComputeNhcAdmissionKinematics(state, C_b_v, omega_ib_b_raw);
  const NhcAdmissionDecision decision_v_b =
      EvaluateNhcAdmissionDecision(admission_kin.v_b, cfg);
  const NhcAdmissionDecision decision_v_wheel_b =
      EvaluateNhcAdmissionDecision(admission_kin.v_wheel_b, cfg);
  const NhcAdmissionDecision decision_v_v =
      EvaluateNhcAdmissionDecision(admission_kin.v_v, cfg);
  const NhcAdmissionVelocitySource selected_source =
      ResolveNhcAdmissionVelocitySource(cfg);
  const Vector3d selected_velocity =
      SelectNhcAdmissionVelocity(admission_kin, selected_source);

  LogNhcAdmissionSample(nhc_admission_log_file, t, selected_source,
                        admission_kin, cfg, decision_v_b,
                        decision_v_wheel_b, decision_v_v);

  const NhcAdmissionDecision *selected_decision = &decision_v_b;
  switch (selected_source) {
    case NhcAdmissionVelocitySource::kWheelBody:
      selected_decision = &decision_v_wheel_b;
      break;
    case NhcAdmissionVelocitySource::kVehicle:
      selected_decision = &decision_v_v;
      break;
    case NhcAdmissionVelocitySource::kBody:
    default:
      break;
  }

  if (selected_decision->below_forward_speed) {
    if (diag.enabled() && !diag.nhc_skip_warned()) {
      cout << "[Warn] NHC skipped at t=" << t
           << " source=" << NhcAdmissionVelocitySourceName(selected_source)
           << " |vx|=" << std::abs(selected_velocity.x())
           << " below forward-speed threshold="
           << cfg.nhc_disable_below_forward_speed << "\n";
    }
    diag.set_nhc_skip_warned(true);
    return;
  }
  if (selected_decision->exceed_lateral_vertical_limit) {
    if (diag.enabled() && !diag.nhc_skip_warned()) {
      cout << "[Warn] NHC skipped at t=" << t
           << " source=" << NhcAdmissionVelocitySourceName(selected_source)
           << " |vy|=" << std::abs(selected_velocity.y())
           << " |vz|=" << std::abs(selected_velocity.z()) << "\n";
    }
    diag.set_nhc_skip_warned(true);
    return;
  }
  diag.set_nhc_skip_warned(false);

  auto model = MeasModels::ComputeNhcModel(state, C_b_v, omega_ib_b_raw,
                                            cfg.sigma_nhc_y, cfg.sigma_nhc_z,
                                            fej);
  StateMask nhc_update_mask;
  const StateMask *nhc_update_mask_ptr = nullptr;
  if (cfg.debug_nhc_disable_bgz_state_update) {
    nhc_update_mask.fill(true);
    nhc_update_mask[StateIdx::kBg + 2] = false;
    nhc_update_mask_ptr = &nhc_update_mask;
  }
  if (CorrectConstraintWithRobustness(engine, "NHC", t, imu, cfg, model.y, model.H,
                                      model.R, false, true, true,
                                      cfg.nhc_nis_gate_prob, cfg.bgz_gate_apply_to_nhc,
                                      nhc_update_mask_ptr,
                                      diag, stats)) {
    last_nhc_update_t = t;
  }
}

/**
 * 执行 ODO 更新，返回最新 ODO 速度。
 */
double RunOdoUpdate(EskfEngine &engine, const Dataset &dataset,
                    const ConstraintConfig &cfg, const GatingConfig &gating,
                    const Vector3d &mounting_base_rpy, int &odo_idx,
                    double t_curr, const ImuData &imu_curr,
                    DiagnosticsEngine &diag,
                    const FejManager *fej, ConstraintUpdateStats &stats,
                    double &last_odo_update_t, double odo_min_interval) {
  double last_odo_speed = 0.0;
  if (!cfg.enable_odo) return last_odo_speed;

  while (odo_idx < dataset.odo.rows()) {
    double t_odo = dataset.odo(odo_idx, 0) + cfg.odo_time_offset;
    if (t_odo > t_curr + gating.time_tolerance) break;

    double odo_vel = dataset.odo(odo_idx, 1);
    bool interpolated_to_curr = false;

    // 时间插值
    int next = odo_idx + 1;
    if (next < dataset.odo.rows()) {
      double t_next = dataset.odo(next, 0) + cfg.odo_time_offset;
      double dt_odo = t_next - t_odo;
      if (dt_odo > 1e-6 && t_curr >= t_odo && t_curr <= t_next) {
        double alpha = (t_curr - t_odo) / dt_odo;
        odo_vel += alpha * (dataset.odo(next, 1) - odo_vel);
        interpolated_to_curr = true;
      }
    }
    double t_meas = interpolated_to_curr ? t_curr : t_odo;
    if (IsTimeInWindow(t_meas, cfg.debug_odo_disable_start_time,
                       cfg.debug_odo_disable_end_time,
                       gating.time_tolerance)) {
      ++odo_idx;
      continue;
    }
    if (odo_min_interval > 0.0 &&
        last_odo_update_t > -1e17 &&
        (t_meas - last_odo_update_t) < odo_min_interval) {
      ++odo_idx;
      continue;
    }
    last_odo_speed = odo_vel;

    // 角速度（使用当前 IMU 数据）
    const State &state = engine.state();
    if (cfg.disable_odo_when_weak_excitation &&
        MeetsWeakExcitationThresholds(state, imu_curr, cfg)) {
      ++odo_idx;
      continue;
    }
    Vector3d omega_ib_b_raw = Vector3d::Zero();
    if (imu_curr.dt > 1e-9)
      omega_ib_b_raw = imu_curr.dtheta / imu_curr.dt;

    // 动态 C_b_v
    Vector3d rpy(mounting_base_rpy.x(),
                 mounting_base_rpy.y() + state.mounting_pitch,
                 mounting_base_rpy.z() + state.mounting_yaw);
    Matrix3d C_b_v = QuatToRot(RpyToQuat(rpy)).transpose();
    auto model = MeasModels::ComputeOdoModel(state, odo_vel,
                                              C_b_v, omega_ib_b_raw,
                                              cfg.sigma_odo, fej);
    if (cfg.debug_odo_disable_bgz_jacobian &&
        model.H.cols() > StateIdx::kBg + 2) {
      model.H(0, StateIdx::kBg + 2) = 0.0;
    }
    StateMask odo_update_mask;
    const StateMask *odo_update_mask_ptr = nullptr;
    if (cfg.debug_odo_disable_bgz_state_update) {
      odo_update_mask.fill(true);
      odo_update_mask[StateIdx::kBg + 2] = false;
      odo_update_mask_ptr = &odo_update_mask;
    }
    if (CorrectConstraintWithRobustness(engine, "ODO", t_meas, imu_curr, cfg,
                                        model.y, model.H, model.R,
                                        true, true, true, cfg.odo_nis_gate_prob,
                                        cfg.bgz_gate_apply_to_odo,
                                        odo_update_mask_ptr, diag, stats)) {
      last_odo_update_t = t_meas;
    }
    ++odo_idx;
  }
  return last_odo_speed;
}

/**
 * 执行 UWB 更新。
 */
void RunUwbUpdate(EskfEngine &engine, const Dataset &dataset,
                  int &uwb_idx, double t_curr,
                  const FusionOptions &options,
                  bool schedule_active, double split_t,
                  const vector<int> &head_idx, const vector<int> &tail_idx,
                  DiagnosticsEngine &diag) {
  while (uwb_idx < dataset.uwb.rows()) {
    double t_uwb = dataset.uwb(uwb_idx, 0);
    if (t_uwb > t_curr + options.gating.time_tolerance) break;

    int n_meas_full = dataset.uwb.cols() - 1;
    const vector<int> *active = nullptr;
    if (schedule_active)
      active = (t_uwb <= split_t) ? &head_idx : &tail_idx;

    // 提取观测和基站
    VectorXd z;
    MatrixXd anchors;
    if (active) {
      int k = static_cast<int>(active->size());
      z.resize(k);
      anchors.resize(k, 3);
      for (int j = 0; j < k; ++j) {
        int idx = (*active)[j];
        z(j) = dataset.uwb(uwb_idx, 1 + idx);
        anchors.row(j) = dataset.anchors.positions.row(idx);
      }
    } else {
      z = dataset.uwb.block(uwb_idx, 1, 1, n_meas_full).transpose();
      anchors = dataset.anchors.positions;
    }

    if (z.size() == 0 || anchors.rows() == 0) { ++uwb_idx; continue; }

    auto model = MeasModels::ComputeUwbModel(engine.state(), z, anchors,
                                              options.noise.sigma_uwb);

    // 残差门控
    double max_r = model.y.cwiseAbs().maxCoeff();
    if (max_r > options.gating.uwb_residual_max) {
      int a0 = active ? (*active)[0] : 0;
      Vector3d a0_pos = anchors.row(0).transpose();
      cout << "[Warn] Large UWB residual at t=" << fixed << t_uwb
           << " | max_r=" << max_r << "\n"
           << "       State p: " << engine.state().p.transpose() << "\n"
           << "       Anchor " << a0 + 1 << ": " << a0_pos.transpose() << "\n"
           << "       Meas z: " << z.transpose() << "\n"
           << "       Pred h: " << (engine.state().p - a0_pos).norm() << "\n"
           << " -> Skipped\n";
    } else {
      (void)diag.Correct(engine, "UWB", t_uwb, model.y, model.H, model.R);
    }
    ++uwb_idx;
  }
}

bool SplitImuMeasurementAtTimestamp(const ImuData &imu_prev,
                                    const ImuData &imu_curr,
                                    double timestamp,
                                    ImuData &mid_imu,
                                    ImuData &tail_imu) {
  const double interval = imu_curr.t - imu_prev.t;
  if (!(interval > 1.0e-9) ||
      timestamp <= imu_prev.t ||
      timestamp >= imu_curr.t) {
    return false;
  }

  const double lambda = (timestamp - imu_prev.t) / interval;
  if (!(lambda > 0.0 && lambda < 1.0)) {
    return false;
  }

  mid_imu = imu_curr;
  mid_imu.t = timestamp;
  mid_imu.dtheta = imu_curr.dtheta * lambda;
  mid_imu.dvel = imu_curr.dvel * lambda;
  mid_imu.dt = timestamp - imu_prev.t;

  tail_imu = imu_curr;
  tail_imu.dtheta = imu_curr.dtheta - mid_imu.dtheta;
  tail_imu.dvel = imu_curr.dvel - mid_imu.dvel;
  tail_imu.dt = imu_curr.dt - mid_imu.dt;
  return tail_imu.dt > 1.0e-9;
}

/**
 * 执行GNSS位置和速度更新。
 * 该函数假设当前引擎状态已经位于 t_curr，并在该时刻执行 GNSS 更新。
 * 精确的“先插值 IMU 到 GNSS 时刻、再更新、再传播剩余 IMU”时序
 * 由 PredictCurrentIntervalWithExactGnss 在外层调度。
 */
bool RunGnssUpdate(EskfEngine &engine, const Dataset &dataset,
                   int &gnss_idx, double t_curr,
                   const FusionOptions &options,
                   DiagnosticsEngine &diag,
                   const ImuData *imu_curr,
                   const FejManager *fej,
                   double gnss_split_t,
                   FusionDebugCapture *debug_capture,
                   FusionPerfStats *perf_stats) {
  if (dataset.gnss.timestamps.size() == 0) return false;
  constexpr double kSigmaVelMin = 1e-4;
  bool any_gnss_updated = false;
  const int gnss_idx_begin = gnss_idx;
  const SteadyClock::time_point call_start =
      (perf_stats != nullptr && perf_stats->enabled) ? SteadyClock::now()
                                                     : SteadyClock::time_point{};

  while (gnss_idx < (int)dataset.gnss.timestamps.size()) {
    double t_gnss = dataset.gnss.timestamps(gnss_idx);
    if (t_gnss > t_curr + options.gating.time_tolerance) break;
    if (perf_stats != nullptr && perf_stats->enabled) {
      cerr << "[Perf][GNSS] begin idx=" << gnss_idx
           << " t_gnss=" << fixed << setprecision(6) << t_gnss
           << " t_curr=" << t_curr << endl;
    }

    // 时间对齐量：t_curr - t_gnss，通常 ≤ 一个 IMU 步长 (~5ms)
    // 对 RTK FIX（mm级精度）：5m/s × 5ms = 2.5cm 系统偏差，需对齐
    double dt_align = t_curr - t_gnss;

    Vector3d gnss_pos = Vector3d::Zero();
    Vector3d gnss_std = Vector3d::Zero();
    if (!ComputeAlignedGnssPositionMeasurement(dataset, options.noise, gnss_idx,
                                               t_curr, gnss_pos, gnss_std)) {
      if (perf_stats != nullptr && perf_stats->enabled) {
        cerr << "[Perf][GNSS] skip_invalid_measurement idx=" << gnss_idx
             << " t_gnss=" << t_gnss << endl;
      }
      ++gnss_idx;
      continue;
    }
    const bool has_gnss_vel_data = HasGnssVelocityData(dataset);

    // 构建GNSS位置量测模型（各向异性R）
    const SteadyClock::time_point model_start =
        (perf_stats != nullptr && perf_stats->enabled) ? SteadyClock::now()
                                                       : SteadyClock::time_point{};
    auto model = MeasModels::ComputeGnssPositionModel(engine.state(), gnss_pos,
                                                      gnss_std, fej);
    const double model_s =
        (perf_stats != nullptr && perf_stats->enabled)
            ? DurationSeconds(SteadyClock::now() - model_start)
            : 0.0;

    double omega_z_deg_s = 0.0;
    if (imu_curr != nullptr && imu_curr->dt > 1.0e-9) {
      omega_z_deg_s =
          (imu_curr->dtheta.z() / imu_curr->dt - engine.state().bg.z()) *
          (180.0 / 3.14159265358979323846);
    }
    double effective_pos_gain_scale = options.gnss_pos_position_gain_scale;
    double effective_lgy_from_y_gain_scale =
        options.gnss_pos_lgy_from_y_gain_scale;
    if (options.gnss_pos_turn_rate_threshold_deg_s > 0.0) {
      const bool strong_turn =
          std::abs(omega_z_deg_s) >= options.gnss_pos_turn_rate_threshold_deg_s;
      if (strong_turn && omega_z_deg_s > 0.0 &&
          options.gnss_pos_positive_turn_position_gain_scale >= 0.0) {
        effective_pos_gain_scale =
            options.gnss_pos_positive_turn_position_gain_scale;
      } else if (strong_turn && omega_z_deg_s < 0.0 &&
                 options.gnss_pos_negative_turn_position_gain_scale >= 0.0) {
        effective_pos_gain_scale =
            options.gnss_pos_negative_turn_position_gain_scale;
      }
      if (strong_turn && omega_z_deg_s > 0.0 &&
          options.gnss_pos_positive_turn_lgy_from_y_gain_scale >= 0.0) {
        effective_lgy_from_y_gain_scale =
            options.gnss_pos_positive_turn_lgy_from_y_gain_scale;
      } else if (strong_turn && omega_z_deg_s < 0.0 &&
                 options.gnss_pos_negative_turn_lgy_from_y_gain_scale >= 0.0) {
        effective_lgy_from_y_gain_scale =
            options.gnss_pos_negative_turn_lgy_from_y_gain_scale;
      }
    }

    StateGainScale gnss_pos_gain_scale;
    gnss_pos_gain_scale.fill(1.0);
    const StateGainScale *gnss_pos_gain_scale_ptr = nullptr;
    if (std::abs(effective_pos_gain_scale - 1.0) > 1.0e-12) {
      for (int axis = 0; axis < 3; ++axis) {
        gnss_pos_gain_scale[StateIdx::kPos + axis] =
            effective_pos_gain_scale;
      }
      gnss_pos_gain_scale_ptr = &gnss_pos_gain_scale;
    }
    StateMeasurementGainScale gnss_pos_gain_element_scale;
    const StateMeasurementGainScale *gnss_pos_gain_element_scale_ptr = nullptr;
    if ((std::abs(options.gnss_pos_lgx_from_y_gain_scale - 1.0) > 1.0e-12 ||
         std::abs(effective_lgy_from_y_gain_scale - 1.0) > 1.0e-12) &&
        model.y.size() >= 2) {
      gnss_pos_gain_element_scale =
          StateMeasurementGainScale::Ones(kStateDim, model.y.size());
      gnss_pos_gain_element_scale(StateIdx::kGnssLever + 0, 1) =
          options.gnss_pos_lgx_from_y_gain_scale;
      gnss_pos_gain_element_scale(StateIdx::kGnssLever + 1, 1) =
          effective_lgy_from_y_gain_scale;
      gnss_pos_gain_element_scale_ptr = &gnss_pos_gain_element_scale;
    }

    const string effective_gnss_pos_update_mode =
        ComputeEffectiveGnssPosUpdateMode(options, t_gnss);

    // 诊断日志：观测姿态列范数与“若当前直接做 joint update”的预测修正量。
    {
      double h_att_norm = model.H.block<3, 3>(0, StateIdx::kAtt).norm();
      cout << "[GNSS_POS] t=" << fixed << setprecision(3) << t_gnss
           << " mode="
           << ((fej != nullptr && fej->enabled) ? "InEKF" : "ESKF")
           << " | ||H_att||_F=" << setprecision(6) << h_att_norm
           << " | update_mode=" << options.gnss_pos_update_mode
           << " | effective_update_mode=" << effective_gnss_pos_update_mode
           << " | pos_gain_scale=" << options.gnss_pos_position_gain_scale
           << " | effective_pos_gain_scale=" << effective_pos_gain_scale
           << " | lgx_from_y_gain_scale="
           << options.gnss_pos_lgx_from_y_gain_scale
           << " | lgy_from_y_gain_scale="
           << options.gnss_pos_lgy_from_y_gain_scale
           << " | effective_lgy_from_y_gain_scale="
           << effective_lgy_from_y_gain_scale
           << " | omega_z_deg_s=" << omega_z_deg_s;
      cout << "\n";
    }

    bool gnss_pos_updated = false;
    const SteadyClock::time_point pos_update_start =
        (perf_stats != nullptr && perf_stats->enabled) ? SteadyClock::now()
                                                       : SteadyClock::time_point{};
    if (effective_gnss_pos_update_mode == "stage_nonpos_then_pos") {
      const StateMask non_position_mask = BuildGnssPosNonPositionMask();
      const StateMask position_only_mask = BuildGnssPosPositionOnlyMask();
      bool stage1_updated = diag.Correct(engine, "GNSS_POS_STAGE1_NONPOS",
                                         t_gnss, model.y, model.H, model.R,
                                         &non_position_mask, nullptr,
                                         gnss_pos_gain_element_scale_ptr);
      any_gnss_updated = any_gnss_updated || stage1_updated;
      gnss_pos_updated = gnss_pos_updated || stage1_updated;

      auto stage2_model = MeasModels::ComputeGnssPositionModel(
          engine.state(), gnss_pos, gnss_std, fej);
      bool stage2_updated = diag.Correct(engine, "GNSS_POS_STAGE2_POS",
                                         t_gnss, stage2_model.y,
                                         stage2_model.H, stage2_model.R,
                                         &position_only_mask,
                                         gnss_pos_gain_scale_ptr, nullptr);
      any_gnss_updated = any_gnss_updated || stage2_updated;
      gnss_pos_updated = gnss_pos_updated || stage2_updated;
    } else if (effective_gnss_pos_update_mode == "position_only") {
      const StateMask position_only_mask = BuildGnssPosPositionOnlyMask();
      gnss_pos_updated =
          diag.Correct(engine, "GNSS_POS_POS_ONLY", t_gnss, model.y, model.H,
                       model.R, &position_only_mask, gnss_pos_gain_scale_ptr,
                       nullptr);
      any_gnss_updated = any_gnss_updated || gnss_pos_updated;
    } else {
      gnss_pos_updated =
          diag.Correct(engine, "GNSS_POS", t_gnss, model.y, model.H, model.R,
                       nullptr, gnss_pos_gain_scale_ptr,
                       gnss_pos_gain_element_scale_ptr);
      any_gnss_updated = any_gnss_updated || gnss_pos_updated;
    }
    const double pos_update_s =
        (perf_stats != nullptr && perf_stats->enabled)
            ? DurationSeconds(SteadyClock::now() - pos_update_start)
            : 0.0;
    if (gnss_pos_updated) {
      MaybeCaptureGnssSplitDebug(debug_capture, engine, "GNSS_POS", t_gnss,
                                 gnss_split_t, options.gating.time_tolerance);
    }

    // GNSS_VEL 是独立量测链，仍由 enable_gnss_velocity 显式控制。
    double vel_update_s = 0.0;
    if (has_gnss_vel_data && options.enable_gnss_velocity) {
      Vector3d gnss_vel = dataset.gnss.velocities.row(gnss_idx).transpose();
      Vector3d gnss_vel_std = dataset.gnss.vel_std.row(gnss_idx).transpose();

      // 速度内插：在相邻两个GNSS帧之间内插到t_curr（类似ODO做法）
      // alpha = dt_align / (t_next - t_gnss)，通常极小（~0.5%），保持接口一致性
      int next_gnss = gnss_idx + 1;
      if (dt_align > 1e-9 && next_gnss < (int)dataset.gnss.timestamps.size()) {
        double t_next = dataset.gnss.timestamps(next_gnss);
        double dt_gnss = t_next - t_gnss;
        if (dt_gnss > 1e-6 && dt_align < dt_gnss) {
          double alpha = dt_align / dt_gnss;
          Vector3d vel_next = dataset.gnss.velocities.row(next_gnss).transpose();
          gnss_vel += alpha * (vel_next - gnss_vel);
        }
      }

      // 各向sigma下限保护
      double sigma_vel_fallback = 0.5;
      for (int k = 0; k < 3; ++k) {
        if (!std::isfinite(gnss_vel_std(k)) || gnss_vel_std(k) <= 0.0) {
          gnss_vel_std(k) = sigma_vel_fallback;
        }
        if (gnss_vel_std(k) < kSigmaVelMin) {
          gnss_vel_std(k) = kSigmaVelMin;
        }
      }

      // 计算IMU角速度（用于杆臂补偿）
      Vector3d omega_ib_b_raw = Vector3d::Zero();
      if (imu_curr != nullptr && imu_curr->dt > 1e-9) {
        omega_ib_b_raw = imu_curr->dtheta / imu_curr->dt;
      }

      // 构建GNSS速度量测模型（各向异性R）
      auto vel_model = MeasModels::ComputeGnssVelocityModel(
          engine.state(), gnss_vel, omega_ib_b_raw, gnss_vel_std, fej);

      const SteadyClock::time_point vel_update_start =
          (perf_stats != nullptr && perf_stats->enabled)
              ? SteadyClock::now()
              : SteadyClock::time_point{};
      bool gnss_vel_updated =
          diag.Correct(engine, "GNSS_VEL", t_gnss, vel_model.y, vel_model.H,
                       vel_model.R, nullptr);
      any_gnss_updated = any_gnss_updated || gnss_vel_updated;
      if (perf_stats != nullptr && perf_stats->enabled) {
        vel_update_s = DurationSeconds(SteadyClock::now() - vel_update_start);
      }
    }
    if (perf_stats != nullptr && perf_stats->enabled) {
      cerr << "[Perf][GNSS] end idx=" << gnss_idx
           << " t_gnss=" << t_gnss
           << " dt_align=" << dt_align
           << " model_s=" << model_s
           << " pos_update_s=" << pos_update_s
           << " vel_update_s=" << vel_update_s
           << " updated=" << (gnss_pos_updated ? 1 : 0) << endl;
    }

    ++gnss_idx;
  }
  if (perf_stats != nullptr && perf_stats->enabled) {
    perf_stats->gnss_update_s += DurationSeconds(SteadyClock::now() - call_start);
    ++perf_stats->gnss_update_calls;
    if (gnss_idx > gnss_idx_begin) {
      perf_stats->gnss_samples += static_cast<size_t>(gnss_idx - gnss_idx_begin);
    }
  }
  return any_gnss_updated;
}

bool PredictCurrentIntervalWithExactGnss(EskfEngine &engine,
                                         const Dataset &dataset,
                                         int &gnss_idx, double t_curr,
                                         const FusionOptions &options,
                                         DiagnosticsEngine &diag,
                                         const ImuData *imu_curr,
                                         const FejManager *fej,
                                         double gnss_split_t,
                                         FusionDebugCapture *debug_capture,
                                         bool &gnss_updated_out,
                                         FusionPerfStats *perf_stats) {
  gnss_updated_out = false;
  if (dataset.gnss.timestamps.size() == 0) {
    return engine.Predict();
  }
  const SteadyClock::time_point call_start =
      (perf_stats != nullptr && perf_stats->enabled) ? SteadyClock::now()
                                                     : SteadyClock::time_point{};
  if (perf_stats != nullptr && perf_stats->enabled) {
    ++perf_stats->gnss_exact_calls;
  }

  const double tol = options.gating.time_tolerance;
  ImuData seg_prev = engine.prev_imu();
  ImuData seg_curr = engine.curr_imu();
  bool propagated_to_curr = false;
  size_t dummy_predict_counter = 0;
  size_t &split_predict_counter =
      (perf_stats != nullptr) ? perf_stats->split_predict_count
                              : dummy_predict_counter;
  size_t &align_curr_predict_counter =
      (perf_stats != nullptr) ? perf_stats->align_curr_predict_count
                              : dummy_predict_counter;
  size_t &tail_predict_counter =
      (perf_stats != nullptr) ? perf_stats->tail_predict_count
                              : dummy_predict_counter;
  const auto timed_predict =
      [&](const ImuData &imu_prev_local, const ImuData &imu_curr_local,
          size_t &counter, const char *debug_tag) -> bool {
    const SteadyClock::time_point predict_start =
        (perf_stats != nullptr && perf_stats->enabled) ? SteadyClock::now()
                                                       : SteadyClock::time_point{};
    bool ok = engine.PredictWithImuPair(imu_prev_local, imu_curr_local);
    if (perf_stats != nullptr && perf_stats->enabled) {
      perf_stats->predict_s +=
          DurationSeconds(SteadyClock::now() - predict_start);
      ++counter;
    }
    if (ok) {
      diag.LogPredict(debug_tag, engine);
    }
    return ok;
  };

  while (gnss_idx < (int)dataset.gnss.timestamps.size()) {
    const double t_gnss = dataset.gnss.timestamps(gnss_idx);
    if (t_gnss > t_curr + tol) {
      break;
    }
    if (t_gnss < seg_prev.t - tol) {
      if (perf_stats != nullptr && perf_stats->enabled) {
        cerr << "[Perf][GNSS_EXACT] drop_stale idx=" << gnss_idx
             << " t_gnss=" << fixed << setprecision(6) << t_gnss
             << " seg_prev_t=" << seg_prev.t
             << " t_curr=" << t_curr << endl;
      }
      ++gnss_idx;
      continue;
    }
    if (IsTimestampAlignedToReference(t_gnss, seg_prev.t, tol)) {
      if (perf_stats != nullptr && perf_stats->enabled) {
        cerr << "[Perf][GNSS_EXACT] align_prev idx=" << gnss_idx
             << " t_gnss=" << t_gnss
             << " seg_prev_t=" << seg_prev.t << endl;
      }
      gnss_updated_out =
          RunGnssUpdate(engine, dataset, gnss_idx, seg_prev.t, options, diag,
                        &seg_prev, fej, gnss_split_t, debug_capture,
                        perf_stats) ||
          gnss_updated_out;
      continue;
    }
    if (IsTimestampAlignedToReference(t_gnss, seg_curr.t, tol)) {
      if (perf_stats != nullptr && perf_stats->enabled) {
        cerr << "[Perf][GNSS_EXACT] align_curr idx=" << gnss_idx
             << " t_gnss=" << t_gnss
             << " seg_curr_t=" << seg_curr.t << endl;
      }
      if (!propagated_to_curr) {
        if (!timed_predict(seg_prev, seg_curr, align_curr_predict_counter,
                           "align_curr")) {
          return false;
        }
        propagated_to_curr = true;
      }
      gnss_updated_out =
          RunGnssUpdate(engine, dataset, gnss_idx, seg_curr.t, options, diag,
                        imu_curr, fej, gnss_split_t, debug_capture, perf_stats) ||
          gnss_updated_out;
      continue;
    }

    ImuData mid_imu;
    ImuData tail_imu;
    if (!SplitImuMeasurementAtTimestamp(seg_prev, seg_curr, t_gnss,
                                        mid_imu, tail_imu)) {
      if (perf_stats != nullptr && perf_stats->enabled) {
        cerr << "[Perf][GNSS_EXACT] split_failed idx=" << gnss_idx
             << " t_gnss=" << t_gnss
             << " seg_prev_t=" << seg_prev.t
             << " seg_curr_t=" << seg_curr.t << endl;
      }
      break;
    }
    if (perf_stats != nullptr && perf_stats->enabled) {
      cerr << "[Perf][GNSS_EXACT] split idx=" << gnss_idx
           << " t_gnss=" << t_gnss
           << " seg_prev_t=" << seg_prev.t
           << " seg_curr_t=" << seg_curr.t
           << " mid_dt=" << mid_imu.dt
           << " tail_dt=" << tail_imu.dt << endl;
    }
    if (!timed_predict(seg_prev, mid_imu, split_predict_counter, "split")) {
      return false;
    }
    gnss_updated_out =
        RunGnssUpdate(engine, dataset, gnss_idx, mid_imu.t, options, diag,
                      &mid_imu, fej, gnss_split_t, debug_capture, perf_stats) ||
        gnss_updated_out;
    seg_prev = mid_imu;
    seg_curr = tail_imu;
  }

  if (!propagated_to_curr) {
    bool ok = timed_predict(seg_prev, seg_curr, tail_predict_counter, "tail");
    if (perf_stats != nullptr && perf_stats->enabled) {
      perf_stats->gnss_exact_s += DurationSeconds(SteadyClock::now() - call_start);
    }
    return ok;
  }
  if (perf_stats != nullptr && perf_stats->enabled) {
    perf_stats->gnss_exact_s += DurationSeconds(SteadyClock::now() - call_start);
  }
  return true;
}

/** 记录一步融合结果 */
void RecordResult(FusionResult &result, const State &s, double t) {
  result.fused_positions.push_back(s.p);
  result.fused_velocities.push_back(s.v);
  result.fused_quaternions.push_back(s.q);
  result.mounting_roll.push_back(s.mounting_roll);
  result.mounting_pitch.push_back(s.mounting_pitch);
  result.mounting_yaw.push_back(s.mounting_yaw);
  result.odo_scale.push_back(s.odo_scale);
  result.sg.push_back(s.sg);
  result.sa.push_back(s.sa);
  result.ba.push_back(s.ba);
  result.bg.push_back(s.bg);
  result.lever_arm.push_back(s.lever_arm);
  result.gnss_lever_arm.push_back(s.gnss_lever_arm);
  result.time_axis.push_back(t);
}

}  // namespace

bool ComputeAlignedGnssPositionMeasurement(const Dataset &dataset,
                                          const NoiseParams &noise,
                                          int gnss_idx, double t_curr,
                                          Vector3d &gnss_pos_ecef,
                                          Vector3d &gnss_std_out) {
  (void)t_curr;
  if (dataset.gnss.timestamps.size() == 0) {
    return false;
  }
  if (gnss_idx < 0 || gnss_idx >= dataset.gnss.timestamps.size()) {
    return false;
  }

  constexpr double kSigmaPosMin = 1e-4;
  const double sigma_pos_fallback =
      noise.sigma_gnss_pos > 0.0 ? noise.sigma_gnss_pos : 1.0;

  gnss_pos_ecef = dataset.gnss.positions.row(gnss_idx).transpose();
  gnss_std_out = dataset.gnss.std.row(gnss_idx).transpose();
  for (int k = 0; k < 3; ++k) {
    if (!std::isfinite(gnss_std_out(k)) || gnss_std_out(k) <= 0.0) {
      gnss_std_out(k) = sigma_pos_fallback;
    }
    if (gnss_std_out(k) < kSigmaPosMin) {
      gnss_std_out(k) = kSigmaPosMin;
    }
  }
  return true;
}

// ============================================================
// RunFusion — 融合主循环
// ============================================================
FusionResult RunFusion(const FusionOptions &options, const Dataset &dataset,
                       const State &x0,
                       const Matrix<double, kStateDim, kStateDim> &P0,
                       FusionDebugCapture *debug_capture) {
  FusionResult result;
  FusionPerfStats perf_stats;
  perf_stats.enabled = IsPerfDebugEnabledFromEnv();
  if (perf_stats.enabled) {
    perf_stats.progress_stride =
        std::max<size_t>(1, dataset.imu.size() / 100);
    perf_stats.wall_start = SteadyClock::now();
    cerr << "[Perf] enabled progress_stride=" << perf_stats.progress_stride
         << " imu_rows=" << dataset.imu.size()
         << " gnss_rows=" << dataset.gnss.timestamps.size() << endl;
  }
  if (dataset.imu.size() < 2) {
    cout << "error: IMU 数据不足，至少需要两帧增量\n";
    return result;
  }

  // 初始化诊断引擎
  DiagnosticsEngine diag(
      options.constraints,
      options.constraints.enable_diagnostics ||
          options.constraints.enable_mechanism_log ||
          !options.first_update_debug_output_path.empty() ||
          !options.gnss_update_debug_output_path.empty() ||
          !options.predict_debug_output_path.empty());
  diag.Initialize(dataset, options);
  std::ofstream nhc_admission_log_file;
  if (options.constraints.enable_nhc_admission_log) {
    namespace fs = std::filesystem;
    fs::path sol_path(options.output_path);
    fs::path log_path =
        sol_path.parent_path() /
        (sol_path.stem().string() + "_nhc_admission.csv");
    if (!log_path.parent_path().empty()) {
      fs::create_directories(log_path.parent_path());
    }
    nhc_admission_log_file.open(log_path, ios::out | ios::trunc);
    if (nhc_admission_log_file.is_open()) {
      nhc_admission_log_file << fixed << setprecision(9);
      nhc_admission_log_file
          << "t,selected_source,selected_accept,"
          << "accept_v_b,accept_v_wheel_b,accept_v_v,"
          << "below_forward_v_b,below_forward_v_wheel_b,below_forward_v_v,"
          << "exceed_lat_vert_v_b,exceed_lat_vert_v_wheel_b,exceed_lat_vert_v_v,"
          << "forward_speed_threshold,max_abs_v_threshold,"
          << "v_b_x,v_b_y,v_b_z,"
          << "v_wheel_b_x,v_wheel_b_y,v_wheel_b_z,"
          << "v_v_x,v_v_y,v_v_z\n";
    }
  }
  const bool runtime_truth_anchor_has_target =
      options.init.runtime_truth_anchor_position ||
      options.init.runtime_truth_anchor_velocity ||
      options.init.runtime_truth_anchor_attitude;
  const bool runtime_truth_anchor_enabled =
      options.init.runtime_truth_anchor_pva && runtime_truth_anchor_has_target;
  if (runtime_truth_anchor_enabled && dataset.truth.timestamps.size() <= 0) {
    cout << "error: runtime_truth_anchor_pva=true 但真值数据为空\n";
    return result;
  }
  if (options.init.runtime_truth_anchor_pva && !runtime_truth_anchor_has_target) {
    cout << "[Init] WARNING: runtime_truth_anchor_pva=true，但 position/velocity/attitude "
            "全部关闭，已自动禁用 runtime anchor\n";
  }
  int truth_anchor_cursor = 0;

  // 初始化 ESKF 引擎（支持状态消融）
  StateAblationConfig active_ablation = options.ablation;
  ConstraintConfig effective_constraints = options.constraints;
  string last_runtime_phase_label;
  const auto build_active_phase_label = [&](double t_now) {
    string label;
    for (const auto &phase : options.runtime_phases) {
      if (!phase.enabled ||
          !IsTimeInWindow(t_now, phase.start_time, phase.end_time,
                          options.gating.time_tolerance)) {
        continue;
      }
      if (!label.empty()) {
        label += ",";
      }
      label += phase.name;
    }
    return label.empty() ? string("none") : label;
  };
  const auto compute_effective_ablation = [&](double t_now) {
    StateAblationConfig effective = active_ablation;
    for (const auto &phase : options.runtime_phases) {
      if (!phase.enabled ||
          !IsTimeInWindow(t_now, phase.start_time, phase.end_time,
                          options.gating.time_tolerance)) {
        continue;
      }
      effective = MergeAblationConfig(effective, phase.ablation);
    }
    if (IsTimeInWindow(t_now,
                       options.constraints.debug_gnss_lever_arm_disable_start_time,
                       options.constraints.debug_gnss_lever_arm_disable_end_time,
                       options.gating.time_tolerance)) {
      effective.disable_gnss_lever_arm = true;
    }
    if (std::isfinite(options.constraints.debug_mounting_yaw_enable_after_time) &&
        t_now + options.gating.time_tolerance <
            options.constraints.debug_mounting_yaw_enable_after_time) {
      effective.disable_mounting_yaw = true;
    }
    return effective;
  };
  const auto compute_effective_constraints = [&](double t_now) {
    ConstraintConfig effective = options.constraints;
    for (const auto &phase : options.runtime_phases) {
      if (!phase.enabled ||
          !IsTimeInWindow(t_now, phase.start_time, phase.end_time,
                          options.gating.time_tolerance)) {
        continue;
      }
      effective = ApplyRuntimeConstraintOverride(effective, phase.constraints);
    }
    return effective;
  };
  const auto build_effective_noise = [&](double t_now,
                                         const StateAblationConfig &ablation) {
    NoiseParams effective = options.noise;
    for (const auto &phase : options.runtime_phases) {
      if (!phase.enabled ||
          !IsTimeInWindow(t_now, phase.start_time, phase.end_time,
                          options.gating.time_tolerance)) {
        continue;
      }
      ApplyRuntimeNoiseOverride(effective, phase.noise);
    }
    ApplyAblationToNoise(effective, ablation);
    return effective;
  };
  const auto build_covariance_floor = [&](const ConstraintConfig &cfg) {
    CovarianceFloor floor;
    floor.enabled = cfg.enable_covariance_floor;
    floor.pos_var = cfg.p_floor_pos_var;
    floor.vel_var = cfg.p_floor_vel_var;
    floor.att_var = std::pow(cfg.p_floor_att_deg * kDegToRad, 2);
    floor.odo_scale_var = cfg.p_floor_odo_scale_var;
    floor.lever_var = cfg.p_floor_lever_arm_vec;
    floor.mounting_var = std::pow(cfg.p_floor_mounting_deg * kDegToRad, 2);
    floor.bg_var = cfg.p_floor_bg_var;
    return floor;
  };
  StateAblationConfig effective_ablation =
      compute_effective_ablation(dataset.imu.front().t);
  effective_constraints = compute_effective_constraints(dataset.imu.front().t);
  NoiseParams runtime_noise =
      build_effective_noise(dataset.imu.front().t, effective_ablation);
  EskfEngine engine(runtime_noise);
  StateMask state_mask = BuildStateMask(effective_ablation);
  engine.SetStateMask(state_mask);
  CorrectionGuard guard;
  guard.enabled = options.constraints.enforce_extrinsic_bounds;
  guard.odo_scale_min = options.constraints.odo_scale_min;
  guard.odo_scale_max = options.constraints.odo_scale_max;
  guard.max_mounting_roll = options.constraints.max_mounting_roll_deg * kDegToRad;
  guard.max_mounting_pitch = options.constraints.max_mounting_pitch_deg * kDegToRad;
  guard.max_mounting_yaw = options.constraints.max_mounting_yaw_deg * kDegToRad;
  guard.max_lever_arm_norm = options.constraints.max_lever_arm_norm;
  guard.max_odo_scale_step = options.constraints.max_odo_scale_step;
  guard.max_mounting_step = options.constraints.max_mounting_step_deg * kDegToRad;
  guard.max_lever_arm_step = options.constraints.max_lever_arm_step;
  engine.SetCorrectionGuard(guard);
  engine.SetCovarianceFloor(build_covariance_floor(effective_constraints));
  Vector3d cfg_mounting_rpy = options.constraints.imu_mounting_angle * kDegToRad;
  // 安装角基准语义：
  // - legacy: 兼容旧逻辑（init 分量非零时不叠加 constraints 对应分量）
  // - non-legacy: 始终以 constraints 为固定基准，init 对应状态增量
  constexpr double kMountInitEps = 1e-12;
  bool init_pitch_nonzero = std::abs(options.init.mounting_pitch0) > kMountInitEps;
  bool init_yaw_nonzero = std::abs(options.init.mounting_yaw0) > kMountInitEps;
  if (options.init.strict_extrinsic_conflict &&
      ((init_pitch_nonzero && std::abs(cfg_mounting_rpy.y()) > kMountInitEps) ||
       (init_yaw_nonzero && std::abs(cfg_mounting_rpy.z()) > kMountInitEps))) {
    cout << "error: init.mounting_* 与 constraints.imu_mounting_angle 冲突，"
         << "且 strict_extrinsic_conflict=true\n";
    return result;
  }

  Vector3d mounting_base_rpy = cfg_mounting_rpy;
  if (options.init.use_legacy_mounting_base_logic) {
    bool use_cfg_pitch_base = !init_pitch_nonzero;
    bool use_cfg_yaw_base = !init_yaw_nonzero;
    mounting_base_rpy = Vector3d(cfg_mounting_rpy.x(),
                                 use_cfg_pitch_base ? cfg_mounting_rpy.y() : 0.0,
                                 use_cfg_yaw_base ? cfg_mounting_rpy.z() : 0.0);
  }
  cout << "[Init] C_b_v base rpy (deg): roll=" << mounting_base_rpy.x() * kRadToDeg
       << " pitch=" << mounting_base_rpy.y() * kRadToDeg
       << " yaw=" << mounting_base_rpy.z() * kRadToDeg
       << " mode="
       << (options.init.use_legacy_mounting_base_logic ? "legacy" : "constraints_base")
       << "\n";

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
  fej.debug_disable_true_reset_gamma =
      options.fej.debug_disable_true_reset_gamma;
  fej.debug_enable_standard_reset_gamma =
      options.fej.debug_enable_standard_reset_gamma;
  engine.SetFejManager(&fej);
  auto apply_runtime_truth_anchor = [&](double t, const char *stage,
                                        bool gnss_updated) -> bool {
    if (!runtime_truth_anchor_enabled) {
      return true;
    }
    if (options.init.runtime_truth_anchor_gnss_only && !gnss_updated) {
      return true;
    }
    if (!ApplyRuntimeTruthAnchor(engine, dataset.truth, options.init, t,
                                 truth_anchor_cursor)) {
      cout << "error: runtime truth anchor failed at stage=" << stage
           << " t=" << fixed << setprecision(6) << t << "\n";
      return false;
    }
    return true;
  };
  engine.Initialize(x0, P0);
  if (!apply_runtime_truth_anchor(dataset.imu.front().t, "initialize", false)) {
    return result;
  }
  cout << "[Init] InEKF: " << (fej.enabled ? "ON" : "OFF") << "\n";
  cout << "[Init] Runtime truth PVA anchor: "
       << (runtime_truth_anchor_enabled ? "ON" : "OFF") << "\n";
  if (!options.runtime_phases.empty()) {
    cout << "[Init] Runtime phases: " << options.runtime_phases.size()
         << " active_at_start=" << build_active_phase_label(dataset.imu.front().t)
         << "\n";
  }
  if (options.constraints.debug_odo_disable_bgz_jacobian ||
      options.constraints.debug_odo_disable_bgz_state_update ||
      options.constraints.debug_nhc_disable_bgz_state_update ||
      options.constraints.enable_nhc_admission_log ||
      options.constraints.nhc_admission_velocity_source != "v_b" ||
      options.constraints.enable_bgz_observability_gate ||
      options.constraints.enable_bgz_covariance_forgetting ||
      options.constraints.debug_run_odo_before_nhc ||
      options.constraints.disable_nhc_when_weak_excitation ||
      options.constraints.disable_odo_when_weak_excitation ||
      std::isfinite(options.constraints.debug_nhc_disable_start_time) ||
      std::isfinite(options.constraints.debug_nhc_disable_end_time) ||
      std::isfinite(options.constraints.debug_odo_disable_start_time) ||
      std::isfinite(options.constraints.debug_odo_disable_end_time) ||
      std::isfinite(
          options.constraints.debug_gnss_lever_arm_disable_start_time) ||
      std::isfinite(
          options.constraints.debug_gnss_lever_arm_disable_end_time) ||
      std::isfinite(options.constraints.debug_nhc_enable_after_time) ||
      std::isfinite(options.constraints.debug_mounting_yaw_enable_after_time) ||
      std::isfinite(
          options.constraints.debug_reset_bg_z_state_and_cov_after_time) ||
      std::isfinite(
          options.constraints.debug_seed_mount_yaw_bgz_cov_before_first_nhc) ||
      std::isfinite(options.constraints.debug_seed_bg_z_before_first_nhc) ||
  options.constraints.debug_seed_bg_z_att_cov_before_first_nhc.allFinite()) {
    cout << "[Init] Constraint debug toggles: odo_disable_bgz_jacobian="
         << (options.constraints.debug_odo_disable_bgz_jacobian ? "ON" : "OFF")
         << " nhc_admission_source="
         << options.constraints.nhc_admission_velocity_source
         << " nhc_admission_log="
         << (options.constraints.enable_nhc_admission_log ? "ON" : "OFF")
         << " odo_disable_bgz_state_update="
         << (options.constraints.debug_odo_disable_bgz_state_update ? "ON" : "OFF")
         << " nhc_disable_bgz_state_update="
         << (options.constraints.debug_nhc_disable_bgz_state_update ? "ON" : "OFF")
         << " bgz_observability_gate="
         << (options.constraints.enable_bgz_observability_gate ? "ON" : "OFF")
         << " bgz_gate_apply_to_odo="
         << (options.constraints.bgz_gate_apply_to_odo ? "ON" : "OFF")
         << " bgz_gate_apply_to_nhc="
         << (options.constraints.bgz_gate_apply_to_nhc ? "ON" : "OFF")
         << " bgz_gate_forward_speed_min="
         << options.constraints.bgz_gate_forward_speed_min
         << " bgz_gate_yaw_rate_min_deg_s="
         << options.constraints.bgz_gate_yaw_rate_min_deg_s
         << " bgz_gate_lateral_acc_min="
         << options.constraints.bgz_gate_lateral_acc_min
         << " bgz_gate_min_scale="
         << options.constraints.bgz_gate_min_scale
         << " bgz_covariance_forgetting="
         << (options.constraints.enable_bgz_covariance_forgetting ? "ON" : "OFF")
         << " bgz_cov_forgetting_tau_s="
         << options.constraints.bgz_cov_forgetting_tau_s
         << " disable_nhc_when_weak_excitation="
         << (options.constraints.disable_nhc_when_weak_excitation ? "ON" : "OFF")
         << " disable_odo_when_weak_excitation="
         << (options.constraints.disable_odo_when_weak_excitation ? "ON" : "OFF")
         << " odo_before_nhc="
         << (options.constraints.debug_run_odo_before_nhc ? "ON" : "OFF")
         << " nhc_disable_window=["
         << options.constraints.debug_nhc_disable_start_time << ", "
         << options.constraints.debug_nhc_disable_end_time << "]"
         << " odo_disable_window=["
         << options.constraints.debug_odo_disable_start_time << ", "
         << options.constraints.debug_odo_disable_end_time << "]"
         << " gnss_lever_disable_window=["
         << options.constraints.debug_gnss_lever_arm_disable_start_time << ", "
         << options.constraints.debug_gnss_lever_arm_disable_end_time << "]"
         << " nhc_enable_after_time="
         << options.constraints.debug_nhc_enable_after_time
         << " mounting_yaw_enable_after_time="
         << options.constraints.debug_mounting_yaw_enable_after_time
         << " reset_bg_z_state_and_cov_after_time="
         << options.constraints.debug_reset_bg_z_state_and_cov_after_time
         << " reset_bg_z_value="
         << options.constraints.debug_reset_bg_z_value
         << " seed_mount_yaw_bgz_cov_before_first_nhc="
         << options.constraints.debug_seed_mount_yaw_bgz_cov_before_first_nhc
         << " seed_bg_z_before_first_nhc="
         << options.constraints.debug_seed_bg_z_before_first_nhc
         << " seed_bg_z_att_cov_before_first_nhc="
         << options.constraints.debug_seed_bg_z_att_cov_before_first_nhc.transpose()
         << "\n";
  }
  if (runtime_truth_anchor_enabled) {
    cout << "[Init] Runtime truth anchor components: pos="
         << (options.init.runtime_truth_anchor_position ? "ON" : "OFF")
         << " vel=" << (options.init.runtime_truth_anchor_velocity ? "ON" : "OFF")
         << " att=" << (options.init.runtime_truth_anchor_attitude ? "ON" : "OFF")
         << " trigger="
         << (options.init.runtime_truth_anchor_gnss_only ? "gnss_only" : "all_stages")
         << "\n";
  }
  if (fej.enabled) {
    cout << "[Init] RI Jacobian sign consistency check skipped "
         << "(new InEKF H is not expected to be opposite-sign to ESKF)\n";
    cout << "[Init] InEKF mode: "
         << (fej.true_iekf_mode ? "true_iekf" : "hybrid_ri") << "\n";
    cout << "[Init] RI toggles: gnss_pos_p_term="
         << (fej.ri_gnss_pos_use_p_ned_local ? "ON" : "OFF")
         << " g_vel_gyro_mode=" << fej.ri_vel_gyro_noise_mode
         << " inject_pos_inverse="
         << (fej.ri_inject_pos_inverse ? "ON" : "OFF")
         << " debug_force_process=" << fej.debug_force_process_model
         << " debug_force_vel_jac=" << fej.debug_force_vel_jacobian
         << " debug_disable_reset_gamma="
         << (fej.debug_disable_true_reset_gamma ? "ON" : "OFF")
         << " debug_enable_std_reset_gamma="
         << (fej.debug_enable_standard_reset_gamma ? "ON" : "OFF")
         << " reset_floor="
         << (fej.apply_covariance_floor_after_reset ? "ON" : "OFF") << "\n";
  }
  cout << "[Init] Ablation: "
       << "gnss_lever=" << (effective_ablation.disable_gnss_lever_arm ? "OFF" : "ON")
       << " gnss_lever_z="
       << ((effective_ablation.disable_gnss_lever_arm || effective_ablation.disable_gnss_lever_z)
               ? "OFF"
               : "ON")
       << " odo_lever=" << (effective_ablation.disable_odo_lever_arm ? "OFF" : "ON")
       << " odo_scale=" << (effective_ablation.disable_odo_scale ? "OFF" : "ON")
       << " accel_bias=" << (effective_ablation.disable_accel_bias ? "OFF" : "ON")
       << " gyro_bias=" << (effective_ablation.disable_gyro_bias ? "OFF" : "ON")
       << " gyro_scale=" << (effective_ablation.disable_gyro_scale ? "OFF" : "ON")
       << " accel_scale=" << (effective_ablation.disable_accel_scale ? "OFF" : "ON")
       << " mounting=" << (effective_ablation.disable_mounting ? "OFF" : "ON")
       << " mounting_roll="
       << ((effective_ablation.disable_mounting || effective_ablation.disable_mounting_roll)
               ? "OFF"
               : "ON")
       << " mounting_pitch="
       << ((effective_ablation.disable_mounting || effective_ablation.disable_mounting_pitch)
               ? "OFF"
               : "ON")
       << " mounting_yaw="
       << ((effective_ablation.disable_mounting || effective_ablation.disable_mounting_yaw)
               ? "OFF"
               : "ON")
       << "\n";

  // UWB 基站调度
  bool uwb_schedule_active = false;
  double uwb_schedule_split_t = 0.0;
  vector<int> uwb_head_indices, uwb_tail_indices;
  if (options.uwb_anchor_schedule.enabled) {
    if (dataset.uwb.rows() == 0 || dataset.uwb.cols() <= 1 ||
        dataset.anchors.positions.rows() == 0) {
      cout << "[Warn] UWB anchor schedule enabled but no UWB data/anchors\n";
    } else {
      int n_a = dataset.anchors.positions.rows();
      vector<int> hu, tu;
      bool ok = BuildUniqueAnchorIndices(
          options.uwb_anchor_schedule.head_anchors, n_a,
          "uwb_anchor_schedule.head_anchors", hu);
      ok = ok && BuildUniqueAnchorIndices(
          options.uwb_anchor_schedule.tail_anchors, n_a,
          "uwb_anchor_schedule.tail_anchors", tu);
      double t0_uwb = 0.0, t1_uwb = 0.0;
      if (ok && ComputeUwbTimeRange(dataset.uwb, t0_uwb, t1_uwb) && t1_uwb > t0_uwb) {
        uwb_schedule_split_t = t0_uwb + options.uwb_anchor_schedule.head_ratio * (t1_uwb - t0_uwb);
        uwb_head_indices = hu;
        uwb_tail_indices = tu;
        uwb_schedule_active = true;
      } else if (ok) {
        cout << "[Warn] UWB anchor schedule skipped: invalid UWB time range\n";
      }
    }
  }

  // GNSS 时间调度（按融合时间轴比例）
  bool gnss_schedule_active = false;
  double gnss_schedule_split_t = std::numeric_limits<double>::infinity();
  bool gnss_schedule_use_windows = false;
  double gnss_schedule_last_window_end = -std::numeric_limits<double>::infinity();
  if (options.gnss_schedule.enabled) {
    if (dataset.gnss.timestamps.size() <= 0) {
      cout << "[Warn] GNSS schedule enabled but GNSS data is empty\n";
    } else if (dataset.imu.empty() || dataset.imu.back().t <= dataset.imu.front().t) {
      cout << "[Warn] GNSS schedule skipped: invalid IMU time range\n";
    } else if (!options.gnss_schedule.enabled_windows.empty()) {
      gnss_schedule_use_windows = true;
      gnss_schedule_active = true;
      gnss_schedule_last_window_end =
          options.gnss_schedule.enabled_windows.back().second;
      cout << "[GNSS] schedule ON: windows="
           << options.gnss_schedule.enabled_windows.size()
           << " last_window_end=" << gnss_schedule_last_window_end << "\n";
    } else {
      double t0_nav = dataset.imu.front().t;
      double t1_nav = dataset.imu.back().t;
      gnss_schedule_split_t =
          t0_nav + options.gnss_schedule.head_ratio * (t1_nav - t0_nav);
      gnss_schedule_active = true;
      cout << "[GNSS] schedule ON: head_ratio=" << options.gnss_schedule.head_ratio
           << " split_t=" << gnss_schedule_split_t << "\n";
    }
  }
  bool post_gnss_ablation_applied = false;

  // 主循环状态
  double static_duration = 0.0;
  int uwb_idx = 0, odo_idx = 0, gnss_idx = 0;
  double last_odo_speed = 0.0;
  double last_nhc_update_t = -1e18;
  double last_odo_update_t = -1e18;
  bool debug_seed_mount_yaw_bgz_cov_applied = false;
  bool debug_reset_bg_z_state_and_cov_applied = false;
  ConstraintUpdateStats nhc_stats;
  ConstraintUpdateStats odo_stats;
  const auto sync_runtime_controls = [&](double t_now) {
    effective_constraints = compute_effective_constraints(t_now);
    effective_ablation = compute_effective_ablation(t_now);
    NoiseParams effective_noise = build_effective_noise(t_now, effective_ablation);
    engine.SetNoiseParams(effective_noise);
    engine.SetStateMask(BuildStateMask(effective_ablation));
    engine.SetCovarianceFloor(build_covariance_floor(effective_constraints));
    const string phase_label = build_active_phase_label(t_now);
    if (phase_label != last_runtime_phase_label) {
      cout << "[Runtime] phases t=" << fixed << setprecision(3) << t_now
           << " active=" << phase_label
           << " enable_odo=" << (effective_constraints.enable_odo ? "ON" : "OFF")
           << " enable_nhc=" << (effective_constraints.enable_nhc ? "ON" : "OFF")
           << " cov_floor=" << (effective_constraints.enable_covariance_floor ? "ON" : "OFF")
           << " p_floor_odo_scale_var=" << effective_constraints.p_floor_odo_scale_var
           << " p_floor_mounting_deg=" << effective_constraints.p_floor_mounting_deg
           << " nis_gating=" << (effective_constraints.enable_nis_gating ? "ON" : "OFF")
           << " odo_gate_prob=" << effective_constraints.odo_nis_gate_prob
           << " nhc_gate_prob=" << effective_constraints.nhc_nis_gate_prob
           << "\n";
      last_runtime_phase_label = phase_label;
    }
  };

  engine.AddImu(dataset.imu[0]);
  for (size_t i = 1; i < dataset.imu.size(); ++i) {
    engine.AddImu(dataset.imu[i]);
    double t = dataset.imu[i].t;
    double dt = dataset.imu[i].dt;
    sync_runtime_controls(t);

    const FejManager *fej_ptr = fej.enabled ? &fej : nullptr;
    const FejManager *fej_ptr_mut = fej.enabled ? &fej : nullptr;

    bool gnss_enabled_now = true;
    bool gnss_schedule_finished = false;
    if (gnss_schedule_active) {
      if (gnss_schedule_use_windows) {
        gnss_enabled_now = IsTimeInAnyWindow(
            t, options.gnss_schedule.enabled_windows, options.gating.time_tolerance);
        gnss_schedule_finished =
            t > gnss_schedule_last_window_end + options.gating.time_tolerance;
      } else {
        gnss_enabled_now =
            t <= gnss_schedule_split_t + options.gating.time_tolerance;
        gnss_schedule_finished = !gnss_enabled_now;
      }
    }
    if (gnss_schedule_use_windows && !gnss_enabled_now &&
        !gnss_schedule_finished) {
      const int drop_begin = gnss_idx;
      const int dropped = AdvanceTimestampIndexToTime(
          dataset.gnss.timestamps, gnss_idx, t, options.gating.time_tolerance);
      if (dropped > 0) {
        const double first_dropped_t = dataset.gnss.timestamps(drop_begin);
        const double last_dropped_t = dataset.gnss.timestamps(gnss_idx - 1);
        cout << "[GNSS] schedule dropped " << dropped
             << " sample(s) in off-window at t=" << fixed
             << setprecision(3) << t
             << " | sample_range=[" << first_dropped_t << ", "
             << last_dropped_t << "]\n";
      }
    }

    bool gnss_updated = false;
    bool predict_ok = false;
    if (gnss_enabled_now) {
      predict_ok = PredictCurrentIntervalWithExactGnss(
          engine, dataset, gnss_idx, t, options, diag, &dataset.imu[i],
          fej_ptr_mut, gnss_schedule_split_t, debug_capture, gnss_updated,
          &perf_stats);
    } else {
      const SteadyClock::time_point predict_start =
          perf_stats.enabled ? SteadyClock::now() : SteadyClock::time_point{};
      predict_ok = engine.Predict();
      if (perf_stats.enabled) {
        perf_stats.predict_s +=
            DurationSeconds(SteadyClock::now() - predict_start);
        ++perf_stats.direct_predict_count;
      }
      if (predict_ok) {
        diag.LogPredict("direct", engine);
      }
    }
    if (!predict_ok) continue;

    if (!apply_runtime_truth_anchor(t, "predict", false)) {
      break;
    }
    ApplyBgzCovarianceForgettingIfNeeded(engine, dataset.imu[i],
                                         effective_constraints);
    ApplyDebugResetBgzStateAndCov(engine, effective_constraints, t,
                                  options.gating.time_tolerance,
                                  debug_reset_bg_z_state_and_cov_applied);

    const bool nhc_time_gate_open =
        !std::isfinite(effective_constraints.debug_nhc_enable_after_time) ||
        t + options.gating.time_tolerance >=
            effective_constraints.debug_nhc_enable_after_time;

    // 1. ZUPT 更新
    const SteadyClock::time_point zupt_start =
        perf_stats.enabled ? SteadyClock::now() : SteadyClock::time_point{};
    bool zupt_ready = RunZuptUpdate(engine, dataset.imu[i], effective_constraints,
                                    static_duration, diag, t);
    if (perf_stats.enabled) {
      perf_stats.zupt_s += DurationSeconds(SteadyClock::now() - zupt_start);
    }
    if (!apply_runtime_truth_anchor(t, "zupt", false)) {
      break;
    }

    // 2. 重力对准诊断（ZUPT 之后、NHC 之前）
    const SteadyClock::time_point gravity_diag_start =
        perf_stats.enabled ? SteadyClock::now() : SteadyClock::time_point{};
    diag.CheckGravityAlignment(t, dt, dataset.imu[i], engine.state(),
                               dataset.truth);
    if (perf_stats.enabled) {
      perf_stats.gravity_diag_s +=
          DurationSeconds(SteadyClock::now() - gravity_diag_start);
    }

    // 3-4. NHC / ODO 更新（默认 NHC->ODO，可用 research toggle 交换顺序）
    double nhc_min_interval =
        std::max(0.0, effective_constraints.nhc_min_update_interval);
    double odo_min_interval =
        std::max(0.0, effective_constraints.odo_min_update_interval);
    if (!effective_constraints.enable_odo) {
      AdvanceMatrixTimeIndexToTime(dataset.odo, odo_idx, 0,
                                   effective_constraints.odo_time_offset, t,
                                   options.gating.time_tolerance);
    }
    if (effective_constraints.debug_run_odo_before_nhc) {
      last_odo_speed = RunOdoUpdate(engine, dataset, effective_constraints,
                                    options.gating, mounting_base_rpy, odo_idx, t,
                                    dataset.imu[i], diag, fej_ptr, odo_stats,
                                    last_odo_update_t, odo_min_interval);
      if (!apply_runtime_truth_anchor(t, "odo", false)) {
        break;
      }

      ApplyDebugSeedBeforeFirstNhc(
          engine, effective_constraints, debug_seed_mount_yaw_bgz_cov_applied);
      if (nhc_time_gate_open) {
        RunNhcUpdate(engine, dataset.imu[i], effective_constraints,
                     mounting_base_rpy, zupt_ready, diag, t, fej_ptr,
                     nhc_stats, last_nhc_update_t, nhc_min_interval,
                     &nhc_admission_log_file);
      }
      if (!apply_runtime_truth_anchor(t, "nhc", false)) {
        break;
      }
    } else {
      ApplyDebugSeedBeforeFirstNhc(
          engine, effective_constraints, debug_seed_mount_yaw_bgz_cov_applied);
      if (nhc_time_gate_open) {
        RunNhcUpdate(engine, dataset.imu[i], effective_constraints,
                     mounting_base_rpy, zupt_ready, diag, t, fej_ptr,
                     nhc_stats, last_nhc_update_t, nhc_min_interval,
                     &nhc_admission_log_file);
      }
      if (!apply_runtime_truth_anchor(t, "nhc", false)) {
        break;
      }

      last_odo_speed = RunOdoUpdate(engine, dataset, effective_constraints,
                                    options.gating, mounting_base_rpy, odo_idx, t,
                                    dataset.imu[i], diag, fej_ptr, odo_stats,
                                    last_odo_update_t, odo_min_interval);
      if (!apply_runtime_truth_anchor(t, "odo", false)) {
        break;
      }
    }
    // 5. UWB 更新
    const SteadyClock::time_point uwb_start =
        perf_stats.enabled ? SteadyClock::now() : SteadyClock::time_point{};
    RunUwbUpdate(engine, dataset, uwb_idx, t, options,
                 uwb_schedule_active, uwb_schedule_split_t,
                 uwb_head_indices, uwb_tail_indices, diag);
    if (perf_stats.enabled) {
      perf_stats.uwb_s += DurationSeconds(SteadyClock::now() - uwb_start);
    }
    if (!apply_runtime_truth_anchor(t, "uwb", false)) {
      break;
    }

    // 6. GNSS 时间窗结束后的状态冻结切换
    if (gnss_schedule_finished && !post_gnss_ablation_applied) {
      gnss_idx = static_cast<int>(dataset.gnss.timestamps.size());
      if (options.post_gnss_ablation.enabled) {
        active_ablation = MergeAblationConfig(active_ablation,
                                              options.post_gnss_ablation.ablation);
        sync_runtime_controls(t);
        cout << "[GNSS] post-gnss ablation applied at t=" << fixed
             << setprecision(3) << t
             << " | gnss_lever=" << (effective_ablation.disable_gnss_lever_arm ? "OFF" : "ON")
             << " gnss_lever_z="
             << ((effective_ablation.disable_gnss_lever_arm || effective_ablation.disable_gnss_lever_z)
                     ? "OFF"
                     : "ON")
             << " odo_lever=" << (effective_ablation.disable_odo_lever_arm ? "OFF" : "ON")
             << " odo_scale=" << (effective_ablation.disable_odo_scale ? "OFF" : "ON")
             << " accel_bias=" << (effective_ablation.disable_accel_bias ? "OFF" : "ON")
             << " gyro_bias=" << (effective_ablation.disable_gyro_bias ? "OFF" : "ON")
             << " mounting=" << (effective_ablation.disable_mounting ? "OFF" : "ON")
             << " mounting_roll="
             << ((effective_ablation.disable_mounting || effective_ablation.disable_mounting_roll)
                     ? "OFF"
                     : "ON")
             << " mounting_pitch="
             << ((effective_ablation.disable_mounting || effective_ablation.disable_mounting_pitch)
                     ? "OFF"
                     : "ON")
             << " mounting_yaw="
             << ((effective_ablation.disable_mounting || effective_ablation.disable_mounting_yaw)
                     ? "OFF"
                     : "ON")
             << "\n";
      } else {
        cout << "[GNSS] updates disabled after split at t=" << fixed
             << setprecision(3) << t << "\n";
      }
      post_gnss_ablation_applied = true;
    }
    if (!apply_runtime_truth_anchor(t, "gnss", gnss_updated)) {
      break;
    }

    // 7. 状态诊断
    const SteadyClock::time_point step_diag_start =
        perf_stats.enabled ? SteadyClock::now() : SteadyClock::time_point{};
    diag.OnStepComplete(t, dt, engine.state(), dataset.imu[i],
                        zupt_ready, dataset.truth);
    if (perf_stats.enabled) {
      perf_stats.step_diag_s +=
          DurationSeconds(SteadyClock::now() - step_diag_start);
    }

    // 7. 记录结果
    RecordResult(result, engine.state(), t);

    // 8. DIAG.txt 输出
    const SteadyClock::time_point diag_write_start =
        perf_stats.enabled ? SteadyClock::now() : SteadyClock::time_point{};
    diag.WriteDiagLine(t, dt, engine, dataset.imu[i], last_odo_speed,
                       dataset.imu.front().t);
    if (perf_stats.enabled) {
      perf_stats.diag_write_s +=
          DurationSeconds(SteadyClock::now() - diag_write_start);
      ++perf_stats.imu_steps;
      if (((i + 1) % perf_stats.progress_stride) == 0) {
        const double wall_s =
            DurationSeconds(SteadyClock::now() - perf_stats.wall_start);
        cerr << "[Perf] step=" << (i + 1) << "/" << dataset.imu.size()
             << " t=" << fixed << setprecision(3) << t
             << " wall_s=" << wall_s
             << " gnss_idx=" << gnss_idx
             << " exact_calls=" << perf_stats.gnss_exact_calls
             << " gnss_update_calls=" << perf_stats.gnss_update_calls
             << " gnss_samples=" << perf_stats.gnss_samples
             << " predict_s=" << perf_stats.predict_s
             << " gnss_exact_s=" << perf_stats.gnss_exact_s
             << " gnss_update_s=" << perf_stats.gnss_update_s
             << " gravity_diag_s=" << perf_stats.gravity_diag_s
             << " step_diag_s=" << perf_stats.step_diag_s
             << " diag_write_s=" << perf_stats.diag_write_s
             << " split_predict_count=" << perf_stats.split_predict_count
             << " align_curr_predict_count="
             << perf_stats.align_curr_predict_count
             << " tail_predict_count=" << perf_stats.tail_predict_count
             << " direct_predict_count=" << perf_stats.direct_predict_count
             << endl;
      }
    }
  }

  if (options.constraints.enable_consistency_log) {
    PrintConstraintStats("NHC", nhc_stats);
    PrintConstraintStats("ODO", odo_stats);
  }
  if (debug_capture != nullptr && debug_capture->gnss_split_cov.valid) {
    const auto &cov = debug_capture->gnss_split_cov;
    cout << "[GNSS_SPLIT_COV] tag=" << cov.tag
         << " split_t=" << cov.split_t
         << " t_meas=" << cov.t_meas
         << " t_state=" << cov.t_state
         << " P_att_bgz=" << cov.P_att_bgz.transpose()
         << " corr_att_bgz=" << cov.corr_att_bgz.transpose() << "\n";
  }
  if (debug_capture != nullptr && debug_capture->reset_consistency.valid) {
    const auto &reset = debug_capture->reset_consistency;
    cout << "[RESET_SNAPSHOT] tag=" << reset.tag
         << " split_t=" << reset.split_t
         << " t_meas=" << reset.t_meas
         << " t_state=" << reset.t_state
         << " floor_after_reset="
         << (reset.covariance_floor_applied ? "ON" : "OFF") << "\n";
  }
  diag.Finalize(result.time_axis.empty() ? 0.0 : result.time_axis.back());
  if (perf_stats.enabled) {
    const double wall_s =
        DurationSeconds(SteadyClock::now() - perf_stats.wall_start);
    cerr << "[Perf] final"
         << " wall_s=" << wall_s
         << " steps=" << perf_stats.imu_steps
         << " gnss_idx=" << gnss_idx
         << " exact_calls=" << perf_stats.gnss_exact_calls
         << " gnss_update_calls=" << perf_stats.gnss_update_calls
         << " gnss_samples=" << perf_stats.gnss_samples
         << " predict_s=" << perf_stats.predict_s
         << " gnss_exact_s=" << perf_stats.gnss_exact_s
         << " gnss_update_s=" << perf_stats.gnss_update_s
         << " zupt_s=" << perf_stats.zupt_s
         << " gravity_diag_s=" << perf_stats.gravity_diag_s
         << " uwb_s=" << perf_stats.uwb_s
         << " step_diag_s=" << perf_stats.step_diag_s
         << " diag_write_s=" << perf_stats.diag_write_s
         << " split_predict_count=" << perf_stats.split_predict_count
         << " align_curr_predict_count="
         << perf_stats.align_curr_predict_count
         << " tail_predict_count=" << perf_stats.tail_predict_count
         << " direct_predict_count=" << perf_stats.direct_predict_count
         << endl;
  }
  return result;
}
