// 融合主流程：数据加载、ESKF 初始化、量测更新、结果记录
#include "app/fusion.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <deque>

#include "app/diagnostics.h"
#include "io/data_io.h"
#include "utils/math_utils.h"

using namespace std;
using namespace Eigen;

// ============================================================
// 内部辅助函数
// ============================================================
namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

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

bool IsWeakExcitation(const State &state, const ImuData &imu,
                      const ConstraintConfig &cfg) {
  if (!cfg.freeze_extrinsics_when_weak_excitation || imu.dt <= 1e-9) {
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

void ZeroExtrinsicSensitivity(MatrixXd &H, bool freeze_scale, bool freeze_mounting,
                              bool freeze_lever) {
  if (freeze_scale && H.cols() > 21) {
    H.col(21).setZero();
  }
  if (freeze_mounting && H.cols() > 24) {
    H.block(0, 22, H.rows(), 3).setZero();
  }
  if (freeze_lever && H.cols() > 27) {
    H.block(0, 25, H.rows(), 3).setZero();
  }
}

bool CorrectConstraintWithRobustness(EskfEngine &engine, const string &tag, double t,
                                     const ImuData &imu, const ConstraintConfig &cfg,
                                     const VectorXd &y, MatrixXd H, MatrixXd R,
                                     bool freeze_scale, bool freeze_mounting,
                                     bool freeze_lever, double nis_prob,
                                     DiagnosticsEngine &diag,
                                     ConstraintUpdateStats &stats) {
  ++stats.seen;

  if (IsWeakExcitation(engine.state(), imu, cfg)) {
    ZeroExtrinsicSensitivity(H, freeze_scale, freeze_mounting, freeze_lever);
  }

  constexpr double kConstraintNoiseScale = 1.0;
  MatrixXd R_eff = R * kConstraintNoiseScale;

  double nis_pre = 0.0;
  if (!ComputeNis(engine, H, R_eff, y, nis_pre)) {
    ++stats.rejected_numeric;
    return false;
  }

  double robust_w = ComputeRobustWeight(cfg, std::sqrt(std::max(0.0, nis_pre)));
  if (cfg.enable_robust_weighting) {
    R_eff /= (robust_w * robust_w);
  }

  double nis = 0.0;
  if (!ComputeNis(engine, H, R_eff, y, nis)) {
    ++stats.rejected_numeric;
    return false;
  }

  int dof = std::max(1, static_cast<int>(y.size()));
  double gate = ChiSquareQuantile(dof, nis_prob);
  if (cfg.enable_nis_gating && nis > gate) {
    ++stats.rejected_nis;
    if (cfg.enable_consistency_log &&
        (stats.rejected_nis <= 20 || (stats.rejected_nis % 5000) == 0)) {
      cout << "[Consistency] " << tag << " reject t=" << t
           << " NIS=" << nis << " gate=" << gate
           << " weight=" << robust_w << " R_scale=" << kConstraintNoiseScale << "\n";
    }
    return false;
  }

  diag.Correct(engine, tag, t, y, H, R_eff, nullptr);
  ++stats.accepted;
  stats.nis_sum += nis;
  stats.nis_max = std::max(stats.nis_max, nis);
  stats.robust_weight_sum += robust_w;
  stats.noise_scale_sum += kConstraintNoiseScale;
  return true;
}

void PrintConstraintStats(const string &tag, const ConstraintUpdateStats &s) {
  if (s.seen <= 0) return;
  double accept_ratio = static_cast<double>(s.accepted) / static_cast<double>(s.seen);
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

StateMask BuildStateMask(const StateAblationConfig &cfg) {
  StateMask mask;
  mask.fill(true);
  auto disable_range = [&](int start, int len) {
    for (int i = 0; i < len; ++i) {
      mask[start + i] = false;
    }
  };
  if (cfg.disable_gyro_scale) {
    disable_range(15, 3);
  }
  if (cfg.disable_accel_scale) {
    disable_range(18, 3);
  }
  if (cfg.disable_odo_scale) {
    disable_range(21, 1);
  }
  if (cfg.disable_mounting) {
    disable_range(22, 3);
  }
  if (cfg.disable_odo_lever_arm) {
    disable_range(25, 3);
  }
  if (cfg.disable_gnss_lever_arm) {
    disable_range(28, 3);
  }
  return mask;
}

StateAblationConfig MergeAblationConfig(const StateAblationConfig &base,
                                        const StateAblationConfig &extra) {
  StateAblationConfig out = base;
  out.disable_gnss_lever_arm =
      out.disable_gnss_lever_arm || extra.disable_gnss_lever_arm;
  out.disable_odo_lever_arm =
      out.disable_odo_lever_arm || extra.disable_odo_lever_arm;
  out.disable_odo_scale =
      out.disable_odo_scale || extra.disable_odo_scale;
  out.disable_gyro_scale =
      out.disable_gyro_scale || extra.disable_gyro_scale;
  out.disable_accel_scale =
      out.disable_accel_scale || extra.disable_accel_scale;
  out.disable_mounting =
      out.disable_mounting || extra.disable_mounting;
  return out;
}

void ApplyAblationToNoise(NoiseParams &noise, const StateAblationConfig &cfg) {
  if (cfg.disable_gyro_scale) {
    noise.sigma_sg = 0.0;
  }
  if (cfg.disable_accel_scale) {
    noise.sigma_sa = 0.0;
  }
  if (cfg.disable_odo_scale) {
    noise.sigma_odo_scale = 0.0;
  }
  if (cfg.disable_mounting) {
    noise.sigma_mounting = 0.0;
  }
  if (cfg.disable_odo_lever_arm) {
    noise.sigma_lever_arm = 0.0;
  }
  if (cfg.disable_gnss_lever_arm) {
    noise.sigma_gnss_lever_arm = 0.0;
  }
}

// ---------- IMU 数据构建与裁剪 ----------
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

void PushWindowSample(double value_sq, int window_size, deque<double> &buf,
                      double &sum_sq) {
  buf.push_back(value_sq);
  sum_sq += value_sq;
  while (static_cast<int>(buf.size()) > window_size) {
    sum_sq -= buf.front();
    buf.pop_front();
  }
}

void UpdateFejLayer2Status(const ImuData &imu, const State &state,
                           const FejConfig &cfg, FejManager &fej,
                           deque<double> &omega_sq_buf,
                           deque<double> &accel_sq_buf,
                           double &omega_sq_sum,
                           double &accel_sq_sum) {
  if (!fej.enabled || imu.dt <= 1e-9) {
    return;
  }

  Vector3d omega_b = imu.dtheta / imu.dt - state.bg;
  Vector3d f_b = imu.dvel / imu.dt - state.ba;
  double omega_norm = omega_b.norm();
  Llh llh = EcefToLlh(state.p);
  Matrix3d R_ne = RotNedToEcef(llh);
  Matrix3d C_bn = R_ne.transpose() * QuatToRot(state.q);
  Vector3d f_n = C_bn * f_b;
  Vector3d g_n = R_ne.transpose() * GravityEcef(state.p);
  double accel_dev = (f_n + g_n).norm();

  PushWindowSample(omega_norm * omega_norm, cfg.imu_window_size,
                   omega_sq_buf, omega_sq_sum);
  PushWindowSample(accel_dev * accel_dev, cfg.imu_window_size,
                   accel_sq_buf, accel_sq_sum);

  if (omega_sq_buf.empty() || accel_sq_buf.empty()) {
    return;
  }
  double omega_rms = std::sqrt(omega_sq_sum / omega_sq_buf.size());
  double accel_rms = std::sqrt(accel_sq_sum / accel_sq_buf.size());
  fej.UpdateLayer2Flag(omega_rms, accel_rms,
                       cfg.omega_threshold, cfg.accel_threshold,
                       cfg.enable_layer2);
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
    diag.Correct(engine, "ZUPT", t, model.y, model.H, model.R);
    return true;
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
                  double &last_nhc_update_t, double nhc_min_interval) {
  if (!cfg.enable_nhc || (cfg.enable_zupt && zupt_ready)) return;
  if (nhc_min_interval > 0.0 &&
      last_nhc_update_t > -1e17 &&
      (t - last_nhc_update_t) < nhc_min_interval) {
    return;
  }

  double dt = imu.dt;
  if (dt <= 1e-9) return;
  const State &state = engine.state();

  // 体坐标系速度检查
  Matrix3d Cbn = QuatToRot(state.q);
  Vector3d v_b = Cbn.transpose() * state.v;
  if (cfg.nhc_max_abs_v > 0.0 &&
      (std::abs(v_b.y()) > cfg.nhc_max_abs_v ||
       std::abs(v_b.z()) > cfg.nhc_max_abs_v)) {
    if (diag.enabled() && !diag.nhc_skip_warned()) {
      cout << "[Warn] NHC skipped at t=" << t
           << " |v_b.y|=" << std::abs(v_b.y())
           << " |v_b.z|=" << std::abs(v_b.z()) << "\n";
    }
    diag.set_nhc_skip_warned(true);
    return;
  }
  diag.set_nhc_skip_warned(false);

  // 动态计算 C_b_v
  Vector3d mounting_rpy(mounting_base_rpy.x(),
                        mounting_base_rpy.y() + state.mounting_pitch,
                        mounting_base_rpy.z() + state.mounting_yaw);
  Matrix3d C_b_v = QuatToRot(RpyToQuat(mounting_rpy));

  Vector3d omega_ib_b_raw = imu.dtheta / dt;
  auto model = MeasModels::ComputeNhcModel(state, C_b_v, omega_ib_b_raw,
                                            cfg.sigma_nhc_y, cfg.sigma_nhc_z,
                                            fej);
  if (CorrectConstraintWithRobustness(engine, "NHC", t, imu, cfg, model.y, model.H,
                                      model.R, false, true, true,
                                      cfg.nhc_nis_gate_prob, diag, stats)) {
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
    if (odo_min_interval > 0.0 &&
        last_odo_update_t > -1e17 &&
        (t_odo - last_odo_update_t) < odo_min_interval) {
      ++odo_idx;
      continue;
    }

    double odo_vel = dataset.odo(odo_idx, 1);

    // 时间插值
    int next = odo_idx + 1;
    if (next < dataset.odo.rows()) {
      double t_next = dataset.odo(next, 0) + cfg.odo_time_offset;
      double dt_odo = t_next - t_odo;
      if (dt_odo > 1e-6 && t_curr >= t_odo && t_curr <= t_next) {
        double alpha = (t_curr - t_odo) / dt_odo;
        odo_vel += alpha * (dataset.odo(next, 1) - odo_vel);
      }
    }
    last_odo_speed = odo_vel;

    // 角速度（使用当前 IMU 数据）
    const State &state = engine.state();
    Vector3d omega_ib_b_raw = Vector3d::Zero();
    if (imu_curr.dt > 1e-9)
      omega_ib_b_raw = imu_curr.dtheta / imu_curr.dt;

    // 动态 C_b_v
    Vector3d rpy(mounting_base_rpy.x(),
                 mounting_base_rpy.y() + state.mounting_pitch,
                 mounting_base_rpy.z() + state.mounting_yaw);
    Matrix3d C_b_v = QuatToRot(RpyToQuat(rpy));
    auto model = MeasModels::ComputeOdoModel(state, odo_vel,
                                              C_b_v, omega_ib_b_raw,
                                              cfg.sigma_odo, fej);
    if (CorrectConstraintWithRobustness(engine, "ODO", t_odo, imu_curr, cfg,
                                        model.y, model.H, model.R,
                                        true, true, true, cfg.odo_nis_gate_prob,
                                        diag, stats)) {
      last_odo_update_t = t_odo;
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
      diag.Correct(engine, "UWB", t_uwb, model.y, model.H, model.R);
    }
    ++uwb_idx;
  }
}

/**
 * 执行GNSS位置和速度更新。
 * 位置：用GNSS速度对量测做时间外推对齐（z_aligned = z(t_gnss) + v * Δt，Δt = t_curr - t_gnss）
 * 速度：在相邻两个GNSS帧之间线性内插到t_curr（类似ODO处理方式）
 * 噪声：位置和速度均使用各向异性R矩阵（NED各向分别的sigma）
 */
void RunGnssUpdate(EskfEngine &engine, const Dataset &dataset,
                   int &gnss_idx, double t_curr,
                   const FusionOptions &options,
                   DiagnosticsEngine &diag,
                   const ImuData *imu_curr,
                   FejManager *fej) {
  if (dataset.gnss.timestamps.size() == 0) return;
  constexpr double kSigmaPosMin = 1e-4;
  constexpr double kSigmaVelMin = 1e-4;

  while (gnss_idx < (int)dataset.gnss.timestamps.size()) {
    double t_gnss = dataset.gnss.timestamps(gnss_idx);
    if (t_gnss > t_curr + options.gating.time_tolerance) break;

    // 时间对齐量：t_curr - t_gnss，通常 ≤ 一个 IMU 步长 (~5ms)
    // 对 RTK FIX（mm级精度）：5m/s × 5ms = 2.5cm 系统偏差，需对齐
    double dt_align = t_curr - t_gnss;

    // 获取GNSS位置和各向精度（NED sigma，单位 m）
    Vector3d gnss_pos = dataset.gnss.positions.row(gnss_idx).transpose();
    Vector3d gnss_std = dataset.gnss.std.row(gnss_idx).transpose();

    // GNSS 各向 sigma 处理：
    // 1) 优先使用数据文件中的逐历元标准差；
    // 2) 若文件值缺失/非法，回退到配置 sigma_gnss_pos；
    // 3) 统一施加数值稳定下限，避免 R 退化。
    double sigma_pos_fallback = options.noise.sigma_gnss_pos > 0.0
                                    ? options.noise.sigma_gnss_pos
                                    : 1.0;
    for (int k = 0; k < 3; ++k) {
      if (!std::isfinite(gnss_std(k)) || gnss_std(k) <= 0.0) {
        gnss_std(k) = sigma_pos_fallback;
      }
      if (gnss_std(k) < kSigmaPosMin) {
        gnss_std(k) = kSigmaPosMin;
      }
    }

    bool has_vel =
        (dataset.gnss.velocities.rows() == dataset.gnss.timestamps.size() &&
         dataset.gnss.vel_std.rows() == dataset.gnss.timestamps.size());

    // 位置时间外推对齐：z_aligned = gnss_pos(t_gnss) + v_ecef * dt_align
    if (dt_align > 1e-9 && has_vel) {
      Vector3d vel_ecef = dataset.gnss.velocities.row(gnss_idx).transpose();
      gnss_pos += vel_ecef * dt_align;
    }

    // 构建GNSS位置量测模型（各向异性R）
    auto model = MeasModels::ComputeGnssPositionModel(engine.state(), gnss_pos,
                                                      gnss_std, fej);

    diag.Correct(engine, "GNSS_POS", t_gnss, model.y, model.H, model.R, nullptr);

    if (fej != nullptr && fej->enabled && !fej->initialized) {
      fej->Initialize(engine.state());
      cout << "[FEJ] Initialized at t=" << fixed << setprecision(3)
           << t_gnss << "\n";
    }

    // 如果存在GNSS速度数据，执行速度更新
    if (has_vel) {
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

      diag.Correct(engine, "GNSS_VEL", t_gnss, vel_model.y, vel_model.H,
                   vel_model.R, nullptr);
    }

    ++gnss_idx;
  }
}

/** 记录一步融合结果 */
void RecordResult(FusionResult &result, const State &s, double t) {
  result.fused_positions.push_back(s.p);
  result.fused_velocities.push_back(s.v);
  result.fused_quaternions.push_back(s.q);
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

// ============================================================
// LoadDataset
// ============================================================
Dataset LoadDataset(const FusionOptions &options) {
  Dataset data;

  // 读取各类数据
  MatrixXd imu_mat = io::LoadMatrix(options.imu_path, 7);

  MatrixXd uwb_mat;
  if (options.anchors.mode == "fixed" && options.anchors.positions.empty()) {
    uwb_mat.resize(0, 0);
  } else {
    int uwb_cols = 5;
    if (options.anchors.mode == "fixed" && !options.anchors.positions.empty())
      uwb_cols = 1 + static_cast<int>(options.anchors.positions.size());
    uwb_mat = io::LoadMatrix(options.uwb_path, uwb_cols);
  }

  MatrixXd truth_raw = io::LoadMatrix(options.pos_path, 10);

  MatrixXd odo_mat;
  if (!options.odo_path.empty())
    odo_mat = io::LoadMatrix(options.odo_path, 2);

  data.imu = BuildImuSequence(imu_mat);
  data.uwb = uwb_mat;
  data.odo = odo_mat;

  // 填充真值（NED 欧拉角 → ECEF 四元数）
  int n_truth = truth_raw.rows();
  data.truth.timestamps = truth_raw.col(0);
  data.truth.positions = truth_raw.block(0, 1, n_truth, 3);
  data.truth.velocities = truth_raw.block(0, 4, n_truth, 3);
  data.truth.quaternions.resize(n_truth, 4);

  // [修复] 自动检测真值坐标格式：LLA（纬度/经度 < 200）vs ECEF（百万米级）
  bool truth_is_lla = (n_truth > 0 &&
      data.truth.positions.row(0).head<2>().cwiseAbs().maxCoeff() < 200.0);
  if (truth_is_lla) {
    cout << "[Load] Detected LLA/NED format in truth data, converting to ECEF...\n";
  }

  for (int i = 0; i < n_truth; ++i) {
    double lat, lon;
    if (truth_is_lla) {
      // 真值为 LLA(deg) + NED(m/s) 格式，需要转换为 ECEF
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
      // 真值已是 ECEF 格式，从 ECEF 反算 lat/lon 用于姿态计算
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

  // 基站
  data.anchors = BuildAnchors(options.anchors, data.truth.positions);

  // 时间裁剪
  double t0 = options.start_time, t1 = options.final_time;

  // [修复] 使用真值初始化时，确保 start_time >= 真值起始时间
  // 否则 IMU 会在初始化位置（真值首帧）之前开始传播，导致 GNSS 残差巨大
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

  // 裁剪真值
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
  cout << "[Load] Truth rows raw: " << n_truth << ", filtered: " << data.truth.timestamps.size() << "\n";
  if (data.truth.timestamps.size() > 0) {
    cout << "[Load] Truth start t=" << data.truth.timestamps(0) << "\n";
    cout << "[Load] Truth start P=" << data.truth.positions.row(0) << "\n";
  } else {
    cout << "[Load] Truth empty after filtering!\n";
  }

  cout << "[Load] IMU pre-rotation: SKIPPED (IMU_converted already flipped y/z)\n";

  // 加载GNSS数据 (如果存在)
  if (!options.gnss_path.empty()) {
    // 检测GNSS文件列数 (13列=带速度, 7列=仅位置)，避免先读13列导致误报
    int gnss_cols = DetectNumericColumnCount(options.gnss_path);
    if (gnss_cols > 0 && gnss_cols < 7) {
      cout << "error: GNSS 文件列数不足（至少需要7列）: " << options.gnss_path << "\n";
    }
    bool has_velocity = (gnss_cols >= 13);

    MatrixXd gnss_mat = io::LoadMatrix(options.gnss_path, has_velocity ? 13 : 7);
    if (gnss_mat.rows() > 0) {
      data.gnss.timestamps = gnss_mat.col(0);
      data.gnss.std = gnss_mat.block(0, 4, gnss_mat.rows(), 3);

      // 加载速度数据 (如果存在)
      if (has_velocity && gnss_mat.cols() >= 13) {
        data.gnss.velocities = gnss_mat.block(0, 7, gnss_mat.rows(), 3);
        data.gnss.vel_std = gnss_mat.block(0, 10, gnss_mat.rows(), 3);
      }

      // [修复] 自动检测 GNSS 坐标格式并转换为 ECEF
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

          // 转换速度从NED到ECEF
          // GNSS_converted.txt 已在 convert_data4.py 中完成 NEU→NED 转换 (vd = -vu)
          if (has_velocity && !data.gnss.velocities.isZero()) {
            double vn = data.gnss.velocities(i, 0);  // North
            double ve = data.gnss.velocities(i, 1);  // East
            double vd = data.gnss.velocities(i, 2);  // Down (已由转换脚本处理)
            Matrix3d R_ne = RotNedToEcef(lat_rad, lon_rad);
            Vector3d v_ned(vn, ve, vd);
            gnss_vel_ecef.row(i) = (R_ne * v_ned).transpose();
          }
        }
        data.gnss.positions = gnss_ecef;
        if (has_velocity && !data.gnss.velocities.isZero()) {
          data.gnss.velocities = gnss_vel_ecef;
        }
        cout << "[Load] GNSS data: " << gnss_mat.rows() << " records (converted LLA->ECEF)";
        if (has_velocity && !data.gnss.velocities.isZero()) {
          cout << " with velocity";
        }
        cout << "\n";
      } else {
        data.gnss.positions = gnss_mat.block(0, 1, gnss_mat.rows(), 3);
        cout << "[Load] GNSS data: " << gnss_mat.rows() << " records (already ECEF)\n";
      }

      // [修复] 按 IMU 时间范围裁剪 GNSS 数据，避免将早期历史量测应用到初始状态
      double t_imu_start = data.imu.empty() ? t0 : data.imu.front().t;
      double t_imu_end = data.imu.empty() ? t1 : data.imu.back().t;
      double tol = options.gating.time_tolerance;
      vector<int> gnss_keep;
      gnss_keep.reserve(data.gnss.timestamps.size());
      for (int i = 0; i < data.gnss.timestamps.size(); ++i) {
        double tg = data.gnss.timestamps(i);
        if (tg + tol >= t_imu_start && tg - tol <= t_imu_end)
          gnss_keep.push_back(i);
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
  }

  return data;
}

// ============================================================
// RunFusion — 融合主循环
// ============================================================
FusionResult RunFusion(const FusionOptions &options, const Dataset &dataset,
                       const State &x0, const Matrix<double, kStateDim, kStateDim> &P0) {
  FusionResult result;
  if (dataset.imu.size() < 2) {
    cout << "error: IMU 数据不足，至少需要两帧增量\n";
    return result;
  }

  // 初始化诊断引擎
  DiagnosticsEngine diag(options.constraints, options.constraints.enable_diagnostics);
  diag.Initialize(dataset, options);

  // 初始化 ESKF 引擎（支持状态消融）
  StateAblationConfig active_ablation = options.ablation;
  NoiseParams runtime_noise = options.noise;
  ApplyAblationToNoise(runtime_noise, active_ablation);
  EskfEngine engine(runtime_noise);
  StateMask state_mask = BuildStateMask(active_ablation);
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
  CovarianceFloor covariance_floor;
  covariance_floor.enabled = options.constraints.enable_covariance_floor;
  covariance_floor.pos_var = options.constraints.p_floor_pos_var;
  covariance_floor.vel_var = options.constraints.p_floor_vel_var;
  covariance_floor.att_var =
      std::pow(options.constraints.p_floor_att_deg * kDegToRad, 2);
  covariance_floor.mounting_var =
      std::pow(options.constraints.p_floor_mounting_deg * kDegToRad, 2);
  covariance_floor.bg_var = options.constraints.p_floor_bg_var;
  engine.SetCovarianceFloor(covariance_floor);
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
  engine.Initialize(x0, P0);

  FejManager fej;
  fej.Enable(options.fej.enable);
  engine.SetFejManager(fej.enabled ? &fej : nullptr);
  cout << "[Init] FEJ-ESKF: " << (fej.enabled ? "ON" : "OFF") << "\n";
  cout << "[Init] Ablation: "
       << "gnss_lever=" << (active_ablation.disable_gnss_lever_arm ? "OFF" : "ON")
       << " odo_lever=" << (active_ablation.disable_odo_lever_arm ? "OFF" : "ON")
       << " odo_scale=" << (active_ablation.disable_odo_scale ? "OFF" : "ON")
       << " gyro_scale=" << (active_ablation.disable_gyro_scale ? "OFF" : "ON")
       << " accel_scale=" << (active_ablation.disable_accel_scale ? "OFF" : "ON")
       << " mounting=" << (active_ablation.disable_mounting ? "OFF" : "ON")
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
  if (options.gnss_schedule.enabled) {
    if (dataset.gnss.timestamps.size() <= 0) {
      cout << "[Warn] GNSS schedule enabled but GNSS data is empty\n";
    } else if (dataset.imu.empty() || dataset.imu.back().t <= dataset.imu.front().t) {
      cout << "[Warn] GNSS schedule skipped: invalid IMU time range\n";
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
  ConstraintUpdateStats nhc_stats;
  ConstraintUpdateStats odo_stats;
  deque<double> fej_omega_sq_buf;
  deque<double> fej_accel_sq_buf;
  double fej_omega_sq_sum = 0.0;
  double fej_accel_sq_sum = 0.0;

  engine.AddImu(dataset.imu[0]);
  for (size_t i = 1; i < dataset.imu.size(); ++i) {
    engine.AddImu(dataset.imu[i]);
    if (!engine.Predict()) continue;

    double t = dataset.imu[i].t;
    double dt = dataset.imu[i].dt;

    UpdateFejLayer2Status(dataset.imu[i], engine.state(), options.fej, fej,
                          fej_omega_sq_buf, fej_accel_sq_buf,
                          fej_omega_sq_sum, fej_accel_sq_sum);

    const FejManager *fej_ptr = fej.enabled ? &fej : nullptr;
    FejManager *fej_ptr_mut = fej.enabled ? &fej : nullptr;

    // 1. ZUPT 更新
    bool zupt_ready = RunZuptUpdate(engine, dataset.imu[i], options.constraints,
                                     static_duration, diag, t);

    // 2. 重力对准诊断（ZUPT 之后、NHC 之前）
    diag.CheckGravityAlignment(t, dt, dataset.imu[i], engine.state(), dataset.truth);

    // 3. NHC 更新
    double nhc_min_interval = std::max(0.0, options.constraints.nhc_min_update_interval);
    RunNhcUpdate(engine, dataset.imu[i], options.constraints, mounting_base_rpy,
                 zupt_ready, diag, t, fej_ptr, nhc_stats, last_nhc_update_t,
                 nhc_min_interval);

    // 4. ODO 更新
    double odo_min_interval = std::max(0.0, options.constraints.odo_min_update_interval);
    last_odo_speed = RunOdoUpdate(engine, dataset, options.constraints, options.gating,
                                    mounting_base_rpy, odo_idx, t, dataset.imu[i], diag,
                                    fej_ptr, odo_stats, last_odo_update_t,
                                    odo_min_interval);

    // 5. UWB 更新
    RunUwbUpdate(engine, dataset, uwb_idx, t, options,
                 uwb_schedule_active, uwb_schedule_split_t,
                 uwb_head_indices, uwb_tail_indices, diag);

    // 6. GNSS 位置更新（可按时间窗关闭，并在关闭后切换状态冻结）
    const ImuData &imu_curr_ref = dataset.imu[i];
    bool gnss_enabled_now = true;
    if (gnss_schedule_active &&
        t > gnss_schedule_split_t + options.gating.time_tolerance) {
      gnss_enabled_now = false;
    }
    if (!gnss_enabled_now && !post_gnss_ablation_applied) {
      gnss_idx = static_cast<int>(dataset.gnss.timestamps.size());
      if (options.post_gnss_ablation.enabled) {
        active_ablation = MergeAblationConfig(active_ablation,
                                              options.post_gnss_ablation.ablation);
        engine.SetStateMask(BuildStateMask(active_ablation));
        cout << "[GNSS] post-gnss ablation applied at t=" << fixed
             << setprecision(3) << t
             << " | gnss_lever=" << (active_ablation.disable_gnss_lever_arm ? "OFF" : "ON")
             << " odo_lever=" << (active_ablation.disable_odo_lever_arm ? "OFF" : "ON")
             << " odo_scale=" << (active_ablation.disable_odo_scale ? "OFF" : "ON")
             << " mounting=" << (active_ablation.disable_mounting ? "OFF" : "ON")
             << "\n";
      } else {
        cout << "[GNSS] updates disabled after split at t=" << fixed
             << setprecision(3) << t << "\n";
      }
      post_gnss_ablation_applied = true;
    }
    if (gnss_enabled_now) {
      RunGnssUpdate(engine, dataset, gnss_idx, t, options, diag, &imu_curr_ref,
                    fej_ptr_mut);
    }

    // 7. 状态诊断
    diag.OnStepComplete(t, dt, engine.state(), dataset.imu[i],
                        zupt_ready, dataset.truth);

    // 7. 记录结果
    RecordResult(result, engine.state(), t);

    // 8. DIAG.txt 输出
    diag.WriteDiagLine(t, dt, engine, dataset.imu[i], last_odo_speed,
                        dataset.imu.front().t);
  }

  if (options.constraints.enable_consistency_log) {
    PrintConstraintStats("NHC", nhc_stats);
    PrintConstraintStats("ODO", odo_stats);
  }
  diag.Finalize(result.time_axis.empty() ? 0.0 : result.time_axis.back());
  return result;
}
