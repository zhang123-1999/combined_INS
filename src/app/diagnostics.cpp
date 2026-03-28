// 融合诊断引擎实现：量测日志、重力对准、漂移/发散检测、DIAG.txt 输出
#include "app/diagnostics.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

#include "app/fusion.h"
#include "core/eskf.h"
#include "utils/math_utils.h"

using namespace std;
using namespace Eigen;
namespace fs = std::filesystem;

// ============================================================
// 内部辅助结构体与函数
// ============================================================
namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

string VectorToCsvField(const VectorXd &vec) {
  ostringstream oss;
  oss << "[";
  for (Eigen::Index i = 0; i < vec.size(); ++i) {
    if (i > 0) oss << ";";
    oss << setprecision(12) << vec(i);
  }
  oss << "]";
  return oss.str();
}

string MatrixToCsvField(const MatrixXd &mat) {
  ostringstream oss;
  oss << "[";
  for (Eigen::Index r = 0; r < mat.rows(); ++r) {
    for (Eigen::Index c = 0; c < mat.cols(); ++c) {
      if (r > 0 || c > 0) oss << ";";
      oss << setprecision(12) << mat(r, c);
    }
  }
  oss << "]";
  return oss.str();
}

// ---------- ZUPT 静止检测（仅 IMU，用于诊断） ----------
bool DetectZuptImuOnly(const ImuData &imu, const ConstraintConfig &config) {
  if (imu.dt <= 1e-9) return false;
  Vector3d omega = imu.dtheta / imu.dt;
  Vector3d f_b = imu.dvel / imu.dt;
  double acc_diff = std::abs(f_b.norm() - 9.81);
  return (omega.norm() < config.zupt_max_gyro && acc_diff < config.zupt_max_acc);
}

// ---------- 重力对准角度计算 ----------
bool ComputeGravityAlignmentAngle(const Vector3d &f_b_avg, const Vector4d &q_nb,
                                   const Vector3d &p_ecef, double &angle_deg_out) {
  Matrix3d Cbn = QuatToRot(q_nb);
  Vector3d f_n = Cbn * f_b_avg;
  Vector3d g_e = GravityEcef(p_ecef);
  double f_norm = f_n.norm(), g_norm = g_e.norm();
  if (f_norm <= 1e-6 || g_norm <= 1e-6) return false;
  double cos_ang = Clamp(f_n.normalized().dot((-g_e).normalized()), -1.0, 1.0);
  angle_deg_out = acos(cos_ang) * kRadToDeg;
  return true;
}

// ---------- 最近真值索引查找 ----------
int FindNearestTruthIndex(const VectorXd &t_truth, double t, int &cursor) {
  while (cursor + 1 < t_truth.size() && t_truth[cursor + 1] <= t) ++cursor;
  int idx = cursor;
  if (cursor + 1 < t_truth.size()) {
    if (std::abs(t_truth[cursor + 1] - t) < std::abs(t - t_truth[cursor]))
      idx = cursor + 1;
  }
  return idx;
}

// ---------- 量测更新日志条目 ----------
struct MeasUpdateLog {
  string tag;
  double t;
  VectorXd y;
  VectorXd dx;
};

void LogMeasUpdate(const string &tag, double t, const VectorXd &y, const VectorXd &dx) {
  cout << "[Diag] " << tag << " update t=" << t
       << " y=" << y.transpose() << " y_norm=" << y.norm();
  if (dx.size() == kStateDim) {
    cout << " dp=" << dx.segment<3>(StateIdx::kPos).transpose()
         << " dv=" << dx.segment<3>(StateIdx::kVel).transpose()
         << " dtheta=" << dx.segment<3>(StateIdx::kAtt).transpose()
         << " dba=" << dx.segment<3>(StateIdx::kBa).transpose()
         << " dbg=" << dx.segment<3>(StateIdx::kBg).transpose();
  } else {
    cout << " dx=" << dx.transpose();
  }
  cout << " dx_norm=" << dx.norm() << "\n";
}

// ---------- 量测日志频率控制 ----------
struct MeasLogLimiter {
  size_t stride = 1;
  size_t max_total = 0;
  size_t logged_total = 0;
  size_t seen_nhc = 0, seen_zupt = 0, seen_uwb = 0, seen_odo = 0;
  bool cap_reported = false;
};

bool TagMatches(const string &tag, const string &key) {
  return tag == key || tag.find(key) != string::npos;
}

bool ShouldLogMeasUpdate(MeasLogLimiter &lim, const string &tag) {
  size_t *seen = nullptr;
  if (TagMatches(tag, "NHC")) seen = &lim.seen_nhc;
  else if (TagMatches(tag, "ZUPT")) seen = &lim.seen_zupt;
  else if (TagMatches(tag, "UWB")) seen = &lim.seen_uwb;
  else if (TagMatches(tag, "ODO")) seen = &lim.seen_odo;

  if (seen) {
    ++(*seen);
    if (lim.stride > 1 && ((*seen - 1) % lim.stride) != 0) return false;
  }
  if (lim.max_total > 0 && lim.logged_total >= lim.max_total) {
    if (!lim.cap_reported) {
      cout << "[Diag] Measurement update logs capped at " << lim.max_total << " entries\n";
      lim.cap_reported = true;
    }
    return false;
  }
  ++lim.logged_total;
  return true;
}

double SafeQuadraticInfo(const MatrixXd &A, const VectorXd &h) {
  if (A.rows() != A.cols() || A.rows() != h.size() || h.size() == 0) {
    return numeric_limits<double>::quiet_NaN();
  }
  LDLT<MatrixXd> ldlt(A);
  if (ldlt.info() != Success) {
    return numeric_limits<double>::quiet_NaN();
  }
  VectorXd solved = ldlt.solve(h);
  if (ldlt.info() != Success || !solved.allFinite()) {
    return numeric_limits<double>::quiet_NaN();
  }
  return h.dot(solved);
}

double SafeTraceInfo(const MatrixXd &A, const MatrixXd &H) {
  if (A.rows() != A.cols() || A.rows() != H.rows() || H.size() == 0) {
    return numeric_limits<double>::quiet_NaN();
  }
  LDLT<MatrixXd> ldlt(A);
  if (ldlt.info() != Success) {
    return numeric_limits<double>::quiet_NaN();
  }
  MatrixXd solved = ldlt.solve(H);
  if (ldlt.info() != Success || !solved.allFinite()) {
    return numeric_limits<double>::quiet_NaN();
  }
  return (H.array() * solved.array()).sum();
}

double SafeNisFromInnovation(const MatrixXd &S, const VectorXd &y) {
  return SafeQuadraticInfo(S, y);
}

double SafeCovValue(const MatrixXd &P, int row, int col) {
  if (row < 0 || col < 0 || row >= P.rows() || col >= P.cols()) {
    return numeric_limits<double>::quiet_NaN();
  }
  double value = P(row, col);
  return std::isfinite(value) ? value : numeric_limits<double>::quiet_NaN();
}

double SafeCorrValue(const MatrixXd &P, int row, int col) {
  if (row < 0 || col < 0 || row >= P.rows() || col >= P.cols()) {
    return numeric_limits<double>::quiet_NaN();
  }
  double var_row = P(row, row);
  double var_col = P(col, col);
  if (!std::isfinite(var_row) || !std::isfinite(var_col) ||
      var_row <= 1e-30 || var_col <= 1e-30) {
    return numeric_limits<double>::quiet_NaN();
  }
  double cov = P(row, col);
  if (!std::isfinite(cov)) {
    return numeric_limits<double>::quiet_NaN();
  }
  return cov / std::sqrt(var_row * var_col);
}

VectorXd SafeMeasurementNumerator(const MatrixXd &P_prior, const MatrixXd &H,
                                  int state_idx) {
  if (state_idx < 0 || state_idx >= P_prior.rows() ||
      P_prior.rows() != P_prior.cols() || H.cols() != P_prior.cols()) {
    return VectorXd();
  }
  return (P_prior.row(state_idx) * H.transpose()).transpose();
}

VectorXd SafeStateBlockNumerator(const MatrixXd &P_prior, const MatrixXd &H,
                                 int row_idx, int block_start, int block_dim) {
  if (row_idx < 0 || row_idx >= P_prior.rows() || block_dim <= 0 ||
      block_start < 0 || block_start + block_dim > P_prior.cols() ||
      P_prior.rows() != P_prior.cols() || H.cols() != P_prior.cols()) {
    return VectorXd();
  }
  Eigen::RowVectorXd p_row =
      P_prior.block(row_idx, block_start, 1, block_dim);
  MatrixXd h_block = H.block(0, block_start, H.rows(), block_dim);
  return (p_row * h_block.transpose()).transpose();
}

VectorXd SafeCovRowBlock(const MatrixXd &P, int row_idx, int block_start,
                         int block_dim) {
  if (row_idx < 0 || row_idx >= P.rows() || block_dim <= 0 ||
      block_start < 0 || block_start + block_dim > P.cols() ||
      P.rows() != P.cols()) {
    return VectorXd();
  }
  return P.block(row_idx, block_start, 1, block_dim).transpose();
}

VectorXd SafeGainRow(const MatrixXd &K, int state_idx) {
  if (state_idx < 0 || state_idx >= K.rows()) {
    return VectorXd();
  }
  return K.row(state_idx).transpose();
}

double SafeCovarianceDeltaLeft(const MatrixXd &P_prior, const MatrixXd &H,
                               const MatrixXd &K, int row_idx, int col_idx) {
  const VectorXd k_row = SafeGainRow(K, row_idx);
  const VectorXd num_col = SafeMeasurementNumerator(P_prior, H, col_idx);
  if (k_row.size() == 0 || num_col.size() == 0 || k_row.size() != num_col.size()) {
    return numeric_limits<double>::quiet_NaN();
  }
  return -k_row.dot(num_col);
}

double SafeCovarianceDeltaRight(const MatrixXd &P_prior, const MatrixXd &H,
                                const MatrixXd &K, int row_idx, int col_idx) {
  const VectorXd num_row = SafeMeasurementNumerator(P_prior, H, row_idx);
  const VectorXd k_col = SafeGainRow(K, col_idx);
  if (num_row.size() == 0 || k_col.size() == 0 || num_row.size() != k_col.size()) {
    return numeric_limits<double>::quiet_NaN();
  }
  return -num_row.dot(k_col);
}

double SafeCovarianceDeltaGain(const MatrixXd &S, const MatrixXd &K, int row_idx,
                               int col_idx) {
  const VectorXd k_row = SafeGainRow(K, row_idx);
  const VectorXd k_col = SafeGainRow(K, col_idx);
  if (k_row.size() == 0 || k_col.size() == 0 || S.rows() != S.cols() ||
      S.rows() != k_row.size() || S.cols() != k_col.size()) {
    return numeric_limits<double>::quiet_NaN();
  }
  return k_row.dot(S * k_col);
}

// ---------- 状态日志条目 ----------
struct StateDiagLog {
  double t, dt;
  Vector4d q;
  Vector3d v, v_b, omega, f_b;
  bool static_imu, static_zupt, has_truth;
  double dq_deg, dv_norm;
  Vector3d dv;
};

struct StateLogLimiter {
  size_t stride = 1;
  size_t seen = 0;
};

bool ShouldLogState(StateLogLimiter &lim) {
  ++lim.seen;
  return (lim.stride <= 1) || ((lim.seen - 1) % lim.stride) == 0;
}

void LogStateWindow(const StateDiagLog &e, const string &label) {
  cout << "[Diag] " << label << " state t=" << e.t << " dt=" << e.dt
       << " q_wxyz=" << e.q.transpose() << " v=" << e.v.transpose()
       << " v_norm=" << e.v.norm() << " v_b=" << e.v_b.transpose()
       << " omega=" << e.omega.transpose() << " omega_norm=" << e.omega.norm()
       << " f_b=" << e.f_b.transpose() << " f_norm=" << e.f_b.norm()
       << " static_imu=" << (e.static_imu ? 1 : 0)
       << " static_zupt=" << (e.static_zupt ? 1 : 0) << "\n";
  if (e.has_truth) {
    cout << "[Diag] " << label << " qv diff t=" << e.t
         << " dq_deg=" << e.dq_deg << " dv=" << e.dv.transpose()
         << " dv_norm=" << e.dv_norm << "\n";
  }
}

}  // namespace

// ============================================================
// DiagnosticsEngine::Impl
// ============================================================
struct DiagnosticsEngine::Impl {
  bool enabled = false;
  ConstraintConfig config;

  // 重力对准诊断状态
  double static_duration = 0.0;
  Vector3d static_sum_f = Vector3d::Zero();
  int static_count = 0;
  bool gravity_reported = false;
  int truth_cursor_gravity = 0;

  // 漂移检测
  bool drift_active = false;
  double drift_start_t = 0.0;
  double drift_peak_deg = 0.0;
  vector<double> drift_diff_vals, drift_times;

  // 量测日志缓存
  deque<MeasUpdateLog> meas_log_buffer;
  size_t meas_log_buffer_max = 0;
  size_t meas_log_stride = 1;
  size_t meas_log_max = 0;
  MeasLogLimiter drift_meas_limiter{}, div_meas_limiter{};

  // 漂移/发散窗口
  double window_pre = 0.0, window_post = 0.0;
  bool drift_window_active = false;
  double drift_window_start = 0.0, drift_window_end = 0.0;
  bool div_window_active = false;
  bool first_divergence_detected = false;
  double div_window_start = 0.0, div_window_end = 0.0;
  double first_div_dq_deg = 0.0, first_div_dv = 0.0, first_div_speed = 0.0;

  // 状态日志
  deque<StateDiagLog> state_log_buffer;
  size_t state_log_stride = 1;
  StateLogLimiter drift_state_limiter{}, div_state_limiter{};
  int truth_cursor_state = 0;

  // DIAG.txt
  ofstream diag_file;
  double last_diag_t = -1e9;
  static constexpr double kDiagInterval = 1.0;

  // 机理归因日志
  ofstream mechanism_file;
  bool mechanism_enabled = false;
  size_t mechanism_stride = 1;
  size_t mechanism_seen_nhc = 0;
  size_t mechanism_seen_odo = 0;
  double gnss_split_t = numeric_limits<double>::infinity();
  double gnss_tol = 0.0;

  ofstream first_update_file;
  bool first_gnss_pos_recorded = false;
  ofstream gnss_update_file;
  ofstream predict_debug_file;
  size_t predict_debug_count = 0;
  double predict_debug_start_t = -numeric_limits<double>::infinity();
  double predict_debug_end_t = numeric_limits<double>::infinity();

  // NHC 跳过警告
  bool nhc_skip_warned = false;

  // ---- 辅助方法 ----
  void ResetMeasLimiter(MeasLogLimiter &lim) {
    lim = MeasLogLimiter{};
    lim.stride = (meas_log_stride == 0) ? 1 : meas_log_stride;
    lim.max_total = meas_log_max;
  }

  void ResetStateLimiter(StateLogLimiter &lim) {
    lim = StateLogLimiter{};
    lim.stride = (state_log_stride == 0) ? 1 : state_log_stride;
  }

  void PushMeasLog(const string &tag, double t, const VectorXd &y, const VectorXd &dx) {
    if (meas_log_buffer_max == 0) return;
    if (meas_log_buffer.size() >= meas_log_buffer_max) meas_log_buffer.pop_front();
    meas_log_buffer.push_back({tag, t, y, dx});
  }

  void FlushMeasLogLimited(const string &prefix, double min_t, double max_t,
                            MeasLogLimiter &lim) {
    for (const auto &e : meas_log_buffer) {
      if (e.t < min_t || e.t > max_t) continue;
      if (!ShouldLogMeasUpdate(lim, e.tag)) continue;
      LogMeasUpdate(prefix + e.tag, e.t, e.y, e.dx);
    }
  }

  void PushStateLog(const StateDiagLog &entry) {
    if (window_pre <= 0.0) return;
    state_log_buffer.push_back(entry);
    double min_t = entry.t - window_pre - 1e-9;
    while (!state_log_buffer.empty() && state_log_buffer.front().t < min_t)
      state_log_buffer.pop_front();
  }

  void FlushStateLog(double min_t, double max_t, StateLogLimiter &lim,
                      const string &label) {
    for (const auto &e : state_log_buffer) {
      if (e.t < min_t || e.t > max_t) continue;
      if (!ShouldLogState(lim)) continue;
      LogStateWindow(e, label);
    }
  }

  bool IsMechanismTimeEnabled(double t) const {
    const double start_time = config.mechanism_log_start_time;
    const double end_time = config.mechanism_log_end_time;
    const double tol = 1.0e-9;
    if (std::isfinite(start_time) && t + tol < start_time) {
      return false;
    }
    if (std::isfinite(end_time) && t - tol > end_time) {
      return false;
    }
    return true;
  }

  bool ShouldLogMechanism(const string &tag, double t) {
    if (!IsMechanismTimeEnabled(t)) {
      return false;
    }
    size_t *seen = nullptr;
    if (TagMatches(tag, "NHC")) {
      seen = &mechanism_seen_nhc;
    } else if (TagMatches(tag, "ODO")) {
      seen = &mechanism_seen_odo;
    } else {
      return false;
    }
    ++(*seen);
    return mechanism_stride <= 1 || (((*seen) - 1) % mechanism_stride) == 0;
  }
};

// ============================================================
// DiagnosticsEngine 公共接口实现
// ============================================================
DiagnosticsEngine::DiagnosticsEngine(const ConstraintConfig &config, bool enabled)
    : impl_(make_unique<Impl>()) {
  impl_->enabled = enabled;
  impl_->config = config;
  if (enabled) {
    impl_->meas_log_buffer_max = static_cast<size_t>(max(0, config.diag_meas_log_buffer));
    impl_->meas_log_stride = static_cast<size_t>(max(1, config.diag_meas_log_stride));
    impl_->meas_log_max = static_cast<size_t>(max(0, config.diag_meas_log_max));
    impl_->window_pre = max(0.0, static_cast<double>(config.diag_drift_window_pre));
    impl_->window_post = max(0.0, static_cast<double>(config.diag_drift_window_post));
    impl_->first_div_dq_deg = max(0.0, static_cast<double>(config.diag_first_divergence_dq_deg));
    impl_->first_div_dv = max(0.0, static_cast<double>(config.diag_first_divergence_dv));
    impl_->first_div_speed = max(0.0, static_cast<double>(config.diag_first_divergence_speed));
    impl_->state_log_stride = static_cast<size_t>(max(1, config.diag_state_log_stride));
    impl_->ResetMeasLimiter(impl_->drift_meas_limiter);
    impl_->ResetMeasLimiter(impl_->div_meas_limiter);
    impl_->ResetStateLimiter(impl_->drift_state_limiter);
    impl_->ResetStateLimiter(impl_->div_state_limiter);
  }
}

DiagnosticsEngine::~DiagnosticsEngine() = default;

bool DiagnosticsEngine::enabled() const { return impl_->enabled; }
bool DiagnosticsEngine::nhc_skip_warned() const { return impl_->nhc_skip_warned; }
void DiagnosticsEngine::set_nhc_skip_warned(bool v) { impl_->nhc_skip_warned = v; }

// ---------- Initialize ----------
void DiagnosticsEngine::Initialize(const Dataset &dataset, const FusionOptions &options) {
  if (!impl_->enabled) return;

  impl_->mechanism_enabled = impl_->config.enable_mechanism_log;
  impl_->mechanism_stride = static_cast<size_t>(max(1, impl_->config.mechanism_log_stride));
  impl_->gnss_tol = options.gating.time_tolerance;
  impl_->predict_debug_start_t =
      std::isfinite(options.predict_debug_start_time)
          ? options.predict_debug_start_time
          : -numeric_limits<double>::infinity();
  impl_->predict_debug_end_t =
      std::isfinite(options.predict_debug_end_time)
          ? options.predict_debug_end_time
          : numeric_limits<double>::infinity();
  if (options.gnss_schedule.enabled && !dataset.imu.empty() &&
      dataset.imu.back().t > dataset.imu.front().t) {
    double t0_nav = dataset.imu.front().t;
    double t1_nav = dataset.imu.back().t;
    impl_->gnss_split_t =
        t0_nav + options.gnss_schedule.head_ratio * (t1_nav - t0_nav);
  } else {
    impl_->gnss_split_t = numeric_limits<double>::infinity();
  }

  // 离线真值重力检查
  if (impl_->config.enable_diagnostics &&
      !dataset.imu.empty() && dataset.truth.timestamps.size() > 0) {
    const auto &imu = dataset.imu;
    const auto &t_truth = dataset.truth.timestamps;
    const auto &pos_truth = dataset.truth.positions;
    const auto &quat_truth = dataset.truth.quaternions;

    double static_dur = 0.0;
    Vector3d sum_f = Vector3d::Zero();
    int count = 0;
    bool reported = false;
    int idx_truth = 0;
    vector<double> angles_deg;

    for (size_t i = 0; i < imu.size(); ++i) {
      const auto &d = imu[i];
      bool is_static = DetectZuptImuOnly(d, impl_->config);
      if (is_static && d.dt > 1e-9) {
        static_dur += d.dt;
        sum_f += d.dvel / d.dt;
        ++count;
      } else {
        static_dur = 0.0; sum_f.setZero(); count = 0; reported = false;
      }
      if (!reported && count > 0 &&
          static_dur + 1e-12 >= impl_->config.diag_gravity_min_duration) {
        int idx = FindNearestTruthIndex(t_truth, d.t, idx_truth);
        double angle_deg = 0.0;
        if (ComputeGravityAlignmentAngle(sum_f / static_cast<double>(count),
                                          quat_truth.row(idx).transpose(),
                                          pos_truth.row(idx).transpose(), angle_deg)) {
          cout << "[Diag] Truth gravity alignment t=" << d.t
               << " angle_deg=" << angle_deg << "\n";
          angles_deg.push_back(angle_deg);
        }
        reported = true;
      }
    }
    if (!angles_deg.empty()) {
      double sum = 0.0, min_v = angles_deg[0], max_v = angles_deg[0];
      for (double a : angles_deg) { sum += a; min_v = min(min_v, a); max_v = max(max_v, a); }
      cout << "[Diag] Truth gravity alignment summary count=" << angles_deg.size()
           << " mean_deg=" << sum / angles_deg.size()
           << " min_deg=" << min_v << " max_deg=" << max_v << "\n";
    } else {
      cout << "[Diag] Truth gravity alignment: no static window found\n";
    }
  }

  // 打开 DIAG.txt 并写表头
  if (impl_->config.enable_diagnostics) {
    impl_->diag_file.open("DIAG.txt");
  }
  if (impl_->diag_file.is_open()) {
    impl_->diag_file << fixed << setprecision(8);
    impl_->diag_file << "t";
    const char* names[kStateDim] = {
      "px","py","pz", "vx","vy","vz", "phiN","phiE","phiD",
      "bax","bay","baz", "bgx","bgy","bgz",
      "sgx","sgy","sgz", "sax","say","saz", "odo_s",
      "mr", "mp", "my", "lx", "ly", "lz", "lgx", "lgy", "lgz"
    };
    for (int j = 0; j < kStateDim; ++j) impl_->diag_file << " std_" << names[j];
    impl_->diag_file << " corr_odo_bgx corr_odo_bgy corr_bax_phiE corr_bay_phiN"
                     << " corr_baz_phiN corr_baz_phiE corr_mp_phiE corr_my_phiD corr_odo_bax";
    impl_->diag_file << " v_fwd omega_z odo_speed\n";
  }

  if (impl_->mechanism_enabled) {
    fs::path sol_path(options.output_path);
    fs::path mechanism_path =
        sol_path.parent_path() / (sol_path.stem().string() + "_mechanism.csv");
    if (!mechanism_path.parent_path().empty()) {
      fs::create_directories(mechanism_path.parent_path());
    }
    impl_->mechanism_file.open(mechanism_path, ios::out | ios::trunc);
    if (impl_->mechanism_file.is_open()) {
      impl_->mechanism_file << fixed << setprecision(9);
      impl_->mechanism_file
          << "tag,t_meas,t_state,post_gnss,used_true_iekf,y_dim,y_norm,nis,s_trace,"
          << "dx_att_z,dx_bg_z,dx_mount_yaw,"
          << "k_row_att_z_norm,k_row_bg_z_norm,k_row_mount_yaw_norm,"
          << "k_att_z_vec,k_bg_z_vec,k_mount_yaw_vec,"
          << "h_col_att_z_norm,h_col_bg_z_norm,h_col_mount_yaw_norm,"
          << "h_block_att_norm,h_block_bg_norm,h_block_mount_norm,"
          << "info_att_z,info_bg_z,info_mount_yaw,info_heading_trace,"
          << "num_att_z_vec,num_bg_z_vec,num_mount_yaw_vec,"
          << "num_bg_z_from_vel_vec,num_bg_z_from_att_vec,"
          << "num_bg_z_from_bg_vec,num_bg_z_from_sg_vec,"
          << "num_bg_z_from_mount_vec,num_bg_z_from_lever_vec,"
          << "p_row_bg_z_att_vec,p_row_bg_z_bg_vec,p_row_bg_z_mount_vec,"
          << "h_att_x_vec,h_att_y_vec,h_att_z_vec,"
          << "h_bg_x_vec,h_bg_y_vec,h_bg_z_vec,"
          << "h_mount_roll_vec,h_mount_pitch_vec,h_mount_yaw_vec,"
          << "num_bg_z_from_att_x_vec,num_bg_z_from_att_y_vec,"
          << "num_bg_z_from_att_z_vec,"
          << "num_bg_z_from_bg_x_vec,num_bg_z_from_bg_y_vec,"
          << "num_bg_z_from_bg_z_vec,"
          << "num_bg_z_from_mount_roll_vec,num_bg_z_from_mount_pitch_vec,"
          << "num_bg_z_from_mount_yaw_vec,"
          << "prior_cov_att_z_bg_z,prior_corr_att_z_bg_z,"
          << "prior_cov_mount_yaw_bg_z,prior_corr_mount_yaw_bg_z,"
          << "delta_cov_att_z_bg_z_left,delta_cov_att_z_bg_z_right,"
          << "delta_cov_att_z_bg_z_gain,delta_cov_att_z_bg_z_total,"
          << "delta_cov_mount_yaw_bg_z_left,delta_cov_mount_yaw_bg_z_right,"
          << "delta_cov_mount_yaw_bg_z_gain,delta_cov_mount_yaw_bg_z_total,"
          << "post_cov_att_z_bg_z,post_corr_att_z_bg_z,"
          << "post_cov_mount_yaw_bg_z,post_corr_mount_yaw_bg_z,"
          << "bg_z_before,bg_z_after,mount_yaw_before,mount_yaw_after\n";
    }
  }

  if (!options.first_update_debug_output_path.empty()) {
    fs::path first_update_path(options.first_update_debug_output_path);
    if (!first_update_path.parent_path().empty()) {
      fs::create_directories(first_update_path.parent_path());
    }
    impl_->first_update_file.open(first_update_path, ios::out | ios::trunc);
    if (impl_->first_update_file.is_open()) {
      impl_->first_update_file << fixed << setprecision(9);
      impl_->first_update_file
          << "tag,gnss_axis,gnss_t,state_t,used_true_iekf,"
          << "lever_before,lever_after,dx_gnss_lever,"
          << "state_p_before_ecef,state_v_before_ecef,state_q_before_wxyz,"
          << "state_p_after_ecef,"
          << "y_vec,h_gnss_lever_vec,k_gnss_lever_vec\n";
    }
  }

  if (!options.gnss_update_debug_output_path.empty()) {
    fs::path gnss_update_path(options.gnss_update_debug_output_path);
    if (!gnss_update_path.parent_path().empty()) {
      fs::create_directories(gnss_update_path.parent_path());
    }
    impl_->gnss_update_file.open(gnss_update_path, ios::out | ios::trunc);
    if (impl_->gnss_update_file.is_open()) {
      impl_->gnss_update_file << fixed << setprecision(9);
      impl_->gnss_update_file
          << "tag,gnss_t,state_t,used_true_iekf,"
          << "y_x,y_y,y_z,y_norm,"
          << "state_p_before_x,state_p_before_y,state_p_before_z,"
          << "state_q_before_w,state_q_before_x,state_q_before_y,state_q_before_z,"
          << "state_v_before_x,state_v_before_y,state_v_before_z,"
          << "state_p_after_x,state_p_after_y,state_p_after_z,"
          << "state_v_after_x,state_v_after_y,state_v_after_z,"
          << "state_q_after_w,state_q_after_x,state_q_after_y,state_q_after_z,"
          << "lever_before_x,lever_before_y,lever_before_z,"
          << "dx_pos_x,dx_pos_y,dx_pos_z,"
          << "dx_vel_x,dx_vel_y,dx_vel_z,"
          << "dx_att_x,dx_att_y,dx_att_z,"
          << "dx_ba_x,dx_ba_y,dx_ba_z,"
          << "dx_bg_x,dx_bg_y,dx_bg_z,"
          << "dx_sg_x,dx_sg_y,dx_sg_z,"
          << "dx_sa_x,dx_sa_y,dx_sa_z,"
          << "dx_gnss_lever_x,dx_gnss_lever_y,dx_gnss_lever_z,"
          << "prior_std_pos_vec,prior_std_att_vec,prior_std_ba_vec,prior_std_bg_vec,prior_std_gnss_lever_vec,"
          << "prior_cov_pos_mat,prior_cov_vel_mat,prior_cov_att_mat,"
          << "prior_cov_pos_vel_mat,prior_cov_pos_att_mat,prior_cov_vel_att_mat,"
          << "prior_cov_pos_gnss_lever_mat,prior_cov_att_gnss_lever_mat,"
          << "prior_cov_ba_pos_mat,prior_cov_ba_att_mat,prior_cov_ba_gnss_lever_mat,"
          << "prior_cov_bg_pos_mat,prior_cov_bg_att_mat,prior_cov_bg_gnss_lever_mat,"
          << "prior_cov_gnss_lever_mat,"
          << "r_mat,"
          << "s_mat,"
          << "h_pos_x_vec,h_pos_y_vec,h_pos_z_vec,"
          << "h_att_x_vec,h_att_y_vec,h_att_z_vec,"
          << "h_gnss_lever_x_vec,h_gnss_lever_y_vec,h_gnss_lever_z_vec,"
          << "raw_k_gnss_lever_y_vec,"
          << "num_gnss_lever_y_vec,"
          << "num_gnss_lever_y_from_pos_vec,"
          << "num_gnss_lever_y_from_att_vec,"
          << "num_gnss_lever_y_from_gnss_lever_vec,"
          << "num_gnss_lever_y_from_lgx_vec,"
          << "num_gnss_lever_y_from_lgy_vec,"
          << "num_gnss_lever_y_from_lgz_vec,"
          << "k_pos_x_vec,k_pos_y_vec,k_pos_z_vec,"
          << "k_vel_x_vec,k_vel_y_vec,k_vel_z_vec,"
          << "k_att_x_vec,k_att_y_vec,k_att_z_vec,"
          << "k_ba_x_vec,k_ba_y_vec,k_ba_z_vec,"
          << "k_bg_x_vec,k_bg_y_vec,k_bg_z_vec,"
          << "k_gnss_lever_x_vec,k_gnss_lever_y_vec,k_gnss_lever_z_vec\n";
    }
  }

  if (!options.predict_debug_output_path.empty()) {
    fs::path predict_debug_path(options.predict_debug_output_path);
    if (!predict_debug_path.parent_path().empty()) {
      fs::create_directories(predict_debug_path.parent_path());
    }
    impl_->predict_debug_file.open(predict_debug_path, ios::out | ios::trunc);
    if (impl_->predict_debug_file.is_open()) {
      impl_->predict_debug_file << fixed << setprecision(9);
      impl_->predict_debug_file
          << "step_index,tag,t_prev,t_curr,dt,"
          << "p_before_common_mat,phi_common_mat,qd_common_mat,"
          << "phi_p_common_mat,p_after_raw_common_mat,p_after_final_common_mat\n";
    }
  }
}

// ---------- Correct ----------
bool DiagnosticsEngine::Correct(EskfEngine &engine, const string &tag, double t,
                                const VectorXd &y, const MatrixXd &H, const MatrixXd &R,
                                const StateMask *update_mask,
                                const StateGainScale *gain_scale,
                                const StateMeasurementGainScale *gain_element_scale) {
  if (!impl_->enabled) {
    return engine.Correct(y, H, R, nullptr, update_mask, gain_scale,
                          gain_element_scale);
  }

  // 捕获修正前状态（用于 NHC 的 dv/dq 额外日志）
  State state_before = engine.state();
  Vector3d v_before = engine.state().v;
  Vector4d q_before = engine.state().q;

  VectorXd dx;
  bool updated = engine.Correct(y, H, R, &dx, update_mask, gain_scale,
                                gain_element_scale);

  bool log_div = false, log_drift = false;
  if (impl_->config.enable_diagnostics) {
    // 缓存日志
    impl_->PushMeasLog(tag, t, y, dx);

    // 漂移/发散窗口日志
    if (impl_->div_window_active) {
      log_div = ShouldLogMeasUpdate(impl_->div_meas_limiter, tag);
      if (log_div) LogMeasUpdate("Div " + tag, t, y, dx);
    }
    if (impl_->drift_window_active) {
      log_drift = ShouldLogMeasUpdate(impl_->drift_meas_limiter, tag);
      if (log_drift) LogMeasUpdate("Drift " + tag, t, y, dx);
    }

    // NHC 特有的修正后 dv/dq 日志
    if (updated && (log_div || log_drift) && tag == "NHC") {
      Vector3d dv = engine.state().v - v_before;
      double dq_deg = QuatDeltaAngleRad(q_before, engine.state().q) * kRadToDeg;
      if (log_div) {
        cout << "[Diag] Div NHC update t=" << t
             << " dv=" << dv.transpose() << " dq_deg=" << dq_deg << "\n";
      }
      if (log_drift) {
        cout << "[Diag] Drift NHC update t=" << t
             << " dv=" << dv.transpose() << " dq_deg=" << dq_deg << "\n";
      }
    }
  }

  if (updated && impl_->mechanism_enabled && impl_->mechanism_file.is_open() &&
      impl_->ShouldLogMechanism(tag, t)) {
    bool post_gnss =
        std::isfinite(impl_->gnss_split_t) && t > impl_->gnss_split_t + impl_->gnss_tol;
    if (!impl_->config.mechanism_log_post_gnss_only || post_gnss) {
      const auto &snap = engine.last_correction_debug();
      if (snap.valid) {
        const MatrixXd H_att = snap.H.block(0, StateIdx::kAtt, snap.H.rows(), 3);
        const MatrixXd H_bg = snap.H.block(0, StateIdx::kBg, snap.H.rows(), 3);
        const MatrixXd H_mount =
            snap.H.block(0, StateIdx::kMountRoll, snap.H.rows(), 3);
        const VectorXd h_att_x = snap.H.col(StateIdx::kAtt + 0);
        const VectorXd h_att_y = snap.H.col(StateIdx::kAtt + 1);
        const VectorXd h_att_z = snap.H.col(StateIdx::kAtt + 2);
        const VectorXd h_bg_x = snap.H.col(StateIdx::kBg + 0);
        const VectorXd h_bg_y = snap.H.col(StateIdx::kBg + 1);
        const VectorXd h_bg_z = snap.H.col(StateIdx::kBg + 2);
        const VectorXd h_mount_roll = snap.H.col(StateIdx::kMountRoll);
        const VectorXd h_mount_pitch = snap.H.col(StateIdx::kMountPitch);
        const VectorXd h_mount_yaw = snap.H.col(StateIdx::kMountYaw);
        const VectorXd k_att_z = SafeGainRow(snap.K, StateIdx::kAtt + 2);
        const VectorXd k_bg_z = SafeGainRow(snap.K, StateIdx::kBg + 2);
        const VectorXd k_mount_yaw = SafeGainRow(snap.K, StateIdx::kMountYaw);
        const VectorXd num_att_z =
            SafeMeasurementNumerator(snap.P_prior, snap.H, StateIdx::kAtt + 2);
        const VectorXd num_bg_z =
            SafeMeasurementNumerator(snap.P_prior, snap.H, StateIdx::kBg + 2);
        const VectorXd num_mount_yaw =
            SafeMeasurementNumerator(snap.P_prior, snap.H, StateIdx::kMountYaw);
        const VectorXd num_bg_z_from_vel = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kVel, 3);
        const VectorXd num_bg_z_from_att = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kAtt, 3);
        const VectorXd num_bg_z_from_bg = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kBg, 3);
        const VectorXd num_bg_z_from_sg = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kSg, 3);
        const VectorXd num_bg_z_from_mount = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kMountRoll, 3);
        const VectorXd num_bg_z_from_lever = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kLever, 3);
        const VectorXd p_row_bg_z_att = SafeCovRowBlock(
            snap.P_prior, StateIdx::kBg + 2, StateIdx::kAtt, 3);
        const VectorXd p_row_bg_z_bg = SafeCovRowBlock(
            snap.P_prior, StateIdx::kBg + 2, StateIdx::kBg, 3);
        const VectorXd p_row_bg_z_mount = SafeCovRowBlock(
            snap.P_prior, StateIdx::kBg + 2, StateIdx::kMountRoll, 3);
        const VectorXd num_bg_z_from_att_x = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kAtt + 0, 1);
        const VectorXd num_bg_z_from_att_y = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kAtt + 1, 1);
        const VectorXd num_bg_z_from_att_z = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kAtt + 2, 1);
        const VectorXd num_bg_z_from_bg_x = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kBg + 0, 1);
        const VectorXd num_bg_z_from_bg_y = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kBg + 1, 1);
        const VectorXd num_bg_z_from_bg_z = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kBg + 2, 1);
        const VectorXd num_bg_z_from_mount_roll = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kMountRoll, 1);
        const VectorXd num_bg_z_from_mount_pitch = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kMountPitch, 1);
        const VectorXd num_bg_z_from_mount_yaw = SafeStateBlockNumerator(
            snap.P_prior, snap.H, StateIdx::kBg + 2, StateIdx::kMountYaw, 1);
        MatrixXd H_heading(snap.H.rows(), 3);
        H_heading.col(0) = h_att_z;
        H_heading.col(1) = h_bg_z;
        H_heading.col(2) = h_mount_yaw;
        const auto &post_cov = engine.cov();
        const double prior_cov_att_bg =
            SafeCovValue(snap.P_prior, StateIdx::kAtt + 2, StateIdx::kBg + 2);
        const double prior_cov_mount_bg =
            SafeCovValue(snap.P_prior, StateIdx::kMountYaw, StateIdx::kBg + 2);
        const double post_cov_att_bg =
            SafeCovValue(post_cov, StateIdx::kAtt + 2, StateIdx::kBg + 2);
        const double post_cov_mount_bg =
            SafeCovValue(post_cov, StateIdx::kMountYaw, StateIdx::kBg + 2);
        const double delta_cov_att_bg_left = SafeCovarianceDeltaLeft(
            snap.P_prior, snap.H, snap.K, StateIdx::kAtt + 2, StateIdx::kBg + 2);
        const double delta_cov_att_bg_right = SafeCovarianceDeltaRight(
            snap.P_prior, snap.H, snap.K, StateIdx::kAtt + 2, StateIdx::kBg + 2);
        const double delta_cov_att_bg_gain = SafeCovarianceDeltaGain(
            snap.S, snap.K, StateIdx::kAtt + 2, StateIdx::kBg + 2);
        const double delta_cov_mount_bg_left = SafeCovarianceDeltaLeft(
            snap.P_prior, snap.H, snap.K, StateIdx::kMountYaw, StateIdx::kBg + 2);
        const double delta_cov_mount_bg_right = SafeCovarianceDeltaRight(
            snap.P_prior, snap.H, snap.K, StateIdx::kMountYaw, StateIdx::kBg + 2);
        const double delta_cov_mount_bg_gain = SafeCovarianceDeltaGain(
            snap.S, snap.K, StateIdx::kMountYaw, StateIdx::kBg + 2);
        impl_->mechanism_file
            << tag << "," << t << "," << snap.t_state << ","
            << (post_gnss ? 1 : 0) << ","
            << (snap.used_true_iekf ? 1 : 0) << ","
            << snap.y.size() << "," << snap.y.norm() << ","
            << SafeNisFromInnovation(snap.S, snap.y) << ","
            << snap.S.trace() << ","
            << snap.dx(StateIdx::kAtt + 2) << ","
            << snap.dx(StateIdx::kBg + 2) << ","
            << snap.dx(StateIdx::kMountYaw) << ","
            << snap.K.row(StateIdx::kAtt + 2).norm() << ","
            << snap.K.row(StateIdx::kBg + 2).norm() << ","
            << snap.K.row(StateIdx::kMountYaw).norm() << ","
            << VectorToCsvField(k_att_z) << ","
            << VectorToCsvField(k_bg_z) << ","
            << VectorToCsvField(k_mount_yaw) << ","
            << h_att_z.norm() << ","
            << h_bg_z.norm() << ","
            << h_mount_yaw.norm() << ","
            << H_att.norm() << ","
            << H_bg.norm() << ","
            << H_mount.norm() << ","
            << SafeQuadraticInfo(snap.R, h_att_z) << ","
            << SafeQuadraticInfo(snap.R, h_bg_z) << ","
            << SafeQuadraticInfo(snap.R, h_mount_yaw) << ","
            << SafeTraceInfo(snap.R, H_heading) << ","
            << VectorToCsvField(num_att_z) << ","
            << VectorToCsvField(num_bg_z) << ","
            << VectorToCsvField(num_mount_yaw) << ","
            << VectorToCsvField(num_bg_z_from_vel) << ","
            << VectorToCsvField(num_bg_z_from_att) << ","
            << VectorToCsvField(num_bg_z_from_bg) << ","
            << VectorToCsvField(num_bg_z_from_sg) << ","
            << VectorToCsvField(num_bg_z_from_mount) << ","
            << VectorToCsvField(num_bg_z_from_lever) << ","
            << VectorToCsvField(p_row_bg_z_att) << ","
            << VectorToCsvField(p_row_bg_z_bg) << ","
            << VectorToCsvField(p_row_bg_z_mount) << ","
            << VectorToCsvField(h_att_x) << ","
            << VectorToCsvField(h_att_y) << ","
            << VectorToCsvField(h_att_z) << ","
            << VectorToCsvField(h_bg_x) << ","
            << VectorToCsvField(h_bg_y) << ","
            << VectorToCsvField(h_bg_z) << ","
            << VectorToCsvField(h_mount_roll) << ","
            << VectorToCsvField(h_mount_pitch) << ","
            << VectorToCsvField(h_mount_yaw) << ","
            << VectorToCsvField(num_bg_z_from_att_x) << ","
            << VectorToCsvField(num_bg_z_from_att_y) << ","
            << VectorToCsvField(num_bg_z_from_att_z) << ","
            << VectorToCsvField(num_bg_z_from_bg_x) << ","
            << VectorToCsvField(num_bg_z_from_bg_y) << ","
            << VectorToCsvField(num_bg_z_from_bg_z) << ","
            << VectorToCsvField(num_bg_z_from_mount_roll) << ","
            << VectorToCsvField(num_bg_z_from_mount_pitch) << ","
            << VectorToCsvField(num_bg_z_from_mount_yaw) << ","
            << prior_cov_att_bg << ","
            << SafeCorrValue(snap.P_prior, StateIdx::kAtt + 2, StateIdx::kBg + 2)
            << ","
            << prior_cov_mount_bg << ","
            << SafeCorrValue(snap.P_prior, StateIdx::kMountYaw, StateIdx::kBg + 2)
            << ","
            << delta_cov_att_bg_left << ","
            << delta_cov_att_bg_right << ","
            << delta_cov_att_bg_gain << ","
            << (post_cov_att_bg - prior_cov_att_bg) << ","
            << delta_cov_mount_bg_left << ","
            << delta_cov_mount_bg_right << ","
            << delta_cov_mount_bg_gain << ","
            << (post_cov_mount_bg - prior_cov_mount_bg) << ","
            << post_cov_att_bg << ","
            << SafeCorrValue(post_cov, StateIdx::kAtt + 2, StateIdx::kBg + 2)
            << ","
            << post_cov_mount_bg << ","
            << SafeCorrValue(post_cov, StateIdx::kMountYaw, StateIdx::kBg + 2)
            << ","
            << state_before.bg.z() << "," << engine.state().bg.z() << ","
            << state_before.mounting_yaw << "," << engine.state().mounting_yaw
            << "\n";
      }
    }
  }

  if (updated && !impl_->first_gnss_pos_recorded &&
      impl_->first_update_file.is_open() && TagMatches(tag, "GNSS_POS")) {
    const auto &snap = engine.last_correction_debug();
    if (snap.valid && snap.H.cols() == kStateDim && snap.K.rows() == kStateDim) {
      static const char *kAxes[3] = {"x", "y", "z"};
      for (int axis = 0; axis < 3; ++axis) {
        VectorXd h_col = snap.H.col(StateIdx::kGnssLever + axis);
        VectorXd k_row = snap.K.row(StateIdx::kGnssLever + axis).transpose();
        double lever_before = state_before.gnss_lever_arm(axis);
        double lever_after = engine.state().gnss_lever_arm(axis);
        impl_->first_update_file
            << tag << "," << kAxes[axis] << "," << t << ","
            << snap.t_state << "," << (snap.used_true_iekf ? 1 : 0) << ","
            << lever_before << "," << lever_after << ","
            << snap.dx(StateIdx::kGnssLever + axis) << ","
            << VectorToCsvField(state_before.p) << ","
            << VectorToCsvField(state_before.v) << ","
            << VectorToCsvField(state_before.q) << ","
            << VectorToCsvField(engine.state().p) << ","
            << VectorToCsvField(snap.y) << ","
            << VectorToCsvField(h_col) << ","
            << VectorToCsvField(k_row) << "\n";
      }
      impl_->first_gnss_pos_recorded = true;
    }
  }

  if (updated && impl_->gnss_update_file.is_open() &&
      (TagMatches(tag, "GNSS_POS") || TagMatches(tag, "GNSS_VEL"))) {
    const auto &snap = engine.last_correction_debug();
    if (snap.valid) {
      const auto innovation_at = [&snap](int idx) -> double {
        return (idx >= 0 && idx < snap.y.size())
                   ? snap.y(idx)
                   : numeric_limits<double>::quiet_NaN();
      };
      const auto safe_h_col = [&snap](int idx) -> VectorXd {
        if (idx >= 0 && idx < snap.H.cols()) {
          return snap.H.col(idx);
        }
        return VectorXd::Constant(snap.H.rows(),
                                  numeric_limits<double>::quiet_NaN());
      };
      const auto safe_k_row = [&snap](int idx) -> VectorXd {
        if (idx >= 0 && idx < snap.K.rows()) {
          return snap.K.row(idx).transpose();
        }
        return VectorXd::Constant(snap.K.cols(),
                                  numeric_limits<double>::quiet_NaN());
      };
      const auto safe_state_block_numerator =
          [&snap](int row_idx, int block_start, int block_dim) -> VectorXd {
        if (row_idx < 0 || row_idx >= snap.P_prior.rows() || block_dim <= 0 ||
            block_start < 0 ||
            block_start + block_dim > snap.P_prior.cols() ||
            snap.H.cols() != snap.P_prior.cols()) {
          return VectorXd::Constant(
              snap.H.rows(), numeric_limits<double>::quiet_NaN());
        }
        Eigen::RowVectorXd p_row =
            snap.P_prior.block(row_idx, block_start, 1, block_dim);
        MatrixXd h_block = snap.H.block(0, block_start, snap.H.rows(), block_dim);
        return (p_row * h_block.transpose()).transpose();
      };
      Vector3d prior_std_pos = Vector3d::Constant(numeric_limits<double>::quiet_NaN());
      Vector3d prior_std_att = Vector3d::Constant(numeric_limits<double>::quiet_NaN());
      Vector3d prior_std_ba = Vector3d::Constant(numeric_limits<double>::quiet_NaN());
      Vector3d prior_std_bg = Vector3d::Constant(numeric_limits<double>::quiet_NaN());
      Vector3d prior_std_gnss_lever =
          Vector3d::Constant(numeric_limits<double>::quiet_NaN());
      if (snap.P_prior.allFinite()) {
        for (int axis = 0; axis < 3; ++axis) {
          prior_std_pos(axis) =
              sqrt(max(0.0, snap.P_prior(StateIdx::kPos + axis,
                                         StateIdx::kPos + axis)));
          prior_std_att(axis) =
              sqrt(max(0.0, snap.P_prior(StateIdx::kAtt + axis,
                                         StateIdx::kAtt + axis)));
          prior_std_ba(axis) =
              sqrt(max(0.0, snap.P_prior(StateIdx::kBa + axis,
                                         StateIdx::kBa + axis)));
          prior_std_bg(axis) =
              sqrt(max(0.0, snap.P_prior(StateIdx::kBg + axis,
                                         StateIdx::kBg + axis)));
          prior_std_gnss_lever(axis) =
              sqrt(max(0.0, snap.P_prior(StateIdx::kGnssLever + axis,
                                         StateIdx::kGnssLever + axis)));
        }
      }
      MatrixXd prior_cov_pos_att =
          snap.P_prior.block<3, 3>(StateIdx::kPos, StateIdx::kAtt);
      MatrixXd prior_cov_pos =
          snap.P_prior.block<3, 3>(StateIdx::kPos, StateIdx::kPos);
      MatrixXd prior_cov_vel =
          snap.P_prior.block<3, 3>(StateIdx::kVel, StateIdx::kVel);
      MatrixXd prior_cov_att =
          snap.P_prior.block<3, 3>(StateIdx::kAtt, StateIdx::kAtt);
      MatrixXd prior_cov_pos_vel =
          snap.P_prior.block<3, 3>(StateIdx::kPos, StateIdx::kVel);
      MatrixXd prior_cov_vel_att =
          snap.P_prior.block<3, 3>(StateIdx::kVel, StateIdx::kAtt);
      MatrixXd prior_cov_ba_pos =
          snap.P_prior.block<3, 3>(StateIdx::kBa, StateIdx::kPos);
      MatrixXd prior_cov_ba_att =
          snap.P_prior.block<3, 3>(StateIdx::kBa, StateIdx::kAtt);
      MatrixXd prior_cov_bg_pos =
          snap.P_prior.block<3, 3>(StateIdx::kBg, StateIdx::kPos);
      MatrixXd prior_cov_bg_att =
          snap.P_prior.block<3, 3>(StateIdx::kBg, StateIdx::kAtt);
      MatrixXd prior_cov_pos_gnss_lever =
          snap.P_prior.block<3, 3>(StateIdx::kPos, StateIdx::kGnssLever);
      MatrixXd prior_cov_att_gnss_lever =
          snap.P_prior.block<3, 3>(StateIdx::kAtt, StateIdx::kGnssLever);
      MatrixXd prior_cov_ba_gnss_lever =
          snap.P_prior.block<3, 3>(StateIdx::kBa, StateIdx::kGnssLever);
      MatrixXd prior_cov_bg_gnss_lever =
          snap.P_prior.block<3, 3>(StateIdx::kBg, StateIdx::kGnssLever);
      MatrixXd prior_cov_gnss_lever =
          snap.P_prior.block<3, 3>(StateIdx::kGnssLever, StateIdx::kGnssLever);
      VectorXd raw_k_gnss_lever_y =
          VectorXd::Constant(snap.K.cols(), numeric_limits<double>::quiet_NaN());
      VectorXd num_gnss_lever_y =
          VectorXd::Constant(snap.H.rows(), numeric_limits<double>::quiet_NaN());
      VectorXd num_gnss_lever_y_from_pos =
          VectorXd::Constant(snap.H.rows(), numeric_limits<double>::quiet_NaN());
      VectorXd num_gnss_lever_y_from_att =
          VectorXd::Constant(snap.H.rows(), numeric_limits<double>::quiet_NaN());
      VectorXd num_gnss_lever_y_from_gnss_lever =
          VectorXd::Constant(snap.H.rows(), numeric_limits<double>::quiet_NaN());
      VectorXd num_gnss_lever_y_from_lgx =
          VectorXd::Constant(snap.H.rows(), numeric_limits<double>::quiet_NaN());
      VectorXd num_gnss_lever_y_from_lgy =
          VectorXd::Constant(snap.H.rows(), numeric_limits<double>::quiet_NaN());
      VectorXd num_gnss_lever_y_from_lgz =
          VectorXd::Constant(snap.H.rows(), numeric_limits<double>::quiet_NaN());
      if (snap.H.rows() == snap.S.rows() && snap.S.rows() == snap.S.cols() &&
          snap.H.cols() == snap.P_prior.cols()) {
        VectorXd num_total = safe_state_block_numerator(StateIdx::kGnssLever + 1, 0,
                                                        kStateDim);
        if (num_total.size() == snap.S.rows() && snap.S.allFinite()) {
          Eigen::LDLT<MatrixXd> ldlt(snap.S);
          if (ldlt.info() == Eigen::Success) {
            raw_k_gnss_lever_y = ldlt.solve(num_total);
          }
        }
        num_gnss_lever_y = num_total;
        num_gnss_lever_y_from_pos =
            safe_state_block_numerator(StateIdx::kGnssLever + 1, StateIdx::kPos, 3);
        num_gnss_lever_y_from_att =
            safe_state_block_numerator(StateIdx::kGnssLever + 1, StateIdx::kAtt, 3);
        num_gnss_lever_y_from_gnss_lever = safe_state_block_numerator(
            StateIdx::kGnssLever + 1, StateIdx::kGnssLever, 3);
        num_gnss_lever_y_from_lgx = safe_state_block_numerator(
            StateIdx::kGnssLever + 1, StateIdx::kGnssLever + 0, 1);
        num_gnss_lever_y_from_lgy = safe_state_block_numerator(
            StateIdx::kGnssLever + 1, StateIdx::kGnssLever + 1, 1);
        num_gnss_lever_y_from_lgz = safe_state_block_numerator(
            StateIdx::kGnssLever + 1, StateIdx::kGnssLever + 2, 1);
      }
      impl_->gnss_update_file
          << tag << "," << t << "," << snap.t_state << ","
          << (snap.used_true_iekf ? 1 : 0) << ","
          << innovation_at(0) << "," << innovation_at(1) << ","
          << innovation_at(2) << "," << snap.y.norm() << ","
          << state_before.p.x() << "," << state_before.p.y() << ","
          << state_before.p.z() << ","
          << state_before.q(0) << "," << state_before.q(1) << ","
          << state_before.q(2) << "," << state_before.q(3) << ","
          << state_before.v.x() << "," << state_before.v.y() << ","
          << state_before.v.z() << ","
          << engine.state().p.x() << "," << engine.state().p.y() << ","
          << engine.state().p.z() << ","
          << engine.state().v.x() << "," << engine.state().v.y() << ","
          << engine.state().v.z() << ","
          << engine.state().q(0) << "," << engine.state().q(1) << ","
          << engine.state().q(2) << "," << engine.state().q(3) << ","
          << state_before.gnss_lever_arm.x() << ","
          << state_before.gnss_lever_arm.y() << ","
          << state_before.gnss_lever_arm.z() << ","
          << snap.dx(StateIdx::kPos + 0) << ","
          << snap.dx(StateIdx::kPos + 1) << ","
          << snap.dx(StateIdx::kPos + 2) << ","
          << snap.dx(StateIdx::kVel + 0) << ","
          << snap.dx(StateIdx::kVel + 1) << ","
          << snap.dx(StateIdx::kVel + 2) << ","
          << snap.dx(StateIdx::kAtt + 0) << ","
          << snap.dx(StateIdx::kAtt + 1) << ","
          << snap.dx(StateIdx::kAtt + 2) << ","
          << snap.dx(StateIdx::kBa + 0) << ","
          << snap.dx(StateIdx::kBa + 1) << ","
          << snap.dx(StateIdx::kBa + 2) << ","
          << snap.dx(StateIdx::kBg + 0) << ","
          << snap.dx(StateIdx::kBg + 1) << ","
          << snap.dx(StateIdx::kBg + 2) << ","
          << snap.dx(StateIdx::kSg + 0) << ","
          << snap.dx(StateIdx::kSg + 1) << ","
          << snap.dx(StateIdx::kSg + 2) << ","
          << snap.dx(StateIdx::kSa + 0) << ","
          << snap.dx(StateIdx::kSa + 1) << ","
          << snap.dx(StateIdx::kSa + 2) << ","
          << snap.dx(StateIdx::kGnssLever + 0) << ","
          << snap.dx(StateIdx::kGnssLever + 1) << ","
          << snap.dx(StateIdx::kGnssLever + 2) << ","
          << VectorToCsvField(prior_std_pos) << ","
          << VectorToCsvField(prior_std_att) << ","
          << VectorToCsvField(prior_std_ba) << ","
          << VectorToCsvField(prior_std_bg) << ","
          << VectorToCsvField(prior_std_gnss_lever) << ","
          << MatrixToCsvField(prior_cov_pos) << ","
          << MatrixToCsvField(prior_cov_vel) << ","
          << MatrixToCsvField(prior_cov_att) << ","
          << MatrixToCsvField(prior_cov_pos_vel) << ","
          << MatrixToCsvField(prior_cov_pos_att) << ","
          << MatrixToCsvField(prior_cov_vel_att) << ","
          << MatrixToCsvField(prior_cov_pos_gnss_lever) << ","
          << MatrixToCsvField(prior_cov_att_gnss_lever) << ","
          << MatrixToCsvField(prior_cov_ba_pos) << ","
          << MatrixToCsvField(prior_cov_ba_att) << ","
          << MatrixToCsvField(prior_cov_ba_gnss_lever) << ","
          << MatrixToCsvField(prior_cov_bg_pos) << ","
          << MatrixToCsvField(prior_cov_bg_att) << ","
          << MatrixToCsvField(prior_cov_bg_gnss_lever) << ","
          << MatrixToCsvField(prior_cov_gnss_lever) << ","
          << MatrixToCsvField(snap.R) << ","
          << MatrixToCsvField(snap.S) << ","
          << VectorToCsvField(safe_h_col(StateIdx::kPos + 0)) << ","
          << VectorToCsvField(safe_h_col(StateIdx::kPos + 1)) << ","
          << VectorToCsvField(safe_h_col(StateIdx::kPos + 2)) << ","
          << VectorToCsvField(safe_h_col(StateIdx::kAtt + 0)) << ","
          << VectorToCsvField(safe_h_col(StateIdx::kAtt + 1)) << ","
          << VectorToCsvField(safe_h_col(StateIdx::kAtt + 2)) << ","
          << VectorToCsvField(safe_h_col(StateIdx::kGnssLever + 0)) << ","
          << VectorToCsvField(safe_h_col(StateIdx::kGnssLever + 1)) << ","
          << VectorToCsvField(safe_h_col(StateIdx::kGnssLever + 2)) << ","
          << VectorToCsvField(raw_k_gnss_lever_y) << ","
          << VectorToCsvField(num_gnss_lever_y) << ","
          << VectorToCsvField(num_gnss_lever_y_from_pos) << ","
          << VectorToCsvField(num_gnss_lever_y_from_att) << ","
          << VectorToCsvField(num_gnss_lever_y_from_gnss_lever) << ","
          << VectorToCsvField(num_gnss_lever_y_from_lgx) << ","
          << VectorToCsvField(num_gnss_lever_y_from_lgy) << ","
          << VectorToCsvField(num_gnss_lever_y_from_lgz) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kPos + 0)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kPos + 1)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kPos + 2)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kVel + 0)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kVel + 1)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kVel + 2)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kAtt + 0)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kAtt + 1)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kAtt + 2)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kBa + 0)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kBa + 1)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kBa + 2)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kBg + 0)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kBg + 1)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kBg + 2)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kGnssLever + 0)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kGnssLever + 1)) << ","
          << VectorToCsvField(safe_k_row(StateIdx::kGnssLever + 2)) << "\n";
    }
  }
  return updated;
}

void DiagnosticsEngine::LogPredict(const string &tag, const EskfEngine &engine) {
  if (!impl_->enabled || !impl_->predict_debug_file.is_open()) {
    return;
  }
  const auto &snap = engine.last_predict_debug();
  if (!snap.valid) {
    return;
  }
  const double tol = impl_->gnss_tol;
  if (snap.t_prev + tol < impl_->predict_debug_start_t ||
      snap.t_prev >= impl_->predict_debug_end_t - tol) {
    return;
  }
  impl_->predict_debug_file
      << impl_->predict_debug_count++ << ","
      << tag << ","
      << snap.t_prev << ","
      << snap.t_curr << ","
      << snap.dt << ","
      << MatrixToCsvField(snap.P_before_common) << ","
      << MatrixToCsvField(snap.Phi_common) << ","
      << MatrixToCsvField(snap.Qd_common) << ","
      << MatrixToCsvField(snap.PhiP_common) << ","
      << MatrixToCsvField(snap.P_after_raw_common) << ","
      << MatrixToCsvField(snap.P_after_final_common) << "\n";
}

// ---------- CheckGravityAlignment ----------
void DiagnosticsEngine::CheckGravityAlignment(double t, double dt, const ImuData &imu,
                                               const State &state, const TruthData &truth) {
  if (!impl_->enabled || !impl_->config.enable_diagnostics) return;

  bool is_static = DetectZuptImuOnly(imu, impl_->config);

  if (is_static && dt > 0.0) {
    impl_->static_duration += dt;
    impl_->static_sum_f += imu.dvel / dt;
    ++impl_->static_count;
  } else if (!is_static) {
    impl_->static_duration = 0.0;
    impl_->static_sum_f.setZero();
    impl_->static_count = 0;
    impl_->gravity_reported = false;
  }

  if (impl_->gravity_reported || impl_->static_count <= 0 ||
      impl_->static_duration + 1e-12 < impl_->config.diag_gravity_min_duration)
    return;

  Vector3d f_b_avg = impl_->static_sum_f / static_cast<double>(impl_->static_count);

  // 滤波器重力对准角
  double filter_angle_deg = 0.0;
  bool has_filter = ComputeGravityAlignmentAngle(f_b_avg, state.q, state.p, filter_angle_deg);
  if (has_filter)
    cout << "[Diag] Gravity alignment t=" << t << " angle_deg=" << filter_angle_deg << "\n";

  // 真值比较 + 漂移检测
  if (truth.timestamps.size() > 0) {
    int idx = FindNearestTruthIndex(truth.timestamps, t, impl_->truth_cursor_gravity);
    Vector4d q_truth = truth.quaternions.row(idx).transpose();
    Vector3d p_truth = truth.positions.row(idx).transpose();
    Vector3d v_truth = truth.velocities.row(idx).transpose();

    double truth_angle_deg = 0.0;
    bool has_truth = ComputeGravityAlignmentAngle(f_b_avg, q_truth, p_truth, truth_angle_deg);

    if (has_filter && has_truth) {
      double diff_deg = std::abs(filter_angle_deg - truth_angle_deg);
      cout << "[Diag] Gravity alignment compare t=" << t
           << " filter_deg=" << filter_angle_deg << " truth_deg=" << truth_angle_deg
           << " diff_deg=" << diff_deg << "\n";

      double dq_deg = QuatDeltaAngleRad(q_truth, state.q) * kRadToDeg;
      Vector3d dv = state.v - v_truth;
      cout << "[Diag] Gravity qv diff t=" << t << " dq_deg=" << dq_deg
           << " dv=" << dv.transpose() << " dv_norm=" << dv.norm()
           << " v_truth=" << v_truth.transpose() << " v_filter=" << state.v.transpose() << "\n";

      impl_->drift_diff_vals.push_back(diff_deg);
      impl_->drift_times.push_back(t);

      constexpr double kDriftThreshold = 20.0;
      if (diff_deg > kDriftThreshold && !impl_->drift_active) {
        impl_->drift_active = true;
        impl_->drift_start_t = t;
        impl_->drift_peak_deg = diff_deg;
        cout << "[Diag] Gravity drift start t=" << t << " diff_deg=" << diff_deg << "\n";

        impl_->ResetMeasLimiter(impl_->drift_meas_limiter);
        impl_->ResetStateLimiter(impl_->drift_state_limiter);
        impl_->drift_window_start = t - impl_->window_pre;
        impl_->drift_window_end = t + impl_->window_post;
        impl_->drift_window_active = true;
        cout << "[Diag] Drift window t=" << impl_->drift_window_start
             << " to " << impl_->drift_window_end << "\n";

        if (!impl_->state_log_buffer.empty()) {
          cout << "[Diag] Pre-drift state window (buffered)\n";
          impl_->FlushStateLog(impl_->drift_window_start, t - 1e-9,
                                impl_->drift_state_limiter, "Drift window");
          impl_->ResetStateLimiter(impl_->drift_state_limiter);
        }
        if (!impl_->meas_log_buffer.empty()) {
          cout << "[Diag] Pre-drift measurement updates (buffered)\n";
          impl_->FlushMeasLogLimited("Pre-drift ", impl_->drift_window_start,
                                      t - 1e-9, impl_->drift_meas_limiter);
          impl_->meas_log_buffer.clear();
        }
      } else if (diff_deg > kDriftThreshold && impl_->drift_active) {
        impl_->drift_peak_deg = max(impl_->drift_peak_deg, diff_deg);
      } else if (diff_deg <= kDriftThreshold && impl_->drift_active) {
        cout << "[Diag] Gravity drift end t=" << t
             << " peak_diff_deg=" << impl_->drift_peak_deg << "\n";
        impl_->drift_active = false;
      }

      if (diff_deg > kDriftThreshold) {
        Matrix3d Cbn = QuatToRot(state.q);
        Vector3d v_b = Cbn.transpose() * state.v;
        Vector3d omega = Vector3d::Zero();
        if (dt > 1e-9) omega = imu.dtheta / dt - state.bg;
        cout << "[Diag] Gravity drift state t=" << t
             << " q_wxyz=" << state.q.transpose() << " v=" << state.v.transpose()
             << " v_b=" << v_b.transpose() << " omega=" << omega.transpose() << "\n";
      }
    }
  }

  impl_->gravity_reported = true;
}

// ---------- OnStepComplete ----------
void DiagnosticsEngine::OnStepComplete(double t, double dt, const State &state,
                                        const ImuData &imu, bool is_static_zupt,
                                        const TruthData &truth) {
  if (!impl_->enabled || !impl_->config.enable_diagnostics) return;

  bool is_static_imu = DetectZuptImuOnly(imu, impl_->config);

  // 构建状态日志条目
  StateDiagLog entry;
  entry.t = t;
  entry.dt = dt;
  entry.q = state.q;
  entry.v = state.v;
  Matrix3d Cbn = QuatToRot(state.q);
  entry.v_b = Cbn.transpose() * state.v;
  entry.omega = Vector3d::Zero();
  entry.f_b = Vector3d::Zero();
  if (dt > 1e-9) {
    entry.omega = imu.dtheta / dt - state.bg;
    entry.f_b = imu.dvel / dt - state.ba;
  }
  entry.static_imu = is_static_imu;
  entry.static_zupt = is_static_zupt;
  entry.has_truth = false;
  entry.dq_deg = 0.0;
  entry.dv = Vector3d::Zero();
  entry.dv_norm = 0.0;

  if (truth.timestamps.size() > 0) {
    int idx = FindNearestTruthIndex(truth.timestamps, t, impl_->truth_cursor_state);
    Vector4d q_truth = truth.quaternions.row(idx).transpose();
    Vector3d v_truth = truth.velocities.row(idx).transpose();
    entry.dq_deg = QuatDeltaAngleRad(q_truth, state.q) * kRadToDeg;
    entry.dv = state.v - v_truth;
    entry.dv_norm = entry.dv.norm();
    entry.has_truth = true;
  }

  // 首次发散检测
  if (!impl_->first_divergence_detected &&
      (impl_->first_div_dq_deg > 0 || impl_->first_div_dv > 0 || impl_->first_div_speed > 0)) {
    bool triggered = false;
    string reason;
    double val = 0.0, threshold = 0.0;

    if (entry.has_truth && impl_->first_div_dq_deg > 0 && entry.dq_deg >= impl_->first_div_dq_deg) {
      triggered = true; reason = "dq_deg"; val = entry.dq_deg; threshold = impl_->first_div_dq_deg;
    } else if (entry.has_truth && impl_->first_div_dv > 0 && entry.dv_norm >= impl_->first_div_dv) {
      triggered = true; reason = "dv_norm"; val = entry.dv_norm; threshold = impl_->first_div_dv;
    } else if (impl_->first_div_speed > 0 && entry.v.norm() >= impl_->first_div_speed) {
      triggered = true; reason = "speed"; val = entry.v.norm(); threshold = impl_->first_div_speed;
    }

    if (triggered) {
      impl_->first_divergence_detected = true;
      impl_->div_window_start = t - impl_->window_pre;
      impl_->div_window_end = t + impl_->window_post;
      impl_->div_window_active = true;
      cout << "[Diag] First divergence start t=" << t << " reason=" << reason
           << " value=" << val << " threshold=" << threshold;
      if (entry.has_truth) cout << " dq_deg=" << entry.dq_deg << " dv_norm=" << entry.dv_norm;
      cout << " v_norm=" << entry.v.norm() << "\n";
      cout << "[Diag] First divergence window t=" << impl_->div_window_start
           << " to " << impl_->div_window_end << "\n";

      impl_->ResetMeasLimiter(impl_->div_meas_limiter);
      impl_->ResetStateLimiter(impl_->div_state_limiter);
      if (!impl_->state_log_buffer.empty()) {
        cout << "[Diag] Pre-divergence state window (buffered)\n";
        impl_->FlushStateLog(impl_->div_window_start, t - 1e-9,
                              impl_->div_state_limiter, "First divergence window");
        impl_->ResetStateLimiter(impl_->div_state_limiter);
      }
      if (!impl_->meas_log_buffer.empty()) {
        cout << "[Diag] Pre-divergence measurement updates (buffered)\n";
        impl_->FlushMeasLogLimited("Pre-divergence ", impl_->div_window_start,
                                    t - 1e-9, impl_->div_meas_limiter);
      }
    }
  }

  // 缓存 + 窗口日志
  impl_->PushStateLog(entry);

  if (impl_->div_window_active && t + 1e-12 >= impl_->div_window_start &&
      t - 1e-12 <= impl_->div_window_end && ShouldLogState(impl_->div_state_limiter))
    LogStateWindow(entry, "First divergence window");

  if (impl_->drift_window_active && t + 1e-12 >= impl_->drift_window_start &&
      t - 1e-12 <= impl_->drift_window_end && ShouldLogState(impl_->drift_state_limiter))
    LogStateWindow(entry, "Drift window");

  if (impl_->div_window_active && t > impl_->div_window_end + 1e-9) {
    cout << "[Diag] First divergence window end t=" << impl_->div_window_end << "\n";
    impl_->div_window_active = false;
  }
  if (impl_->drift_window_active && t > impl_->drift_window_end + 1e-9) {
    cout << "[Diag] Drift window end t=" << impl_->drift_window_end << "\n";
    impl_->drift_window_active = false;
  }
}

// ---------- WriteDiagLine ----------
void DiagnosticsEngine::WriteDiagLine(double t, double dt, const EskfEngine &engine,
                                       const ImuData &imu, double last_odo_speed, double t0) {
  if (!impl_->enabled || !impl_->config.enable_diagnostics ||
      !impl_->diag_file.is_open() ||
      (t - impl_->last_diag_t) < Impl::kDiagInterval) {
    return;
  }
  impl_->last_diag_t = t;

  const auto &P = engine.cov();
  impl_->diag_file << (t - t0);

  // P 对角线标准差
  for (int j = 0; j < kStateDim; ++j)
    impl_->diag_file << " " << sqrt(max(0.0, P(j, j)));

  // 互相关系数
  auto corr = [&](int a, int b) -> double {
    double pa = P(a, a), pb = P(b, b);
    return (pa < 1e-30 || pb < 1e-30) ? 0.0 : P(a, b) / sqrt(pa * pb);
  };
  impl_->diag_file << " " << corr(21, 12) << " " << corr(21, 13)
                    << " " << corr(9, 7) << " " << corr(10, 6)
                    << " " << corr(11, 6) << " " << corr(11, 7)
                    << " " << corr(22, 7) << " " << corr(23, 8)
                    << " " << corr(21, 9);

  // 运动模式指标
  Matrix3d Cbn = QuatToRot(engine.state().q);
  Vector3d v_b = Cbn.transpose() * engine.state().v;
  double omega_z = (dt > 1e-9) ? (imu.dtheta.z() / dt - engine.state().bg.z()) : 0.0;
  impl_->diag_file << " " << v_b.x() << " " << omega_z << " " << last_odo_speed << "\n";
}

// ---------- Finalize ----------
void DiagnosticsEngine::Finalize(double end_t) {
  if (!impl_->enabled || !impl_->config.enable_diagnostics) return;

  if (impl_->drift_active) {
    cout << "[Diag] Gravity drift end t=" << end_t
         << " peak_diff_deg=" << impl_->drift_peak_deg << "\n";
  }
  if (!impl_->drift_diff_vals.empty()) {
    double sum = 0.0, min_v = impl_->drift_diff_vals[0], max_v = impl_->drift_diff_vals[0];
    for (double v : impl_->drift_diff_vals) { sum += v; min_v = min(min_v, v); max_v = max(max_v, v); }
    cout << "[Diag] Gravity compare summary count=" << impl_->drift_diff_vals.size()
         << " mean_deg=" << sum / impl_->drift_diff_vals.size()
         << " min_deg=" << min_v << " max_deg=" << max_v << "\n";
  } else {
    cout << "[Diag] Gravity compare summary: no samples\n";
  }
}
