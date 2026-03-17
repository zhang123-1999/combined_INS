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

  bool ShouldLogMechanism(const string &tag) {
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
          << "h_col_att_z_norm,h_col_bg_z_norm,h_col_mount_yaw_norm,"
          << "h_block_att_norm,h_block_bg_norm,h_block_mount_norm,"
          << "info_att_z,info_bg_z,info_mount_yaw,info_heading_trace,"
          << "bg_z_before,bg_z_after,mount_yaw_before,mount_yaw_after\n";
    }
  }
}

// ---------- Correct ----------
bool DiagnosticsEngine::Correct(EskfEngine &engine, const string &tag, double t,
                                const VectorXd &y, const MatrixXd &H, const MatrixXd &R,
                                const StateMask *update_mask) {
  if (!impl_->enabled) {
    return engine.Correct(y, H, R, nullptr, update_mask);
  }

  // 捕获修正前状态（用于 NHC 的 dv/dq 额外日志）
  State state_before = engine.state();
  Vector3d v_before = engine.state().v;
  Vector4d q_before = engine.state().q;

  VectorXd dx;
  bool updated = engine.Correct(y, H, R, &dx, update_mask);

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
      impl_->ShouldLogMechanism(tag)) {
    bool post_gnss =
        std::isfinite(impl_->gnss_split_t) && t > impl_->gnss_split_t + impl_->gnss_tol;
    if (!impl_->config.mechanism_log_post_gnss_only || post_gnss) {
      const auto &snap = engine.last_correction_debug();
      if (snap.valid) {
        const MatrixXd H_att = snap.H.block(0, StateIdx::kAtt, snap.H.rows(), 3);
        const MatrixXd H_bg = snap.H.block(0, StateIdx::kBg, snap.H.rows(), 3);
        const MatrixXd H_mount =
            snap.H.block(0, StateIdx::kMountRoll, snap.H.rows(), 3);
        const VectorXd h_att_z = snap.H.col(StateIdx::kAtt + 2);
        const VectorXd h_bg_z = snap.H.col(StateIdx::kBg + 2);
        const VectorXd h_mount_yaw = snap.H.col(StateIdx::kMountYaw);
        MatrixXd H_heading(snap.H.rows(), 3);
        H_heading.col(0) = h_att_z;
        H_heading.col(1) = h_bg_z;
        H_heading.col(2) = h_mount_yaw;
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
            << state_before.bg.z() << "," << engine.state().bg.z() << ","
            << state_before.mounting_yaw << "," << engine.state().mounting_yaw
            << "\n";
      }
    }
  }
  return updated;
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
