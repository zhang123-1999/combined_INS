// 融合应用接口：配置、初始化、管线与评估
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "core/eskf.h"
#include "core/uwb.h"
#include "utils/math_utils.h"

using namespace std;
using namespace Eigen;

/**
 * 基站配置参数。
 * mode 表示布站方式（fixed/auto）；margin 是自动布站的外扩边距；
 * positions 保存四个基站的坐标列表，固定布站时必须给定。
 */
struct AnchorsConfig {
  // fixed | auto
  string mode = "auto";
  double margin = 1.0;
  vector<Vector3d> positions;
};

/**
 * UWB anchor schedule config.
 * head_ratio splits the UWB time range into head/tail segments.
 */
struct UwbAnchorSchedule {
  bool enabled = false;
  double head_ratio = 0.0;
  vector<int> head_anchors; // 0-based after parsing
  vector<int> tail_anchors; // 0-based after parsing
};

/**
 * GNSS 时间调度配置。
 * enabled=true 时，仅在融合时间轴前 head_ratio 段启用 GNSS 更新。
 */
struct GnssSchedule {
  bool enabled = false;
  double head_ratio = 1.0;
  vector<pair<double, double>> enabled_windows;
};

/**
 * 运行期约束覆盖。
 * has_* 为 true 时覆盖对应约束开关。
 */
struct RuntimeConstraintOverride {
  bool has_enable_nhc = false;
  bool enable_nhc = false;
  bool has_enable_odo = false;
  bool enable_odo = false;
  bool has_enable_covariance_floor = false;
  bool enable_covariance_floor = false;
  bool has_gnss_pos_update_mode = false;
  string gnss_pos_update_mode;
  bool has_enable_nis_gating = false;
  bool enable_nis_gating = false;
  bool has_odo_nis_gate_prob = false;
  double odo_nis_gate_prob = 0.99;
  bool has_nhc_nis_gate_prob = false;
  double nhc_nis_gate_prob = 0.995;
  bool has_p_floor_mounting_deg = false;
  double p_floor_mounting_deg = 0.1;
  bool has_p_floor_odo_scale_var = false;
  double p_floor_odo_scale_var = 0.0;
  bool has_p_floor_lever_arm_vec = false;
  Vector3d p_floor_lever_arm_vec = Vector3d::Zero();
};

/**
 * 运行期过程噪声覆盖。
 * NaN 表示保持原值不变；向量三轴均有限时覆盖对应字段。
 */
struct RuntimeNoiseOverride {
  double sigma_acc = std::numeric_limits<double>::quiet_NaN();
  double sigma_gyro = std::numeric_limits<double>::quiet_NaN();
  double sigma_ba = std::numeric_limits<double>::quiet_NaN();
  double sigma_bg = std::numeric_limits<double>::quiet_NaN();
  double sigma_sg = std::numeric_limits<double>::quiet_NaN();
  double sigma_sa = std::numeric_limits<double>::quiet_NaN();
  Vector3d sigma_ba_vec =
      Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Vector3d sigma_bg_vec =
      Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Vector3d sigma_sg_vec =
      Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Vector3d sigma_sa_vec =
      Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double sigma_odo_scale = std::numeric_limits<double>::quiet_NaN();
  double sigma_mounting = std::numeric_limits<double>::quiet_NaN();
  double sigma_mounting_roll = std::numeric_limits<double>::quiet_NaN();
  double sigma_mounting_pitch = std::numeric_limits<double>::quiet_NaN();
  double sigma_mounting_yaw = std::numeric_limits<double>::quiet_NaN();
  double sigma_lever_arm = std::numeric_limits<double>::quiet_NaN();
  double sigma_gnss_lever_arm = std::numeric_limits<double>::quiet_NaN();
  Vector3d sigma_lever_arm_vec =
      Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Vector3d sigma_gnss_lever_arm_vec =
      Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double sigma_uwb = std::numeric_limits<double>::quiet_NaN();
  double sigma_gnss_pos = std::numeric_limits<double>::quiet_NaN();
  double markov_corr_time = std::numeric_limits<double>::quiet_NaN();
  bool has_disable_nominal_ba_bg_decay = false;
  bool disable_nominal_ba_bg_decay = false;
};

/**
 * 量测门控参数。
 * uwb_residual_max 控制 UWB 残差阈值；time_tolerance 用于时间对齐；
 * max_dt 为 IMU 异常步长阈值，<=0 表示不启用。
 */
struct GatingConfig {
  double uwb_residual_max = 3.0;
  double time_tolerance = 1.0e-6;
  // <= 0 表示不启用异常 dt 门限
  double max_dt = -1.0;
};

/**
 * 伪量测约束配置。
 * NHC 约束车体侧向/垂向速度；ZUPT 在静止时将速度约束为 0。
 */
struct ConstraintConfig {
  bool enable_nhc = false;
  bool enable_odo = false; 
  bool enable_zupt = false;
  double sigma_nhc_y = 0.2;
  double sigma_nhc_z = 0.2;
  double sigma_odo = 0.1;
  Vector3d odo_lever_arm{0.605, -1.025, 0.705}; // ODO lever arm in b-frame (FRD)
  Vector3d imu_mounting_angle{0.0, -0.532, 1.38}; // Stored mounting angle (deg), runtime C_b^v uses inverse rotation
  double sigma_zupt = 0.05;
  double zupt_min_duration = 0.5;
  double zupt_max_speed = 0.2;
  double zupt_max_gyro = 0.05;
  double zupt_max_acc = 0.5;
  bool enable_diagnostics = false;
  double nhc_max_abs_v = 5.0;
  // 当车体前向速度过低时，直接停用 NHC，避免低速起步阶段的错误约束主导。
  double nhc_disable_below_forward_speed = 0.0;  // m/s, <=0 表示不启用
  // NHC admission 使用的速度口径：v_b / v_wheel_b / v_v。
  string nhc_admission_velocity_source = "v_b";
  // 使用已有 weak-excitation 判据直接停用 NHC/ODO，便于做联合激励门控实验。
  bool disable_nhc_when_weak_excitation = false;
  bool disable_odo_when_weak_excitation = false;
  // ODO/NHC 一致性与鲁棒更新配置
  bool enable_nis_gating = true;
  // NIS 卡方门控置信度（ODO=1维，NHC=2维）
  double odo_nis_gate_prob = 0.99;
  double nhc_nis_gate_prob = 0.995;
  bool enable_robust_weighting = true;
  // 支持 "huber" / "cauchy"
  string robust_kernel = "huber";
  // 鲁棒核阈值（白化残差范数域）
  double robust_tuning = 2.5;
  double robust_min_weight = 0.1;
  // 预测后协方差下界保护
  bool enable_covariance_floor = true;
  double p_floor_pos_var = 0.01;          // m^2
  double p_floor_vel_var = 0.001;         // (m/s)^2
  double p_floor_att_deg = 0.01;          // deg
  double p_floor_odo_scale_var = 0.0;     // dimensionless^2
  Vector3d p_floor_lever_arm_vec = Vector3d::Zero(); // m^2
  double p_floor_mounting_deg = 0.1;      // deg
  double p_floor_bg_var = 1.0e-8;         // (rad/s)^2
  // 弱激励阶段冻结 ODO/NHC 外参相关列，避免弱可观时错误收敛
  bool freeze_extrinsics_when_weak_excitation = true;
  double excitation_min_speed = 1.0;      // m/s
  double excitation_min_yaw_rate = 0.03;  // rad/s
  double excitation_min_lateral_acc = 0.3; // m/s^2
  // bg_z 可观性门控：仅在存在足够前向速度与 turning 激励时放开 ODO/NHC->bg_z
  bool enable_bgz_observability_gate = false;
  bool bgz_gate_apply_to_odo = true;
  bool bgz_gate_apply_to_nhc = true;
  double bgz_gate_forward_speed_min = 3.0;     // m/s
  double bgz_gate_yaw_rate_min_deg_s = 8.0;    // deg/s
  double bgz_gate_lateral_acc_min = 0.3;       // m/s^2
  double bgz_gate_min_scale = 0.0;             // [0,1]
  // 低激励段对 P(bg_z,*) 的非对角项施加遗忘，削弱错误互协方差累积
  bool enable_bgz_covariance_forgetting = false;
  double bgz_cov_forgetting_tau_s = 15.0;      // s
  // ODO 时间偏移补偿（秒，测量时间 + offset）
  double odo_time_offset = 0.0;
  // 约束最小更新间隔（秒），用于削弱高频相关伪量测的过度主导
  double odo_min_update_interval = 0.02;
  double nhc_min_update_interval = 0.02;
  // 状态保护：限制每次修正步长并施加物理边界
  bool enforce_extrinsic_bounds = true;
  double odo_scale_min = 0.5;
  double odo_scale_max = 1.5;
  double max_mounting_roll_deg = 45.0;
  double max_mounting_pitch_deg = 30.0;
  double max_mounting_yaw_deg = 45.0;
  double max_lever_arm_norm = 5.0;
  double max_odo_scale_step = 0.02;
  double max_mounting_step_deg = 0.5;
  double max_lever_arm_step = 0.05;
  // 一致性摘要日志
  bool enable_consistency_log = true;
  // 记录 NHC precheck 在三种速度口径下的 admission 决策，便于离线对齐分析。
  bool enable_nhc_admission_log = false;
  // 机理归因日志：为 ODO/NHC 的每次成功更新记录 heading 相关信息量与修正量。
  bool enable_mechanism_log = false;
  int mechanism_log_stride = 1;
  bool mechanism_log_post_gnss_only = true;
  double mechanism_log_start_time =
      std::numeric_limits<double>::quiet_NaN();
  double mechanism_log_end_time =
      std::numeric_limits<double>::quiet_NaN();
  double diag_gravity_min_duration = 0.5;
  int diag_meas_log_buffer = 200;
  int diag_meas_log_stride = 10;
  int diag_meas_log_max = 300;
  double diag_drift_window_pre = 2.0;
  double diag_drift_window_post = 2.0;
  int diag_state_log_stride = 10;
  double diag_first_divergence_dq_deg = 5.0;
  double diag_first_divergence_dv = 5.0;
  double diag_first_divergence_speed = 50.0;
  // Research-only switches for isolating ODO/NHC interaction mechanisms.
  bool debug_odo_disable_bgz_jacobian = false;
  bool debug_odo_disable_bgz_state_update = false;
  bool debug_nhc_disable_bgz_state_update = false;
  bool debug_run_odo_before_nhc = false;
  double debug_nhc_disable_start_time =
      std::numeric_limits<double>::quiet_NaN();
  double debug_nhc_disable_end_time =
      std::numeric_limits<double>::quiet_NaN();
  double debug_odo_disable_start_time =
      std::numeric_limits<double>::quiet_NaN();
  double debug_odo_disable_end_time =
      std::numeric_limits<double>::quiet_NaN();
  double debug_gnss_lever_arm_disable_start_time =
      std::numeric_limits<double>::quiet_NaN();
  double debug_gnss_lever_arm_disable_end_time =
      std::numeric_limits<double>::quiet_NaN();
  double debug_nhc_enable_after_time =
      std::numeric_limits<double>::quiet_NaN();
  double debug_mounting_yaw_enable_after_time =
      std::numeric_limits<double>::quiet_NaN();
  double debug_reset_bg_z_state_and_cov_after_time =
      std::numeric_limits<double>::quiet_NaN();
  double debug_reset_bg_z_value =
      std::numeric_limits<double>::quiet_NaN();
  double debug_seed_mount_yaw_bgz_cov_before_first_nhc =
      std::numeric_limits<double>::quiet_NaN();
  double debug_seed_bg_z_before_first_nhc =
      std::numeric_limits<double>::quiet_NaN();
  Vector3d debug_seed_bg_z_att_cov_before_first_nhc =
      Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
};

/**
 * InEKF 配置（复用原 FEJ 配置节）。
 * `enable=true` 表示启用 Right-Invariant EKF 模式。
 * 其余字段仅为兼容旧配置保留，当前实现中均为 no-op。
 */
struct FejConfig {
  bool enable = false;
  bool true_iekf_mode = false;
  bool apply_covariance_floor_after_reset = false;
  bool enable_layer2 = true;    // deprecated
  int imu_window_size = 100;    // deprecated
  double omega_threshold = 0.05;  // deprecated
  double accel_threshold = 0.5;   // deprecated
  // RI GNSS 位置雅可比中是否包含 p_ned_local 项。
  bool ri_gnss_pos_use_p_ned_local = true;
  // RI 速度-陀螺噪声映射模式：-1(默认), 0(关闭), +1(正号)。
  int ri_vel_gyro_noise_mode = -1;
  // RI 注入逆变换中，位置项 dr -= Skew(p_local)*dphi 是否启用。
  bool ri_inject_pos_inverse = true;
  // 机理归因调试开关。
  string debug_force_process_model = "auto";
  string debug_force_vel_jacobian = "auto";
  bool debug_disable_true_reset_gamma = false;
  bool debug_enable_standard_reset_gamma = false;
};

/**
 * 状态消融配置。
 * true 表示冻结对应状态块（不参与估计），用于减维对比实验。
 */
struct StateAblationConfig {
  bool disable_gnss_lever_arm = false;  // x[28:30]
  bool disable_gnss_lever_z = false;    // x[30]
  bool disable_odo_lever_arm = false;   // x[25:27]
  bool disable_odo_scale = false;       // x[21]
  bool disable_accel_bias = false;      // x[9:11]
  bool disable_gyro_bias = false;       // x[12:14]
  bool disable_gyro_scale = false;      // x[15:17]
  bool disable_accel_scale = false;     // x[18:20]
  bool disable_mounting = false;        // x[22:24]
  bool disable_mounting_roll = false;   // x[22]
  bool disable_mounting_pitch = false;  // x[23]
  bool disable_mounting_yaw = false;    // x[24]
};

/**
 * GNSS 结束后的状态冻结配置。
 * ablation 指定需要在 GNSS 结束后追加冻结的状态块。
 */
struct PostGnssAblationConfig {
  bool enabled = false;
  StateAblationConfig ablation;
};

/**
 * 运行期阶段控制。
 * 在指定时间窗内叠加状态冻结、约束开关与过程噪声覆盖。
 */
struct RuntimePhaseConfig {
  bool enabled = true;
  string name;
  double start_time = std::numeric_limits<double>::quiet_NaN();
  double end_time = std::numeric_limits<double>::quiet_NaN();
  StateAblationConfig ablation;
  RuntimeConstraintOverride constraints;
  RuntimeNoiseOverride noise;
};

/**
 * 初始化相关配置。
 * use_truth_pva 决定是否用真值初始化 p/v/q；
 * runtime_truth_anchor_pva 为 true 时，会在运行期持续用真值锚定选定的 P/V/A 分量；
 * runtime_truth_anchor_position/velocity/attitude 控制锚定分量；
 * runtime_truth_anchor_gnss_only 为 true 时，仅在成功 GNSS 更新后执行锚定；
 * ba0/bg0 为加速度计/陀螺零偏初值；
 * 若显式提供 `P0_diag`，则其优先于 `std_*` 字段直接构造初始协方差对角线。
 */
struct InitConfig {
  bool use_truth_pva = true;
  bool runtime_truth_anchor_pva = false;
  bool runtime_truth_anchor_position = true;
  bool runtime_truth_anchor_velocity = true;
  bool runtime_truth_anchor_attitude = true;
  bool runtime_truth_anchor_gnss_only = false;
  // 手动初始值
  Vector3d init_pos_lla{Vector3d::Zero()}; // deg, deg, m
  Vector3d init_vel_ned{Vector3d::Zero()}; // m/s
  Vector3d init_att_rpy{Vector3d::Zero()}; // deg (Roll, Pitch, Yaw)
  Vector3d ba0{Vector3d::Zero()};
  Vector3d bg0{Vector3d::Zero()};
  Vector3d sg0{Vector3d::Zero()};  // 陀螺比例因子初值 (无量纲, 如 0.001 = 1000ppm)
  Vector3d sa0{Vector3d::Zero()};  // 加速度计比例因子初值
  double odo_scale = 1.0;
  double mounting_roll0 = 0.0;   // 初始安装横滚角 (deg)
  double mounting_pitch0 = 0.0;  // 初始安装俯仰角 (deg)
  double mounting_yaw0 = 0.0;    // 初始安装航向角 (deg)
  Vector3d lever_arm0{0.605, -1.025, 0.705};  // 杆臂初值 (m, FRD)

  // 初始误差标准差 (用于构建 P0)
  Vector3d std_pos{0.1, 0.1, 0.1};
  Vector3d std_vel{0.1, 0.1, 0.1};
  Vector3d std_att{0.1, 0.1, 0.1};
  Vector3d std_ba{0.01, 0.01, 0.01};
  Vector3d std_bg{0.01, 0.01, 0.01};
  Vector3d std_sg{0.001, 0.001, 0.001};  // 比例因子初始 STD (无量纲)
  Vector3d std_sa{0.001, 0.001, 0.001};
  double std_odo_scale = 0.05;
  double std_mounting_roll = 0.5;   // deg
  double std_mounting_pitch = 0.5;  // deg
  double std_mounting_yaw = 0.5;    // deg
  Vector3d std_lever_arm{0.1, 0.1, 0.1};      // ODO杆臂初始 STD (m)
  Vector3d gnss_lever_arm0{Vector3d::Zero()};  // GNSS天线杆臂初值 (m, b系)
  Vector3d std_gnss_lever_arm{0.1, 0.1, 0.1}; // GNSS天线杆臂初始 STD (m)
  // 外参配置语义控制
  // constraints | init
  string lever_arm_source = "constraints";
  // true 时沿用“init 安装角为 0 才叠加 constraints 基准”的旧逻辑
  bool use_legacy_mounting_base_logic = true;
  // true 时 init 与 constraints 外参冲突将直接报错
  bool strict_extrinsic_conflict = false;

  bool has_custom_P0_diag = false;
  Matrix<double, kStateDim, 1> P0_diag =
      (Matrix<double, kStateDim, 1>() <<
       // [0-2] pos
       0.1, 0.1, 0.1,
       // [3-5] vel
       0.1, 0.1, 0.1,
       // [6-8] att
       0.01, 0.01, 0.01,
       // [9-11] ba
       0.01, 0.01, 0.01,
       // [12-14] bg
       0.01, 0.01, 0.01,
       // [15-17] sg
       0.001, 0.001, 0.001,
       // [18-20] sa
       0.001, 0.001, 0.001,
       // [21] odo_scale
       0.01,
       // [22] mounting_roll
       0.5,
       // [23-24] mounting_pitch, mounting_yaw
       0.5, 0.5,
       // [25-27] lever_arm
       0.1, 0.1, 0.1,
       // [28-30] gnss_lever_arm
       0.1, 0.1, 0.1)
          .finished();
};

/**
 * 融合主配置。
 * imu/uwb/pos/output 为文件路径；anchors/gating/init/noise 为子模块配置；
 * start_time/final_time 控制时间裁剪范围（绝对时间）。
 */
struct FusionOptions {
  // IMU/ODO/UWB/GNSS/真值/输出文件路径
  string odo_path = "iODO.txt";
  string imu_path = "IMU.txt";
  string uwb_path = "UWB_simulated.txt";
  string gnss_path = "";       // GNSS数据路径 (可选)
  // 仅控制是否执行 GNSS_VEL 量测更新；GNSS_POS 的位置时间对齐若数据集自带速度列，仍可独立使用。
  bool enable_gnss_velocity = true;
  string pos_path = "POS.txt";
  string output_path = "SOL.txt";
  string state_series_output_path = "";
  string first_update_debug_output_path = "";
  string gnss_update_debug_output_path = "";
  string predict_debug_output_path = "";
  double predict_debug_start_time = std::numeric_limits<double>::quiet_NaN();
  double predict_debug_end_time = std::numeric_limits<double>::quiet_NaN();
  // GNSS_POS 更新策略（研究/诊断用途）：
  // - joint: 单次联合更新（默认）
  // - stage_nonpos_then_pos: 先让 non-position 状态吸收 innovation，再做 position-only 补更新
  // - position_only: 仅允许位置状态吸收 GNSS_POS innovation
  string gnss_pos_update_mode = "joint";
  // 仅对 GNSS_POS 更新生效：缩放 position(0:2) 状态对应的 Kalman 增益行。
  // 1.0 = 默认行为；0.0 = 完全抑制 GNSS_POS 对 position 的直接校正。
  double gnss_pos_position_gain_scale = 1.0;
  // 仅对 GNSS_POS 更新生效：缩放 `K(l_gx, meas_y)` 这一路径。
  // 用于研究 turning 条件下 `x` 杆臂是否主要被 `y` 向 innovation 污染。
  double gnss_pos_lgx_from_y_gain_scale = 1.0;
  // 仅对 GNSS_POS 更新生效：缩放 `K(l_gy, meas_y)` 这一路径。
  // 用于研究 `y` 杆臂的 same-axis 通道是否主要在特定 turn direction 下给出错误更新。
  double gnss_pos_lgy_from_y_gain_scale = 1.0;
  // 条件化 turning 缓解：使用运行时 IMU `omega_z` 作为 turn proxy。
  // 当 |omega_z| 超过阈值（deg/s）时，可按符号覆盖 GNSS_POS 的 position gain。
  // 负值表示不启用该方向的条件化覆盖。
  double gnss_pos_turn_rate_threshold_deg_s = 0.0;
  double gnss_pos_positive_turn_position_gain_scale = -1.0;
  double gnss_pos_negative_turn_position_gain_scale = -1.0;
  double gnss_pos_positive_turn_lgy_from_y_gain_scale = -1.0;
  double gnss_pos_negative_turn_lgy_from_y_gain_scale = -1.0;
  // 共享配置
  AnchorsConfig anchors;
  UwbAnchorSchedule uwb_anchor_schedule;
  GnssSchedule gnss_schedule;
  GatingConfig gating;
  ConstraintConfig constraints;
  FejConfig fej;
  StateAblationConfig ablation;
  PostGnssAblationConfig post_gnss_ablation;
  vector<RuntimePhaseConfig> runtime_phases;
  InitConfig init;
  // 噪声参数统一放在 NoiseParams 中
  NoiseParams noise;
  // 数据截取时间范围（绝对时间），留默认值则不裁剪
  double start_time = -1e18;
  double final_time = 1e18;
};

/**
 * UWB 数据生成器配置。
 * pos/output 为真值输入与输出路径；uwb_hz/sigma/seed 控制采样与噪声；
 * anchors/gating 复用基站与时间门控设置。
 */
struct GeneratorOptions {
  string pos_path = "POS.txt";
  string output_path = "UWB_simulated.txt";
  double uwb_hz = 10.0;
  double sigma = 0.1;
  unsigned int seed = 42;
  AnchorsConfig anchors;
  GatingConfig gating;
};

/**
 * 从配置文件读取融合参数。
 * @param path 配置文件路径
 * @return 解析后的 FusionOptions，失败时返回默认值
 */
FusionOptions LoadFusionOptions(const string &path);
/**
 * 从配置文件读取 UWB 生成器参数。
 * @param path 配置文件路径
 * @return 解析后的 GeneratorOptions，失败时返回默认值
 */
GeneratorOptions LoadGeneratorOptions(const string &path);

/**
 * 根据配置构建基站坐标。
 * fixed 模式使用 config.positions；auto 模式根据轨迹 pos_xyz 自动布站。
 */
Anchors BuildAnchors(const AnchorsConfig &config, const MatrixXd &pos_xyz);

/**
 * 真值轨迹数据。
 * timestamps 为时间轴；positions/velocities/quaternions 为对应的真值序列。
 */
struct TruthData {
  VectorXd timestamps;
  MatrixXd positions;    // Nx3
  MatrixXd velocities;   // Nx3
  MatrixXd quaternions;  // Nx4 (w, x, y, z)
};

/**
 * 按时间 t 对真值 p/v/q 做线性/归一化四元数插值。
 * cursor 作为可复用游标；越界时钳位到首/尾样本。
 */
bool InterpolateTruthPva(const TruthData &truth, double t, int &cursor,
                         Vector3d &p_out, Vector3d &v_out, Vector4d &q_out);

/**
 * 初始化 ESKF 状态与协方差。
 * 根据 options 决定是否使用真值 p/v/q，并写入 ba/bg 与 P0。
 */
bool InitializeState(const FusionOptions &options, const vector<ImuData> &imu,
                     const TruthData &truth, State &x0,
                     Matrix<double, kStateDim, kStateDim> &P0);

/**
 * GNSS数据。
 * timestamps 为时间轴；positions 为LLA位置 (lat, lon, height)；
 * std 为标准差 (north, east, down)；
 * velocities 为NED速度 (vn, ve, vd)；
 * vel_std 为速度标准差。
 */
struct GnssData {
  VectorXd timestamps;
  MatrixXd positions;    // Nx3 (lat, lon, height) in degrees and meters
  MatrixXd std;          // Nx3 (north, east, down) in meters
  MatrixXd velocities;   // Nx3 (vn, ve, vd) in m/s
  MatrixXd vel_std;      // Nx3 (sigma_vn, sigma_ve, sigma_vd) in m/s
};

/**
 * 融合数据集封装。
 * imu 为 IMU 序列；uwb 为 UWB 矩阵；truth 为真值；anchors 为基站坐标；
 * gnss 为GNSS数据。
 */
struct Dataset {
  vector<ImuData> imu;
  MatrixXd odo; // t, v
  MatrixXd uwb;
  TruthData truth;
  Anchors anchors;
  GnssData gnss;
};

/**
 * 判断数据集中是否存在与时间轴对齐的 GNSS 速度与速度标准差。
 * 该判断只描述“数据是否可用”，不涉及是否启用 GNSS_VEL 更新。
 */
inline bool HasGnssVelocityData(const Dataset &dataset) {
  return dataset.gnss.timestamps.size() > 0 &&
         dataset.gnss.velocities.rows() == dataset.gnss.timestamps.size() &&
         dataset.gnss.vel_std.rows() == dataset.gnss.timestamps.size();
}

inline bool IsTimestampAlignedToReference(double timestamp,
                                          double reference,
                                          double tolerance) {
  return std::abs(timestamp - reference) <= std::max(0.0, tolerance);
}

/**
 * 读取并按 t_curr 对齐 GNSS 位置量测。
 * 优先使用数据集自带 GNSS 速度做位置时间外推；
 * 若输入仅有 7 列位置数据，则回退到相邻 GNSS 位置的有限差分速度。
 */
bool ComputeAlignedGnssPositionMeasurement(const Dataset &dataset,
                                          const NoiseParams &noise,
                                          int gnss_idx, double t_curr,
                                          Vector3d &gnss_pos_ecef,
                                          Vector3d &gnss_std_out);

/**
 * 融合结果。
 * time_axis 为输出时间轴；fused_positions 为融合轨迹；fused_velocities 为速度；
 * fused_quaternions stores attitude quaternion (wxyz).
 */
struct FusionResult {
  vector<double> time_axis;
  vector<Vector3d> fused_positions;
  vector<Vector3d> fused_velocities;
  vector<Vector4d> fused_quaternions;
  vector<double> mounting_roll;   // 安装横滚角 (rad)
  vector<double> mounting_pitch;  // 安装俯仰角 (rad)
  vector<double> mounting_yaw;    // 安装航向角 (rad)
  vector<double> odo_scale;       // 里程计比例因子
  vector<Vector3d> sg;            // 陀螺比例因子
  vector<Vector3d> sa;            // 加速度计比例因子
  vector<Vector3d> ba;            // 加速度计零偏
  vector<Vector3d> bg;            // 陀螺零偏
  vector<Vector3d> lever_arm;     // 杆臂估计值
  vector<Vector3d> gnss_lever_arm;  // GNSS天线杆臂估计值
};

struct GnssSplitCovarianceCapture {
  bool valid = false;
  string tag;
  double split_t = 0.0;
  double t_meas = 0.0;
  double t_state = 0.0;
  Vector3d P_att_bgz{Vector3d::Zero()};
  Vector3d corr_att_bgz{Vector3d::Zero()};
  Vector3d att_var{Vector3d::Zero()};
  double bgz_var = 0.0;
};

struct ResetConsistencyCapture {
  bool valid = false;
  string tag;
  double split_t = 0.0;
  double t_meas = 0.0;
  double t_state = 0.0;
  Matrix<double, kStateDim, kStateDim> P_tilde =
      Matrix<double, kStateDim, kStateDim>::Zero();
  Matrix<double, kStateDim, kStateDim> P_after_reset =
      Matrix<double, kStateDim, kStateDim>::Zero();
  Matrix<double, kStateDim, kStateDim> P_after_all =
      Matrix<double, kStateDim, kStateDim>::Zero();
  Matrix<double, kStateDim, 1> dx = Matrix<double, kStateDim, 1>::Zero();
  bool covariance_floor_applied = false;
};

struct FusionDebugCapture {
  bool capture_last_gnss_before_split = false;
  GnssSplitCovarianceCapture gnss_split_cov;
  ResetConsistencyCapture reset_consistency;
};

/**
 * 读取并组织全量数据集（IMU/UWB/真值/基站）。
 */
Dataset LoadDataset(const FusionOptions &options);

/**
 * 执行融合主流程，输出融合轨迹与纯惯导轨迹。
 */
FusionResult RunFusion(const FusionOptions &options, const Dataset &dataset,
                       const State &x0,
                       const Matrix<double, kStateDim, kStateDim> &P0,
                       FusionDebugCapture *debug_capture = nullptr);

/**
 * 评估输出摘要。
 * rmse_fused 为融合 RMSE，output_matrix 用于保存结果。
 */
struct EvaluationSummary {
  Vector3d rmse_fused{Vector3d::Zero()};
  MatrixXd output_matrix;
};

/**
 * 计算 RMSE 与输出矩阵，用于评估融合效果。
 */
EvaluationSummary EvaluateFusion(const FusionResult &result,
                                 const Dataset &dataset);

/**
 * 将评估结果保存为文本矩阵文件。
 */
void SaveFusionResult(const string &path, const EvaluationSummary &summary);

/**
 * 导出实验专用状态序列文件。
 */
void SaveStateSeries(const string &path, const FusionResult &result,
                     const FusionOptions &options);
