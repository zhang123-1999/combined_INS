// 融合应用接口：配置、初始化、管线与评估
#pragma once

#include <Eigen/Dense>
#include <string>
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
  Vector3d imu_mounting_angle{0.0, -0.532, 1.38}; // Mounting angle (deg) IMU->Vehicle
  double sigma_zupt = 0.05;
  double zupt_min_duration = 0.5;
  double zupt_max_speed = 0.2;
  double zupt_max_gyro = 0.05;
  double zupt_max_acc = 0.5;
  bool enable_diagnostics = false;
  double nhc_max_abs_v = 5.0;
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
  double p_floor_mounting_deg = 0.1;      // deg
  double p_floor_bg_var = 1.0e-8;         // (rad/s)^2
  // 弱激励阶段冻结 ODO/NHC 外参相关列，避免弱可观时错误收敛
  bool freeze_extrinsics_when_weak_excitation = true;
  double excitation_min_speed = 1.0;      // m/s
  double excitation_min_yaw_rate = 0.03;  // rad/s
  double excitation_min_lateral_acc = 0.3; // m/s^2
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
};

/**
 * InEKF 配置（复用原 FEJ 配置节）。
 * `enable=true` 表示启用 Right-Invariant EKF 模式。
 * 其余字段仅为兼容旧配置保留，当前实现中均为 no-op。
 */
struct FejConfig {
  bool enable = false;
  bool enable_layer2 = true;    // deprecated
  int imu_window_size = 100;    // deprecated
  double omega_threshold = 0.05;  // deprecated
  double accel_threshold = 0.5;   // deprecated
};

/**
 * 状态消融配置。
 * true 表示冻结对应状态块（不参与估计），用于减维对比实验。
 */
struct StateAblationConfig {
  bool disable_gnss_lever_arm = false;  // x[28:30]
  bool disable_odo_lever_arm = false;   // x[25:27]
  bool disable_odo_scale = false;       // x[21]
  bool disable_gyro_scale = false;      // x[15:17]
  bool disable_accel_scale = false;     // x[18:20]
  bool disable_mounting = false;        // x[22:24]
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
 * 初始化相关配置。
 * use_truth_pva 决定是否用真值初始化 p/v/q；
 * ba0/bg0 为加速度计/陀螺零偏初值；P0_diag 为初始协方差对角线。
 */
struct InitConfig {
  bool use_truth_pva = true;
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
  string pos_path = "POS.txt";
  string output_path = "SOL.txt";
  // 共享配置
  AnchorsConfig anchors;
  UwbAnchorSchedule uwb_anchor_schedule;
  GnssSchedule gnss_schedule;
  GatingConfig gating;
  ConstraintConfig constraints;
  FejConfig fej;
  StateAblationConfig ablation;
  PostGnssAblationConfig post_gnss_ablation;
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
 * 融合结果。
 * time_axis 为输出时间轴；fused_positions 为融合轨迹；fused_velocities 为速度；
 * fused_quaternions stores attitude quaternion (wxyz).
 */
struct FusionResult {
  vector<double> time_axis;
  vector<Vector3d> fused_positions;
  vector<Vector3d> fused_velocities;
  vector<Vector4d> fused_quaternions;
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

/**
 * 读取并组织全量数据集（IMU/UWB/真值/基站）。
 */
Dataset LoadDataset(const FusionOptions &options);

/**
 * 执行融合主流程，输出融合轨迹与纯惯导轨迹。
 */
FusionResult RunFusion(const FusionOptions &options, const Dataset &dataset,
                       const State &x0,
                       const Matrix<double, kStateDim, kStateDim> &P0);

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
