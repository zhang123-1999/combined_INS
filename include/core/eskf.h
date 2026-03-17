// ESKF 核心：状态/噪声/IMU + 惯导传播 + 引擎
#pragma once

#include <array>
#include <cmath>
#include <Eigen/Dense>

#include "utils/math_utils.h"

using namespace std;
using namespace Eigen;

/**
 * ESKF 名义状态定义。
 * p/v/q 为位置速度与姿态四元数(wxyz)；ba/bg 为加速度计与陀螺零偏。
 */
struct State {
  // 位置、速度、姿态（四元数 wxyz）、加速度计/陀螺零偏
  Vector3d p{Vector3d::Zero()};
  Vector3d v{Vector3d::Zero()};
  Vector4d q{1.0, 0.0, 0.0, 0.0};
  Vector3d ba{Vector3d::Zero()};
  Vector3d bg{Vector3d::Zero()};
  // 陀螺/加速度计比例因子（无量纲，ppm 级别，如 1000ppm = 0.001）
  Vector3d sg{Vector3d::Zero()};
  Vector3d sa{Vector3d::Zero()};
  double odo_scale = 1.0;
  // 安装角误差（俯仰、航向，以及横滚）
  // 注：当前横滚在量测模型中默认按固定量处理，状态保留主要用于兼容接口。
  double mounting_roll = 0.0;   // rad
  double mounting_pitch = 0.0;  // rad
  double mounting_yaw = 0.0;    // rad
  // 杆臂误差（IMU→ODO，b系 FRD，单位 m）
  Vector3d lever_arm{Vector3d::Zero()};
  // GNSS杆臂误差（IMU→GNSS天线，b系 FRD，单位 m）- 新增
  Vector3d gnss_lever_arm{Vector3d::Zero()};
};

/**
 * InEKF 模式开关。
 * 说明：保留 FejManager 类型名仅用于兼容旧调用点；
 * 仅承载 InEKF 开关与 RI 参考原点（非 FEJ 冻结语义）。
 */
struct InEkfConfig {
  bool enabled = false;
  bool true_iekf_mode = false;
  bool apply_covariance_floor_after_reset = false;
  Vector3d p_init_ecef = Vector3d::Zero();  // RI 参考原点（Initialize 时设置）
  // RI GNSS 位置雅可比是否包含 p_ned_local 项。
  bool ri_gnss_pos_use_p_ned_local = true;
  // RI 速度-陀螺过程噪声映射模式：
  // -1: -Skew(v_ned) * C_bn（当前实现默认）
  //  0: 关闭该项
  // +1: +Skew(v_ned) * C_bn
  int ri_vel_gyro_noise_mode = -1;
  // RI 注入逆变换中，是否对位置执行 dr -= Skew(p_local) * dphi。
  bool ri_inject_pos_inverse = true;
  // Debug-only attribution switches. Accepted values:
  // - process: auto | eskf | true_iekf
  // - vel jacobian: auto | eskf | true_iekf | hybrid_zero
  string debug_force_process_model = "auto";
  string debug_force_vel_jacobian = "auto";
  bool debug_disable_true_reset_gamma = false;
  bool debug_enable_standard_reset_gamma = false;

  void Enable(bool flag) { enabled = flag; }
  bool IsEnabled() const { return enabled; }
  bool UseTrueInEkfMode() const { return enabled && true_iekf_mode; }
};

using FejManager = InEkfConfig;

/**
 * 过程与量测噪声参数。
 * sigma_acc/gyro/ba/bg 为连续时间噪声密度，sigma_uwb 为 UWB 量测标准差。
 */
struct NoiseParams {
  // 连续时间随机游走/白噪声密度
  double sigma_acc = 0.0;    // m/s^2/√Hz
  double sigma_gyro = 0.0;  // rad/s/√Hz
  double sigma_ba = 0.0;   // m/s^2/√Hz
  double sigma_bg = 0.0;   // rad/s/√Hz
  double sigma_sg = 0.0;   // 1/√Hz（陀螺比例因子过程噪声）
  double sigma_sa = 0.0;   // 1/√Hz（加速度计比例因子过程噪声）
  // 逐轴过程噪声；各元素 >= 0 时优先于对应标量字段，否则回退到标量字段
  Vector3d sigma_ba_vec = Vector3d::Constant(-1.0);
  Vector3d sigma_bg_vec = Vector3d::Constant(-1.0);
  Vector3d sigma_sg_vec = Vector3d::Constant(-1.0);
  Vector3d sigma_sa_vec = Vector3d::Constant(-1.0);
  double sigma_odo_scale = 0.0; // 1/√Hz
  double sigma_mounting = 0.0;  // rad/√Hz（安装角过程噪声，可设极小值）
  // 可选细分过程噪声；<0 表示回退到 sigma_mounting
  double sigma_mounting_roll = -1.0;   // rad/√Hz
  double sigma_mounting_pitch = -1.0;  // rad/√Hz
  double sigma_mounting_yaw = -1.0;    // rad/√Hz
  double sigma_lever_arm = 0.0; // m/√Hz（杆臂过程噪声）
  double sigma_gnss_lever_arm = 0.0; // m/√Hz（GNSS杆臂过程噪声）
  Vector3d sigma_lever_arm_vec = Vector3d::Constant(-1.0);
  Vector3d sigma_gnss_lever_arm_vec = Vector3d::Constant(-1.0);
  double sigma_uwb = 0.0;     // m
  double sigma_gnss_pos = 0.0; // m（GNSS位置量测噪声）
  // 一阶高斯马尔可夫相关时间（秒），适用于 ba/bg/sg/sa
  // >0 时启用马尔可夫模型：sigma 参数被解释为不稳定性（稳态标准差），
  //   驱动噪声自动计算为 σ_w = σ_ss * √(2/T)
  // ≤0 时使用随机游走模型：sigma 参数直接作为驱动噪声密度
  double markov_corr_time = 0.0;
};

// 状态维数定义: p(3)+v(3)+phi(3)+ba(3)+bg(3)+sg(3)+sa(3)+odo_scale(1)+mounting(3)+lever(3)+gnss_lever(3)=31
// 注: 横滚安装角(mounting_roll)保留在状态向量中以兼容接口；默认按固定量处理
constexpr int kStateDim = 31;
// 实际使用的状态维度(不含预留)
constexpr int kActualStateDim = 31;
using StateMask = std::array<bool, kStateDim>;

// 31 维状态索引枚举，量测模型、注入与状态掩码优先使用该索引访问。
struct StateIdx {
  static constexpr int kPos = 0;          // p (3)
  static constexpr int kVel = 3;          // v (3)
  static constexpr int kAtt = 6;          // phi (3)
  static constexpr int kBa = 9;           // ba (3)
  static constexpr int kBg = 12;          // bg (3)
  static constexpr int kSg = 15;          // sg (3)
  static constexpr int kSa = 18;          // sa (3)
  static constexpr int kOdoScale = 21;    // odo_scale (1)
  static constexpr int kMountRoll = 22;   // mounting_roll (1)
  static constexpr int kMountPitch = 23;  // mounting_pitch (1)
  static constexpr int kMountYaw = 24;    // mounting_yaw (1)
  static constexpr int kLever = 25;       // lever_arm (3)
  static constexpr int kGnssLever = 28;   // gnss_lever_arm (3)
};

/**
 * 外参/约束相关状态修正保护参数。
 * 用于限制单次修正步长并约束物理可行区间，防止异常量测触发连锁漂移。
 */
struct CorrectionGuard {
  bool enabled = false;
  double odo_scale_min = 0.5;
  double odo_scale_max = 1.5;
  double max_mounting_roll = 45.0 * EIGEN_PI / 180.0;
  double max_mounting_pitch = 30.0 * EIGEN_PI / 180.0;
  double max_mounting_yaw = 45.0 * EIGEN_PI / 180.0;
  double max_lever_arm_norm = 5.0;
  double max_odo_scale_step = 0.02;
  double max_mounting_step = 0.5 * EIGEN_PI / 180.0;
  double max_lever_arm_step = 0.05;
};

/**
 * 预测步协方差下界保护参数。
 * 仅在预测后对角线做 floor，防止长期重复约束导致协方差退化。
 */
struct CovarianceFloor {
  bool enabled = false;
  double pos_var = 0.01;   // m^2
  double vel_var = 0.001;  // (m/s)^2
  double att_var = (0.01 * EIGEN_PI / 180.0) * (0.01 * EIGEN_PI / 180.0);       // rad^2
  double mounting_var = (0.1 * EIGEN_PI / 180.0) * (0.1 * EIGEN_PI / 180.0);     // rad^2
  double bg_var = 1e-8;    // (rad/s)^2
};

struct TrueInEkfCorrectionSnapshot {
  bool valid = false;
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

struct CorrectionDebugSnapshot {
  bool valid = false;
  bool used_true_iekf = false;
  double t_state = 0.0;
  Matrix<double, kStateDim, kStateDim> P_prior =
      Matrix<double, kStateDim, kStateDim>::Zero();
  VectorXd y;
  MatrixXd H;
  MatrixXd R;
  MatrixXd S;
  MatrixXd K;
  Matrix<double, kStateDim, 1> dx = Matrix<double, kStateDim, 1>::Zero();
};

/**
 * 单帧 IMU 增量数据。
 * t/dt 为时间戳与采样间隔；dtheta/dvel 为角增量与比力增量。
 */
struct ImuData {
  // 时间戳、采样间隔与测得的角增量/比力增量
  double t = 0.0;
  double dt = 0.0;
  Vector3d dtheta{Vector3d::Zero()};
  Vector3d dvel{Vector3d::Zero()};
};

/**
 * 惯导传播结果。
 * state 为传播后的名义状态；Cbn 为姿态矩阵（C_b^n，body→nav）；f_b/omega_b 为中间量。
 */
struct PropagationResult {
  // 传播后的名义状态、姿态矩阵以及中间量
  State state;
  Matrix3d Cbn;   ///< C_b^n：将体系向量变换到导航系，即 v_n = Cbn * v_b
  Vector3d f_b;
  Vector3d omega_b;
};

/**
 * 纯惯导传播模型。
 * 负责根据 IMU 增量更新名义状态，并提供误差状态的过程模型。
 */
class InsMech {
 public:
  /**
   * 使用相邻两帧 IMU 增量完成一次姿态/速度/位置传播。
   * @param state 传播前状态
   * @param imu_prev 前一帧 IMU 增量
   * @param imu_curr 当前帧 IMU 增量
   * @return 传播后的名义状态与中间量
   */
  // 使用两帧 IMU 增量完成一次姿态/速度/位置传播
  static PropagationResult Propagate(const State &state, const ImuData &imu_prev,
                                     const ImuData &imu_curr);

  /**
   * 构建 NED 系下的完整误差状态转移矩阵与离散过程噪声。
   * 参考 kf-gins-docs.tex 中的误差微分方程。
   *
   * 误差状态（NED系）：[δr^n, δv^n, φ, ba, bg, sg, sa, ...]
   *
   * @param C_bn 姿态矩阵 C_b^n (body -> NED)
   * @param f_b 机体系比力
   * @param omega_ib_b 机体系角速度
   * @param v_ned NED速度
   * @param lat 纬度（弧度）
   * @param h 高度（米）
   * @param dt 时间步长
   * @param np 噪声参数
   * @param Phi 输出离散状态转移矩阵
   * @param Qd 输出离散过程噪声
   */
  static void BuildProcessModel(const Matrix3d &C_bn,
                                const Vector3d &f_b,
                                const Vector3d &omega_ib_b,
                                const Vector3d &v_ned,
                                double lat, double h, double dt,
                                const NoiseParams &np,
                                Matrix<double, kStateDim, kStateDim> &Phi,
                                Matrix<double, kStateDim, kStateDim> &Qd,
                                const FejManager *fej = nullptr);
};

/**
 * ESKF 引擎：维护 IMU 缓存、执行传播与通用量测更新。
 */
class EskfEngine {
 public:
  /**
   * 构造函数，保存噪声参数。
   */
  explicit EskfEngine(const NoiseParams &noise);

  /**
   * 设置初始状态与协方差。
   * @param state 初始名义状态
   * @param P0 初始协方差
   */
  // 设置初始状态与协方差
  void Initialize(const State &state, const Matrix<double, kStateDim, kStateDim> &P0);

  /**
   * 添加一帧 IMU 数据，内部维护 prev/curr。
   */
  // 添加一帧 IMU 数据（内部维护 prev/curr）
  void AddImu(const ImuData &imu);

  /**
   * 用缓存的 prev/curr 完成一次预测。
   * @return 是否成功完成传播
   */
  // 使用缓存的 prev+curr 完成一次预测
  bool Predict();

  /**
   * 通用量测更新接口。
   * @param y 残差向量
   * @param H 观测矩阵
   * @param R 量测噪声矩阵
   * @return 是否执行更新
   */
  // 通用量测更新：残差 y，观测矩阵 H，噪声 R
  bool Correct(const VectorXd &y, const MatrixXd &H,
               const MatrixXd &R, VectorXd *dx_out = nullptr,
               const StateMask *update_mask = nullptr);

  /**
   * 获取当前名义状态（只读引用）。
   */
  const State &state() const { return state_; }
  /**
   * 获取当前协方差矩阵（只读引用）。
   */
  const Matrix<double, kStateDim, kStateDim> &cov() const { return P_; }
  const TrueInEkfCorrectionSnapshot &last_true_iekf_correction() const {
    return last_true_iekf_correction_;
  }
  const CorrectionDebugSnapshot &last_correction_debug() const {
    return last_correction_debug_;
  }
  /**
   * 获取当前 IMU 时间戳。
   */
  double timestamp() const { return curr_imu_.t; }
  void SetFejManager(FejManager *fej) { fej_ = fej; }
  void SetStateMask(const StateMask &mask);
  void SetCorrectionGuard(const CorrectionGuard &guard) { correction_guard_ = guard; }
  void SetNoiseParams(const NoiseParams &noise) { noise_ = noise; }
  void SetCovarianceFloor(const CovarianceFloor &floor) { covariance_floor_ = floor; }

 private:
  // IMU 缓存状态机
  enum class ImuCacheState { Empty, HasCurr, Ready };

  /**
   * 计算卡尔曼增益 K。
   * @param H 观测矩阵
   * @param R 量测噪声
   * @return 卡尔曼增益矩阵
   */
  // 卡尔曼增益计算、误差注入、Joseph 协方差更新
  MatrixXd ComputeKalmanGain(const MatrixXd &H,
                             const MatrixXd &R) const;
  /**
   * 将误差状态注入名义状态。
   * @param dx 15 维误差状态增量
   */
  void InjectErrorState(const VectorXd &dx);
  /**
   * 使用 Joseph 形式更新协方差，保持对称正定。
   */
  void UpdateCovarianceJoseph(const MatrixXd &K, const MatrixXd &H,
                              const MatrixXd &R);
  void ApplyTrueInEkfReset(const VectorXd &dx);
  void ApplyStandardEskfReset(const VectorXd &dx);
  bool IsStateEnabledByMasks(int idx, const StateMask *update_mask) const;
  void ApplyStateMaskToDx(VectorXd &dx, const StateMask *update_mask) const;
  void ApplyUpdateMaskToKalmanGain(MatrixXd &K, const StateMask *update_mask) const;
  void ApplyStateMaskToCov();
  void ApplyCovarianceFloor();
  Matrix<double, kStateDim, kStateDim> BuildTrueInEkfResetGamma(
      const VectorXd &dx) const;
  Matrix<double, kStateDim, kStateDim> BuildStandardEskfResetGamma(
      const VectorXd &dx) const;

  NoiseParams noise_;
  State state_;
  Matrix<double, kStateDim, kStateDim> P_;

  ImuData prev_imu_{};
  ImuData curr_imu_{};
  ImuCacheState imu_state_{ImuCacheState::Empty};
  bool initialized_{false};
  FejManager *fej_{nullptr};
  StateMask state_mask_{};
  CorrectionGuard correction_guard_{};
  CovarianceFloor covariance_floor_{};
  TrueInEkfCorrectionSnapshot last_true_iekf_correction_{};
  CorrectionDebugSnapshot last_correction_debug_{};
};
