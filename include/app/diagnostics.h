// 融合诊断引擎：封装所有诊断日志、漂移检测和 DIAG.txt 输出逻辑
#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>
#include "core/eskf.h"

// 前向声明（避免引入完整头文件）
struct State;
struct ImuData;
struct ConstraintConfig;
struct TruthData;
struct Dataset;
struct FusionOptions;
class EskfEngine;

/**
 * 诊断引擎：将融合主循环中的所有诊断逻辑封装为独立模块。
 * 包含：量测更新日志、重力对准检测、漂移/发散窗口管理、DIAG.txt 输出。
 * 使用 Pimpl 模式隐藏实现细节。
 */
class DiagnosticsEngine {
 public:
  DiagnosticsEngine(const ConstraintConfig &config, bool enabled);
  ~DiagnosticsEngine();

  // 禁止拷贝
  DiagnosticsEngine(const DiagnosticsEngine &) = delete;
  DiagnosticsEngine &operator=(const DiagnosticsEngine &) = delete;

  bool enabled() const;

  /** 融合开始前调用：离线重力检查 + 打开 DIAG.txt 并写表头 */
  void Initialize(const Dataset &dataset, const FusionOptions &options);

  /**
   * 执行量测更新并记录诊断日志。
   * 替换原来的 if(diag){Correct+log} else {Correct} 模式。
   */
  bool Correct(EskfEngine &engine, const std::string &tag, double t,
               const Eigen::VectorXd &y, const Eigen::MatrixXd &H,
               const Eigen::MatrixXd &R,
               const StateMask *update_mask = nullptr,
               const StateGainScale *gain_scale = nullptr,
               const StateMeasurementGainScale *gain_element_scale = nullptr);

  /** 记录一次 predict 调试快照（若配置了 predict_debug_output_path）。 */
  void LogPredict(const std::string &tag, const EskfEngine &engine);

  /** 重力对准静止窗口诊断（在 ZUPT 更新之后、NHC 更新之前调用） */
  void CheckGravityAlignment(double t, double dt, const ImuData &imu,
                              const State &state, const TruthData &truth);

  /** 每个 IMU 步结束后的状态诊断（漂移/发散检测） */
  void OnStepComplete(double t, double dt, const State &state,
                      const ImuData &imu, bool is_static_zupt,
                      const TruthData &truth);

  /** DIAG.txt 周期性输出一行 */
  void WriteDiagLine(double t, double dt, const EskfEngine &engine,
                     const ImuData &imu, double last_odo_speed, double t0);

  /** 融合结束后输出诊断摘要 */
  void Finalize(double end_t);

  /** NHC 跳过警告状态（避免重复打印） */
  bool nhc_skip_warned() const;
  void set_nhc_skip_warned(bool v);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
