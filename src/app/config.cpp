#include "app/fusion.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;

namespace {

// State dimension for P0 (31 states)
constexpr int kP0Size = kStateDim;

/**
 * 抛出配置解析异常。
 * @param message 错误信息文本
 */
[[noreturn]] void ThrowConfigError(const string &message) {
  throw invalid_argument(message);
}

/**
 * 解析长度为 3 的向量字段。
 * @param node YAML 节点
 * @param field 字段名（用于报错提示）
 * @return 解析后的三维向量
 */
Vector3d ParseVector3(const YAML::Node &node, const string &field) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    ThrowConfigError("error: " + field + " 必须是长度为3的数组");
  }
  return Vector3d(node[0].as<double>(), node[1].as<double>(),
                  node[2].as<double>());
}

/**
 * 解析长度为 16 的向量字段（P0 对角线）。
 * @param node YAML 节点
 * @param field 字段名（用于报错提示）
 * @return 16x1 向量
 */
Matrix<double, kP0Size, 1> ParseVectorStateDim(const YAML::Node &node,
                                    const string &field) {
  if (!node || !node.IsSequence() || node.size() != kP0Size) {
    ThrowConfigError("error: " + field + " 必须是长度为" + to_string(kP0Size) + "的数组");
  }
  Matrix<double, kP0Size, 1> out;
  for (int i = 0; i < kP0Size; ++i) {
    out(i) = node[i].as<double>();
  }
  return out;
}

/**
 * 解析基站坐标数组。
 * @param node YAML 节点
 * @param field 字段名（用于报错提示）
 * @return 基站坐标列表
 */
vector<Vector3d> ParsePositions(const YAML::Node &node,
                                const string &field) {
  if (node.IsNull()) {
    return {};
  }
  if (!node || !node.IsSequence()) {
    ThrowConfigError("error: " + field + " 必须是坐标数组");
  }
  vector<Vector3d> positions;
  positions.reserve(node.size());
  for (size_t i = 0; i < node.size(); ++i) {
    positions.push_back(ParseVector3(node[i], field + "[" + to_string(i) + "]"));
  }
  return positions;
}

/**
 * 解析旧版基站字符串格式（x,y,z;...）。
 * @param text 字符串内容
 * @param field 字段名（用于报错提示）
 * @return 基站坐标列表
 */
vector<Vector3d> ParseAnchorsString(const string &text,
                                    const string &field) {
  vector<Vector3d> positions;
  if (text.empty()) {
    return positions;
  }
  stringstream ss(text);
  string token;
  while (getline(ss, token, ';')) {
    if (token.empty()) continue;
    stringstream ts(token);
    string num;
    vector<double> values;
    while (getline(ts, num, ',')) {
      if (!num.empty()) {
        values.push_back(stod(num));
      }
    }
    if (values.size() != 3) {
      ThrowConfigError("error: " + field + " 每个基站必须包含3个数字");
    }
    positions.emplace_back(values[0], values[1], values[2]);
  }
  return positions;
}

/**
 * Parse anchor index list (1-based in config, stored as 0-based).
 */
vector<int> ParseAnchorIndices(const YAML::Node &node, const string &field) {
  if (!node || !node.IsSequence()) {
    ThrowConfigError("error: " + field + " 必须是索引数组");
  }
  vector<int> indices;
  indices.reserve(node.size());
  for (size_t i = 0; i < node.size(); ++i) {
    int idx = node[i].as<int>();
    if (idx <= 0) {
      ThrowConfigError("error: " + field + " 必须为从1开始的正整数");
    }
    indices.push_back(idx - 1);
  }
  return indices;
}

/**
 * 校验基站配置有效性。
 * @param config 基站配置
 * @param context 错误上下文前缀
 */
void ValidateAnchorsConfig(const AnchorsConfig &config,
                           const string &context) {
  if (config.mode != "fixed" && config.mode != "auto") {
    ThrowConfigError("error: " + context +
                     ".anchors.mode 只能为 fixed 或 auto");
  }
  if (config.margin <= 0.0) {
    ThrowConfigError("error: " + context + ".anchors.margin 必须为正");
  }
  // 移除对 positions.size() == 4 的强制检查，允许任意非空数量
  // if (config.mode == "fixed" && config.positions.empty()) {
  //   ThrowConfigError("error: " + context +
  //                    ".anchors.positions 在 fixed 模式下不能为空");
  // }
}

/**
 * 将 YAML 中 anchors 节点应用到现有配置。
 * @param base 基础配置
 * @param node YAML 节点
 * @param context 错误上下文前缀
 * @return 合并后的配置
 */
AnchorsConfig ApplyAnchorsNode(const AnchorsConfig &base,
                               const YAML::Node &node,
                               const string &context) {
  AnchorsConfig out = base;
  if (!node) {
    return out;
  }
  if (node.IsScalar()) {
    string legacy = node.as<string>();
    if (!legacy.empty()) {
      out.positions = ParseAnchorsString(legacy, context + ".anchors");
      out.mode = "fixed";
    }
    return out;
  }
  if (!node.IsMap()) {
    ThrowConfigError("error: " + context + ".anchors 必须为 map 或字符串");
  }
  if (node["mode"]) {
    out.mode = node["mode"].as<string>();
  }
  if (node["margin"]) {
    out.margin = node["margin"].as<double>();
  }
  if (node["positions"]) {
    out.positions = ParsePositions(node["positions"],
                                   context + ".anchors.positions");
  }
  return out;
}

/**
 * Merge UWB anchor schedule node into config.
 */
UwbAnchorSchedule ApplyUwbAnchorScheduleNode(const UwbAnchorSchedule &base,
                                             const YAML::Node &node,
                                             const string &context) {
  UwbAnchorSchedule out = base;
  if (!node) {
    return out;
  }
  if (!node.IsMap()) {
    ThrowConfigError("error: " + context + ".uwb_anchor_schedule 必须为 map");
  }
  out.enabled = true;
  if (node["enabled"]) {
    out.enabled = node["enabled"].as<bool>();
  }
  if (node["head_ratio"]) {
    out.head_ratio = node["head_ratio"].as<double>();
  }
  if (node["head_anchors"]) {
    out.head_anchors = ParseAnchorIndices(
        node["head_anchors"], context + ".uwb_anchor_schedule.head_anchors");
  }
  if (node["tail_anchors"]) {
    out.tail_anchors = ParseAnchorIndices(
        node["tail_anchors"], context + ".uwb_anchor_schedule.tail_anchors");
  }
  return out;
}

/**
 * Validate UWB anchor schedule config.
 */
void ValidateUwbAnchorSchedule(const UwbAnchorSchedule &config,
                               const string &context) {
  if (!config.enabled) {
    return;
  }
  if (config.head_ratio <= 0.0 || config.head_ratio >= 1.0) {
    ThrowConfigError("error: " + context +
                     ".uwb_anchor_schedule.head_ratio 必须在 (0, 1) 之间");
  }
  // 允许为空，表示不使用基站
  // if (config.head_anchors.empty() || config.tail_anchors.empty()) {
  //   ThrowConfigError("error: " + context +
  //                    ".uwb_anchor_schedule 必须同时提供 head_anchors 与 tail_anchors");
  // }
}

/**
 * Merge GNSS schedule node into config.
 */
GnssSchedule ApplyGnssScheduleNode(const GnssSchedule &base,
                                   const YAML::Node &node,
                                   const string &context) {
  GnssSchedule out = base;
  if (!node) {
    return out;
  }
  if (!node.IsMap()) {
    ThrowConfigError("error: " + context + ".gnss_schedule 必须为 map");
  }
  out.enabled = true;
  if (node["enabled"]) {
    out.enabled = node["enabled"].as<bool>();
  }
  if (node["head_ratio"]) {
    out.head_ratio = node["head_ratio"].as<double>();
  }
  return out;
}

/**
 * Validate GNSS schedule config.
 */
void ValidateGnssSchedule(const GnssSchedule &config, const string &context) {
  if (!config.enabled) {
    return;
  }
  if (config.head_ratio <= 0.0 || config.head_ratio > 1.0) {
    ThrowConfigError("error: " + context +
                     ".gnss_schedule.head_ratio 必须在 (0, 1] 之间");
  }
}

/**
 * 将 gating 节点合并到配置中。
 * @param base 基础配置
 * @param node YAML 节点
 * @param context 错误上下文前缀
 * @return 合并后的配置
 */
GatingConfig ApplyGatingNode(const GatingConfig &base, const YAML::Node &node,
                             const string &context) {
  GatingConfig out = base;
  if (!node) {
    return out;
  }
  if (!node.IsMap()) {
    ThrowConfigError("error: " + context + ".gating 必须为 map");
  }
  if (node["uwb_residual_max"]) {
    out.uwb_residual_max = node["uwb_residual_max"].as<double>();
  }
  if (node["time_tolerance"]) {
    out.time_tolerance = node["time_tolerance"].as<double>();
  }
  if (node["max_dt"]) {
    double max_dt = node["max_dt"].as<double>();
    out.max_dt = max_dt;
  }
  return out;
}

/**
 * 校验 gating 配置有效性。
 * @param config 门控配置
 * @param context 错误上下文前缀
 */
void ValidateGatingConfig(const GatingConfig &config,
                          const string &context) {
  if (config.uwb_residual_max <= 0.0) {
    ThrowConfigError("error: " + context +
                     ".gating.uwb_residual_max 必须为正");
  }
  if (config.time_tolerance <= 0.0) {
    ThrowConfigError("error: " + context +
                     ".gating.time_tolerance 必须为正");
  }
}

/**
 * 将 constraints 节点合并到配置中。
 * @param base 基础配置
 * @param node YAML 节点
 * @param context 错误上下文前缀
 * @return 合并后的配置
 */
ConstraintConfig ApplyConstraintsNode(const ConstraintConfig &base,
                                      const YAML::Node &node,
                                      const string &context) {
  ConstraintConfig out = base;
  if (!node) {
    return out;
  }
  if (!node.IsMap()) {
    ThrowConfigError("error: " + context + ".constraints 必须为 map");
  }
  if (node["enable_nhc"]) {
    out.enable_nhc = node["enable_nhc"].as<bool>();
  }
  if (node["enable_zupt"]) {
    out.enable_zupt = node["enable_zupt"].as<bool>();
  }
  if (node["sigma_nhc_y"]) {
    out.sigma_nhc_y = node["sigma_nhc_y"].as<double>();
  }
  if (node["sigma_nhc_z"]) {
    out.sigma_nhc_z = node["sigma_nhc_z"].as<double>();
  }
  if (node["sigma_zupt"]) {
    out.sigma_zupt = node["sigma_zupt"].as<double>();
  }
  if (node["enable_odo"]) {
    out.enable_odo = node["enable_odo"].as<bool>();
  }
  if (node["sigma_odo"]) {
    out.sigma_odo = node["sigma_odo"].as<double>();
  }
  if (node["odo_lever_arm"]) {
    out.odo_lever_arm = ParseVector3(node["odo_lever_arm"], context + ".constraints.odo_lever_arm");
  }
  if (node["imu_mounting_angle"]) {
    out.imu_mounting_angle = ParseVector3(node["imu_mounting_angle"], context + ".constraints.imu_mounting_angle");
  }
  if (node["zupt_min_duration"]) {
    out.zupt_min_duration = node["zupt_min_duration"].as<double>();
  }
  if (node["zupt_max_speed"]) {
    out.zupt_max_speed = node["zupt_max_speed"].as<double>();
  }
  if (node["zupt_max_gyro"]) {
    out.zupt_max_gyro = node["zupt_max_gyro"].as<double>();
  }
  if (node["zupt_max_acc"]) {
    out.zupt_max_acc = node["zupt_max_acc"].as<double>();
  }
  if (node["enable_diagnostics"]) {
    out.enable_diagnostics = node["enable_diagnostics"].as<bool>();
  }
  if (node["nhc_max_abs_v"]) {
    out.nhc_max_abs_v = node["nhc_max_abs_v"].as<double>();
  }
  if (node["enable_nis_gating"]) {
    out.enable_nis_gating = node["enable_nis_gating"].as<bool>();
  }
  if (node["odo_nis_gate_prob"]) {
    out.odo_nis_gate_prob = node["odo_nis_gate_prob"].as<double>();
  }
  if (node["nhc_nis_gate_prob"]) {
    out.nhc_nis_gate_prob = node["nhc_nis_gate_prob"].as<double>();
  }
  if (node["enable_robust_weighting"]) {
    out.enable_robust_weighting = node["enable_robust_weighting"].as<bool>();
  }
  if (node["robust_kernel"]) {
    out.robust_kernel = node["robust_kernel"].as<string>();
  }
  if (node["robust_tuning"]) {
    out.robust_tuning = node["robust_tuning"].as<double>();
  }
  if (node["robust_min_weight"]) {
    out.robust_min_weight = node["robust_min_weight"].as<double>();
  }
  if (node["enable_covariance_floor"]) {
    out.enable_covariance_floor = node["enable_covariance_floor"].as<bool>();
  }
  if (node["p_floor_pos_var"]) {
    out.p_floor_pos_var = node["p_floor_pos_var"].as<double>();
  }
  if (node["p_floor_vel_var"]) {
    out.p_floor_vel_var = node["p_floor_vel_var"].as<double>();
  }
  if (node["p_floor_att_deg"]) {
    out.p_floor_att_deg = node["p_floor_att_deg"].as<double>();
  }
  if (node["p_floor_mounting_deg"]) {
    out.p_floor_mounting_deg = node["p_floor_mounting_deg"].as<double>();
  }
  if (node["p_floor_bg_var"]) {
    out.p_floor_bg_var = node["p_floor_bg_var"].as<double>();
  }
  if (node["freeze_extrinsics_when_weak_excitation"]) {
    out.freeze_extrinsics_when_weak_excitation =
        node["freeze_extrinsics_when_weak_excitation"].as<bool>();
  }
  if (node["excitation_min_speed"]) {
    out.excitation_min_speed = node["excitation_min_speed"].as<double>();
  }
  if (node["excitation_min_yaw_rate"]) {
    out.excitation_min_yaw_rate = node["excitation_min_yaw_rate"].as<double>();
  }
  if (node["excitation_min_lateral_acc"]) {
    out.excitation_min_lateral_acc = node["excitation_min_lateral_acc"].as<double>();
  }
  if (node["odo_time_offset"]) {
    out.odo_time_offset = node["odo_time_offset"].as<double>();
  }
  if (node["odo_min_update_interval"]) {
    out.odo_min_update_interval = node["odo_min_update_interval"].as<double>();
  }
  if (node["nhc_min_update_interval"]) {
    out.nhc_min_update_interval = node["nhc_min_update_interval"].as<double>();
  }
  if (node["enforce_extrinsic_bounds"]) {
    out.enforce_extrinsic_bounds = node["enforce_extrinsic_bounds"].as<bool>();
  }
  if (node["odo_scale_min"]) {
    out.odo_scale_min = node["odo_scale_min"].as<double>();
  }
  if (node["odo_scale_max"]) {
    out.odo_scale_max = node["odo_scale_max"].as<double>();
  }
  if (node["max_mounting_roll_deg"]) {
    out.max_mounting_roll_deg = node["max_mounting_roll_deg"].as<double>();
  }
  if (node["max_mounting_pitch_deg"]) {
    out.max_mounting_pitch_deg = node["max_mounting_pitch_deg"].as<double>();
  }
  if (node["max_mounting_yaw_deg"]) {
    out.max_mounting_yaw_deg = node["max_mounting_yaw_deg"].as<double>();
  }
  if (node["max_lever_arm_norm"]) {
    out.max_lever_arm_norm = node["max_lever_arm_norm"].as<double>();
  }
  if (node["max_odo_scale_step"]) {
    out.max_odo_scale_step = node["max_odo_scale_step"].as<double>();
  }
  if (node["max_mounting_step_deg"]) {
    out.max_mounting_step_deg = node["max_mounting_step_deg"].as<double>();
  }
  if (node["max_lever_arm_step"]) {
    out.max_lever_arm_step = node["max_lever_arm_step"].as<double>();
  }
  if (node["enable_consistency_log"]) {
    out.enable_consistency_log = node["enable_consistency_log"].as<bool>();
  }
  if (node["diag_gravity_min_duration"]) {
    out.diag_gravity_min_duration =
        node["diag_gravity_min_duration"].as<double>();
  }
  if (node["diag_meas_log_buffer"]) {
    out.diag_meas_log_buffer = node["diag_meas_log_buffer"].as<int>();
  }
  if (node["diag_meas_log_stride"]) {
    out.diag_meas_log_stride = node["diag_meas_log_stride"].as<int>();
  }
  if (node["diag_meas_log_max"]) {
    out.diag_meas_log_max = node["diag_meas_log_max"].as<int>();
  }
  if (node["diag_drift_window_pre"]) {
    out.diag_drift_window_pre = node["diag_drift_window_pre"].as<double>();
  }
  if (node["diag_drift_window_post"]) {
    out.diag_drift_window_post = node["diag_drift_window_post"].as<double>();
  }
  if (node["diag_state_log_stride"]) {
    out.diag_state_log_stride = node["diag_state_log_stride"].as<int>();
  }
  if (node["diag_first_divergence_dq_deg"]) {
    out.diag_first_divergence_dq_deg =
        node["diag_first_divergence_dq_deg"].as<double>();
  }
  if (node["diag_first_divergence_dv"]) {
    out.diag_first_divergence_dv =
        node["diag_first_divergence_dv"].as<double>();
  }
  if (node["diag_first_divergence_speed"]) {
    out.diag_first_divergence_speed =
        node["diag_first_divergence_speed"].as<double>();
  }
  return out;
}

/**
 * 将 fej 节点合并到配置中。
 */
FejConfig ApplyFejNode(const FejConfig &base, const YAML::Node &node,
                       const string &context) {
  FejConfig out = base;
  if (!node) {
    return out;
  }
  if (!node.IsMap()) {
    ThrowConfigError("error: " + context + ".fej 必须为 map");
  }
  if (node["enable"]) {
    out.enable = node["enable"].as<bool>();
  }
  if (node["true_iekf_mode"]) {
    out.true_iekf_mode = node["true_iekf_mode"].as<bool>();
  }
  if (node["enable_layer2"]) {
    out.enable_layer2 = node["enable_layer2"].as<bool>();
  }
  if (node["imu_window_size"]) {
    out.imu_window_size = node["imu_window_size"].as<int>();
  }
  if (node["omega_threshold"]) {
    out.omega_threshold = node["omega_threshold"].as<double>();
  }
  if (node["accel_threshold"]) {
    out.accel_threshold = node["accel_threshold"].as<double>();
  }
  if (node["ri_gnss_pos_use_p_ned_local"]) {
    out.ri_gnss_pos_use_p_ned_local =
        node["ri_gnss_pos_use_p_ned_local"].as<bool>();
  }
  if (node["ri_vel_gyro_noise_mode"]) {
    out.ri_vel_gyro_noise_mode = node["ri_vel_gyro_noise_mode"].as<int>();
  }
  if (node["ri_inject_pos_inverse"]) {
    out.ri_inject_pos_inverse = node["ri_inject_pos_inverse"].as<bool>();
  }
  return out;
}

/**
 * 将 ablation 节点合并到配置中。
 */
StateAblationConfig ApplyAblationNode(const StateAblationConfig &base,
                                      const YAML::Node &node,
                                      const string &context) {
  StateAblationConfig out = base;
  if (!node) {
    return out;
  }
  if (!node.IsMap()) {
    ThrowConfigError("error: " + context + ".ablation 必须为 map");
  }
  if (node["disable_gnss_lever_arm"]) {
    out.disable_gnss_lever_arm = node["disable_gnss_lever_arm"].as<bool>();
  }
  if (node["disable_odo_lever_arm"]) {
    out.disable_odo_lever_arm = node["disable_odo_lever_arm"].as<bool>();
  }
  if (node["disable_odo_scale"]) {
    out.disable_odo_scale = node["disable_odo_scale"].as<bool>();
  }
  if (node["disable_gyro_bias"]) {
    out.disable_gyro_bias = node["disable_gyro_bias"].as<bool>();
  }
  if (node["disable_gyro_scale"]) {
    out.disable_gyro_scale = node["disable_gyro_scale"].as<bool>();
  }
  if (node["disable_accel_scale"]) {
    out.disable_accel_scale = node["disable_accel_scale"].as<bool>();
  }
  if (node["disable_mounting"]) {
    out.disable_mounting = node["disable_mounting"].as<bool>();
  }
  return out;
}

/**
 * 将 post_gnss_ablation 节点合并到配置中。
 */
PostGnssAblationConfig ApplyPostGnssAblationNode(
    const PostGnssAblationConfig &base, const YAML::Node &node,
    const string &context) {
  PostGnssAblationConfig out = base;
  if (!node) {
    return out;
  }
  if (!node.IsMap()) {
    ThrowConfigError("error: " + context + ".post_gnss_ablation 必须为 map");
  }
  if (node["enabled"]) {
    out.enabled = node["enabled"].as<bool>();
  } else {
    out.enabled = true;
  }
  out.ablation = ApplyAblationNode(out.ablation, node, context + ".post_gnss_ablation");
  return out;
}

/**
 * 校验 constraints 配置有效性。
 * @param config 约束配置
 * @param context 错误上下文前缀
 */
void ValidateConstraintsConfig(const ConstraintConfig &config,
                               const string &context) {
  if (config.sigma_nhc_y <= 0.0 || config.sigma_nhc_z <= 0.0 ||
      config.sigma_zupt <= 0.0 || config.sigma_odo <= 0.0) {
    ThrowConfigError("error: " + context +
                     ".constraints sigma_* 必须为正");
  }
  if (config.zupt_min_duration < 0.0) {
    ThrowConfigError("error: " + context +
                     ".constraints zupt_min_duration 必须为非负");
  }
  if (config.zupt_max_speed <= 0.0 || config.zupt_max_gyro <= 0.0 ||
      config.zupt_max_acc <= 0.0) {
    ThrowConfigError("error: " + context +
                     ".constraints zupt_max_* 必须为正");
  }
  if (config.nhc_max_abs_v < 0.0) {
    ThrowConfigError("error: " + context +
                     ".constraints nhc_max_abs_v 必须为非负");
  }
  if (config.odo_nis_gate_prob <= 0.0 || config.odo_nis_gate_prob >= 1.0 ||
      config.nhc_nis_gate_prob <= 0.0 || config.nhc_nis_gate_prob >= 1.0) {
    ThrowConfigError("error: " + context +
                     ".constraints *_nis_gate_prob 必须在 (0,1) 内");
  }
  if (config.robust_tuning <= 0.0) {
    ThrowConfigError("error: " + context +
                     ".constraints robust_tuning 必须为正");
  }
  if (config.robust_min_weight <= 0.0 || config.robust_min_weight > 1.0) {
    ThrowConfigError("error: " + context +
                     ".constraints robust_min_weight 必须在 (0,1] 内");
  }
  if (config.robust_kernel != "huber" && config.robust_kernel != "cauchy") {
    ThrowConfigError("error: " + context +
                     ".constraints robust_kernel 仅支持 huber/cauchy");
  }
  if (config.p_floor_pos_var < 0.0 || config.p_floor_vel_var < 0.0 ||
      config.p_floor_att_deg < 0.0 || config.p_floor_mounting_deg < 0.0 ||
      config.p_floor_bg_var < 0.0) {
    ThrowConfigError("error: " + context +
                     ".constraints 协方差下界参数必须为非负");
  }
  if (config.excitation_min_speed < 0.0 || config.excitation_min_yaw_rate < 0.0 ||
      config.excitation_min_lateral_acc < 0.0) {
    ThrowConfigError("error: " + context +
                     ".constraints excitation_min_* 必须为非负");
  }
  if (config.odo_min_update_interval < 0.0 ||
      config.nhc_min_update_interval < 0.0) {
    ThrowConfigError("error: " + context +
                     ".constraints *_min_update_interval 必须为非负");
  }
  if (config.odo_scale_min <= 0.0 || config.odo_scale_max <= 0.0 ||
      config.odo_scale_min >= config.odo_scale_max) {
    ThrowConfigError("error: " + context +
                     ".constraints odo_scale_min/max 必须为正且 min < max");
  }
  if (config.max_mounting_roll_deg <= 0.0 || config.max_mounting_pitch_deg <= 0.0 ||
      config.max_mounting_yaw_deg <= 0.0) {
    ThrowConfigError("error: " + context +
                     ".constraints max_mounting_*_deg 必须为正");
  }
  if (config.max_lever_arm_norm <= 0.0 || config.max_odo_scale_step <= 0.0 ||
      config.max_mounting_step_deg <= 0.0 || config.max_lever_arm_step <= 0.0) {
    ThrowConfigError("error: " + context +
                     ".constraints 外参步长/边界参数必须为正");
  }
  if (config.diag_gravity_min_duration < 0.0) {
    ThrowConfigError("error: " + context +
                     ".constraints diag_gravity_min_duration 必须为非负");
  }
  if (config.diag_meas_log_buffer < 0) {
    ThrowConfigError("error: " + context +
                     ".constraints diag_meas_log_buffer 必须为非负");
  }
  if (config.diag_meas_log_stride <= 0) {
    ThrowConfigError("error: " + context +
                     ".constraints diag_meas_log_stride 必须为正");
  }
  if (config.diag_meas_log_max < 0) {
    ThrowConfigError("error: " + context +
                     ".constraints diag_meas_log_max 必须为非负");
  }
  if (config.diag_drift_window_pre < 0.0 ||
      config.diag_drift_window_post < 0.0) {
    ThrowConfigError("error: " + context +
                     ".constraints diag_drift_window_pre/post 必须为非负");
  }
  if (config.diag_state_log_stride <= 0) {
    ThrowConfigError("error: " + context +
                     ".constraints diag_state_log_stride 必须为正");
  }
  if (config.diag_first_divergence_dq_deg < 0.0 ||
      config.diag_first_divergence_dv < 0.0 ||
      config.diag_first_divergence_speed < 0.0) {
    ThrowConfigError("error: " + context +
                     ".constraints diag_first_divergence_* 必须为非负");
  }
}

void ValidateFejConfig(const FejConfig &config, const string &context) {
  if (!std::isfinite(config.omega_threshold) ||
      !std::isfinite(config.accel_threshold)) {
    ThrowConfigError("error: " + context +
                     ".fej.omega_threshold/accel_threshold 必须为有限数值");
  }
  if (config.ri_vel_gyro_noise_mode != -1 &&
      config.ri_vel_gyro_noise_mode != 0 &&
      config.ri_vel_gyro_noise_mode != 1) {
    ThrowConfigError("error: " + context +
                     ".fej.ri_vel_gyro_noise_mode 仅支持 -1/0/1");
  }
}

void ValidateInitConfig(const InitConfig &config, const string &context) {
  if (config.lever_arm_source != "constraints" &&
      config.lever_arm_source != "init") {
    ThrowConfigError("error: " + context +
                     ".init.lever_arm_source 仅支持 constraints/init");
  }
}

/**
 * 解析 init 节点并合并到配置中。
 * @param base 基础配置
 * @param node YAML 节点
 * @param context 错误上下文前缀
 * @return 合并后的配置
 */
InitConfig ApplyInitNode(const InitConfig &base, const YAML::Node &node,
                         const string &context) {
  InitConfig out = base;
  if (!node) {
    return out;
  }
  if (!node.IsMap()) {
    ThrowConfigError("error: " + context + ".init 必须为 map");
  }
  if (node["use_truth_pva"]) {
    out.use_truth_pva = node["use_truth_pva"].as<bool>();
  }
  if (node["ba0"]) {
    out.ba0 = ParseVector3(node["ba0"], context + ".init.ba0");
  }
  if (node["bg0"]) {
    out.bg0 = ParseVector3(node["bg0"], context + ".init.bg0");
  }
  if (node["sg0"]) {
    out.sg0 = ParseVector3(node["sg0"], context + ".init.sg0");
  }
  if (node["sa0"]) {
    out.sa0 = ParseVector3(node["sa0"], context + ".init.sa0");
  }

  // Parses new init parameters
  if (node["init_pos_lla"]) {
    out.init_pos_lla = ParseVector3(node["init_pos_lla"], context + ".init.init_pos_lla");
  }
  if (node["init_vel_ned"]) {
    out.init_vel_ned = ParseVector3(node["init_vel_ned"], context + ".init.init_vel_ned");
  }
  if (node["init_att_rpy"]) {
    out.init_att_rpy = ParseVector3(node["init_att_rpy"], context + ".init.init_att_rpy");
  }
  if (node["std_pos"]) {
    out.std_pos = ParseVector3(node["std_pos"], context + ".init.std_pos");
  }
  if (node["std_vel"]) {
    out.std_vel = ParseVector3(node["std_vel"], context + ".init.std_vel");
  }
  if (node["std_att"]) {
    out.std_att = ParseVector3(node["std_att"], context + ".init.std_att");
  }
  if (node["std_ba"]) {
    out.std_ba = ParseVector3(node["std_ba"], context + ".init.std_ba");
  }
  if (node["std_bg"]) {
    out.std_bg = ParseVector3(node["std_bg"], context + ".init.std_bg");
  }
  if (node["std_sg"]) {
    out.std_sg = ParseVector3(node["std_sg"], context + ".init.std_sg");
  }
  if (node["std_sa"]) {
    out.std_sa = ParseVector3(node["std_sa"], context + ".init.std_sa");
  }
  if (node["odo_scale"]) {
    out.odo_scale = node["odo_scale"].as<double>();
  }
  if (node["std_odo_scale"]) {
    out.std_odo_scale = node["std_odo_scale"].as<double>();
  }
  if (node["mounting_roll0"]) {
    out.mounting_roll0 = node["mounting_roll0"].as<double>();
  }
  if (node["mounting_pitch0"]) {
    out.mounting_pitch0 = node["mounting_pitch0"].as<double>();
  }
  if (node["mounting_yaw0"]) {
    out.mounting_yaw0 = node["mounting_yaw0"].as<double>();
  }
  if (node["std_mounting_roll"]) {
    out.std_mounting_roll = node["std_mounting_roll"].as<double>();
  }
  if (node["std_mounting_pitch"]) {
    out.std_mounting_pitch = node["std_mounting_pitch"].as<double>();
  }
  if (node["std_mounting_yaw"]) {
    out.std_mounting_yaw = node["std_mounting_yaw"].as<double>();
  }
  if (node["lever_arm0"]) {
    out.lever_arm0 = ParseVector3(node["lever_arm0"], context + ".init.lever_arm0");
  }
  if (node["std_lever_arm"]) {
    out.std_lever_arm = ParseVector3(node["std_lever_arm"], context + ".init.std_lever_arm");
  }
  if (node["gnss_lever_arm0"]) {
    out.gnss_lever_arm0 = ParseVector3(node["gnss_lever_arm0"], context + ".init.gnss_lever_arm0");
  }
  if (node["std_gnss_lever_arm"]) {
    out.std_gnss_lever_arm = ParseVector3(node["std_gnss_lever_arm"], context + ".init.std_gnss_lever_arm");
  }
  if (node["lever_arm_source"]) {
    out.lever_arm_source = node["lever_arm_source"].as<string>();
  }
  if (node["use_legacy_mounting_base_logic"]) {
    out.use_legacy_mounting_base_logic =
        node["use_legacy_mounting_base_logic"].as<bool>();
  }
  if (node["strict_extrinsic_conflict"]) {
    out.strict_extrinsic_conflict = node["strict_extrinsic_conflict"].as<bool>();
  }

  if (node["P0_diag"]) {
    out.P0_diag = ParseVectorStateDim(node["P0_diag"], context + ".init.P0_diag");
  }
  return out;
}

/**
 * 解析 noise 节点并合并到配置中。
 * @param base 基础噪声参数
 * @param node YAML 节点
 * @param context 错误上下文前缀
 * @return 合并后的噪声参数
 */
NoiseParams ApplyNoiseNode(const NoiseParams &base, const YAML::Node &node,
                           const string &context) {
  NoiseParams out = base;
  if (!node) {
    return out;
  }
  if (!node.IsMap()) {
    ThrowConfigError("error: " + context + ".noise 必须为 map");
  }
  if (node["sigma_uwb"]) {
    out.sigma_uwb = node["sigma_uwb"].as<double>();
  }
  if (node["sigma_acc"]) {
    out.sigma_acc = node["sigma_acc"].as<double>();
  }
  if (node["sigma_gyro"]) {
    out.sigma_gyro = node["sigma_gyro"].as<double>();
  }
  if (node["sigma_ba"]) {
    out.sigma_ba = node["sigma_ba"].as<double>();
  }
  if (node["sigma_bg"]) {
    out.sigma_bg = node["sigma_bg"].as<double>();
  }
  if (node["sigma_odo_scale"]) {
    out.sigma_odo_scale = node["sigma_odo_scale"].as<double>();
  }
  if (node["sigma_mounting"]) {
    out.sigma_mounting = node["sigma_mounting"].as<double>();
  }
  if (node["sigma_mounting_roll"]) {
    out.sigma_mounting_roll = node["sigma_mounting_roll"].as<double>();
  }
  if (node["sigma_mounting_pitch"]) {
    out.sigma_mounting_pitch = node["sigma_mounting_pitch"].as<double>();
  }
  if (node["sigma_mounting_yaw"]) {
    out.sigma_mounting_yaw = node["sigma_mounting_yaw"].as<double>();
  }
  if (node["sigma_sg"]) {
    out.sigma_sg = node["sigma_sg"].as<double>();
  }
  if (node["sigma_sa"]) {
    out.sigma_sa = node["sigma_sa"].as<double>();
  }
  if (node["markov_corr_time"]) {
    out.markov_corr_time = node["markov_corr_time"].as<double>();
  }
  if (node["sigma_lever_arm"]) {
    out.sigma_lever_arm = node["sigma_lever_arm"].as<double>();
  }
  if (node["sigma_gnss_lever_arm"]) {
    out.sigma_gnss_lever_arm = node["sigma_gnss_lever_arm"].as<double>();
  }
  if (node["sigma_gnss_pos"]) {
    out.sigma_gnss_pos = node["sigma_gnss_pos"].as<double>();
  }
  return out;
}

/**
 * 校验噪声参数是否为正。
 * @param noise 噪声参数
 * @param context 错误上下文前缀
 */
void ValidateNoiseParams(const NoiseParams &noise, const string &context) {
  if (noise.sigma_acc <= 0.0 || noise.sigma_gyro <= 0.0 ||
      noise.sigma_ba <= 0.0 || noise.sigma_bg <= 0.0 ||
      noise.sigma_uwb <= 0.0 || noise.sigma_odo_scale <= 0.0) {
    ThrowConfigError("error: " + context +
                     ".noise sigma_* 必须全部为正");
  }
  // sigma_mounting 允许为 0（随机常数模型，无过程噪声）
  auto valid_optional_mounting = [](double v) {
    return v >= 0.0 || std::abs(v + 1.0) < 1e-12;
  };
  if (!valid_optional_mounting(noise.sigma_mounting_roll) ||
      !valid_optional_mounting(noise.sigma_mounting_pitch) ||
      !valid_optional_mounting(noise.sigma_mounting_yaw)) {
    ThrowConfigError("error: " + context +
                     ".noise sigma_mounting_roll/sigma_mounting_pitch/sigma_mounting_yaw 必须为非负，或 -1 表示回退");
  }
}

/**
 * 解析 anchors 配置，合并 common 与 override 节点。
 * @param common 通用配置节点
 * @param override_node 覆盖配置节点
 * @param context 错误上下文前缀
 * @return 完整的 AnchorsConfig
 */
AnchorsConfig ParseAnchorsConfig(const YAML::Node &common,
                                 const YAML::Node &override_node,
                                 const string &context) {
  AnchorsConfig config;
  config = ApplyAnchorsNode(config, common, context);
  config = ApplyAnchorsNode(config, override_node, context);
  ValidateAnchorsConfig(config, context);
  return config;
}

/**
 * 解析 gating 配置，合并 common 与 override 节点。
 * @param common 通用配置节点
 * @param override_node 覆盖配置节点
 * @param context 错误上下文前缀
 * @return 完整的 GatingConfig
 */
GatingConfig ParseGatingConfig(const YAML::Node &common,
                               const YAML::Node &override_node,
                               const string &context) {
  GatingConfig config;
  config = ApplyGatingNode(config, common, context);
  config = ApplyGatingNode(config, override_node, context);
  ValidateGatingConfig(config, context);
  return config;
}

}  // namespace

/**
 * 读取融合配置文件并解析为 FusionOptions。
 * @param path 配置文件路径
 * @return 解析后的配置，失败时返回默认值
 */
FusionOptions LoadFusionOptions(const string &path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const exception &) {
    cout << "error: 无法读取配置文件 " << path << "\n";
    return FusionOptions();
  }

  if (!root["fusion"]) {
    cout << "error: 配置文件中缺少 fusion 节点\n";
    return FusionOptions();
  }

  FusionOptions opt;
  YAML::Node common = root["common"];
  YAML::Node f = root["fusion"];

  opt.imu_path = f["imu_path"] ? f["imu_path"].as<string>() : opt.imu_path;
  opt.uwb_path = f["uwb_path"] ? f["uwb_path"].as<string>() : opt.uwb_path;
  opt.pos_path = f["pos_path"] ? f["pos_path"].as<string>() : opt.pos_path;
  opt.odo_path = f["odo_path"] ? f["odo_path"].as<string>() : opt.odo_path;
  opt.gnss_path = f["gnss_path"] ? f["gnss_path"].as<string>() : opt.gnss_path;
  opt.enable_gnss_velocity = f["enable_gnss_velocity"]
                                 ? f["enable_gnss_velocity"].as<bool>()
                                 : opt.enable_gnss_velocity;
  opt.output_path =
      f["output_path"] ? f["output_path"].as<string>() : opt.output_path;
  opt.start_time = f["starttime"] ? f["starttime"].as<double>() : opt.start_time;
  opt.final_time = f["finaltime"] ? f["finaltime"].as<double>() : opt.final_time;

  opt.anchors = ParseAnchorsConfig(common ? common["anchors"] : YAML::Node(),
                                   f["anchors"], "fusion");
  opt.uwb_anchor_schedule =
      ApplyUwbAnchorScheduleNode(opt.uwb_anchor_schedule,
                                 f["uwb_anchor_schedule"], "fusion");
  opt.gnss_schedule =
      ApplyGnssScheduleNode(opt.gnss_schedule, f["gnss_schedule"], "fusion");
  opt.gating = ParseGatingConfig(common ? common["gating"] : YAML::Node(),
                                 f["gating"], "fusion");
  opt.constraints = ApplyConstraintsNode(opt.constraints, f["constraints"],
                                         "fusion");
  opt.fej = ApplyFejNode(opt.fej, f["fej"], "fusion");
  opt.ablation = ApplyAblationNode(opt.ablation, f["ablation"], "fusion");
  opt.post_gnss_ablation = ApplyPostGnssAblationNode(
      opt.post_gnss_ablation, f["post_gnss_ablation"], "fusion");
  opt.init = ApplyInitNode(opt.init, f["init"], "fusion");
  opt.noise = ApplyNoiseNode(opt.noise, f["noise"], "fusion");

  ValidateUwbAnchorSchedule(opt.uwb_anchor_schedule, "fusion");
  ValidateGnssSchedule(opt.gnss_schedule, "fusion");
  ValidateNoiseParams(opt.noise, "fusion");
  ValidateConstraintsConfig(opt.constraints, "fusion");
  ValidateFejConfig(opt.fej, "fusion");
  ValidateInitConfig(opt.init, "fusion");

  return opt;
}

/**
 * 读取 UWB 生成器配置文件并解析为 GeneratorOptions。
 * @param path 配置文件路径
 * @return 解析后的配置，失败时返回默认值
 */
GeneratorOptions LoadGeneratorOptions(const string &path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const exception &) {
    cout << "error: 无法读取配置文件 " << path << "\n";
    return GeneratorOptions();
  }

  if (!root["generator"]) {
    cout << "error: 配置文件中缺少 generator 节点\n";
    return GeneratorOptions();
  }

  GeneratorOptions opt;
  YAML::Node common = root["common"];
  YAML::Node g = root["generator"];

  opt.pos_path = g["pos_path"] ? g["pos_path"].as<string>() : opt.pos_path;
  opt.output_path =
      g["output_path"] ? g["output_path"].as<string>() : opt.output_path;
  opt.uwb_hz = g["uwb_hz"] ? g["uwb_hz"].as<double>() : opt.uwb_hz;
  opt.sigma = g["sigma"] ? g["sigma"].as<double>() : opt.sigma;
  opt.seed = g["seed"] ? g["seed"].as<unsigned int>() : opt.seed;

  if (opt.uwb_hz <= 0.0) {
    ThrowConfigError("error: generator.uwb_hz 必须为正");
  }
  if (opt.sigma <= 0.0) {
    ThrowConfigError("error: generator.sigma 必须为正");
  }

  opt.anchors = ParseAnchorsConfig(common ? common["anchors"] : YAML::Node(),
                                   g["anchors"], "generator");
  opt.gating = ParseGatingConfig(common ? common["gating"] : YAML::Node(),
                                 g["gating"], "generator");

  return opt;
}

/**
 * 根据基站配置构建 Anchors。
 * fixed 模式直接使用 positions，auto 模式根据轨迹自动布站。
 */
Anchors BuildAnchors(const AnchorsConfig &config, const MatrixXd &pos_xyz) {
  if (config.mode == "fixed") {
    if (config.positions.empty()) {
      // 允许空基站，用于纯惯导
      return Anchors{};
    }
    int n = config.positions.size();
    Anchors anchors;
    // 使用动态大小矩阵
    anchors.positions.resize(n, 3);
    for (int i = 0; i < n; ++i) {
      anchors.positions.row(i) = config.positions[i].transpose();
    }
    return anchors;
  }
  if (config.mode == "auto") {
    if (pos_xyz.rows() == 0) {
      ThrowConfigError("error: 自动布站需要非空轨迹");
    }
    return AutoPlaceAnchors(pos_xyz, config.margin);
  }
  ThrowConfigError("error: anchors.mode 只能为 fixed 或 auto");
}
