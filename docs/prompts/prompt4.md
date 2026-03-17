````md
# 代码修改提示词：扩展 config.yaml 以实现“全局可控调参”，消除参数分散

你是一名资深 C++ 工程师。请在**不改变算法输出（除浮点微小差异）**的前提下，对当前 VSCode + CMake 项目继续做一次“配置集中化”重构：让 **config.yaml 成为唯一的全局参数入口**，避免在多个 .cpp/.h 中硬编码初值、阈值与环境参数（例如 UWB 四基站坐标、门控阈值、初始化 P0、时间对齐容差等）。

---

## 0. 强约束（必须遵守）
- 不引入新第三方库（现有 yaml-cpp / Eigen 可继续使用）。
- 工程应保持可编译可运行（CMake 一键构建）。
- 保持现有模块分层：smech（计算）/ engine（调度）/ app（流程）。
- 所有“可调参数”必须集中到 config.yaml；代码内仅保留**合理默认值**（用于配置缺失时 fallback）。
- 需要做参数合法性检查（维度、范围、枚举字符串），并给出清晰错误信息。

---

## 1) 需要收敛到 config.yaml 的参数清单（必须完成）
### 1.1 UWB 基站（Anchors）
- 支持两种模式：
  - `fixed`：直接从 YAML 读 `positions`（4x3）
  - `auto`：从轨迹范围自动布设（需要 `margin`）
- 任何基站坐标、margin、基站数量都不能再在代码里硬编码。

### 1.2 门控与时间对齐（Gating & Sync）
- UWB 残差门控阈值（例如 `uwb_residual_max`）
- 时间对齐容差（例如 `time_tolerance`，替换所有 `1e-6` 类魔法数）
-（可选）dt 异常阈值（例如 `max_dt`）

### 1.3 初始化（Init）
- 是否使用真值首点初始化（`use_truth_pva`）
- 初始偏置 `ba0/bg0`
- 初始协方差 `P0_diag`（长度必须为 31；若显式提供，则应优先于 `std_*`）
-（可选）静止对准窗口长度（若你有“前N秒对准”策略）

### 1.4 噪声参数（Noise）
- IMU 噪声与随机游走（如 `sigma_acc/sigma_gyro/sigma_ba/sigma_bg`）
- UWB 测距噪声（`sigma_uwb`）

### 1.5 UWB Generator（生成器参数）
- 生成频率 `uwb_hz`
- 噪声 `sigma`
- 随机种子 `seed`
- generator 也应复用同一套 anchors 配置（默认使用 `common.anchors`，允许 generator 覆写）

---

## 2) 推荐的 config.yaml 结构（请按此实现解析）
请将 YAML 结构扩展为 “common + fusion + generator”，并实现“common 默认 + 子模块覆写”：

```yaml
common:
  anchors:
    mode: fixed            # fixed | auto
    margin: 1.0            # mode=auto 生效
    positions:
      - [0.0, 0.0, 1.5]
      - [10.0, 0.0, 1.5]
      - [0.0, 10.0, 1.5]
      - [10.0, 10.0, 1.5]

  gating:
    uwb_residual_max: 3.0
    time_tolerance: 1.0e-6

fusion:
  imu_path: IMU.txt
  uwb_path: UWB_simulated.txt
  pos_path: POS.txt
  output_path: SOL.txt
  starttime: -1e18
  finaltime: 1e18

  noise:
    sigma_acc: 0.05
    sigma_gyro: 0.005
    sigma_ba: 0.0005
    sigma_bg: 0.0005
    sigma_uwb: 0.1

  init:
    use_truth_pva: true
    ba0: [0, 0, 0]
    bg0: [0, 0, 0]
    P0_diag: [0.1,0.1,0.1, 0.1,0.1,0.1, 0.01,0.01,0.01,
              1e-6,1e-6,1e-6, 1e-6,1e-6,1e-6,
              2.5e-7,2.5e-7,2.5e-7, 2.5e-7,2.5e-7,2.5e-7,
              2.5e-5, 1.0e-4,0.09,0.01, 0.01,0.01,0.01, 0.01,0.01,0.01]

generator:
  pos_path: POS.txt
  output_path: UWB_simulated.txt
  uwb_hz: 10.0
  sigma: 0.1
  seed: 42
  # 可选覆写 anchors（若不写则继承 common.anchors）
  # anchors:
  #   mode: auto
  #   margin: 2.0
````

---

## 3) 代码改动要求（必须按最小侵入方式实现）

### 3.1 扩展 Options 结构体（集中在 app/config.*）

新增结构体并注入到现有 Options：

* `struct AnchorsConfig { std::string mode; double margin; std::vector<Eigen::Vector3d> positions; };`
* `struct GatingConfig { double uwb_residual_max; double time_tolerance; };`
* `struct InitConfig { bool use_truth_pva; Eigen::Vector3d ba0, bg0; Eigen::Matrix<double,31,1> P0_diag; };`

将它们作为字段加入：

* `FusionOptions.anchors / FusionOptions.gating / FusionOptions.init`
* `GeneratorOptions.anchors（可选）` 或让 GeneratorOptions 也携带 `AnchorsConfig`

实现“common 默认 + 子块覆写”的解析逻辑：

* `ParseAnchorsConfig(common_node, fusion_node)`（fusion 下 anchors 可覆写 common）
* `ParseGatingConfig(common_node, fusion_node)`
* generator 同理

### 3.2 统一配置加载入口（消除 generator 的重复 LoadConfig）

* 将 generator 的 `LoadConfig()` 合并到 `app/config.*`，提供：

  * `FusionOptions LoadFusionOptions(yaml_path)`
  * `GeneratorOptions LoadGeneratorOptions(yaml_path)`
* 两者共享 anchors/gating 的解析函数（避免两套实现）

### 3.3 彻底替换所有硬编码阈值

请全局搜索并替换以下类别魔法数为 Options 字段：

* 时间容差：`1e-6` → `options.gating.time_tolerance`
* UWB 残差门控：`3.0` → `options.gating.uwb_residual_max`
* auto anchors margin：`1.0` → `options.anchors.margin`
* seed/uwb_hz/sigma：从 generator options 读取
* `P0.diagonal() << ...`：若显式给出 `options.init.P0_diag`，则以其为准；否则从 `std_*` 构建

### 3.4 anchors 逻辑统一

* 若 `anchors.mode == fixed`：使用 YAML 中 `positions`
* 若 `anchors.mode == auto`：使用 `AutoPlaceAnchors(traj, anchors.margin)`
* 禁止再使用 `anchor_str` 或分散的 ParseAnchors 字符串方式（如需兼容旧字段，可保留 fallback，但新流程优先 YAML positions）

### 3.5 合法性检查（必须实现）

* anchors.positions 必须为 4 个元素（当前系统假设 4 基站；如你希望可扩展，则让数量可变，但要同步更新 UWB 观测处理）
* 每个 position 必须有 3 个数
* `P0_diag.size()==31`
* mode 只能为 fixed/auto
* gating/time_tolerance > 0，sigma>0 等范围检查

---

## 4) 必须交付的内容

1. 修改后的 `config.yaml` 示例（含 common/fusion/generator）
2. 变更清单（逐文件列出新增/修改点）
3. 关键代码片段：

   * Options 结构体定义
   * YAML 解析与覆写逻辑
   * pipeline 中阈值替换示例
   * initialization 中 P0/ba0/bg0 替换示例
4. 回归验证说明：

   * 如何确认融合结果未发生非预期变化（轨迹/RMSE/残差统计对比）
   * 若有差异，解释原因（浮点顺序变化等）

---

## 5) 建议的实施顺序（按最小风险）

1. 扩展 Options + YAML 解析（保持旧逻辑不删，先并行）
2. pipeline/initialization/generator 替换为 options 字段
3. 删除/弃用旧的 anchor_str 与散落魔法数
4. 做一次对比回归（同数据集输出对齐）

请按上述要求直接修改代码并给出完整变更说明。

```
```
