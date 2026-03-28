# UWB / INS / GNSS 融合工程（31 维 ESKF）

本项目实现了基于误差状态卡尔曼滤波（ESKF）的组合导航系统，支持以下量测与约束：

- IMU 预测传播（ECEF）
- UWB 距离更新
- GNSS 位置 + 速度更新
- ODO 里程计前向速度约束
- NHC 非完整约束
- ZUPT 零速约束

系统状态维度为 31，含比例因子、安装角、ODO 杆臂、GNSS 杆臂等外参估计项。

## 当前 Canonical Baseline

- 官方 baseline 配置：`config_data2_baseline_eskf.yaml`
- 官方 baseline runner：`python scripts/analysis/run_data2_baseline_current.py`
- 官方 baseline 产物目录：`output/data2_baseline_current/`
- supporting / historical sweep seed：`config_data2_research_seed_eskf.yaml`
- 最小保留证据链：
  - `output/data2_staged_g5_no_imu_scale_r2_20260325/`
  - `output/data2_staged_g5_odo_scale_phase2_seed_sweep_r1_20260326/`
  - `output/data2_staged_g5_odo_lever_phase2_seed_sweep_r2_20260326/`
  - `output/data2_staged_g5_odo_lever_process_q_sweep_r1_20260326/`
- `2026-03-26` baseline finalization 后，其余历史 `output/data2_*` 目录统一归档到 `archive/output_legacy/20260326-baseline-finalization/`

## 特性概览

- 31 维状态：`p v att ba bg sg sa odo_scale mounting(roll/pitch/yaw) lever_odo lever_gnss`
- 坐标统一：内部名义状态在 ECEF；误差状态线性化在 NED
- 量测鲁棒性：NIS 门控 + Huber/Cauchy 鲁棒权重
- 稳定性机制：修正步长/边界保护、协方差下界（covariance floor）
- 可观性增强：支持 FEJ-ESKF（两层开关）
- 策略控制：GNSS 时间调度、UWB 基站分段调度、状态消融/后 GNSS 消融
- 诊断能力：重力对准、漂移/发散窗口日志、一致性统计、`DIAG.txt`

## 工程结构

```text
apps/
  eskf_fusion_main.cpp         # 融合主程序（支持 --config）
  uwb_generator_main.cpp       # UWB 距离模拟生成
  data_converter_main.cpp      # 原始 IMU/POS 转换

include/
  app/fusion.h                 # 配置/数据结构/流程接口
  app/diagnostics.h            # 诊断引擎接口
  core/eskf.h                  # ESKF 核心状态、噪声、引擎接口
  core/uwb.h                   # UWB/GNSS/NHC/ODO/ZUPT 量测模型接口
  io/data_io.h                 # 文本矩阵读写
  utils/math_utils.h           # 坐标与数学工具

src/
  app/config.cpp               # YAML 配置解析与校验
  app/initialization.cpp       # 初始状态/P0 构建
  app/pipeline_fusion.cpp      # 数据加载 + 主融合循环
  app/evaluation.cpp           # 评估与结果保存（31 列）
  app/diagnostics.cpp          # 诊断实现与 DIAG 输出
  core/ins_mech.cpp            # 预测传播与过程模型
  core/eskf_engine.cpp         # 预测/更新/误差注入
  core/measurement_models_uwb.cpp
  core/anchors.cpp
  io/data_io.cpp
  utils/math_utils.cpp

config.yaml                    # 默认配置
config_data2_*.yaml            # data2 实验配置样例
plot_navresult.py              # 结果绘图脚本
```

## 编译

```bash
cmake -S . -B build
cmake --build build --config Release
```

说明：

- 工程使用 `FetchContent` 获取 `Eigen 3.4.0` 与 `yaml-cpp 0.8.0`。
- Windows 下可执行文件通常位于 `build/Release/`。

## 运行方式

### 1) 融合主程序

```bash
./build/Release/eskf_fusion.exe
./build/Release/eskf_fusion.exe --config config_data2_gnss.yaml
```

- 默认读取 `config.yaml`
- 支持 `--config <path>` 指定其他 YAML

### 2) UWB 数据生成

```bash
./build/Release/uwb_generator.exe
```

- 读取 `config.yaml` 中 `generator` 与 `common`（可被 `generator.anchors/gating` 覆盖）配置
- 输出 `timestamp + dist_1...dist_N`

### 3) 原始数据转换

```bash
./build/Release/data_converter.exe IMU.txt REF_NAV.txt IMU_converted.txt POS_converted.txt
```

其中：

- `IMU.txt`：`t gx gy gz ax ay az`（增量形式）
- `REF_NAV.txt`：`week t lat lon h vn ve vd roll pitch yaw`
- 输出 `POS_converted.txt` 为 `ECEF位置 + ECEF速度 + NED欧拉角`

### 4) 结果绘图

```bash
python plot_navresult.py output/data2_baseline_current/SOL_data2_baseline_current.txt
```

- 支持读取 28/31 列新格式 SOL 文件
- 图像输出到 `output/result_*`

### 5) 官方 baseline 一键重跑

```bash
python scripts/analysis/run_data2_baseline_current.py
```

- 读取 `config_data2_baseline_eskf.yaml`
- 输出到 `output/data2_baseline_current/`
- 生成 `summary.md / manifest.json / case_metrics.csv / phase_metrics.csv / plots/`

## 输入数据格式

### IMU (`imu_path`)

```text
timestamp dtheta_x dtheta_y dtheta_z dvel_x dvel_y dvel_z
```

### 真值/参考轨迹 (`pos_path`, 10 列)

```text
timestamp p1 p2 p3 v1 v2 v3 roll pitch yaw
```

- 若 `p1,p2` 绝对值小于 200，按 `LLA(deg)` 自动识别并转 ECEF
- 否则按 `ECEF(m)` 读取
- `roll/pitch/yaw` 按 NED 欧拉角（deg）解释

### GNSS (`gnss_path`, 可选)

- 7 列：`timestamp pos(3) std(3)`
- 13 列：`timestamp pos(3) std(3) vel(3) vel_std(3)`
- 位置支持自动识别 `LLA/ECEF`；速度输入为 NED，内部转 ECEF 使用

### ODO (`odo_path`, 可选)

```text
timestamp v_forward
```

### UWB (`uwb_path`, 可选)

```text
timestamp dist_1 dist_2 ... dist_N
```

## 输出格式（`output_path`）

`EvaluateFusion` 当前输出 **31 列**：

```text
timestamp
fused_x fused_y fused_z
fused_vx fused_vy fused_vz
fused_roll fused_pitch fused_yaw
mounting_pitch mounting_yaw odo_scale
sg_x sg_y sg_z
sa_x sa_y sa_z
ba_x ba_y ba_z
bg_x bg_y bg_z
lever_x lever_y lever_z
gnss_lever_x gnss_lever_y gnss_lever_z
```

说明：

- 位置/速度：ECEF
- 姿态角：NED 欧拉角（deg）
- `sg/sa`：以 ppm 输出（内部为无量纲）

## 关键配置项（`fusion`）

- `imu_path / pos_path / gnss_path / odo_path / uwb_path / output_path`
- `starttime / finaltime`：时间裁剪窗口
- `noise.*`：过程噪声与量测噪声参数（含 `sigma_mounting_*`, `sigma_gnss_lever_arm`）
- `constraints.*`：
  - 开关：`enable_nhc / enable_odo / enable_zupt`
  - 鲁棒与一致性：`enable_nis_gating`, `robust_kernel`, `robust_tuning`
  - 保护项：`enforce_extrinsic_bounds`, `enable_covariance_floor`
  - 诊断项：`enable_diagnostics`, `enable_consistency_log`
- `init.*`：
  - `use_truth_pva`（真值初始化 or 手动初始化）
  - 外参与初始方差（`mounting_*0`, `lever_arm0`, `gnss_lever_arm0`, `std_*`）
  - 冲突策略（`lever_arm_source`, `strict_extrinsic_conflict`）
- `fej.*`：FEJ 开关与阈值
- `gnss_schedule.*`：GNSS 前段启用比例
- `uwb_anchor_schedule.*`：UWB 基站头尾分段调度
- `ablation.*` 与 `post_gnss_ablation.*`：状态消融实验

## 诊断输出

当 `constraints.enable_diagnostics: true` 时，程序会输出：

- `DIAG.txt`：每秒记录一次状态标准差、关键相关系数、运动指标
- 控制台诊断：重力对准、漂移窗口、首次发散窗口

当 `constraints.enable_consistency_log: true` 时，会输出 NHC/ODO 一致性统计（accept ratio、NIS 均值等）。

## 备注

- 本仓库默认 `main` 分支可直接运行配置样例（需数据文件路径存在）。
- 若只做 INS+GNSS 或 INS-only，可将对应量测路径置空并关闭相应约束开关。
