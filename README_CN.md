# INS/GNSS/ODO/NHC 组合导航

[English README](README.md)

本仓库实现了一套 C++17 组合导航程序，主线是 `INS/GNSS/ODO/NHC` 融合，并在
GNSS 周期性失锁和极端初值条件下对比标准 ESKF 与 InEKF 方法。

这个项目适合作为简历展示型工程：同一套程序中包含 IMU 惯导机械编排、GNSS
位置/速度更新、里程计前向速度约束、非完整约束、外参/比例因子在线估计，以及
ESKF/InEKF 的对比实验。

![GNSS outage 下三维位置误差](assets/position_error_outage.png)

## 项目亮点

- **多源组合导航**：支持 IMU 预测传播、GNSS 位置/速度、ODO、NHC、可选 UWB 与
  ZUPT 更新。
- **ESKF / InEKF 对比**：量测管线保持一致，通过配置切换标准 ESKF 与 InEKF
  过程模型和 reset 语义。
- **31 维状态估计**：位置、速度、姿态、IMU 零偏、IMU 比例因子、ODO scale、
  安装角、ODO 杆臂、GNSS 杆臂。
- **GNSS outage 实验**：通过 YAML 配置周期性 GNSS 可用窗口，并按 outage 段评估。
- **鲁棒更新机制**：支持 NIS 门控、鲁棒权重、协方差下界、运行阶段控制和诊断日志。

## 方法流程

```text
IMU 增量
    -> INS 名义状态递推
    -> ESKF / InEKF 误差状态预测
    -> GNSS 位置 / 速度更新
    -> ODO 前向速度更新
    -> NHC 侧向 / 垂向速度约束
    -> 误差注入、reset、诊断与结果输出
```

本次展示实验的核心结论是：在初始位置、速度、姿态和外参都较差的情况下，InEKF
在 GNSS 间歇失锁场景下表现出更强的收敛和稳定能力。

## 展示实验

`data2` 实验设置：

- 融合模式：INS/GNSS/ODO/NHC
- GNSS outage：初始对准/标定后，周期性关闭 GNSS，每段失锁约 200 s
- 极端初值：位置有偏移，速度置零，姿态置零，安装角和杆臂初值置零
- 两个配置：
  - `configs/data2_ins_gnss_odo_nhc_eskf_large_initial_error.yaml`
  - `configs/data2_ins_gnss_odo_nhc_inekf_large_initial_error.yaml`

| 方法 | 全程 3D RMSE (m) | outage 平均 RMSE (m) | 最差 outage RMSE (m) | 终点 3D 误差 (m) |
|---|---:|---:|---:|---:|
| ESKF | 908.583 | 560.468 | 2932.327 | 26.436 |
| InEKF | 10.836 | 9.819 | 23.426 | 3.785 |

![指标汇总](assets/metrics_summary.png)

![分段 outage RMSE](assets/outage_segment_rmse.png)

![局部轨迹对比](assets/trajectory_comparison.png)

![状态收敛对比](assets/state_convergence.png)

## 仓库结构

```text
apps/
  eskf_fusion_main.cpp       # 融合主程序入口
  data_converter_main.cpp    # 原始 IMU / 参考轨迹转换工具
  uwb_generator_main.cpp     # 可选 UWB 测距数据生成器

include/
  app/                       # 配置、数据集、运行时、诊断接口
  core/                      # 兼容包装与 UWB 工具
  navigation/                # 状态定义、过程模型、量测模型
  io/, utils/                # 文本 I/O 与坐标/数学工具

src/
  app/                       # 配置解析、初始化、调度、融合运行时
  navigation/                # ESKF/InEKF 过程模型与量测模型实现
  core/, io/, utils/

configs/                     # 展示实验的 ESKF / InEKF 配置
scripts/                     # 绘图、README 图像生成、数据转换
assets/                      # README 图像和指标摘要
docs/                        # 简明技术说明
```

## 编译

项目使用 CMake、Eigen 3.4 和 yaml-cpp 0.8。依赖通过 CMake `FetchContent`
自动获取。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

运行 InEKF 展示配置：

```bash
./build/eskf_fusion --config configs/data2_ins_gnss_odo_nhc_inekf_large_initial_error.yaml
```

Windows/MSVC 下可执行文件通常位于 `build/Release/`。

## 数据说明

大体积数据集和求解输出不提交到仓库。展示配置默认使用如下本地数据结构：

```text
dataset/
  data2_converted/
    IMU_converted.txt
    ODO_converted.txt
    POS_converted.txt
  data2/
    rtk.txt
```

`assets/` 中的图像来自两组已完成的 ESKF/InEKF 结果，可通过以下脚本重新生成：

```bash
python scripts/analysis/make_readme_assets.py
```

该脚本默认读取仓库上一级目录中的两个结果文件夹，并更新 README 图像和
`assets/metrics_summary.csv`。

## 输出格式

`eskf_fusion` 输出 31 列空格分隔的 `SOL_*.txt`：

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

位置和速度为 ECEF 坐标，姿态为 NED 欧拉角，单位为度。

## 作者

张畅帆  
GitHub: [zhang123-1999](https://github.com/zhang123-1999)  
Email: <zzchangfan@gmail.com>
