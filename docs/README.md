# UWB-IMU ESKF 融合工程

本项目实现了基于 ESKF 的 IMU + UWB 距离融合定位，状态在 ECEF 坐标系下传播与更新；支持 UWB 量测更新、NHC 非完整约束、ZUPT 零速约束，并提供评估与绘图脚本。

## 代码结构

```
apps/
  eskf_fusion_main.cpp        # 融合主程序入口
  uwb_generator_main.cpp      # UWB 量测生成器入口
  data_converter_main.cpp     # 原始 IMU/POS 转换工具入口
include/
  app/fusion.h                # 配置/初始化/管线/评估接口
  core/eskf.h                 # ESKF 状态、噪声与引擎接口
  core/uwb.h                  # 基站与 UWB/NHC/ZUPT 量测模型接口
  io/data_io.h                # 文本矩阵读写接口
  utils/math_utils.h          # 数学与坐标工具接口
src/
  app/                        # 配置解析、初始化、融合管线、评估实现
  core/                       # 惯导传播、ESKF 更新、UWB 量测模型、基站
  io/                         # 文本矩阵读写实现
  utils/                      # 数学与坐标工具实现
plot_navresult.py             # 结果绘图与误差统计脚本
config.yaml                   # 统一配置文件
```

## 功能概览

- IMU 惯导传播：双子样圆锥/划桨补偿，ECEF 中引入重力与地球自转。
- ESKF 更新：通用量测更新接口，支持 UWB 距离、NHC 侧向/垂向速度约束、ZUPT 零速约束。
- 基站布设：固定基站或根据真值轨迹自动布设四角基站。
- 数据管线：读取/裁剪 IMU、UWB、真值；时间对齐并执行融合；输出解算结果与 RMSE。
- 数据生成与转换：原始 IMU/POS 转工程格式；真值轨迹生成 UWB 距离数据。
- 绘图评估：位置误差、轨迹、速度、姿态曲线与 RMSE 统计。

## 执行流程（融合主线）

1. 读取配置 `config.yaml`（`LoadFusionOptions`）。
2. 加载数据集（`LoadDataset`）：IMU/UWB/真值 + 基站构建 + 时间裁剪。
3. 初始化状态（`InitializeState`）：按配置是否使用真值 p/v/q，写入零偏与协方差。
4. 融合主循环（`RunFusion`）：
   - 使用 IMU 进行预测传播（`InsMech::Propagate` + 过程噪声）。
   - 触发 ZUPT/NHC 约束（可选）。
   - 对齐时间并执行 UWB 更新（带残差门控）。
   - 记录融合结果时间轴与状态序列。
5. 评估与保存（`EvaluateFusion` + `SaveFusionResult`）：输出 `SOL.txt` 并打印 RMSE。

## 输入/输出数据格式

- `IMU_converted.txt`：7 列  
  `timestamp dtheta_x dtheta_y dtheta_z dvel_x dvel_y dvel_z`
- `POS_converted.txt`：11 列  
  `timestamp x y z vx vy vz qw qx qy qz`  
  生成 UWB 时仅使用前 4 列 `timestamp x y z`。
- `UWB_simulated.txt`：`timestamp` + N 个基站距离列（N 由基站数量决定）。
- `SOL.txt`：11 列  
  `timestamp fused_x fused_y fused_z fused_vx fused_vy fused_vz fused_qw fused_qx fused_qy fused_qz`

## 运行步骤（典型）

1. 编译：
   ```
   cmake -S . -B build
   cmake --build build --config Release
   ```
2. （可选）原始 IMU/POS 转换：
   ```
   .\build\Release\data_converter.exe IMU.txt POS.txt IMU_converted.txt POS_converted.txt
   ```
3. （可选）生成 UWB 距离数据：
   ```
   .\build\Release\uwb_generator.exe
   ```
4. 执行融合：
   ```
   .\build\Release\eskf_fusion.exe
   ```
5. 绘图评估：
   ```
   python plot_navresult.py
   ```

## 配置要点

- `common.anchors`：基站配置（`fixed` 或 `auto`）。
- `common.gating`：UWB 残差门控与时间容差。
- `fusion.*`：融合输入/输出路径、噪声参数、约束开关、初始化参数、时间裁剪窗口。
- `fusion.uwb_anchor_schedule`：按时间分段选择 UWB 基站（`head_ratio` 划分前段比例，`head_anchors`/`tail_anchors` 为 1-based 索引）。
- `generator.*`：UWB 生成器频率/噪声/种子及输入输出路径。
