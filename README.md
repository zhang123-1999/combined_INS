# UWB-IMU ESKF 融合工程

本项目实现了基于 31 维 ESKF 的多源融合定位系统，支持 IMU + UWB / GNSS / ODO / NHC / ZUPT，状态在 **ECEF** 坐标系下传播与更新。

---

## 坐标系与数据格式规范

### 内部统一坐标系

| 物理量 | 内部坐标系 | 说明 |
|--------|-----------|------|
| 位置 `p` | **ECEF** (m) | 地心地固坐标 |
| 速度 `v` | **ECEF** (m/s) | 地心地固速度 |
| 姿态 `q` | **ECEF 四元数** (wxyz) | Body→ECEF 旋转 $C_b^e$ |
| IMU 量测 | **Body (FRD)** | 前-右-下体坐标系增量 |
| 零偏/比例因子 | **Body (FRD)** | 加速度计/陀螺仪 |
| 杆臂 | **Body (FRD)** (m) | ODO/GNSS 杆臂 |
| 安装角 | **Body→Vehicle** (rad) | IMU 到车体旋转角 |

**原则**：所有外部输入数据在 `LoadDataset` 阶段统一转换为 ECEF 坐标，后续融合主循环中 **不再** 出现 LLA/NED 等坐标系的即席转换。输出评估阶段将 ECEF 结果转回 NED 欧拉角供人工阅读。

### 输入数据格式

#### IMU 数据 (`IMU_converted.txt`)

| 列 | 含义 | 单位 | 坐标系 |
|----|------|------|--------|
| 1 | 时间戳 (GPS 周内秒) | s | — |
| 2-4 | 角增量 $d\theta_x, d\theta_y, d\theta_z$ | rad | Body (FRD) |
| 5-7 | 速度增量 $dv_x, dv_y, dv_z$ | m/s | Body (FRD) |

```
timestamp  dtheta_x  dtheta_y  dtheta_z  dvel_x  dvel_y  dvel_z
```

#### 真值/参考轨迹 (`POS_converted.txt`)

| 列 | 含义 | 单位 | 坐标系 |
|----|------|------|--------|
| 1 | 时间戳 | s | — |
| 2-4 | 位置 $x, y, z$ | m | ECEF |
| 5-7 | 速度 $v_x, v_y, v_z$ | m/s | ECEF |
| 8-10 | 姿态欧拉角 Roll, Pitch, Yaw | deg | NED (Body→NED) |

```
timestamp  x  y  z  vx  vy  vz  roll  pitch  yaw
```

> **注意**：位置/速度为 ECEF，姿态欧拉角为 NED 系下的 Roll/Pitch/Yaw（度）。`LoadDataset` 会自动检测输入是 LLA 还是 ECEF 格式（通过判断前两列绝对值是否小于 200），若为 LLA 则自动转换。

#### GNSS 观测 (`GNSS_converted.txt`)

| 列 | 含义 | 单位 | 坐标系 |
|----|------|------|--------|
| 1 | 时间戳 | s | — |
| 2-4 | 位置 lat, lon, h 或 x, y, z | deg / m | LLA 或 ECEF（自动检测） |
| 5-7 | 位置精度 $\sigma_N, \sigma_E, \sigma_D$ | m | NED |
| 8-10 | 速度 $v_N, v_E, v_D$ | m/s | NED |
| 11-13 | 速度精度 $\sigma_{vN}, \sigma_{vE}, \sigma_{vD}$ | m/s | NED |

```
timestamp  lat/x  lon/y  h/z  sigma_n  sigma_e  sigma_d  vn  ve  vu  sigma_vn  sigma_ve  sigma_vu
```

> `LoadDataset` 自动检测坐标格式并转换为 ECEF。速度从NED坐标系转换到ECEF坐标系进行量测更新。

#### UWB 距离观测 (`UWB_simulated.txt`)

| 列 | 含义 | 单位 |
|----|------|------|
| 1 | 时间戳 | s |
| 2~N+1 | 到各基站的距离 | m |

```
timestamp  dist_1  dist_2  ...  dist_N
```

基站坐标在配置文件中给出，单位为 **ECEF (m)**。

#### 里程计 (`iODO.txt`)

| 列 | 含义 | 单位 |
|----|------|------|
| 1 | 时间戳 | s |
| 2 | 前向速度 | m/s |

```
timestamp  v_forward
```

### 输出数据格式

#### 融合结果 (`SOL.txt`) — 28 列

| 列 | 含义 | 单位 | 坐标系 |
|----|------|------|--------|
| 1 | 时间戳 | s | — |
| 2-4 | 位置 | m | ECEF |
| 5-7 | 速度 | m/s | ECEF |
| 8-10 | 姿态 Roll, Pitch, Yaw | deg | NED |
| 11-12 | 安装角 Pitch, Yaw | deg | Body→Vehicle |
| 13 | ODO 比例因子 | — | — |
| 14-16 | 陀螺比例因子 | ppm | Body |
| 17-19 | 加速度计比例因子 | ppm | Body |
| 20-22 | 加速度计零偏 | m/s² | Body |
| 23-25 | 陀螺零偏 | rad/s | Body |
| 26-28 | 杆臂 | m | Body |

### 坐标转换数据流

```
原始数据 (LLA/NED)
    │
    ▼  data_converter.exe
    │  位置: LLA(deg) → ECEF(m)
    │  速度: NED(m/s) → ECEF(m/s)
    │  姿态: 欧拉角(deg)原样保留
    │  IMU:  增量数据直接输出(Body FRD)
    │
    ▼  转换后数据 (ECEF + NED 欧拉角)
    │
    ▼  LoadDataset()
    │  自动检测 LLA/ECEF → 统一为 ECEF
    │  NED 欧拉角 → R_ne · R_nb → ECEF 四元数
    │  GNSS: 自动检测 LLA/ECEF → 统一为 ECEF
    │
    ▼  融合主循环 (全部 ECEF)
    │  惯导传播 / UWB / GNSS / NHC / ODO / ZUPT
    │
    ▼  EvaluateFusion()
       ECEF 四元数 → R_ne^T · R_eb → NED 欧拉角(deg)
       ECEF 位置/速度直接输出
```

---

## 代码结构

```
apps/
  eskf_fusion_main.cpp        # 融合主程序入口
  uwb_generator_main.cpp      # UWB 量测生成器入口
  data_converter_main.cpp     # 原始 IMU/POS 转换工具入口
include/
  app/fusion.h                # 配置/初始化/管线/评估接口
  app/diagnostics.h           # 诊断引擎接口
  core/eskf.h                 # ESKF 状态(31维)、噪声与引擎接口
  core/uwb.h                  # 基站与量测模型接口 (UWB/NHC/ZUPT/ODO/GNSS)
  io/data_io.h                # 文本矩阵读写接口
  utils/math_utils.h          # 数学与坐标工具统一接口
src/
  app/config.cpp              # YAML 配置解析
  app/initialization.cpp      # 状态初始化 (真值/手动)
  app/pipeline_fusion.cpp     # 数据加载与融合主循环
  app/evaluation.cpp          # 结果评估与保存
  app/diagnostics.cpp         # 诊断引擎实现
  core/ins_mech.cpp           # ECEF 惯导传播 (双子样+圆锥/划桨补偿)
  core/eskf_engine.cpp        # ESKF 引擎 (预测/更新/误差注入)
  core/measurement_models_uwb.cpp  # 量测模型实现
  core/anchors.cpp            # 基站解析与自动布站
  io/data_io.cpp              # 文本矩阵读写实现
  utils/math_utils.cpp        # 坐标变换与数学工具统一实现
config.yaml                   # 默认配置文件
```

### 坐标变换封装原则

所有坐标变换函数统一在 `utils/math_utils.h` / `math_utils.cpp` 中实现，其他模块 **禁止** 自行实现坐标转换逻辑：

| 函数 | 变换方向 |
|------|----------|
| `EcefToLlh(Vector3d)` → `Llh` | ECEF → LLH (rad) |
| `EcefToLatLon(Vector3d, &lat, &lon)` | ECEF → lat/lon (rad)，便利接口 |
| `LlhToEcef(Llh)` | LLH (rad) → ECEF |
| `LlhToEcef(lat, lon, h)` | LLH (rad) → ECEF，便利接口 |
| `RotNedToEcef(Llh)` | NED→ECEF 旋转矩阵 $R_n^e$ |
| `RotNedToEcef(lat, lon)` | 同上，便利接口 |
| `RpyToQuat(Vector3d)` | RPY (rad) → 四元数 (wxyz) |
| `EulerToRotation(r, p, y)` | RPY (rad) → 旋转矩阵 (ZYX) |
| `RotToEuler(Matrix3d)` | 旋转矩阵 → RPY (rad) |
| `QuatToRot(Vector4d)` | 四元数 → 旋转矩阵 |
| `GravityEcef(Vector3d)` | ECEF 位置 → 重力向量 (Somigliana) |

## 功能概览

- **IMU 惯导传播**：双子样圆锥/划桨补偿，ECEF 中引入重力与地球自转
- **ESKF 更新**：通用量测更新接口 (Joseph 协方差更新)
- **量测源**：UWB 距离、GNSS 位置、NHC 侧/垂向速度约束、ODO 里程计、ZUPT 零速约束
- **基站布设**：固定基站或真值轨迹自动布设
- **数据管线**：自动格式检测 + 时间裁剪 + 坐标统一转换
- **诊断引擎**：重力对准校验、漂移/发散检测
- **数据工具**：原始 IMU/POS 格式转换、UWB 距离模拟生成

## 执行流程

1. 读取配置 (`LoadFusionOptions`)
2. 加载数据集 (`LoadDataset`)：IMU/UWB/真值/GNSS/ODO + 坐标统一转换 + 时间裁剪
3. 初始化状态 (`InitializeState`)：真值模式或手动 LLA/NED/RPY 初始化
4. 融合主循环 (`RunFusion`)：
   - IMU 传播 → ZUPT → NHC → ODO → UWB → GNSS → 记录
5. 评估与保存 (`EvaluateFusion` + `SaveFusionResult`)

## 运行步骤

1. 编译：
   ```sh
   cmake -S . -B build
   cmake --build build --config Release
   ```
2. （可选）原始数据转换：
   ```sh
   .\build\Release\data_converter.exe IMU.txt REF_NAV.txt IMU_converted.txt POS_converted.txt
   ```
3. （可选）生成 UWB 距离数据：
   ```sh
   .\build\Release\uwb_generator.exe
   ```
4. 执行融合：
   ```sh
   .\build\Release\eskf_fusion.exe
   .\build\Release\eskf_fusion.exe --config config_data2_gnss.yaml
   ```
5. 绘图评估：
   ```sh
   python scripts/visualization/plot_navresult.py
   ```

## 配置要点

| 配置项 | 说明 |
|--------|------|
| `common.anchors` | 基站模式（`fixed`/`auto`）与 ECEF 坐标 |
| `common.gating` | UWB 残差门控阈值与时间容差 |
| `fusion.imu_path` 等 | 各输入文件路径 |
| `fusion.noise.*` | 各传感器噪声参数 |
| `fusion.constraints.*` | NHC/ODO/ZUPT 开关与参数 |
| `fusion.init.*` | 初始化参数：位置(LLA deg)、速度(NED m/s)、姿态(RPY deg) |
| `fusion.uwb_anchor_schedule` | UWB 基站时间分段调度 |
| `generator.*` | UWB 生成器参数 |
