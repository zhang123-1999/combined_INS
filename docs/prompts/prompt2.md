```md
# C++/CMake 项目重构：代码修改提示词（Engine vs SMech 分层）

你是一名经验丰富的 C++ 工程师，请在**不改变算法结果**的前提下，对现有 VSCode + CMake 工程进行结构重构与代码整理。项目包含 UWB 数据生成与 INS+UWB 松组合（ESKF）两部分。重构目标：**将计算函数（smech）与程序调度枢纽（engine）彻底分离**，并把当前 main 中混杂的 IO/配置/初始化/循环/评估拆分为可维护模块。

---

## 0. 总体约束（必须遵守）
- 保持数值结果一致（允许浮点误差微小变化，但 RMSE/轨迹趋势应一致）。
- 编译链：CMake + g++/clang++，VSCode 可直接构建运行。
- 仅重构与整理：禁止引入第三方依赖（Eigen 已在用可继续）。
- 对外接口尽量稳定：优先新增封装而非大规模改函数签名；若必须改动，请同时更新调用处。
- 代码风格：头文件只放声明；实现放 .cpp；减少 inline 巨头文件。

---

## 1. 分层与职责边界（必须落地到目录/文件）
### 1.1 smech（计算层）只做“数学/模型”
**允许放入：**
- INS 机械编排：姿态/速度/位置更新、过程模型 F/G/Q 离散化等
- 量测模型：根据状态计算预测量测、雅可比 H、残差 y 等（UWB/ODO/NHC/ZUPT 后续扩展）
- 通用数学工具：四元数、skew、插值、RMSE、角度归一等

**禁止放入：**
- 文件 IO、路径、配置解析
- 数据集特定初始化策略（例如“取前 1 秒 IMU 平均对准重力/用真值首点赋值”）
- 主循环调度、索引推进、缓存队列、日志写文件

### 1.2 engine（调度层）只做“滤波器生命周期 + 缓存 + 一步预测/更新”
- 管理 IMU 缓存（prev/curr），计算 dt，执行 Predict
- 暴露通用 Correct(y,H,R) 接口，不直接依赖 UWB 特定数据结构
- 提供必要的状态读写接口：GetState/SetState/GetCov/SetCov

### 1.3 app/pipeline（应用层）只做“读配置/读数据/初始化策略/时间同步/调用 engine/输出评估”
- main 必须变薄：只负责组装 options、调用 runner、输出结果

---

## 2. 目标目录结构（按此迁移/新增文件）
请将工程调整为以下结构（可保持现有文件名，但建议迁移到对应目录）：

```

include/
core/
eskf.h
eskf_engine.h
ins_mech.h
measurement_models.h
anchors.h
io/
data_io.h
utils/
math_utils.h
app/
config.h
initialization.h
pipeline_fusion.h
evaluation.h

src/
core/
eskf_engine.cpp
ins_mech.cpp
measurement_models_uwb.cpp
anchors.cpp
io/
data_io.cpp
utils/
math_utils.cpp
app/
config.cpp
initialization.cpp
pipeline_fusion.cpp
evaluation.cpp

apps/
eskf_fusion_main.cpp
uwb_generator_main.cpp

```

---

## 3. 必做拆分：把当前 main 拆成 5 个模块
请从现有 `eskf_fusion.cpp`（或同类 main）中拆分并迁移逻辑：

1) `app/config.*`
- 定义 `struct FusionOptions { ... }`
- `FusionOptions LoadConfig(const std::string& path);`
- 将原先散落的参数读取、默认值、合法性检查集中在这里

2) `app/initialization.*`
- `bool InitializeState(const FusionOptions&, const std::vector<ImuData>&, const TruthData&, State& x0, Eigen::MatrixXd& P0);`
- 这里允许使用“前若干秒 IMU 平均对准重力”“用真值首点初始化位置”等策略
- smech/engine 内不允许出现这些策略判断

3) `app/pipeline_fusion.*`
- 封装主循环：时间同步、uwb_idx 推进、何时调用 Predict/Correct
- 入口函数建议：
  - `FusionResult RunFusion(const FusionOptions&, const Dataset&);`
  - 或封装为 `class FusionRunner { public: FusionResult Run(); };`
- `FusionResult` 统一承载轨迹、残差统计、运行时长、日志信息

4) `app/evaluation.*`
- 插值、对齐、RMSE、导出 CSV 等都搬到这里
- main 禁止出现插值细节与 RMSE 公式实现

5) `apps/eskf_fusion_main.cpp`
- 只保留：LoadConfig -> LoadDataset(IO) -> Initialize -> RunFusion -> Evaluate/Save
- main 控制在 30~60 行为宜

---

## 4. measurement_models：从 inline 头文件迁到 .cpp
请将现有 `measurement_models.h` 中的 inline 实现迁移为：
- `include/core/measurement_models.h`：只保留声明与数据结构
- `src/core/measurement_models_uwb.cpp`：实现 UWB 模型计算
要求：
- 对外提供统一接口，例如：
  - `bool ComputeUwbModel(const State& x, const Anchor& a, double range_meas, Eigen::VectorXd& y, Eigen::MatrixXd& H, Eigen::MatrixXd& R);`
- 保持计算等价（雅可比与残差符号一致）

---

## 5. engine 内部建议（不改变算法，仅提升可维护性）
在 `EskfEngine` 中完成以下整理（可选但强烈建议）：
- 将 `Correct` 拆为三个私有函数：
  - `ComputeKalmanGain(P,H,R) -> K`
  - `InjectErrorState(dx)`：专门负责 dp/dv/dtheta/dba/dbg 注入与姿态更新
  - `UpdateCovarianceJoseph(P,K,H,R)`：Joseph form
- IMU 缓存状态建议使用 enum 状态机（替代多个 bool），提高可读性与可调试性
- 明确 dt 异常处理策略（dt<=0 或跳变阈值）：返回 false 或记录告警

---

## 6. IO 与工具函数清理
- `io/data_io.*`：只做读写，不做滤波逻辑
- `utils/math_utils.*`：只放数学小工具，不放流程
- 新增（如需要）：
  - `utils/logging.h/.cpp`（仅 std::cout / 文件日志，不引第三方）
  - `core/constants.h`：重力常量、角速度常量、单位说明集中管理

---

## 7. CMake 调整要求
- 将库按层拆分（至少 core/utils/io/app 分离），示例：
  - `uwb_core`（core + utils）
  - `uwb_io`（io）
  - `uwb_app`（app，依赖 core/io）
- `apps/*_main.cpp` 只生成可执行程序并 link `uwb_app`
- 确保 `target_include_directories` 正确指向 `include/`

---

## 8. 交付物要求（你必须输出这些内容）
1) 变更清单（按文件列出新增/删除/移动/修改）
2) 新的目录树（文本形式）
3) 关键接口定义（FusionOptions、FusionResult、RunFusion/Runner 等）
4) 至少一个示例 main（`apps/eskf_fusion_main.cpp`）展示新的调用方式
5) 若有函数签名变化：给出迁移说明（旧调用点如何改）

---

## 9. 回归验证（必须说明你如何验证“结果不变”）
- 输出相同格式的轨迹文件/残差文件
- 对比 RMSE（位置/速度/姿态）与关键统计量
- 若存在微小差异，说明原因（例如浮点计算顺序变化），并证明误差在可接受范围

---

## 10. 实施顺序（建议按最小风险执行）
1) 抽离 config
2) 抽离 initialization
3) 封装 pipeline_fusion（主循环）
4) 抽离 evaluation
5) measurement_models 迁移到 .cpp
6) engine 内部重构 Correct（可选）

请直接开始重构，按上述要求给出修改后的代码与说明。
```
