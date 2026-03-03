#include <Eigen/Dense>
#include <iostream>

#include "app/fusion.h"
#include "core/eskf.h"

using namespace std;
using namespace Eigen;
//.\build\Release\eskf_fusion.exe
//.\build\Release\eskf_fusion.exe --config config_data2_gnss.yaml 2>&1
/**
 * 融合主程序入口。
 * 读取配置与数据集，初始化状态并运行融合流程，最后输出评估结果。
 */
int main(int argc, char **argv) {
  try {
    // 1) 读取配置（支持 --config <path> 命令行参数）
    string config_path = "config.yaml";
    for (int i = 1; i < argc - 1; ++i) {
      if (string(argv[i]) == "--config") {
        config_path = argv[i + 1];
        break;
      }
    }
    FusionOptions options = LoadFusionOptions(config_path);

    // 2) 加载数据集（IMU/UWB/真值/基站）
    Dataset dataset = LoadDataset(options);
    if (dataset.imu.size() < 2) {
      cout << "error: IMU 数据不足\n";
      return 1;
    }

    // 3) 初始化状态与协方差
    State x0;
    Matrix<double, kStateDim, kStateDim> P0;
    if (!InitializeState(options, dataset.imu, dataset.truth, x0, P0)) {
      return 1;
    }

    // 4) 主融合流程 + 评估
    FusionResult result = RunFusion(options, dataset, x0, P0);
    EvaluationSummary summary = EvaluateFusion(result, dataset);
    SaveFusionResult(options.output_path, summary);

    cout << "RMSE (融合) [m]: " << summary.rmse_fused.transpose() << "\n";
    cout << "结果已保存到: " << options.output_path << "\n";
  } catch (const exception &e) {
    cerr << "运行失败: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
