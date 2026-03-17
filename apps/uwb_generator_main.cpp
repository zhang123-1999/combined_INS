#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>

#include "app/fusion.h"
#include "io/data_io.h"

using namespace std;
using namespace Eigen;
//.\build\Release\uwb_generator.exe

/**
 * UWB 数据生成程序入口。
 * 读取配置与真值轨迹，生成带噪声的四基站距离并保存到文件。
 */
int main(int argc, char **argv) {
  try {
    // 1) 读取配置
    string config_path = "config.yaml";
    for (int i = 1; i < argc - 1; ++i) {
      if (string(argv[i]) == "--config") {
        config_path = argv[i + 1];
        break;
      }
    }
    GeneratorOptions opt = LoadGeneratorOptions(config_path);

    // 2) 读取真值轨迹 (t, x, y, z)
    MatrixXd pos = io::LoadMatrix(opt.pos_path, 4);
    VectorXd timestamps = pos.col(0);
    MatrixXd xyz = pos.block(0, 1, pos.rows(), 3);

    // 3) 基站：指定则解析，否则自动布设
    Anchors anchors = BuildAnchors(opt.anchors, xyz);
    int num_anchors = anchors.positions.rows();

    // 4) 随机噪声发生器与采样步长
    double dt = 1.0 / opt.uwb_hz;
    double next_t = timestamps[0];

    mt19937 gen(opt.seed);
    normal_distribution<double> noise(0.0, opt.sigma);

    vector<double> out_time;
    vector<VectorXd> out_ranges;

    // 5) 按频率采样真值并生成含噪声的多基站距离
    for (int i = 0; i < pos.rows(); ++i) {
      double t = timestamps[i];
      if (t + opt.gating.time_tolerance < next_t) continue;
      next_t += dt;

      Vector3d p = xyz.row(i).transpose();
      VectorXd range(num_anchors);
      for (int k = 0; k < num_anchors; ++k) {
        Vector3d diff = p - anchors.positions.row(k).transpose();
        double dist = diff.norm();
        range[k] = dist + noise(gen);
      }
      out_time.push_back(t);
      out_ranges.push_back(range);
    }

    // 6) 组装并保存 UWB 量测文件
    int n = static_cast<int>(out_time.size());
    // 第一列是时间，后续是基站距离
    MatrixXd out(n, 1 + num_anchors);
    for (int i = 0; i < n; ++i) {
      out(i, 0) = out_time[i];
      if (num_anchors > 0) {
        out.block(i, 1, 1, num_anchors) = out_ranges[i].transpose();
      }
    }
    
    // 构造表头
    string header = "timestamp";
    for(int i=0; i<num_anchors; ++i) header += " dist_" + to_string(i+1);
    
    io::SaveMatrix(opt.output_path, out, header);

    cout << "生成 UWB 数据条数: " << n << "\n";
    for (int i = 0; i < num_anchors; ++i) {
      cout << "Anchor " << i + 1 << ": " << anchors.positions.row(i) << "\n";
    }
  } catch (const exception &e) {
    cerr << "运行失败: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
