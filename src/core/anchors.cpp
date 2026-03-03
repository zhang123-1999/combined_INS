#include "core/uwb.h"

#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace std;
using namespace Eigen;

/**
 * 解析基站字符串并生成 Anchors。
 */
Anchors ParseAnchors(const string &anchor_str) {
  Anchors a;
  stringstream ss(anchor_str);
  string token;
  vector<Vector3d> param_anchors;

  // 解析 "x,y,z;..."
  while (getline(ss, token, ';')) {
    if (token.empty()) continue;
    stringstream ts(token);
    string num;
    int c = 0;
    Vector3d p;
    while (getline(ts, num, ',')) {
      if (c >= 3) break;
      p(c) = stod(num);
      ++c;
    }
    if (c != 3) {
      cout << "error: 每个基站必须包含3个数字\n";
      return Anchors(); // Return empty
    }
    param_anchors.push_back(p);
  }

  if (param_anchors.empty()) {
    cout << "error: 未解析到基站数据\n";
    return Anchors();
  }

  a.positions.resize(param_anchors.size(), 3);
  for(size_t i=0; i<param_anchors.size(); ++i) {
      a.positions.row(i) = param_anchors[i].transpose();
  }
  return a;
}

/**
 * 根据轨迹自动布设四角基站。
 * @param pos_xyz 轨迹位置矩阵（Nx3）
 * @param margin 外扩边距
 * @return 自动布设的基站坐标
 */
Anchors AutoPlaceAnchors(const MatrixXd &pos_xyz, double margin) {
  Anchors a;
  a.positions.resize(4, 3);
  // 以真值轨迹外包框四角布设，Z 取平均高度
  Vector3d mins = pos_xyz.colwise().minCoeff();
  Vector3d maxs = pos_xyz.colwise().maxCoeff();
  double z_ref = pos_xyz.col(2).mean();
  a.positions << mins.x() - margin, mins.y() - margin, z_ref,  
      maxs.x() + margin, mins.y() - margin, z_ref,             
      mins.x() - margin, maxs.y() + margin, z_ref,             
      maxs.x() + margin, maxs.y() + margin, z_ref;
  return a;
}
