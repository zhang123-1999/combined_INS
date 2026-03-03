#include "io/data_io.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace io {

using namespace std;
using namespace Eigen;

/**
 * 从文本文件读取指定列数的矩阵。
 * 自动支持空格/逗号分隔，并跳过非数值行。
 */
MatrixXd LoadMatrix(const string &path, int cols) {
  ifstream fin(path);
  if (!fin) {
    cout << "error: 无法打开文件 " << path << "\n";
    return MatrixXd();
  }
  // 按行读取，支持逗号或空格分隔，自动跳过非数值行（表头）
  vector<double> values;
  string line;
  int rows = 0;
  while (getline(fin, line)) {
    if (line.empty()) continue;
    replace(line.begin(), line.end(), ',', ' ');
    stringstream ss(line);
    vector<double> row_vals;
    double v = 0.0;
    while (ss >> v) {
      row_vals.push_back(v);
    }
    if (row_vals.empty()) continue;  // 可能是表头
    if (static_cast<int>(row_vals.size()) < cols) {
      cout << "error: 文件列数不足 " << path << "\n";
      return MatrixXd();
    }
    // 仅保留前 cols 列，忽略额外字段
    for (int c = 0; c < cols; ++c) {
      values.push_back(row_vals[c]);
    }
    ++rows;
  }
  MatrixXd mat(rows, cols);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      mat(r, c) = values[r * cols + c];
    }
  }
  return mat;
}

/**
 * 保存矩阵到文本文件。
 * @param path 输出文件路径
 * @param mat 矩阵数据
 * @param header 可选表头
 */
void SaveMatrix(const string &path, const MatrixXd &mat,
                const string &header) {
  ofstream fout(path);
  if (!fout) {
    cout << "error: 无法写入文件 " << path << "\n";
    return;
  }
  // 可选表头写入
  if (!header.empty()) {
    fout << header << "\n";
  }
  for (int r = 0; r < mat.rows(); ++r) {
    for (int c = 0; c < mat.cols(); ++c) {
      // 提高输出精度，避免小量级状态（如 ba/bg/scale）在后处理时出现量化台阶。
      fout << fixed << setprecision(9) << mat(r, c);
      if (c + 1 != mat.cols()) fout << " ";
    }
    fout << "\n";
  }
}

}  // namespace io
