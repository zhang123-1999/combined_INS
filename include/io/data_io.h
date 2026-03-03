// IO 工具：读取/保存文本矩阵（空格或逗号分隔）
#pragma once

#include <Eigen/Dense>
#include <string>

namespace io {

using namespace std;
using namespace Eigen;

/**
 * 从文本读取指定列数的矩阵。
 * 支持空格或逗号分隔，自动跳过表头或非数值行。
 */
MatrixXd LoadMatrix(const string &path, int cols);

/**
 * 保存矩阵到文本文件，可附带表头。
 * @param path 输出文件路径
 * @param mat 矩阵数据
 * @param header 可选表头字符串
 */
void SaveMatrix(const string &path, const MatrixXd &mat,
                const string &header);

}  // namespace io
