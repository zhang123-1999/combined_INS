#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "utils/math_utils.h"

using namespace std;
using namespace Eigen;

//.\build\Release\data_converter.exe
namespace {

/**
 * 原始 IMU 记录。
 * t 为时间戳，g* 为角速度，a* 为线加速度（均为原始输入单位）。
 */
struct RawImu {
  double t;
  double gx;
  double gy;
  double gz;
  double ax;
  double ay;
  double az;
};

/**
 * 原始 POS 记录。
 * t 为时间戳；lat/lon/h 为大地坐标；
 * vn/ve/vd 为 NED 速度；roll/pitch/yaw 为欧拉角（度）。
 */
struct RawPos {
  double t;
  double lat_deg;
  double lon_deg;
  double h;
  double vn, ve, vd;          // NED 速度
  double roll, pitch, yaw;    // 欧拉角 (deg)
};

// WGS84 常量与坐标变换已统一使用 math_utils 模块
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

/**
 * 读取原始 IMU 文本文件。
 * @param path 文件路径
 * @return IMU 记录列表
 */
vector<RawImu> LoadImu(const string &path) {
  ifstream fin(path);
  if (!fin) {
    cerr << "无法打开 IMU 文件: " << path << "\n";
    return {};
  }
  vector<RawImu> data;
  string line;
  while (getline(fin, line)) {
    if (line.empty()) continue;
    stringstream ss(line);
    RawImu row{};
    if (!(ss >> row.t >> row.gx >> row.gy >> row.gz >> row.ax >> row.ay >> row.az)) {
      continue;
    }
    data.push_back(row);
  }
  return data;
}

/**
 * 读取原始 POS 文本文件。
 * @param path 文件路径
 * @return POS 记录列表
 */
vector<RawPos> LoadPos(const string &path) {
  ifstream fin(path);
  if (!fin) {
    cerr << "无法打开 POS 文件: " << path << "\n";
    return {};
  }
  vector<RawPos> data;
  string line;
  while (getline(fin, line)) {
    if (line.empty()) continue;
    stringstream ss(line);
    double week = 0.0;
    RawPos row{};
    // 输入格式: week t lat lon h vn ve vd roll pitch yaw
    // 新数据格式第一列是周数，需要忽略，第二列是周内秒
    if (!(ss >> week >> row.t >> row.lat_deg >> row.lon_deg >> row.h
             >> row.vn >> row.ve >> row.vd
             >> row.roll >> row.pitch >> row.yaw)) {
      continue;
    }
    data.push_back(row);
  }
  return data;
}

/**
 * 保存 IMU 数据到工程格式文本。
 * @param path 输出文件路径
 * @param imu 原始 IMU 记录
 * @param t0 时间基准（占位，兼容调用）
 */
void SaveImu(const string &path, const vector<RawImu> &imu, double t0) {
  (void)t0;  // 保留参数以兼容调用，未使用
  ofstream fout(path);
  if (!fout) {
    cerr << "无法写入 IMU 输出: " << path << "\n";
    return;
  }
  fout << "timestamp dtheta_x dtheta_y dtheta_z dvel_x dvel_y dvel_z\n";
  for (size_t i = 0; i < imu.size(); ++i) {
    double dt = 0.0;
    if (i > 0) {
      dt = imu[i].t - imu[i - 1].t;
      if (dt < 0.0) dt = 0.0;
    }
    double t_abs = imu[i].t;
    // data is already increment
    double dtheta_x = imu[i].gx;
    double dtheta_y = imu[i].gy;
    double dtheta_z = imu[i].gz;
    double dvel_x = imu[i].ax;
    double dvel_y = imu[i].ay;
    double dvel_z = imu[i].az;
    fout << fixed << setprecision(15);
    fout << t_abs << " " << dtheta_x << " " << dtheta_y << " " << dtheta_z << " "
         << dvel_x << " " << dvel_y << " " << dvel_z << "\n";
  }
}

/**
 * 保存 POS 数据到工程格式文本。
 * @param path 输出文件路径
 * @param pos 原始 POS 记录
 * @param t0 时间基准（占位，兼容调用）
 */
void SavePos(const string &path, const vector<RawPos> &pos, double t0) {
  (void)t0;  // 保留参数以兼容调用，未使用
  if (pos.empty()) return;
  ofstream fout(path);
  if (!fout) {
    cerr << "无法写入 POS 输出: " << path << "\n";
    return;
  }
  fout << "timestamp x y z vx vy vz roll pitch yaw\n";
  fout << fixed << setprecision(15);
  for (const auto &p : pos) {
    double lat = p.lat_deg * kDegToRad;
    double lon = p.lon_deg * kDegToRad;
    
    // 1. 位置转换 (Llh -> Ecef)
    Vector3d ecef = LlhToEcef(lat, lon, p.h);
    
    // 2. 速度转换 (NED -> Ecef)
    Matrix3d R_ne = RotNedToEcef(lat, lon);
    Vector3d v_n(p.vn, p.ve, p.vd);
    Vector3d v_e = R_ne * v_n;
    
    // 3. 姿态直接输出欧拉角 (deg)
    double t_abs = p.t;
    fout << t_abs << " " 
         << ecef.x() << " " << ecef.y() << " " << ecef.z() << " "
         << v_e.x() << " " << v_e.y() << " " << v_e.z() << " "
         << p.roll << " " << p.pitch << " " << p.yaw
         << "\n";
  }
}

}  // namespace

/**
 * 数据转换工具入口。
 * 读取原始 IMU/POS 文件并输出工程格式（ECEF 坐标、四元数）。
 */
int main(int argc, char **argv) {
  const string imu_in = (argc > 1) ? argv[1] : "IMU.txt";
  const string pos_in = (argc > 2) ? argv[2] : "REF_NAV.txt";
  const string imu_out = (argc > 3) ? argv[3] : "IMU_converted.txt";
  const string pos_out = (argc > 4) ? argv[4] : "POS_converted.txt";

  // 1) 加载原始 IMU/POS
  vector<RawImu> imu = LoadImu(imu_in);
  vector<RawPos> pos = LoadPos(pos_in);
  if (imu.empty() || pos.empty()) {
    cerr << "输入数据为空，转换中止\n";
    return 1;
  }

  // 2) 输出符合工程格式的 IMU/POS（保留原始绝对时间）
  SaveImu(imu_out, imu, 0.0);
  SavePos(pos_out, pos, 0.0);

  cout << "转换完成:\n  IMU -> " << imu_out << "\n  POS -> " << pos_out << "\n";
  cout << "已保留原始绝对时间戳，无时间平移。\n";
  return 0;
}
