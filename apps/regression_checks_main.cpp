#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "app/diagnostics.h"
#include "app/fusion.h"

using namespace std;
using namespace Eigen;
namespace fs = std::filesystem;

namespace {

void Expect(bool cond, const string &message) {
  if (!cond) {
    throw runtime_error(message);
  }
}

bool NearlyEqual(double a, double b, double tol = 1e-12) {
  return std::abs(a - b) <= tol;
}

void WriteTextFile(const string &path, const string &text) {
  ofstream fout(path, ios::out | ios::trunc);
  Expect(static_cast<bool>(fout), "failed to create temp file: " + path);
  fout << text;
}

void TestLoadOptionsFailFast() {
  bool fusion_threw = false;
  try {
    (void)LoadFusionOptions("__missing_config__.yaml");
  } catch (const exception &) {
    fusion_threw = true;
  }
  Expect(fusion_threw, "LoadFusionOptions should fail fast on missing YAML");

  bool generator_threw = false;
  try {
    (void)LoadGeneratorOptions("__missing_generator_config__.yaml");
  } catch (const exception &) {
    generator_threw = true;
  }
  Expect(generator_threw, "LoadGeneratorOptions should fail fast on missing YAML");
}

void TestCustomP0DiagTakesPrecedence() {
  FusionOptions options;
  options.init.use_truth_pva = false;
  options.init.has_custom_P0_diag = true;
  options.init.init_pos_lla = Vector3d(30.0, 114.0, 10.0);
  options.init.init_vel_ned = Vector3d(1.0, 2.0, -0.5);
  options.init.init_att_rpy = Vector3d(0.1, -0.2, 3.0);
  options.init.std_pos = Vector3d(10.0, 10.0, 10.0);
  options.init.std_vel = Vector3d(10.0, 10.0, 10.0);
  options.init.std_att = Vector3d(30.0, 30.0, 30.0);
  for (int i = 0; i < kStateDim; ++i) {
    options.init.P0_diag(i) = 0.001 * static_cast<double>(i + 1);
  }

  vector<ImuData> imu(1);
  imu[0].t = 0.0;
  imu[0].dt = 0.0;

  TruthData truth;
  State x0;
  Matrix<double, kStateDim, kStateDim> P0;
  bool ok = InitializeState(options, imu, truth, x0, P0);
  Expect(ok, "InitializeState should succeed for custom P0_diag regression check");
  for (int i = 0; i < kStateDim; ++i) {
    Expect(NearlyEqual(P0(i, i), options.init.P0_diag(i)),
           "InitializeState did not honor explicit P0_diag at index " + to_string(i));
  }
}

void TestInitializeStateInterpolatesTruthToFirstImuTime() {
  FusionOptions options;
  options.init.use_truth_pva = true;

  vector<ImuData> imu(1);
  imu[0].t = 10.25;
  imu[0].dt = 0.0;

  TruthData truth;
  truth.timestamps = VectorXd(2);
  truth.timestamps << 10.0, 10.5;
  truth.positions = MatrixXd(2, 3);
  truth.positions << 1.0, 2.0, 3.0,
                     5.0, 6.0, 7.0;
  truth.velocities = MatrixXd(2, 3);
  truth.velocities << 0.5, -1.0, 2.0,
                      4.5, 3.0, -2.0;
  truth.quaternions = MatrixXd(2, 4);
  truth.quaternions << 1.0, 0.0, 0.0, 0.0,
                       std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5);

  State x0;
  Matrix<double, kStateDim, kStateDim> P0;
  bool ok = InitializeState(options, imu, truth, x0, P0);
  Expect(ok, "InitializeState should succeed for truth interpolation regression check");

  const Vector3d expected_p(3.0, 4.0, 5.0);
  const Vector3d expected_v(2.5, 1.0, 0.0);
  const Vector4d expected_q =
      (0.5 * truth.quaternions.row(0).transpose() +
       0.5 * truth.quaternions.row(1).transpose()).normalized();

  Expect((x0.p - expected_p).norm() <= 1e-12,
         "InitializeState should interpolate truth position to imu.front().t");
  Expect((x0.v - expected_v).norm() <= 1e-12,
         "InitializeState should interpolate truth velocity to imu.front().t");
  Expect(std::abs(std::abs(x0.q.dot(expected_q)) - 1.0) <= 1e-12,
         "InitializeState should interpolate truth attitude to imu.front().t");
}

void TestDiagnosticsCorrectPropagatesFailure() {
  NoiseParams noise;
  noise.sigma_acc = 0.05;
  noise.sigma_gyro = 0.005;
  noise.sigma_ba = 0.0005;
  noise.sigma_bg = 0.0005;
  noise.sigma_uwb = 0.1;
  noise.sigma_odo_scale = 1.0e-5;

  EskfEngine engine(noise);
  State state;
  Matrix<double, kStateDim, kStateDim> P0 =
      Matrix<double, kStateDim, kStateDim>::Identity();
  engine.Initialize(state, P0);

  ConstraintConfig cfg;
  DiagnosticsEngine diag(cfg, true);

  VectorXd y(3);
  y << 2.0e6, 0.0, 0.0;
  MatrixXd H = MatrixXd::Zero(3, kStateDim);
  H.block<3, 3>(0, StateIdx::kPos) = Matrix3d::Identity();
  MatrixXd R = Matrix3d::Identity() * 1.0e-9;

  bool updated = diag.Correct(engine, "TEST_POS", 0.0, y, H, R, nullptr);
  Expect(!updated, "DiagnosticsEngine::Correct should propagate engine.Correct failure");
  Expect(engine.state().p.norm() < 1.0e-12,
         "State should remain unchanged after failed correction");
}

void TestBuildAnchorsContracts() {
  AnchorsConfig empty_fixed;
  empty_fixed.mode = "fixed";
  Anchors empty = BuildAnchors(empty_fixed, MatrixXd::Zero(0, 3));
  Expect(empty.positions.rows() == 0 && empty.positions.cols() == 0,
         "BuildAnchors should allow empty fixed-anchor config for pure INS runs");

  AnchorsConfig fixed;
  fixed.mode = "fixed";
  fixed.positions = {Vector3d(1.0, 2.0, 3.0), Vector3d(-1.0, 0.5, 4.0)};
  Anchors parsed = BuildAnchors(fixed, MatrixXd::Zero(0, 3));
  Expect(parsed.positions.rows() == 2 && parsed.positions.cols() == 3,
         "BuildAnchors should preserve explicit anchor count");
  Expect(NearlyEqual(parsed.positions(0, 0), 1.0) &&
             NearlyEqual(parsed.positions(1, 2), 4.0),
         "BuildAnchors returned unexpected fixed anchor coordinates");

  bool auto_threw = false;
  try {
    AnchorsConfig auto_cfg;
    auto_cfg.mode = "auto";
    (void)BuildAnchors(auto_cfg, MatrixXd::Zero(0, 3));
  } catch (const exception &) {
    auto_threw = true;
  }
  Expect(auto_threw,
         "BuildAnchors should fail fast when auto placement receives empty truth");
}

void TestData2BaselineOfficialConfig() {
  FusionOptions options = LoadFusionOptions("config_data2_baseline_eskf.yaml");
  Expect(options.gnss_path == "dataset/data2/rtk.txt",
         "config_data2_baseline_eskf.yaml should use corrected data2 RTK GNSS path");
  Expect(!options.enable_gnss_velocity,
         "config_data2_baseline_eskf.yaml should disable GNSS velocity updates");
  Expect(!options.ablation.disable_mounting_roll,
         "config_data2_baseline_eskf.yaml should keep mounting_roll estimation enabled");
  Expect(!options.ablation.disable_gnss_lever_z,
         "config_data2_baseline_eskf.yaml should keep gnss_lever_z estimation enabled");
  Expect(!options.ablation.disable_mounting,
         "config_data2_baseline_eskf.yaml should keep mounting pitch/yaw estimable");
  Expect(!options.ablation.disable_gnss_lever_arm,
         "config_data2_baseline_eskf.yaml should keep gnss_lever x/y estimable");
}

void TestGnssVelocityAvailabilityIndependentFromEnableFlag() {
  Dataset dataset;
  dataset.gnss.timestamps = VectorXd::LinSpaced(2, 10.0, 11.0);
  dataset.gnss.velocities = MatrixXd::Zero(2, 3);
  dataset.gnss.vel_std = MatrixXd::Constant(2, 3, 0.2);

  FusionOptions options;
  options.enable_gnss_velocity = false;
  Expect(HasGnssVelocityData(dataset),
         "GNSS velocity availability should depend on dataset columns, not enable_gnss_velocity");
  Expect(!options.enable_gnss_velocity,
         "sanity check: test requires GNSS_VEL update switch to remain disabled");

  dataset.gnss.vel_std = MatrixXd::Constant(1, 3, 0.2);
  Expect(!HasGnssVelocityData(dataset),
         "HasGnssVelocityData should reject missing or misaligned GNSS velocity std rows");
}

void TestTimestampAlignmentUsesConfiguredTolerance() {
  Expect(!IsTimestampAlignedToReference(528077.0, 528076.999368, 1.0e-6),
         "GNSS exact-time alignment should not snap to a previous IMU sample outside time_tolerance");
  Expect(IsTimestampAlignedToReference(528077.0, 528077.0 + 5.0e-7, 1.0e-6),
         "GNSS exact-time alignment should still accept timestamps within time_tolerance");
}

void TestGnssPositionAlignmentFallsBackWithoutVelocityData() {
  Dataset dataset;
  dataset.gnss.timestamps = VectorXd(3);
  dataset.gnss.timestamps << 100.0, 101.0, 102.0;
  dataset.gnss.positions = MatrixXd(3, 3);
  dataset.gnss.positions << 0.0, 0.0, 0.0,
                            10.0, 0.0, 0.0,
                            20.0, 0.0, 0.0;
  dataset.gnss.std = MatrixXd::Constant(3, 3, 0.2);

  NoiseParams noise;
  noise.sigma_gnss_pos = 1.0;

  Vector3d gnss_pos = Vector3d::Zero();
  Vector3d gnss_std = Vector3d::Zero();
  bool ok = ComputeAlignedGnssPositionMeasurement(dataset, noise, 1, 101.2,
                                                  gnss_pos, gnss_std);
  Expect(ok, "ComputeAlignedGnssPositionMeasurement should succeed without GNSS velocity columns");
  Expect(std::abs(gnss_pos.x() - 10.0) <= 1e-12,
         "GNSS position measurement should stay at the raw GNSS timestamp instead of being extrapolated");
  Expect(std::abs(gnss_std.x() - 0.2) <= 1e-12 &&
             std::abs(gnss_std.y() - 0.2) <= 1e-12 &&
             std::abs(gnss_std.z() - 0.2) <= 1e-12,
         "GNSS position alignment should preserve valid per-axis std values");
}

void TestPredictKeepsNominalImuErrorStatesPiecewiseConstant() {
  NoiseParams noise;
  noise.sigma_acc = 0.05;
  noise.sigma_gyro = 0.005;
  noise.sigma_ba = 1.0e-4;
  noise.sigma_bg = 1.0e-4;
  noise.sigma_sg = 1.0e-4;
  noise.sigma_sa = 1.0e-4;
  noise.markov_corr_time = 60.0;

  EskfEngine engine(noise);
  State state;
  state.p = LlhToEcef(30.0 * EIGEN_PI / 180.0, 114.0 * EIGEN_PI / 180.0, 10.0);
  state.ba = Vector3d(0.11, -0.22, 0.33);
  state.bg = Vector3d(-0.44, 0.55, -0.66);
  state.sg = Vector3d(100.0, -200.0, 300.0) * 1.0e-6;
  state.sa = Vector3d(-400.0, 500.0, -600.0) * 1.0e-6;

  Matrix<double, kStateDim, kStateDim> P0 =
      Matrix<double, kStateDim, kStateDim>::Identity();
  engine.Initialize(state, P0);

  ImuData imu0;
  imu0.t = 0.0;
  imu0.dt = 0.0;
  imu0.dtheta.setZero();
  imu0.dvel.setZero();

  ImuData imu1;
  imu1.t = 0.01;
  imu1.dt = 0.01;
  imu1.dtheta = Vector3d(1.0e-4, -2.0e-4, 3.0e-4);
  imu1.dvel = Vector3d(0.01, -0.02, 0.03);

  engine.AddImu(imu0);
  engine.AddImu(imu1);
  Expect(engine.Predict(),
         "Predict should succeed for nominal IMU error piecewise-constant regression");

  Expect((engine.state().ba - state.ba).norm() <= 1.0e-15,
         "nominal accel bias should stay piecewise constant between updates");
  Expect((engine.state().bg - state.bg).norm() <= 1.0e-15,
         "nominal gyro bias should stay piecewise constant between updates");
  Expect((engine.state().sg - state.sg).norm() <= 1.0e-15,
         "nominal gyro scale should stay piecewise constant between updates");
  Expect((engine.state().sa - state.sa).norm() <= 1.0e-15,
         "nominal accel scale should stay piecewise constant between updates");
}

void TestVectorNoiseParsingAndStateSeriesPath() {
  const string temp_path = "__regression_vector_noise_config__.yaml";
  WriteTextFile(
      temp_path,
      "common:\n"
      "  anchors:\n"
      "    mode: fixed\n"
      "    positions: []\n"
      "  gating:\n"
      "    uwb_residual_max: 1.0\n"
      "    time_tolerance: 1.0e-6\n"
      "fusion:\n"
      "  imu_path: imu.txt\n"
      "  odo_path: odo.txt\n"
      "  pos_path: pos.txt\n"
      "  output_path: out.txt\n"
      "  state_series_output_path: tmp/state_series.csv\n"
      "  noise:\n"
      "    sigma_acc: 1.0\n"
      "    sigma_gyro: 1.0\n"
      "    sigma_ba: 0.0\n"
      "    sigma_ba_vec: [0.11, 0.12, 0.13]\n"
      "    sigma_bg: 0.0\n"
      "    sigma_bg_vec: [0.21, 0.22, 0.23]\n"
      "    sigma_sg: 0.0\n"
      "    sigma_sg_vec: [0.31, 0.32, 0.33]\n"
      "    sigma_sa: 0.0\n"
      "    sigma_sa_vec: [0.41, 0.42, 0.43]\n"
      "    sigma_odo_scale: 1.0\n"
      "    sigma_mounting: 0.0\n"
      "    sigma_lever_arm: 0.0\n"
      "    sigma_lever_arm_vec: [0.51, 0.52, 0.53]\n"
      "    sigma_gnss_lever_arm: 0.0\n"
      "    sigma_gnss_lever_arm_vec: [0.61, 0.62, 0.63]\n"
      "    sigma_gnss_pos: 1.0\n"
      "    sigma_uwb: 1.0\n"
      "  constraints:\n"
      "    enable_nhc: false\n"
      "    enable_odo: false\n"
      "    enable_zupt: false\n"
      "    sigma_odo: 1.0\n"
      "    sigma_nhc_y: 1.0\n"
      "    sigma_nhc_z: 1.0\n"
      "    sigma_zupt: 1.0\n");

  FusionOptions options = LoadFusionOptions(temp_path);
  Expect(options.state_series_output_path == "tmp/state_series.csv",
         "state_series_output_path should be parsed from YAML");
  Expect(NearlyEqual(options.noise.sigma_ba_vec.x(), 0.11) &&
             NearlyEqual(options.noise.sigma_ba_vec.z(), 0.13),
         "sigma_ba_vec parsing failed");
  Expect(NearlyEqual(options.noise.sigma_bg_vec.y(), 0.22),
         "sigma_bg_vec parsing failed");
  Expect(NearlyEqual(options.noise.sigma_sg_vec.z(), 0.33),
         "sigma_sg_vec parsing failed");
  Expect(NearlyEqual(options.noise.sigma_sa_vec.x(), 0.41),
         "sigma_sa_vec parsing failed");
  Expect(NearlyEqual(options.noise.sigma_lever_arm_vec.y(), 0.52),
         "sigma_lever_arm_vec parsing failed");
  Expect(NearlyEqual(options.noise.sigma_gnss_lever_arm_vec.z(), 0.63),
         "sigma_gnss_lever_arm_vec parsing failed");
  fs::remove(temp_path);
}

void TestParseStandardResetGammaFlag() {
  const string temp_path = "__regression_standard_reset_gamma.yaml";
  WriteTextFile(
      temp_path,
      "common:\n"
      "  anchors:\n"
      "    mode: fixed\n"
      "    positions: []\n"
      "  gating:\n"
      "    uwb_residual_max: 1.0\n"
      "    time_tolerance: 1.0e-6\n"
      "fusion:\n"
      "  imu_path: imu.txt\n"
      "  odo_path: odo.txt\n"
      "  pos_path: pos.txt\n"
      "  output_path: out.txt\n"
      "  fej:\n"
      "    enable: false\n"
      "    debug_enable_standard_reset_gamma: true\n"
      "  noise:\n"
      "    sigma_acc: 1.0\n"
      "    sigma_gyro: 1.0\n"
      "    sigma_ba: 1.0\n"
      "    sigma_bg: 1.0\n"
      "    sigma_sg: 0.0\n"
      "    sigma_sa: 0.0\n"
      "    sigma_odo_scale: 1.0\n"
      "    sigma_mounting: 0.0\n"
      "    sigma_lever_arm: 0.0\n"
      "    sigma_gnss_lever_arm: 0.0\n"
      "    sigma_gnss_pos: 1.0\n"
      "    sigma_uwb: 1.0\n"
      "  constraints:\n"
      "    enable_nhc: false\n"
      "    enable_odo: false\n"
      "    enable_zupt: false\n"
      "    sigma_odo: 1.0\n"
      "    sigma_nhc_y: 1.0\n"
      "    sigma_nhc_z: 1.0\n"
      "    sigma_zupt: 1.0\n");

  FusionOptions options = LoadFusionOptions(temp_path);
  Expect(options.fej.debug_enable_standard_reset_gamma,
         "debug_enable_standard_reset_gamma should be parsed from fusion.fej");
  fs::remove(temp_path);
}

void TestProcessModelVectorPriorityBgZ() {
  NoiseParams noise;
  noise.sigma_acc = 0.05;
  noise.sigma_gyro = 0.005;
  noise.sigma_ba = 1.0e-6;
  noise.sigma_bg = 1.0e-6;
  noise.sigma_sg = 1.0e-6;
  noise.sigma_sa = 1.0e-6;
  noise.sigma_odo_scale = 1.0e-6;
  noise.sigma_mounting = 0.0;
  noise.sigma_mounting_roll = 0.0;
  noise.sigma_mounting_pitch = 0.0;
  noise.sigma_mounting_yaw = 0.0;
  noise.sigma_lever_arm = 1.0e-6;
  noise.sigma_gnss_lever_arm = 1.0e-6;
  noise.sigma_bg_vec = Vector3d(1.0e-6, 1.0e-6, 1.0e-3);

  Matrix<double, kStateDim, kStateDim> Phi =
      Matrix<double, kStateDim, kStateDim>::Zero();
  Matrix<double, kStateDim, kStateDim> Qd =
      Matrix<double, kStateDim, kStateDim>::Zero();
  InsMech::BuildProcessModel(Matrix3d::Identity(), Vector3d::Zero(),
                             Vector3d::Zero(), Vector3d::Zero(),
                             Vector3d::Zero(), Vector3d::Ones(),
                             Vector3d::Ones(), Vector3d::Zero(),
                             30.0 * EIGEN_PI / 180.0, 20.0, 1.0,
                             noise, Phi, Qd, nullptr);

  double bgx = Qd(StateIdx::kBg + 0, StateIdx::kBg + 0);
  double bgy = Qd(StateIdx::kBg + 1, StateIdx::kBg + 1);
  double bgz = Qd(StateIdx::kBg + 2, StateIdx::kBg + 2);
  Expect(bgz > bgx * 1.0e4,
         "bg_z process noise should dominate when sigma_bg_vec only magnifies z");
  Expect(std::abs(bgx - bgy) <= std::max(1.0e-18, bgx * 1.0e-6),
         "bg_x and bg_y should stay equal when their vector noise entries match");
}

void TestProcessModelBiasScaleCouplingMatchesNominalCorrection() {
  NoiseParams noise;
  noise.sigma_acc = 0.05;
  noise.sigma_gyro = 0.005;
  noise.sigma_ba = 1.0e-6;
  noise.sigma_bg = 1.0e-6;
  noise.sigma_sg = 1.0e-6;
  noise.sigma_sa = 1.0e-6;
  noise.sigma_odo_scale = 1.0e-6;
  noise.sigma_mounting = 0.0;
  noise.sigma_mounting_roll = 0.0;
  noise.sigma_mounting_pitch = 0.0;
  noise.sigma_mounting_yaw = 0.0;
  noise.sigma_lever_arm = 1.0e-6;
  noise.sigma_gnss_lever_arm = 1.0e-6;

  const double dt = 0.01;
  const Vector3d sf_a(1.0 / 1.11, 1.0 / 1.23, 1.0 / 1.34);
  const Vector3d sf_g(1.0 / 1.07, 1.0 / 1.18, 1.0 / 1.29);
  const Vector3d f_unbiased(4.0, -3.0, 8.0);
  const Vector3d omega_unbiased(0.1, -0.2, 0.3);
  const Vector3d f_corrected = sf_a.cwiseProduct(f_unbiased);
  const Vector3d omega_corrected = sf_g.cwiseProduct(omega_unbiased);

  Matrix<double, kStateDim, kStateDim> Phi =
      Matrix<double, kStateDim, kStateDim>::Zero();
  Matrix<double, kStateDim, kStateDim> Qd =
      Matrix<double, kStateDim, kStateDim>::Zero();
  InsMech::BuildProcessModel(Matrix3d::Identity(), f_corrected,
                             omega_corrected, f_unbiased, omega_unbiased,
                             sf_a, sf_g, Vector3d::Zero(),
                             30.0 * EIGEN_PI / 180.0, 20.0, dt,
                             noise, Phi, Qd, nullptr);

  const Matrix<double, kStateDim, kStateDim> F =
      (Phi - Matrix<double, kStateDim, kStateDim>::Identity()) / dt;
  const Matrix3d F_vba = F.block<3, 3>(StateIdx::kVel, StateIdx::kBa);
  const Matrix3d F_vsa = F.block<3, 3>(StateIdx::kVel, StateIdx::kSa);
  const Matrix3d F_phibg = F.block<3, 3>(StateIdx::kAtt, StateIdx::kBg);
  const Matrix3d F_phisg = F.block<3, 3>(StateIdx::kAtt, StateIdx::kSg);
  const Matrix3d expected_F_vba = -sf_a.asDiagonal().toDenseMatrix();
  const Matrix3d expected_F_vsa =
      -(f_unbiased.cwiseProduct(sf_a.cwiseProduct(sf_a))).asDiagonal().toDenseMatrix();
  const Matrix3d expected_F_phibg = -sf_g.asDiagonal().toDenseMatrix();
  const Matrix3d expected_F_phisg =
      -(omega_unbiased.cwiseProduct(sf_g.cwiseProduct(sf_g))).asDiagonal().toDenseMatrix();

  Expect((F_vba - expected_F_vba).cwiseAbs().maxCoeff() <= 1.0e-12,
         "F_vba should include reciprocal accel scale correction diag((1+sa)^-1)");
  Expect((F_vsa - expected_F_vsa).cwiseAbs().maxCoeff() <= 1.0e-12,
         "F_vsa should use reciprocal-scale sensitivity f_unbiased/(1+sa)^2");
  Expect((F_phibg - expected_F_phibg).cwiseAbs().maxCoeff() <= 1.0e-12,
         "F_phibg should include reciprocal gyro scale correction diag((1+sg)^-1)");
  Expect((F_phisg - expected_F_phisg).cwiseAbs().maxCoeff() <=
             1.0e-12,
         "F_phisg should use reciprocal-scale sensitivity omega/(1+sg)^2");
}

void TestInsMechanizationUsesReciprocalScaleCompensation() {
  constexpr double kOmegaEarthForTest = 7.292115e-5;
  State state;
  state.p = LlhToEcef(30.0 * EIGEN_PI / 180.0, 114.0 * EIGEN_PI / 180.0, 10.0);
  state.q = Vector4d(1.0, 0.0, 0.0, 0.0);
  state.ba.setZero();
  state.bg.setZero();
  state.sg = Vector3d(0.10, 0.20, 0.25);
  state.sa = Vector3d(0.05, 0.10, 0.20);

  ImuData imu_prev;
  imu_prev.t = 0.0;
  imu_prev.dt = 0.0;
  imu_prev.dtheta.setZero();
  imu_prev.dvel.setZero();

  ImuData imu_curr;
  imu_curr.t = 0.02;
  imu_curr.dt = 0.02;

  imu_curr.dtheta = Vector3d(0.02, 0.024, 0.03);
  imu_curr.dvel.setZero();
  PropagationResult gyro_only = InsMech::Propagate(state, imu_prev, imu_curr);
  const Vector3d omega_ie_b = kOmegaEarthForTest * Vector3d::UnitZ();
  const Vector3d expected_omega =
      (imu_curr.dtheta.cwiseQuotient(Vector3d::Ones() + state.sg) -
       omega_ie_b * imu_curr.dt) /
      imu_curr.dt;
  Expect((gyro_only.omega_b - expected_omega).norm() <= 1.0e-12,
         "mechanization should compensate gyro scale with reciprocal 1/(1+sg)");

  imu_curr.dtheta.setZero();
  imu_curr.dvel = Vector3d(0.0, 0.0, 0.24);
  PropagationResult accel_only = InsMech::Propagate(state, imu_prev, imu_curr);
  const Vector3d dvel_corrected =
      imu_curr.dvel.cwiseQuotient(Vector3d::Ones() + state.sa);
  const Vector3d expected_fb = dvel_corrected / imu_curr.dt;
  Expect((accel_only.f_b - expected_fb).norm() <= 1.0e-12,
         "mechanization should compensate accel scale with reciprocal 1/(1+sa)");
}

void TestRunFusionGnssUpdateOccursAtGnssTimestamp() {
  FusionOptions options;
  options.enable_gnss_velocity = false;
  options.constraints.enable_nhc = false;
  options.constraints.enable_odo = false;
  options.constraints.enable_zupt = false;
  options.noise.sigma_acc = 0.05;
  options.noise.sigma_gyro = 0.005;
  options.noise.sigma_gnss_pos = 1.0e-4;

  State x0;
  x0.p = LlhToEcef(0.0, 0.0, 0.0);
  x0.v = Vector3d(0.0, 1.0, 0.0);

  Matrix<double, kStateDim, kStateDim> P0 =
      Matrix<double, kStateDim, kStateDim>::Identity();
  P0.block<3, 3>(StateIdx::kPos, StateIdx::kPos) = Matrix3d::Identity() * 100.0;
  P0.block<3, 3>(StateIdx::kVel, StateIdx::kVel) = Matrix3d::Identity() * 100.0;

  Dataset dataset;
  dataset.imu.resize(2);
  dataset.imu[0].t = 0.0;
  dataset.imu[0].dt = 0.0;
  dataset.imu[0].dtheta.setZero();
  dataset.imu[0].dvel.setZero();
  dataset.imu[1].t = 0.02;
  dataset.imu[1].dt = 0.02;
  dataset.imu[1].dtheta.setZero();
  dataset.imu[1].dvel.setZero();

  const double t_gnss = 0.01;
  dataset.gnss.timestamps = VectorXd(1);
  dataset.gnss.timestamps << t_gnss;
  dataset.gnss.positions = MatrixXd(1, 3);
  const Vector3d gravity_e = GravityEcef(x0.p);
  const Vector3d gnss_pos =
      x0.p + x0.v * t_gnss + 0.5 * gravity_e * t_gnss * t_gnss;
  dataset.gnss.positions.row(0) = gnss_pos.transpose();
  dataset.gnss.std = MatrixXd::Constant(1, 3, 1.0e-4);

  FusionResult result = RunFusion(options, dataset, x0, P0, nullptr);
  Expect(!result.fused_positions.empty(),
         "RunFusion should produce at least one fused state for GNSS timing regression");
  const Vector3d final_pos = result.fused_positions.back();
  const double final_y = final_pos.y() - x0.p.y();
  Expect(final_y > 0.015,
         "GNSS update should occur at the GNSS timestamp, then propagate the remaining IMU segment");
}

void TestSaveStateSeriesSmoke() {
  const string temp_path = "__regression_state_series_smoke.csv";
  FusionOptions options;
  options.constraints.imu_mounting_angle = Vector3d(0.0, 0.2, 1.0);
  options.init.use_legacy_mounting_base_logic = false;

  FusionResult result;
  result.time_axis.push_back(1.0);
  result.ba.push_back(Vector3d(1.0e-5, -2.0e-5, 3.0e-5));
  result.bg.push_back(Vector3d(1.0e-4, -2.0e-4, 3.0e-4));
  result.sg.push_back(Vector3d(1.0e-6, 2.0e-6, 3.0e-6));
  result.sa.push_back(Vector3d(4.0e-6, 5.0e-6, 6.0e-6));
  result.odo_scale.push_back(1.01);
  result.mounting_roll.push_back(0.1 * EIGEN_PI / 180.0);
  result.mounting_pitch.push_back(0.2 * EIGEN_PI / 180.0);
  result.mounting_yaw.push_back(0.3 * EIGEN_PI / 180.0);
  result.lever_arm.push_back(Vector3d(0.1, 0.2, 0.3));
  result.gnss_lever_arm.push_back(Vector3d(0.4, 0.5, 0.6));

  SaveStateSeries(temp_path, result, options);

  {
    ifstream fin(temp_path);
    Expect(static_cast<bool>(fin), "SaveStateSeries should create output CSV");
    string header;
    string row;
    getline(fin, header);
    getline(fin, row);
    Expect(header.find("mounting_roll_deg") != string::npos,
           "SaveStateSeries header should include mounting_roll_deg");
    Expect(header.find("total_mounting_yaw_deg") != string::npos,
           "SaveStateSeries header should include total_mounting_yaw_deg");
    Expect(!row.empty() && row.find(',') != string::npos,
           "SaveStateSeries should write at least one CSV row");
  }
  fs::remove(temp_path);
}

}  // namespace

int main() {
  try {
    TestLoadOptionsFailFast();
    TestCustomP0DiagTakesPrecedence();
    TestInitializeStateInterpolatesTruthToFirstImuTime();
    TestDiagnosticsCorrectPropagatesFailure();
    TestBuildAnchorsContracts();
    TestData2BaselineOfficialConfig();
    TestGnssVelocityAvailabilityIndependentFromEnableFlag();
    TestTimestampAlignmentUsesConfiguredTolerance();
    TestGnssPositionAlignmentFallsBackWithoutVelocityData();
    TestPredictKeepsNominalImuErrorStatesPiecewiseConstant();
    TestVectorNoiseParsingAndStateSeriesPath();
    TestParseStandardResetGammaFlag();
    TestProcessModelVectorPriorityBgZ();
    TestProcessModelBiasScaleCouplingMatchesNominalCorrection();
    TestInsMechanizationUsesReciprocalScaleCompensation();
    TestRunFusionGnssUpdateOccursAtGnssTimestamp();
    TestSaveStateSeriesSmoke();
    cout << "regression_checks: PASS\n";
  } catch (const exception &e) {
    cerr << "regression_checks: FAIL: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
