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
#include "io/data_io.h"
#include "navigation/filter_contracts.h"
#include "navigation/filter_engine.h"
#include "navigation/measurement_model.h"
#include "navigation/process_model.h"

using namespace std;
using namespace Eigen;
namespace fs = std::filesystem;

namespace {

constexpr double kDegToRad = EIGEN_PI / 180.0;

void Expect(bool cond, const string &message) {
  if (!cond) {
    throw runtime_error(message);
  }
}

bool NearlyEqual(double a, double b, double tol = 1e-12) {
  return std::abs(a - b) <= tol;
}

void WriteTextFile(const string &path, const string &text) {
  ofstream fout = io::OpenOutputFile(path);
  Expect(static_cast<bool>(fout), "failed to create temp file: " + path);
  fout << text;
}

string ReadTextFile(const string &path) {
  ifstream fin(path, ios::binary);
  Expect(static_cast<bool>(fin), "failed to open text file: " + path);
  return string((istreambuf_iterator<char>(fin)),
                istreambuf_iterator<char>());
}

size_t CountOccurrences(const string &text, const string &needle) {
  if (needle.empty()) {
    return 0;
  }
  size_t count = 0;
  size_t pos = 0;
  while ((pos = text.find(needle, pos)) != string::npos) {
    ++count;
    pos += needle.size();
  }
  return count;
}

Vector3d RotationVectorFromQuat(const Vector4d &q_delta_raw) {
  Vector4d q_delta = NormalizeQuat(q_delta_raw);
  if (q_delta[0] < 0.0) {
    q_delta = -q_delta;
  }
  const Vector3d imag = q_delta.tail<3>();
  const double imag_norm = imag.norm();
  if (imag_norm < 1.0e-12) {
    return 2.0 * imag;
  }
  const double angle = 2.0 * atan2(imag_norm, Clamp(q_delta[0], -1.0, 1.0));
  return imag * (angle / imag_norm);
}

State ApplyStandardEskfErrorState(const State &nominal, const VectorXd &dx) {
  State out = nominal;
  const Vector3d dr_ned = dx.segment<3>(StateIdx::kPos);
  const Vector3d dv_ned = dx.segment<3>(StateIdx::kVel);
  const Vector3d dphi_ned = dx.segment<3>(StateIdx::kAtt);

  const Llh llh = EcefToLlh(nominal.p);
  const Matrix3d R_ne = RotNedToEcef(llh);
  out.p += R_ne * dr_ned;
  out.v += R_ne * dv_ned;

  const Vector3d dphi_ecef = R_ne * dphi_ned;
  const Vector4d dq = QuatFromSmallAngle(-dphi_ecef);
  out.q = NormalizeQuat(QuatMultiply(dq, out.q));

  out.ba += dx.segment<3>(StateIdx::kBa);
  out.bg += dx.segment<3>(StateIdx::kBg);
  out.sg += dx.segment<3>(StateIdx::kSg);
  out.sa += dx.segment<3>(StateIdx::kSa);
  out.odo_scale += dx(StateIdx::kOdoScale);
  out.mounting_roll += dx(StateIdx::kMountRoll);
  out.mounting_pitch += dx(StateIdx::kMountPitch);
  out.mounting_yaw += dx(StateIdx::kMountYaw);
  out.lever_arm += dx.segment<3>(StateIdx::kLever);
  out.gnss_lever_arm += dx.segment<3>(StateIdx::kGnssLever);
  return out;
}

State ApplyHistoricalStandardEskfErrorState(const State &nominal,
                                            const VectorXd &dx) {
  State out = nominal;
  const Vector3d dr_ned = dx.segment<3>(StateIdx::kPos);
  const Vector3d dv_ned = dx.segment<3>(StateIdx::kVel);
  const Vector3d dphi_ned = dx.segment<3>(StateIdx::kAtt);

  const Llh llh = EcefToLlh(nominal.p);
  const Matrix3d R_ne = RotNedToEcef(llh);
  out.p += R_ne * dr_ned;
  out.v += R_ne * dv_ned;

  const Vector3d dphi_ecef = R_ne * dphi_ned;
  const Vector4d dq = QuatFromSmallAngle(-dphi_ecef);
  out.q = NormalizeQuat(QuatMultiply(dq, out.q));

  out.ba += dx.segment<3>(StateIdx::kBa);
  out.bg += dx.segment<3>(StateIdx::kBg);
  out.sg += dx.segment<3>(StateIdx::kSg);
  out.sa += dx.segment<3>(StateIdx::kSa);
  out.odo_scale += dx(StateIdx::kOdoScale);
  out.mounting_roll += dx(StateIdx::kMountRoll);
  out.mounting_pitch += dx(StateIdx::kMountPitch);
  out.mounting_yaw += dx(StateIdx::kMountYaw);
  out.lever_arm += dx.segment<3>(StateIdx::kLever);
  out.gnss_lever_arm += dx.segment<3>(StateIdx::kGnssLever);
  return out;
}

VectorXd ComputeStandardEskfErrorState(const State &nominal,
                                       const State &perturbed) {
  VectorXd dx = VectorXd::Zero(kStateDim);

  const Llh llh = EcefToLlh(nominal.p);
  const Matrix3d R_ne = RotNedToEcef(llh);
  dx.segment<3>(StateIdx::kPos) = R_ne.transpose() * (perturbed.p - nominal.p);
  dx.segment<3>(StateIdx::kVel) = R_ne.transpose() * (perturbed.v - nominal.v);

  const Vector4d q_delta =
      NormalizeQuat(QuatMultiply(perturbed.q, QuatConjugate(nominal.q)));
  const Vector3d dphi_ecef = -RotationVectorFromQuat(q_delta);
  dx.segment<3>(StateIdx::kAtt) = R_ne.transpose() * dphi_ecef;

  dx.segment<3>(StateIdx::kBa) = perturbed.ba - nominal.ba;
  dx.segment<3>(StateIdx::kBg) = perturbed.bg - nominal.bg;
  dx.segment<3>(StateIdx::kSg) = perturbed.sg - nominal.sg;
  dx.segment<3>(StateIdx::kSa) = perturbed.sa - nominal.sa;
  dx(StateIdx::kOdoScale) = perturbed.odo_scale - nominal.odo_scale;
  dx(StateIdx::kMountRoll) = perturbed.mounting_roll - nominal.mounting_roll;
  dx(StateIdx::kMountPitch) = perturbed.mounting_pitch - nominal.mounting_pitch;
  dx(StateIdx::kMountYaw) = perturbed.mounting_yaw - nominal.mounting_yaw;
  dx.segment<3>(StateIdx::kLever) = perturbed.lever_arm - nominal.lever_arm;
  dx.segment<3>(StateIdx::kGnssLever) =
      perturbed.gnss_lever_arm - nominal.gnss_lever_arm;
  return dx;
}

State ApplyUnifiedInEkfErrorState(const State &nominal, const VectorXd &dx) {
  State out = nominal;
  Vector3d dr_body = dx.segment<3>(StateIdx::kPos);
  Vector3d dv_body = dx.segment<3>(StateIdx::kVel);
  Vector3d dphi_body = dx.segment<3>(StateIdx::kAtt);

  const Llh llh = EcefToLlh(nominal.p);
  const Matrix3d R_ne = RotNedToEcef(llh);
  const Matrix3d C_bn = R_ne.transpose() * QuatToRot(nominal.q);

  out.p += R_ne * (C_bn * dr_body);
  out.v += R_ne * (C_bn * dv_body);
  const Vector4d dq = QuatFromSmallAngle(dphi_body);
  out.q = NormalizeQuat(QuatMultiply(out.q, dq));

  out.ba += dx.segment<3>(StateIdx::kBa);
  out.bg += dx.segment<3>(StateIdx::kBg);
  out.sg += dx.segment<3>(StateIdx::kSg);
  out.sa += dx.segment<3>(StateIdx::kSa);
  out.odo_scale += dx(StateIdx::kOdoScale);
  out.mounting_roll += dx(StateIdx::kMountRoll);
  out.mounting_pitch += dx(StateIdx::kMountPitch);
  out.mounting_yaw += dx(StateIdx::kMountYaw);
  out.lever_arm += dx.segment<3>(StateIdx::kLever);
  out.gnss_lever_arm += dx.segment<3>(StateIdx::kGnssLever);
  return out;
}

void BuildStandardEskfAnalyticPhi(const State &state, const ImuData &imu_prev,
                                  ImuData imu_curr, const NoiseParams &noise,
                                  Matrix<double, kStateDim, kStateDim> &Phi) {
  if (imu_curr.dt <= 0.0) {
    imu_curr.dt = imu_curr.t - imu_prev.t;
  }

  const PropagationResult res = InsMech::Propagate(state, imu_prev, imu_curr);
  const Llh llh = EcefToLlh(res.state.p);
  const Matrix3d R_ne = RotNedToEcef(llh);
  const Vector3d v_ned = R_ne.transpose() * res.state.v;

  Vector3d omega_ib_b_corr = Vector3d::Zero();
  Vector3d omega_ib_b_unbiased = Vector3d::Zero();
  Vector3d f_b_unbiased = Vector3d::Zero();
  Vector3d sf_g = Vector3d::Ones();
  Vector3d sf_a = Vector3d::Ones();
  if (imu_curr.dt > 1.0e-9) {
    const Vector3d omega_ib_b_raw = imu_curr.dtheta / imu_curr.dt;
    omega_ib_b_unbiased = omega_ib_b_raw - state.bg;
    f_b_unbiased = imu_curr.dvel / imu_curr.dt - state.ba;
    sf_g = (Vector3d::Ones() + state.sg).cwiseInverse();
    sf_a = (Vector3d::Ones() + state.sa).cwiseInverse();
    omega_ib_b_corr = sf_g.cwiseProduct(omega_ib_b_unbiased);
  }

  Matrix<double, kStateDim, kStateDim> Qd =
      Matrix<double, kStateDim, kStateDim>::Zero();
  InsMech::BuildProcessModel(R_ne.transpose() * res.Cbn, res.f_b,
                             omega_ib_b_corr, f_b_unbiased,
                             omega_ib_b_unbiased, sf_a, sf_g, v_ned, llh.lat,
                             llh.h, imu_curr.dt, noise, Phi, Qd, nullptr);
}

ProcessModelInput BuildStandardEskfProcessModelInputForRegression() {
  ProcessModelInput input;
  input.semantics = BuildStandardEskfSemantics();
  input.noise.sigma_acc = 0.05;
  input.noise.sigma_gyro = 0.005;
  input.noise.sigma_ba = 1.0e-6;
  input.noise.sigma_bg = 1.0e-6;
  input.noise.sigma_sg = 1.0e-6;
  input.noise.sigma_sa = 1.0e-6;
  input.noise.markov_corr_time = 3600.0;

  input.nominal.p = LlhToEcef(31.0 * EIGEN_PI / 180.0, 114.0 * EIGEN_PI / 180.0, 15.0);
  input.nominal.v = Vector3d(6.0, -1.5, 0.8);
  input.nominal.q = NormalizeQuat(RpyToQuat(Vector3d(0.05, -0.03, 0.1)));
  input.nominal.ba = Vector3d(0.015, -0.008, 0.02);
  input.nominal.bg = Vector3d(-1.0e-4, 2.0e-4, -1.5e-4);
  input.nominal.sg = Vector3d(120.0, -80.0, 40.0) * 1.0e-6;
  input.nominal.sa = Vector3d(-150.0, 100.0, 60.0) * 1.0e-6;

  input.imu_prev.t = 0.0;
  input.imu_prev.dt = 0.0;
  input.imu_prev.dtheta = Vector3d::Zero();
  input.imu_prev.dvel = Vector3d::Zero();

  input.imu_curr.t = 1.0e-4;
  input.imu_curr.dt = 1.0e-4;
  input.imu_curr.dtheta = Vector3d(2.0e-6, -1.0e-6, 3.0e-6);
  input.imu_curr.dvel = Vector3d(5.0e-4, -2.0e-4, 1.0e-3);
  return input;
}

State BuildMeasurementContractState() {
  State state;
  state.p = LlhToEcef(31.2 * EIGEN_PI / 180.0, 121.5 * EIGEN_PI / 180.0, 18.0);
  state.v = Vector3d(4.5, -1.2, 0.6);
  state.q = NormalizeQuat(RpyToQuat(Vector3d(0.06, -0.02, 0.15)));
  state.ba = Vector3d(0.01, -0.015, 0.02);
  state.bg = Vector3d(-1.5e-4, 0.5e-4, -2.0e-4);
  state.sg = Vector3d(45.0, -35.0, 25.0) * 1.0e-6;
  state.sa = Vector3d(-55.0, 65.0, -30.0) * 1.0e-6;
  state.odo_scale = 1.003;
  state.mounting_pitch = -0.4 * kDegToRad;
  state.mounting_yaw = 0.8 * kDegToRad;
  state.lever_arm = Vector3d(0.35, -0.12, 0.08);
  state.gnss_lever_arm = Vector3d(0.95, 0.18, -0.10);
  return state;
}

ProcessLinearization BuildFiniteDifferenceProcessLinearization(
    const ProcessModelInput &input) {
  ProcessLinearization numeric = BuildStandardEskfProcessLinearization(input);
  numeric.Phi.setIdentity();
  numeric.F.setZero();

  const State nominal_next =
      BuildNominalPropagation(input.nominal, input.imu_prev, input.imu_curr).state;
  VectorXd eps = VectorXd::Zero(kStateDim);
  eps.segment<3>(StateIdx::kBa).setConstant(1.0e-5);
  eps.segment<3>(StateIdx::kBg).setConstant(1.0e-6);
  eps.segment<3>(StateIdx::kSg).setConstant(1.0e-6);
  eps.segment<3>(StateIdx::kSa).setConstant(1.0e-6);
  for (int j = StateIdx::kBa; j < StateIdx::kSa + 3; ++j) {
    VectorXd dx = VectorXd::Zero(kStateDim);
    dx(j) = eps(j);
    const State plus0 = ApplyStandardEskfErrorState(input.nominal, dx);
    const State minus0 = ApplyStandardEskfErrorState(input.nominal, -dx);
    const State plus1 =
        BuildNominalPropagation(plus0, input.imu_prev, input.imu_curr).state;
    const State minus1 =
        BuildNominalPropagation(minus0, input.imu_prev, input.imu_curr).state;
    const VectorXd dx_plus = ComputeStandardEskfErrorState(nominal_next, plus1);
    const VectorXd dx_minus = ComputeStandardEskfErrorState(nominal_next, minus1);
    numeric.Phi.col(j) = (dx_plus - dx_minus) / (2.0 * eps(j));
  }
  numeric.F = (numeric.Phi -
               Matrix<double, kStateDim, kStateDim>::Identity()) /
              input.imu_curr.dt;
  return numeric;
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
      "  inekf:\n"
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
  Expect(options.inekf.debug_enable_standard_reset_gamma,
         "debug_enable_standard_reset_gamma should be parsed from fusion.inekf");
  fs::remove(temp_path);
}

void TestLoadDatasetUsesSecondColumnAsTruthTimeForWeekSowFormat() {
  const string imu_path = "__regression_truth_week_imu__.txt";
  const string truth_path = "__regression_truth_week_truth.nav";

  WriteTextFile(
      imu_path,
      "456300.000 0 0 0 0 0 0\n"
      "456300.010 0 0 0 0 0 0\n");
  WriteTextFile(
      truth_path,
      "2017 456300.000 30.0000000000 114.0000000000 20.0 0.0 0.0 0.0 1.0 2.0 3.0\n"
      "2017 456300.010 30.0000001000 114.0000001000 20.1 0.0 0.0 0.0 1.1 2.1 3.1\n");

  FusionOptions options;
  options.imu_path = imu_path;
  options.pos_path = truth_path;
  options.odo_path.clear();
  options.gnss_path.clear();
  options.uwb_path.clear();
  options.output_path = "out.txt";
  options.start_time = 456300.0;
  options.final_time = 456300.01;
  options.gating.time_tolerance = 1.0e-6;
  options.gating.max_dt = 0.2;
  options.anchors.mode = "fixed";
  options.anchors.positions.clear();

  const Dataset data = LoadDataset(options);
  Expect(data.truth.timestamps.size() == 2,
         "LoadDataset should keep truth rows when truth.nav uses week+sow columns");
  Expect(std::abs(data.truth.timestamps(0) - 456300.0) <= 1.0e-9,
         "LoadDataset should use the second truth.nav column as time when the first column is GPS week/year");
  Expect(data.truth.positions.rows() == 2,
         "LoadDataset should parse week+sow truth positions into the cropped truth buffer");

  fs::remove(imu_path);
  fs::remove(truth_path);
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
  const Matrix3d expected_F_phibg = sf_g.asDiagonal().toDenseMatrix();
  const Matrix3d expected_F_phisg =
      (omega_unbiased.cwiseProduct(sf_g.cwiseProduct(sf_g))).asDiagonal().toDenseMatrix();

  Expect((F_vba - expected_F_vba).cwiseAbs().maxCoeff() <= 1.0e-12,
         "F_vba should include reciprocal accel scale correction diag((1+sa)^-1)");
  Expect((F_vsa - expected_F_vsa).cwiseAbs().maxCoeff() <= 1.0e-12,
         "F_vsa should use reciprocal-scale sensitivity f_unbiased/(1+sa)^2");
  Expect((F_phibg - expected_F_phibg).cwiseAbs().maxCoeff() <= 1.0e-12,
         "F_phibg should follow the standard ESKF attitude-error convention for gyro-bias coupling");
  Expect((F_phisg - expected_F_phisg).cwiseAbs().maxCoeff() <=
             1.0e-12,
         "F_phisg should follow the standard ESKF attitude-error convention for gyro-scale coupling");
}

void TestStandardEskfSemanticContract() {
  FilterSemantics semantics = BuildStandardEskfSemantics();
  Expect(semantics.residual_convention ==
             ResidualConvention::kMeasurementMinusPrediction,
         "standard ESKF residual must be y = z - h");
  Expect(semantics.imu_error_convention ==
             ImuErrorConvention::kTrueMinusNominal,
         "IMU bias/scale contract must be delta_b = b_true - b_hat");
  Expect(semantics.attitude_error_convention ==
             AttitudeErrorConvention::kStandardEskfRightError,
         "standard ESKF attitude contract must match q_true ~= q_hat ⊗ Exp(-phi)");
}

void TestEnabledInEkfSemanticsCollapseToInEkf() {
  InEkfConfig cfg;
  cfg.enabled = true;

  const FilterSemantics filter_semantics =
      BuildFilterSemanticsFromInEkfConfig(cfg);
  Expect(filter_semantics.flavor == FilterFlavor::kInEkf,
         "enabled InEKF should map to InEKF filter semantics");

  const FilterSemantics process_semantics =
      BuildProcessSemanticsFromInEkfConfig(cfg);
  Expect(process_semantics.flavor == FilterFlavor::kInEkf,
         "enabled InEKF should map to InEKF process semantics");

  const MeasurementModelContext context =
      BuildMeasurementModelContextFromInEkfConfig(&cfg);
  Expect(context.semantics.flavor == FilterFlavor::kInEkf,
         "measurement contexts built from enabled InEKF configs should use InEKF semantics");
}

void TestInEkfBuilderMatchesCanonicalSemantics() {
  ProcessModelResolvedInput input;
  input.semantics = BuildInEkfSemantics();
  input.noise.sigma_acc = 0.05;
  input.noise.sigma_gyro = 0.005;
  input.noise.sigma_ba = 1.0e-6;
  input.noise.sigma_bg = 1.0e-6;
  input.noise.sigma_sg = 1.0e-6;
  input.noise.sigma_sa = 1.0e-6;
  input.noise.markov_corr_time = 3600.0;
  input.C_bn = AngleAxisd(0.2, Vector3d::UnitZ()).toRotationMatrix() *
               AngleAxisd(-0.1, Vector3d::UnitY()).toRotationMatrix();
  input.f_b_unbiased = Vector3d(4.0, -3.0, 8.0);
  input.omega_ib_b_unbiased = Vector3d(0.1, -0.2, 0.3);
  input.sf_a = Vector3d(1.0 / 1.11, 1.0 / 1.23, 1.0 / 1.34);
  input.sf_g = Vector3d(1.0 / 1.07, 1.0 / 1.18, 1.0 / 1.29);
  input.f_b_corr = input.sf_a.cwiseProduct(input.f_b_unbiased);
  input.omega_ib_b_corr = input.sf_g.cwiseProduct(input.omega_ib_b_unbiased);
  input.v_ned = Vector3d(12.0, -4.0, 0.5);
  input.lat = 30.0 * EIGEN_PI / 180.0;
  input.h = 20.0;
  input.dt = 0.01;
  input.ri_vel_gyro_noise_mode = 1;

  const ProcessLinearization inekf_linearization =
      BuildProcessLinearization(input);
  const ProcessLinearization canonical_linearization =
      BuildInEkfProcessLinearization(input);

  Expect((inekf_linearization.F - canonical_linearization.F).cwiseAbs().maxCoeff() <=
             1.0e-12,
         "InEKF process builder should match the canonical InEKF F matrix");
  Expect((inekf_linearization.Phi - canonical_linearization.Phi).cwiseAbs().maxCoeff() <=
             1.0e-12,
         "InEKF process builder should match the canonical InEKF Phi matrix");
  Expect((inekf_linearization.Qd - canonical_linearization.Qd).cwiseAbs().maxCoeff() <=
             1.0e-12,
         "InEKF process builder should match the canonical InEKF Qd matrix");
}

void TestGnssPositionMeasurementContractUsesMeasurementMinusPrediction() {
  const MeasurementModelContext context =
      BuildMeasurementModelContext(BuildStandardEskfSemantics());
  const State state = BuildMeasurementContractState();
  const Vector3d z_ecef = state.p + Vector3d(2.0, -1.0, 0.5);
  const Vector3d sigma_gnss(0.3, 0.4, 0.5);

  GnssPositionMeasurementInput input;
  input.state = state;
  input.z_ecef = z_ecef;
  input.sigma_gnss = sigma_gnss;
  input.context = context;
  const MeasurementLinearization model = BuildGnssPositionMeasurement(input);
  Expect(model.residual_convention ==
             ResidualConvention::kMeasurementMinusPrediction,
         "GNSS position measurement residual convention drifted from y = z - h");
  Expect(model.model_name == "GNSS_POS",
         "GNSS position builder should advertise a stable model_name");
}

void TestGnssVelocityMeasurementContractUsesMeasurementMinusPrediction() {
  const MeasurementModelContext context =
      BuildMeasurementModelContext(BuildStandardEskfSemantics());
  const State state = BuildMeasurementContractState();
  const Vector3d z_vel_ecef = Vector3d(3.2, -0.4, 0.1);
  const Vector3d omega_ib_b_raw(0.015, -0.01, 0.02);
  const Vector3d sigma_gnss_vel(0.05, 0.06, 0.07);

  GnssVelocityMeasurementInput input;
  input.state = state;
  input.z_gnss_vel_ecef = z_vel_ecef;
  input.omega_ib_b_raw = omega_ib_b_raw;
  input.sigma_gnss_vel = sigma_gnss_vel;
  input.context = context;
  const MeasurementLinearization model = BuildGnssVelocityMeasurement(input);
  Expect(model.residual_convention ==
             ResidualConvention::kMeasurementMinusPrediction,
         "GNSS velocity measurement residual convention drifted from y = z - h");
  Expect(model.model_name == "GNSS_VEL",
         "GNSS velocity builder should advertise a stable model_name");
}

void TestRoadMeasurementContractsUseMeasurementMinusPrediction() {
  const MeasurementModelContext context =
      BuildMeasurementModelContext(BuildStandardEskfSemantics());
  const State state = BuildMeasurementContractState();
  const Matrix3d C_b_v =
      QuatToRot(RpyToQuat(Vector3d(0.0, -0.3 * kDegToRad, 1.2 * kDegToRad)))
          .transpose();
  const Vector3d omega_ib_b_raw(0.03, -0.02, 0.04);

  NhcMeasurementInput nhc_input;
  nhc_input.state = state;
  nhc_input.C_b_v = C_b_v;
  nhc_input.omega_ib_b_raw = omega_ib_b_raw;
  nhc_input.sigma_nhc_y = 0.2;
  nhc_input.sigma_nhc_z = 0.25;
  nhc_input.context = context;
  const MeasurementLinearization nhc = BuildNhcMeasurement(nhc_input);
  Expect(nhc.residual_convention ==
             ResidualConvention::kMeasurementMinusPrediction,
         "NHC residual convention drifted from y = z - h");
  Expect(nhc.model_name == "NHC",
         "NHC builder should advertise a stable model_name");

  OdoMeasurementInput odo_input;
  odo_input.state = state;
  odo_input.odo_speed = 5.4;
  odo_input.C_b_v = C_b_v;
  odo_input.omega_ib_b_raw = omega_ib_b_raw;
  odo_input.sigma_odo = 0.15;
  odo_input.context = context;
  const MeasurementLinearization odo = BuildOdoMeasurement(odo_input);
  Expect(odo.residual_convention ==
             ResidualConvention::kMeasurementMinusPrediction,
         "ODO residual convention drifted from y = z - h");
  Expect(odo.model_name == "ODO",
         "ODO builder should advertise a stable model_name");
}

void TestUwbMeasurementContractUsesMeasurementMinusPrediction() {
  const MeasurementModelContext context =
      BuildMeasurementModelContext(BuildStandardEskfSemantics());
  const State state = BuildMeasurementContractState();
  MatrixXd anchors(4, 3);
  anchors << state.p.transpose() + RowVector3d(5.0, 0.0, 0.0),
      state.p.transpose() + RowVector3d(0.0, 6.0, 0.0),
      state.p.transpose() + RowVector3d(0.0, 0.0, 4.0),
      state.p.transpose() + RowVector3d(-3.0, 2.0, 1.0);
  VectorXd z(4);
  z << 5.2, 6.1, 4.3, 3.8;

  UwbMeasurementInput input;
  input.state = state;
  input.z = z;
  input.anchors = anchors;
  input.sigma_uwb = 0.12;
  input.context = context;
  const MeasurementLinearization model = BuildUwbMeasurement(input);
  Expect(model.residual_convention ==
             ResidualConvention::kMeasurementMinusPrediction,
         "UWB residual convention drifted from y = z - h");
  Expect(model.model_name == "UWB",
         "UWB builder should advertise a stable model_name");
}

void TestLegacyMeasurementWrappersMatchUnifiedBuilders() {
  const State state = BuildMeasurementContractState();
  const Matrix3d C_b_v =
      QuatToRot(RpyToQuat(Vector3d(0.0, -0.3 * kDegToRad, 1.2 * kDegToRad)))
          .transpose();
  const Vector3d omega_ib_b_raw(0.03, -0.02, 0.04);
  const Vector3d gnss_pos = state.p + Vector3d(2.0, -1.0, 0.5);
  const Vector3d gnss_pos_sigma(0.3, 0.4, 0.5);
  const Vector3d gnss_vel(3.2, -0.4, 0.1);
  const Vector3d gnss_vel_sigma(0.05, 0.06, 0.07);

  MatrixXd anchors(4, 3);
  anchors << state.p.transpose() + RowVector3d(5.0, 0.0, 0.0),
      state.p.transpose() + RowVector3d(0.0, 6.0, 0.0),
      state.p.transpose() + RowVector3d(0.0, 0.0, 4.0),
      state.p.transpose() + RowVector3d(-3.0, 2.0, 1.0);
  VectorXd uwb_z(4);
  uwb_z << 5.2, 6.1, 4.3, 3.8;

  const auto expect_wrapper_matches_builder = [&](const InEkfConfig &cfg,
                                                  const string &tag) {
    const MeasurementModelContext context =
        BuildMeasurementModelContextFromInEkfConfig(&cfg);
    const auto nhc_legacy =
        MeasModels::ComputeNhcModel(state, C_b_v, omega_ib_b_raw, 0.2, 0.25,
                                    &cfg);
    NhcMeasurementInput nhc_input;
    nhc_input.state = state;
    nhc_input.C_b_v = C_b_v;
    nhc_input.omega_ib_b_raw = omega_ib_b_raw;
    nhc_input.sigma_nhc_y = 0.2;
    nhc_input.sigma_nhc_z = 0.25;
    nhc_input.context = context;
    const auto nhc_unified = BuildNhcMeasurement(nhc_input);
    Expect((nhc_legacy.y - nhc_unified.y).norm() <= 1.0e-12,
           "legacy NHC wrapper residual drifted from the unified builder (" +
               tag + ")");
    Expect((nhc_legacy.H - nhc_unified.H).norm() <= 1.0e-12,
           "legacy NHC wrapper Jacobian drifted from the unified builder (" +
               tag + ")");
    Expect((nhc_legacy.R - nhc_unified.R).norm() <= 1.0e-12,
           "legacy NHC wrapper noise drifted from the unified builder (" +
               tag + ")");

    const auto odo_legacy = MeasModels::ComputeOdoModel(
        state, 5.4, C_b_v, omega_ib_b_raw, 0.15, &cfg);
    OdoMeasurementInput odo_input;
    odo_input.state = state;
    odo_input.odo_speed = 5.4;
    odo_input.C_b_v = C_b_v;
    odo_input.omega_ib_b_raw = omega_ib_b_raw;
    odo_input.sigma_odo = 0.15;
    odo_input.context = context;
    const auto odo_unified = BuildOdoMeasurement(odo_input);
    Expect((odo_legacy.y - odo_unified.y).norm() <= 1.0e-12,
           "legacy ODO wrapper residual drifted from the unified builder (" +
               tag + ")");
    Expect((odo_legacy.H - odo_unified.H).norm() <= 1.0e-12,
           "legacy ODO wrapper Jacobian drifted from the unified builder (" +
               tag + ")");
    Expect((odo_legacy.R - odo_unified.R).norm() <= 1.0e-12,
           "legacy ODO wrapper noise drifted from the unified builder (" +
               tag + ")");

    const auto gnss_pos_legacy =
        MeasModels::ComputeGnssPositionModel(state, gnss_pos, gnss_pos_sigma,
                                             &cfg);
    GnssPositionMeasurementInput gnss_pos_input;
    gnss_pos_input.state = state;
    gnss_pos_input.z_ecef = gnss_pos;
    gnss_pos_input.sigma_gnss = gnss_pos_sigma;
    gnss_pos_input.context = context;
    const auto gnss_pos_unified =
        BuildGnssPositionMeasurement(gnss_pos_input);
    Expect((gnss_pos_legacy.y - gnss_pos_unified.y).norm() <= 1.0e-12,
           "legacy GNSS_POS wrapper residual drifted from the unified builder (" +
               tag + ")");
    Expect((gnss_pos_legacy.H - gnss_pos_unified.H).norm() <= 1.0e-12,
           "legacy GNSS_POS wrapper Jacobian drifted from the unified builder (" +
               tag + ")");
    Expect((gnss_pos_legacy.R - gnss_pos_unified.R).norm() <= 1.0e-12,
           "legacy GNSS_POS wrapper noise drifted from the unified builder (" +
               tag + ")");

    const auto gnss_vel_legacy = MeasModels::ComputeGnssVelocityModel(
        state, gnss_vel, omega_ib_b_raw, gnss_vel_sigma, &cfg);
    GnssVelocityMeasurementInput gnss_vel_input;
    gnss_vel_input.state = state;
    gnss_vel_input.z_gnss_vel_ecef = gnss_vel;
    gnss_vel_input.omega_ib_b_raw = omega_ib_b_raw;
    gnss_vel_input.sigma_gnss_vel = gnss_vel_sigma;
    gnss_vel_input.context = context;
    const auto gnss_vel_unified =
        BuildGnssVelocityMeasurement(gnss_vel_input);
    Expect((gnss_vel_legacy.y - gnss_vel_unified.y).norm() <= 1.0e-12,
           "legacy GNSS_VEL wrapper residual drifted from the unified builder (" +
               tag + ")");
    Expect((gnss_vel_legacy.H - gnss_vel_unified.H).norm() <= 1.0e-12,
           "legacy GNSS_VEL wrapper Jacobian drifted from the unified builder (" +
               tag + ")");
    Expect((gnss_vel_legacy.R - gnss_vel_unified.R).norm() <= 1.0e-12,
           "legacy GNSS_VEL wrapper noise drifted from the unified builder (" +
               tag + ")");
  };

  InEkfConfig standard_cfg;
  const MeasurementModelContext standard_context =
      BuildMeasurementModelContextFromInEkfConfig(&standard_cfg);
  const auto uwb_legacy =
      MeasModels::ComputeUwbModel(state, uwb_z, anchors, 0.12);
  UwbMeasurementInput uwb_input;
  uwb_input.state = state;
  uwb_input.z = uwb_z;
  uwb_input.anchors = anchors;
  uwb_input.sigma_uwb = 0.12;
  uwb_input.context = standard_context;
  const auto uwb_unified = BuildUwbMeasurement(uwb_input);
  Expect((uwb_legacy.y - uwb_unified.y).norm() <= 1.0e-12,
         "legacy UWB wrapper residual drifted from the unified builder");
  Expect((uwb_legacy.H - uwb_unified.H).norm() <= 1.0e-12,
         "legacy UWB wrapper Jacobian drifted from the unified builder");
  Expect((uwb_legacy.R - uwb_unified.R).norm() <= 1.0e-12,
         "legacy UWB wrapper noise drifted from the unified builder");

  const auto zupt_legacy = MeasModels::ComputeZuptModel(state, 0.08);
  ZuptMeasurementInput zupt_input;
  zupt_input.state = state;
  zupt_input.sigma_zupt = 0.08;
  zupt_input.context = standard_context;
  const auto zupt_unified = BuildZuptMeasurement(zupt_input);
  Expect((zupt_legacy.y - zupt_unified.y).norm() <= 1.0e-12,
         "legacy ZUPT wrapper residual drifted from the unified builder");
  Expect((zupt_legacy.H - zupt_unified.H).norm() <= 1.0e-12,
         "legacy ZUPT wrapper Jacobian drifted from the unified builder");
  Expect((zupt_legacy.R - zupt_unified.R).norm() <= 1.0e-12,
         "legacy ZUPT wrapper noise drifted from the unified builder");

  expect_wrapper_matches_builder(standard_cfg, "standard");

  InEkfConfig inekf_cfg;
  inekf_cfg.enabled = true;
  inekf_cfg.ri_gnss_pos_use_p_ned_local = true;
  inekf_cfg.p_init_ecef = state.p - Vector3d(2.0, -1.5, 0.8);
  expect_wrapper_matches_builder(inekf_cfg, "inekf");
}

void TestInEkfGnssPositionAttitudeJacobianMatchesFiniteDifference() {
  InEkfConfig cfg;
  cfg.enabled = true;
  cfg.ri_gnss_pos_use_p_ned_local = true;

  const MeasurementModelContext context =
      BuildMeasurementModelContextFromInEkfConfig(&cfg);
  const State state = BuildMeasurementContractState();
  const Llh llh = EcefToLlh(state.p);
  const Matrix3d R_ne = RotNedToEcef(llh);
  const Matrix3d C_bn = R_ne.transpose() * QuatToRot(state.q);
  const Vector3d body_target(1.2, -0.4, 0.3);
  const Vector3d z_ecef = state.p + R_ne * (C_bn * body_target);

  GnssPositionMeasurementInput input;
  input.state = state;
  input.z_ecef = z_ecef;
  input.sigma_gnss = Vector3d(0.3, 0.4, 0.5);
  input.context = context;
  const MeasurementLinearization analytic = BuildGnssPositionMeasurement(input);

  MatrixXd numeric = MatrixXd::Zero(3, kStateDim);
  VectorXd eps = VectorXd::Zero(kStateDim);
  eps.segment<3>(StateIdx::kPos).setConstant(1.0e-3);
  eps.segment<3>(StateIdx::kAtt).setConstant(1.0e-7);
  eps.segment<3>(StateIdx::kGnssLever).setConstant(1.0e-5);
  for (int j = 0; j < kStateDim; ++j) {
    if (eps(j) <= 0.0) {
      continue;
    }
    VectorXd dx = VectorXd::Zero(kStateDim);
    dx(j) = eps(j);
    GnssPositionMeasurementInput plus = input;
    GnssPositionMeasurementInput minus = input;
    plus.state = ApplyUnifiedInEkfErrorState(state, dx);
    minus.state = ApplyUnifiedInEkfErrorState(state, -dx);
    const MeasurementLinearization model_plus =
        BuildGnssPositionMeasurement(plus);
    const MeasurementLinearization model_minus =
        BuildGnssPositionMeasurement(minus);
    numeric.col(j) = -(model_plus.y - model_minus.y) / (2.0 * eps(j));
  }

  const Matrix3d H_pos_analytic =
      analytic.H.block<3, 3>(0, StateIdx::kPos);
  const Matrix3d H_pos_numeric =
      numeric.block<3, 3>(0, StateIdx::kPos);
  const Matrix3d H_att_analytic =
      analytic.H.block<3, 3>(0, StateIdx::kAtt);
  const Matrix3d H_att_numeric =
      numeric.block<3, 3>(0, StateIdx::kAtt);
  const Matrix3d H_lever_analytic =
      analytic.H.block<3, 3>(0, StateIdx::kGnssLever);
  const Matrix3d H_lever_numeric =
      numeric.block<3, 3>(0, StateIdx::kGnssLever);

  Expect((H_pos_analytic - H_pos_numeric).cwiseAbs().maxCoeff() <= 1.0e-6,
         "true-InEKF GNSS_POS position Jacobian should match finite differences");
  Expect((H_att_analytic - H_att_numeric).cwiseAbs().maxCoeff() <= 1.0e-6,
         "true-InEKF GNSS_POS attitude Jacobian should match finite differences even when d_body != lever");
  Expect((H_lever_analytic - H_lever_numeric).cwiseAbs().maxCoeff() <= 1.0e-6,
         "true-InEKF GNSS_POS GNSS-lever Jacobian should match finite differences");
}

void TestFilterResetSourceHasNoDeadInEkfBranches() {
  const string source = ReadTextFile("src/navigation/filter_reset.cpp");
  Expect(source.find("else if (use_inekf)") == string::npos,
         "filter_reset.cpp should not keep an unreachable else-if(use_inekf) branch");
  Expect(source.find("QuatFromSmallAngle(dphi_ecef)") == string::npos,
         "filter_reset.cpp should not keep the dead positive-dphi ESKF attitude injection branch");
}

void TestGnssMeasurementSourceHasNoDeadInEkfBranches() {
  const string source =
      ReadTextFile("src/navigation/measurement_model_gnss.cpp");
  Expect(source.find(
             "const bool use_local_position = input.context.ri_gnss_pos_use_p_ned_local;") ==
             string::npos,
         "measurement_model_gnss.cpp should not keep the dead GNSS_POS ri_gnss_pos_use_p_ned_local branch");
  Expect(CountOccurrences(
             source,
             "model.H.block<3, 3>(0, StateIdx::kAtt) = H_phi;") == 1,
         "measurement_model_gnss.cpp should assign GNSS_VEL H_phi exactly once");
}

void TestInEkfSemanticBranches() {
  InEkfConfig disabled;
  disabled.enabled = false;
  const FilterSemantics standard =
      BuildFilterSemanticsFromInEkfConfig(disabled);
  Expect(standard.flavor == FilterFlavor::kStandardEskf,
         "disabled InEKF config should map to standard ESKF");

  InEkfConfig inekf;
  inekf.enabled = true;
  const FilterSemantics inekf_semantics =
      BuildFilterSemanticsFromInEkfConfig(inekf);
  Expect(inekf_semantics.flavor == FilterFlavor::kInEkf,
         "enabled InEKF config should map to InEKF semantics");
}

void TestStateBlockCatalogMatchesLegacyStateIndices() {
  auto blocks = BuildDefaultStateBlockCatalog();
  Expect(blocks.size() == 13,
         "state block catalog size drifted from expected default entries");

  const auto expect_block = [&](const string &name, int expected_start,
                                int expected_size) {
    const auto it = blocks.find(name);
    Expect(it != blocks.end(), "missing state block: " + name);
    Expect(it->second.start == expected_start,
           name + " block index drifted");
    Expect(it->second.size == expected_size,
           name + " block size drifted");
  };

  expect_block("pos", StateIdx::kPos, 3);
  expect_block("vel", StateIdx::kVel, 3);
  expect_block("att", StateIdx::kAtt, 3);
  expect_block("ba", StateIdx::kBa, 3);
  expect_block("bg", StateIdx::kBg, 3);
  expect_block("sg", StateIdx::kSg, 3);
  expect_block("sa", StateIdx::kSa, 3);
  expect_block("odo_scale", StateIdx::kOdoScale, 1);
  expect_block("mounting_roll", StateIdx::kMountRoll, 1);
  expect_block("mounting_pitch", StateIdx::kMountPitch, 1);
  expect_block("mounting_yaw", StateIdx::kMountYaw, 1);
  expect_block("lever", StateIdx::kLever, 3);
  expect_block("gnss_lever", StateIdx::kGnssLever, 3);
}

void TestStandardEskfCorrectionInjectionUsesUnifiedImuErrorConvention() {
  NoiseParams noise;
  noise.sigma_acc = 0.05;
  noise.sigma_gyro = 0.005;
  noise.sigma_ba = 1.0e-6;
  noise.sigma_bg = 1.0e-6;
  noise.sigma_sg = 1.0e-6;
  noise.sigma_sa = 1.0e-6;

  EskfEngine engine(noise);
  State initial;
  initial.p = LlhToEcef(30.0 * EIGEN_PI / 180.0, 114.0 * EIGEN_PI / 180.0, 20.0);
  initial.ba = Vector3d(0.01, -0.02, 0.03);
  initial.bg = Vector3d(-0.04, 0.05, -0.06);
  initial.sg = Vector3d(70.0, -80.0, 90.0) * 1.0e-6;
  initial.sa = Vector3d(-100.0, 110.0, -120.0) * 1.0e-6;

  Matrix<double, kStateDim, kStateDim> P0 =
      Matrix<double, kStateDim, kStateDim>::Identity() * 100.0;
  engine.Initialize(initial, P0);

  MatrixXd H = MatrixXd::Identity(kStateDim, kStateDim);
  MatrixXd R = MatrixXd::Identity(kStateDim, kStateDim) * 1.0e-12;
  VectorXd y = VectorXd::Zero(kStateDim);
  y.segment<3>(StateIdx::kBa) = Vector3d(1.2e-3, -2.1e-3, 0.7e-3);
  y.segment<3>(StateIdx::kBg) = Vector3d(-0.9e-4, 1.7e-4, -2.5e-4);
  y.segment<3>(StateIdx::kSg) = Vector3d(4.0, -5.0, 6.0) * 1.0e-6;
  y.segment<3>(StateIdx::kSa) = Vector3d(-7.0, 8.0, -9.0) * 1.0e-6;

  VectorXd dx = VectorXd::Zero(kStateDim);
  bool updated = engine.Correct(y, H, R, &dx, nullptr);
  Expect(updated,
         "Correct should succeed for IMU error injection convention check");

  const State expected = ApplyStandardEskfErrorState(initial, dx);
  const State &actual = engine.state();
  Expect((actual.ba - expected.ba).norm() <= 1.0e-12,
         "accel-bias injection should follow the standard ESKF additive error convention");
  Expect((actual.bg - expected.bg).norm() <= 1.0e-12,
         "gyro-bias injection should follow the same additive error convention as ba/sg/sa");
  Expect((actual.sg - expected.sg).norm() <= 1.0e-12,
         "gyro-scale injection should follow the standard ESKF additive error convention");
  Expect((actual.sa - expected.sa).norm() <= 1.0e-12,
         "accel-scale injection should follow the standard ESKF additive error convention");
}

void TestNavigationFilterEngineUsesAdditiveImuErrorInjection() {
  NoiseParams noise;
  noise.sigma_acc = 0.05;
  noise.sigma_gyro = 0.005;
  noise.sigma_ba = 1.0e-6;
  noise.sigma_bg = 1.0e-6;
  noise.sigma_sg = 1.0e-6;
  noise.sigma_sa = 1.0e-6;

  NavigationFilterEngine engine(noise, BuildStandardEskfSemantics());
  State initial;
  initial.p = LlhToEcef(30.0 * EIGEN_PI / 180.0, 114.0 * EIGEN_PI / 180.0, 20.0);
  initial.ba = Vector3d(0.01, -0.02, 0.03);
  initial.bg = Vector3d(-0.04, 0.05, -0.06);
  initial.sg = Vector3d(70.0, -80.0, 90.0) * 1.0e-6;
  initial.sa = Vector3d(-100.0, 110.0, -120.0) * 1.0e-6;

  Matrix<double, kStateDim, kStateDim> P0 =
      Matrix<double, kStateDim, kStateDim>::Identity() * 100.0;
  engine.Initialize(initial, P0);

  MeasurementLinearization meas;
  meas.y = VectorXd::Zero(kStateDim);
  meas.H = MatrixXd::Identity(kStateDim, kStateDim);
  meas.R = MatrixXd::Identity(kStateDim, kStateDim) * 1.0e-12;
  meas.model_name = "IDENTITY_CORE";
  meas.frame_tag = "regression";
  meas.y.segment<3>(StateIdx::kBa) = Vector3d(1.2e-3, -2.1e-3, 0.7e-3);
  meas.y.segment<3>(StateIdx::kBg) = Vector3d(-0.9e-4, 1.7e-4, -2.5e-4);
  meas.y.segment<3>(StateIdx::kSg) = Vector3d(4.0, -5.0, 6.0) * 1.0e-6;
  meas.y.segment<3>(StateIdx::kSa) = Vector3d(-7.0, 8.0, -9.0) * 1.0e-6;

  VectorXd dx = VectorXd::Zero(kStateDim);
  const bool updated = engine.Correct(meas, &dx);
  Expect(updated,
         "NavigationFilterEngine correction should succeed for IMU additive injection regression");

  const State expected = ApplyStandardEskfErrorState(initial, dx);
  const State &actual = engine.state();
  Expect((actual.ba - expected.ba).norm() <= 1.0e-12,
         "NavigationFilterEngine accel-bias injection should remain additive");
  Expect((actual.bg - expected.bg).norm() <= 1.0e-12,
         "NavigationFilterEngine gyro-bias injection should remain additive");
  Expect((actual.sg - expected.sg).norm() <= 1.0e-12,
         "NavigationFilterEngine gyro-scale injection should remain additive");
  Expect((actual.sa - expected.sa).norm() <= 1.0e-12,
         "NavigationFilterEngine accel-scale injection should remain additive");
}

void TestNavigationFilterEngineUsesHistoricalStandardEskfAttitudeInjection() {
  NoiseParams noise;
  noise.sigma_acc = 0.05;
  noise.sigma_gyro = 0.005;
  noise.sigma_ba = 1.0e-6;
  noise.sigma_bg = 1.0e-6;
  noise.sigma_sg = 1.0e-6;
  noise.sigma_sa = 1.0e-6;

  NavigationFilterEngine engine(noise, BuildStandardEskfSemantics());
  State initial = BuildMeasurementContractState();
  Matrix<double, kStateDim, kStateDim> P0 =
      Matrix<double, kStateDim, kStateDim>::Identity() * 20.0;
  engine.Initialize(initial, P0);

  MeasurementLinearization meas;
  meas.y = VectorXd::Zero(3);
  meas.H = MatrixXd::Zero(3, kStateDim);
  meas.R = MatrixXd::Identity(3, 3) * 1.0e-12;
  meas.model_name = "STANDARD_RIGHT_ERROR_ATT";
  meas.frame_tag = "regression";
  meas.H.block<3, 3>(0, StateIdx::kAtt) = Matrix3d::Identity();
  meas.y = Vector3d(3.0e-4, -2.0e-4, 1.0e-4);

  VectorXd dx = VectorXd::Zero(kStateDim);
  const bool updated = engine.Correct(meas, &dx);
  Expect(updated,
         "NavigationFilterEngine attitude correction should succeed for historical-attitude regression");

  const State expected = ApplyStandardEskfErrorState(initial, dx);
  Expect(std::abs(std::abs(engine.state().q.dot(expected.q)) - 1.0) <= 1.0e-12,
         "NavigationFilterEngine standard attitude injection must remain on the historical accepted-baseline contract");
}

void TestNavigationFilterEngineMatchesHistoricalStandardEskfAttitudeInjection() {
  NoiseParams noise;
  noise.sigma_acc = 0.05;
  noise.sigma_gyro = 0.005;
  noise.sigma_ba = 1.0e-6;
  noise.sigma_bg = 1.0e-6;
  noise.sigma_sg = 1.0e-6;
  noise.sigma_sa = 1.0e-6;

  NavigationFilterEngine engine(noise, BuildStandardEskfSemantics());
  State initial = BuildMeasurementContractState();
  Matrix<double, kStateDim, kStateDim> P0 =
      Matrix<double, kStateDim, kStateDim>::Identity() * 20.0;
  engine.Initialize(initial, P0);

  MeasurementLinearization meas;
  meas.y = VectorXd::Zero(3);
  meas.H = MatrixXd::Zero(3, kStateDim);
  meas.R = MatrixXd::Identity(3, 3) * 1.0e-12;
  meas.model_name = "STANDARD_HISTORICAL_ATT";
  meas.frame_tag = "regression";
  meas.H.block<3, 3>(0, StateIdx::kAtt) = Matrix3d::Identity();
  meas.y = Vector3d(3.0e-4, -2.0e-4, 1.0e-4);

  VectorXd dx = VectorXd::Zero(kStateDim);
  const bool updated = engine.Correct(meas, &dx);
  Expect(updated,
         "NavigationFilterEngine attitude correction should succeed for historical-contract regression");

  const State expected = ApplyHistoricalStandardEskfErrorState(initial, dx);
  Expect(std::abs(std::abs(engine.state().q.dot(expected.q)) - 1.0) <= 1.0e-12,
         "NavigationFilterEngine standard attitude injection drifted from the historical left-multiply contract");
}

void TestNavigationFilterEngineCorrectionMatchesEskfCompatibilityWrapper() {
  NoiseParams noise;
  noise.sigma_acc = 0.05;
  noise.sigma_gyro = 0.005;
  noise.sigma_ba = 1.0e-6;
  noise.sigma_bg = 1.0e-6;
  noise.sigma_sg = 1.0e-6;
  noise.sigma_sa = 1.0e-6;

  State initial = BuildMeasurementContractState();
  Matrix<double, kStateDim, kStateDim> P0 =
      Matrix<double, kStateDim, kStateDim>::Identity() * 10.0;
  P0(StateIdx::kBg + 2, StateIdx::kBg + 2) = 25.0;

  EskfEngine legacy(noise);
  legacy.Initialize(initial, P0);

  NavigationFilterEngine extracted(noise, BuildStandardEskfSemantics());
  extracted.Initialize(initial, P0);

  MeasurementLinearization meas;
  meas.y = VectorXd::Zero(6);
  meas.H = MatrixXd::Zero(6, kStateDim);
  meas.R = MatrixXd::Identity(6, 6) * 1.0e-4;
  meas.model_name = "RESET_ALIGNMENT";
  meas.frame_tag = "regression";
  meas.H.block<3, 3>(0, StateIdx::kAtt) = Matrix3d::Identity();
  meas.H.block<3, 3>(3, StateIdx::kBg) = Matrix3d::Identity();
  meas.y.segment<3>(0) = Vector3d(2.0e-4, -3.0e-4, 1.0e-4);
  meas.y.segment<3>(3) = Vector3d(-4.0e-5, 5.0e-5, -6.0e-5);

  VectorXd dx_legacy = VectorXd::Zero(kStateDim);
  const bool legacy_updated =
      legacy.Correct(meas.y, meas.H, meas.R, &dx_legacy, nullptr);
  Expect(legacy_updated, "legacy EskfEngine correction should succeed");

  VectorXd dx_extracted = VectorXd::Zero(kStateDim);
  const bool extracted_updated = extracted.Correct(meas, &dx_extracted);
  Expect(extracted_updated, "NavigationFilterEngine correction should succeed");

  Expect((dx_extracted - dx_legacy).norm() <= 1.0e-12,
         "NavigationFilterEngine dx should match the compatibility wrapper");
  Expect((extracted.state().p - legacy.state().p).norm() <= 1.0e-12,
         "NavigationFilterEngine position state drifted from legacy wrapper");
  Expect((extracted.state().v - legacy.state().v).norm() <= 1.0e-12,
         "NavigationFilterEngine velocity state drifted from legacy wrapper");
  Expect(std::abs(std::abs(extracted.state().q.dot(legacy.state().q)) - 1.0) <= 1.0e-12,
         "NavigationFilterEngine attitude state drifted from legacy wrapper");
  Expect((extracted.state().bg - legacy.state().bg).norm() <= 1.0e-12,
         "NavigationFilterEngine IMU state drifted from legacy wrapper");
  Expect((extracted.cov() - legacy.cov()).norm() <= 1.0e-12,
         "NavigationFilterEngine covariance drifted from legacy wrapper");
}

void TestNavigationFilterEngineTrueInEkfResetMatchesEskfCompatibilityWrapper() {
  NoiseParams noise;
  noise.sigma_acc = 0.05;
  noise.sigma_gyro = 0.005;
  noise.sigma_ba = 1.0e-6;
  noise.sigma_bg = 1.0e-6;
  noise.sigma_sg = 1.0e-6;
  noise.sigma_sa = 1.0e-6;

  State initial = BuildMeasurementContractState();
  Matrix<double, kStateDim, kStateDim> P0 =
      Matrix<double, kStateDim, kStateDim>::Identity() * 3.0;
  P0(StateIdx::kPos + 0, StateIdx::kAtt + 2) = 0.12;
  P0(StateIdx::kAtt + 2, StateIdx::kPos + 0) = 0.12;
  P0(StateIdx::kVel + 1, StateIdx::kAtt + 0) = -0.09;
  P0(StateIdx::kAtt + 0, StateIdx::kVel + 1) = -0.09;

  InEkfManager legacy_inekf;
  legacy_inekf.Enable(true);

  InEkfManager extracted_inekf;
  extracted_inekf.Enable(true);

  EskfEngine legacy(noise);
  legacy.SetInEkfManager(&legacy_inekf);
  legacy.Initialize(initial, P0);

  NavigationFilterEngine extracted(noise, BuildInEkfSemantics());
  extracted.SetInEkfManager(&extracted_inekf);
  extracted.Initialize(initial, P0);

  MeasurementLinearization meas;
  meas.y = VectorXd::Zero(9);
  meas.H = MatrixXd::Zero(9, kStateDim);
  meas.R = MatrixXd::Identity(9, 9) * 1.0e-4;
  meas.model_name = "INEKF_RESET_ALIGNMENT";
  meas.frame_tag = "regression";
  meas.H.block<3, 3>(0, StateIdx::kPos) = Matrix3d::Identity();
  meas.H.block<3, 3>(3, StateIdx::kVel) = Matrix3d::Identity();
  meas.H.block<3, 3>(6, StateIdx::kAtt) = Matrix3d::Identity();
  meas.y.segment<3>(0) = Vector3d(5.0e-3, -4.0e-3, 3.0e-3);
  meas.y.segment<3>(3) = Vector3d(-2.0e-3, 1.5e-3, -1.0e-3);
  meas.y.segment<3>(6) = Vector3d(8.0e-4, -6.0e-4, 4.0e-4);

  VectorXd dx_legacy = VectorXd::Zero(kStateDim);
  const bool legacy_updated =
      legacy.Correct(meas.y, meas.H, meas.R, &dx_legacy, nullptr);
  Expect(legacy_updated, "legacy InEKF correction should succeed");

  VectorXd dx_extracted = VectorXd::Zero(kStateDim);
  const bool extracted_updated = extracted.Correct(meas, &dx_extracted);
  Expect(extracted_updated,
         "NavigationFilterEngine InEKF correction should succeed");

  const auto &legacy_reset = legacy.last_inekf_correction();
  const auto &extracted_reset = extracted.last_inekf_correction();
  Expect(legacy_reset.valid,
         "legacy wrapper should expose a valid InEKF reset snapshot");
  Expect(extracted_reset.valid,
         "NavigationFilterEngine should expose a valid InEKF reset snapshot");
  Expect((dx_extracted - dx_legacy).norm() <= 1.0e-12,
         "NavigationFilterEngine InEKF dx drifted from legacy wrapper");
  Expect((extracted.state().p - legacy.state().p).norm() <= 1.0e-12,
         "NavigationFilterEngine InEKF position state drifted from legacy wrapper");
  Expect((extracted.state().v - legacy.state().v).norm() <= 1.0e-12,
         "NavigationFilterEngine InEKF velocity state drifted from legacy wrapper");
  Expect(std::abs(std::abs(extracted.state().q.dot(legacy.state().q)) - 1.0) <= 1.0e-12,
         "NavigationFilterEngine InEKF attitude state drifted from legacy wrapper");
  Expect((extracted.cov() - legacy.cov()).norm() <= 1.0e-12,
         "NavigationFilterEngine InEKF covariance drifted from legacy wrapper");
  Expect((extracted_reset.P_after_reset - legacy_reset.P_after_reset).norm() <= 1.0e-12,
         "NavigationFilterEngine InEKF reset covariance drifted from legacy wrapper");
  Expect((extracted_reset.P_after_all - legacy_reset.P_after_all).norm() <= 1.0e-12,
         "NavigationFilterEngine InEKF final covariance drifted from legacy wrapper");
}

void TestNavigationFilterEngineStandardResetMatchesEskfCompatibilityWrapper() {
  NoiseParams noise;
  noise.sigma_acc = 0.05;
  noise.sigma_gyro = 0.005;
  noise.sigma_ba = 1.0e-6;
  noise.sigma_bg = 1.0e-6;
  noise.sigma_sg = 1.0e-6;
  noise.sigma_sa = 1.0e-6;

  State initial = BuildMeasurementContractState();
  Matrix<double, kStateDim, kStateDim> P0 =
      Matrix<double, kStateDim, kStateDim>::Identity() * 4.0;
  P0(StateIdx::kAtt + 0, StateIdx::kAtt + 1) = 0.08;
  P0(StateIdx::kAtt + 1, StateIdx::kAtt + 0) = 0.08;
  P0(StateIdx::kAtt + 2, StateIdx::kBg + 2) = -0.11;
  P0(StateIdx::kBg + 2, StateIdx::kAtt + 2) = -0.11;

  InEkfManager legacy_inekf;
  legacy_inekf.debug_enable_standard_reset_gamma = true;
  InEkfManager extracted_inekf;
  extracted_inekf.debug_enable_standard_reset_gamma = true;

  EskfEngine legacy(noise);
  legacy.SetInEkfManager(&legacy_inekf);
  legacy.Initialize(initial, P0);

  NavigationFilterEngine extracted(noise, BuildStandardEskfSemantics());
  extracted.SetInEkfManager(&extracted_inekf);
  extracted.Initialize(initial, P0);

  MeasurementLinearization meas;
  meas.y = VectorXd::Zero(6);
  meas.H = MatrixXd::Zero(6, kStateDim);
  meas.R = MatrixXd::Identity(6, 6) * 1.0e-4;
  meas.model_name = "STANDARD_RESET_ALIGNMENT";
  meas.frame_tag = "regression";
  meas.H.block<3, 3>(0, StateIdx::kAtt) = Matrix3d::Identity();
  meas.H.block<3, 3>(3, StateIdx::kBg) = Matrix3d::Identity();
  meas.y.segment<3>(0) = Vector3d(7.0e-4, -5.0e-4, 3.0e-4);
  meas.y.segment<3>(3) = Vector3d(-3.0e-5, 2.0e-5, -4.0e-5);

  VectorXd dx_legacy = VectorXd::Zero(kStateDim);
  const bool legacy_updated =
      legacy.Correct(meas.y, meas.H, meas.R, &dx_legacy, nullptr);
  Expect(legacy_updated, "legacy standard-reset correction should succeed");

  VectorXd dx_extracted = VectorXd::Zero(kStateDim);
  const bool extracted_updated = extracted.Correct(meas, &dx_extracted);
  Expect(extracted_updated,
         "NavigationFilterEngine standard-reset correction should succeed");

  Expect((dx_extracted - dx_legacy).norm() <= 1.0e-12,
         "NavigationFilterEngine standard-reset dx drifted from legacy wrapper");
  Expect((extracted.state().p - legacy.state().p).norm() <= 1.0e-12,
         "NavigationFilterEngine standard-reset position state drifted from legacy wrapper");
  Expect((extracted.state().v - legacy.state().v).norm() <= 1.0e-12,
         "NavigationFilterEngine standard-reset velocity state drifted from legacy wrapper");
  Expect(std::abs(std::abs(extracted.state().q.dot(legacy.state().q)) - 1.0) <= 1.0e-12,
         "NavigationFilterEngine standard-reset attitude state drifted from legacy wrapper");
  Expect((extracted.cov() - legacy.cov()).norm() <= 1.0e-12,
         "NavigationFilterEngine standard-reset covariance drifted from legacy wrapper");
}

void TestNavigationFilterEngineOwnsPredictCovarianceAndDebugSnapshot() {
  NoiseParams noise;
  noise.sigma_acc = 0.05;
  noise.sigma_gyro = 0.005;
  noise.sigma_ba = 1.0e-4;
  noise.sigma_bg = 1.0e-4;
  noise.sigma_sg = 1.0e-4;
  noise.sigma_sa = 1.0e-4;
  noise.markov_corr_time = 60.0;

  State initial = BuildMeasurementContractState();
  Matrix<double, kStateDim, kStateDim> P0 =
      Matrix<double, kStateDim, kStateDim>::Identity() * 2.0;

  ImuData imu_prev;
  imu_prev.t = 1.0;
  imu_prev.dt = 0.0;
  imu_prev.dtheta = Vector3d::Zero();
  imu_prev.dvel = Vector3d::Zero();

  ImuData imu_curr;
  imu_curr.t = 1.01;
  imu_curr.dt = 0.01;
  imu_curr.dtheta = Vector3d(1.0e-4, -2.0e-4, 1.5e-4);
  imu_curr.dvel = Vector3d(0.01, -0.015, 0.02);

  EskfEngine legacy(noise);
  legacy.Initialize(initial, P0);
  legacy.AddImu(imu_prev);
  legacy.AddImu(imu_curr);
  Expect(legacy.Predict(),
         "legacy EskfEngine predict should succeed for predict-ownership regression");

  ProcessModelInput input;
  input.nominal = initial;
  input.imu_prev = imu_prev;
  input.imu_curr = imu_curr;
  input.noise = noise;
  input.semantics = BuildStandardEskfSemantics();
  const ProcessLinearization process = BuildProcessLinearization(input);

  NavigationFilterEngine extracted(noise, BuildStandardEskfSemantics());
  extracted.Initialize(initial, P0);
  Expect(extracted.Predict(process, imu_prev.t, imu_curr.t),
         "NavigationFilterEngine direct predict should succeed");

  const auto &legacy_snap = legacy.last_predict_debug();
  const auto &extracted_snap = extracted.last_predict_debug();
  Expect(legacy_snap.valid, "legacy predict snapshot should be valid");
  Expect(extracted_snap.valid, "NavigationFilterEngine predict snapshot should be valid");
  Expect((extracted.state().p - legacy.state().p).norm() <= 1.0e-12,
         "NavigationFilterEngine predict position state drifted from legacy wrapper");
  Expect((extracted.state().v - legacy.state().v).norm() <= 1.0e-12,
         "NavigationFilterEngine predict velocity state drifted from legacy wrapper");
  Expect(std::abs(std::abs(extracted.state().q.dot(legacy.state().q)) - 1.0) <= 1.0e-12,
         "NavigationFilterEngine predict attitude state drifted from legacy wrapper");
  Expect((extracted.cov() - legacy.cov()).norm() <= 1.0e-12,
         "NavigationFilterEngine predict covariance drifted from legacy wrapper");
  Expect((extracted_snap.P_after_raw_common - legacy_snap.P_after_raw_common).norm() <= 1.0e-12,
         "NavigationFilterEngine predict raw covariance snapshot drifted from legacy wrapper");
  Expect((extracted_snap.P_after_final_common - legacy_snap.P_after_final_common).norm() <= 1.0e-12,
         "NavigationFilterEngine predict final covariance snapshot drifted from legacy wrapper");
}

void TestStandardEskfErrorStateRoundTrip() {
  State nominal;
  nominal.p = LlhToEcef(32.0 * EIGEN_PI / 180.0, 118.0 * EIGEN_PI / 180.0, 25.0);
  nominal.v = Vector3d(3.0, -2.0, 0.5);
  nominal.q = NormalizeQuat(RpyToQuat(Vector3d(0.08, -0.04, 0.12)));
  nominal.ba = Vector3d(0.01, -0.02, 0.03);
  nominal.bg = Vector3d(-2.0e-4, 1.0e-4, -1.5e-4);
  nominal.sg = Vector3d(40.0, -30.0, 20.0) * 1.0e-6;
  nominal.sa = Vector3d(-50.0, 60.0, -70.0) * 1.0e-6;
  nominal.odo_scale = 1.002;
  nominal.mounting_roll = 0.5 * EIGEN_PI / 180.0;
  nominal.mounting_pitch = -0.3 * EIGEN_PI / 180.0;
  nominal.mounting_yaw = 0.8 * EIGEN_PI / 180.0;
  nominal.lever_arm = Vector3d(0.4, -0.2, 0.1);
  nominal.gnss_lever_arm = Vector3d(1.1, 0.3, -0.15);

  VectorXd dx = VectorXd::Zero(kStateDim);
  dx.segment<3>(StateIdx::kPos) = Vector3d(0.02, -0.01, 0.03);
  dx.segment<3>(StateIdx::kVel) = Vector3d(2.0e-3, -1.5e-3, 1.0e-3);
  dx.segment<3>(StateIdx::kAtt) = Vector3d(3.0e-4, -2.0e-4, 1.5e-4);
  dx.segment<3>(StateIdx::kBa) = Vector3d(1.0e-4, -2.0e-4, 1.5e-4);
  dx.segment<3>(StateIdx::kBg) = Vector3d(-3.0e-6, 2.0e-6, -1.0e-6);
  dx.segment<3>(StateIdx::kSg) = Vector3d(2.0, -3.0, 1.0) * 1.0e-6;
  dx.segment<3>(StateIdx::kSa) = Vector3d(-4.0, 5.0, -2.0) * 1.0e-6;
  dx(StateIdx::kOdoScale) = 3.0e-4;
  dx(StateIdx::kMountRoll) = 5.0e-5;
  dx(StateIdx::kMountPitch) = -4.0e-5;
  dx(StateIdx::kMountYaw) = 6.0e-5;
  dx.segment<3>(StateIdx::kLever) = Vector3d(1.0e-3, -2.0e-3, 1.5e-3);
  dx.segment<3>(StateIdx::kGnssLever) = Vector3d(-1.5e-3, 1.0e-3, -0.5e-3);

  const State perturbed = ApplyStandardEskfErrorState(nominal, dx);
  const VectorXd recovered = ComputeStandardEskfErrorState(nominal, perturbed);
  const VectorXd diff = recovered - dx;
  Eigen::Index worst_idx = 0;
  const double max_abs = diff.cwiseAbs().maxCoeff(&worst_idx);
  Expect(max_abs <= 1.0e-8,
         "standard ESKF error-state round-trip helper should preserve sign conventions"
         " (worst_idx=" + to_string(static_cast<int>(worst_idx)) +
         ", recovered=" + to_string(recovered(static_cast<int>(worst_idx))) +
         ", expected=" + to_string(dx(static_cast<int>(worst_idx))) + ")");
}

void TestStandardEskfProcessModelImuCouplingMatchesFiniteDifference() {
  NoiseParams noise;
  noise.sigma_acc = 0.05;
  noise.sigma_gyro = 0.005;
  noise.sigma_ba = 1.0e-6;
  noise.sigma_bg = 1.0e-6;
  noise.sigma_sg = 1.0e-6;
  noise.sigma_sa = 1.0e-6;
  noise.markov_corr_time = 3600.0;

  State state;
  state.p = LlhToEcef(31.0 * EIGEN_PI / 180.0, 114.0 * EIGEN_PI / 180.0, 15.0);
  state.v = Vector3d(6.0, -1.5, 0.8);
  state.q = NormalizeQuat(RpyToQuat(Vector3d(0.05, -0.03, 0.1)));
  state.ba = Vector3d(0.015, -0.008, 0.02);
  state.bg = Vector3d(-1.0e-4, 2.0e-4, -1.5e-4);
  state.sg = Vector3d(120.0, -80.0, 40.0) * 1.0e-6;
  state.sa = Vector3d(-150.0, 100.0, 60.0) * 1.0e-6;

  ImuData imu_prev;
  imu_prev.t = 0.0;
  imu_prev.dt = 0.0;
  imu_prev.dtheta = Vector3d::Zero();
  imu_prev.dvel = Vector3d::Zero();

  ImuData imu_curr;
  imu_curr.t = 1.0e-4;
  imu_curr.dt = 1.0e-4;
  imu_curr.dtheta = Vector3d(2.0e-6, -1.0e-6, 3.0e-6);
  imu_curr.dvel = Vector3d(5.0e-4, -2.0e-4, 1.0e-3);

  Matrix<double, kStateDim, kStateDim> Phi =
      Matrix<double, kStateDim, kStateDim>::Zero();
  BuildStandardEskfAnalyticPhi(state, imu_prev, imu_curr, noise, Phi);

  const State nominal_next = InsMech::Propagate(state, imu_prev, imu_curr).state;
  Matrix<double, kStateDim, kStateDim> Phi_num =
      Matrix<double, kStateDim, kStateDim>::Zero();

  VectorXd eps = VectorXd::Zero(kStateDim);
  eps.segment<3>(StateIdx::kBa).setConstant(1.0e-5);
  eps.segment<3>(StateIdx::kBg).setConstant(1.0e-6);
  eps.segment<3>(StateIdx::kSg).setConstant(1.0e-6);
  eps.segment<3>(StateIdx::kSa).setConstant(1.0e-6);

  for (int j = StateIdx::kBa; j < StateIdx::kSa + 3; ++j) {
    VectorXd dx = VectorXd::Zero(kStateDim);
    dx(j) = eps(j);
    const State plus0 = ApplyStandardEskfErrorState(state, dx);
    const State minus0 = ApplyStandardEskfErrorState(state, -dx);
    const State plus1 = InsMech::Propagate(plus0, imu_prev, imu_curr).state;
    const State minus1 = InsMech::Propagate(minus0, imu_prev, imu_curr).state;
    const VectorXd dx_plus = ComputeStandardEskfErrorState(nominal_next, plus1);
    const VectorXd dx_minus = ComputeStandardEskfErrorState(nominal_next, minus1);
    Phi_num.col(j) = (dx_plus - dx_minus) / (2.0 * eps(j));
  }

  const double dt = imu_curr.dt;
  const Matrix3d F_vba = Phi.block<3, 3>(StateIdx::kVel, StateIdx::kBa) / dt;
  const Matrix3d F_vsa = Phi.block<3, 3>(StateIdx::kVel, StateIdx::kSa) / dt;
  const Matrix3d F_phibg = Phi.block<3, 3>(StateIdx::kAtt, StateIdx::kBg) / dt;
  const Matrix3d F_phisg = Phi.block<3, 3>(StateIdx::kAtt, StateIdx::kSg) / dt;

  const Matrix3d F_num_vba =
      Phi_num.block<3, 3>(StateIdx::kVel, StateIdx::kBa) / dt;
  const Matrix3d F_num_vsa =
      Phi_num.block<3, 3>(StateIdx::kVel, StateIdx::kSa) / dt;
  const Matrix3d F_num_phibg =
      Phi_num.block<3, 3>(StateIdx::kAtt, StateIdx::kBg) / dt;
  const Matrix3d F_num_phisg =
      Phi_num.block<3, 3>(StateIdx::kAtt, StateIdx::kSg) / dt;

  auto expect_block_close = [&](const Matrix3d &analytic, const Matrix3d &numeric,
                                const string &tag, double max_abs_tol) {
    const double max_abs = (analytic - numeric).cwiseAbs().maxCoeff();
    Expect(max_abs <= max_abs_tol,
           tag + " should match finite-difference propagation under the solver's standard ESKF convention"
           " (max_abs=" + to_string(max_abs) +
           ", analytic_00=" + to_string(analytic(0, 0)) +
           ", numeric_00=" + to_string(numeric(0, 0)) + ")");
  };

  expect_block_close(F_vba, F_num_vba, "F_vba", 5.0e-6);
  expect_block_close(F_vsa, F_num_vsa, "F_vsa", 5.0e-6);
  expect_block_close(F_phibg, F_num_phibg, "F_phibg", 1.2e-1);
  expect_block_close(F_phisg, F_num_phisg, "F_phisg", 1.2e-1);
}

void TestStandardEskfProcessBuilderMatchesFiniteDifference() {
  const ProcessModelInput input = BuildStandardEskfProcessModelInputForRegression();
  const ProcessLinearization analytic =
      BuildStandardEskfProcessLinearization(input);
  const ProcessLinearization numeric =
      BuildFiniteDifferenceProcessLinearization(input);

  const Matrix3d F_vba = analytic.F.block<3, 3>(StateIdx::kVel, StateIdx::kBa);
  const Matrix3d F_vsa = analytic.F.block<3, 3>(StateIdx::kVel, StateIdx::kSa);
  const Matrix3d F_phibg =
      analytic.F.block<3, 3>(StateIdx::kAtt, StateIdx::kBg);
  const Matrix3d F_phisg =
      analytic.F.block<3, 3>(StateIdx::kAtt, StateIdx::kSg);

  const Matrix3d F_num_vba =
      numeric.F.block<3, 3>(StateIdx::kVel, StateIdx::kBa);
  const Matrix3d F_num_vsa =
      numeric.F.block<3, 3>(StateIdx::kVel, StateIdx::kSa);
  const Matrix3d F_num_phibg =
      numeric.F.block<3, 3>(StateIdx::kAtt, StateIdx::kBg);
  const Matrix3d F_num_phisg =
      numeric.F.block<3, 3>(StateIdx::kAtt, StateIdx::kSg);

  auto expect_block_close = [&](const Matrix3d &analytic_block,
                                const Matrix3d &numeric_block,
                                const string &tag,
                                double max_abs_tol) {
    const double max_abs = (analytic_block - numeric_block).cwiseAbs().maxCoeff();
    Expect(max_abs <= max_abs_tol,
           tag + " drifted from finite-difference reference"
           " (max_abs=" + to_string(max_abs) +
           ", analytic_00=" + to_string(analytic_block(0, 0)) +
           ", numeric_00=" + to_string(numeric_block(0, 0)) + ")");
  };

  expect_block_close(F_vba, F_num_vba, "F_v,ba", 5.0e-6);
  expect_block_close(F_vsa, F_num_vsa, "F_v,sa", 5.0e-6);
  expect_block_close(F_phibg, F_num_phibg, "F_phi,bg", 1.2e-1);
  expect_block_close(F_phisg, F_num_phisg, "F_phi,sg", 1.2e-1);
}

void TestInEkfProcessSemanticBridgeHonorsDebugForceOverrides() {
  InEkfConfig standard_forced;
  standard_forced.enabled = true;
  standard_forced.debug_force_process_model = "eskf";
  const FilterSemantics standard =
      BuildProcessSemanticsFromInEkfConfig(standard_forced);
  Expect(standard.flavor == FilterFlavor::kStandardEskf,
         "process semantic bridge should honor debug_force_process_model=eskf");

  InEkfConfig inekf_forced;
  inekf_forced.enabled = false;
  inekf_forced.debug_force_process_model = "inekf";
  const FilterSemantics forced_inekf =
      BuildProcessSemanticsFromInEkfConfig(inekf_forced);
  Expect(forced_inekf.flavor == FilterFlavor::kInEkf,
         "process semantic bridge should honor debug_force_process_model=inekf");

  InEkfConfig inekf_enabled;
  inekf_enabled.enabled = true;
  const FilterSemantics inekf_semantics =
      BuildProcessSemanticsFromInEkfConfig(inekf_enabled);
  Expect(inekf_semantics.flavor == FilterFlavor::kInEkf,
         "process semantic bridge should map enabled InEKF configs onto the unified InEKF path");
}

void TestInsMechProcessWrapperMatchesBuilderDispatch() {
  struct WrapperCase {
    string tag;
    InEkfConfig inekf;
  };

  const vector<WrapperCase> cases = {
      {"standard_default", InEkfConfig{}},
      {"enabled_default", [] {
         InEkfConfig cfg;
         cfg.enabled = true;
         cfg.ri_vel_gyro_noise_mode = -1;
         return cfg;
       }()},
      {"enabled_positive_gyro_noise", [] {
         InEkfConfig cfg;
         cfg.enabled = true;
         cfg.ri_vel_gyro_noise_mode = 1;
         return cfg;
       }()},
      {"forced_eskf", [] {
         InEkfConfig cfg;
         cfg.enabled = true;
         cfg.debug_force_process_model = "eskf";
         cfg.ri_vel_gyro_noise_mode = 1;
         return cfg;
       }()},
      {"forced_inekf", [] {
         InEkfConfig cfg;
         cfg.enabled = false;
         cfg.debug_force_process_model = "inekf";
         cfg.ri_vel_gyro_noise_mode = 1;
         return cfg;
       }()},
  };

  const double dt = 0.01;
  const Matrix3d C_bn = AngleAxisd(0.2, Vector3d::UnitZ()).toRotationMatrix() *
                        AngleAxisd(-0.1, Vector3d::UnitY()).toRotationMatrix();
  const Vector3d sf_a(1.0 / 1.11, 1.0 / 1.23, 1.0 / 1.34);
  const Vector3d sf_g(1.0 / 1.07, 1.0 / 1.18, 1.0 / 1.29);
  const Vector3d f_unbiased(4.0, -3.0, 8.0);
  const Vector3d omega_unbiased(0.1, -0.2, 0.3);
  const Vector3d f_corrected = sf_a.cwiseProduct(f_unbiased);
  const Vector3d omega_corrected = sf_g.cwiseProduct(omega_unbiased);
  const Vector3d v_ned(12.0, -4.0, 0.5);
  const double lat = 30.0 * EIGEN_PI / 180.0;
  const double h = 20.0;

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
  noise.markov_corr_time = 3600.0;

  for (const WrapperCase &test_case : cases) {
    Matrix<double, kStateDim, kStateDim> wrapper_Phi =
        Matrix<double, kStateDim, kStateDim>::Zero();
    Matrix<double, kStateDim, kStateDim> wrapper_Qd =
        Matrix<double, kStateDim, kStateDim>::Zero();
    InsMech::BuildProcessModel(C_bn, f_corrected, omega_corrected, f_unbiased,
                               omega_unbiased, sf_a, sf_g, v_ned, lat, h, dt,
                               noise, wrapper_Phi, wrapper_Qd, &test_case.inekf);

    ProcessModelResolvedInput builder_input;
    builder_input.semantics = BuildProcessSemanticsFromInEkfConfig(test_case.inekf);
    builder_input.noise = noise;
    builder_input.C_bn = C_bn;
    builder_input.f_b_corr = f_corrected;
    builder_input.omega_ib_b_corr = omega_corrected;
    builder_input.f_b_unbiased = f_unbiased;
    builder_input.omega_ib_b_unbiased = omega_unbiased;
    builder_input.sf_a = sf_a;
    builder_input.sf_g = sf_g;
    builder_input.v_ned = v_ned;
    builder_input.lat = lat;
    builder_input.h = h;
    builder_input.dt = dt;
    builder_input.ri_vel_gyro_noise_mode = test_case.inekf.ri_vel_gyro_noise_mode;

    const ProcessLinearization builder =
        BuildProcessLinearization(builder_input);
    const double phi_diff =
        (wrapper_Phi - builder.Phi).cwiseAbs().maxCoeff();
    const double qd_diff =
        (wrapper_Qd - builder.Qd).cwiseAbs().maxCoeff();
    Expect(phi_diff <= 1.0e-12,
           "InsMech::BuildProcessModel should forward Phi semantics identically to the extracted builder"
           " for case=" + test_case.tag + " (max_abs=" + to_string(phi_diff) + ")");
    Expect(qd_diff <= 1.0e-12,
           "InsMech::BuildProcessModel should forward Qd semantics identically to the extracted builder"
           " for case=" + test_case.tag + " (max_abs=" + to_string(qd_diff) + ")");
  }
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

void TestRunFusionRuntimeRecordsExactGnssSplitScheduling() {
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

  const FusionRuntimeOutput runtime =
      RunFusionRuntime(options, dataset, x0, P0, nullptr);
  Expect(!runtime.result.fused_positions.empty(),
         "RunFusionRuntime should produce at least one fused state for GNSS timing regression");
  Expect(runtime.stats.gnss_exact_calls == 1,
         "RunFusionRuntime should record one exact-GNSS scheduling call for the split-step smoke test");
  Expect(runtime.stats.gnss_update_calls == 1,
         "RunFusionRuntime should record one GNSS update call for the split-step smoke test");
  Expect(runtime.stats.gnss_samples == 1,
         "RunFusionRuntime should record one consumed GNSS sample for the split-step smoke test");
  Expect(runtime.stats.gnss_align_prev_count == 0,
         "GNSS split smoke should not align the measurement to the previous IMU sample unexpectedly");
  Expect(runtime.stats.gnss_align_curr_predict_count == 0,
         "GNSS split smoke should not take the align-current predict path");
  Expect(runtime.stats.gnss_split_predict_count == 1,
         "GNSS split smoke should exercise the split predict path exactly once");
  Expect(runtime.stats.gnss_tail_predict_count == 1,
         "GNSS split smoke should propagate the post-GNSS tail segment exactly once");
  Expect(runtime.stats.direct_predict_count == 0,
         "GNSS split smoke should not fall back to the direct predict path");
}

void TestRunFusionRuntimeCreatesNhcAdmissionLogParentDir() {
  const fs::path temp_dir = "__regression_nhc_admission_log_dir__";
  const fs::path sol_path = temp_dir / "nested" / "sol.txt";
  const fs::path log_path = temp_dir / "nested" / "sol_nhc_admission.csv";
  fs::remove_all(temp_dir);

  FusionOptions options;
  options.output_path = sol_path.string();
  options.enable_gnss_velocity = false;
  options.constraints.enable_nhc = false;
  options.constraints.enable_odo = false;
  options.constraints.enable_zupt = false;
  options.constraints.enable_nhc_admission_log = true;
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

  const FusionRuntimeOutput runtime =
      RunFusionRuntime(options, dataset, x0, P0, nullptr);
  Expect(!runtime.result.fused_positions.empty(),
         "RunFusionRuntime should still produce states when NHC admission logging is enabled");

  {
    ifstream fin(log_path);
    Expect(static_cast<bool>(fin),
           "RunFusionRuntime should create parent directories for NHC admission logs");
  }

  fs::remove_all(temp_dir);
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

void TestSaveMatrixCreatesParentDir() {
  const fs::path temp_dir = "__regression_save_matrix_dir__";
  const fs::path temp_path = temp_dir / "nested" / "matrix.txt";
  fs::remove_all(temp_dir);

  MatrixXd mat(1, 2);
  mat << 1.0, 2.0;
  io::SaveMatrix(temp_path.string(), mat, "a b");

  {
    ifstream fin(temp_path);
    Expect(static_cast<bool>(fin),
           "SaveMatrix should create parent directories for fresh output paths");
  }

  fs::remove_all(temp_dir);
}

void TestSaveFusionResultCreatesParentDir() {
  const fs::path temp_dir = "__regression_save_fusion_result_dir__";
  const fs::path temp_path = temp_dir / "nested" / "sol.txt";
  fs::remove_all(temp_dir);

  EvaluationSummary summary;
  summary.output_matrix = MatrixXd::Zero(1, 31);
  summary.output_matrix(0, 0) = 1.0;

  SaveFusionResult(temp_path.string(), summary);

  {
    ifstream fin(temp_path);
    Expect(static_cast<bool>(fin),
           "SaveFusionResult should create parent directories for fresh output paths");
  }

  fs::remove_all(temp_dir);
}

void TestOpenOutputFileCreatesParentDir() {
  const fs::path temp_dir = "__regression_open_output_file_dir__";
  const fs::path temp_path = temp_dir / "nested" / "debug.csv";
  fs::remove_all(temp_dir);

  {
    ofstream fout = io::OpenOutputFile(temp_path.string());
    Expect(static_cast<bool>(fout),
           "OpenOutputFile should return an open stream for fresh output paths");
    fout << "header\n";
  }

  {
    ifstream fin(temp_path);
    Expect(static_cast<bool>(fin),
           "OpenOutputFile should create parent directories for fresh output paths");
  }

  fs::remove_all(temp_dir);
}

}  // namespace

int main() {
  try {
    TestStandardEskfSemanticContract();
    TestEnabledInEkfSemanticsCollapseToInEkf();
    TestInEkfBuilderMatchesCanonicalSemantics();
    TestGnssPositionMeasurementContractUsesMeasurementMinusPrediction();
    TestGnssVelocityMeasurementContractUsesMeasurementMinusPrediction();
    TestRoadMeasurementContractsUseMeasurementMinusPrediction();
    TestUwbMeasurementContractUsesMeasurementMinusPrediction();
    TestLegacyMeasurementWrappersMatchUnifiedBuilders();
    TestInEkfGnssPositionAttitudeJacobianMatchesFiniteDifference();
    TestFilterResetSourceHasNoDeadInEkfBranches();
    TestGnssMeasurementSourceHasNoDeadInEkfBranches();
    TestInEkfSemanticBranches();
    TestStateBlockCatalogMatchesLegacyStateIndices();
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
    TestLoadDatasetUsesSecondColumnAsTruthTimeForWeekSowFormat();
    TestProcessModelVectorPriorityBgZ();
    TestProcessModelBiasScaleCouplingMatchesNominalCorrection();
    TestStandardEskfCorrectionInjectionUsesUnifiedImuErrorConvention();
    TestNavigationFilterEngineUsesAdditiveImuErrorInjection();
    TestNavigationFilterEngineUsesHistoricalStandardEskfAttitudeInjection();
    TestNavigationFilterEngineMatchesHistoricalStandardEskfAttitudeInjection();
    TestNavigationFilterEngineCorrectionMatchesEskfCompatibilityWrapper();
    TestNavigationFilterEngineTrueInEkfResetMatchesEskfCompatibilityWrapper();
    TestNavigationFilterEngineStandardResetMatchesEskfCompatibilityWrapper();
    TestNavigationFilterEngineOwnsPredictCovarianceAndDebugSnapshot();
    TestStandardEskfErrorStateRoundTrip();
    TestStandardEskfProcessModelImuCouplingMatchesFiniteDifference();
    TestStandardEskfProcessBuilderMatchesFiniteDifference();
    TestInEkfProcessSemanticBridgeHonorsDebugForceOverrides();
    TestInsMechProcessWrapperMatchesBuilderDispatch();
    TestInsMechanizationUsesReciprocalScaleCompensation();
    TestRunFusionRuntimeRecordsExactGnssSplitScheduling();
    TestRunFusionRuntimeCreatesNhcAdmissionLogParentDir();
    TestSaveMatrixCreatesParentDir();
    TestSaveFusionResultCreatesParentDir();
    TestOpenOutputFileCreatesParentDir();
    TestSaveStateSeriesSmoke();
    cout << "regression_checks: PASS\n";
  } catch (const exception &e) {
    cerr << "regression_checks: FAIL: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
