#include "navigation/filter_engine.h"

#include <limits>
#include <stdexcept>

#include "utils/math_utils.h"

using namespace std;
using namespace Eigen;

namespace {

Matrix<double, kPredictDebugCommonDim, kPredictDebugCommonDim>
ExtractPredictCommonBlock(
    const Matrix<double, kStateDim, kStateDim> &mat) {
  return mat.block<kPredictDebugCommonDim, kPredictDebugCommonDim>(0, 0);
}

}  // namespace

NavigationFilterEngine::NavigationFilterEngine(const NoiseParams &noise,
                                               const FilterSemantics &semantics)
    : noise_(noise), semantics_(semantics) {
  state_mask_.fill(true);
}

void NavigationFilterEngine::Initialize(
    const State &state, const Matrix<double, kStateDim, kStateDim> &P0) {
  state_ = state;
  P_ = P0;
  if (inekf_ != nullptr && inekf_->enabled) {
    inekf_->p_init_ecef = state.p;
  }
  ApplyStateMaskToCov();
  initialized_ = true;
}

void NavigationFilterEngine::OverrideStateAndCov(
    const State &state, const Matrix<double, kStateDim, kStateDim> &P) {
  state_ = state;
  P_ = 0.5 * (P + P.transpose());
  ApplyStateMaskToCov();
}

bool NavigationFilterEngine::Predict(const ProcessLinearization &process,
                                     double t_prev, double t_curr) {
  last_predict_debug_.valid = false;
  if (!initialized_) {
    return false;
  }

  const double dt = t_curr - t_prev;
  if (dt <= 1.0e-9) {
    return false;
  }

  State predicted_state = process.propagation.state;
  ApplyMarkovNominalPropagation(predicted_state, dt);

  const Matrix<double, kStateDim, kStateDim> P_before = P_;
  const Matrix<double, kStateDim, kStateDim> PhiP =
      process.Phi * P_before * process.Phi.transpose();
  const Matrix<double, kStateDim, kStateDim> P_after_raw =
      PhiP + process.Qd;

  PredictDebugSnapshot snapshot;
  snapshot.valid = true;
  snapshot.t_prev = t_prev;
  snapshot.t_curr = t_curr;
  snapshot.dt = dt;
  snapshot.P_before_common = ExtractPredictCommonBlock(P_before);
  snapshot.Phi_common = ExtractPredictCommonBlock(process.Phi);
  snapshot.Qd_common = ExtractPredictCommonBlock(process.Qd);
  snapshot.PhiP_common = ExtractPredictCommonBlock(PhiP);
  snapshot.P_after_raw_common = ExtractPredictCommonBlock(P_after_raw);
  snapshot.omega_ie_b = process.propagation.omega_ie_b;
  snapshot.dtheta_prev_imu_corr = process.propagation.dtheta_prev_imu_corr;
  snapshot.dtheta_curr_imu_corr = process.propagation.dtheta_curr_imu_corr;
  snapshot.dtheta_prev_corr = process.propagation.dtheta_prev_corr;
  snapshot.dtheta_curr_corr = process.propagation.dtheta_curr_corr;
  snapshot.dvel_prev_corr = process.propagation.dvel_prev_corr;
  snapshot.dvel_curr_corr = process.propagation.dvel_curr_corr;
  snapshot.coning = process.propagation.coning;
  snapshot.sculling = process.propagation.sculling;
  snapshot.dv_nav = process.propagation.dv_nav;
  snapshot.dv_nav_prev_att = process.propagation.dv_nav_prev_att;
  snapshot.gravity_dt = process.propagation.gravity_dt;
  snapshot.coriolis_dt = process.propagation.coriolis_dt;

  state_timestamp_ = t_curr;
  ApplyPredictionResult(predicted_state, P_after_raw, &snapshot);
  return true;
}

void NavigationFilterEngine::ApplyPredictionResult(
    const State &state, const Matrix<double, kStateDim, kStateDim> &P,
    const PredictDebugSnapshot *predict_debug) {
  OverrideStateAndCov(state, P);
  ApplyCovarianceFloor();
  if (predict_debug != nullptr) {
    last_predict_debug_ = *predict_debug;
    last_predict_debug_.valid = predict_debug->valid;
    if (last_predict_debug_.valid) {
      last_predict_debug_.P_after_final_common = ExtractPredictCommonBlock(P_);
    }
  } else {
    last_predict_debug_.valid = false;
  }
}

void NavigationFilterEngine::ApplyMarkovNominalPropagation(State &state,
                                                           double dt) const {
  (void)state;
  (void)dt;
  (void)noise_;
  // Keep nominal inertial sensor error states piecewise constant between
  // measurement-feedback steps. The GM model is kept only in F/Q.
}

bool NavigationFilterEngine::Correct(
    const MeasurementLinearization &measurement, VectorXd *dx_out,
    const StateMask *update_mask, const StateGainScale *gain_scale,
    const StateMeasurementGainScale *gain_element_scale) {
  return Correct(measurement.y, measurement.H, measurement.R, dx_out,
                 update_mask, gain_scale, gain_element_scale);
}

bool NavigationFilterEngine::Correct(
    const VectorXd &y, const MatrixXd &H, const MatrixXd &R, VectorXd *dx_out,
    const StateMask *update_mask, const StateGainScale *gain_scale,
    const StateMeasurementGainScale *gain_element_scale) {
  last_inekf_correction_.valid = false;
  last_correction_debug_.valid = false;
  if (!initialized_ || y.size() == 0) {
    return false;
  }
  if (H.cols() != kStateDim || H.rows() != y.size() || R.rows() != y.size() ||
      R.cols() != y.size()) {
    throw invalid_argument("measurement dimension mismatch");
  }

  const FilterSemantics effective_semantics = ResolveEffectiveSemantics();
  const bool use_inekf =
      effective_semantics.flavor == FilterFlavor::kInEkf;

  const Matrix<double, kStateDim, kStateDim> P_prior = P_;
  const MatrixXd S = H * P_ * H.transpose() + R;
  MatrixXd K = ComputeKalmanGain(H, R);
  ApplyUpdateMaskToKalmanGain(K, update_mask);
  ApplyGainScaleToKalmanGain(K, gain_scale);
  ApplyElementGainScaleToKalmanGain(K, gain_element_scale);
  if (K.rows() != kStateDim || K.cols() != y.size() || !K.allFinite()) {
    return false;
  }

  VectorXd dx = K * y;
  ApplyStateMaskToDx(dx, update_mask);
  if (!dx.allFinite()) {
    return false;
  }
  if (dx.segment<3>(StateIdx::kPos).norm() > 1.0e6) {
    return false;
  }
  if (dx.segment<3>(StateIdx::kVel).norm() > 1.0e3) {
    return false;
  }
  if (dx.segment<3>(StateIdx::kAtt).norm() > EIGEN_PI) {
    return false;
  }

  if (dx_out != nullptr) {
    *dx_out = dx;
  }

  last_correction_debug_.valid = true;
  last_correction_debug_.used_inekf = use_inekf;
  last_correction_debug_.t_state = state_timestamp_;
  last_correction_debug_.P_prior = P_prior;
  last_correction_debug_.y = y;
  last_correction_debug_.H = H;
  last_correction_debug_.R = R;
  last_correction_debug_.S = S;
  last_correction_debug_.K = K;
  last_correction_debug_.dx = dx;

  if (use_inekf) {
    UpdateCovarianceJoseph(K, H, R);
    last_inekf_correction_.t_state = state_timestamp_;
    last_inekf_correction_.P_tilde = P_;
    last_inekf_correction_.dx = dx;
    last_inekf_correction_.covariance_floor_applied = false;
    InjectErrorState(dx, update_mask);
    ApplyInEkfReset(dx);
    last_inekf_correction_.P_after_reset = P_;
    if (inekf_ != nullptr && inekf_->apply_covariance_floor_after_reset) {
      ApplyCovarianceFloor();
      last_inekf_correction_.covariance_floor_applied = true;
    }
    last_inekf_correction_.P_after_all = P_;
    last_inekf_correction_.valid = true;
    return true;
  }

  const bool apply_standard_reset =
      (inekf_ != nullptr && inekf_->debug_enable_standard_reset_gamma);
  if (apply_standard_reset) {
    UpdateCovarianceJoseph(K, H, R);
    InjectErrorState(dx, update_mask);
    ApplyStandardEskfReset(dx);
    return true;
  }

  InjectErrorState(dx, update_mask);
  UpdateCovarianceJoseph(K, H, R);
  return true;
}

void NavigationFilterEngine::SetInEkfManager(InEkfManager *inekf) {
  inekf_ = inekf;
  if (initialized_ && inekf_ != nullptr && inekf_->enabled) {
    inekf_->p_init_ecef = state_.p;
  }
}

void NavigationFilterEngine::SetStateMask(const StateMask &mask) {
  state_mask_ = mask;
  if (initialized_) {
    ApplyStateMaskToCov();
  }
}

FilterSemantics NavigationFilterEngine::ResolveEffectiveSemantics() const {
  if (inekf_ != nullptr) {
    return BuildFilterSemanticsFromInEkfConfig(*inekf_);
  }
  return semantics_;
}

MatrixXd NavigationFilterEngine::ComputeKalmanGain(const MatrixXd &H,
                                                   const MatrixXd &R) const {
  const MatrixXd S = H * P_ * H.transpose() + R;
  const MatrixXd PHt = P_ * H.transpose();
  LDLT<MatrixXd> ldlt(S);
  if (ldlt.info() != Success) {
    return MatrixXd::Constant(kStateDim, H.rows(),
                              numeric_limits<double>::quiet_NaN());
  }
  const MatrixXd X = ldlt.solve(PHt.transpose());
  if (ldlt.info() != Success || !X.allFinite()) {
    return MatrixXd::Constant(kStateDim, H.rows(),
                              numeric_limits<double>::quiet_NaN());
  }
  return X.transpose();
}

void NavigationFilterEngine::UpdateCovarianceJoseph(const MatrixXd &K,
                                                    const MatrixXd &H,
                                                    const MatrixXd &R) {
  const Matrix<double, kStateDim, kStateDim> I =
      Matrix<double, kStateDim, kStateDim>::Identity();
  const Matrix<double, kStateDim, kStateDim> A = I - K * H;
  P_ = A * P_ * A.transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
  ApplyStateMaskToCov();
}
