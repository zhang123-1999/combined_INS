#include "navigation/measurement_model.h"

using namespace std;
using namespace Eigen;

MeasurementLinearization BuildGnssPositionMeasurement(
    const GnssPositionMeasurementInput &input) {
  using namespace measurement_model_detail;

  const bool use_inekf =
      input.context.semantics.flavor != FilterFlavor::kStandardEskf;
  MeasurementLinearization model =
      MakeMeasurementLinearization(input.context, 3, "GNSS_POS", "NED");

  const MeasurementFrameContext frame = BuildMeasurementFrameContext(input.state);

  if (use_inekf) {
    const Vector3d d_ecef = input.z_ecef - input.state.p;
    const Vector3d d_ned = frame.R_ne.transpose() * d_ecef;
    const Vector3d d_body = frame.C_bn.transpose() * d_ned;

    model.y = d_body - input.state.gnss_lever_arm;
    model.H.block<3, 3>(0, StateIdx::kPos) = Matrix3d::Identity();
    model.H.block<3, 3>(0, StateIdx::kAtt) = -Skew(d_body);
    model.H.block<3, 3>(0, StateIdx::kGnssLever) = Matrix3d::Identity();

    const Matrix3d R_ned =
        (input.sigma_gnss.cwiseProduct(input.sigma_gnss)).asDiagonal();
    model.R = frame.C_bn.transpose() * R_ned * frame.C_bn;
    model.frame_tag = "BODY";
    return model;
  }

  const Vector3d lever_ned = frame.C_bn * input.state.gnss_lever_arm;
  const Vector3d p_pred_ecef = input.state.p + frame.R_ne * lever_ned;

  const Vector3d dr_ecef = p_pred_ecef - input.z_ecef;
  const Vector3d dr_ned = frame.R_ne.transpose() * dr_ecef;
  model.y = -dr_ned;

  model.H.block<3, 3>(0, StateIdx::kPos) = Matrix3d::Identity();
  model.H.block<3, 3>(0, StateIdx::kAtt) = Skew(lever_ned);
  model.H.block<3, 3>(0, StateIdx::kGnssLever) = frame.C_bn;
  model.R = (input.sigma_gnss.cwiseProduct(input.sigma_gnss)).asDiagonal();
  return model;
}

MeasurementLinearization BuildGnssVelocityMeasurement(
    const GnssVelocityMeasurementInput &input) {
  using namespace measurement_model_detail;

  const bool use_inekf = UseInEkf(input.context);
  MeasurementLinearization model =
      MakeMeasurementLinearization(input.context, 3, "GNSS_VEL", "NED");

  const MeasurementFrameContext frame =
      BuildMeasurementFrameContext(input.state);
  const AngularRateContext rates =
      BuildAngularRateContext(input.state, frame, input.omega_ib_b_raw);

  const Vector3d lever_vel_b =
      Skew(rates.omega_nb_b) * input.state.gnss_lever_arm;
  const Vector3d lever_vel_n = frame.C_bn * lever_vel_b;

  const Vector3d v_pred_ned = frame.v_ned + lever_vel_n;
  const Vector3d z_gnss_vel_ned =
      frame.R_ne.transpose() * input.z_gnss_vel_ecef;
  model.y = z_gnss_vel_ned - v_pred_ned;

  model.H.block<3, 3>(0, StateIdx::kVel) = Matrix3d::Identity();

  const Matrix3d H_phi_1 =
      -Skew(rates.omega_in_n) * Skew(frame.C_bn * input.state.gnss_lever_arm);
  const Vector3d Cb_l_cross_omega_H =
      frame.C_bn * Skew(input.state.gnss_lever_arm) * rates.omega_ib_corr;
  const Matrix3d H_phi_2 = -Skew(Cb_l_cross_omega_H);
  const Matrix3d H_phi = H_phi_1 + H_phi_2;
  model.H.block<3, 3>(0, StateIdx::kAtt) = H_phi;

  model.H.block<3, 3>(0, StateIdx::kBg) =
      frame.C_bn * Skew(input.state.gnss_lever_arm) * rates.sf_g.asDiagonal();
  model.H.block<3, 3>(0, StateIdx::kSg) =
      frame.C_bn * Skew(input.state.gnss_lever_arm) *
      (rates.omega_ib_unbiased.cwiseProduct(
           rates.sf_g.cwiseProduct(rates.sf_g)))
          .asDiagonal();
  model.H.block<3, 3>(0, StateIdx::kGnssLever) =
      frame.C_bn * Skew(rates.omega_nb_b);

  if (use_inekf) {
    TransformAdditiveCoreJacobianToInEkf(model.H, frame.C_bn);
  }

  model.R =
      input.sigma_gnss_vel.cwiseProduct(input.sigma_gnss_vel).asDiagonal();
  return model;
}
