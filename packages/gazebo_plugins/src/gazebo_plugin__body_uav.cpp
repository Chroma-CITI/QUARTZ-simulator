/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 * Copyright 2018-2020 Vincent LE DOZE, INRIA, Chroma Team, CITI Lab Lyon, France
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "gazebo_plugins/gazebo_plugin__body_uav.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BodyUAVPlugin);

/////////////////////////////////////////////////
BodyUAVPlugin::BodyUAVPlugin() : BodyPlugin() {
}

/////////////////////////////////////////////////
BodyUAVPlugin::~BodyUAVPlugin() {
}

/////////////////////////////////////////////////
void BodyUAVPlugin::Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf) {
  // Get the other parameters
  getSdfParam<bool>(_in_sdf, "radial_symmetry", radial_symmetry_, KDEFAULT_RADIAL_SYMMETRY);
  //
  getSdfParam<double>(_in_sdf, "area", area_, KDEFAULT_AREA);
  getSdfParam<double>(_in_sdf, "air_density", rho_, KDEFAULT_RHO);
  //
  getSdfParam<double>(_in_sdf, "alpha_zero", alpha_zero_, KDEFAULT_ALPHA_ZERO);
  //
  getSdfParam<double>(_in_sdf, "lift_coef_zero",   lift_coef_zero_,   KDEFAULT_LIFT_COEF_ZERO);
  getSdfParam<double>(_in_sdf, "drag_coef_zero",  drag_coef_zero_,  KDEFAULT_DRAG_COEF_ZERO);
  getSdfParam<double>(_in_sdf, "moment_coef_zero", moment_coef_zero_, KDEFAULT_MOMENT_COEF_ZERO);
  //
  getSdfParam<double>(_in_sdf, "alpha_stall", alpha_stall_, KDEFAULT_ALPHA_STALL);
  getSdfParam<double>(_in_sdf, "lift_coef_stall", lift_coef_stall_, KDEFAULT_LIFT_COEF_STALL);
  getSdfParam<double>(_in_sdf, "drag_coef_stall", drag_coef_stall_, KDEFAULT_DRAG_COEF_STALL);
  getSdfParam<double>(_in_sdf, "moment_coef_stall", moment_coef_stall_, KDEFAULT_MOMENT_COEF_STALL);
  //
  getSdfParam<ignition::math::Vector3<double>>(_in_sdf, "chord_direction", chord_direction_, KDEFAULT_CHORD_DIRECTION);
  getSdfParam<ignition::math::Vector3<double>>(_in_sdf, "center_of_pressure", center_of_pressure_, KDEFAULT_CENTER_OF_PRESSURE);
  //
  chord_direction_.Normalize(); // chord direction in link frame
  // Lift function Coefficients
  funct_lift_coef_ = GetLinearFunctionCoefs(alpha_zero_, lift_coef_zero_, alpha_stall_, lift_coef_stall_);
  funct_lift_coef_stall_pos_ = GetLinearFunctionCoefs(alpha_stall_, lift_coef_stall_, M_PI/2.0, 0.0);
  funct_lift_coef_stall_neg_ = GetLinearFunctionCoefs(
    -alpha_stall_, funct_lift_coef_.first*(-alpha_stall_) + funct_lift_coef_.second, -M_PI/2.0, 0.0);
  // Lift function Coefficients
  funct_drag_coef_pos_ = GetLinearFunctionCoefs(alpha_zero_, drag_coef_zero_, alpha_stall_, drag_coef_stall_);
  funct_drag_coef_neg_ = GetLinearFunctionCoefs(alpha_zero_, drag_coef_zero_, -alpha_stall_, drag_coef_stall_);
  funct_drag_coef_stall_pos_ = GetLinearFunctionCoefs(alpha_stall_, drag_coef_stall_, M_PI/2.0, 1.0);
  funct_drag_coef_stall_neg_ = GetLinearFunctionCoefs(
    -alpha_stall_, funct_drag_coef_neg_.first*(-alpha_stall_) + funct_drag_coef_neg_.second, -M_PI/2.0, 1.0);
  // Lift function Coefficients
  funct_moment_coef_ = GetLinearFunctionCoefs(alpha_zero_, moment_coef_zero_, alpha_stall_, moment_coef_stall_);
  funct_moment_coef_stall_pos_ = GetLinearFunctionCoefs(alpha_stall_, moment_coef_stall_, M_PI/2.0, 0.0);
  funct_moment_coef_stall_neg_ = GetLinearFunctionCoefs(
    -alpha_stall_, funct_moment_coef_.first*(-alpha_stall_) + funct_lift_coef_.second, -M_PI/2.0, 0.0);
  // Flaperon joint
  std::string flapperon_joint_name;
  getSdfParam<std::string>(_in_sdf, "flapperon_joint_name", flapperon_joint_name, "");
  flaperon_joint_ = _in_model->GetJoint(flapperon_joint_name);
  // if (!flaperon_joint_)
  //   gzerr << "Flaperon Joint with name [" << flapperon_joint_name << "] does not exist.\n";
  // Flaperon lift coef
  getSdfParam<double>(_in_sdf, "lift_coef_flaperon", lift_coef_flaperon_, KDEFAULT_LIFT_COEF_FLAPERON);
  // Parent load
  BodyPlugin::Load(_in_model, _in_sdf);
}

/////////////////////////////////////////////////
void BodyUAVPlugin::OnUpdate() {
  // Get state and time
  GetStateAndTime();
  // Calculate body aerodynamics
  SetAero();
  // send ground truth information
  SendGroundTruth();
}

/// AERODYNAMIC /////////////////////////////////
void BodyUAVPlugin::SetAero() {
  // get linear velocity at center_pressure
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3<double> vector_vel = state_.Rot().RotateVectorReverse(
    link_->WorldLinearVel(center_of_pressure_));
#else
  ignition::math::Vector3<double> vector_vel = state_.Rot().RotateVectorReverse(
    GazeboToIgnition(link_->GetWorldLinearVel(center_of_pressure_)));
#endif
  // continue if vector_vel > vector_vel min
  double vel = vector_vel.Length();
  if (vel <= 0.01)
    return;
  // We normalize the velocity vector
  vector_vel.Normalize();
  // Angle of attack
  double alpha = alpha_zero_ - atan2(vector_vel.Z(), vector_vel.X());
  // Normalize to within +/-90 deg
  if (alpha > 0.5 * M_PI)
    alpha = M_PI - alpha;
  else if (alpha > 0.5 * M_PI)
    alpha = -M_PI - alpha;
  // Cos of the angle of drag for lift correction
  double cos_beta = abs(vector_vel.X());
  // Compute dynamic pressureforward
  double q = 0.5 * rho_ * vel * vel;
  // Compute Lift, Drag and Moment Coefficients
  double lift_coef, drag_coef, moment_coef;
  if (alpha > alpha_stall_) {
    lift_coef = funct_lift_coef_stall_pos_.first*alpha + funct_lift_coef_stall_pos_.second;
    drag_coef = funct_drag_coef_stall_pos_.first*alpha + funct_drag_coef_stall_pos_.second;
    moment_coef = funct_moment_coef_stall_pos_.first*alpha + funct_moment_coef_stall_pos_.second;
  }
  else if(alpha < -alpha_stall_){
    lift_coef = funct_lift_coef_stall_neg_.first*alpha + funct_lift_coef_stall_neg_.second;
    drag_coef = funct_drag_coef_stall_neg_.first*alpha + funct_drag_coef_stall_neg_.second;
    moment_coef = funct_moment_coef_stall_neg_.first*alpha + funct_moment_coef_stall_neg_.second;
  }
  else{
    lift_coef = funct_lift_coef_.first*alpha + funct_lift_coef_.second;
    if (alpha > 0.0)
      drag_coef = funct_drag_coef_pos_.first*alpha + funct_drag_coef_pos_.second;
    else
      drag_coef = funct_drag_coef_neg_.first*alpha + funct_drag_coef_neg_.second;
    moment_coef = funct_moment_coef_.first*alpha + funct_moment_coef_.second;
  }
  // Lift correction
  lift_coef *= cos_beta;
  // modify lift_coef per control joint value
  if (flaperon_joint_)
  {
#if GAZEBO_MAJOR_VERSION >= 9
    double controlAngle = flaperon_joint_->Position(0);
#else
    double controlAngle = flaperon_joint_->GetAngle(0).Radian();
#endif
    lift_coef = lift_coef + lift_coef_flaperon_ * controlAngle;
  }
  // Lift and Drag directions
  ignition::math::Vector3<double> drag_direction = -vector_vel;
  ignition::math::Vector3<double> lift_direction = chord_direction_.Cross(drag_direction);
  // compute lift and drag force at center_pressure
  ignition::math::Vector3<double> lift = lift_coef * q * area_ * lift_direction;
  ignition::math::Vector3<double> drag = drag_coef * q * area_ * drag_direction;
  /// \TODO: implement cm
  /// for now, reset cm to zero, as cm needs testing
  moment_coef = 0.0;
  // compute moment (torque) at center_pressure
  ignition::math::Vector3<double> moment = moment_coef * q * area_ * chord_direction_;
  // force and torque about center of pressure in inertial framevector_vel
  ignition::math::Vector3<double> force = state_.Rot().RotateVector(lift + drag);
  ignition::math::Vector3<double> torque = state_.Rot().RotateVector(moment);
  // Correct for nan or inf
  force.Correct();
  torque.Correct();
  // apply forces and torque at the center of pressure
  link_->AddForceAtRelativePosition(force, center_of_pressure_);
  link_->AddTorque(torque);
}
