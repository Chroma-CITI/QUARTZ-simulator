/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2018-2020 Vincent LE DOZE, INRIA, Chroma Team, CITI Lab Lyon, France
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "gazebo_plugins/gazebo_plugin__rotor.h"

namespace gazebo {

/////////////////////////////////////////////////
RotorModel::RotorModel() : ModelPlugin(),
  rotorspeed_(0),
  rotorspeed_cmd_(0),
  rotorspeed_cmd_filt_(0),
  last_time_(0),
  sampling_time_(0.01) {}

/////////////////////////////////////////////////
RotorModel::~RotorModel() {
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_.reset();
  #else
  event::Events::DisconnectWorldUpdateBegin(update_connection_ptr_);
  #endif
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

/////////////////////////////////////////////////
void RotorModel::Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf) {
  // get model
  model_ = _in_model;
  // get world
  world_ = model_->GetWorld();
  // get robot namespace param
  getSdfParam<std::string>(_in_sdf, "robot_namespace", namespace_, KDEFAULT_NAMESPACE);
  // get link name param
  getSdfParam<std::string>(_in_sdf, "link_name", link_name_, "");
  // Get the pointer to the link.
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("Couldn't find specified link \"" << link_name_ << "\".");
  // get joint name param
  getSdfParam<std::string>(_in_sdf, "joint_name", joint_name_, "");
  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("Couldn't find specified joint \"" << joint_name_ << "\".");
  // get rotor id param.
  if (getSdfParam<int>(_in_sdf, "rotor_id", rotor_id_, -1) == false)
    gzthrow("Please specify a rotor_id .\n");
  // get turnin direction
  std::string turning_direction;
  getSdfParam<std::string>(_in_sdf, "turning_direction", turning_direction, "");
  if (turning_direction == "cw")
    turning_direction_ = turning_direction::CW;
  else if (turning_direction == "ccw")
    turning_direction_ = turning_direction::CCW;
  else
    gzthrow("Please only use 'cw' or 'ccw' as turning_direction.\n");
  // get ros topic names
  getSdfParam<std::string>(_in_sdf, "topic_rotorspeed_cmd", topic_rotorspeed_cmd_, KDEFAULT_TOPIC_ROTORSPEED_CMD);
  getSdfParam<std::string>(_in_sdf, "topic_rotorspeed_val", topic_rotorspeed_val_, KDEFAULT_TOPIC_ROTORSPEED_VAL);
  // get rotor parameters
  getSdfParam<double>(_in_sdf, "bodyspeed_max", bodyspeed_max_, KDEFAULT_BODYSPEED_MAX);
  getSdfParam<double>(_in_sdf, "rotorspeed_max", rotorspeed_max_, KDEFAULT_ROTORSPEED_MAX);
  getSdfParam<double>(_in_sdf, "thrust_constant", thrust_constant_, KDEFAULT_THRUST_CONSTANT);
  getSdfParam<double>(_in_sdf, "torque_constant", torque_constant_, KDEFAULT_TORQUE_CONSTANT);
  getSdfParam<double>(_in_sdf, "rotor_drag_coef", rotor_drag_coef_, KDEFAULT_ROTOR_DRAG_COEF);
  getSdfParam<double>(_in_sdf, "rotor_moment_coef", rotor_moment_coef_, KDEFAULT_ROTOR_MOMENT_COEF);
  // Set the maximumForce on the joint. This is deprecated from V5 on, and the joint won't move.
  #if GAZEBO_MAJOR_VERSION < 5
    joint_->SetMaxForce(0, max_force_);
  #endif
  // get filter parameters
  getSdfParam<double>(_in_sdf, "filter_time_constant_up", filter_time_constant_up_, KDEFAULT_FILTER_TIME_CONSTANT_UP);
  getSdfParam<double>(_in_sdf, "filter_time_constant_down", filter_time_constant_down_, KDEFAULT_FILTER_TIME_CONSTANT_DOWN);
  getSdfParam<double>(_in_sdf, "rotor_velocity_slowdown_sim", rotor_velocity_slowdown_sim_, KDEFAULT_ROTOR_VELOCITY_SLOWDOWN_SIM);
  // Gazebo initialisation
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(std::bind(&RotorModel::OnUpdate, this, std::placeholders::_1));
  #else
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&RotorModel::OnUpdate, this, _1));
  #endif
  // Reset model attributes
  Reset();
  // ROS initialisation
  node_handle_ = new ros::NodeHandle(namespace_);
  sub_rotorspeed_cmd_ = node_handle_->subscribe(topic_rotorspeed_cmd_, 1000, &RotorModel::CallbackRotorspeedCmd, this);
  pub_rotorspeed_val_ = node_handle_->advertise<std_msgs::Float32>(topic_rotorspeed_val_, 10);
}

void RotorModel::Reset() {
  rotorspeed_ = 0.0;
  rotorspeed_cmd_ = 0.0;
  rotorspeed_cmd_filt_ = 0.0;
  rotorspeed_filter_.reset(new FirstOrderFilter<double>(filter_time_constant_up_, filter_time_constant_down_, rotorspeed_cmd_));
  #if GAZEBO_MAJOR_VERSION >= 9
    last_time_ = world_->SimTime().Double();
  #else
    last_time_ = world_->GetSimTime().Double();
  #endif
  sampling_time_ = 0.1;
  joint_->SetVelocity(0, 0.0);
  link_->ResetPhysicsStates();
}

void RotorModel::CallbackRotorspeedCmd(const chroma_gazebo_plugins_msgs::Rotorspeed& msg_rotorspeed_cmd) {
  if (msg_rotorspeed_cmd.values.size() < rotor_id_)
    gzerr << "You tried to access index " << rotor_id_
          << " of the MotorSpeed message array which is of size "
          << msg_rotorspeed_cmd.values.size() << ".";
  else
    rotorspeed_cmd_ = std::min(msg_rotorspeed_cmd.values[rotor_id_], rotorspeed_max_);
}

/////////////////////////////////////////////////
// This gets called by the world update start event.
void RotorModel::OnUpdate(const common::UpdateInfo& _info) {
  #if GAZEBO_MAJOR_VERSION >= 9
    double cur_time = world_->SimTime().Double();
  #else
    double cur_time = world_->GetSimTime().Double();
  #endif
  sampling_time_ = cur_time - last_time_;
  last_time_ = cur_time;
  GetRotorSpeed();
  UpdateForcesAndMoments();
  SetRotorSpeed();
  Publish();
}

/////////////////////////////////////////////////
void RotorModel::UpdateForcesAndMoments() {
  // Taken from :
  //   Forces from Philppe Martin's and Erwan Salaün's
  //   2010 IEEE Conference on Robotics and Automation paper
  //   The True Role of Accelerometer Feedback in Quadrotor Control
  // Induced thrust from the rotation velocity
  double thrust = thrust_constant_ * rotorspeed_ * rotorspeed_;
  // scale down force linearly with forward speed
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3<double> vec_bodyspeed = link_->WorldLinearVel();
#else
  ignition::math::Vector3<double> vec_bodyspeed = GazeboToIgnition(link_->GetWorldLinearVel());
#endif
  double bodyspeed = vec_bodyspeed.Length();
  double efficiency = ignition::math::clamp(1.0 - (bodyspeed / bodyspeed_max_), 0.0, 1.0); // at bodyspeed_max_ the rotor will not produce any force anymore
  link_->AddRelativeForce(ignition::math::Vector3<double>(0, 0, efficiency * thrust));
  // Calculation of the drag induced by the surface swept by the rotor
  //   - \omega * \lambda_1 * V_A^{\perp}
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3<double> rotor_axis = joint_->GlobalAxis(0);
#else
  ignition::math::Vector3<double> rotor_axis = GazeboToIgnition(joint_->GetGlobalAxis(0));
#endif
  ignition::math::Vector3<double> vec_bodyspeed_proj = rotor_axis.Cross(vec_bodyspeed.Cross(rotor_axis));  // vector bodyspeed projected on the rotor plane
  ignition::math::Vector3<double> force_drag = -std::abs(rotorspeed_) * (rotor_drag_coef_ * vec_bodyspeed_proj);
  link_->AddForce(force_drag);
  // Contra-rotating torque induced by blade drag (wind relative to blade rotation speed)
  // Projection in the frame body because the moment does not apply to the rotor (which rotates and therefore has no effect)
  double torque = -turning_direction_ * torque_constant_ * rotorspeed_ * rotorspeed_;
  physics::Link_V body_link = link_->GetParentJointsLinks();
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3<double> pose_bodycog = link_->WorldCoGPose() - body_link.at(0)->WorldCoGPose();
#else
  ignition::math::Pose3<double> pose_bodycog = GazeboToIgnition(link_->GetWorldCoGPose() - body_link.at(0)->GetWorldCoGPose());
#endif
  body_link.at(0)->AddRelativeTorque(pose_bodycog.Rot().RotateVector(ignition::math::Vector3<double>(0, 0, torque)));
  // Rolling moment induced by the surface swept by the rotor
  // -> Relative wind on the blades = bodyspeed_proj +- rotorspeed
  // -> One side of the rotor achieves more thrust than the other (higher relative wind)
  // -> A rolling moment is generated :  \omega * \mu_1 * V_A^{\perp}
  ignition::math::Vector3<double> moment_roll = - std::abs(rotorspeed_) * rotor_moment_coef_ * vec_bodyspeed_proj;
  body_link.at(0)->AddTorque(moment_roll);
}

/////////////////////////////////////////////////
void RotorModel::GetRotorSpeed() {
  // Simulated motor speed recovery
  double rotorspeed_sim = joint_->GetVelocity(0);
  if (rotorspeed_sim / (2 * M_PI) > 1 / (2 * sampling_time_)) {
    gzerr << "Aliasing on motor [" << rotor_id_  << "] might occur."
          << "Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }
  // Actual motor speed
  // Bounded to avoid problems with speed "spikes" when the drone falls into the gazebo
  rotorspeed_ = std::max(std::min((turning_direction_ * rotorspeed_sim * rotor_velocity_slowdown_sim_), rotorspeed_max_), 0.0);
}

/////////////////////////////////////////////////
void RotorModel::SetRotorSpeed() {
  // Apply the filter on the motor's velocity.
  rotorspeed_cmd_filt_ = rotorspeed_filter_->UpdateFilter(rotorspeed_cmd_, sampling_time_);
  if (rotorspeed_cmd_filt_ >= rotorspeed_max_){
    gzerr << "motor [" << rotor_id_  << "] rotorspeed_cmd too high. \n";
    rotorspeed_cmd_filt_ = rotorspeed_max_;
  }
  else if (rotorspeed_cmd_filt_ < 0.0){
    gzerr << "motor [" << rotor_id_  << "] rotorspeed_cmd too low. \n";
    rotorspeed_cmd_filt_ = 0.0;
  }
  // Send new speed to simulated motor
  double rotorspeed_sim = turning_direction_ * rotorspeed_cmd_filt_ / rotor_velocity_slowdown_sim_;
  joint_->SetVelocity(0, rotorspeed_sim);
}

/////////////////////////////////////////////////
void RotorModel::Publish() {
  std_msgs::Float32 msg_rotorspeed_val;
  msg_rotorspeed_val.data = rotorspeed_;
  pub_rotorspeed_val_.publish(msg_rotorspeed_val);
}

GZ_REGISTER_MODEL_PLUGIN(RotorModel);
}
