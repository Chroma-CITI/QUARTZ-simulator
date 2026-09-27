/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2022 Johan FAURE, INRIA, Chroma Team, CITI Lab Lyon, France
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


#include "gazebo_plugins/gazebo_plugin__motor.h"

namespace gazebo {

/////////////////////////////////////////////////
GazeboMotorModel::~GazeboMotorModel() {
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

void GazeboMotorModel::InitializeParams() {}

/////////////////////////////////////////////////
void GazeboMotorModel::Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf) {
  // get model
  model_ = _in_model;
  // get world
  world_ = model_->GetWorld();
  // get robot namespace param
  getSdfParam<std::string>(_in_sdf, "robotNamespace", namespace_, kDefaultNamespace);
  // get link name param
  getSdfParam<std::string>(_in_sdf, "linkName", link_name_, "");
  // Get the pointer to the link.
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("Couldn't find specified link \"" << link_name_ << "\".");
  // get joint name param
  getSdfParam<std::string>(_in_sdf, "jointName", joint_name_, "");
  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("Couldn't find specified joint \"" << joint_name_ << "\".");
  // get rotor id param.
  if (_in_sdf->HasElement("motorNumber"))
  motor_number_ = _in_sdf->GetElement("motorNumber")->Get<int>();
  else
  gzerr << "Please specify a motorNumber.\n";
  // get turning direction
  std::string turning_direction;
  getSdfParam<std::string>(_in_sdf, "turningDirection", turning_direction, "");
  if (turning_direction == "cw")
    turning_direction_ = turning_direction::CW;
  else if (turning_direction == "ccw")
    turning_direction_ = turning_direction::CCW;
  else
    gzthrow("Please only use 'cw' or 'ccw' as turning_direction.\n");

  if(_in_sdf->HasElement("reversible")) {
    reversible_ = _in_sdf->GetElement("reversible")->Get<bool>();
  }

  getSdfParam<std::string>(_in_sdf, "commandSubTopic", command_sub_topic_, kDefaultCommandSubTopic);
  getSdfParam<std::string>(_in_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
                           motor_speed_pub_topic_);

  getSdfParam<double>(_in_sdf, "rotorDragCoefficient", rotor_drag_coefficient_, rotor_drag_coefficient_);
  getSdfParam<double>(_in_sdf, "rollingMomentCoefficient", rolling_moment_coefficient_,
                      rolling_moment_coefficient_);
  getSdfParam<double>(_in_sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
  getSdfParam<double>(_in_sdf, "motorConstant", motor_constant_, motor_constant_);
  getSdfParam<double>(_in_sdf, "momentConstant", moment_constant_, moment_constant_);

  getSdfParam<double>(_in_sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
  getSdfParam<double>(_in_sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
  getSdfParam<double>(_in_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);
  // Set the maximumForce on the joint. This is deprecated from V5 on, and the joint won't move.
  #if GAZEBO_MAJOR_VERSION < 5
    joint_->SetMaxForce(0, max_force_);
  #endif
  // Gazebo initialisation
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboMotorModel::OnUpdate, this, std::placeholders::_1));
  #else
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboMotorModel::OnUpdate, this, _1));
  #endif
  // ROS initialisation
  node_handle_ = new ros::NodeHandle(namespace_);
  command_sub_ = node_handle_->subscribe(command_sub_topic_ ,1000, &GazeboMotorModel::VelocityCallback, this);
  motor_velocity_pub_ = node_handle_->advertise<std_msgs::Float32>(motor_speed_pub_topic_, 1);
  //wind_sub_ = node_handle_->subscribe(wind_sub_topic_, &GazeboMotorModel::WindVelocityCallback, this);
  // Create the first order filter.
  rotor_velocity_filter_.reset(new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));
}

void GazeboMotorModel::VelocityCallback(CommandMotorSpeedPtr &msg_rotorspeed_cmd) {
  if(msg_rotorspeed_cmd->values.size() < motor_number_) {
    std::cout  << "You tried to access index " << motor_number_
      << " of the MotorSpeed message array which is of size " << msg_rotorspeed_cmd->values.size() << "." << std::endl;
  } else ref_motor_rot_vel_ = std::min(static_cast<double>(msg_rotorspeed_cmd->values[motor_number_]), static_cast<double>(max_rot_velocity_));
}

void GazeboMotorModel::WindVelocityCallback(WindPtr& msg) {
  wind_vel_ = ignition::math::Vector3<double>(msg->velocity.x,
            msg->velocity.y,
            msg->velocity.z);
}

/////////////////////////////////////////////////
// This gets called by the world update start event.
void GazeboMotorModel::OnUpdate(const common::UpdateInfo& _info) {
  #if GAZEBO_MAJOR_VERSION >= 9
    double cur_time = world_->SimTime().Double();
  #else
    double cur_time = world_->GetSimTime().Double();
  #endif
  sampling_time_ = cur_time - last_time_;
  last_time_ = cur_time;
  UpdateForcesAndMoments();
  Publish();
}

/////////////////////////////////////////////////
void GazeboMotorModel::UpdateForcesAndMoments() {
  // Taken from :
  //   Forces from Philppe Martin's and Erwan Salaün's
  //   2010 IEEE Conference on Robotics and Automation paper
  //   The True Role of Accelerometer Feedback in Quadrotor Control
  // Induced thrust by the rotation velocity
  motor_rot_vel_ = joint_->GetVelocity(0);
  if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
    gzerr << "Aliasing on motor [" << motor_number_ << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }
  double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
  double force = real_motor_velocity * std::abs(real_motor_velocity) * motor_constant_;
  if(!reversible_) {
    // Not allowed to have negative thrust.
    force = std::abs(force);
  }

  // scale down force linearly with forward speed
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3<double> body_velocity = link_->WorldLinearVel();
#else
  ignition::math::Vector3<double> body_velocity = GazeboToIgnition(link_->GetWorldLinearVel());
#endif

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3<double> joint_axis = joint_->GlobalAxis(0);
#else
  ignition::math::Vector3<double> joint_axis = GazeboToIgnition(joint_->GetGlobalAxis(0));
#endif

  // double bodyspeed = body_velocity.Length();
  // double efficiency = ignition::math::clamp(1.0 - (bodyspeed / 25.0), 0.0, 1.0); // at 25m/s the rotor will not produce any force anymore
  // link_->AddRelativeForce(ignition::math::Vector3<double>(0, 0, force * efficiency));
  ignition::math::Vector3<double> relative_wind_velocity = body_velocity - wind_vel_;
  ignition::math::Vector3<double> velocity_parallel_to_rotor_axis = (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
  double vel = velocity_parallel_to_rotor_axis.Length();
  double scalar = 1 - vel / 25.0; // at 25 m/s the rotor will not produce any force anymore
  scalar = ignition::math::clamp(scalar, 0.0, 1.0);
  // Apply a force to the link.
  link_->AddRelativeForce(ignition::math::Vector3<double>(0, 0, force * scalar));

  ignition::math::Vector3<double> velocity_perpendicular_to_rotor_axis = relative_wind_velocity - (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
  ignition::math::Vector3<double> air_drag = -std::abs(real_motor_velocity) * rotor_drag_coefficient_ * velocity_perpendicular_to_rotor_axis;
  // Apply air_drag to link.
  link_->AddForce(air_drag);

  // ignition::math::Vector3<double> vec_bodyspeed_proj = joint_axis.Cross(body_velocity.Cross(joint_axis));  // vector bodyspeed projected on the rotor plane
  // ignition::math::Vector3<double> force_drag = -std::abs(real_motor_velocity) * (rotor_drag_coefficient_ * vec_bodyspeed_proj);
  // link_->AddForce(force_drag);

  // Moments
  // Getting the parent link, such that the resulting torques can be applied to it.
  physics::Link_V parent_links = link_->GetParentJointsLinks();
  // The tansformation from the parent_link to the link_.
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3<double> pose_difference = link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
#else
  ignition::math::Pose3<double> pose_difference = ignitionFromGazeboMath(link_->GetWorldCoGPose() - parent_links.at(0)->GetWorldCoGPose());
#endif
  ignition::math::Vector3<double> drag_torque(0, 0, -turning_direction_ * force * moment_constant_);
  // Transforming the drag torque into the parent frame to handle arbitrary rotor orientations.
  ignition::math::Vector3<double> drag_torque_parent_frame = pose_difference.Rot().RotateVector(drag_torque);
  parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

  ignition::math::Vector3<double> rolling_moment;
  // - \omega * \mu_1 * V_A^{\perp}
  rolling_moment = -std::abs(real_motor_velocity) * turning_direction_ * rolling_moment_coefficient_ * velocity_perpendicular_to_rotor_axis;
  parent_links.at(0)->AddTorque(rolling_moment);
  // Apply the filter on the motor's velocity.
  double ref_motor_rot_vel;
  ref_motor_rot_vel = rotor_velocity_filter_->UpdateFilter(ref_motor_rot_vel_, sampling_time_);
  joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
}

/////////////////////////////////////////////////
void GazeboMotorModel::Publish() {
  std_msgs::Float32 turning_velocity_msg_;
  turning_velocity_msg_.data = std::max(std::min((turning_direction_ * motor_rot_vel_ * rotor_velocity_slowdown_sim_), max_rot_velocity_), 0.0);;
  // FIXME: Commented out to prevent warnings about queue limit reached.
  motor_velocity_pub_.publish(turning_velocity_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel);
}
