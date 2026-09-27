// Author : vledoze
// Date   : 20/02/2019

/*
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


#include "gazebo_plugins/gazebo_plugin__quadcopter_attitude_controller.h"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(QuadcopterAttitudeControllerPlugin);

/////////////////////////////////////////////////
QuadcopterAttitudeControllerPlugin::QuadcopterAttitudeControllerPlugin() : ModelPlugin() {
    //Etats
    q_n_ = ignition::math::Quaternion<double>(0.0, 0.0, 0.0);
    w1_n_ = ignition::math::Vector3<double>(0.0, 0.0, 0.0);
    w1_n_1_ = ignition::math::Vector3<double>(0.0, 0.0, 0.0);
    a_n_ = ignition::math::Vector3<double>(0.0, 0.0, 0.0);
    //Commandes
    cmd_q_n_ = ignition::math::Quaternion<double>(0.0, 0.0, 0.0);
    cmd_q_n_1_ = ignition::math::Quaternion<double>(0.0, 0.0, 0.0);
    dcmd_q_n_ = ignition::math::Quaternion<double>(0.0, 0.0, 0.0);
    dcmd_q_n_1_ = ignition::math::Quaternion<double>(0.0, 0.0, 0.0);
    cmd_a_n_ = ignition::math::Vector3<double>(0.0, 0.0, 0.0);
    //Pas de temps
    dt_.push_back(0.0);
}

/////////////////////////////////////////////////
QuadcopterAttitudeControllerPlugin::~QuadcopterAttitudeControllerPlugin() {
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
void QuadcopterAttitudeControllerPlugin::Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf) {
  // Store the pointer to the model.
  model_ = _in_model;
  // Store the pointer to the world
  world_ = model_->GetWorld();
  // Initialise temporal variables
  #if GAZEBO_MAJOR_VERSION >= 9
  current_time_ = world_->SimTime();
  last_time_ = world_->SimTime();
  #else
  current_time_ = world_->GetSimTime();
  last_time_ = world_->GetSimTime();
  #endif
  // Get namespace param
  getSdfParam<std::string>(_in_sdf, "robot_namespace", robot_namespace_, KDEFAULT_NAMESPACE);
  // Get link name
  std::string link_name;
  getSdfParam<std::string>(_in_sdf, "link_name", link_name, KDEFAULT_LINK_NAME);
  // Get ros topic names
  getSdfParam<std::string>(_in_sdf, "topic_rotorspeed", topic_rotorspeed_, KDEFAULT_TOPIC_ROTORSPEED);
  getSdfParam<std::string>(_in_sdf, "topic_cmd", topic_cmd_, KDEFAULT_TOPIC_CMD);
  getSdfParam<std::string>(_in_sdf, "topic_imu", topic_imu_, KDEFAULT_TOPIC_IMU);
  // Get rotors informations
  getSdfParam<int>(_in_sdf, "id_front_right_rotor", id_front_right_rotor_, KDEFAULT_ID_FRONT_RIGHT_ROTOR);
  getSdfParam<int>(_in_sdf, "id_front_left_rotor",  id_front_left_rotor_,  KDEFAULT_ID_FRONT_LEFT_ROTOR);
  getSdfParam<int>(_in_sdf, "id_rear_right_rotor",  id_rear_right_rotor_,  KDEFAULT_ID_REAR_RIGHT_ROTOR);
  getSdfParam<int>(_in_sdf, "id_rear_left_rotor",   id_rear_left_rotor_,   KDEFAULT_ID_REAR_LEFT_ROTOR);
  // Get rotors directions
  getSdfParam<double>(_in_sdf, "dir_front_right_rotor", dir_front_right_rotor_, KDEFAULT_DIR_FRONT_RIGHT_ROTOR);
  getSdfParam<double>(_in_sdf, "dir_front_left_rotor",  dir_front_left_rotor_,  KDEFAULT_DIR_FRONT_LEFT_ROTOR);
  getSdfParam<double>(_in_sdf, "dir_rear_right_rotor",  dir_rear_right_rotor_,  KDEFAULT_DIR_REAR_RIGHT_ROTOR);
  getSdfParam<double>(_in_sdf, "dir_rear_left_rotor",   dir_rear_left_rotor_,   KDEFAULT_DIR_REAR_LEFT_ROTOR);
  // Rotormaxspped
  getSdfParam<double>(_in_sdf, "rotorspeed_max", rotorspeed_max_, KDEFAULT_ROTORSPEED_MAX);
  rotorspeed_min_ = rotorspeed_max_/10.0;
  // Get gains
  getSdfParam<double>(_in_sdf, "kp_yaw_rate", kp_yaw_rate_, KDEFAULT_KP_YAW_RATE);
  getSdfParam<double>(_in_sdf, "kp_pit_rate", kp_pit_rate_, KDEFAULT_KP_PIT_RATE);
  getSdfParam<double>(_in_sdf, "kp_rol_rate", kp_rol_rate_, KDEFAULT_KP_ROL_RATE);
  getSdfParam<double>(_in_sdf, "kp_thrust", kp_thrust_, KDEFAULT_KP_THRUST);
  // Get Trim
  getSdfParam<double>(_in_sdf, "trim_pit", trim_pit_, 0.0);
  getSdfParam<double>(_in_sdf, "trim_rol", trim_rol_, 0.0);
  // Get frequency
  double freq;
  getSdfParam<double>(_in_sdf, "freq", freq, KDEFAULT_FREQ);
  interval_ = 1.0/freq;
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(std::bind(&QuadcopterAttitudeControllerPlugin::OnUpdate, this, std::placeholders::_1));
  #else
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&QuadcopterAttitudeControllerPlugin::OnUpdate, this, _1));
  #endif
  // ROS initialisation
  node_handle_ = new ros::NodeHandle(robot_namespace_);
  // ROS publishers
  pub_rotorspeed_ = node_handle_->advertise<chroma_gazebo_plugins_msgs::Rotorspeed>("/" + robot_namespace_ + "/" + topic_rotorspeed_, 1);
  pub_yaw_ = node_handle_->advertise<geometry_msgs::PointStamped>("/" + robot_namespace_ + "/yaw", 1);
  pub_pit_ = node_handle_->advertise<geometry_msgs::PointStamped>("/" + robot_namespace_ + "/pit", 1);
  pub_rol_ = node_handle_->advertise<geometry_msgs::PointStamped>("/" + robot_namespace_ + "/rol", 1);
  // Prepare msg
  msg_rotorspeed_.header.frame_id = "/" + robot_namespace_ + "/" + link_name;
  // ROS Subscribers
  sub_imu_ = node_handle_->subscribe("/" + robot_namespace_ +  "/" + topic_imu_, 1000, &QuadcopterAttitudeControllerPlugin::CallbackImu, this);
  sub_cmd_ = node_handle_->subscribe("/" + robot_namespace_ +  "/" + topic_cmd_, 1000, &QuadcopterAttitudeControllerPlugin::CallbackCmd, this);
}

/////////////////////////////////////////////////
// This gets called by the world update start event.
void QuadcopterAttitudeControllerPlugin::OnUpdate(const common::UpdateInfo&  /*_info*/) {
  // Get current time
#if GAZEBO_MAJOR_VERSION >= 9
  current_time_ = world_->SimTime();
#else
  current_time_ = world_->GetSimTime();
#endif
  // Creat msg and publish at the specified rate
  double dt = (current_time_ - last_time_).Double();
  if ( dt > interval_) {
    GetState();
    ComputeThrustCtrl();
    ComputeAttCtrl(dt);
    ComputeMotorCmd();
    Publish();
    last_time_ = current_time_;
  }
}

void QuadcopterAttitudeControllerPlugin::GetState(){
  // Get model pose
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3<double> state = model_->WorldPose();
#else
  ignition::math::Pose3<double> state = GazeboToIgnition(model_->GetWorldPose());
#endif
  // Get orientation
  ignition::math::Quaternion<double> q_flu_to_nwu = state.Rot();
  ignition::math::Quaternion<double> q_flu_to_enu = Q_NWU_TO_ENU*state.Rot();
  q_n_ = q_flu_to_enu;
  // linear Acceleration
#if GAZEBO_MAJOR_VERSION >= 9
  a_n_ = model_->RelativeLinearAccel();
#else
  a_n_ = GazeboToIgnition(model_->GetRelativeLinearAccel());
#endif
  // Angular velocity
  w1_n_1_ = w1_n_;
#if GAZEBO_MAJOR_VERSION >= 9
  w1_n_ = model_->RelativeAngularVel();
#else
  w1_n_ = GazeboToIgnition(model_->GetRelativeAngularVel());
#endif
}

void QuadcopterAttitudeControllerPlugin::ComputeThrustCtrl() {
  //Erreur
  double err_az = cmd_a_n_.Z();//- a_n_.Z();
  //Commande
  u_thrust_ = kp_thrust_*err_az;
}

void QuadcopterAttitudeControllerPlugin::ComputeAttCtrl(double dt){
  //Angles courants
  double yaw = q_n_.Yaw();
  double pit = q_n_.Pitch();
  double rol = q_n_.Roll();
  //Rotation exprimee dans le repère local
  ignition::math::Vector3<double> w0_n = q_n_.RotateVector(w1_n_);
  //Matrice de Transfert de rotation
  ignition::math::Matrix3<double> T = ignition::math::Matrix3<double>(
    cos(yaw)*tan(pit), sin(yaw)*tan(pit), 1.0,
            -sin(yaw),          cos(yaw), 0.0,
    cos(yaw)/cos(pit), sin(yaw)/cos(pit), 0.0);
  //Matrice inverse de Transfert de rotation
  ignition::math::Matrix3<double> Ti = ignition::math::Matrix3<double>(
    0.0, -sin(yaw), cos(pit)*cos(yaw),
    0.0,  cos(yaw), cos(pit)*sin(yaw),
    1.0,       0.0,         -sin(pit));
  //Derivée premiere des angles
  ignition::math::Vector3<double> dA = T*w0_n;
  double dyaw = dA.X();
  double dpit = dA.Y();
  double drol = dA.Z();
  // Matrices dérivée de Transfert de rotation
  ignition::math::Matrix3<double> dT = ignition::math::Matrix3<double>(
    (-dyaw*sin(yaw)*tan(pit) + dpit*cos(yaw)/(cos(pit)*cos(pit))), (dyaw*cos(yaw)*tan(pit) + dpit*sin(yaw)/(cos(pit)*cos(pit))), 0.0,
    (-dyaw*sin(yaw)),                                              (-dyaw*sin(yaw)),                                             0.0,
    (-dyaw*sin(yaw)/cos(pit) + dpit*tan(pit)*cos(yaw)/cos(pit)),   (dyaw*cos(yaw)/cos(pit) + dpit*tan(pit)*sin(yaw)/cos(pit)),   0.0);
  //Derivée premiere commande
  dcmd_q_n_ = cmd_q_n_*cmd_q_n_1_.Inverse();
  double dcmd_yaw_n = dcmd_q_n_.Yaw()/dt;
  double dcmd_pit_n = (cmd_q_n_.Pitch() - cmd_q_n_1_.Pitch())/dt;
  double dcmd_rol_n = (cmd_q_n_.Roll() - cmd_q_n_1_.Roll())/dt;
  cmd_q_n_1_ = ignition::math::Quaternion<double>(cmd_q_n_);
  //Dérivée seconde commande
  ignition::math::Quaternion<double> ddcmd_q_n = dcmd_q_n_*dcmd_q_n_1_.Inverse();
  double ddcmd_yaw_n = ddcmd_q_n.Yaw()/dt;
  double ddcmd_pit_n = (dcmd_q_n_.Pitch() - dcmd_q_n_1_.Pitch())/dt;
  double ddcmd_rol_n = (dcmd_q_n_.Roll() - dcmd_q_n_1_.Roll())/dt;
  dcmd_q_n_1_ = ignition::math::Quaternion<double>(dcmd_q_n_);
  //Erreur courante
  ignition::math::Quaternion<double> err_q_n = cmd_q_n_*q_n_.Inverse();
  double err_yaw_n = err_q_n.Yaw();
  double err_pit_n = cmd_q_n_.Pitch() - q_n_.Pitch() + trim_pit_;
  double err_rol_n = cmd_q_n_.Roll() - q_n_.Roll() + trim_rol_;
  //Erreur derivée premiere
  double derr_yaw_n = dcmd_yaw_n - dyaw;
  double derr_pit_n = dcmd_pit_n - dpit;
  double derr_rol_n = dcmd_rol_n - drol;
  //Dérivée seconde des angles
  double ddyaw = err_yaw_n + 2.0*derr_yaw_n + ddcmd_yaw_n;
  double ddpit = err_pit_n + 2.0*derr_pit_n + ddcmd_pit_n;
  double ddrol = err_rol_n + 2.0*derr_rol_n + ddcmd_rol_n;
  ignition::math::Vector3<double> ddA = ignition::math::Vector3<double>(ddyaw, ddpit, ddrol);
  //Dérivee premiere vitesse de rotation exprimée dans le repère local
  ignition::math::Vector3<double> dw0_n = Ti*(ddA - dT*w0_n);
  //Dérivee premiere vitesse de rotation exprimée dans le repère inertiel
  ignition::math::Vector3<double> dw1_n = q_n_.RotateVectorReverse(dw0_n);
  // Commandes
  u_yaw_rate_ = kp_yaw_rate_*dw1_n.Z();
  u_pit_rate_ = kp_pit_rate_*dw1_n.Y();
  u_rol_rate_ = kp_rol_rate_*dw1_n.X();
  // Message
  geometry_msgs::PointStamped msg;
  msg.header.stamp.sec = current_time_.sec;
  msg.header.stamp.nsec = current_time_.nsec;
  msg.point.x = q_n_.Yaw();
  msg.point.y = cmd_q_n_.Yaw();
  pub_yaw_.publish(msg);
  msg.point.x = q_n_.Pitch();
  msg.point.y = cmd_q_n_.Pitch();
  pub_pit_.publish(msg);
  msg.point.x = q_n_.Roll();
  msg.point.y = cmd_q_n_.Roll();
  pub_rol_.publish(msg);
}

void QuadcopterAttitudeControllerPlugin::ComputeMotorCmd(){
  //Forces moteur
  double f_front_left_rotor  = std::max((u_thrust_ - dir_front_left_rotor_ * u_yaw_rate_  - u_pit_rate_ + u_rol_rate_)/4.0, 0.0);
  double f_front_right_rotor = std::max((u_thrust_ - dir_front_right_rotor_* u_yaw_rate_  - u_pit_rate_ - u_rol_rate_)/4.0, 0.0);
  double f_rear_right_rotor  = std::max((u_thrust_ - dir_rear_right_rotor_ * u_yaw_rate_  + u_pit_rate_ - u_rol_rate_)/4.0, 0.0);
  double f_rear_left_rotor   = std::max((u_thrust_ - dir_rear_left_rotor_  * u_yaw_rate_  + u_pit_rate_ + u_rol_rate_)/4.0, 0.0);
  //Vitesses de rotations moteurs
  cmd_front_left_rotor_  = std::min(std::max(sqrt(f_front_left_rotor ), rotorspeed_min_), rotorspeed_max_);
  cmd_front_right_rotor_ = std::min(std::max(sqrt(f_front_right_rotor), rotorspeed_min_), rotorspeed_max_);
  cmd_rear_right_rotor_  = std::min(std::max(sqrt(f_rear_right_rotor ), rotorspeed_min_), rotorspeed_max_);
  cmd_rear_left_rotor_   = std::min(std::max(sqrt(f_rear_left_rotor  ), rotorspeed_min_), rotorspeed_max_);
}

void QuadcopterAttitudeControllerPlugin::CallbackCmd(const sensor_msgs::Imu::ConstPtr& _cmd_msg) {
  cmd_q_n_ = ignition::math::Quaternion<double>(
    _cmd_msg->orientation.w,
    _cmd_msg->orientation.x,
    _cmd_msg->orientation.y,
    _cmd_msg->orientation.z);
  cmd_a_n_ = ignition::math::Vector3<double>(
    _cmd_msg->linear_acceleration.x,
    _cmd_msg->linear_acceleration.y,
    _cmd_msg->linear_acceleration.z);
}

void QuadcopterAttitudeControllerPlugin::CallbackImu(const sensor_msgs::Imu::ConstPtr& _imu_msg) {

}


/////////////////////////////////////////////////
void QuadcopterAttitudeControllerPlugin::Publish(){
  msg_rotorspeed_.header.stamp.sec = current_time_.sec;
  msg_rotorspeed_.header.stamp.nsec = current_time_.nsec;
  msg_rotorspeed_.values.push_back(0);
  msg_rotorspeed_.values[id_front_right_rotor_] = cmd_front_right_rotor_;
  msg_rotorspeed_.values.push_back(0);
  msg_rotorspeed_.values[id_front_left_rotor_] = cmd_front_left_rotor_;
  msg_rotorspeed_.values.push_back(0);
  msg_rotorspeed_.values[id_rear_right_rotor_] = cmd_rear_right_rotor_;
  msg_rotorspeed_.values.push_back(0);
  msg_rotorspeed_.values[id_rear_left_rotor_] = cmd_rear_left_rotor_;
  pub_rotorspeed_.publish(msg_rotorspeed_);
}

}
