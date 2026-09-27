/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 * Copyright 2020 Vincent LE DOZE, INRIA, Chroma Team, CITI Lab Lyon, France
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

#include "gazebo_plugins/gazebo_plugin__body.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BodyPlugin);

/////////////////////////////////////////////////
BodyPlugin::BodyPlugin() : ModelPlugin() {
}

/////////////////////////////////////////////////
BodyPlugin::~BodyPlugin() {
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
void BodyPlugin::Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf) {
  // Get namespace
  getSdfParam<std::string>(_in_sdf, "robot_namespace", robot_namespace_, "");
  // Get base link
  std::string link_name;
  getSdfParam<std::string>(_in_sdf, "link_name", link_name, "");
  link_ = _in_model->GetLink(link_name);
  if (link_ == NULL)
    gzthrow("Couldn't find specified link \"" << link_name << "\".");
  // Get the home position
  getSdfParam<double>(_in_sdf, "lat_home", lat_home_, KDEFAULT_LAT_HOME);
  getSdfParam<double>(_in_sdf, "lon_home", lon_home_, KDEFAULT_LON_HOME);
  getSdfParam<double>(_in_sdf, "alt_home", alt_home_, KDEFAULT_ALT_HOME);
  // Get frame_id i.e. the body frame
  getSdfParam<std::string>(_in_sdf, "child_frame_id", child_frame_id_, "/" + robot_namespace_ + "/base_link");
  // Get parent_frame_id i.e. the frame in which the body pose is expressed
  getSdfParam<std::string>(_in_sdf, "frame_id", frame_id_, "world");
  // Gazebo connection
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(std::bind(&BodyPlugin::OnUpdate, this));
  #else
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&BodyPlugin::OnUpdate, this));
  #endif
  // Reset
  Reset();
  // Init state & time
  GetStateAndTime();
  // Ros handler
  node_handle_ = new ros::NodeHandle(robot_namespace_);
  // ROS groundtruth publishers
  gt_pub_ = node_handle_->advertise<chroma_gazebo_plugins_msgs::BodyGroundTruth>("/" + robot_namespace_ + "/groundtruth", 10);
  // ROS odom publishers
  getSdfParam<double>(_in_sdf, "odom_freq", gt_odom_freq_, KDEFAULT_GT_ODOM_FREQ);
  if (gt_odom_freq_ > 0.0) {
    #if GAZEBO_MAJOR_VERSION >= 9
    gt_odom_last_time_ = link_->GetWorld()->SimTime();
    #else
    gt_odom_last_time_ = link_->GetWorld()->GetSimTime();
    #endif
    gt_odom_pub_ = node_handle_->advertise<nav_msgs::Odometry>("/" + robot_namespace_ + "/groundtruth/odom", 10);
  }
}

//////////////////////////////////////////////////
void BodyPlugin::Reset() {
  link_->ResetPhysicsStates();
}

/////////////////////////////////////////////////
void BodyPlugin::OnUpdate() {
  // Get state & time
  GetStateAndTime();
  // send ground truth information
  SendGroundTruth();
}

////////////////////////////////////////////////
void BodyPlugin::GetStateAndTime() {
  // link not null
  GZ_ASSERT(link_, "Link was NULL");
  // state of body
  #if GAZEBO_MAJOR_VERSION >= 9
  state_ = link_->WorldPose();
  #else
  state_ = GazeboToIgnition(link_->GetWorldPose());
  #endif
  // Get current time
  #if GAZEBO_MAJOR_VERSION >= 9
  time_ = link_->GetWorld()->SimTime();
  #else
  time_ = link_->GetWorld()->GetSimTime();
  #endif
}

///////////////////////////////////////////////
void BodyPlugin::SendGroundTruth() {
  // Get model orientation in ENU frame
  ignition::math::Quaternion<double> q_flu_to_enu = Q_NWU_TO_ENU*state_.Rot();
  // get angular velocity
  #if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3<double> angular_velocity_flu = link_->RelativeAngularVel();
  #else
  ignition::math::Vector3<double> angular_velocity_flu = GazeboToIgnition(link_->GetRelativeAngularVel());
  #endif
  // Get model pose in ENU frame
  ignition::math::Vector3<double> pose_enu = Q_NWU_TO_ENU.RotateVector(state_.Pos());
  // Get model latitude / longitude
  ignition::math::Vector3<double> pose_gps = ENUtoGeodetic(
    pose_enu, lat_home_, lon_home_, alt_home_);
  // get linear velocity
  #if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3<double> linear_velocity_enu = Q_NWU_TO_ENU.RotateVector(link_->WorldLinearVel());
  ignition::math::Vector3<double> linear_velocity_flu = link_->RelativeLinearVel();
  #else
  ignition::math::Vector3<double> linear_velocity_enu = Q_NWU_TO_ENU.RotateVector(GazeboToIgnition(link_->GetWorldLinearVel()));
  ignition::math::Vector3<double> linear_velocity_flu = GazeboToIgnition(link_->GetRelativeLinearVel());
  #endif
  // get linear acceleration
  #if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3<double> linear_acceleration_enu = Q_NWU_TO_ENU.RotateVector(link_->WorldLinearAccel());
  ignition::math::Vector3<double> linear_acceleration_flu = link_->RelativeLinearAccel();
  #else
  ignition::math::Vector3<double> linear_acceleration_enu = Q_NWU_TO_ENU.RotateVector(GazeboToIgnition(link_->GetWorldLinearAccel()));
  ignition::math::Vector3<double> linear_acceleration_flu = GazeboToIgnition(link_->GetRelativeLinearAccel());
  #endif
  // fill Groundtruth msg at full rate
  chroma_gazebo_plugins_msgs::BodyGroundTruth msg_groundtruth;
  msg_groundtruth.header.stamp.sec = time_.sec;
  msg_groundtruth.header.stamp.nsec = time_.nsec;
  msg_groundtruth.header.frame_id = frame_id_;
  msg_groundtruth.child_frame_id = child_frame_id_;
  msg_groundtruth.x_enu = pose_enu.X();
  msg_groundtruth.y_enu = pose_enu.Y();
  msg_groundtruth.z_enu = pose_enu.Z();
  msg_groundtruth.lat = pose_gps.X();
  msg_groundtruth.lon = pose_gps.Y();
  msg_groundtruth.alt = pose_gps.Z();
  msg_groundtruth.vx_enu = linear_velocity_enu.X();
  msg_groundtruth.vy_enu = linear_velocity_enu.Y();
  msg_groundtruth.vz_enu = linear_velocity_enu.Z();
  msg_groundtruth.vx_flu = linear_velocity_flu.X();
  msg_groundtruth.vy_flu = linear_velocity_flu.Y();
  msg_groundtruth.vz_flu = linear_velocity_flu.Z();
  msg_groundtruth.ax_enu = linear_acceleration_enu.X();
  msg_groundtruth.ay_enu = linear_acceleration_enu.Y();
  msg_groundtruth.az_enu = linear_acceleration_enu.Z();
  msg_groundtruth.ax_flu = linear_acceleration_flu.X();
  msg_groundtruth.ay_flu = linear_acceleration_flu.Y();
  msg_groundtruth.az_flu = linear_acceleration_flu.Z();
  msg_groundtruth.airflow = state_.Rot().RotateVector(linear_velocity_enu).X();
  msg_groundtruth.wind = linear_velocity_enu.Length();
  msg_groundtruth.qx = q_flu_to_enu.X();
  msg_groundtruth.qy = q_flu_to_enu.Y();
  msg_groundtruth.qz = q_flu_to_enu.Z();
  msg_groundtruth.qw = q_flu_to_enu.W();
  msg_groundtruth.wx_flu = angular_velocity_flu.X();
  msg_groundtruth.wy_flu = angular_velocity_flu.Y();
  msg_groundtruth.wz_flu = angular_velocity_flu.Z();
  gt_pub_.publish(msg_groundtruth);
  // Groundtruth odom msg at given frequency
  if (gt_odom_freq_ > 0.0) {
    if ((time_ - gt_odom_last_time_).Double() >= (1.0/gt_odom_freq_)) {
      nav_msgs::Odometry msg_odom;
      msg_odom.header = msg_groundtruth.header;
      msg_odom.child_frame_id = msg_groundtruth.child_frame_id;
      msg_odom.pose.pose.position.x = pose_enu.X();
      msg_odom.pose.pose.position.y = pose_enu.Y();
      msg_odom.pose.pose.position.z = pose_enu.Z();
      msg_odom.pose.pose.orientation.x = q_flu_to_enu.X();
      msg_odom.pose.pose.orientation.y = q_flu_to_enu.Y();
      msg_odom.pose.pose.orientation.z = q_flu_to_enu.Z();
      msg_odom.pose.pose.orientation.w = q_flu_to_enu.W();
      msg_odom.twist.twist.linear.x = linear_velocity_flu.X();
      msg_odom.twist.twist.linear.y = linear_velocity_flu.Y();
      msg_odom.twist.twist.linear.z = linear_velocity_flu.Z();
      msg_odom.twist.twist.angular.x = angular_velocity_flu.X();
      msg_odom.twist.twist.angular.y = angular_velocity_flu.Y();
      msg_odom.twist.twist.angular.z = angular_velocity_flu.Z();
      gt_odom_pub_.publish(msg_odom);
      gt_odom_last_time_ = time_;
    }
  }
  // TF
  geometry_msgs::TransformStamped msg_tf;
  msg_tf.header = msg_groundtruth.header;
  msg_tf.child_frame_id = msg_groundtruth.child_frame_id;
  msg_tf.transform.translation.x = pose_enu.X();
  msg_tf.transform.translation.y = pose_enu.Y();
  msg_tf.transform.translation.z = pose_enu.Z();
  msg_tf.transform.rotation.x = q_flu_to_enu.X();
  msg_tf.transform.rotation.y = q_flu_to_enu.Y();
  msg_tf.transform.rotation.z = q_flu_to_enu.Z();
  msg_tf.transform.rotation.w = q_flu_to_enu.W();
  //send the transform
  tf_br_.sendTransform(msg_tf);
}
