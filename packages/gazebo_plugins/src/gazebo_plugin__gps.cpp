/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 * Copyright (C) 2017-2018 PX4 Pro Development Team
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

#include <gazebo_plugins/gazebo_plugin__gps.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(GpsPlugin)

GpsPlugin::GpsPlugin() : ModelPlugin() {
}

GpsPlugin::~GpsPlugin() {
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

void GpsPlugin::Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf)
{
  // Store the pointer to the model.
  model_ = _in_model;
  // Initialise temporal variables
#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ =  model_->GetWorld()->SimTime();
  last_gps_time_ =  model_->GetWorld()->SimTime();
#else
  last_time_ =  model_->GetWorld()->GetSimTime();
  last_gps_time_ =  model_->GetWorld()->GetSimTime();
#endif
  // Get the robot namespace
  getSdfParam<std::string>(_in_sdf, "robot_namespace", robot_namespace_, KDEFAULT_NAMESPACE);
  // Get the home position
  getSdfParam<double>(_in_sdf, "lat_home", lat_home_, KDEFAULT_LAT_HOME);
  getSdfParam<double>(_in_sdf, "lon_home", lon_home_, KDEFAULT_LON_HOME);
  getSdfParam<double>(_in_sdf, "alt_home", alt_home_, KDEFAULT_ALT_HOME);
  // Get noise param
  getSdfParam<bool>(_in_sdf, "gps_noise", gps_noise_, KDEFAULT_NOISE);
  // Gazebo binding
  // Listen to the update event. This event is broadcast every simulation iteration.
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(std::bind(&GpsPlugin::OnUpdate, this, std::placeholders::_1));
  #else
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GpsPlugin::OnUpdate, this, _1));
  #endif
  // Initialize ROS Node
  node_handle_ = new ros::NodeHandle(robot_namespace_);
  gps_pub_ = node_handle_->advertise<chroma_gazebo_plugins_msgs::Gps>("/" + robot_namespace_ + "/gps", 10);
}

/// On Update //////////////////////////////////////////////
void GpsPlugin::OnUpdate(const common::UpdateInfo&){
  // Get current time
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time =  model_->GetWorld()->SimTime();
#else
  common::Time current_time =  model_->GetWorld()->GetSimTime();
#endif
  // Get time step
  double dt = (current_time - last_time_).Double();
  // Get model state
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3<double> state = model_->WorldPose();
#else
  ignition::math::Pose3<double> state = GazeboToIgnition(model_->GetWorldPose());
#endif
  // Get model pose in ENU frame
  ignition::math::Vector3<double> pos_enu = Q_NWU_TO_ENU.RotateVector(state.Pos());
  // Get model velocity in ENU frame.
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3<double> vel_enu = Q_NWU_TO_ENU.RotateVector(model_->WorldLinearVel());
#else
  ignition::math::Vector3<double> vel_enu = Q_NWU_TO_ENU.RotateVector(GazeboToIgnition(model_->GetWorldLinearVel()));
#endif
  // Velocity on ground plan
  ignition::math::Vector3<double> vel_en = vel_enu;
  vel_en.Z() = 0;
  // update noise parameters if gps_noise_ is set
  ignition::math::Vector3<double> noise_pos_enu;
  ignition::math::Vector3<double> noise_vel_enu;
  ignition::math::Vector3<double> random_walk_pos_enu;
  if (gps_noise_) {
    // Pose noise
    noise_pos_enu.X() = GPS_XY_NOISE_DENSITY * sqrt(dt) * randn_(rand_);
    noise_pos_enu.Y() = GPS_XY_NOISE_DENSITY * sqrt(dt) * randn_(rand_);
    noise_pos_enu.Z() = GPS_Z_NOISE_DENSITY * sqrt(dt) * randn_(rand_);
    // Pose random walk
    random_walk_pos_enu.X() = GPS_XY_RANDOM_WALK * sqrt(dt) * randn_(rand_);
    random_walk_pos_enu.Y() = GPS_XY_RANDOM_WALK * sqrt(dt) * randn_(rand_);
    random_walk_pos_enu.Z() = GPS_Z_RANDOM_WALK * sqrt(dt) * randn_(rand_);
    // Velocity noise
    noise_vel_enu.X() = GPS_VXY_NOISE_DENSITY * sqrt(dt) * randn_(rand_);
    noise_vel_enu.Y() = GPS_VXY_NOISE_DENSITY * sqrt(dt) * randn_(rand_);
    noise_vel_enu.Z() = GPS_VZ_NOISE_DENSITY * sqrt(dt) * randn_(rand_);
  }
  else {
    // Pose noise
    noise_pos_enu.X() = 0.0;
    noise_pos_enu.Y() = 0.0;
    noise_pos_enu.Z() = 0.0;
    // Pose random walk
    random_walk_pos_enu.X() = 0.0;
    random_walk_pos_enu.Y() = 0.0;
    random_walk_pos_enu.Z() = 0.0;
    //Velocity noise
    noise_vel_enu.X() = 0.0;
    noise_vel_enu.Y() = 0.0;
    noise_vel_enu.Z() = 0.0;
  }
  // gps bias integration
  bias_pos_enu_.X() += random_walk_pos_enu.X() * dt - bias_pos_enu_.X() / GPS_CORELLATION_TIME;
  bias_pos_enu_.Y() += random_walk_pos_enu.Y() * dt - bias_pos_enu_.Y() / GPS_CORELLATION_TIME;
  bias_pos_enu_.Z() += random_walk_pos_enu.Z() * dt - bias_pos_enu_.Z() / GPS_CORELLATION_TIME;
  // Reproject ENU position with noise into geographic coordinates
  ignition::math::Vector3<double> pose_gps = ENUtoGeodetic(
    pos_enu + noise_pos_enu + bias_pos_enu_,
    lat_home_, lon_home_, alt_home_);
  // fill SITLGps msg
  chroma_gazebo_plugins_msgs::Gps gps_msg;
  gps_msg.header.stamp.sec = current_time.sec;
  gps_msg.header.stamp.nsec = current_time.nsec;
  gps_msg.lat = pose_gps.X();
  gps_msg.lon = pose_gps.Y();
  gps_msg.alt = pose_gps.Z();
  // gps_msg.eph = 65535.0 / 100.0;
  // gps_msg.epv = 65535.0 / 100.0;
  gps_msg.eph = 1.0;
  gps_msg.epv = 1.0;
  gps_msg.vel = vel_en.Length();
  gps_msg.vel_east  = (vel_enu.X() + noise_vel_enu.X());
  gps_msg.vel_north = (vel_enu.Y() + noise_vel_enu.Y());
  gps_msg.vel_up    = (vel_enu.Z() + noise_vel_enu.Z());
  if (vel_en.Length() > 1.0) {
    gps_msg.cog = GetDegrees360(-atan2(vel_enu.Y(), vel_enu.X()));
  }
  else {
    gps_msg.cog = 65535.0 / 100.0;
  }
  // add msg to buffer
  gps_delay_buffer_.push(gps_msg);
  // apply GPS delay
  if ((current_time - last_gps_time_).Double() > GPS_UPDATE_INTERVAL) {
    last_gps_time_ = current_time;
    while (true) {
      // abort if buffer is empty already
      if (gps_delay_buffer_.empty()) break;
      // remove data if buffer too large
      if (gps_delay_buffer_.size() > GPS_BUFFER_SIZE_MAX) {
        gps_delay_buffer_.pop();
      }
      else {
        // get oldest msg
        gps_msg = gps_delay_buffer_.front();
        // remove data that is too old
        if ((current_time.Double() - (gps_msg.header.stamp.sec + gps_msg.header.stamp.nsec*1.0e6)) > GPS_DELAY) {
          gps_delay_buffer_.pop();
        }
        else {
          gps_delay_buffer_.pop();
          break;  // if we get here, we have good data, stop
        }
      }
    }
    // publish msg at 5hz
    gps_pub_.publish(gps_msg);
  }
  // save time
  last_time_ = current_time;
}
} // namespace gazebo
