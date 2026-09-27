/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2018 PX4 Pro Development Team
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

#include "gazebo_plugins/gazebo_plugin__magnetometer.h"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(MagnetometerPlugin);

/////////////////////////////////////////////////
MagnetometerPlugin::MagnetometerPlugin() : ModelPlugin() {
  // Magnetic field data for Zurich from WMM2015 (10^5xnanoTesla (N, E D) n-frame )
  // mag_n_ = {0.21523, 0.00771, -0.42741};
  // We set the world Y component to zero because we apply
  // the declination based on the global position,
  // and so we need to start without any offsets.
  // The real value for Zurich would be 0.00771
  magfield_ned_ = ignition::math::Vector3<double>(0.21523, 0.00771, 0.42741);
}

/////////////////////////////////////////////////
MagnetometerPlugin::~MagnetometerPlugin() {
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
void MagnetometerPlugin::Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf) {
  // Store the pointer to the model.
  model_ = _in_model;
  // Store the pointer to the world
  world_ = model_->GetWorld();
  // Initialise temporal variables
#if GAZEBO_MAJOR_VERSION >= 9
  current_time_ = world_->SimTime();
  last_mag_time_ = world_->SimTime();
#else
  current_time_ = world_->GetSimTime();
  last_mag_time_ = world_->GetSimTime();
#endif
  dt_ = 0.0;
  // Get namespace param
  getSdfParam<std::string>(_in_sdf, "robot_namespace", robot_namespace_, KDEFAULT_NAMESPACE);
  // Get link name
  std::string link_name;
  getSdfParam<std::string>(_in_sdf, "link_name", link_name, KDEFAULT_LINK_NAME);
  // Get ros topic names
  getSdfParam<std::string>(_in_sdf, "topic_name", topic_magnetometer_, KDEFAULT_TOPIC_MAGNETOMETER);
  // Get noise density
  getSdfParam<double>(_in_sdf, "noise_density", noise_density_, KDEFAULT_NOISE_DENSITY);
  // Get the home position
  getSdfParam<double>(_in_sdf, "lat_home", lat_home_, KDEFAULT_LAT_HOME);
  getSdfParam<double>(_in_sdf, "lon_home", lon_home_, KDEFAULT_LON_HOME);
  getSdfParam<double>(_in_sdf, "alt_home", alt_home_, KDEFAULT_ALT_HOME);
  // Get magnetometer frequencyignition::
  double freq;
  getSdfParam<double>(_in_sdf, "freq", freq, KDEFAULT_FREQ);
  mag_interval_ = 1.0/freq;
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(std::bind(&MagnetometerPlugin::OnUpdate, this, std::placeholders::_1));
  #else
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&MagnetometerPlugin::OnUpdate, this, _1));
  #endif
  // ROS initialisation
  node_handle_ = new ros::NodeHandle(robot_namespace_);
  // ROS Publisher
  pub_magnetometer_ = node_handle_->advertise<sensor_msgs::MagneticField>("/" + robot_namespace_ + "/" + topic_magnetometer_, 1);
  // Prepare msg
  msg_magnetometer_.header.frame_id = "/" + robot_namespace_ + "/" + link_name;
  msg_magnetometer_.magnetic_field_covariance[0] = noise_density_ * noise_density_;
  msg_magnetometer_.magnetic_field_covariance[4] = noise_density_ * noise_density_;
  msg_magnetometer_.magnetic_field_covariance[8] = noise_density_ * noise_density_;
}

/////////////////////////////////////////////////
// This gets called by the world update start event.
void MagnetometerPlugin::OnUpdate(const common::UpdateInfo&  /*_info*/) {
  // Get current time
#if GAZEBO_MAJOR_VERSION >= 9
  current_time_ = world_->SimTime();
#else
  current_time_ = world_->GetSimTime();
#endif
  dt_ = (current_time_ - last_mag_time_).Double();
  // Creat msg and publish at the specified rate
  if ( dt_ > mag_interval_) {
    CalculateState();
    CalculateDeclination();
    CalculateMagneticField();
    Publish();
    last_mag_time_ = current_time_;
  }
}

void MagnetometerPlugin::CalculateState(){
  // Get model pose
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3<double> state = model_->WorldPose();
#else
  ignition::math::Pose3<double> state = GazeboToIgnition(model_->GetWorldPose());
#endif
  // Magnetometer position
  ignition::math::Vector3<double> pose_enu = Q_NWU_TO_ENU.RotateVector(state.Pos());
  // Reproject ENU position without noise into geographic coordinates
  ignition::math::Vector3<double> pose_gps = ENUtoGeodetic(
    pose_enu, lat_home_, lon_home_, alt_home_);
  lat_deg_ = pose_gps.X();
  lon_deg_ = pose_gps.Y();
  // Magnetometer attitude
  q_ned_to_flu_ = state.Rot().Inverse()*Q_ENU_TO_NWU*Q_NED_TO_ENU;
}

/////////////////////////////////////////////////
void MagnetometerPlugin::CalculateDeclination() {/*
	 * If the values exceed valid ranges, return zero as default
	 * as we have no way of knowing what the closest real value
	 * would be.
	 */
	if (lat_deg_ < -90.0 || lat_deg_ > 90.0 ||
	    lon_deg_ < -180.0 || lon_deg_ > 180.0) {
    declination_rad_ = 0.0;
    return;
	}
	/* round down to nearest sampling resolution */
	int min_lat = (int)(lat_deg_ / SAMPLING_RES) * SAMPLING_RES;
	int min_lon = (int)(lon_deg_ / SAMPLING_RES) * SAMPLING_RES;
	/* for the rare case of hitting the bounds exactly
	 * the rounding logic wouldn't fit, so enforce it.
	 */
	/* limit to table bounds - required for maxima even when table spans full globe range */
	if (lat_deg_ <= SAMPLING_MIN_LAT) {
		min_lat = SAMPLING_MIN_LAT;
	}
	if (lat_deg_ >= SAMPLING_MAX_LAT) {
		min_lat = (int)(lat_deg_ / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES;
	}
	if (lon_deg_ <= SAMPLING_MIN_LON) {
		min_lon = SAMPLING_MIN_LON;
	}
	if (lon_deg_ >= SAMPLING_MAX_LON) {
		min_lon = (int)(lon_deg_ / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES;
	}
	/* find index of nearest low sampling point */
	unsigned min_lat_index = (-(SAMPLING_MIN_LAT) + min_lat)  / SAMPLING_RES;
	unsigned min_lon_index = (-(SAMPLING_MIN_LON) + min_lon) / SAMPLING_RES;
  // Get declination in lookup table
	float declination_sw = GetValFromLookupTable(min_lat_index, min_lon_index);
	float declination_se = GetValFromLookupTable(min_lat_index, min_lon_index + 1);
	float declination_ne = GetValFromLookupTable(min_lat_index + 1, min_lon_index + 1);
	float declination_nw = GetValFromLookupTable(min_lat_index + 1, min_lon_index);
	/* perform bilinear interpolation on the four grid corners */
	float declination_min = ((lon_deg_ - min_lon) / SAMPLING_RES) * (declination_se - declination_sw) + declination_sw;
	float declination_max = ((lon_deg_ - min_lon) / SAMPLING_RES) * (declination_ne - declination_nw) + declination_nw;
	float declination_deg = ((lat_deg_ - min_lat) / SAMPLING_RES) * (declination_max - declination_min) + declination_min;
  // result
	declination_rad_ = declination_deg * D2R;
}

/////////////////////////////////////////////////
void MagnetometerPlugin::CalculateMagneticField(){
  // Quaternion of the declination
  ignition::math::Quaternion<double> q_declination(0.0, 0.0, declination_rad_);
  // Magnetic field in FLU frame
  magfield_flu_ = q_ned_to_flu_.RotateVector(q_declination.RotateVector(magfield_ned_));
  // Noise on Magnetic field in FLU frame
  double sigma = 1 / sqrt(dt_) * noise_density_;
  ignition::math::Vector3<double> mag_noise_flu_(
    sigma * randn_(rand_),
    sigma * randn_(rand_),
    sigma * randn_(rand_));
  // Magnetic field in FLU frame
  magfield_flu_ += mag_noise_flu_;
}

/////////////////////////////////////////////////
void MagnetometerPlugin::Publish(){
  msg_magnetometer_.header.stamp.sec = current_time_.sec;
  msg_magnetometer_.header.stamp.nsec = current_time_.nsec;
  msg_magnetometer_.magnetic_field.x = magfield_flu_.X();
  msg_magnetometer_.magnetic_field.y = magfield_flu_.Y();
  msg_magnetometer_.magnetic_field.z = magfield_flu_.Z();
  pub_magnetometer_.publish(msg_magnetometer_);
}

/////////////////////////////////////////////////
float MagnetometerPlugin::GetValFromLookupTable(unsigned lat_index, unsigned lon_index) {
	return DECLINATION_TABLE[lat_index][lon_index];
}

}
