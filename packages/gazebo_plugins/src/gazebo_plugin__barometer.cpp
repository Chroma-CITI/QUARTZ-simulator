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

#include <gazebo_plugins/gazebo_plugin__barometer.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(BarometerPlugin);

/////////////////////////////////////////////////
BarometerPlugin::BarometerPlugin() : ModelPlugin() {
}

/////////////////////////////////////////////////
BarometerPlugin::~BarometerPlugin() {
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
void BarometerPlugin::Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf) {
  // Store the pointer to the model.
  model_ = _in_model;
  // Store the pointer to the world
  world_ = model_->GetWorld();
  // Initialise temporal variables
#if GAZEBO_MAJOR_VERSION >= 9
  current_time_ = world_->SimTime();
  last_baro_time_ = world_->SimTime();
#else
  current_time_ = world_->GetSimTime();
  last_baro_time_ = world_->GetSimTime();
#endif
  dt_ = 0.0;
  // Get namespace param
  getSdfParam<std::string>(_in_sdf, "robot_namespace", robot_namespace_, KDEFAULT_NAMESPACE);
  // Get link name
  std::string link_name;
  getSdfParam<std::string>(_in_sdf, "link_name", link_name, KDEFAULT_LINK_NAME);
  // Get the home position
  getSdfParam<double>(_in_sdf, "alt_home", alt_home_, KDEFAULT_ALT_HOME);
  // Get ros topic names
  getSdfParam<std::string>(_in_sdf, "topic_name", topic_barometer_, KDEFAULT_TOPIC_BAROMETER);
  // Get noise density
  getSdfParam<double>(_in_sdf, "noise_density", noise_density_pressure_, KDEFAULT_NOISE_DENSITY_PRESSURE);
  // Get barometer frequencyignition::
  double freq;
  getSdfParam<double>(_in_sdf, "freq", freq, KDEFAULT_FREQ);
  baro_interval_ = 1.0/freq;
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(std::bind(&BarometerPlugin::OnUpdate, this, std::placeholders::_1));
  #else
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&BarometerPlugin::OnUpdate, this, _1));
  #endif
  // ROS initialisation
  node_handle_ = new ros::NodeHandle(robot_namespace_);
  // ROS Publisher
  pub_barometer_ = node_handle_->advertise<chroma_gazebo_plugins_msgs::Barometer>("/" + robot_namespace_ + "/" + topic_barometer_, 1);
  // Prepare msg
  msg_barometer_.header.frame_id = "/" + robot_namespace_ + "/" + link_name;
  msg_barometer_.pressure_variance = noise_density_pressure_ * noise_density_pressure_;
}

/////////////////////////////////////////////////
// This gets called by the world update start event.
void BarometerPlugin::OnUpdate(const common::UpdateInfo&  /*_info*/) {
  // Get current time
#if GAZEBO_MAJOR_VERSION >= 9
  current_time_ = world_->SimTime();
#else
  current_time_ = world_->GetSimTime();
#endif
  // Creat msg and publish at the specified rate
  dt_ = (current_time_ - last_baro_time_).Double();
  if ( dt_ > baro_interval_) {
    GetRealInformations();
    GetMeasurements();
    Publish();
    last_baro_time_ = current_time_;
  }
}

void BarometerPlugin::GetRealInformations(){
  // Get model pose
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3<double> state = model_->WorldPose();
#else
  ignition::math::Pose3<double> state = GazeboToIgnition(model_->GetWorldPose());
#endif
  // barometer altitude
  alt_real_ = alt_home_ + state.Pos().Z();
  // calculate abs_pressure using an ISA model for the tropsphere (valid up to 11km above MSL)
  temperature_real_ = BASE_TEMPERATURE + ALTITUDE_TO_TEMPERATURE_RATE * (alt_real_ - BASE_ALTITUDE);
  // Real pressure
  pressure_real_ = MSL_PRESSURE * pow((temperature_real_/MSL_TEMPERATURE) , TEMPERATURE_TO_PRESSURE_POW);
}

/////////////////////////////////////////////////
void BarometerPlugin::GetMeasurements(){
    // generate Gaussian noise sequence using polar form of Box-Muller transformation
    // http://www.design.caltech.edu/erik/Misc/Gaussian.html
    double x1, x2, w;
    do {
     x1 = 2.0 * (rand() * (1.0 / (double)RAND_MAX)) - 1.0;
     x2 = 2.0 * (rand() * (1.0 / (double)RAND_MAX)) - 1.0;
     w = x1 * x1 + x2 * x2;
    } while ( w >= 1.0 );
    w = sqrt( (-2.0 * log( w ) ) / w );
    // standard deviation
    double sigma = 1.0 / sqrt(dt_) * noise_density_pressure_;
    // Pressure measured
    pressure_mes_ = pressure_real_ + sigma * w;
    // Diff pressure
    diff_pressure_mes_ = pressure_mes_ - pressure_real_;
    // Temperature measured
    temperature_est_ = MSL_TEMPERATURE * pow(( pressure_mes_ / MSL_PRESSURE ), PRESSURE_TO_TEMPERATURE_POW);
    // Altitude measured
    alt_est_ = (temperature_est_ - MSL_TEMPERATURE)/ALTITUDE_TO_TEMPERATURE_RATE;
    // Density measured
    density_est_ = (MOLAR_MASS*pressure_mes_) / (GAS_CONSTANT*temperature_est_);
}

/////////////////////////////////////////////////
void BarometerPlugin::Publish(){
  msg_barometer_.header.stamp.sec = current_time_.sec;
  msg_barometer_.header.stamp.nsec = current_time_.nsec;
  msg_barometer_.pressure = pressure_mes_ * 0.01; // convert to hPa
  msg_barometer_.diff_pressure = diff_pressure_mes_ * 0.01; // convert to hPa
  msg_barometer_.temperature = temperature_est_;
  msg_barometer_.density = density_est_;
  msg_barometer_.altitude = alt_est_;
  pub_barometer_.publish(msg_barometer_);
}


}
