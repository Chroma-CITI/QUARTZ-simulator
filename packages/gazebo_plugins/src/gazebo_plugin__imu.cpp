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

#include "gazebo_plugins/gazebo_plugin__imu.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ImuPlugin);

/////////////////////////////////////////////////
ImuPlugin::ImuPlugin() : ModelPlugin() {
}

/////////////////////////////////////////////////
ImuPlugin::~ImuPlugin() {
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
void ImuPlugin::Load(physics::ModelPtr _in__model, sdf::ElementPtr _in__sdf) {
  // Store the pointer to the model
  model_ = _in__model;
  // Get the world
  world_ = model_->GetWorld();
  #if GAZEBO_MAJOR_VERSION >= 8
  gravity_ = world_->Gravity();
  #else
  gravity_ = GazeboToIgnition(world_->GetPhysicsEngine()->GetGravity());
  #endif
  // Get the robot namespace
  getSdfParam<std::string>(_in__sdf, "robot_namespace", robot_namespace_, "");
  // Get link name
  getSdfParam<std::string>(_in__sdf, "link_name", link_name_, "");
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("Couldn't find specified link \"" << link_name_ << "\".");
  // Get frequency
  getSdfParam<double>(_in__sdf, "freq", freq_, KDEFAULT_FREQ);
  // Get Gyroscope parameters
  getSdfParam<double>(_in__sdf, "gyroscope_noise_density", gyroscope_noise_density_, KDEFAULT_GYROSCOPE_NOISE_DENSITY);
  getSdfParam<double>(_in__sdf, "gyroscope_random_walk", gyroscope_random_walk_, KDEFAULT_GYROSCOPE_RANDOM_WALK);
  getSdfParam<double>(_in__sdf, "gyroscope_bias_correlation_time", gyroscope_bias_correlation_time_, KDEFAULT_GYROSCOPE_BIAS_CORRELATION_TIME);
  assert(gyroscope_bias_correlation_time_ > 0.0);
  getSdfParam<double>(_in__sdf, "gyroscope_turn_on_bias_sigma", gyroscope_turn_on_bias_sigma_, KDEFAULT_GYROSCOPE_TURN_ON_BIAS_SIGMA);
  // Get Accelerometer parameters
  getSdfParam<double>(_in__sdf, "accelerometer_noise_density", accelerometer_noise_density_, KDEFAULT_ACCELEROMETER_NOISE_DENSITY);
  getSdfParam<double>(_in__sdf, "accelerometer_random_walk",accelerometer_random_walk_, KDEFAULT_ACCELEROMETER_RANDOM_WALK);
  getSdfParam<double>(_in__sdf, "accelerometer_bias_correlation_time", accelerometer_bias_correlation_time_, KDEFAULT_ACCELEROMETER_BIAS_CORRELATION_TIME);
  assert(accelerometer_bias_correlation_time_ > 0.0);
  getSdfParam<double>(_in__sdf, "accelerometer_turn_on_bias_sigma", accelerometer_turn_on_bias_sigma_, KDEFAULT_ACCELEROMETER_TURN_ON_BIAS_SIGMA);
  // Get the imu topic name
  getSdfParam<std::string>(_in__sdf, "imu_topic", imu_topic_, KDEFAULT_IMU_TOPIC);
  // Initialisation of the time reference
  #if GAZEBO_MAJOR_VERSION >= 9
    last_time_ = world_->SimTime();
  #else
    last_time_ = world_->GetSimTime();
  #endif
  // Gazebo connection
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(std::bind(&ImuPlugin::OnUpdate, this, std::placeholders::_1));
  #else
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ImuPlugin::OnUpdate, this, _1));
  #endif
  // ROS connection
  node_handle_ = new ros::NodeHandle(robot_namespace_);
  imu_pub_ = node_handle_->advertise<sensor_msgs::Imu>("/" + robot_namespace_ + "/" + imu_topic_, 1);
  // Fill imu message.
  imu_msg_.header.frame_id = "/" + robot_namespace_ + "/" + link_name_;
  // We assume uncorrelated noise on the 3 channels -> only set diagonal
  // elements. Only the broadband noise component is considered, specified as a
  // continuous-time density (two-sided spectrum); not the true covariance of
  // the measurements.
  // Angular velocity measurement covariance.
  imu_msg_.angular_velocity_covariance[0] = gyroscope_noise_density_ * gyroscope_noise_density_;
  imu_msg_.angular_velocity_covariance[4] = gyroscope_noise_density_ * gyroscope_noise_density_;
  imu_msg_.angular_velocity_covariance[8] = gyroscope_noise_density_ * gyroscope_noise_density_;
  // Linear acceleration measurement covariance.
  imu_msg_.linear_acceleration_covariance[0] = accelerometer_noise_density_ * accelerometer_noise_density_;
  imu_msg_.linear_acceleration_covariance[4] = accelerometer_noise_density_ * accelerometer_noise_density_;
  imu_msg_.linear_acceleration_covariance[8] = accelerometer_noise_density_ * accelerometer_noise_density_;
  // Orientation estimate covariance (no estimate provided).
  imu_msg_.orientation_covariance[0] = -1.0;
  // Turn on bias
  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
  gyroscope_turn_on_bias_ = ignition::math::Vector3<double>(
    gyroscope_turn_on_bias_sigma_ * standard_normal_distribution_(random_generator_),
    gyroscope_turn_on_bias_sigma_ * standard_normal_distribution_(random_generator_),
    gyroscope_turn_on_bias_sigma_ * standard_normal_distribution_(random_generator_));
  accelerometer_turn_on_bias_ = ignition::math::Vector3<double>(
    accelerometer_turn_on_bias_sigma_ * standard_normal_distribution_(random_generator_),
    accelerometer_turn_on_bias_sigma_ * standard_normal_distribution_(random_generator_),
    accelerometer_turn_on_bias_sigma_ * standard_normal_distribution_(random_generator_));
  // TODO(nikolicj) incorporate steady-state covariance of bias process
  gyroscope_bias_ = ignition::math::Vector3<double>(0.0, 0.0, 0.0);
  accelerometer_bias_ = ignition::math::Vector3<double>(0.0, 0.0, 0.0);
  //
  first_sample_ = true;
}

/////////////////////////////////////////////////
/// \brief This function adds noise to acceleration and angular rates for
///        accelerometer and gyroscope measurement simulation.
void ImuPlugin::AddNoise(const double dt) {
  // If no data, we stop
  if (dt <= 0.0)
    return;
  // Gyroscope ============================================
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_g_d = 1 / sqrt(dt) * gyroscope_noise_density_;
  double sigma_b_g = gyroscope_random_walk_;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_g_d =
      sqrt( - sigma_b_g * sigma_b_g * gyroscope_bias_correlation_time_ / 2.0 *
      (exp(-2.0 * dt / gyroscope_bias_correlation_time_) - 1.0));
  // Compute state-transition.
  double phi_g_d = exp(-1.0 / gyroscope_bias_correlation_time_ * dt);
  // Simulate gyroscope noise processes and add them to the true angular rate.
  ignition::math::Vector3<double> gyroscope_bias_noise(
    sigma_b_g_d * standard_normal_distribution_(random_generator_),
    sigma_b_g_d * standard_normal_distribution_(random_generator_),
    sigma_b_g_d * standard_normal_distribution_(random_generator_));
  gyroscope_bias_ = phi_g_d * gyroscope_bias_ + gyroscope_bias_noise;
  ignition::math::Vector3<double> gyroscope_noise(
    sigma_g_d * standard_normal_distribution_(random_generator_),
    sigma_g_d * standard_normal_distribution_(random_generator_),
    sigma_g_d * standard_normal_distribution_(random_generator_));
  angular_velocity_ = angular_velocity_ + gyroscope_noise + gyroscope_bias_ + gyroscope_turn_on_bias_;
  // Accelerometer ============================================
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_a_d = 1 / sqrt(dt) * accelerometer_noise_density_;
  double sigma_b_a = accelerometer_random_walk_;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_a_d =
      sqrt( - sigma_b_a * sigma_b_a * accelerometer_bias_correlation_time_ / 2.0 *
      (exp(-2.0 * dt / accelerometer_bias_correlation_time_) - 1.0));
  // Compute state-transition.
  double phi_a_d = exp(-1.0 / accelerometer_bias_correlation_time_ * dt);
  // Simulate accelerometer noise processes and add them to the true linear
  // acceleration.
  ignition::math::Vector3<double> accelerometer_bias_noise(
    sigma_b_a_d * standard_normal_distribution_(random_generator_),
    sigma_b_a_d * standard_normal_distribution_(random_generator_),
    sigma_b_a_d * standard_normal_distribution_(random_generator_));
  accelerometer_bias_ = phi_a_d * accelerometer_bias_ + accelerometer_bias_noise;
  ignition::math::Vector3<double> accelerometer_noise(
    sigma_a_d * standard_normal_distribution_(random_generator_),
    sigma_a_d * standard_normal_distribution_(random_generator_),
    sigma_a_d * standard_normal_distribution_(random_generator_));
  linear_acceleration_ = linear_acceleration_ + accelerometer_noise + accelerometer_bias_ + accelerometer_turn_on_bias_;
}

/////////////////////////////////////////////////
// \brief This gets called by the world update start event.
void ImuPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get current time
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif
  // time step
  double dt = (current_time - last_time_).Double();
  // Frequency condition
  if (dt >= 1.0/freq_) {
    // Get state
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3<double> state = link_->WorldPose();
#else
    ignition::math::Pose3<double> state = GazeboToIgnition(link_->GetWorldPose());
#endif
    // Get orientation
    ignition::math::Quaternion<double> q_imu_to_nwu = state.Rot();
    ignition::math::Quaternion<double> q_imu_to_enu = Q_NWU_TO_ENU*state.Rot();
    ignition::math::Quaternion<double> q_enu_to_imu = q_imu_to_enu.Inverse();
    // linear Acceleration
#if GAZEBO_MAJOR_VERSION >= 9
    linear_acceleration_ = link_->RelativeLinearAccel();
#else
    linear_acceleration_ = GazeboToIgnition(link_->GetRelativeLinearAccel());
#endif
    linear_acceleration_ += q_enu_to_imu.RotateVector(-gravity_); // without gravity
    // Angular velocity
#if GAZEBO_MAJOR_VERSION >= 9
    angular_velocity_ = link_->RelativeAngularVel();
#else
    angular_velocity_ = GazeboToIgnition(link_->GetRelativeAngularVel());
#endif
    // Add noises
    AddNoise(dt);
    // Compute Orientation
    ComputeInclination();
    // Fill IMU message.
    imu_msg_.header.stamp.sec = current_time.sec;
    imu_msg_.header.stamp.nsec = current_time.nsec;
    imu_msg_.orientation.w = q_imu_.W();
    imu_msg_.orientation.x = q_imu_.X();
    imu_msg_.orientation.y = q_imu_.Y();
    imu_msg_.orientation.z = q_imu_.Z();
    imu_msg_.linear_acceleration.x = linear_acceleration_.X();
    imu_msg_.linear_acceleration.y = linear_acceleration_.Y();
    imu_msg_.linear_acceleration.z = linear_acceleration_.Z();
    imu_msg_.angular_velocity.x = angular_velocity_.X();
    imu_msg_.angular_velocity.y = angular_velocity_.Y();
    imu_msg_.angular_velocity.z = angular_velocity_.Z();
    // Publish IMU message.
    imu_pub_.publish(imu_msg_);
    // Save values
    last_time_ = current_time;
  }
}

void ImuPlugin::ComputeInclination(){
  // Variables locales
  double gyro_trust_factor = 9.0;
  //Direction of Inertial Acceleration
  ignition::math::Vector3<double> dir_acc = linear_acceleration_.Normalized();
  //Angles of the gravity vector with respect to its initial direction
  ignition::math::Vector2<double> ang_gravity;
  //Corrected Angular velocity vector
  ignition::math::Vector3<double> gyro_corr;
  // premiere mesure
  if (first_sample_) {
    // initialize with accelerometer readings
    dir_gravity_ = dir_acc;
  }
  else {
    //evaluate gyro_corr vector
    if (fabs(dir_gravity_.Z()) < 0.1) {
      //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
      //in this case skip the gyro data and just use previous estimate
      gyro_corr = dir_gravity_;
    }
    else {
      //get angles between projection of R on ZX/ZY plane and Z axis, based on last dir_gravity_
      ang_gravity.X() = atan2(dir_gravity_.X(), dir_gravity_.Z());    //get angle and convert to degrees
      ang_gravity.X() += angular_velocity_.X() / freq_;               //get updated angle according to gyro movement in deg/s
      ang_gravity.Y() = atan2(dir_gravity_.Y(), dir_gravity_.Z());    //get angle and convert to degrees
      ang_gravity.Y() += angular_velocity_.Y() / freq_;               //get updated angle according to gyro movement in deg/s
      //estimate sign of RzGyro by looking in what qudrant the angle Axz is,
      //RzGyro is pozitive if  Axz in range -90 ..90 => cos(ang_gravity) >= 0
      int sign_z_gyro = (cos(ang_gravity.X()) >= 0) ? 1 : -1;
      //reverse calculation of gyro_corr from ang_gravity angles,
      //for formulas deductions see  http://starlino.com/imu_guide.html
      gyro_corr.X() = sin(ang_gravity.X());
      gyro_corr.X() /= sqrt(1 + sqr(cos(ang_gravity.X())) * sqr(tan(ang_gravity.Y())));
      gyro_corr.Y() = sin(ang_gravity.Y());
      gyro_corr.Y() /= sqrt(1 + sqr(cos(ang_gravity.Y())) * sqr(tan(ang_gravity.X())));
      gyro_corr.Z() = sign_z_gyro * sqrt(1 - sqr(gyro_corr.X()) - sqr(gyro_corr.Y()));
    }
    //combine Accelerometer and gyro readings
    dir_gravity_ =
        (dir_acc + gyro_trust_factor * gyro_corr)
        / (1 + gyro_trust_factor);
    //Normalizing the estimates
    dir_gravity_ = dir_gravity_.Normalize();
  }
  first_sample_ = false;
  //Computing the angles
  double imu_phi = dir_gravity_.X() * M_PI/2.0;
  double imu_tet = dir_gravity_.Y() * M_PI/2.0;
  double imu_psi = dir_gravity_.Z() * M_PI/2.0;
  //Computing the orientation
  q_imu_ = ignition::math::Quaternion<double>(imu_phi, imu_tet, imu_psi);
}
