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

#ifndef GAZEBO_PLUGINS_IMU_H_
#define GAZEBO_PLUGINS_IMU_H_

#include "gazebo_plugins/common.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <random>

#include <sensor_msgs/Imu.h>


namespace gazebo {

// Default values for use with ADIS16448 IMU
static constexpr double KDEFAULT_FREQ = 100.0;
static constexpr double KDEFAULT_GYROSCOPE_NOISE_DENSITY = 2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
static constexpr double KDEFAULT_GYROSCOPE_RANDOM_WALK = 2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
static constexpr double KDEFAULT_GYROSCOPE_BIAS_CORRELATION_TIME = 1.0e+3;
static constexpr double KDEFAULT_GYROSCOPE_TURN_ON_BIAS_SIGMA = 0.5 / 180.0 * M_PI;
static constexpr double KDEFAULT_ACCELEROMETER_NOISE_DENSITY = 2.0 * 2.0e-3;
static constexpr double KDEFAULT_ACCELEROMETER_RANDOM_WALK = 2.0 * 3.0e-3;
static constexpr double KDEFAULT_ACCELEROMETER_BIAS_CORRELATION_TIME = 300.0;
static constexpr double KDEFAULT_ACCELEROMETER_TURN_ON_BIAS_SIGMA = 20.0e-3 * 9.8;
static constexpr double KDEFAULT_GRAVITY_MAGNITUDE = 9.8068;

static const std::string KDEFAULT_IMU_TOPIC = "imu";

/// \brief A plugin that simulate an IMU
class GAZEBO_VISIBLE ImuPlugin : public ModelPlugin {

  /// Methods //////////////////////////////////////////////////////////
  public:
    ImuPlugin();  /// \brief Constructor.
    ~ImuPlugin(); /// \brief Destructor.

  protected:
    /// \brief Load model parameters
    virtual void Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf);
    /// \brief Update
    virtual void OnUpdate(const common::UpdateInfo&);
    /// \brief Add noises in IMU measurements
    virtual void AddNoise(const double dt);
    /// \brief compute orientation
    virtual void ComputeInclination();


  /// Attributes ///////////////////////////////////////////////////////
  private:
    /// Gazebo related /////////////.x////////////////////
    /// \brief Pointer to the world
    physics::WorldPtr world_;
    /// \brief Pointer to the model
    physics::ModelPtr model_;
    /// \brief Pointer to the link
    physics::LinkPtr link_;

    /// \brief Pointer to the update event connection
    event::ConnectionPtr update_connection_ptr_;

    /// ROS related ////////////////////////////////////
    /// \brief node handler
    ros::NodeHandle* node_handle_;
    /// \brief imu publisher
    ros::Publisher imu_pub_;
    /// \brief imu topic
    std::string imu_topic_;
    /// \brief imu message
    sensor_msgs::Imu imu_msg_;

    /// Parameters /////////////////////////////////////
    /// \brief Robot namespace.
    std::string robot_namespace_;
    /// \brief Frame name in which the IMU measurement are relatives to
    std::string link_name_;
    /// \brief Sensor frequency
    double freq_;
    /// \brief
    bool first_sample_;

    /// IMU description ////////////////////////////////
    /// https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model-and-Intrinsics
    /// \brief Gyroscope noise density (two-sided spectrum) [rad/s/sqrt(Hz)]
    double gyroscope_noise_density_;
    /// \brief Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
    double gyroscope_random_walk_;
    /// \brief Gyroscope bias correlation time constant [s]
    double gyroscope_bias_correlation_time_;
    /// \brief Gyroscope turn on bias standard deviation [rad/s]
    double gyroscope_turn_on_bias_sigma_;
    /// \brief Accelerometer noise density (two-sided spectrum) [m/s^2/sqrt(Hz)]
    double accelerometer_noise_density_;
    /// \brief Accelerometer bias random walk. [m/s^2/s/sqrt(Hz)]
    double accelerometer_random_walk_;
    /// \brief Accelerometer bias correlation time constant [s]
    double accelerometer_bias_correlation_time_;
    /// \brief Accelerometer turn on bias standard deviation [m/s^2]
    double accelerometer_turn_on_bias_sigma_;
    /// \brief
    ignition::math::Vector3<double> gyroscope_bias_;
    /// \brief
    ignition::math::Vector3<double> accelerometer_bias_;
    /// \brief
    ignition::math::Vector3<double> gyroscope_turn_on_bias_;
    /// \brief
    ignition::math::Vector3<double> accelerometer_turn_on_bias_;

    /// IMU measures ////////////////////////////////////////
    ignition::math::Vector3<double> linear_acceleration_;
    ignition::math::Vector3<double> dir_gravity_;
    ignition::math::Vector3<double> angular_velocity_;
    /// \brief Initial orientation of the IMU (Inverse)
    ignition::math::Quaternion<double> q_imu_;

    /// Random generation /////////////////////////////
    /// \brief
    std::default_random_engine random_generator_;
    /// \brief
    std::normal_distribution<double> standard_normal_distribution_;

    /// Others ///////////////////////////////////////
    /// \brief
    common::Time last_time_;
    /// \brief
    ignition::math::Vector3<double> gravity_;
};
}

#endif // GAZEBO_PLUGINS_IMU_H_
