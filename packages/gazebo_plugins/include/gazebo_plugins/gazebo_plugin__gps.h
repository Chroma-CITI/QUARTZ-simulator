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


#ifndef GAZEBO_PLUGINS_GPS_H_
#define GAZEBO_PLUGINS_GPS_H_

#include "gazebo_plugins/common.h"

#include <cstdio>
#include <cstdlib>
#include <queue>
#include <random>

#include <chroma_gazebo_plugins_msgs/Gps.h>


#define GPS_UPDATE_INTERVAL     0.2     // 5hz
#define GPS_DELAY              0.12     // 120 ms
#define GPS_BUFFER_SIZE_MAX    1000

#define GPS_CORELLATION_TIME   60.0    // s
#define GPS_XY_RANDOM_WALK      2.0    // (m/s) / sqrt(hz)
#define GPS_Z_RANDOM_WALK       4.0    // (m/s) / sqrt(hz)
#define GPS_XY_NOISE_DENSITY   2e-4    // (m) / sqrt(hz)
#define GPS_Z_NOISE_DENSITY    4e-4    // (m) / sqrt(hz)
#define GPS_VXY_NOISE_DENSITY  2e-1    // (m/s) / sqrt(hz)
#define GPS_VZ_NOISE_DENSITY   4e-1    // (m/s) / sqrt(hz)

namespace gazebo
{
static const std::string KDEFAULT_NAMESPACE = "";
static const double KDEFAULT_LAT_HOME = 47.397742;  // deg
static const double KDEFAULT_LON_HOME = 8.545594;   // deg
static const double KDEFAULT_ALT_HOME = 488.0;      // meters
static const bool KDEFAULT_NOISE = false;   // Noise on measurements

/// \brief A plugin that simulate a GPS sensor
class GAZEBO_VISIBLE GpsPlugin : public ModelPlugin
{
  /// Methods ////////////////////////////////////////////////////////
  public:
    GpsPlugin();  /// \brief Constructor.
    ~GpsPlugin(); /// \brief Destructor.

  protected:
    /// \brief Load model parameters
    virtual void Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf);
    /// \brief Update
    virtual void OnUpdate(const common::UpdateInfo&);

  /// Attributes /////////////////////////////////////////////////////
  private:
    /// Gazebo related ///////////////////////////////
    /// \brief Pointer to the model
    physics::ModelPtr model_;

    /// \brief
    event::ConnectionPtr update_connection_ptr_;

    /// ROS related //////////////////////////////////
    /// \brief Pointer to Node handler
    ros::NodeHandle* node_handle_;
    /// \brief GPS msg publisher
    ros::Publisher gps_pub_;

    /// Parameters ///////////////////////////////////
    /// \brief robot namespace
    std::string robot_namespace_;
    /// \brief Set global reference point
    double lat_home_;
    double lon_home_;
    double alt_home_;
    /// \brief Boolean to add noise in GPS measurement
    bool gps_noise_;

    /// GPS parameters ///////////////////////////////
    /// \brief
    std::queue<chroma_gazebo_plugins_msgs::Gps> gps_delay_buffer_;
    /// \brief
    ignition::math::Vector3<double> bias_pos_enu_;

    // Noises generator //////////////////////////////
    /// \brief
    std::default_random_engine rand_;
    /// \brief
    std::normal_distribution<float> randn_;

    /// Others ///////////////////////////////////////
    /// \brief
    common::Time last_gps_time_;
    /// \brief
    common::Time last_time_;

  };// class GAZEBO_VISIBLE GpsPlugin
}// namespace gazebo

#endif // GAZEBO_PLUGINS_GPS_H_
