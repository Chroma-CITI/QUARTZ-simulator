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

#ifndef GAZEBO_PLUGINS_MAGNETOMETER_H_
#define GAZEBO_PLUGINS_MAGNETOMETER_H_

#include "gazebo_plugins/common.h"

#include <random>
#include <string>

#include <sensor_msgs/MagneticField.h>


/** set this always to the sampling in degrees for the table below */
#define SAMPLING_RES		10.0f
#define SAMPLING_MIN_LAT	-60.0f
#define SAMPLING_MAX_LAT	60.0f
#define SAMPLING_MIN_LON	-180.0f
#define SAMPLING_MAX_LON	180.0f
#define EARTH_RADIUS      6353000.0     // meters

namespace gazebo {

static const int8_t DECLINATION_TABLE[13][37] = \
{
	{ 46, 45, 44, 42, 41, 40, 38, 36, 33, 28, 23, 16, 10, 4, -1, -5, -9, -14, -19, -26, -33, -40, -48, -55, -61, -66, -71, -74, -75, -72, -61, -25, 22, 40, 45, 47, 46 },
	{ 30, 30, 30, 30, 29, 29, 29, 29, 27, 24, 18, 11, 3, -3, -9, -12, -15, -17, -21, -26, -32, -39, -45, -51, -55, -57, -56, -53, -44, -31, -14, 0, 13, 21, 26, 29, 30 },
	{ 21, 22, 22, 22, 22, 22, 22, 22, 21, 18, 13, 5, -3, -11, -17, -20, -21, -22, -23, -25, -29, -35, -40, -44, -45, -44, -40, -32, -22, -12, -3, 3, 9, 14, 18, 20, 21 },
	{ 16, 17, 17, 17, 17, 17, 16, 16, 16, 13, 8, 0, -9, -16, -21, -24, -25, -25, -23, -20, -21, -24, -28, -31, -31, -29, -24, -17, -9, -3, 0, 4, 7, 10, 13, 15, 16 },
	{ 12, 13, 13, 13, 13, 13, 12, 12, 11, 9, 3, -4, -12, -19, -23, -24, -24, -22, -17, -12, -9, -10, -13, -17, -18, -16, -13, -8, -3, 0, 1, 3, 6, 8, 10, 12, 12 },
	{ 10, 10, 10, 10, 10, 10, 10, 9, 9, 6, 0, -6, -14, -20, -22, -22, -19, -15, -10, -6, -2, -2, -4, -7, -8, -8, -7, -4, 0, 1, 1, 2, 4, 6, 8, 10, 10 },
	{ 9, 9, 9, 9, 9, 9, 8, 8, 7, 4, -1, -8, -15, -19, -20, -18, -14, -9, -5, -2, 0, 1, 0, -2, -3, -4, -3, -2, 0, 0, 0, 1, 3, 5, 7, 8, 9 },
	{ 8, 8, 8, 9, 9, 9, 8, 8, 6, 2, -3, -9, -15, -18, -17, -14, -10, -6, -2, 0, 1, 2, 2, 0, -1, -1, -2, -1, 0, 0, 0, 0, 1, 3, 5, 7, 8 },
	{ 8, 9, 9, 10, 10, 10, 10, 8, 5, 0, -5, -11, -15, -16, -15, -12, -8, -4, -1, 0, 2, 3, 2, 1, 0, 0, 0, 0, 0, -1, -2, -2, -1, 0, 3, 6, 8 },
	{ 6, 9, 10, 11, 12, 12, 11, 9, 5, 0, -7, -12, -15, -15, -13, -10, -7, -3, 0, 1, 2, 3, 3, 3, 2, 1, 0, 0, -1, -3, -4, -5, -5, -2, 0, 3, 6 },
	{ 5, 8, 11, 13, 15, 15, 14, 11, 5, -1, -9, -14, -17, -16, -14, -11, -7, -3, 0, 1, 3, 4, 5, 5, 5, 4, 3, 1, -1, -4, -7, -8, -8, -6, -2, 1, 5 },
	{ 4, 8, 12, 15, 17, 18, 16, 12, 5, -3, -12, -18, -20, -19, -16, -13, -8, -4, -1, 1, 4, 6, 8, 9, 9, 9, 7, 3, -1, -6, -10, -12, -11, -9, -5, 0, 4 },
	{ 3, 9, 14, 17, 20, 21, 19, 14, 4, -8, -19, -25, -26, -25, -21, -17, -12, -7, -2, 1, 5, 9, 13, 15, 16, 16, 13, 7, 0, -7, -12, -15, -14, -11, -6, -1, 3 },
};

/// Default parameters
static const std::string KDEFAULT_NAMESPACE = "";
static const std::string KDEFAULT_LINK_NAME = "mag_link";
static const std::string KDEFAULT_TOPIC_MAGNETOMETER = "magnetometer";
static const double KDEFAULT_NOISE_DENSITY = 0.001;
static const double KDEFAULT_FREQ = 50.0; // Hz
static const double KDEFAULT_LAT_HOME = 47.397742;  // rad
static const double KDEFAULT_LON_HOME = 8.545594;   // rad
static const double KDEFAULT_ALT_HOME = 488.0;      // meters

/// \brief A plugin that interface an FCU with Gazebo
class GAZEBO_VISIBLE MagnetometerPlugin : public ModelPlugin {

  /// Methods //////////////////////////////////////////////////////////
  public:
    MagnetometerPlugin();   /// \brief Constructor.
    ~MagnetometerPlugin();  /// \brief Destructor.

  protected:
    /// \brief Load model parameters
    void Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf);
    /// \brief Update
    void OnUpdate(const common::UpdateInfo&  /*_info*/);

  private:
    /// \brief
    void CalculateState();
    /// \brief
    void CalculateDeclination();
    /// \brief
    void CalculateMagneticField();
    /// \brief Publish ROS topics
    void Publish();
    /// \brief
    float GetValFromLookupTable(unsigned lat, unsigned lon);

  /// Attributes ///////////////////////////////////////////////////////
  private:
    /// Gazebo related /////////////////////////////////
    /// \brief Pointer to the world
    physics::WorldPtr world_;
    /// \brief Pointer to the model
    physics::ModelPtr model_;

    /// \brief Pointer to the update event connection.
    event::ConnectionPtr update_connection_ptr_;

    /// \brief
    ignition::math::Vector3<double> magfield_ned_;

    /// ROS related ////////////////////////////////////
    /// \brief node handler
    ros::NodeHandle* node_handle_;
    /// \brief publisher
    ros::Publisher pub_magnetometer_;
    /// \brief
    sensor_msgs::MagneticField msg_magnetometer_;

    /// Parameters /////////////////////////////////////
    /// \brief Robot namespace.
    std::string robot_namespace_;
    /// \brief Topic name to control the rotor speed
    std::string topic_magnetometer_;
    /// \brief Interval of time between each measurements
    double mag_interval_;
    /// \brief noise density
    double noise_density_;
    /// \brief
    double lat_home_;
    /// \brief
    double lon_home_;
    /// \brief
    double alt_home_;

    /// Random generator //////////////////////////////
    /// \brief
    std::default_random_engine rand_;
    /// \brief
    std::normal_distribution<float> randn_;

    /// Others ///////////////////////////////////////
    /// \brief current time
    common::Time current_time_;
    /// \brief last time of measurment
    common::Time last_mag_time_;
    /// \brief time step
    double dt_;
    /// \brief Latitude in deg
    double lat_deg_;
    /// \brief Longitude in deg
    double lon_deg_;
    /// \brief Magnetic declination
    double declination_rad_;
    /// \brief Magnetic field
    ignition::math::Vector3<double> magfield_flu_;
    /// \brief Magnetometer orientation
    ignition::math::Quaternion<double> q_ned_to_flu_;

  };
}
#endif // GAZEBO_PLUGINS_MAGNETOMETER_H_
