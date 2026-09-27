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

#ifndef GAZEBO_PLUGINS_BAROMETER_H_
#define GAZEBO_PLUGINS_BAROMETER_H_

#include "gazebo_plugins/common.h"

#include <random>
#include <string>

#include <chroma_gazebo_plugins_msgs/Barometer.h>


namespace gazebo {

/// Constants
/// https://en.wikipedia.org/wiki/International_Standard_Atmosphere
static constexpr double ALTITUDE_TO_TEMPERATURE_RATE = -0.0065; // reduction in temperature with altitude (Kelvin/m)
static constexpr double TEMPERATURE_TO_PRESSURE_POW = 5.255;
static constexpr double PRESSURE_TO_TEMPERATURE_POW = 1.0/TEMPERATURE_TO_PRESSURE_POW;
static constexpr double GAS_CONSTANT = 8.3144598; //kg.m2.s−2.K−1.mol−1
static constexpr double MOLAR_MASS = 28.97e-3; //kg/mol
static constexpr double BASE_ALTITUDE = -611.0; // base geopotential altitude from Mean Sea Level (m)(lowest point on earth)
static constexpr double BASE_TEMPERATURE = 273.15 + 19.0; // temperature at base altitude (K)
static constexpr double MSL_TEMPERATURE = BASE_TEMPERATURE + ALTITUDE_TO_TEMPERATURE_RATE * (-BASE_ALTITUDE); // temperature at MSL (K)
static constexpr double MSL_PRESSURE = 101325; // pressure at MSL (Pa)

/// Default parameters
static const std::string KDEFAULT_NAMESPACE = "";
static const std::string KDEFAULT_LINK_NAME = "baro_link";
static const std::string KDEFAULT_TOPIC_BAROMETER = "barometer";
static const double KDEFAULT_ALT_HOME = 488.0;  // meters
static const double KDEFAULT_NOISE_DENSITY_PRESSURE = 0.001;
static const double KDEFAULT_FREQ = 50.0; // Hz

/// \brief A plugin that interface an FCU with Gazebo
class GAZEBO_VISIBLE BarometerPlugin : public ModelPlugin {

  /// Methods //////////////////////////////////////////////////////////
  public:
    BarometerPlugin();   /// \brief Constructor.
    ~BarometerPlugin();  /// \brief Destructor.

  protected:
    /// \brief Load model parameters
    void Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf);
    /// \brief Update
    void OnUpdate(const common::UpdateInfo&  /*_info*/);

  private:
    /// \brief
    void GetRealInformations();
    /// \brief
    void GetMeasurements();
    /// \brief
    void Publish();

  /// Attributes ///////////////////////////////////////////////////////
  private:
    /// Gazebo related /////////////////////////////////
    /// \brief Pointer to the world
    physics::WorldPtr world_;
    /// \brief Pointer to the model
    physics::ModelPtr model_;

    /// \brief Pointer to the update event connection.
    event::ConnectionPtr update_connection_ptr_;

    /// ROS related ////////////////////////////////////
    /// \brief node handler
    ros::NodeHandle* node_handle_;
    /// \brief publisher
    ros::Publisher pub_barometer_;
    /// \brief
    chroma_gazebo_plugins_msgs::Barometer msg_barometer_;

    /// Parameters /////////////////////////////////////
    /// \brief Robot namespace.
    std::string robot_namespace_;
    /// \brief Topic name to control the rotor speed
    std::string topic_barometer_;
    /// \brief Interval of time between each measurements
    double baro_interval_;
    /// \brief noise density
    double noise_density_pressure_;
    /// \brief
    double lat_home_;
    /// \brief
    double lon_home_;
    /// \brief
    double alt_home_;

    /// Others ///////////////////////////////////////
    /// \brief current time
    common::Time current_time_;
    /// \brief last time of measurment
    common::Time last_baro_time_;
    /// \brief time step
    double dt_;
    /// \brief
    double alt_real_;
    double temperature_real_;
    double density_real_;
    double pressure_real_;
    /// \brief
    double pressure_mes_;
    double diff_pressure_mes_;
    double alt_est_;
    double density_est_;
    double temperature_est_;

  };
}
#endif // GAZEBO_PLUGINS_BAROMETER_H_
