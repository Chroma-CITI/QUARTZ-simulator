/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS_UWB_H_
#define GAZEBO_PLUGINS_UWB_H_

#include "gazebo_plugins/common.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <random>
#include <regex>
#include <thread>

#include "chroma_gazebo_plugins_msgs/Uwb.h"
#include "chroma_gazebo_plugins_msgs/UwbStatus.h"
#include "chroma_gazebo_plugins_msgs/UwbToken.h"
#include "nav_msgs/Odometry.h"

//#define DEBUG


namespace gazebo {

// Default values for parameters
static const double KDEFAULT_FREQ = 10.0;
static const double KDEFAULT_NOISE_DENSITY = 0.0;
static const double KDEFAULT_RX_GAIN = 2.5;
static const double KDEFAULT_RX_POWER = -1000.0;
static const double KDEFAULT_TX_GAIN = 2.5;
static const double KDEFAULT_TX_POWER = 1;
static const double KDEFAULT_TX_FREQ = 3.5; // in GHz, UWB communication frequency (2.4 for WIFI)
static const double KDEFAULT_STDDEV_PER_METERS = 0.015;
static const std::string KDEFAULT_ESSID = "uwb-unknown";
static const std::string KDEFAULT_UWB_TOPIC = "uwb";
static const std::string KDEFAULT_ODOM_TOPIC = "odom";

// Default values for propagation
static constexpr double NEMPTY = 2.0;
static constexpr double NOBSTACLE = 4.0;
static constexpr double MODELSTDDESV = 6.0;
static constexpr double DEFAULT_D0 = 6.0; // Base Distance to compute PL0 (Free Space Path Loss Model)

/// \brief A plugin that simulate an IMU
class GAZEBO_VISIBLE UwbPlugin : public ModelPlugin {

  /// Methods //////////////////////////////////////////////////////////
  public:
    UwbPlugin();  /// \brief Constructor.
    ~UwbPlugin(); /// \brief Destructor.

    // Inherited
    virtual void Reset();
    virtual void Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf);
    virtual void OnUpdate();

    // PathLoss Model Choice

    double PathLossComputation(double measured_distance,double d_obst);

    // UWB measurement
    void GetMesurements(
        const ignition::math::Vector3<double>& emitter_pos,
        const ignition::math::Vector3<double>& receivr_pos,
        std::string &friend_name,
              double& receivd_power,
              double& measured_distance);

  private:
    /// ROS Topic Callback
    void CallbackOdom(const nav_msgs::Odometry::ConstPtr& _odom_msg);
    /// ROS Service Callback
    bool CallbackStatus(
      chroma_gazebo_plugins_msgs::UwbStatus::Request  &req,
      chroma_gazebo_plugins_msgs::UwbStatus::Response &res);
    bool CallbackGetToken(
      chroma_gazebo_plugins_msgs::UwbToken::Request  &req,
      chroma_gazebo_plugins_msgs::UwbToken::Response &res);

  /// Attributes ///////////////////////////////////////////////////////
  private:
    /// Gazebo related /////////////////////////////////
    physics::WorldPtr world_;
    physics::LinkPtr link_;
    event::ConnectionPtr update_connection_ptr_;

    /// ROS related ////////////////////////////////////
    ros::NodeHandle*   node_handle_;
    ros::Publisher     uwb_pub_;
    ros::Subscriber    odom_sub_;
    ros::ServiceServer status_srv_;
    ros::ServiceServer token_srv_;
    nav_msgs::Odometry::ConstPtr odom_msg_;

    /// Parameters /////////////////////////////////////
    std::string robot_namespace_;
    std::string uwb_topic_;
    std::string odom_topic_;
    std::string link_name_;
    std::string essid_;
    double freq_;
    double rx_gain_;
    double rx_power_;
    double tx_gain_;
    double tx_power_;
    double tx_freq_;
    double stddev_per_meter_;
    double stddev_per_obstacle_;

    /// Random generation /////////////////////////////
    std::default_random_engine random_generator_;
    std::normal_distribution<double> standard_normal_distribution_;

    /// Communication management /////////////////////
    bool has_comm_token_;
    bool has_next_token_owner_;
    std::string next_token_owner_;
    std::vector<std::string> previous_comms_;
    std::vector<std::string> previous_token_users_;
    std::vector<std::string> friend_list_names_;
    physics::Link_V friend_list_links_;
    std::vector<std::string> neighbor_list;
    int count;
    bool loaded_once;
    int global_count;

    /// Others ///////////////////////////////////////
    physics::RayShapePtr test_ray_;
    ignition::math::Vector3<double> link_pos_;
    common::Time last_time_;
};

}

#endif // GAZEBO_PLUGINS_UWB_H_
