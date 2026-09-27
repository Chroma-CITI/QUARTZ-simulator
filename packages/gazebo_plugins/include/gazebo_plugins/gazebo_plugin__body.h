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

#ifndef GAZEBO_PLUGINS_BODY_H_
#define GAZEBO_PLUGINS_BODY_H_

#include "gazebo_plugins/common.h"

#include <string>

#include <chroma_gazebo_plugins_msgs/BodyGroundTruth.h>
#include <nav_msgs/Odometry.h>

#include "tf/transform_broadcaster.h"

namespace gazebo
{

static const double KDEFAULT_LAT_HOME = 47.397742;  // deg
static const double KDEFAULT_LON_HOME = 8.545594;   // deg
static const double KDEFAULT_ALT_HOME = 488.0;      // meters

static const double KDEFAULT_GT_ODOM_FREQ = -1.0;

/// \brief A plugin that simulate aerodynamics and give Ground truth
class GAZEBO_VISIBLE BodyPlugin : public ModelPlugin {

  /// Methods ////////////////////////////////////////////////////////
  public:
    BodyPlugin();  /// \brief Constructor.
    ~BodyPlugin(); /// \brief Destructor.

  protected:
    /// \brief Load model
    virtual void Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf);
    /// \brief Reset Function
    virtual void Reset();
    /// \brief Callback for World Update events.
    virtual void OnUpdate();
    /// \brief Send Ground Truth
    void GetStateAndTime();
    /// \brief Send Ground Truth
    void SendGroundTruth();

  /// Attributes /////////////////////////////////////////////////////
  protected:
    /// Gazebo related ///////////////////////////////////////////////
    /// \brief Pointer to link associated to the plugin
    physics::LinkPtr link_;
    /// \brief Current model state
    ignition::math::Pose3<double> state_;
    /// \brief Current time
    common::Time time_;

    /// \brief Connection to World Update events.
    event::ConnectionPtr update_connection_ptr_;

    /// ROS related //////////////////////////////////////////////////
    /// \brief ROS node handler
    ros::NodeHandle* node_handle_;
    /// \brief ROS publisher : Ground Truth
    ros::Publisher gt_pub_;
    ros::Publisher gt_odom_pub_;
    common::Time gt_odom_last_time_;
    double gt_odom_freq_;
    // \brief transform publisher
    tf::TransformBroadcaster tf_br_;

    /// Parameters ///////////////////////////////////////////////////
    /// \brief Robot namespace.
    std::string robot_namespace_;
    /// \brief Frame id of the body frame
    std::string child_frame_id_;
    /// \brief Frame in which the body pose is expressed
    std::string frame_id_;
    /// \brief Set global reference point
    double lat_home_;
    double lon_home_;
    double alt_home_;
};

}
#endif
