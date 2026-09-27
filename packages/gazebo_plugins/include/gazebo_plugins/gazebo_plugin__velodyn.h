/*
 * Copyright 2013 Open Source Robotics Foundation
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


#ifndef GAZEBO_PLUGINS_VELODYN_H_
#define GAZEBO_PLUGINS_VELODYN_H_

#include "gazebo_plugins/common.h"

#include <ros/callback_queue.h>

#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


namespace gazebo {
// Default values
static const std::string KDEFAULT_NAMESPACE = "";
static const std::string KDEFAULT_TOPIC_LASER_SCAN = "laser_scan";
static const std::string KDEFAULT_TOPIC_VELODYN_SCAN = "velodyn_scan";
static const std::string KDEFAULT_FRAME_VELODYN_SCAN = "base_link";

static constexpr double KDEFAULT_UPDATE_RATE = 10.0; // Hz

/// \brief A plugin that simulate an the rotor physics
class GAZEBO_VISIBLE VelodynModel : public ModelPlugin {

  /// Methods //////////////////////////////////////////////////////////
  public:
    VelodynModel();   /// \brief Constructor.
    ~VelodynModel();  /// \brief Destructor.

    /// \brief Publish ROS msgs
    virtual void Publish();

  protected:
    /// \brief Load model parameters
    virtual void Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf);
    /// \brief Load model parameters
    virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

  private:
    /// \brief
    void CallbackVelocityCmd(const std_msgs::Float64& msg_velocity_cmd);
    void CallbackLaserScan(const sensor_msgs::PointCloud2::ConstPtr& msg_laser_scan);

  /// Attributes ///////////////////////////////////////////////////////
  protected:
    /// Gazebo related ///////////////////////////////////////////////
    physics::WorldPtr world_;
    physics::ModelPtr model_;
    physics::JointPtr joint_;
    physics::LinkPtr link_;
    physics::LinkPtr base_link_;

    /// \brief Pointer to the update event connection.
    event::ConnectionPtr update_connection_ptr_;

    /// ROS related //////////////////////////////////////////////////
    ros::NodeHandle* node_handle_;
    ros::Publisher   pub_velodyn_scan_;
    ros::Subscriber  sub_velocity_cmd_;
    ros::Subscriber  sub_laser_scan_;

    /// SDF parameters ///////////////////////////////////////////////
    /// \brief
    std::string namespace_;
    /// \brief
    std::string link_name_;
    std::string base_link_name_;
    /// \brief
    std::string joint_name_;
    /// \brief topic name for the rotor speed control
    std::string topic_laser_scan_;
    std::string topic_velodyn_scan_;
    std::string frame_velodyn_scan_;

    /// Update Frequency /////////////////////////////////////////////
    double update_dt_;

    /// Time reference ///////////////////////////////////////////////
    common::Time last_time_;

    /// Outputs //////////////////////////////////////////////////////
    std::vector<pcl::PointCloud<pcl::PointXYZ>> pc_laser_scans_wf_;
    bool ok_thread_;
};

}
#endif // GAZEBO_PLUGINS_VELODYN_H_
