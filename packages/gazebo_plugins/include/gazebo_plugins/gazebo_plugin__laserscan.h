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

#ifndef GAZEBO_ROS_BLOCK_LASER_HH
#define GAZEBO_ROS_BLOCK_LASER_HH

// Custom Callback Queue
#include "gazebo_plugins/common.h"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/plugins/RayPlugin.hh>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace gazebo
{

  class LaserscanPlugin : public RayPlugin
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: LaserscanPlugin();

    /// \brief Destructor
    public: ~LaserscanPlugin();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void OnNewLaserScans();

    /// \brief Put laser data to the ROS topic
    private: void PutLaserData(common::Time &_updateTime);

    private: common::Time last_update_time_;

    /// \brief Keep track of number of connctions
    private: int laser_connect_count_;
    private: void LaserConnect();
    private: void LaserDisconnect();

    // Pointer to the model
    private: physics::WorldPtr world_;
    /// \brief The parent sensor
    private: sensors::SensorPtr parent_sensor_;
    private: sensors::RaySensorPtr parent_ray_sensor_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief Measurement resolution
    private: double resolution_;

    /// \brief Gaussian noise
    private: double gaussian_noise_;
    private: double range_noise_slope_;

    /// \brief Interpoles laser rays
    private: bool interpolation_;

    /// \brief Gaussian noise generator
    private: double GaussianKernel(double mu,double sigma);

    /// \brief A mutex to lock access to fields that are used in message callbacks
    private: boost::mutex lock;

    /// update rate of this sensor
    private: double update_rate_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    // Custom Callback Queue
    private: ros::CallbackQueue laser_queue_;
    private: void LaserQueueThread();
    private: boost::thread callback_laser_queue_thread_;

    // subscribe to world stats
    private: transport::NodePtr node_;
    private: common::Time sim_time_;
    public: void OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg);

  };

}

#endif
