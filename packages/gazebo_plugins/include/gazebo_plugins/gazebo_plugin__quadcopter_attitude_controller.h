// Author : vledoze
// Date   : 20/02/2019

/*
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

#ifndef GAZEBO_PLUGINS_MAVLINK_INTERFACE_H_
#define GAZEBO_PLUGINS_MAVLINK_INTERFACE_H_

#include "gazebo_plugins/common.h"

#include <random>
#include <string>

#include <chroma_gazebo_plugins_msgs/Rotorspeed.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>

namespace gazebo {

/// Default parameters
static const std::string KDEFAULT_NAMESPACE = "";
static const std::string KDEFAULT_LINK_NAME = "base_link";
static const std::string KDEFAULT_TOPIC_ROTORSPEED = "rotorspeed";
static const std::string KDEFAULT_TOPIC_CMD = "cmd";
static const std::string KDEFAULT_TOPIC_IMU = "imu";
static const int KDEFAULT_ID_FRONT_RIGHT_ROTOR = 0;
static const int KDEFAULT_ID_FRONT_LEFT_ROTOR = 1;
static const int KDEFAULT_ID_REAR_RIGHT_ROTOR = 2;
static const int KDEFAULT_ID_REAR_LEFT_ROTOR = 3;
static const int KDEFAULT_DIR_FRONT_RIGHT_ROTOR = -1;
static const int KDEFAULT_DIR_FRONT_LEFT_ROTOR = 1;
static const int KDEFAULT_DIR_REAR_RIGHT_ROTOR = -1;
static const int KDEFAULT_DIR_REAR_LEFT_ROTOR = 1;
static const double KDEFAULT_ROTORSPEED_MAX = 1.0;
static const double KDEFAULT_KP_YAW_RATE = 1.0;
static const double KDEFAULT_KP_ROL_RATE = 1.0;
static const double KDEFAULT_KP_PIT_RATE = 1.0;
static const double KDEFAULT_KP_THRUST = 1.0;
static const double KDEFAULT_FREQ = 50.0; // Hz

/// \brief A plugin that interface an FCU with Gazebo
class GAZEBO_VISIBLE QuadcopterAttitudeControllerPlugin : public ModelPlugin {

  /// Types ////////////////////////////////////////////////////////////
  private:
    typedef typename Eigen::Matrix<double, 3, 3> Matrix3;
    typedef typename Eigen::Matrix<double, 3, 1> Vector3;

  /// Methods //////////////////////////////////////////////////////////
  public:
    QuadcopterAttitudeControllerPlugin();   /// \brief Constructor.
    ~QuadcopterAttitudeControllerPlugin();  /// \brief Destructor.

  protected:
    /// \brief Load model parameters
    void Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf);
    /// \brief Update
    void OnUpdate(const common::UpdateInfo&  /*_info*/);

  private:
    /// \brief ROS Callback on IMU Topic
    void CallbackCmd(const sensor_msgs::Imu::ConstPtr& _cmd_msg);
    void CallbackImu(const sensor_msgs::Imu::ConstPtr& _imu_msg);
    /// \brief
    void GetState();
    /// \brief
    void ComputeThrustCtrl();
    void ComputeAttCtrl(double dt);
    void ComputeMotorCmd();
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
    ros::Publisher pub_rotorspeed_;
    ros::Publisher pub_yaw_;
    ros::Publisher pub_pit_;
    ros::Publisher pub_rol_;
    /// \brief
    chroma_gazebo_plugins_msgs::Rotorspeed msg_rotorspeed_;
    /// \brief subscribers
    ros::Subscriber sub_cmd_;
    ros::Subscriber sub_imu_;

    /// Parameters /////////////////////////////////////
    /// \brief Robot namespace.
    std::string robot_namespace_;
    /// \brief Topic name to control the rotor speed
    std::string topic_rotorspeed_;
    /// \brief Topic name for command
    std::string topic_cmd_;
    /// \brief Topic name for imu
    std::string topic_imu_;
    /// \brief rotors ids
    int id_front_right_rotor_;
    int id_front_left_rotor_;
    int id_rear_right_rotor_;
    int id_rear_left_rotor_;
    /// \brief rotors direction
    double dir_front_right_rotor_;
    double dir_front_left_rotor_;
    double dir_rear_right_rotor_;
    double dir_rear_left_rotor_;
    /// \brief
    double rotorspeed_max_;
    double rotorspeed_min_;
    /// \brief Gains
    double kp_yaw_rate_;
    double kp_rol_rate_;
    double kp_pit_rate_;
    double kp_thrust_;
    /// \brief Trims
    double trim_rol_;
    double trim_pit_;
    /// \brief Interval of time between each measurements
    double interval_;

    /// Others ///////////////////////////////////////
    /// \brief current time
    common::Time current_time_;
    /// \brief last time of measurment
    common::Time last_time_;
    /// \brief rotors cmds
    double cmd_front_right_rotor_;
    double cmd_front_left_rotor_;
    double cmd_rear_right_rotor_;
    double cmd_rear_left_rotor_;
    /// \brief
    ignition::math::Vector3<double> a_n_;
    ignition::math::Quaternion<double> q_n_;
    ignition::math::Vector3<double> w1_n_;
    ignition::math::Vector3<double> w1_n_1_;
    /// \brief
    ignition::math::Quaternion<double> cmd_q_n_;
    ignition::math::Quaternion<double> cmd_q_n_1_;
    ignition::math::Quaternion<double> dcmd_q_n_;
    ignition::math::Quaternion<double> dcmd_q_n_1_;
    ignition::math::Vector3<double> cmd_a_n_;
    /// \brief
    double u_rol_rate_;
    double u_pit_rate_;
    double u_yaw_rate_;
    double u_thrust_;
    /// \brief
    std::list<double> dt_;


  };
}
#endif // GAZEBO_PLUGINS_MAVLINK_INTERFACE_H_
