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

#ifndef GAZEBO_PLUGINS_MAVLINK_INTERFACE_H_
#define GAZEBO_PLUGINS_MAVLINK_INTERFACE_H_

#include "gazebo_plugins/common.h"
#include "gazebo_plugins/geo_mag_declination.h"
#include "gazebo_plugins/msg_buffer.h"

#include <vector>
#include <thread>
#include <mutex>
#include <deque>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <cassert>
#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#include <boost/system/system_error.hpp>

#include <iostream>
#include <random>
#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <chroma_gazebo_plugins_msgs/Barometer.h>
#include <chroma_gazebo_plugins_msgs/BodyGroundTruth.h>
#include <chroma_gazebo_plugins_msgs/Gps.h>
#include <chroma_gazebo_plugins_msgs/IRLock.h>
#include <chroma_gazebo_plugins_msgs/OpticalFlow.h>
#include <chroma_gazebo_plugins_msgs/Rotorspeed.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/MagneticField.h>

using lock_guard = std::lock_guard<std::recursive_mutex>;


static const unsigned NB_MAX_ACTUATOR = 16;

//! Maximum buffer size with padding for CRC bytes (280 + padding)
static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
static constexpr size_t  MAX_TX_BUFFER_SIZE = 1000;

namespace gazebo {

// Default values
static constexpr float KDEFAULT_PROTOCOL_VERSION = 2.0;
static constexpr bool KDEFAULT_MODE_HIL= false;
static constexpr bool KDEFAULT_SERIAL_ENABLED = false;
static constexpr bool KDEFAULT_VEHICLE_IS_TAILSITTER = false;
static constexpr uint32_t KDEFAULT_BAUDRATE = 921600;
static constexpr uint32_t KDEFAULT_FCU_PORT = 14560;
static constexpr uint32_t KDEFAULT_QGC_PORT = 14550;

static const std::string KDEFAULT_NAMESPACE = "";
static const std::string KDEFAULT_SERIAL_DEVICE = "/dev/ttyACM0";

// Default ROS topic names
static const std::string KDEFAULT_TOPIC_ROTORSPEED_CMD = "command/rotorspeed";
static const std::string KDEFAULT_TOPIC_GPS = "gps";
static const std::string KDEFAULT_TOPIC_IMU = "imu";
static const std::string KDEFAULT_TOPIC_MAG = "magnetometer";
static const std::string KDEFAULT_TOPIC_BARO = "barometer";
static const std::string KDEFAULT_TOPIC_IRLOCK = "camera/link/irlock";
static const std::string KDEFAULT_TOPIC_LIDAR = "link/lidar";
static const std::string KDEFAULT_TOPIC_OPTICALFLOW = "px4flow/link/opticalFlow";
static const std::string KDEFAULT_TOPIC_SONAR = "sonar_model/link/sonar";
static const std::string KDEFAULT_TOPIC_VISUALODOMETRY = "vision_odom";

//! Rx packer framing status. (same as @p mavlink::mavlink_framing_t)
enum class Framing : uint8_t {
	incomplete = MAVLINK_FRAMING_INCOMPLETE,
	ok = MAVLINK_FRAMING_OK,
	bad_crc = MAVLINK_FRAMING_BAD_CRC,
	bad_signature = MAVLINK_FRAMING_BAD_SIGNATURE,
};

/// \brief A plugin that interface an FCU with Gazebo
class GAZEBO_VISIBLE MavlinkInterfacePlugin : public ModelPlugin {

  /// Methods //////////////////////////////////////////////////////////
  public:
    MavlinkInterfacePlugin();   /// \brief Constructor.
    ~MavlinkInterfacePlugin();  /// \brief Destructor.

  protected:
    /// \brief Load model parameters
    void Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf);
    /// \brief Update
    void OnUpdate(const common::UpdateInfo&  /*_info*/);
    /// \brief Publish ROS topics
    void Publish();

  private:
    /// \brief ROS Callback on Groundtruth Topic
    void CallbackGroundtruth(const chroma_gazebo_plugins_msgs::BodyGroundTruth& _in_msg_groundtruth);
    /// \brief ROS Callback on IMU Topic
    void CallbackImu(const sensor_msgs::Imu& _in_imu_msg);
    /// \brief ROS Callback on Magnetometer Topic
    void CallbackMag(const sensor_msgs::MagneticField& _in_msg_mag);
    /// \brief ROS Callback on Barometer Topic
    void CallbackBaro(const chroma_gazebo_plugins_msgs::Barometer& _in_msg_baro);
    /// \brief ROS Callback on GPS Topic
    void CallbackGps(const chroma_gazebo_plugins_msgs::Gps& _in_msg_gps);
    /// \brief ROS Callback on Sonar Topic
    void CallbackSonar(const sensor_msgs::Range& _in_msg_sonar);
    /// \brief ROS Callback on Lidar Topic
    void CallbackLidar(const sensor_msgs::Range& _in_msg_lidar);
    /// \brief ROS Callback on OpticalFlow Topic
    void CallbackOpticalFlow(const chroma_gazebo_plugins_msgs::OpticalFlow& _in_msg_optflow);
    /// \brief ROS Callback on IRLock Topic
    void CallbackIRLock(const chroma_gazebo_plugins_msgs::IRLock& _in_msg_irlock);
    /// \brief ROS Callback on Vision Topic
    void CallbackVision(const nav_msgs::Odometry& _in_msg_visualodom);

    /// \brief Send Mavlink Msgs
    void SendMavlinkMsg(const mavlink_message_t *message, const int destination_port = 0);
    /// \brief Handle received Mavlink msg
    void HandleMavlinkMsg(mavlink_message_t *msg);
    /// \brief
    void PollMavlinkMsg(double _dt, uint32_t _timeoutMs);

    /// \brief
    void OpenSerialInterface();
    /// \brief
    void CloseSerialInterface();
    /// \brief
    void ReadSerialInterface();
    /// \brief
    void ParseSerialBuffer(const boost::system::error_code& err, std::size_t bytes_t);
    /// \brief
    void WriteSerialInterface(bool check_tx_state);

    /// \brief
    void HandleActuators(double _dt);

  /// Attributes ///////////////////////////////////////////////////////
  private:
    /// Gazebo related /////////////////////////////////
    /// \brief Pointer to the world
    physics::WorldPtr world_;
    /// \brief Pointer to the model
    physics::ModelPtr model_;
    /// \brief Pointers to the models' actuators joints
    std::vector<physics::JointPtr> actuators_joint_;

    /// \brief Pointer to the update event connection.
    event::ConnectionPtr update_connection_ptr_;

    /// ROS related ////////////////////////////////////
    /// \brief node handler
    ros::NodeHandle* node_handle_;
    ros::Publisher pub_rotorspeed_cmd_;

    ros::Subscriber sub_groundtruth_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_mag_;
    ros::Subscriber sub_baro_;
    ros::Subscriber sub_gps_;
    ros::Subscriber sub_irlock_;
    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_opticalflow_;
    ros::Subscriber sub_sonar_;
    ros::Subscriber sub_visualodometry_;

    /// Parameters /////////////////////////////////////
    /// \brief Robot namespace.
    std::string robot_namespace_;
    /// \brief Name of the link
    std::string link_name_;
    /// \brief Topic name to control the rotor speed
    std::string topic_rotorspeed_cmd_;
    /// \brief
    uint32_t baudrate_;
    /// \brief
    uint32_t fcu_port_;
    /// \brief
    uint32_t qgc_port_;

    std::string topic_imu_;
    std::string topic_mag_;
    std::string topic_baro_;
    std::string topic_gps_;
    std::string topic_irlock_;
    std::string topic_lidar_;
    std::string topic_opticalflow_;
    std::string topic_sonar_;
    std::string topic_visualodometry_;

    bool serial_enabled_;
    std::string serial_device_;

    /// Channel Parameters //////////////////////////////
    /// \brief
    double actuators_zero_position_disarmed_[NB_MAX_ACTUATOR];
    /// \brief
    double actuators_zero_position_armed_[NB_MAX_ACTUATOR];
    /// \brief
    double actuators_cmd_offset_[NB_MAX_ACTUATOR];
    /// \brief
    double actuators_cmd_scaling_[NB_MAX_ACTUATOR];
    /// \brief
    std::string actuators_cmd_type_[NB_MAX_ACTUATOR];

    /// Actuator Control ///////////////////////////////
    /// \brief Commands to give to the actuators
    Eigen::VectorXd actuators_cmd_;
    /// \brief  PID on actuator commands
    std::vector<common::PID> actuators_pid_;

    /// MAVLINK protocol ///////////////////////////////
    /// \brief Boolean to notify that the plugin has already rcv
    /// MAVLINK msgs
    bool rcv_first_mavlink_msg_;
    /// \brief Mavlink protocol version
    float protocol_version_;
    /// \brief
    int socket_;
    /// \brief
    struct pollfd socket_info_[1];
    /// \brief
    unsigned char buffer_[65535];
    /// \brief
    mavlink_status_t mavlink_status_;
    /// \brief
    mavlink_message_t mavlink_msg_;

    /// Serial Communication //////////////////////////
    /// \brief
    std::thread io_thread_;
    /// \brief
    boost::asio::io_service io_service_;
    /// \brief
    boost::asio::serial_port serial_port_;
    /// \brief
    std::array<uint8_t, MAX_SIZE> rx_buffer_;
    /// \brief
    std::recursive_mutex mutex_;
    /// \brief
    std::atomic<bool> tx_in_progress_;
    /// \brief
    std::deque<MsgBuffer> tx_buffer_;

    /// FCU communication /////////////////////////////
    /// \brief The locally bound address
    struct sockaddr_in fcu_sockaddr_;
    /// \brief
    in_addr_t fcu_addr_;

    /// QGroundControl communication ///////////////////
    /// \brief
    struct sockaddr_in qgc_sockaddr_;
    /// \brief
    socklen_t qgc_sockaddr_len_;
    /// \brief
    in_addr_t qgc_addr_;

    /// Random generator //////////////////////////////
    /// \brief
    std::default_random_engine rand_;
    /// \brief
    std::normal_distribution<float> randn_;

    /// Others ////////////////////////////////////////
    /// \brief
    double alt_home_;

    /// \brief
    common::Time current_time_;
    /// \brief
    common::Time last_time_;
    /// \brief
    common::Time last_actuator_time_;
    /// \brief
    uint32_t last_imu_time_us_;

    /// \brief
    sensor_msgs::MagneticField msg_mag_;
    /// \brief
    chroma_gazebo_plugins_msgs::Barometer msg_baro_;

    /// \brief
    ignition::math::Vector3<double> optflow_gyro_;
    /// \brief
    double optflow_dist_;

  };
}
#endif // GAZEBO_PLUGINS_MAVLINK_INTERFACE_H_
