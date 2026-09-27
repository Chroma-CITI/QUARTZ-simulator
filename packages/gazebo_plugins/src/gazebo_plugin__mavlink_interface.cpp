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

#include "gazebo_plugins/gazebo_plugin__mavlink_interface.h"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(MavlinkInterfacePlugin);

/////////////////////////////////////////////////
MavlinkInterfacePlugin::MavlinkInterfacePlugin() : ModelPlugin(),
  rcv_first_mavlink_msg_(false),
  actuators_cmd_offset_ {},
  actuators_cmd_scaling_ {},
  actuators_zero_position_disarmed_ {},
  actuators_zero_position_armed_ {},
  serial_enabled_(false),
  optflow_gyro_  {},
  tx_buffer_ {},
  rx_buffer_ {},
  mavlink_status_ {},
  mavlink_msg_ {},
  io_service_(),
  serial_port_(io_service_)
{
  const char *env_alt = std::getenv("PX4_HOME_ALT");
  if (env_alt) {
    gzmsg << "Home altitude is set to " << env_alt << ".\n";
    alt_home_ = std::stod(env_alt);
  }
  else {
    alt_home_ = 480.0;
  }
  // Mag msg initialisation
  msg_mag_.magnetic_field.x = 0.0;
  msg_mag_.magnetic_field.y = 0.0;
  msg_mag_.magnetic_field.z = 0.0;
  // Baro msg initialisation
  msg_baro_.pressure = 0.0;
  msg_baro_.altitude = 0.0;
  msg_baro_.temperature = 0.0;
}

/////////////////////////////////////////////////
MavlinkInterfacePlugin::~MavlinkInterfacePlugin() {
  CloseSerialInterface();
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_.reset();
  #else
  event::Events::DisconnectWorldUpdateBegin(update_connection_ptr_);
  #endif
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf) {
  // Store the pointer to the model.
  model_ = _in_model;
  // Store the pointer to the world.
  world_ = model_->GetWorld();
  // Get world informations
#if GAZEBO_MAJOR_VERSION >= 9
  current_time_ = world_->SimTime();
  last_time_ = world_->SimTime();
  last_imu_time_us_ = world_->SimTime().nsec * 1e-3;
#else
  current_time_ = world_->GetSimTime();
  last_time_ = world_->GetSimTime();
  last_imu_time_us_ = world_->GetSimTime().nsec * 1e-3;
#endif
  // Get namespace param
  getSdfParam<std::string>(_in_sdf, "robot_namespace", robot_namespace_, KDEFAULT_NAMESPACE);
  // Get mavlink protocol version
  getSdfParam<float>(_in_sdf, "protocol_version", protocol_version_, KDEFAULT_PROTOCOL_VERSION);
  // Get ros topic names
  getSdfParam<std::string>(_in_sdf, "topic_gps", topic_gps_, KDEFAULT_TOPIC_GPS);
  getSdfParam<std::string>(_in_sdf, "topic_imu", topic_imu_, KDEFAULT_TOPIC_IMU);
  getSdfParam<std::string>(_in_sdf, "topic_mag", topic_mag_, KDEFAULT_TOPIC_MAG);
  getSdfParam<std::string>(_in_sdf, "topic_baro", topic_baro_, KDEFAULT_TOPIC_BARO);
  getSdfParam<std::string>(_in_sdf, "topic_irlock", topic_irlock_, KDEFAULT_TOPIC_IRLOCK);
  getSdfParam<std::string>(_in_sdf, "topic_lidar", topic_lidar_, KDEFAULT_TOPIC_LIDAR);
  getSdfParam<std::string>(_in_sdf, "topic_opticalflow", topic_opticalflow_, KDEFAULT_TOPIC_OPTICALFLOW);
  getSdfParam<std::string>(_in_sdf, "topic_rotorspeed_cmd", topic_rotorspeed_cmd_, KDEFAULT_TOPIC_ROTORSPEED_CMD);
  getSdfParam<std::string>(_in_sdf, "topic_sonar", topic_sonar_, KDEFAULT_TOPIC_SONAR);
  getSdfParam<std::string>(_in_sdf, "topic_visualodometry", topic_visualodometry_, KDEFAULT_TOPIC_VISUALODOMETRY);
  // Get serial params
  getSdfParam<bool>(_in_sdf, "serial_enabled", serial_enabled_, KDEFAULT_SERIAL_ENABLED);
  getSdfParam<std::string>(_in_sdf, "serial_device", serial_device_, KDEFAULT_SERIAL_DEVICE);
  getSdfParam<uint32_t>(_in_sdf, "baudrate", baudrate_, KDEFAULT_BAUDRATE);
  // get fcu mavlink params
  std::string fcu_addr;
  bool has_fcu_addr = getSdfParam<std::string>(_in_sdf, "fcu_addr", fcu_addr, "");
  getSdfParam<uint32_t>(_in_sdf, "fcu_port", fcu_port_, KDEFAULT_FCU_PORT);
  // get Qgroundcontrol mavlink params
  std::string qgc_addr;
  bool has_qgc_addr = getSdfParam<std::string>(_in_sdf, "qgc_addr", qgc_addr, "");
  getSdfParam<uint32_t>(_in_sdf, "qgc_port", qgc_port_, KDEFAULT_QGC_PORT);
  // set actuators_cmd_ from inputs.control
  actuators_cmd_.resize(NB_MAX_ACTUATOR);
  actuators_pid_.resize(NB_MAX_ACTUATOR);
  actuators_joint_.resize(NB_MAX_ACTUATOR);
  for (int i = 0; i < NB_MAX_ACTUATOR; ++i) {
    actuators_pid_[i].Init(0, 0, 0, 0, 0, 0, 0);
    actuators_cmd_[i] = 0;
  }
  // read channel parameters
  if (_in_sdf->HasElement("control_channels")) {
    sdf::ElementPtr control_channels = _in_sdf->GetElement("control_channels");
    sdf::ElementPtr channel = control_channels->GetElement("channel");
    while (channel) {
      if (channel->HasElement("id")) {
        int id = channel->Get<int>("id");
        if (id < NB_MAX_ACTUATOR) {
          // Arming / disarming position
          actuators_zero_position_disarmed_[id] = channel->Get<double>("zero_position_disarmed");
          actuators_zero_position_armed_[id] = channel->Get<double>("zero_position_armed");
          // Cmd informations
          actuators_cmd_offset_[id] = channel->Get<double>("cmd_offset");
          actuators_cmd_scaling_[id] = channel->Get<double>("cmd_scaling");
          if (channel->HasElement("cmd_type")) {
            actuators_cmd_type_[id] = channel->Get<std::string>("cmd_type");
          }
          else {
            gzwarn << "cmd_type[" << id << "] not specified, using angular_speed as default.\n";
            actuators_cmd_type_[id] = "angular_speed";
          }
          if (channel->HasElement("joint_name")) {
            std::string joint_name = channel->Get<std::string>("joint_name");
            actuators_joint_[id] = model_->GetJoint(joint_name);
            if (actuators_joint_[id] == nullptr) {
              gzwarn << "joint [" << joint_name << "] not found for channel["<< id << "]"
                     << " - no joint control for this channel.\n";
            }
            else {
              gzdbg << "joint [" << joint_name << "] found for channel["<< id << "]"
                    <<" - joint control active for this channel.\n";
            }
          }
          else {
            gzdbg << "<joint_name> not found for channel[" << id
                  << "] no joint control will be performed for this channel.\n";
          }
          // setup joint control pid to control joint
          if (channel->HasElement("control_pid"))
          {
            sdf::ElementPtr pid = channel->GetElement("control_pid");
            double p = 0;
            if (pid->HasElement("p"))
              p = pid->Get<double>("p");
            double i = 0;
            if (pid->HasElement("i"))
              i = pid->Get<double>("i");
            double d = 0;
            if (pid->HasElement("d"))
              d = pid->Get<double>("d");
            double iMax = 0;
            if (pid->HasElement("iMax"))
              iMax = pid->Get<double>("iMax");
            double iMin = 0;
            if (pid->HasElement("iMin"))
              iMin = pid->Get<double>("iMin");
            double cmdMax = 0;
            if (pid->HasElement("cmdMax"))
              cmdMax = pid->Get<double>("cmdMax");
            double cmdMin = 0;
            if (pid->HasElement("cmdMin"))
              cmdMin = pid->Get<double>("cmdMin");
            actuators_pid_[id].Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
          }
        }
        else {
          gzerr << "id[" << id << "] out of range, not parsing.\n";
        }
      }
      else {
        gzerr << "no id, not parsing.\n";
        break;
      }
      channel = channel->GetNextElement("channel");
    }
  }
  // Serial communication initialisation
  if(serial_enabled_) {
    // Set up serial interface
    io_service_.post(std::bind(&MavlinkInterfacePlugin::ReadSerialInterface, this));
    // run io_service_ for async io
    io_thread_ = std::thread([this] () {
      io_service_.run();
    });
    OpenSerialInterface();
  }
  //Create socket with FCU
  fcu_addr_ = htonl(INADDR_ANY);
  if (has_fcu_addr) {
    if (fcu_addr != "INADDR_ANY") {
      fcu_addr_ = inet_addr(fcu_addr.c_str());
      if (fcu_addr_ == INADDR_NONE) {
        gzerr << "invalid fcu_addr \n";
        fprintf(stderr, "invalid fcu_addr \"%s\"\n", fcu_addr.c_str());
        return;
      }
    }
  }
#if GAZEBO_MAJOR_VERSION >= 9
  model_param(world_->Name(), model_->GetName(), "fcu_port", fcu_port_);
#else
  model_param(world_->GetName(), model_->GetName(), "fcu_port", fcu_port_);
#endif
  //Create socket with QGroundControl
  qgc_addr_ = htonl(INADDR_ANY);
  if (has_qgc_addr) {
    if (qgc_addr != "INADDR_ANY") {
      qgc_addr_ = inet_addr(qgc_addr.c_str());
      if (qgc_addr_ == INADDR_NONE) {
        gzerr << "invalid qgc_addr \n";
        fprintf(stderr, "invalid qgc_addr \"%s\"\n", qgc_addr.c_str());
        return;
      }
    }
  }
  // try to setup udp socket for communication
  if ((socket_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    gzerr << "Create socket failed\n";
    return;
  }
  memset((char *)&fcu_sockaddr_, 0, sizeof(fcu_sockaddr_));
  fcu_sockaddr_.sin_family = AF_INET;
  qgc_sockaddr_.sin_family = AF_INET;
  if (serial_enabled_) {
    // gcs link
    gzmsg << "Serial Connection\n";
    fcu_sockaddr_.sin_addr.s_addr = fcu_addr_;
    fcu_sockaddr_.sin_port = htons(fcu_port_);
    qgc_sockaddr_.sin_addr.s_addr = qgc_addr_;
    qgc_sockaddr_.sin_port = htons(qgc_port_);
  }
  else {
    gzmsg << "Ethernet Connection\n";
    fcu_sockaddr_.sin_addr.s_addr = htonl(INADDR_ANY);
    // Let the OS pick the port
    fcu_sockaddr_.sin_port = htons(0);
    qgc_sockaddr_.sin_addr.s_addr = fcu_addr_;
    qgc_sockaddr_.sin_port = htons(fcu_port_);
  }
  qgc_sockaddr_len_ = sizeof(qgc_sockaddr_);
  if (bind(socket_, (struct sockaddr *)&fcu_sockaddr_, sizeof(fcu_sockaddr_)) < 0) {
    printf("bind failed\n");
    return;
  }
  socket_info_[0].fd = socket_;
  socket_info_[0].events = POLLIN;
  // set the Mavlink protocol version to use on the link
  mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);
  if (protocol_version_ == 2.0) {
    chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    gzmsg << "Using MAVLink protocol v2.0\n";
  }
  else if (protocol_version_ == 1.0) {
    chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    gzmsg << "Using MAVLink protocol v1.0\n";
  }
  else {
    gzerr << "Unkown protocol version! Using v" << protocol_version_ << "by default \n";
  }
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(std::bind(&MavlinkInterfacePlugin::OnUpdate, this, std::placeholders::_1));
  #else
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&MavlinkInterfacePlugin::OnUpdate, this, _1));
  #endif
  // ROS initialisation
  node_handle_ = new ros::NodeHandle(robot_namespace_);
  // Subscriber to IMU sensor_msgs::Imu Mess
  sub_groundtruth_ = node_handle_->subscribe("/" + robot_namespace_ + "/groundtruth", 1000, &MavlinkInterfacePlugin::CallbackGroundtruth, this);
  sub_imu_ = node_handle_->subscribe("/" + robot_namespace_ +  "/" +topic_imu_, 1000, &MavlinkInterfacePlugin::CallbackImu, this);
  sub_mag_ = node_handle_->subscribe("/" + robot_namespace_ +  "/" +topic_mag_, 1000,  &MavlinkInterfacePlugin::CallbackMag, this);
  sub_baro_ = node_handle_->subscribe("/" + robot_namespace_ +  "/" +topic_baro_, 1000, &MavlinkInterfacePlugin::CallbackBaro, this);
  sub_gps_ = node_handle_->subscribe("/" + robot_namespace_ +  "/" +topic_gps_, 1000, &MavlinkInterfacePlugin::CallbackGps, this);
  sub_sonar_ = node_handle_->subscribe("/" + robot_namespace_ +  "/" +topic_sonar_, 1000, &MavlinkInterfacePlugin::CallbackSonar, this);
  sub_lidar_ = node_handle_->subscribe("/" + robot_namespace_ +  "/" +topic_lidar_, 1000, &MavlinkInterfacePlugin::CallbackLidar, this);
  sub_opticalflow_ = node_handle_->subscribe("/" + robot_namespace_ +  "/" +topic_opticalflow_, 1000, &MavlinkInterfacePlugin::CallbackOpticalFlow, this);
  sub_irlock_ = node_handle_->subscribe("/" + robot_namespace_ +  "/" +topic_irlock_, 1000, &MavlinkInterfacePlugin::CallbackIRLock, this);
  sub_visualodometry_ = node_handle_->subscribe("/" + robot_namespace_ +  "/" +topic_visualodometry_, 1000, &MavlinkInterfacePlugin::CallbackVision, this);
  // Publish gazebo's motor_speed message
  pub_rotorspeed_cmd_ = node_handle_->advertise<chroma_gazebo_plugins_msgs::Rotorspeed>("/" + robot_namespace_ +  "/" + topic_rotorspeed_cmd_, 10);
}

/////////////////////////////////////////////////
// This gets called by the world update start event.
void MavlinkInterfacePlugin::OnUpdate(const common::UpdateInfo&  /*_info*/) {
#if GAZEBO_MAJOR_VERSION >= 9
  current_time_ = world_->SimTime();
#else
  current_time_ = world_->GetSimTime();
#endif
  double dt = (current_time_ - last_time_).Double();
  PollMavlinkMsg(dt, 1000);
  HandleActuators(dt);
  // Save current time
  last_time_ = current_time_;
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::SendMavlinkMsg(const mavlink_message_t *message, const int destination_port)
{
  if(serial_enabled_ && destination_port == 0) {
    assert(message != nullptr);
    if (!serial_port_.is_open()) {
      gzerr << "Serial port closed! \n";
      return;
    }
    else {
      lock_guard lock(mutex_);
      if (tx_buffer_.size() >= MAX_TX_BUFFER_SIZE) {
        gzwarn << "TX queue overflow. \n";
      }
      tx_buffer_.emplace_back(message);
    }
    io_service_.post(std::bind(&MavlinkInterfacePlugin::WriteSerialInterface, this, true));
  }
  else {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen = mavlink_msg_to_send_buffer(buffer, message);
    struct sockaddr_in dest_addr;
    memcpy(&dest_addr, &qgc_sockaddr_, sizeof(qgc_sockaddr_));
    if (destination_port != 0) {
      dest_addr.sin_port = htons(destination_port);
    }
    ssize_t len = sendto(socket_, buffer, packetlen, 0, (struct sockaddr *)&qgc_sockaddr_, sizeof(qgc_sockaddr_));
    if (len <= 0) {
      gzerr << "Failed sending mavlink message\n";
      printf("Failed sending mavlink message\n");
    }
  }
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::CallbackGroundtruth(const chroma_gazebo_plugins_msgs::BodyGroundTruth& _in_msg_groundtruth){
    // send ground truth
    mavlink_hil_state_quaternion_t mavlink_msg_hil_state;
    mavlink_msg_hil_state.time_usec = HeaderToUsec(_in_msg_groundtruth.header);
    mavlink_msg_hil_state.lat = _in_msg_groundtruth.lat * 1e7;  // conversion degE7
    mavlink_msg_hil_state.lon = _in_msg_groundtruth.lon * 1e7;  // conversion degE7
    mavlink_msg_hil_state.alt = _in_msg_groundtruth.alt * 1000; // conversion en mm
    mavlink_msg_hil_state.vx = _in_msg_groundtruth.vy_enu * 100; // North speed cm/s
    mavlink_msg_hil_state.vy = _in_msg_groundtruth.vx_enu * 100; // East Speed cm/s
    mavlink_msg_hil_state.vz = -_in_msg_groundtruth.vz_enu * 100; // Alt speed cm/s
    mavlink_msg_hil_state.ind_airspeed = _in_msg_groundtruth.airflow; // assumed indicated airspeed due to flow aligned with pitot (body x)
    mavlink_msg_hil_state.true_airspeed = _in_msg_groundtruth.wind * 100;   //no wind simulated
    // Acceleration
    ignition::math::Vector3<double> linear_acceleration_frd = Q_FLU_TO_FRD.RotateVector(ignition::math::Vector3<double>(
      _in_msg_groundtruth.ax_flu,
      _in_msg_groundtruth.ay_flu,
      _in_msg_groundtruth.az_flu));
    mavlink_msg_hil_state.xacc = linear_acceleration_frd.X() * 1000;
    mavlink_msg_hil_state.yacc = linear_acceleration_frd.Y() * 1000;
    mavlink_msg_hil_state.zacc = linear_acceleration_frd.Z() * 1000;
    ignition::math::Quaternion<double> q_frd_to_ned =
      Q_ENU_TO_NED *
      ignition::math::Quaternion<double>(
        _in_msg_groundtruth.qw, _in_msg_groundtruth.qx, _in_msg_groundtruth.qy, _in_msg_groundtruth.qz) *
      Q_FLU_TO_FRD.Inverse();
    mavlink_msg_hil_state.attitude_quaternion[0] = q_frd_to_ned.W();
    mavlink_msg_hil_state.attitude_quaternion[1] = q_frd_to_ned.X();
    mavlink_msg_hil_state.attitude_quaternion[2] = q_frd_to_ned.Y();
    mavlink_msg_hil_state.attitude_quaternion[3] = q_frd_to_ned.Z();
    ignition::math::Vector3<double> angular_velocity_frd = Q_FLU_TO_FRD.RotateVector(ignition::math::Vector3<double>(
      _in_msg_groundtruth.wx_flu,
      _in_msg_groundtruth.wy_flu,
      _in_msg_groundtruth.wz_flu));
    mavlink_msg_hil_state.rollspeed  = angular_velocity_frd.X();
    mavlink_msg_hil_state.pitchspeed = angular_velocity_frd.Y();
    mavlink_msg_hil_state.yawspeed   = angular_velocity_frd.Z();
    mavlink_message_t msg;
    mavlink_msg_hil_state_quaternion_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &mavlink_msg_hil_state);
    SendMavlinkMsg(&msg);
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::CallbackImu(const sensor_msgs::Imu& _in_msg_imu) {
  // Mavlink message creation
  mavlink_hil_sensor_t mavlink_msg_hil_sensor;
  mavlink_msg_hil_sensor.time_usec = HeaderToUsec(_in_msg_imu.header);
  // Acceleration
  ignition::math::Vector3<double> linear_acceleration_frd = Q_FLU_TO_FRD.RotateVector(ignition::math::Vector3<double>(
    _in_msg_imu.linear_acceleration.x,
    _in_msg_imu.linear_acceleration.y,
    _in_msg_imu.linear_acceleration.z));
  mavlink_msg_hil_sensor.xacc = linear_acceleration_frd.X();
  mavlink_msg_hil_sensor.yacc = linear_acceleration_frd.Y();
  mavlink_msg_hil_sensor.zacc = linear_acceleration_frd.Z();
  // Angular speed
  ignition::math::Vector3<double> angular_velocity_frd = Q_FLU_TO_FRD.RotateVector(ignition::math::Vector3<double>(
    _in_msg_imu.angular_velocity.x,
    _in_msg_imu.angular_velocity.y,
    _in_msg_imu.angular_velocity.z));
  mavlink_msg_hil_sensor.xgyro = angular_velocity_frd.X();
  mavlink_msg_hil_sensor.ygyro = angular_velocity_frd.Y();
  mavlink_msg_hil_sensor.zgyro = angular_velocity_frd.Z();
  // Magnetometer
  ignition::math::Vector3<double> magfield_frd = Q_FLU_TO_FRD.RotateVector(ignition::math::Vector3<double>(
    msg_mag_.magnetic_field.x,
    msg_mag_.magnetic_field.y,
    msg_mag_.magnetic_field.z));
  mavlink_msg_hil_sensor.xmag = magfield_frd.X();
  mavlink_msg_hil_sensor.ymag = magfield_frd.Y();
  mavlink_msg_hil_sensor.zmag = magfield_frd.Z();
  // barometer
  mavlink_msg_hil_sensor.abs_pressure = msg_baro_.pressure; // convert to mBar (hPa = mBar)
  mavlink_msg_hil_sensor.pressure_alt = msg_baro_.altitude; // calculate pressure altitude including effect of pressure noise
  mavlink_msg_hil_sensor.diff_pressure = msg_baro_.diff_pressure; // convert to mBar (hPa = mBar)
  mavlink_msg_hil_sensor.temperature = msg_baro_.temperature - 273.15; // calculate temperature in Celsius

  mavlink_msg_hil_sensor.fields_updated = 4095;

  //accumulate gyro measurements that are needed for the optical flow message
  uint32_t dt_us = mavlink_msg_hil_sensor.time_usec - last_imu_time_us_;
  if (dt_us > 1000) {
    optflow_gyro_ += angular_velocity_frd * (dt_us / 1000000.0f);
  }
  last_imu_time_us_ = mavlink_msg_hil_sensor.time_usec;

  mavlink_message_t msg;
  mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &mavlink_msg_hil_sensor);
  SendMavlinkMsg(&msg);
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::CallbackMag(const sensor_msgs::MagneticField& _in_msg_mag) {
  msg_mag_.magnetic_field.x = _in_msg_mag.magnetic_field.x;
  msg_mag_.magnetic_field.y = _in_msg_mag.magnetic_field.y;
  msg_mag_.magnetic_field.z = _in_msg_mag.magnetic_field.z;
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::CallbackBaro(const chroma_gazebo_plugins_msgs::Barometer& _in_msg_baro) {
  msg_baro_.pressure = _in_msg_baro.pressure;
  msg_baro_.diff_pressure = _in_msg_baro.diff_pressure;
  msg_baro_.altitude = _in_msg_baro.altitude;
  msg_baro_.temperature = _in_msg_baro.temperature;
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::CallbackGps(const chroma_gazebo_plugins_msgs::Gps& _in_msg_gps){
  // fill HIL GPS Mavlink msg
  mavlink_hil_gps_t mavlink_msg_hil_gps;
  mavlink_msg_hil_gps.time_usec =  HeaderToUsec(_in_msg_gps.header);
  mavlink_msg_hil_gps.fix_type = 3;
  mavlink_msg_hil_gps.lat = _in_msg_gps.lat * 1e7;
  mavlink_msg_hil_gps.lon = _in_msg_gps.lon * 1e7;
  mavlink_msg_hil_gps.alt = _in_msg_gps.alt * 1000.0;
  mavlink_msg_hil_gps.eph = (uint16_t) (_in_msg_gps.eph * 100.0);
  mavlink_msg_hil_gps.epv = (uint16_t) (_in_msg_gps.epv * 100.0);
  mavlink_msg_hil_gps.vel = _in_msg_gps.vel * 100.0;
  mavlink_msg_hil_gps.vn = _in_msg_gps.vel_north * 100.0;
  mavlink_msg_hil_gps.ve = _in_msg_gps.vel_east * 100.0;
  mavlink_msg_hil_gps.vd = -_in_msg_gps.vel_up * 100.0;
  mavlink_msg_hil_gps.cog = (uint16_t) (_in_msg_gps.cog * 100.0);
  mavlink_msg_hil_gps.satellites_visible = 10;
  // send HIL_GPS Mavlink msg
  mavlink_message_t msg;
  mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &mavlink_msg_hil_gps);
  SendMavlinkMsg(&msg);
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::CallbackSonar(const sensor_msgs::Range& _in_msg_sonar) {
  mavlink_distance_sensor_t mavlink_msg_distance_sensor;
  mavlink_msg_distance_sensor.time_boot_ms = HeaderToMsec(_in_msg_sonar.header);
  mavlink_msg_distance_sensor.min_distance = _in_msg_sonar.min_range * 100.0;
  mavlink_msg_distance_sensor.max_distance = _in_msg_sonar.max_range * 100.0;
  mavlink_msg_distance_sensor.current_distance = _in_msg_sonar.range * 100.0;
  mavlink_msg_distance_sensor.type = 1;
  mavlink_msg_distance_sensor.id = 1;
  mavlink_msg_distance_sensor.orientation = 0;  // forward facing
  mavlink_msg_distance_sensor.covariance = 0;

  mavlink_message_t msg;
  mavlink_msg_distance_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &mavlink_msg_distance_sensor);
  SendMavlinkMsg(&msg);
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::CallbackLidar(const sensor_msgs::Range& _in_msg_lidar) {
  mavlink_distance_sensor_t mavlink_msg_distance_sensor;
  mavlink_msg_distance_sensor.time_boot_ms = HeaderToMsec(_in_msg_lidar.header);
  mavlink_msg_distance_sensor.min_distance = _in_msg_lidar.min_range * 100.0;
  mavlink_msg_distance_sensor.max_distance = _in_msg_lidar.max_range * 100.0;
  mavlink_msg_distance_sensor.current_distance = _in_msg_lidar.range * 100.0;
  mavlink_msg_distance_sensor.type = 0;
  mavlink_msg_distance_sensor.id = 0;
  mavlink_msg_distance_sensor.orientation = 25;//downward facing
  mavlink_msg_distance_sensor.covariance = 0;
  //distance needed for optical flow message
  optflow_dist_ = _in_msg_lidar.range;  //[m]
  mavlink_message_t msg;
  mavlink_msg_distance_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &mavlink_msg_distance_sensor );
  SendMavlinkMsg(&msg);
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::CallbackOpticalFlow(const chroma_gazebo_plugins_msgs::OpticalFlow& _in_msg_optflow) {
  mavlink_hil_optical_flow_t mavlink_msg_optflow_sensor ;
  mavlink_msg_optflow_sensor.time_usec = HeaderToUsec(_in_msg_optflow.header);
  mavlink_msg_optflow_sensor.integration_time_us = _in_msg_optflow.integration_time_us;
  mavlink_msg_optflow_sensor.sensor_id = _in_msg_optflow.sensor_id;
  mavlink_msg_optflow_sensor.integrated_x = _in_msg_optflow.integrated_x;
  mavlink_msg_optflow_sensor.integrated_y = _in_msg_optflow.integrated_y;
  mavlink_msg_optflow_sensor.integrated_xgyro = _in_msg_optflow.quality ? -optflow_gyro_.Y() : 0.0f;//xy switched
  mavlink_msg_optflow_sensor.integrated_ygyro = _in_msg_optflow.quality ? optflow_gyro_.X() : 0.0f;  //xy switched
  mavlink_msg_optflow_sensor.integrated_zgyro = _in_msg_optflow.quality ? -optflow_gyro_.Z() : 0.0f;//change direction
  mavlink_msg_optflow_sensor.temperature = _in_msg_optflow.temperature;
  mavlink_msg_optflow_sensor.quality = _in_msg_optflow.quality;
  mavlink_msg_optflow_sensor.time_delta_distance_us = _in_msg_optflow.time_delta_distance_us;
  mavlink_msg_optflow_sensor.distance = optflow_dist_;
  mavlink_message_t msg;
  mavlink_msg_hil_optical_flow_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &mavlink_msg_optflow_sensor);
  SendMavlinkMsg(&msg);
  //reset gyro integral
  optflow_gyro_.Set();
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::CallbackIRLock(const chroma_gazebo_plugins_msgs::IRLock& _in_msg_irlock) {
  mavlink_landing_target_t mavlink_msg_landing_target;
  mavlink_msg_landing_target.time_usec = HeaderToUsec(_in_msg_irlock.header);
  mavlink_msg_landing_target.target_num = _in_msg_irlock.signature;
  mavlink_msg_landing_target.angle_x = _in_msg_irlock.pos_x;
  mavlink_msg_landing_target.angle_y = _in_msg_irlock.pos_y;
  mavlink_msg_landing_target.size_x = _in_msg_irlock.size_x;
  mavlink_msg_landing_target.size_y = _in_msg_irlock.size_y;
  mavlink_msg_landing_target.position_valid = false;
  mavlink_msg_landing_target.type = LANDING_TARGET_TYPE_LIGHT_BEACON;
  mavlink_message_t msg;
  mavlink_msg_landing_target_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &mavlink_msg_landing_target);
  SendMavlinkMsg(&msg);
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::CallbackVision(const nav_msgs::Odometry& _in_msg_visualodom) {
  // transform position from local ENU to local NED frame
  ignition::math::Vector3<double> position = Q_ENU_TO_NED.RotateVector(ignition::math::Vector3<double>(
    _in_msg_visualodom.pose.pose.position.x,
    _in_msg_visualodom.pose.pose.position.y,
    _in_msg_visualodom.pose.pose.position.z));
  // q_flu_to_enu is the quaternion that represents a rotation from ENU earth/local
  // frame to XYZ body FLU frame
  ignition::math::Quaternion<double> q_flu_to_enu = ignition::math::Quaternion<double>(
    _in_msg_visualodom.pose.pose.orientation.w,
    _in_msg_visualodom.pose.pose.orientation.x,
    _in_msg_visualodom.pose.pose.orientation.y,
    _in_msg_visualodom.pose.pose.orientation.z);
  // transform orientation from body FLU to body FRD frame:
  // q_frd_to_ned is the quaternion that represents a rotation from body FRD frame to NED earth/local frame
  ignition::math::Quaternion<double> q_frd_to_ned = Q_ENU_TO_NED * q_flu_to_enu * Q_FRD_TO_FLU;
  // Only sends ODOMETRY msgs if the protocol version is 2.0
  if (protocol_version_ == 2.0) {
    // send ODOMETRY Mavlink msg
    mavlink_odometry_t mavlink_msg_odom;
    mavlink_msg_odom.time_usec = HeaderToUsec(_in_msg_visualodom.header);
    mavlink_msg_odom.frame_id = MAV_FRAME_RESERVED_16; //MAV_FRAME_VISION_NED;
    mavlink_msg_odom.child_frame_id = MAV_FRAME_BODY_FRD;
    mavlink_msg_odom.x = position.X();
    mavlink_msg_odom.y = position.Y();
    mavlink_msg_odom.z = position.Z();
    mavlink_msg_odom.q[0] = q_frd_to_ned.W();
    mavlink_msg_odom.q[1] = q_frd_to_ned.X();
    mavlink_msg_odom.q[2] = q_frd_to_ned.Y();
    mavlink_msg_odom.q[3] = q_frd_to_ned.Z();
    // transform linear velocity from body FLU to body FRD frame
    ignition::math::Vector3<double> linear_velocity = Q_FLU_TO_FRD.RotateVector(ignition::math::Vector3<double>(
        _in_msg_visualodom.twist.twist.linear.x,
        _in_msg_visualodom.twist.twist.linear.y,
        _in_msg_visualodom.twist.twist.linear.z));
    mavlink_msg_odom.vx = linear_velocity.X();
    mavlink_msg_odom.vy = linear_velocity.Y();
    mavlink_msg_odom.vz = linear_velocity.Z();
    // transform angular velocity from body FLU to body FRD frame
    ignition::math::Vector3<double> angular_velocity = Q_FLU_TO_FRD.RotateVector(ignition::math::Vector3<double>(
      _in_msg_visualodom.twist.twist.angular.x,
      _in_msg_visualodom.twist.twist.angular.y,
      _in_msg_visualodom.twist.twist.angular.z));
    mavlink_msg_odom.rollspeed= angular_velocity.X();
    mavlink_msg_odom.pitchspeed = angular_velocity.Y();
    mavlink_msg_odom.yawspeed = angular_velocity.Z();
    // parse covariance matrices
    // The main diagonal values are always positive (variance), so a transform
    // in the covariance matrices from one frame to another would only
    // change the values of the main diagonal. Since they are all zero,
    // there's no need to apply the rotation
    size_t count = 0;
    for (size_t x = 0; x < 6; x++) {
      for (size_t y = x; y < 6; y++) {
        size_t index = 6 * x + y;
        mavlink_msg_odom.pose_covariance[count++] = _in_msg_visualodom.pose.covariance[index];
        mavlink_msg_odom.velocity_covariance[count++] = _in_msg_visualodom.twist.covariance[index];
      }
    }
    mavlink_message_t msg;
    mavlink_msg_odometry_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &mavlink_msg_odom);
    SendMavlinkMsg(&msg);
  }
  else {
    // send VISION_POSITION_ESTIMATE Mavlink msg
    mavlink_vision_position_estimate_t mavlink_msg_vision;
    mavlink_msg_vision.usec = HeaderToUsec(_in_msg_visualodom.header);
    // transform position from local ENU to local NED frame
    mavlink_msg_vision.x = position.X();
    mavlink_msg_vision.y = position.Y();
    mavlink_msg_vision.z = position.Z();
    // q_frd_to_ned is the quaternion that represents a rotation from NED earth/local
    // frame to XYZ body FRD frame
    ignition::math::Vector3<double> euler = q_frd_to_ned.Euler();
    mavlink_msg_vision.roll  = euler.X();
    mavlink_msg_vision.pitch = euler.Y();
    mavlink_msg_vision.yaw   = euler.Z();
    // parse covariance matrix
    // The main diagonal values are always positive (variance), so a transform
    // in the covariance matrix from one frame to another would only
    // change the values of the main diagonal. Since they are all zero,
    // there's no need to apply the rotation
    size_t count = 0;
    for (size_t x = 0; x < 6; x++) {
      for (size_t y = x; y < 6; y++) {
        size_t index = 6 * x + y;
        mavlink_msg_vision.covariance[count++] = _in_msg_visualodom.pose.covariance[index];
      }
    }
    mavlink_message_t msg;
    mavlink_msg_vision_position_estimate_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &mavlink_msg_vision);
    SendMavlinkMsg(&msg);
  }
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::PollMavlinkMsg(double _dt, uint32_t _timeoutMs)
{
  // convert timeout in ms to timeval
  struct timeval tv;
  tv.tv_sec = _timeoutMs / 1000;
  tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

  // poll
  ::poll(&socket_info_[0], (sizeof(socket_info_[0]) / sizeof(socket_info_[0])), 0);

  if (socket_info_[0].revents & POLLIN) {
    int len = recvfrom(socket_, buffer_, sizeof(buffer_), 0, (struct sockaddr *)&qgc_sockaddr_, &qgc_sockaddr_len_);
    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i)
      {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer_[i], &msg, &status))
        {
          if (serial_enabled_) {
            // forward message from qgc to serial
            SendMavlinkMsg(&msg);
          }
          // have a message, handle it
          HandleMavlinkMsg(&msg);
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::HandleMavlinkMsg(mavlink_message_t *msg)
{
  switch (msg->msgid) {
  case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
    mavlink_hil_actuator_controls_t controls;
    mavlink_msg_hil_actuator_controls_decode(msg, &controls);
    bool armed = false;
    if ((controls.mode & MAV_MODE_FLAG_SAFETY_ARMED) > 0) {
      armed = true;
    }
#if GAZEBO_MAJOR_VERSION >= 9
    last_actuator_time_ = world_->SimTime();
#else
    last_actuator_time_ = world_->GetSimTime();
#endif
    // set rotor speeds, controller targets
    actuators_cmd_.resize(NB_MAX_ACTUATOR);
    for (int i = 0; i < actuators_cmd_.size(); i++) {
      if (armed) {
        actuators_cmd_[i] = (controls.controls[i] + actuators_cmd_offset_[i])
            * actuators_cmd_scaling_[i] + actuators_zero_position_armed_[i];
      } else {
        actuators_cmd_[i] = actuators_zero_position_disarmed_[i];
      }
    }
    rcv_first_mavlink_msg_ = true;
    break;
  }
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::HandleActuators(double _dt) {
  // set joint positions
  if (rcv_first_mavlink_msg_ == true) {
    // Rotorspeed commanded to the rotors
    chroma_gazebo_plugins_msgs::Rotorspeed msg_rotorspeed_cmd;
    // Actuator control
    for (int i = 0; i < actuators_cmd_.size(); i++) {
      // Joints
      if (actuators_joint_[i]) {
        if (actuators_cmd_type_[i] == "angular_speed") {
          double actuator_vel = actuators_joint_[i]->GetVelocity(0);
          double err = actuator_vel - actuators_cmd_[i];
          double force = actuators_pid_[i].Update(err, _dt);
          actuators_joint_[i]->SetForce(0, force);
        }
        else if (actuators_cmd_type_[i] == "angular_position") {
  #if GAZEBO_MAJOR_VERSION >= 9
          double actuator_pos = actuators_joint_[i]->Position(0);
  #else
          double actuator_pos = actuators_joint_[i]->GetAngle(0).Radian();
  #endif
          double err = actuator_pos - actuators_cmd_[i];
          double force = actuators_pid_[i].Update(err, _dt);
          actuators_joint_[i]->SetForce(0, force);
        }
        else {
          gzerr << "actuators_cmd_type[" << actuators_cmd_type_[i] << "] undefined.\n";
        }
      }
      // Rotors
      msg_rotorspeed_cmd.values.push_back(0);
      if (actuators_cmd_type_[i] == "rotor_speed"){
        if (last_actuator_time_ != 0){
          msg_rotorspeed_cmd.values[i] = actuators_cmd_[i];
        }
      }
    }
    msg_rotorspeed_cmd.header.stamp.sec = current_time_.sec;
    msg_rotorspeed_cmd.header.stamp.nsec = current_time_.nsec;
    pub_rotorspeed_cmd_.publish(msg_rotorspeed_cmd);
  }
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::OpenSerialInterface() {
  try{
    serial_port_.open(serial_device_);
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    gzdbg << "Opened serial device " << serial_device_ << "\n";
  }
  catch (boost::system::system_error &err) {
    gzerr <<"Error opening serial device: " << err.what() << "\n";
  }
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::CloseSerialInterface()
{
  lock_guard lock(mutex_);
  if (!serial_port_.is_open())
    return;

  io_service_.stop();
  serial_port_.close();

  if (io_thread_.joinable())
    io_thread_.join();
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::ReadSerialInterface(void)
{
  serial_port_.async_read_some(boost::asio::buffer(rx_buffer_), boost::bind(
      &MavlinkInterfacePlugin::ParseSerialBuffer, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred
      )
  );
}

/////////////////////////////////////////////////
// Based on MAVConnInterface::parse_buffer in MAVROS
void MavlinkInterfacePlugin::ParseSerialBuffer(const boost::system::error_code& err, std::size_t bytes_t){
  mavlink_status_t status;
  mavlink_message_t message;
  uint8_t *buffer = this->rx_buffer_.data();

  assert(rx_buffer_.size() >= bytes_t);

  for(; bytes_t > 0; bytes_t--)
  {
    auto c = *buffer++;

    auto msg_received = static_cast<Framing>(mavlink_frame_char_buffer(&mavlink_msg_, &mavlink_status_, c, &message, &status));
    if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
      _mav_parse_error(&mavlink_status_);
      mavlink_status_.msg_received = MAVLINK_FRAMING_INCOMPLETE;
      mavlink_status_.parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (c == MAVLINK_STX) {
        mavlink_status_.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
        mavlink_msg_.len = 0;
        mavlink_start_checksum(&mavlink_msg_);
      }
    }

    if(msg_received != Framing::incomplete){
      // send to gcs
      SendMavlinkMsg(&message, qgc_port_);
      HandleMavlinkMsg(&message);
    }
  }
  ReadSerialInterface();
}

/////////////////////////////////////////////////
void MavlinkInterfacePlugin::WriteSerialInterface(bool check_tx_state){
  if (check_tx_state && tx_in_progress_)
    return;

  lock_guard lock(mutex_);
  if (tx_buffer_.empty())
    return;

  tx_in_progress_ = true;
  auto &buf_ref = tx_buffer_.front();

  serial_port_.async_write_some(
    boost::asio::buffer(buf_ref.dpos(), buf_ref.nbytes()), [this, &buf_ref] (boost::system::error_code error,   size_t bytes_transferred)
    {
      assert(bytes_transferred <= buf_ref.len);
      if(error) {
        gzerr << "Serial error: " << error.message() << "\n";
      return;
      }

    lock_guard lock(mutex_);

    if (tx_buffer_.empty()) {
      tx_in_progress_ = false;
      return;
    }

    buf_ref.pos += bytes_transferred;
    if (buf_ref.nbytes() == 0) {
      tx_buffer_.pop_front();
    }

    if (!tx_buffer_.empty()) {
      WriteSerialInterface(false);
    }
    else {
      tx_in_progress_ = false;
    }
  });
}

}
