/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 * Copyright 2018-2020 Vincent LE DOZE, INRIA, Chroma Team, CITI Lab Lyon, France
 * Copyright 2020-2021 Vincent DUFOUR, INRIA, Chroma Team, CITI Lab Lyon, France
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

#include "gazebo_plugins/gazebo_plugin__uwb.h"
#include "ros/ros.h"

using namespace gazebo;
using namespace std::chrono_literals;

/////////////////////////////////////////////////
bool is_in(
  std::vector<std::string> const& list_name,
  std::string const& name)
{
  for (auto it = list_name.begin(); it != list_name.end(); ++it) {
    if (it->compare(name) == 0)
      return true;
  }
  return false;
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(UwbPlugin);

/////////////////////////////////////////////////
UwbPlugin::UwbPlugin() : ModelPlugin() {
  has_comm_token_ = false;
  has_next_token_owner_ = false;
}

/////////////////////////////////////////////////
UwbPlugin::~UwbPlugin() {
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_.reset();
  #else
  event::Events::DisconnectWorldUpdateBegin(update_connection_ptr_);
  #endif
  test_ray_.reset();
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

//////////////////////////////////////////////////
void UwbPlugin::Reset() {
  has_comm_token_ = false;
  has_next_token_owner_ = false;
  previous_comms_.clear();
  previous_token_users_.clear();
  friend_list_names_.clear();
  friend_list_links_.clear();
  neighbor_list.clear();
  loaded_once = false;
}

/////////////////////////////////////////////////
void UwbPlugin::Load(physics::ModelPtr _in__model, sdf::ElementPtr _in__sdf) {
  // Get the world
  world_ = _in__model->GetWorld();
  // Get the robot namespace
  getSdfParam<std::string>(_in__sdf, "robot_namespace", robot_namespace_, "");
  // Get link name
  getSdfParam<std::string>(_in__sdf, "link_name", link_name_, "");
  // Get the pointer to the link
  link_ = _in__model->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("Couldn't find specified link \"" << link_name_ << "\".");
  // Get frequency
  getSdfParam<double>(_in__sdf, "freq", freq_, KDEFAULT_FREQ);
  // Get the Antenna properties
  getSdfParam<std::string>(_in__sdf, "essid",  essid_,  KDEFAULT_ESSID);
  getSdfParam<double>(_in__sdf, "rx_gain",  rx_gain_,  KDEFAULT_RX_GAIN);
  getSdfParam<double>(_in__sdf, "rx_power", rx_power_, KDEFAULT_RX_POWER);
  getSdfParam<double>(_in__sdf, "tx_gain",  tx_gain_,  KDEFAULT_TX_GAIN);
  getSdfParam<double>(_in__sdf, "tx_power", tx_power_, KDEFAULT_TX_POWER);
  getSdfParam<double>(_in__sdf, "tx_freq",  tx_freq_,  KDEFAULT_TX_FREQ);
  getSdfParam<double>(_in__sdf, "stddev_per_meter",  stddev_per_meter_,  KDEFAULT_STDDEV_PER_METERS);
  // Get the topic
  getSdfParam<std::string>(_in__sdf, "uwb_topic", uwb_topic_, KDEFAULT_UWB_TOPIC);
  getSdfParam<std::string>(_in__sdf, "odom_topic", odom_topic_, KDEFAULT_ODOM_TOPIC);
  //Wait for other uwb_plugins
  std::this_thread::sleep_for(1000ms);
  // Initialisation of the time reference
  #if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  #else
  last_time_ = world_->GetSimTime();
  #endif
  // Gazebo connection
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(std::bind(&UwbPlugin::OnUpdate, this));
  #else
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&UwbPlugin::OnUpdate, this));
  #endif
  // This ray will be used in GetMesurements() for checking obstacles
  // between the transmitter and a given point.
  #if GAZEBO_MAJOR_VERSION >= 9
  test_ray_ = boost::dynamic_pointer_cast<physics::RayShape>(
     world_->Physics()->CreateShape("ray", physics::CollisionPtr()));
  test_ray_->SetWorld(world_);
  #else
  test_ray_ = boost::dynamic_pointer_cast<physics::RayShape>(
     world_->GetPhysicsEngine()->CreateShape("ray", physics::CollisionPtr()));
  test_ray_->SetWorld(world_);
  #endif
  // Mesurement noise
  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 2.0);
  // ROS handler
  node_handle_ = new ros::NodeHandle(robot_namespace_);
  // ROS publisher
  uwb_pub_ = node_handle_->advertise<chroma_gazebo_plugins_msgs::Uwb>("/" + robot_namespace_ + "/" + uwb_topic_, 1);
  // ROS subscriber
  odom_sub_ = node_handle_->subscribe(odom_topic_, 1, &UwbPlugin::CallbackOdom, this);
  // ROS service
  status_srv_ = node_handle_->advertiseService("/" + link_name_ + "/status", &UwbPlugin::CallbackStatus, this);
  token_srv_ = node_handle_->advertiseService("/" + link_name_ + "/get_token", &UwbPlugin::CallbackGetToken, this);
  count = 0;
  global_count = 0;
}

/////////////////////////////////////////////////
// \brief This gets called by the world update start event.
void UwbPlugin::OnUpdate() {
  // Initialization of link with friend (do it only once at the first call OnUpdate)
  // models crawling
  if(!loaded_once){
    #if GAZEBO_MAJOR_VERSION >= 9
    physics::Model_V models = world_->Models();
    #else
    physics::Model_V models = world_->GetModels();
    #endif
    std::string friend_link_name;
    //gzmsg << "Current link: " << link_name_ << "\n";
    for (physics::Model_V::iterator it_model = models.begin(); it_model  != models.end(); ++it_model ) {
      // Not self
      if (!std::regex_match( (*it_model)->GetName(), std::regex("(.*)"+robot_namespace_+"(.*)") ) ) {
        // link crawlink
        physics::Link_V links = (*it_model)->GetLinks();
        //physics::Link_V current_link;
        for (physics::Link_V::iterator it_link = links.begin(); it_link != links.end(); ++it_link) {
          // Find other antenna link
          if ( std::regex_match( (*it_link)->GetName(), std::regex("(.*)(_uwb_antenna_)(.*)") ) ) {
            // New antenna
            friend_link_name = (*it_link)->GetName();
            // Test if has already Communicated with
            if (is_in(friend_list_names_, friend_link_name))
              continue;
            else
              friend_list_names_.push_back(friend_link_name);
              friend_list_links_.push_back(*it_link);
               // #ifdef DEBUG
               //   gzmsg << "Link registered: " << friend_link_name << "\n";
               // #endif
          }
        }
      }
    }
    loaded_once = true;
  }
  // Get current time
  #if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
  #else
  common::Time current_time = world_->GetSimTime();
  #endif
  // time step
  double dt = (current_time - last_time_).Double();
  // Current friend link
  std::string current_friend_name;
  // Has right to exchange message
  if (has_comm_token_) {
    // Exchange frequency
    if (dt >= 1.0/freq_) {
    // Current pose
      #if GAZEBO_MAJOR_VERSION >= 9
      ignition::math::Pose3<double> self_state = link_->WorldPose();
      #else
      ignition::math::Pose3<double> self_state = GazeboToIgnition(link_->GetWorldPose());
      #endif
      // Choose next link to talk to
      if(count < friend_list_names_.size()) {
        current_friend_name = friend_list_names_[count];
        // Test if has already Communicated with
        // if (is_in(previous_comms_, current_friend_name))
        //   return;
        // else
        //   previous_comms_.push_back(current_friend_name);
        // Get friend state
        #if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Pose3<double> friend_state = (friend_list_links_[count])->WorldPose();
        #else
        ignition::math::Pose3<double> friend_state = GazeboToIgnition((friend_list_links_[count])->GetWorldPose());
        #endif
        // Get distance and received power
        double receivd_power, measured_distance;
        GetMesurements(self_state.Pos(), friend_state.Pos(), current_friend_name, receivd_power, measured_distance);
        //gzmsg<<"Get Mesurement passed\n";
        // Check if we can get the signal
        if (receivd_power >= rx_power_) {
          // Debug
          // #ifdef DEBUG
          // gzmsg << ": Connected\n";
          // #endif
          // Get friend status
          ros::ServiceClient status_scl = node_handle_->serviceClient<chroma_gazebo_plugins_msgs::UwbStatus>("/"+ current_friend_name + "/status");
          chroma_gazebo_plugins_msgs::UwbStatus friend_status;
          status_scl.call(friend_status);
          // Get current time
          #if GAZEBO_MAJOR_VERSION >= 9
          common::Time current_time = world_->SimTime();
          #else
          common::Time current_time = world_->GetSimTime();
          #endif
          // Create message
          chroma_gazebo_plugins_msgs::Uwb uwb_msg;
          uwb_msg.header.stamp.sec = current_time.sec;
          uwb_msg.header.stamp.nsec = current_time.nsec;
          uwb_msg.header.frame_id = link_name_;
          uwb_msg.essid = essid_;
          uwb_msg.friend_essid = friend_status.response.essid;
          uwb_msg.friend_odom = friend_status.response.odom;
          uwb_msg.distance = measured_distance;
          uwb_msg.receivd_pwd = receivd_power;
          uwb_pub_.publish(uwb_msg);
          // Save current time
          last_time_ = current_time;
        }
        // Debug
        #ifdef DEBUG
        else
          gzmsg << ": Unreached (Rx = " << receivd_power << "dBm / "<< rx_power_ << "dBm)\n";
        #endif
        previous_comms_.push_back(current_friend_name);
        neighbor_list.push_back(current_friend_name);
        count+=1;
        //gzmsg << "Drone: " << link_name_ << "\n";
        //std::cout<<"Count: " << count << std::endl;
        //std::cout<<"Neighbor list size: " << neighbor_list.size() << std::endl;
      }
      else {
        // neighbor_list.clear();
        // count = 0;
        // return;
        global_count+=1;
        #ifdef DEBUG
        gzmsg << "Global count: " << global_count << "\n";
        #endif
        // Chose next antenna to get token
        if (!has_next_token_owner_) {
          for (auto friend_name : friend_list_names_) {
            if (!is_in(previous_token_users_, friend_name)) {
              has_next_token_owner_ = true;
              next_token_owner_ = friend_name;
              previous_token_users_.push_back(link_name_);
              break;
            }
          }
          if (!has_next_token_owner_) {
            previous_token_users_.clear();
            next_token_owner_ = friend_list_names_[0];
            #ifdef DEBUG
            gzmsg << "clear round \n";
            #endif
          }
          ros::ServiceClient token_scl = node_handle_->serviceClient<chroma_gazebo_plugins_msgs::UwbToken>("/" + next_token_owner_ + "/get_token");
          chroma_gazebo_plugins_msgs::UwbToken token_msg;
          token_msg.request.previous_token_users = previous_token_users_;
          token_scl.call(token_msg);
            // Reicurrent_friend_namenit
          has_comm_token_ = false;
          has_next_token_owner_ = false;
          previous_token_users_.clear();
          previous_comms_.clear();
          neighbor_list.clear();
          count = 0;
        }
      }
    }
  }
}

/////////////////////////////////////////////////
// Calcul à partir des modèles de FreeSpacePathLoss et Log-Distance Path Loss
//Formules Papier Alexandre Bonnefond
double UwbPlugin::PathLossComputation(double measured_distance,double d_obst)
{
  double PL0 = 20*log10(DEFAULT_D0)+20*log10(tx_freq_)+32.44; // pour d0 en mètre et f en GHz
  double PLobst = 10*NOBSTACLE*log10(d_obst); // K=0, négligeable donc pas inséré dans la formule
  double Xg = stddev_per_meter_*standard_normal_distribution_(random_generator_);
  double TotalPL;

  if ( (measured_distance - d_obst) > DEFAULT_D0)
    TotalPL = PL0 + 10*NEMPTY*log10((measured_distance-d_obst)/DEFAULT_D0) + Xg + PLobst;
  else
    TotalPL = PL0 + PLobst;

  return TotalPL;
}

/////////////////////////////////////////////////
void UwbPlugin::GetMesurements(
    const ignition::math::Vector3<double>& emitter_pos,
    const ignition::math::Vector3<double>& receivr_pos,
    std::string &friend_name,
          double& receivd_power,
          double& measured_distance)
{
  // Distance mesurée sans obstacle
  double real_distance = emitter_pos.Distance(receivr_pos);
  if (real_distance < 0.01)
  {
    receivd_power = tx_power_;
    measured_distance = 0.0;
    return;
  }

  measured_distance = real_distance * (1.0 + stddev_per_meter_*standard_normal_distribution_(random_generator_));
  // Acquire the mutex for avoiding race condition with the physics engine
  #if GAZEBO_MAJOR_VERSION >= 9
  boost::recursive_mutex::scoped_lock lock( *(world_->Physics()->GetPhysicsUpdateMutex()) );
  #else
  boost::recursive_mutex::scoped_lock lock( *(world_->GetPhysicsEngine()->GetPhysicsUpdateMutex()) );
  #endif
  // Looking for obstacles between start and end points
  double n = NEMPTY;   //value of n depending on the obstacles between Tx and Rx
  double obstacle_distance;
  std::string obstacle_entity;
  std::string identifier;
  double d_obst;
  // Put the starting point of ray out of the drone's collision box (avoid self colliding)
  ignition::math::Vector3<double> new_emitter = emitter_pos;
  //sdf::ElementPtr caract;
  #if GAZEBO_MAJOR_VERSION > 9
  ignition::math::AxisAlignedBox caract;
  #else
  ignition::math::Box caract;
  #endif

  new_emitter[2]+=0.5;
  new_emitter[1]+=0.5;
  new_emitter[0]+=0.5;
  test_ray_->SetPoints(new_emitter, receivr_pos);
  test_ray_->GetIntersection(obstacle_distance, obstacle_entity);
  //std::cout << "obstacle_entity: " << obstacle_entity << std::endl;
  // ToDo: The ray intersects with my own collision model. Fix it.
  for(int i = 0; i<9;i++)
    identifier.push_back(friend_name[i]);

  if (obstacle_entity != "" && obstacle_entity.rfind(identifier,0) != 0){
    //calcul de d_obst --> récupérer longueur de l'objet
    physics::Model_V models = world_->Models();
    for (physics::Model_V::iterator it_model = models.begin(); it_model  != models.end(); ++it_model )
      {
        //if ((*it_model)->GetName() == obstacle_entity)
        if (obstacle_entity.rfind((*it_model)->GetName(),0) == 0)
        {
          caract = (*it_model)->BoundingBox();
          d_obst = caract.Size().Max();
          break;
        }
      }
  }
  else{
    d_obst=1;}
  // Hata-Okumara propagation model - Litteral application
  /*
  #if GAZEBO_MAJOR_VERSION >= 9
  double x = abs(ignition::math::Rand::DblNormal(0.0, MODELSTDDESV));
  #else
  double x = abs(math::Rand::GetDblNormal(0.0, MODELSTDDESV));
  #endif
  double wavelength = common::SpeedOfLight / (tx_freq_ * 1000000);
  receivd_power = tx_power_ + tx_gain_ + rx_gain_ - x +
      20*log10(wavelength) - 20*log10(4 * M_PI) - 10*n*log10(measured_distance);
  */

  //Version from Alexandre Bonnefond's Paper
  double PL = PathLossComputation(measured_distance,d_obst);
  receivd_power = tx_power_ + tx_gain_ + rx_gain_ - PL;

  //std::cout<<"Power: " << receivd_power << std::endl;
  //std::cout<<"Obstacle: " << d_obst << std::endl;

}

/////////////////////////////////////////////////
void UwbPlugin::CallbackOdom(const nav_msgs::Odometry::ConstPtr& _odom_msg) {
  odom_msg_ = _odom_msg;
}

/////////////////////////////////////////////////
bool UwbPlugin::CallbackStatus(
  chroma_gazebo_plugins_msgs::UwbStatus::Request  &req,
  chroma_gazebo_plugins_msgs::UwbStatus::Response &res)
{
  if (odom_msg_ != NULL)
    res.odom = *odom_msg_;
  res.essid = essid_;
  return true;
}

/////////////////////////////////////////////////
bool UwbPlugin::CallbackGetToken(
  chroma_gazebo_plugins_msgs::UwbToken::Request  &req,
  chroma_gazebo_plugins_msgs::UwbToken::Response &res)
{
  has_comm_token_ = true;
  has_next_token_owner_ = false;
  previous_token_users_.clear();
  previous_comms_.clear();
  previous_token_users_ = req.previous_token_users;
  #ifdef DEBUG
  gzmsg << "TOKEN @ " << essid_ << "\n";
  #endif
}
