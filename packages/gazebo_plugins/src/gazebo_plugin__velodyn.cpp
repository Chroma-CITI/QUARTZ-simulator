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


#include "gazebo_plugins/gazebo_plugin__velodyn.h"

namespace gazebo {

Eigen::Affine3f IgnitionQuaternionToEigen(const ignition::math::Quaternion<double> &quaternion) {
  Eigen::Affine3f rpsi = Eigen::Affine3f(Eigen::AngleAxisf(quaternion.Yaw(), Eigen::Vector3f(0, 0, 1)));
  Eigen::Affine3f rtet = Eigen::Affine3f(Eigen::AngleAxisf(quaternion.Pitch(), Eigen::Vector3f(0, 1, 0)));
  Eigen::Affine3f rphi = Eigen::Affine3f(Eigen::AngleAxisf(quaternion.Roll(), Eigen::Vector3f(1, 0, 0)));
  return rpsi * rtet * rphi;
}

Eigen::Affine3f IgnitionPoseToEigen(const ignition::math::Vector3<double> &pose) {
  return Eigen::Affine3f(Eigen::Translation3f(Eigen::Vector3f(
    pose.X(), pose.Y(), pose.Z() )));
}

Eigen::Matrix4f IgnitionStateToEigen(const ignition::math::Pose3<double> &state) {
  ignition::math::Vector3<double> pose = state.Pos();
  ignition::math::Quaternion<double> quaternion = state.Rot();
  Eigen::Affine3f r = IgnitionQuaternionToEigen(quaternion);
  Eigen::Affine3f t = IgnitionPoseToEigen(pose);
  Eigen::Matrix4f transform = (t * r).matrix();
  return transform;
}

void transformPointCloud (
  const ignition::math::Pose3<double> &state,
  const sensor_msgs::PointCloud2 &in,
        sensor_msgs::PointCloud2 &out)
{
  // Ignition -> Eigen
  Eigen::Matrix4f transform = IgnitionStateToEigen(state);
  // pcl_ros::transformPointCloud(transform, in, out);
  // Get X-Y-Z indices
  int x_idx = pcl::getFieldIndex (in, "x");
  int y_idx = pcl::getFieldIndex (in, "y");
  int z_idx = pcl::getFieldIndex (in, "z");
  if (x_idx == -1 || y_idx == -1 || z_idx == -1)
  {
    ROS_ERROR ("Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.");
    return;
  }
  if (in.fields[x_idx].datatype != sensor_msgs::PointField::FLOAT32 ||
      in.fields[y_idx].datatype != sensor_msgs::PointField::FLOAT32 ||
      in.fields[z_idx].datatype != sensor_msgs::PointField::FLOAT32)
  {
    ROS_ERROR ("X-Y-Z coordinates not floats. Currently only floats are supported.");
    return;
  }
  // Check if distance is available
  int dist_idx = pcl::getFieldIndex (in, "distance");
  // Copy the other data
  if (&in != &out)
  {
    out.header = in.header;
    out.height = in.height;
    out.width  = in.width;
    out.fields = in.fields;
    out.is_bigendian = in.is_bigendian;
    out.point_step   = in.point_step;
    out.row_step     = in.row_step;
    out.is_dense     = in.is_dense;
    out.data.resize (in.data.size ());
    // Copy everything as it's faster than copying individual elements
    memcpy (&out.data[0], &in.data[0], in.data.size ());
  }
  Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);
  for (size_t i = 0; i < in.width * in.height; ++i)
  {
    Eigen::Vector4f pt (*(float*)&in.data[xyz_offset[0]], *(float*)&in.data[xyz_offset[1]], *(float*)&in.data[xyz_offset[2]], 1);
    Eigen::Vector4f pt_out;

    bool max_range_point = false;
    int distance_ptr_offset = i*in.point_step + in.fields[dist_idx].offset;
    float* distance_ptr = (dist_idx < 0 ? NULL : (float*)(&in.data[distance_ptr_offset]));
    if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2]))
    {
      if (distance_ptr==NULL || !std::isfinite(*distance_ptr))  // Invalid point
      {
        pt_out = pt;
      }
      else  // max range point
      {
        pt[0] = *distance_ptr;  // Replace x with the x value saved in distance
        pt_out = transform * pt;
        max_range_point = true;
        //std::cout << pt[0]<<","<<pt[1]<<","<<pt[2]<<" => "<<pt_out[0]<<","<<pt_out[1]<<","<<pt_out[2]<<"\n";
      }
    }
    else
    {
      pt_out = transform * pt;
    }
    if (max_range_point)
    {
      // Save x value in distance again
      *(float*)(&out.data[distance_ptr_offset]) = pt_out[0];
      pt_out[0] = std::numeric_limits<float>::quiet_NaN();
    }
    memcpy (&out.data[xyz_offset[0]], &pt_out[0], sizeof (float));
    memcpy (&out.data[xyz_offset[1]], &pt_out[1], sizeof (float));
    memcpy (&out.data[xyz_offset[2]], &pt_out[2], sizeof (float));
    xyz_offset += in.point_step;
  }
  // Check if the viewpoint information is present
  int vp_idx = pcl::getFieldIndex (in, "vp_x");
  if (vp_idx != -1)
  {
    // Transform the viewpoint info too
    for (size_t i = 0; i < out.width * out.height; ++i)
    {
      float *pstep = (float*)&out.data[i * out.point_step + out.fields[vp_idx].offset];
      // Assume vp_x, vp_y, vp_z are consecutive
      Eigen::Vector4f vp_in (pstep[0], pstep[1], pstep[2], 1);
      Eigen::Vector4f vp_out = transform * vp_in;
      pstep[0] = vp_out[0];
      pstep[1] = vp_out[1];
      pstep[2] = vp_out[2];
    }
  }
}

/////////////////////////////////////////////////
VelodynModel::VelodynModel() : ModelPlugin(){
}

/////////////////////////////////////////////////
VelodynModel::~VelodynModel() {
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
void VelodynModel::Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf) {
  // Save the model
  model_ = _in_model;
  // Get the world
  world_ = model_->GetWorld();
  // get robot namespace param
  getSdfParam<std::string>(_in_sdf, "robot_namespace", namespace_, KDEFAULT_NAMESPACE);
  // get link name param
  getSdfParam<std::string>(_in_sdf, "link_name", link_name_, "");
  getSdfParam<std::string>(_in_sdf, "base_link_name", base_link_name_, "");
  // Get the pointer to the link.
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("Couldn't find specified link \"" << link_name_ << "\".");
  base_link_ = model_->GetLink(base_link_name_);
  if (base_link_ == NULL)
    gzthrow("Couldn't find specified link \"" << base_link_name_ << "\".");
  // get joint name param
  getSdfParam<std::string>(_in_sdf, "joint_name", joint_name_, "");
  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("Couldn't find specified joint \"" << joint_name_ << "\".");
  // get ros topic names
  getSdfParam<std::string>(_in_sdf, "topic_laser_scan", topic_laser_scan_, KDEFAULT_TOPIC_LASER_SCAN);
  getSdfParam<std::string>(_in_sdf, "topic_velodyn_scan", topic_velodyn_scan_, KDEFAULT_TOPIC_VELODYN_SCAN);
  getSdfParam<std::string>(_in_sdf, "frame_velodyn_scan", frame_velodyn_scan_, KDEFAULT_FRAME_VELODYN_SCAN);
  // Frequence
  double update_rate;
  getSdfParam<double>(_in_sdf, "update_rate", update_rate, KDEFAULT_UPDATE_RATE);
  if (update_rate > 0) update_dt_ = 1.0/update_rate;
  else update_dt_ = 1.0/KDEFAULT_UPDATE_RATE;
  // Vitesse de rotation
  joint_->SetVelocity(0, 2.0*M_PI / update_dt_);
  // Initialisation of the time reference
  #if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  #else
  last_time_ = world_->GetSimTime();
  #endif
  // Gazebo initialisation
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(std::bind(&VelodynModel::OnUpdate, this, std::placeholders::_1));
  #else
  update_connection_ptr_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&VelodynModel::OnUpdate, this, _1));
  #endif
  // ROS initialisation
  node_handle_ = new ros::NodeHandle(namespace_);
  sub_laser_scan_ = node_handle_->subscribe(topic_laser_scan_, 1, &VelodynModel::CallbackLaserScan, this);
  pub_velodyn_scan_ = node_handle_->advertise<sensor_msgs::PointCloud2>(topic_velodyn_scan_, 10);
  // Controle execution
  ok_thread_ = true;
}

void VelodynModel::CallbackLaserScan(const sensor_msgs::PointCloud2::ConstPtr& msg_pc2_laser_scan) {
  // Get current model orientation
  #if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3<double> state = link_->WorldPose();
  #else
  ignition::math::Pose3<double> state = GazeboToIgnition(link_->GetWorldPose());
  #endif
  // transform the point cloud into world_frame
  sensor_msgs::PointCloud2 msg_pc2_laser_scan_wf;
  transformPointCloud(
    state, *msg_pc2_laser_scan, msg_pc2_laser_scan_wf);
  // Convert ROS point cloud to PCL point cloud
  // See http://wiki.ros.org/hydro/Migration for the source of this magic.
  pcl::PCLPointCloud2 pc2_laser_scan_wf;
  pcl_conversions::toPCL(msg_pc2_laser_scan_wf, pc2_laser_scan_wf);
  // Convert point cloud to PCL native point cloud
  pcl::PointCloud<pcl::PointXYZ> pc_laser_scan_wf;
  pcl::fromPCLPointCloud2(pc2_laser_scan_wf, pc_laser_scan_wf);
  // Controle execution
  while (!ok_thread_)
    usleep(10);
  ok_thread_ = false;
  // Append
  pc_laser_scans_wf_.push_back(pc_laser_scan_wf);
  // Controle execution
  ok_thread_ = true;
}

/////////////////////////////////////////////////
// This gets called by the world update start event.
void VelodynModel::OnUpdate(const common::UpdateInfo& _info) {
  // Publish condition
  #if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
  #else
  common::Time current_time = world_->GetSimTime();
  #endif
  // time step
  double dt = (current_time - last_time_).Double();
  // Frequency condition
  if (dt >= update_dt_){
    Publish();
    last_time_ = current_time;
  }
}

/////////////////////////////////////////////////
void VelodynModel::Publish() {
  // Time
  #if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
  #else
  common::Time current_time = world_->GetSimTime();
  #endif
  // Get current model orientation
  #if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3<double> base_state = base_link_->WorldPose();
  #else
  ignition::math::Pose3<double> base_state = GazeboToIgnition(base_link_->GetWorldPose());
  #endif
  // Controle execution
  while (!ok_thread_)
    usleep(10);
  ok_thread_ = false;
  // Laserscan points fused
  pcl::PointCloud<pcl::PointXYZ> pc_velodyn_scan_wf;
  for (pcl::PointCloud<pcl::PointXYZ> pc_laser_scan_wf: pc_laser_scans_wf_)
    pc_velodyn_scan_wf += pc_laser_scan_wf;
  pc_laser_scans_wf_.clear();
  // Controle execution
  ok_thread_ = true;
  // auto it_end_pc_laser_scans_wf = pc_laser_scans_wf_.end();
  // for (auto it = pc_laser_scans_wf_.begin(); it != it_end_pc_laser_scans_wf; it++) {
  //   pc_velodyn_scan_wf += *it;
  //   pc_laser_scans_wf_.erase(it--);
  // }
  // Convert to pointcloud 2
  pcl::PCLPointCloud2 pc2_velodyn_scan_wf;       // wf stands for world frame
  pcl::toPCLPointCloud2(pc_velodyn_scan_wf, pc2_velodyn_scan_wf);
  // Convert to ros msg
  sensor_msgs::PointCloud2 msg_pc2_velodyn_scan_wf; // wf stands for world frame
  pcl_conversions::fromPCL(pc2_velodyn_scan_wf, msg_pc2_velodyn_scan_wf);
  // transfert in velodyne frame
  sensor_msgs::PointCloud2 msg_pc2_velodyn_scan_vf; // vf stands for Velodyn frame
  transformPointCloud(
    -base_state, msg_pc2_velodyn_scan_wf, msg_pc2_velodyn_scan_vf);
  // Publish the data
  msg_pc2_velodyn_scan_vf.header.frame_id = namespace_ + frame_velodyn_scan_;
  msg_pc2_velodyn_scan_vf.header.stamp.sec = current_time.sec;
  msg_pc2_velodyn_scan_vf.header.stamp.nsec = current_time.nsec;
  pub_velodyn_scan_.publish(msg_pc2_velodyn_scan_vf);
}

GZ_REGISTER_MODEL_PLUGIN(VelodynModel);
}
