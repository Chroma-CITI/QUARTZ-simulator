/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 * Copyright 2020 Cedric Pradalier, Associate Prof. GeorgiaTech Lorraine
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

#include "gazebo_plugins/gazebo_plugin__body_auv.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BodyAUVPlugin);

/////////////////////////////////////////////////
BodyAUVPlugin::BodyAUVPlugin() : BodyPlugin() {
    last_wrench_msg_.force.x = 0.0;
    last_wrench_msg_.force.y = 0.0;
    last_wrench_msg_.force.z = 0.0;
    last_wrench_msg_.torque.x = 0.0;
    last_wrench_msg_.torque.y = 0.0;
    last_wrench_msg_.torque.z = 0.0;
}

/////////////////////////////////////////////////
BodyAUVPlugin::~BodyAUVPlugin() {
}

/////////////////////////////////////////////////
void BodyAUVPlugin::Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf)  {
    // Load parameters
    getSdfParam<double>(_in_sdf, "Cx", Cx_, 10.0);
    getSdfParam<double>(_in_sdf, "Cw", Cw_, 10.0);
    getSdfParam<double>(_in_sdf, "buoyancy", buoyancy_, 1.05);
    getSdfParam<double>(_in_sdf, "buoyancy_offset", buoyancy_offset_, 0.01);
    // Parent load
    BodyPlugin::Load(_in_model, _in_sdf);
    // Others ROS publisher
    last_usbl_pub_ = time_;
    usbl_pub_ = node_handle_->advertise<geometry_msgs::PointStamped>("/" + robot_namespace_ + "/usbl", 1);
    heading_pub_ = node_handle_->advertise<std_msgs::Float32>("/" + robot_namespace_ + "/heading", 1);
    // Others ROS subscribers
    wrench_sub_ = node_handle_->subscribe<geometry_msgs::Wrench>("/" + robot_namespace_ + "/cmd_wrench", 1, &BodyAUVPlugin::wrenchCb,this);
}

/////////////////////////////////////////////////
void BodyAUVPlugin::wrenchCb(const geometry_msgs::WrenchConstPtr & msg) {
    last_wrench_msg_ = *msg;
}

/////////////////////////////////////////////////
void BodyAUVPlugin::OnUpdate() {
    // Get state and time
    GetStateAndTime();
    // publish USBL & Heading message @5Hz
    if ((time_ - last_usbl_pub_).Double() > 0.2) {
        geometry_msgs::PointStamped usbl;
        usbl.header.stamp.sec = time_.sec;
        usbl.header.stamp.nsec = time_.nsec;
        usbl.header.frame_id = "world";
        usbl.point.x = state_.Pos()[0];
        usbl.point.y = state_.Pos()[1];
        usbl.point.z = state_.Pos()[2];
        usbl_pub_.publish(usbl);
        ignition::math::Vector3d euler = state_.Rot().Euler();
        std_msgs::Float32 mag;
        mag.data = euler[2];
        heading_pub_.publish(mag);
        last_usbl_pub_ = time_;
    }
    #if GAZEBO_MAJOR_VERSION >= 11
    ignition::math::AxisAlignedBox b = link_->BoundingBox();
    #elif GAZEBO_MAJOR_VERSION >= 9 
    ignition::math::Box b = link_->BoundingBox();
    #else
    ignition::math::Box b = GazeboToIgnition(link_->GetBoundingBox());
    #endif
    float submerged_portion = buoyancy_;
    if (b.Min()[2] > 0) {
        submerged_portion = 0.0;
    } else if (b.Max()[2] > 0) {
        submerged_portion *= (1.0 - b.Max()[2] / b.ZLength());
    }
    #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d vel = link_->WorldCoGLinearVel();
    #else
    ignition::math::Vector3<double> vel = GazeboToIgnition(link_->GetWorldCoGLinearVel());
    #endif
    float speed = vel.Length();
    vel = -vel.Normalize();
    #if GAZEBO_MAJOR_VERSION >= 9
    link_->AddForceAtRelativePosition(-submerged_portion * link_->GetInertial()->Mass() * link_->GetWorld()->Gravity(),
            ignition::math::Vector3d(0,0,buoyancy_offset_));
    #else
    link_->AddForceAtRelativePosition(-submerged_portion * link_->GetInertial()->GetMass() * link_->GetWorld()->Gravity(),
            gazebo::math::Vector3(0,0,buoyancy_offset_));
    #endif
    link_->AddForce(Cx_*speed*vel);
    link_->AddRelativeForce(ignition::math::Vector3d(last_wrench_msg_.force.x,0,last_wrench_msg_.force.z));
    link_->AddRelativeTorque(ignition::math::Vector3d(0,last_wrench_msg_.torque.y,last_wrench_msg_.torque.z));
    #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d omega = link_->WorldAngularVel();
    #else
    ignition::math::Vector3<double> omega = GazeboToIgnition(link_->GetWorldAngularVel());
    #endif
    link_->AddTorque(-Cw_*omega);
    // send ground truth information
    SendGroundTruth();
}
