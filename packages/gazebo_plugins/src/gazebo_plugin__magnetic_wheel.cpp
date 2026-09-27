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

#include "gazebo_plugins/gazebo_plugin__magnetic_wheel.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MagneticWheelPlugin);

/////////////////////////////////////////////////
MagneticWheelPlugin::MagneticWheelPlugin() : ModelPlugin() {
}

/////////////////////////////////////////////////
MagneticWheelPlugin::~MagneticWheelPlugin() {
  #if GAZEBO_MAJOR_VERSION >= 8
  update_connection_ptr.reset();
  #else
  event::Events::DisconnectWorldUpdateBegin(update_connection_ptr);
  #endif
}

//////////////////////////////////////////////////
void MagneticWheelPlugin::Reset() {
  link->ResetPhysicsStates();
}

/////////////////////////////////////////////////
void MagneticWheelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)  {
    // Get model
    model = _model;
    // Get base link
    std::string link_name;
    getSdfParam<std::string>(_sdf, "link_name", link_name, "");
    link = model->GetChildLink(link_name);
    if (link == NULL)
      gzthrow("Couldn't find specified link \"" << link_name << "\".");
    printf("Magnetic wheel plugin loaded: %s\n", link_name.c_str());
    // Get right wheel link
    getSdfParam<std::string>(_sdf, "parent_link_name", link_name, "");
    parent_link = model->GetLink(link_name);
    if (parent_link == NULL)
      gzthrow("Couldn't find specified link \"" << link_name << "\".");
    //Other parameters
    getSdfParam<double>(_sdf, "magnetic_force", magnetic_force, 4000);
    // simulation iteration.
    #if GAZEBO_MAJOR_VERSION >= 8
    update_connection_ptr = event::Events::ConnectWorldUpdateBegin(std::bind(&MagneticWheelPlugin::OnUpdate, this));
    #else
    update_connection_ptr = event::Events::ConnectWorldUpdateBegin(boost::bind(&MagneticWheelPlugin::OnUpdate, this));
    #endif
    // Reset
    Reset();
}

/////////////////////////////////////////////////
void MagneticWheelPlugin::OnUpdate() {
// Force to keep the crawler on the surface
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d pose = parent_link->WorldPose();
    ignition::math::Vector3d force = pose.Rot().RotateVector(ignition::math::Vector3d(0, 0, -magnetic_force));
    link->AddForce(force);
#else
    gazebo::math::Pose pose = parent_link->GetWorldPose();
    gazebo::math::Vector3 force = pose.rot.RotateVector(gazebo::math::Vector3(0, 0, -magnetic_force));
    link->AddForce(force);
    // link->AddForce(gazebo::math::Vector3(0, 0, -magnetic_force));
#endif
}
