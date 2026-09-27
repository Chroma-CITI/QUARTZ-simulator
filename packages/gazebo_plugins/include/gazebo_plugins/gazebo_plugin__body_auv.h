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
 
#ifndef GAZEBO_PLUGINS_BODY_AUV_H_
#define GAZEBO_PLUGINS_BODY_AUV_H_

#include "gazebo_plugins/common.h"
#include "gazebo_plugins/gazebo_plugin__body.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float32.h>

namespace gazebo
{

/// \brief A plugin that simulate hydrodynamics for an AUV
class GAZEBO_VISIBLE BodyAUVPlugin : public BodyPlugin {

    /// Methods ////////////////////////////////////////////////////////
    public:
      BodyAUVPlugin();  /// \brief Constructor.
      ~BodyAUVPlugin(); /// \brief Destructor.

    protected:
      /// \brief Load model_
      virtual void Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf);
      /// \brief Callback for World Update events.
      virtual void OnUpdate();

    private:
      // \brief Callback topics
      void wrenchCb(const geometry_msgs::WrenchConstPtr & msg);

    /// Attributes /////////////////////////////////////////////////////
    protected:

      /// Parameters ///////////////////////////////////////////////////
      double Cx_;
      double Cw_;
      double buoyancy_;
      double buoyancy_offset_;
      /// ROS related //////////////////////////////////////////////////
      ros::Subscriber wrench_sub_;
      ros::Publisher usbl_pub_;
      ros::Publisher heading_pub_;
      common::Time last_usbl_pub_;
      geometry_msgs::Wrench last_wrench_msg_;
};

}
#endif
