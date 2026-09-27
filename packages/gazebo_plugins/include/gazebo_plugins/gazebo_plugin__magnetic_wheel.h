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

#ifndef GAZEBO_PLUGINS_MAGNETIC_WHEEL_H_
#define GAZEBO_PLUGINS_MAGNETIC_WHEEL_H_

#include "gazebo_plugins/common.h"

namespace gazebo
{

/// \brief A plugin that simulate a crawler movements
class GAZEBO_VISIBLE MagneticWheelPlugin : public ModelPlugin {

  /// Methods ////////////////////////////////////////////////////////
  public:
    MagneticWheelPlugin();  /// \brief Constructor.
    ~MagneticWheelPlugin(); /// \brief Destructor.

  protected:
    /// \brief Load model
    virtual void Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf);
    /// \brief Reset Function
    virtual void Reset();
    /// \brief Callback for World Update events.
    virtual void OnUpdate();

  /// Attributes /////////////////////////////////////////////////////
  protected:
    /// Gazebo related ///////////////////////////////////////////////
    physics::ModelPtr model;
    physics::LinkPtr link;
    physics::LinkPtr parent_link;
    /// Paramters ////////////////////////////////////////////////////
    double magnetic_force;

  private:
    /// \brief Connection to World Update events.
    event::ConnectionPtr update_connection_ptr;
};

}
#endif
