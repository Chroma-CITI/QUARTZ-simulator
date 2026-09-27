/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
 
#ifndef GAZEBO_PLUGINS_BODY_UAV_H_
#define GAZEBO_PLUGINS_BODY_UAV_H_

#include "gazebo_plugins/common.h"
#include "gazebo_plugins/gazebo_plugin__body.h"

namespace gazebo
{

static const bool KDEFAULT_RADIAL_SYMMETRY = false;

static const double KDEFAULT_AREA = 1.0;
static const double KDEFAULT_RHO = 1.2041;
static const double KDEFAULT_ALPHA_ZERO = 0.0;
static const double KDEFAULT_LIFT_COEF_ZERO = 1.0;
static const double KDEFAULT_DRAG_COEF_ZERO = 0.01;
static const double KDEFAULT_MOMENT_COEF_ZERO = 0.01;
static const double KDEFAULT_ALPHA_STALL = 0.5*M_PI;
static const double KDEFAULT_LIFT_COEF_STALL = 0.0;
static const double KDEFAULT_DRAG_COEF_STALL = 1.0;
static const double KDEFAULT_MOMENT_COEF_STALL = 0.0;
static const double KDEFAULT_LIFT_COEF_FLAPERON = 4.0;

static const ignition::math::Vector3<double> KDEFAULT_CENTER_OF_PRESSURE = ignition::math::Vector3<double>(0.0, 0.0, 0.0);
static const ignition::math::Vector3<double> KDEFAULT_CHORD_DIRECTION = ignition::math::Vector3<double>(0.0, 1.0, 0.0);

/// \brief A plugin that simulate aerodynamics and give Ground truth
class GAZEBO_VISIBLE BodyUAVPlugin : public BodyPlugin {

  /// Methods ////////////////////////////////////////////////////////
  public:
    BodyUAVPlugin();  /// \brief Constructor.
    ~BodyUAVPlugin(); /// \brief Destructor.

  protected:
    /// \brief Load model
    virtual void Load(physics::ModelPtr _in_model, sdf::ElementPtr _in_sdf);
    /// \brief Callback for World Update events.
    virtual void OnUpdate();
    /// \brief Set Aerodynamics
    virtual void SetAero();

  /// Attributes /////////////////////////////////////////////////////
  protected:
    /// Gazebo related ///////////////////////////////////////////////
    /// \brief Pointer to a joint that actuates a control surface for
    /// this lifting body
    physics::JointPtr flaperon_joint_;

    /// Parameters ///////////////////////////////////////////////////
    /// \brief if the shape is aerodynamically radially symmetric about
    /// the vector_forward direction. Defaults to false for wing shapes.
    /// If set to true, the vector_upward direction is determined by the
    /// angle of attack.
    bool radial_symmetry_;
    /// \brief air density
    /// at 25 deg C it's about 1.1839 kg/m^3
    /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
    double rho_;
    /// \brief effective planeform surface area
    double area_;
    /// \brief initial angle of attack
    double alpha_zero_;
    /// \brief angle of attach when airfoil stalls
    double alpha_stall_;
    /// \brief Lift Coefficient (C_L) for alpha = alpha_zero
    /// Lift = C_L * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    double lift_coef_zero_;
    /// \brief Lift Coefficient for alpha = alpha_stall
    double lift_coef_stall_;
    /// \brief Functions lift Coefficient = f(alpha)
    std::pair<double, double> funct_lift_coef_;
    std::pair<double, double> funct_lift_coef_stall_pos_;
    std::pair<double, double> funct_lift_coef_stall_neg_;
    /// \brief how much to change CL per radian of control surface joint
    /// value.
    double lift_coef_flaperon_;
    /// \brief drag Coefficient (C_D) for alpha = alpha_zero
    /// Drag = C_D * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    double drag_coef_zero_;
    /// \brief drag Coefficient (C_D) for alpha = alpha_stall
    double drag_coef_stall_;
    /// \brief Functions drag Coefficient = f(alpha)
    std::pair<double, double> funct_drag_coef_pos_;
    std::pair<double, double> funct_drag_coef_neg_;
    std::pair<double, double> funct_drag_coef_stall_pos_;
    std::pair<double, double> funct_drag_coef_stall_neg_;
    /// \brief Moment Coefficient (C_M) for alpha = alpha_zero
    /// Moment = C_M * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    double moment_coef_zero_;
    /// \brief Moment Coefficient (C_M) for alpha = alpha_stall
    double moment_coef_stall_;
    /// \brief Functions Moment Coefficient = f(alpha)
    std::pair<double, double> funct_moment_coef_;
    std::pair<double, double> funct_moment_coef_stall_pos_;
    std::pair<double, double> funct_moment_coef_stall_neg_;
    /// \brief: \TODO: make a stall velocity curve
    double velocity_stall_;
    /// \brief center of pressure in link local coordinates
    ignition::math::Vector3<double> center_of_pressure_;
    /// \brief The chord direction of the wing expressed in the link frame.
    ignition::math::Vector3<double> chord_direction_;

};

}
#endif
