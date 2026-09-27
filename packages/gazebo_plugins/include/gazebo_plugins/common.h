/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_GAZEBO_PLUGINS_COMMON_H_
#define ROTORS_GAZEBO_PLUGINS_COMMON_H_

#include <stdio.h>
#include <math.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/util/system.hh>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>

#include <ignition/math.hh>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <tinyxml.h>

#if GAZEBO_MAJOR_VERSION < 8
  #include <boost/bind.hpp>
#endif

#define EARTH_RADIUS      6353000.0     // meters
#define EARTH_RAD_A       6378100.0     // meters
#define EARTH_RAD_B       6371000.0     // meters

#define D2R               M_PI/180.0
#define R2D               180.0/M_PI

namespace gazebo {

/**
 * \brief Obtains a parameter from sdf.
 * \param[in] sdf Pointer to the sdf object.
 * \param[in] name Name of the parameter.
 * \param[out] param Param Variable to write the parameter to.
 * \param[in] default_value Default value, if the parameter not available.
 * \param[in] verbose If true, gzerror if the parameter is not available.
 */
template <class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value, const bool& verbose = false) {
  if (sdf->HasElement(name)) {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  }
  else {
    param = default_value;
    if (verbose)
      gzerr << "[quad_gazebo_plugins] Please specify a value for parameter \"" << name << "\".\n";
    return false;
  }
}

template <typename T>
void model_param(const std::string& world_name, const std::string& model_name, const std::string& param, T& param_value) {
  TiXmlElement* e_param = nullptr;
  TiXmlElement* e_param_tmp = nullptr;
  std::string dbg_param;

  TiXmlDocument doc(world_name + ".xml");
  if (doc.LoadFile()) {
    TiXmlHandle h_root(doc.RootElement());
    TiXmlElement* e_model = h_root.FirstChild("model").Element();
    for( e_model; e_model; e_model=e_model->NextSiblingElement("model") ) {
      const char* attr_name = e_model->Attribute("name");
      if (attr_name) {
        //specific
        if (model_name.compare(attr_name) == 0) {
          e_param_tmp = e_model->FirstChildElement(param);
          if (e_param_tmp) {
            e_param = e_param_tmp;
            dbg_param = "";
          }
          break;
        }
      }
      else {
        //common
        e_param = e_model->FirstChildElement(param);
        dbg_param = "common ";
      }
    }
    if (e_param) {
      std::istringstream iss(e_param->GetText());
      iss >> param_value;
      gzdbg << model_name << " model: " << dbg_param << "parameter " << param << " = " << param_value << " from " << doc.Value() << "\n";
    }
  }
}

/**
 * \brief Get a math::Angle as an angle from [0, 360)
 */
double GetDegrees360(const ignition::math::Angle& angle) {
  double degrees = angle.Degree();
  while (degrees < 0.) degrees += 360.0;
  while (degrees >= 360.0) degrees -= 360.0;
  return degrees;
}

double GetDegrees360(double degrees) {
  while (degrees < 0.) degrees += 360.0;
  while (degrees >= 360.0) degrees -= 360.0;
  return degrees;
}

std::pair<double, double> GetLinearFunctionCoefs(
  double x1,
  double y1,
  double x2,
  double y2){
    if (std::abs(x2) < 1.0e-10){
        return GetLinearFunctionCoefs(x2, y2, x1, y1);
    }
    else{
      if (std::abs(x2 - x1) > 1.0e-10){
        double b = (y1 - y2/x2)/(1.0 - x1/x2);
        double a = (y2 - b) / x2;
        return std::make_pair(a, b);
      }
      else{
        gzdbg << x1 << " , " << x2 << " , " << std::abs(x2 - x1) <<"\n";
        gzerr << "Probleme GetLinearFunctionCoefs() : x1 = x2 \n";
        return std::make_pair(0.0, 0.0);
      }
    }
}

/// \brief Project ECEF pose to Geodetic pose
/// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
ignition::math::Vector3<double> ECEFtoGeodetic(
  const ignition::math::Vector3<double> _in_pos_ecef,
  const double  _in_lat_home,
  const double  _in_lon_home,
  const double  _in_alt_home)
{
  // Variables intermediaires
  double X2 = _in_pos_ecef.X()*_in_pos_ecef.X();
  double Y2 = _in_pos_ecef.Y()*_in_pos_ecef.Y();
  double Z2 = _in_pos_ecef.Z()*_in_pos_ecef.Z();
  // Application of Ferrari's solution
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
  double r2  = X2 + Y2;
  if (r2 > 0.0){
    double r   = sqrt(r2);
    double a2  = EARTH_RAD_A*EARTH_RAD_A;
    double b2  = EARTH_RAD_B*EARTH_RAD_B;
    double e2  = 1.0 - b2/a2;
    double e4  = e2*e2;
    double ep2 = (a2 - b2)/b2;
    double E2  = (a2 - b2);
    double F   = 54.0*b2*Z2;
    double G   = r2 + (1.0 - e2)*Z2 - e2*E2;
    double C   = e4*F*r2 / (G*G*G);
    double S   = pow(1.0 + C + sqrt(C*C + 2.0*C), 1.0/3.0);
    double P   = F / (3.0*(S + 1.0/S + 1.0)*(S + 1.0/S + 1.0)*G*G);
    double Q   = sqrt(1.0 + 2.0*e4*P);
    double r0  = -(P*e2*r)/(1.0+Q) + sqrt(0.5*a2*(1.0+1.0/Q) - (P*(1.0-e2)*Z2)/(Q*(1.0+Q)) - 0.5*P*r2);
    double U   = sqrt((r - e2*r0)*(r - e2*r0) + Z2);
    double V   = sqrt((r - e2*r0)*(r - e2*r0) + (1.0 - e2)*Z2);
    double Z0  = b2*_in_pos_ecef.Z()/(EARTH_RAD_A*V);
    // Coordonnées geodesique
    double alt = U*(1.0 - b2/(EARTH_RAD_A*V));
    double lat = atan((_in_pos_ecef.Z() + ep2*Z0)/r)*R2D;
    double lon = atan2(_in_pos_ecef.Y(), _in_pos_ecef.X())*R2D;
    return ignition::math::Vector3<double>(lat, lon, alt);
  }
  else{
    return ignition::math::Vector3<double>(
      _in_lat_home, _in_lon_home, _in_alt_home);
  }
}

/// \brief Project Geodetic pose to ECEF pose
/// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
ignition::math::Vector3<double> GeodetictoECEF(
  const double  _in_lat,
  const double  _in_lon,
  const double  _in_alt)
{
  // Geodesic in radians
  double lat = _in_lat*D2R;
  double lon = _in_lon*D2R;
  // Variable intermediaire
  double a2 = EARTH_RAD_A*EARTH_RAD_A;
  double b2 = EARTH_RAD_B*EARTH_RAD_B;
  double e2 = 1.0 - b2/a2;
  double n = EARTH_RAD_A/sqrt(1.0 - e2*sin(lat)*sin(lat));
  // ECEF coordinates
  double x = (n + _in_alt)*cos(lat)*cos(lon);
  double y = (n + _in_alt)*cos(lat)*sin(lon);
  double z = ((b2/a2)*n + _in_alt)*sin(lat);
  return ignition::math::Vector3<double>(x, y, z);
}

/// \brief Project ENU pose to ECEF pose
/// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
ignition::math::Vector3<double> ENUtoECEF(
  const ignition::math::Vector3<double> _in_pos_enu,
  const double  _in_lat_home,
  const double  _in_lon_home,
  const double  _in_alt_home)
{
  // Position ECEF initiale
  ignition::math::Vector3<double> pos_ecef_0 = GeodetictoECEF(
    _in_lat_home, _in_lon_home, _in_alt_home);
  // Position ECEF courante
  if (_in_pos_enu.Length() > 0.0) {
    // Geodesic in radians
    double lat_home = _in_lat_home*D2R;
    double lon_home = _in_lon_home*D2R;
    // Matrice de passage ECEF -> ENU
    ignition::math::Matrix3<double> R(
      -sin(lon_home),                cos(lon_home),               0.0,
      -sin(lat_home)*cos(lon_home), -sin(lat_home)*sin(lon_home), cos(lat_home),
       cos(lat_home)*cos(lon_home),  cos(lat_home)*sin(lon_home), sin(lat_home));
    // Difference de position ECEF
    ignition::math::Vector3<double> diff_pos_ecef =
      R.Inverse()*_in_pos_enu;
    // Position ECEF courante
    return diff_pos_ecef + pos_ecef_0;
  }
  else{
    // Position ECEF courante = Position ECEF initiale
    return pos_ecef_0;
  }
}

/// \brief Project ENU pose to Geodetic pose
/// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
ignition::math::Vector3<double> ENUtoGeodetic(
  const ignition::math::Vector3<double> _in_pos_enu,
  const double  _in_lat_home,
  const double  _in_lon_home,
  const double  _in_alt_home)
{
  //Position ECEF
  ignition::math::Vector3<double> pos_ecef = ENUtoECEF(
    _in_pos_enu, _in_lat_home, _in_lon_home, _in_alt_home);
  //Position Geodesic
  return ECEFtoGeodetic(
    pos_ecef, _in_lat_home, _in_lon_home, _in_alt_home);
}


//Squared function
double sqr(double x) {
  return x * x;
}

}

/*
This class can be used to apply a first order filter on a signal.
It allows different acceleration and deceleration time constants.

Short reveiw of discrete time implementation of firest order system:
Laplace:
    X(s)/U(s) = 1/(tau*s + 1)
continous time system:
    dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
discretized system (ZoH):
    x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k)
*/
template <typename T>
class FirstOrderFilter {

  public:
    // Creation of First order filter
    FirstOrderFilter(double timeConstantUp, double timeConstantDown, T initialState):
      timeConstantUp_(timeConstantUp),
      timeConstantDown_(timeConstantDown),
      previousState_(initialState) {}

    // Destructor
    ~FirstOrderFilter() {}

    // This method will apply a first order filter on the inputState.
    T UpdateFilter(T inputState, double samplingTime) {
      T outputState;
      if(inputState > previousState_){
        // Calcuate the outputState if accelerating.
        double alphaUp = exp(- samplingTime / timeConstantUp_);
        // x(k+1) = Ad*x(k) + Bd*u(k)
        outputState = alphaUp * previousState_ + (1 - alphaUp) * inputState;
      }
      else{
        // Calculate the outputState if decelerating.
        double alphaDown = exp(- samplingTime / timeConstantDown_);
        // x(k+1) = Ad*x(k) + Bd*u(k)
        outputState = alphaDown * previousState_ + (1 - alphaDown) * inputState;
      }
      previousState_ = outputState;
      return outputState;
    }

  protected:
    double timeConstantUp_;
    double timeConstantDown_;
    T previousState_;
};

/// Conversion fonction from Gazebo Math types to Ignition Math types
#if GAZEBO_MAJOR_VERSION < 9
inline ignition::math::Vector3<double> GazeboToIgnition(const gazebo::math::Vector3 &vec_gz) {
  return ignition::math::Vector3<double>(vec_gz.x, vec_gz.y, vec_gz.z);
}
inline ignition::math::Pose3<double> GazeboToIgnition(const gazebo::math::Pose &pose_gz) {
  return ignition::math::Pose3<double>(
    pose_gz.pos.x, pose_gz.pos.y, pose_gz.pos.z,
    pose_gz.rot.w, pose_gz.rot.x, pose_gz.rot.y, pose_gz.rot.z);
}
inline ignition::math::Box GazeboToIgnition(const gazebo::math::Box &box) {
  return ignition::math::Box(GazeboToIgnition(box.min), GazeboToIgnition(box.max));
}
#endif

/**
 * @note Frames of reference:
 * g - gazebo (ENU), east, north, up
 * r - rotors imu frame (FLU), forward, left, up
 * b - px4 (FRD) forward, right down
 * n - px4 (NED) north, east, down
 */

/**
 * @brief Quaternion for rotation from ENU to NED frames
 *
 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
 */
static const auto Q_NED_TO_ENU = ignition::math::Quaternion<double>(M_PI, 0.0, M_PI/2.0);
static const auto Q_ENU_TO_NED = Q_NED_TO_ENU.Inverse();

/**
 * @brief Quaternion for rotation from body FLU to body FRD frames
 *
 * +PI rotation around X (Forward) axis rotates from Forward, Right, Down (aircraft)
 * to Forward, Left, Up (base_link) frames and vice-versa.
 */
static const auto Q_FLU_TO_FRD = ignition::math::Quaternion<double>(M_PI, 0.0, 0.0);
static const auto Q_FRD_TO_FLU = Q_FLU_TO_FRD.Inverse();

/**
 * @brief Quaternion for rotation from NWU to ENU frames
 *
 * -PI/2 rotation around Z (Forward) axis rotates from North, West, UP (Gazebo World)
 * to East, North, Up (Convention) frames and vice-versa.
 */
static const auto Q_NWU_TO_ENU = ignition::math::Quaternion<double>(0.0, 0.0, M_PI/2.0);
static const auto Q_ENU_TO_NWU = Q_NWU_TO_ENU.Inverse();

/**
 * @brief Header time stamp conversion function
 */

inline double HeaderToUsec(const std_msgs::Header& header){
  return ((double) header.stamp.sec) * 1e6 + ((double) header.stamp.nsec) * 1e-3;
}
inline double HeaderToMsec(const std_msgs::Header& header){
  return ((double) header.stamp.sec) * 1e3 + ((double) header.stamp.nsec) * 1e-6;
}


#endif /* ROTORS_GAZEBO_PLUGINS_COMMON_H_ */
