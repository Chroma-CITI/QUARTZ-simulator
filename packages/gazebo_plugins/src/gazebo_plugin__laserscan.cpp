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

#include <algorithm>
#include <assert.h>

#include "gazebo_plugins/gazebo_plugin__laserscan.h"
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/Node.hh>

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

#include <tf/tf.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(LaserscanPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
LaserscanPlugin::LaserscanPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
LaserscanPlugin::~LaserscanPlugin()
{
  ////////////////////////////////////////////////////////////////////////////////
  // Finalize the controller / Custom Callback Queue
  this->laser_queue_.clear();
  this->laser_queue_.disable();
  this->rosnode_->shutdown();
  this->callback_laser_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void LaserscanPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, _sdf);

  // Get then name of the parent sensor
  this->parent_sensor_ = _parent;

  // Get the world name.
  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);

#if GAZEBO_MAJOR_VERSION >= 9
  last_update_time_ = this->world_->SimTime();
#else
  last_update_time_ = this->world_->GetSimTime();
#endif

  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init(worldName);

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(this->parent_sensor_);

  if (!this->parent_ray_sensor_)
    gzthrow("LaserscanPlugin controller requires a Ray Sensor as its parent");

  // get robot namespace param
  getSdfParam<std::string>(_sdf, "robot_namespace", robot_namespace_, "");
  // Get frame name
  getSdfParam<std::string>(_sdf, "frame_name", frame_name_, "/laserscan");
  // Get topic name
  getSdfParam<std::string>(_sdf, "topic_name", topic_name_, "/laserscan");
  // Get gaussian noise
  getSdfParam<double>(_sdf, "gaussian_noise", gaussian_noise_, 0.0);
  getSdfParam<double>(_sdf, "range_noise_slope", range_noise_slope_, 0.0);
  ROS_INFO_NAMED("laserscan", "Received parameter gaussian_noise = %f", gaussian_noise_);
  ROS_INFO_NAMED("laserscan", "Received parameter range_noise_slope = %f", range_noise_slope_);
    parent_ray_sensor_->Noise(sensors::SensorNoiseType::NO_NOISE);
  // Get update rate
  getSdfParam<double>(_sdf, "update_rate", update_rate_, 0.0);
  // Get resolution
  getSdfParam<double>(_sdf, "resolution", resolution_, 0.001);
  // Interpolation
  getSdfParam<bool>(_sdf, "interpolation", interpolation_, true);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("laserscan", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    // Custom Callback Queue
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
      this->topic_name_,1,
      boost::bind( &LaserscanPlugin::LaserConnect,this),
      boost::bind( &LaserscanPlugin::LaserDisconnect,this), ros::VoidPtr(), &this->laser_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
  }

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
  // start custom queue for laser
  this->callback_laser_queue_thread_ = boost::thread( boost::bind( &LaserscanPlugin::LaserQueueThread,this ) );


}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void LaserscanPlugin::LaserConnect()
{
  this->laser_connect_count_++;
  this->parent_ray_sensor_->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void LaserscanPlugin::LaserDisconnect()
{
  this->laser_connect_count_--;

  if (this->laser_connect_count_ == 0)
    this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void LaserscanPlugin::OnNewLaserScans()
{
  if (this->topic_name_ != "")
  {
    // Current time
    common::Time sensor_update_time = parent_sensor_->LastUpdateTime();
    if (sensor_update_time < last_update_time_)
    {
        ROS_WARN_NAMED("laserscan", "Negative sensor update time difference detected.");
        last_update_time_ = sensor_update_time;
    }
    // Update if it's time to
    if ( (sensor_update_time - last_update_time_) >= (1.0/update_rate_) )
    {
      // Get laser data
      PutLaserData(sensor_update_time);
      last_update_time_ = sensor_update_time;
    }
  }
  else
  {
    ROS_INFO_NAMED("laserscan", "gazebo_ros_laserscan topic name not set");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void LaserscanPlugin::PutLaserData(common::Time &_updateTime)
{
  int i, hja, hjb;
  int j, vja, vjb;
  double vb, hb;
  int    k, k1, k2, k3, k4; // corners indices
  double r1, r2, r3, r4, r; // four corner values + interpolated range
  double intensity;

  parent_ray_sensor_->SetActive(false);

  double resRange = parent_ray_sensor_->RangeResolution();
  double maxRange = parent_ray_sensor_->RangeMax() - resRange;
  double minRange = parent_ray_sensor_->RangeMin() + resRange;

  int horizontalRayCount   = parent_ray_sensor_->RayCount();
  int horizontalRangeCount = parent_ray_sensor_->RangeCount();
  auto horizontalMaxAngle  = parent_ray_sensor_->AngleMax();
  auto horizontalMinAngle  = parent_ray_sensor_->AngleMin();

  int verticalRayCount   = parent_ray_sensor_->VerticalRayCount();
  int verticalRangeCount = parent_ray_sensor_->VerticalRangeCount();
  auto verticalMaxAngle  = parent_ray_sensor_->VerticalAngleMax();
  auto verticalMinAngle  = parent_ray_sensor_->VerticalAngleMin();

  double yAngle, yAngleStep;
  if (horizontalRayCount > 1)
    yAngleStep = ( (horizontalMaxAngle.Radian() - horizontalMinAngle.Radian()) / ((double) (horizontalRayCount - 1)));
  else
    yAngleStep = 0.0;

  double pAngle, pAngleStep;
  if (verticalRayCount > 1)
    pAngleStep = ( (verticalMaxAngle.Radian() - verticalMinAngle.Radian())     / ((double) (verticalRayCount - 1)));
  else
    pAngleStep = 0.0;

  // set size of cloud message everytime!
  sensor_msgs::PointCloud pointcloud_msg; // SF stands for sensor frame
  pointcloud_msg.channels.push_back(sensor_msgs::ChannelFloat32());

  // Lock
  boost::mutex::scoped_lock sclock(this->lock);

  // Add Frame Name
  pointcloud_msg.header.frame_id = robot_namespace_ + frame_name_;
  pointcloud_msg.header.stamp.sec = _updateTime.sec;
  pointcloud_msg.header.stamp.nsec = _updateTime.nsec;

  for (j = 0; j<verticalRangeCount; j++)
  {
    if (interpolation_) {
      // interpolating in vertical direction
      vb = (verticalRangeCount == 1) ? 0 : (double) j * (verticalRayCount - 1) / (verticalRangeCount - 1);
      vja = (int) floor(vb);
      vjb = std::min(vja + 1, verticalRayCount - 1);
      vb = vb - floor(vb); // fraction from min

      assert(vja >= 0 && vja < verticalRayCount);
      assert(vjb >= 0 && vjb < verticalRayCount);
    }

    for (i = 0; i<horizontalRangeCount; i++)
    {
      if (interpolation_) {
        // Interpolate the range readings from the rays in horizontal direction
        hb = (horizontalRangeCount == 1)? 0 : (double) i * (horizontalRayCount - 1) / (horizontalRangeCount - 1);
        hja = (int) floor(hb);
        hjb = std::min(hja + 1, horizontalRayCount - 1);
        hb = hb - floor(hb); // fraction from min

        assert(hja >= 0 && hja < horizontalRayCount);
        assert(hjb >= 0 && hjb < horizontalRayCount);

        // indices of 4 corners
        k1 = hja + vja * horizontalRayCount;
        k2 = hjb + vja * horizontalRayCount;
        k3 = hja + vjb * horizontalRayCount;
        k4 = hjb + vjb * horizontalRayCount;

        // range readings of 4 corners
        r1 = this->parent_ray_sensor_->LaserShape()->GetRange(k1);
        r2 = this->parent_ray_sensor_->LaserShape()->GetRange(k2);
        r3 = this->parent_ray_sensor_->LaserShape()->GetRange(k3);
        r4 = this->parent_ray_sensor_->LaserShape()->GetRange(k4);

        // Range is linear interpolation if values are close,
        // and min if they are very different
        r = (1-vb)*((1 - hb) * r1 + hb * r2)
           +   vb *((1 - hb) * r3 + hb * r4);

        // get angles of ray to get xyz for point
        yAngle = ( 0.5*((double) hja+hjb) * yAngleStep ) + horizontalMinAngle.Radian();
        pAngle = ( 0.5*((double) vja+vjb) * pAngleStep ) + verticalMinAngle.Radian();

        // Intensity is averaged
        intensity = 0.25*(this->parent_ray_sensor_->LaserShape()->GetRetro(k1) +
                          this->parent_ray_sensor_->LaserShape()->GetRetro(k2) +
                          this->parent_ray_sensor_->LaserShape()->GetRetro(k3) +
                          this->parent_ray_sensor_->LaserShape()->GetRetro(k4));
      }
      else {
        // indice
        k = i + j*horizontalRayCount;

        // Range = direct measurement
        r = this->parent_ray_sensor_->LaserShape()->GetRange(k);

        // get angles of ray to get xyz for point
        yAngle = ( ((double) i) * yAngleStep ) + horizontalMinAngle.Radian();
        pAngle = ( ((double) j) * pAngleStep ) + verticalMinAngle.Radian();

        // Direct intensity
        intensity = this->parent_ray_sensor_->LaserShape()->GetRetro(k);
      }

      if ((r > minRange) && (r < maxRange )){
        if (gaussian_noise_ < 0.0) {
          // no noise if at max range
          geometry_msgs::Point32 point;
          //pAngle is rotated by yAngle:
          point.x = r * cos(pAngle) * cos(yAngle);
          point.y = r * cos(pAngle) * sin(yAngle);
          point.z = r * sin(pAngle);
          pointcloud_msg.points.push_back(point);
          pointcloud_msg.channels[0].values.push_back(intensity) ;
        }
        else {
          geometry_msgs::Point32 point;
          //pAngle is rotated by yAngle: // dont uncomment

          // old noise method
          // point.x = r * cos(pAngle) * cos(yAngle) + GaussianKernel(0, gaussian_noise_);
          // point.y = r * cos(pAngle) * sin(yAngle) + GaussianKernel(0, gaussian_noise_);
          // point.z = r * sin(pAngle)               + GaussianKernel(0, gaussian_noise_);
          // pointcloud_msg.points.push_back(point);
          // pointcloud_msg.channels[0].values.push_back(intensity + GaussianKernel(0, gaussian_noise_)) ;

          const double cP = cos(pAngle);
          const double sP = sin(pAngle);
          const double cY = cos(yAngle);
          const double sY = sin(yAngle);

          // unit direction of the ray in sensor frame
          const double dx = cP * cY;
          const double dy = cP * sY;
          const double dz = sP;

          // range-dependent sigma: sigma(r)=sqrt(sigma0^2 + (slope*r)^2)
          const double sigma0 = gaussian_noise_;           // your existing parameter (0.008)
          const double slope  = range_noise_slope_;        // NEW parameter (e.g. 0.00127)

          double sigma_r = sigma0;
          if (slope > 0.0)
            sigma_r = sqrt(sigma0*sigma0 + (slope*r)*(slope*r));

          double r_noisy = r + GaussianKernel(0.0, sigma_r);

          // optional clamp to valid interval
          r_noisy = std::max(minRange, std::min(maxRange, r_noisy));

          point.x = r_noisy * dx;
          point.y = r_noisy * dy;
          point.z = r_noisy * dz;

          pointcloud_msg.points.push_back(point);
          pointcloud_msg.channels[0].values.push_back(intensity);
        } // only 1 channel
      }
    }
  }
  this->parent_ray_sensor_->SetActive(true);

  // Conversion in PointCloud2 msg
  sensor_msgs::PointCloud2 pointcloud2_msg;
  sensor_msgs::convertPointCloudToPointCloud2(
    pointcloud_msg,
    pointcloud2_msg);
  // publication
  pub_.publish(pointcloud2_msg);
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double LaserscanPlugin::GaussianKernel(double mu,double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

////////////////////////////////////////////////////////////////////////////////
// Custom Callback Queue
void LaserscanPlugin::LaserQueueThread()
{
  static const double timeout = 0.01;

  while (rosnode_->ok())
  {
    laser_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
void LaserscanPlugin::OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
{
  this->sim_time_  = msgs::Convert( _msg->sim_time() );

  ignition::math::Pose3d pose;
  pose.Pos().X() = 0.5*sin(0.01*this->sim_time_.Double());
  gzdbg << "plugin simTime [" << this->sim_time_.Double() << "] update pose [" << pose.Pos().X() << "]\n";
}


}
