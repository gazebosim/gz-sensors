/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <cmath>
#include <limits>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/CpuLidarSensor.hh"
#include "gz/sensors/SensorFactory.hh"

using namespace gz;
using namespace sensors;

/// \brief Private data for CpuLidarSensor
class gz::sensors::CpuLidarSensorPrivate
{
  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief SDF lidar config
  public: sdf::Lidar sdfLidar;

  /// \brief Computed range values from the latest raycast results
  public: std::vector<double> ranges;

  /// \brief Transport node
  public: transport::Node node;

  /// \brief LaserScan publisher
  public: transport::Node::Publisher scanPub;

  /// \brief LaserScan message (reused)
  public: msgs::LaserScan laserMsg;
};

//////////////////////////////////////////////////
CpuLidarSensor::CpuLidarSensor()
  : dataPtr(new CpuLidarSensorPrivate())
{
}

//////////////////////////////////////////////////
CpuLidarSensor::~CpuLidarSensor()
{
}

//////////////////////////////////////////////////
bool CpuLidarSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  if (_sdf.Type() != sdf::SensorType::LIDAR)
  {
    gzerr << "Attempting to load a CpuLidar sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.LidarSensor() == nullptr)
  {
    gzerr << "Attempting to load a CpuLidar sensor, but received "
      << "a null lidar sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfLidar = *_sdf.LidarSensor();

  if (this->Topic().empty())
    this->SetTopic("/cpu_lidar");

  this->dataPtr->scanPub =
      this->dataPtr->node.Advertise<gz::msgs::LaserScan>(this->Topic());
  if (!this->dataPtr->scanPub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  auto &msg = this->dataPtr->laserMsg;
  msg.set_count(this->RayCount() * this->VerticalRayCount());
  msg.set_range_min(this->RangeMin());
  msg.set_range_max(this->RangeMax());
  msg.set_angle_min(this->AngleMin().Radian());
  msg.set_angle_max(this->AngleMax().Radian());
  msg.set_angle_step(
    this->RayCount() > 1
      ? (this->AngleMax().Radian() - this->AngleMin().Radian())
        / (this->RayCount() - 1)
      : 0.0);
  msg.set_vertical_angle_min(this->VerticalAngleMin().Radian());
  msg.set_vertical_angle_max(this->VerticalAngleMax().Radian());
  msg.set_vertical_angle_step(
    this->VerticalRayCount() > 1
      ? (this->VerticalAngleMax().Radian() - this->VerticalAngleMin().Radian())
        / (this->VerticalRayCount() - 1)
      : 0.0);
  msg.set_vertical_count(this->VerticalRayCount());

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool CpuLidarSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool CpuLidarSensor::Update(
    const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("CpuLidarSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->scanPub.HasConnections())
    return false;

  auto &msg = this->dataPtr->laserMsg;

  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  msg.mutable_header()->clear_data();
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->FrameId());
  msg.set_frame(this->FrameId());

  msgs::Set(msg.mutable_world_pose(), this->Pose());

  const int numRays = static_cast<int>(this->dataPtr->ranges.size());
  if (msg.ranges_size() != numRays)
  {
    msg.clear_ranges();
    msg.clear_intensities();
    for (int i = 0; i < numRays; ++i)
    {
      msg.add_ranges(gz::math::NAN_F);
      msg.add_intensities(0.0);
    }
  }

  for (int i = 0; i < numRays; ++i)
  {
    msg.set_ranges(i, this->dataPtr->ranges[i]);
    msg.set_intensities(i, 0.0);
  }

  this->AddSequence(msg.mutable_header());
  this->dataPtr->scanPub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
bool CpuLidarSensor::HasConnections() const
{
  return this->dataPtr->scanPub &&
         this->dataPtr->scanPub.HasConnections();
}

//////////////////////////////////////////////////
gz::math::Angle CpuLidarSensor::AngleMin() const
{
  return this->dataPtr->sdfLidar.HorizontalScanMinAngle();
}

//////////////////////////////////////////////////
gz::math::Angle CpuLidarSensor::AngleMax() const
{
  return this->dataPtr->sdfLidar.HorizontalScanMaxAngle();
}

//////////////////////////////////////////////////
gz::math::Angle CpuLidarSensor::VerticalAngleMin() const
{
  return this->dataPtr->sdfLidar.VerticalScanMinAngle();
}

//////////////////////////////////////////////////
gz::math::Angle CpuLidarSensor::VerticalAngleMax() const
{
  return this->dataPtr->sdfLidar.VerticalScanMaxAngle();
}

//////////////////////////////////////////////////
double CpuLidarSensor::RangeMin() const
{
  return this->dataPtr->sdfLidar.RangeMin();
}

//////////////////////////////////////////////////
double CpuLidarSensor::RangeMax() const
{
  return this->dataPtr->sdfLidar.RangeMax();
}

//////////////////////////////////////////////////
unsigned int CpuLidarSensor::RayCount() const
{
  return this->dataPtr->sdfLidar.HorizontalScanSamples();
}

//////////////////////////////////////////////////
unsigned int CpuLidarSensor::VerticalRayCount() const
{
  return this->dataPtr->sdfLidar.VerticalScanSamples();
}

//////////////////////////////////////////////////
std::vector<std::pair<gz::math::Vector3d, gz::math::Vector3d>>
CpuLidarSensor::GenerateRays() const
{
  const unsigned int hSamples = this->RayCount();
  const unsigned int vSamples = this->VerticalRayCount();
  const double hMin = this->AngleMin().Radian();
  const double hMax = this->AngleMax().Radian();
  const double vMin = this->VerticalAngleMin().Radian();
  const double vMax = this->VerticalAngleMax().Radian();
  const double rMin = this->RangeMin();
  const double rMax = this->RangeMax();

  const double hStep = hSamples > 1 ? (hMax - hMin) / (hSamples - 1) : 0.0;
  const double vStep = vSamples > 1 ? (vMax - vMin) / (vSamples - 1) : 0.0;

  std::vector<std::pair<gz::math::Vector3d, gz::math::Vector3d>> rays;
  rays.reserve(hSamples * vSamples);

  for (unsigned int v = 0; v < vSamples; ++v)
  {
    const double inclination = vMin + v * vStep;
    for (unsigned int h = 0; h < hSamples; ++h)
    {
      const double azimuth = hMin + h * hStep;
      const gz::math::Vector3d dir(
        std::cos(inclination) * std::cos(azimuth),
        std::cos(inclination) * std::sin(azimuth),
        std::sin(inclination));

      rays.emplace_back(dir * rMin, dir * rMax);
    }
  }

  return rays;
}

//////////////////////////////////////////////////
void CpuLidarSensor::SetRaycastResults(
    const std::vector<RayResult> &_results)
{
  const double rMin = this->RangeMin();
  const double rMax = this->RangeMax();
  const double rayLength = rMax - rMin;

  this->dataPtr->ranges.resize(_results.size());

  for (size_t i = 0; i < _results.size(); ++i)
  {
    if (std::isnan(_results[i].fraction))
    {
      this->dataPtr->ranges[i] =
        std::numeric_limits<double>::infinity();
    }
    else
    {
      double range = rMin + _results[i].fraction * rayLength;
      if (range < rMin)
        range = -std::numeric_limits<double>::infinity();
      else if (range > rMax)
        range = std::numeric_limits<double>::infinity();
      this->dataPtr->ranges[i] = range;
    }
  }
}

//////////////////////////////////////////////////
void CpuLidarSensor::Ranges(std::vector<double> &_ranges) const
{
  _ranges = this->dataPtr->ranges;
}
