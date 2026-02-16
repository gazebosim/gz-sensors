/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include <map>
#include <unordered_map>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/CpuLidarSensor.hh"
#include "gz/sensors/GaussianNoiseModel.hh"
#include "gz/sensors/Noise.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/SensorTypes.hh"

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

  /// \brief Noise model for lidar data
  public: std::unordered_map<gz::sensors::SensorNoiseType,
      gz::sensors::NoisePtr> noises;

  /// \brief PointCloud publisher
  public: transport::Node::Publisher pointPub;

  /// \brief PointCloud message (reused)
  public: msgs::PointCloudPacked pointMsg;
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

  const std::map<SensorNoiseType, sdf::Noise> noises = {
    {LIDAR_NOISE, this->dataPtr->sdfLidar.LidarNoise()},
  };

  for (const auto & [noiseType, noiseSdf] : noises)
  {
    if (noiseSdf.Type() == sdf::NoiseType::GAUSSIAN)
    {
      if (!math::equal(noiseSdf.Mean(), 0.0) ||
          !math::equal(noiseSdf.StdDev(), 0.0) ||
          !math::equal(noiseSdf.BiasMean(), 0.0) ||
          !math::equal(noiseSdf.DynamicBiasStdDev(), 0.0) ||
          !math::equal(noiseSdf.DynamicBiasCorrelationTime(), 0.0))
      {
        this->dataPtr->noises[noiseType] =
          NoiseFactory::NewNoiseModel(noiseSdf);
      }
    }
    else if (noiseSdf.Type() != sdf::NoiseType::NONE)
    {
      gzwarn << "The cpu lidar sensor only supports Gaussian noise. "
       << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
       << "] is not supported." << std::endl;
    }
  }

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

  msgs::InitPointCloudPacked(this->dataPtr->pointMsg, this->FrameId(), false,
      {{"xyz", msgs::PointCloudPacked::Field::FLOAT32},
      {"intensity", msgs::PointCloudPacked::Field::FLOAT32},
      {"ring", msgs::PointCloudPacked::Field::UINT16}});

  this->dataPtr->pointMsg.set_width(this->RayCount());
  this->dataPtr->pointMsg.set_height(this->VerticalRayCount());
  this->dataPtr->pointMsg.set_row_step(
      this->dataPtr->pointMsg.point_step() *
      this->dataPtr->pointMsg.width());

  this->dataPtr->pointPub =
      this->dataPtr->node.Advertise<gz::msgs::PointCloudPacked>(
          this->Topic() + "/points");
  if (!this->dataPtr->pointPub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() << "/points].\n";
    return false;
  }

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

  if (!this->HasConnections())
    return false;

  if (this->dataPtr->scanPub.HasConnections())
  {
    auto &msg = this->dataPtr->laserMsg;

    *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
    msg.mutable_header()->clear_data();
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->FrameId());
    msg.set_frame(this->FrameId());

    msgs::Set(msg.mutable_world_pose(), this->Pose());

    const unsigned int hCount = this->RayCount();
    const unsigned int vCount = this->VerticalRayCount();

    // Determine the number of samples to publish in the LaserScan message.
    // For multi-ring lidars (vertical ray count > 1), we extract only the
    // middle ring to provide a 2D slice for ROS compatibility. ROS laser_scan
    // messages expect a single horizontal scan line. The middle ring is chosen
    // as it represents the horizontal plane of the sensor. The PointCloudPacked
    // message still contains all ranges (all rings) for full 3D data.
    const bool isMultiRing = (vCount > 1);
    const unsigned int publishCount = isMultiRing ? hCount : (hCount * vCount);
    const unsigned int midRingIndex = vCount / 2;

    const int numRays = static_cast<int>(this->dataPtr->ranges.size());
    if (msg.ranges_size() != static_cast<int>(publishCount))
    {
      msg.clear_ranges();
      msg.clear_intensities();
      msg.set_count(publishCount);
      msg.set_vertical_count(isMultiRing ? 1 : vCount);

      if (isMultiRing)
      {
        const double vMin = this->VerticalAngleMin().Radian();
        const double vMax = this->VerticalAngleMax().Radian();
        const double vStep = vCount > 1 ? (vMax - vMin) / (vCount - 1) : 0.0;
        const double midAngle = vMin + midRingIndex * vStep;
        msg.set_vertical_angle_min(midAngle);
        msg.set_vertical_angle_max(midAngle);
        msg.set_vertical_angle_step(0.0);
      }

      for (unsigned int i = 0; i < publishCount; ++i)
      {
        msg.add_ranges(gz::math::NAN_F);
        msg.add_intensities(0.0);
      }
    }

    if (isMultiRing)
    {
      // Extract only the middle ring for 2D laser scan compatibility
      const unsigned int ringStart = midRingIndex * hCount;
      for (unsigned int i = 0; i < hCount; ++i)
      {
        msg.set_ranges(i, this->dataPtr->ranges[ringStart + i]);
        msg.set_intensities(i, 0.0);
      }
    }
    else
    {
      // Single ring: use all ranges (existing behavior)
      for (int i = 0; i < numRays; ++i)
      {
        msg.set_ranges(i, this->dataPtr->ranges[i]);
        msg.set_intensities(i, 0.0);
      }
    }

    this->AddSequence(msg.mutable_header());
    this->dataPtr->scanPub.Publish(msg);
  }

  if (this->dataPtr->pointPub.HasConnections())
  {
    *this->dataPtr->pointMsg.mutable_header()->mutable_stamp() =
      msgs::Convert(_now);
    this->dataPtr->pointMsg.mutable_header()->clear_data();
    auto pcFrame = this->dataPtr->pointMsg.mutable_header()->add_data();
    pcFrame->set_key("frame_id");
    pcFrame->add_value(this->FrameId());

    const unsigned int hSamples = this->RayCount();
    const unsigned int vSamples = this->VerticalRayCount();
    const double hMin = this->AngleMin().Radian();
    const double hMax = this->AngleMax().Radian();
    const double vMin = this->VerticalAngleMin().Radian();
    const double vMax = this->VerticalAngleMax().Radian();
    const double hStep = hSamples > 1 ? (hMax - hMin) / (hSamples - 1) : 0.0;
    const double vStep = vSamples > 1 ? (vMax - vMin) / (vSamples - 1) : 0.0;

    std::string *msgBuffer = this->dataPtr->pointMsg.mutable_data();
    msgBuffer->resize(this->dataPtr->pointMsg.row_step() *
        this->dataPtr->pointMsg.height());
    char *msgBufferIndex = msgBuffer->data();
    bool isDense = true;

    for (unsigned int j = 0; j < vSamples; ++j)
    {
      const double inclination = vMin + j * vStep;
      for (unsigned int i = 0; i < hSamples; ++i)
      {
        const double azimuth = hMin + i * hStep;
        const int index = j * hSamples + i;
        const double range = this->dataPtr->ranges[index];

        if (isDense)
          isDense = !(std::isnan(range) || std::isinf(range));

        float depth = static_cast<float>(range);
        int fieldIndex = 0;

        *reinterpret_cast<float *>(msgBufferIndex +
            this->dataPtr->pointMsg.field(fieldIndex++).offset()) =
          depth * static_cast<float>(std::cos(inclination) * std::cos(azimuth));

        *reinterpret_cast<float *>(msgBufferIndex +
            this->dataPtr->pointMsg.field(fieldIndex++).offset()) =
          depth * static_cast<float>(std::cos(inclination) * std::sin(azimuth));

        *reinterpret_cast<float *>(msgBufferIndex +
            this->dataPtr->pointMsg.field(fieldIndex++).offset()) =
          depth * static_cast<float>(std::sin(inclination));

        *reinterpret_cast<float *>(msgBufferIndex +
            this->dataPtr->pointMsg.field(fieldIndex++).offset()) = 0.0f;

        *reinterpret_cast<uint16_t *>(msgBufferIndex +
            this->dataPtr->pointMsg.field(fieldIndex++).offset()) =
          static_cast<uint16_t>(j);

        msgBufferIndex += this->dataPtr->pointMsg.point_step();
      }
    }
    this->dataPtr->pointMsg.set_is_dense(isDense);

    this->AddSequence(this->dataPtr->pointMsg.mutable_header());
    this->dataPtr->pointPub.Publish(this->dataPtr->pointMsg);
  }

  return true;
}

//////////////////////////////////////////////////
bool CpuLidarSensor::HasConnections() const
{
  return (this->dataPtr->scanPub &&
         this->dataPtr->scanPub.HasConnections()) ||
         (this->dataPtr->pointPub &&
         this->dataPtr->pointPub.HasConnections());
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

  if (this->dataPtr->noises.find(LIDAR_NOISE) != this->dataPtr->noises.end())
  {
    for (size_t i = 0; i < this->dataPtr->ranges.size(); ++i)
    {
      if (!std::isinf(this->dataPtr->ranges[i]))
      {
        this->dataPtr->ranges[i] =
          this->dataPtr->noises[LIDAR_NOISE]->Apply(this->dataPtr->ranges[i]);
      }
    }
  }
}

//////////////////////////////////////////////////
void CpuLidarSensor::Ranges(std::vector<double> &_ranges) const
{
  _ranges = this->dataPtr->ranges;
}
