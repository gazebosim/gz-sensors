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
    const std::chrono::steady_clock::duration &/*_now*/)
{
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  return false;
}

//////////////////////////////////////////////////
bool CpuLidarSensor::HasConnections() const
{
  return false;
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
