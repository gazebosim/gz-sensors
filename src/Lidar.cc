/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <ignition/common/Console.hh>
#include <sdf/Lidar.hh>

#include "ignition/sensors/Lidar.hh"
#include "ignition/sensors/Noise.hh"
#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/SensorTypes.hh"
#include "ignition/sensors/GaussianNoiseModel.hh"

using namespace ignition::sensors;

/// \brief Private data for Lidar class
class ignition::sensors::LidarPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub;

  /// \brief Laser message to publish data.
  public: ignition::msgs::LaserScan laserMsg;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Sdf sensor.
  public: sdf::Lidar sdfLidar;
};

//////////////////////////////////////////////////
Lidar::Lidar()
  : dataPtr(new LidarPrivate())
{
}

//////////////////////////////////////////////////
Lidar::~Lidar()
{
  this->Fini();
}

//////////////////////////////////////////////////
bool Lidar::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
void Lidar::Fini()
{
  if (this->laserBuffer)
  {
    delete [] this->laserBuffer;
    this->laserBuffer = nullptr;
  }
}

//////////////////////////////////////////////////
bool Lidar::CreateLidar()
{
  return false;
}

//////////////////////////////////////////////////
bool Lidar::Load(const sdf::Sensor &_sdf)
{
  // Load sensor element
  if (!this->Sensor::Load(_sdf))
  {
    return false;
  }

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::LIDAR &&
      _sdf.Type() != sdf::SensorType::GPU_LIDAR)
  {
    ignerr << "Attempting to a load a Lidar sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.LidarSensor() == nullptr)
  {
    ignerr << "Attempting to a load a Lidar sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  // Register publisher
  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::LaserScan>(
        this->Topic());
  if (!this->dataPtr->pub)
  {
    ignerr << "Failed to advertise on topic [" << this->Topic()
      << "]." << std::endl;
    return false;
  }
  ignmsg << "Publishing laser scans on [" << this->Topic() << "]" << std::endl;

  // Load ray atributes
  this->dataPtr->sdfLidar = *_sdf.LidarSensor();

  if (this->RayCount() == 0 || this->VerticalRayCount() == 0)
  {
    ignerr << "Lidar: Image has 0 size!\n";
  }

  // create message
  this->dataPtr->laserMsg.set_count(this->RangeCount());
  this->dataPtr->laserMsg.set_range_min(this->RangeMin());
  this->dataPtr->laserMsg.set_range_max(this->RangeMax());
  this->dataPtr->laserMsg.set_angle_min(this->AngleMin().Radian());
  this->dataPtr->laserMsg.set_angle_max(this->AngleMax().Radian());
  this->dataPtr->laserMsg.set_angle_step(this->AngleResolution());
  this->dataPtr->laserMsg.set_vertical_angle_min(
      this->VerticalAngleMin().Radian());
  this->dataPtr->laserMsg.set_vertical_angle_max(
      this->VerticalAngleMax().Radian());
  this->dataPtr->laserMsg.set_vertical_angle_step(
      this->VerticalAngleResolution());
  this->dataPtr->laserMsg.set_vertical_count(
      this->VerticalRangeCount());

  // Handle noise model settings.
  const std::map<SensorNoiseType, sdf::Noise> noises = {
    {LIDAR_NOISE, this->dataPtr->sdfLidar.LidarNoise()},
  };

  for (const auto & [noiseType, noiseSdf] : noises)
  {
    if (noiseSdf.Type() == sdf::NoiseType::GAUSSIAN)
    {
      this->dataPtr->noises[noiseType] =
        NoiseFactory::NewNoiseModel(noiseSdf);
    }
    else if (noiseSdf.Type() != sdf::NoiseType::NONE)
    {
      ignwarn << "The lidar sensor only supports Gaussian noise. "
       << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
       << "] is not supported." << std::endl;
    }
  }

  this->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool Lidar::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

/////////////////////////////////////////////////
ignition::common::ConnectionPtr Lidar::ConnectNewLidarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _heighti, unsigned int _channels,
                  const std::string &/*_format*/)> /*_subscriber*/)
{
  return nullptr;
}

//////////////////////////////////////////////////
bool Lidar::Update(const ignition::common::Time &/*_now*/)
{
  ignerr << "No lidar data being updated.\n";
  return false;
}

//////////////////////////////////////////////////
bool Lidar::PublishLidarScan(const ignition::common::Time &_now)
{
  if (!this->laserBuffer)
    return false;

  std::lock_guard<std::mutex> lock(this->lidarMutex);

  this->dataPtr->laserMsg.mutable_header()->mutable_stamp()->set_sec(
      _now.sec);
  this->dataPtr->laserMsg.mutable_header()->mutable_stamp()->set_nsec(
      _now.nsec);
  auto frame = this->dataPtr->laserMsg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());
  this->dataPtr->laserMsg.set_frame(this->Name());

  // Store the latest laser scans into laserMsg
  msgs::Set(this->dataPtr->laserMsg.mutable_world_pose(),
      this->Pose());

  const int numRays = this->RayCount() * this->VerticalRayCount();
  if (this->dataPtr->laserMsg.ranges_size() != numRays)
  {
    // igndbg << "Size mismatch; allocating memory\n";
    this->dataPtr->laserMsg.clear_ranges();
    this->dataPtr->laserMsg.clear_intensities();
    for (int i = 0; i < numRays; ++i)
    {
      this->dataPtr->laserMsg.add_ranges(ignition::math::NAN_F);
      this->dataPtr->laserMsg.add_intensities(ignition::math::NAN_F);
    }
  }

  for (unsigned int j = 0; j < this->VerticalRangeCount(); ++j)
  {
    for (unsigned int i = 0; i < this->RangeCount(); ++i)
    {
      int index = j * this->RangeCount() + i;
      double range = this->laserBuffer[index*3];

      if (this->dataPtr->noises.find(LIDAR_NOISE) !=
          this->dataPtr->noises.end())
      {
        range = this->dataPtr->noises[LIDAR_NOISE]->Apply(range);
        range = ignition::math::clamp(range,
            this->RangeMin(), this->RangeMax());
      }

      range = ignition::math::isnan(range) ? this->RangeMax() : range;
      this->dataPtr->laserMsg.set_ranges(index, range);
      this->dataPtr->laserMsg.set_intensities(index,
          this->laserBuffer[index * 3 + 1]);
    }
  }

  // publish
  this->dataPtr->pub.Publish(this->dataPtr->laserMsg);

  return true;
}

//////////////////////////////////////////////////
bool Lidar::IsHorizontal() const
{
//  return this->dataPtr->laserCam->IsHorizontal();
  return 0;
}

//////////////////////////////////////////////////
double Lidar::RangeCountRatio() const
{
  return static_cast<double>(this->RangeCount()) / this->VerticalRangeCount();
}

//////////////////////////////////////////////////
ignition::math::Angle Lidar::AngleMin() const
{
  return this->dataPtr->sdfLidar.HorizontalScanMinAngle();
}

//////////////////////////////////////////////////
void Lidar::SetAngleMin(double _angle)
{
  this->dataPtr->sdfLidar.SetHorizontalScanMinAngle(_angle);
}

//////////////////////////////////////////////////
ignition::math::Angle Lidar::AngleMax() const
{
  return this->dataPtr->sdfLidar.HorizontalScanMaxAngle();
}

//////////////////////////////////////////////////
void Lidar::SetAngleMax(double _angle)
{
  this->dataPtr->sdfLidar.SetHorizontalScanMaxAngle(_angle);
}

//////////////////////////////////////////////////
double Lidar::RangeMin() const
{
  return this->dataPtr->sdfLidar.RangeMin();
}

//////////////////////////////////////////////////
double Lidar::RangeMax() const
{
  return this->dataPtr->sdfLidar.RangeMax();
}

/////////////////////////////////////////////////
double Lidar::AngleResolution() const
{
  return (this->AngleMax() - this->AngleMin()).Radian() /
    (this->RangeCount()-1);
}

//////////////////////////////////////////////////
double Lidar::RangeResolution() const
{
  return this->dataPtr->sdfLidar.RangeResolution();
}

//////////////////////////////////////////////////
unsigned int Lidar::RayCount() const
{
  return this->dataPtr->sdfLidar.HorizontalScanSamples();
}

//////////////////////////////////////////////////
unsigned int Lidar::RangeCount() const
{
  return this->RayCount() * this->dataPtr->sdfLidar.HorizontalScanResolution();
}

//////////////////////////////////////////////////
unsigned int Lidar::VerticalRayCount() const
{
  return this->dataPtr->sdfLidar.VerticalScanSamples();
}

//////////////////////////////////////////////////
unsigned int Lidar::VerticalRangeCount() const
{
  int rows = this->VerticalRayCount() *
    this->dataPtr->sdfLidar.VerticalScanResolution();
  if (rows > 1)
    return rows;
  else
    return 1;
}

//////////////////////////////////////////////////
void Lidar::SetParent(const std::string &_parent)
{
  Sensor::SetParent(_parent);
}

//////////////////////////////////////////////////
ignition::math::Angle Lidar::VerticalAngleMin() const
{
  return this->dataPtr->sdfLidar.VerticalScanMinAngle();
}

//////////////////////////////////////////////////
void Lidar::SetVerticalAngleMin(const double _angle)
{
  this->dataPtr->sdfLidar.SetVerticalScanMinAngle(_angle);
}

//////////////////////////////////////////////////
ignition::math::Angle Lidar::VerticalAngleMax() const
{
  return this->dataPtr->sdfLidar.VerticalScanMaxAngle();
}

//////////////////////////////////////////////////
double Lidar::VerticalAngleResolution() const
{
  return (this->VerticalAngleMax() - this->VerticalAngleMin()).Radian() /
    (this->VerticalRangeCount()-1);
}

//////////////////////////////////////////////////
void Lidar::SetVerticalAngleMax(const double _angle)
{
  this->dataPtr->sdfLidar.SetVerticalScanMaxAngle(_angle);
}

//////////////////////////////////////////////////
void Lidar::Ranges(std::vector<double> &_ranges) const
{
  std::lock_guard<std::mutex> lock(this->lidarMutex);

  _ranges.resize(this->dataPtr->laserMsg.ranges_size());
  memcpy(&_ranges[0], this->dataPtr->laserMsg.ranges().data(),
         sizeof(_ranges[0]) * this->dataPtr->laserMsg.ranges_size());
}

//////////////////////////////////////////////////
double Lidar::Range(const int _index) const
{
  std::lock_guard<std::mutex> lock(this->lidarMutex);

  if (this->dataPtr->laserMsg.ranges_size() == 0)
  {
    ignwarn << "ranges not constructed yet (zero sized)\n";
    return 0.0;
  }
  if (_index < 0 || _index > this->dataPtr->laserMsg.ranges_size())
  {
    ignerr << "Invalid range index[" << _index << "]\n";
    return 0.0;
  }

  return this->dataPtr->laserMsg.ranges(_index);
}

//////////////////////////////////////////////////
double Lidar::Retro(const int /*_index*/) const
{
  return 0.0;
}

//////////////////////////////////////////////////
int Lidar::Fiducial(const unsigned int /*_index*/) const
{
  return -1;
}


//////////////////////////////////////////////////
bool Lidar::IsActive() const
{
//  return Sensor::IsActive() ||
//    (this->dataPtr->pub && this->dataPtr->pub->HasConnections());
  return true;
}

IGN_SENSORS_REGISTER_SENSOR(Lidar)
