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
#include <ignition/msgs/pointcloud_packed.pb.h>
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
  /// \brief Fill the point cloud packed message
  public: void FillPointCloudMsg(float *_laserBuffer);

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

  /// \brief The point cloud message.
  public: msgs::PointCloudPacked pointMsg;

  /// \brief Publisher for the publish point cloud message.
  public: transport::Node::Publisher pointPub;
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
  // Set the values on the point message.
  this->dataPtr->pointMsg.set_width(this->RangeCount());
  this->dataPtr->pointMsg.set_height(this->VerticalRangeCount());
  this->dataPtr->pointMsg.set_row_step(
      this->dataPtr->pointMsg.point_step() *
      this->dataPtr->pointMsg.width() * this->dataPtr->pointMsg.height());

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

  // Create the point cloud publisher
  this->dataPtr->pointPub =
      this->dataPtr->node.Advertise<ignition::msgs::PointCloudPacked>(
          this->Topic() + "/points");

  if (!this->dataPtr->pointPub)
    return false;

  // Initialize the point message.
  // Initialize the point message.
  // \todo(anyone) The true value in the following function call forces
  // the xyz and rgb fields to be aligned to memory boundaries. This is need
  // by ROS1: https://github.com/ros/common_msgs/pull/77. Ideally, memory
  // alignment should be configured. This same problem is in the
  // RgbdCameraSensor.
  msgs::InitPointCloudPacked(this->dataPtr->pointMsg, this->Name(), true,
      {{"xyz", msgs::PointCloudPacked::Field::FLOAT32}});

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

  if (this->dataPtr->pub.HasConnections())
  {
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
  }

  // Publish the point cloud
  if (this->dataPtr->pointPub.HasConnections())
  {
    // Set the time stamp
    this->dataPtr->pointMsg.mutable_header()->mutable_stamp()->set_sec(
        _now.sec);
    this->dataPtr->pointMsg.mutable_header()->mutable_stamp()->set_nsec(
        _now.nsec);

    this->dataPtr->pointMsg.set_is_dense(true);
    this->dataPtr->FillPointCloudMsg(this->laserBuffer);
    this->dataPtr->pointPub.Publish(this->dataPtr->pointMsg);
  }

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

//////////////////////////////////////////////////
void LidarPrivate::FillPointCloudMsg(float *_laserBuffer)
{
  uint32_t width = this->pointMsg.width();
  uint32_t height = this->pointMsg.height();
  unsigned int channels = 3;

  unsigned int rangeCount = this->sdfLidar.HorizontalScanSamples() *
    this->sdfLidar.HorizontalScanResolution();
  unsigned int vertRangeCount = std::max(
      this->sdfLidar.VerticalScanSamples() *
      this->sdfLidar.VerticalScanResolution(), 1.0);

  float angleStep =
    (this->sdfLidar.HorizontalScanMaxAngle() -
     this->sdfLidar.HorizontalScanMinAngle()).Radian() /
    (rangeCount - 1);

  float verticleAngleStep =
    (this->sdfLidar.VerticalScanMaxAngle() -
     this->sdfLidar.VerticalScanMinAngle()).Radian() /
    (vertRangeCount - 1);

  // Angles of ray currently processing, azimuth is horizontal, inclination
  // is vertical
  float inclination = this->sdfLidar.VerticalScanMinAngle().Radian();

  std::string *msgBuffer = this->pointMsg.mutable_data();
  msgBuffer->resize(this->pointMsg.row_step());
  char *msgBufferIndex = msgBuffer->data();

  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < height; ++j)
  {
    float azimuth = this->sdfLidar.HorizontalScanMinAngle().Radian();

    for (uint32_t i = 0; i < width; ++i)
    {
      // Index of current point, and the depth value at that point
      auto index = j * width * channels + i * channels;
      float depth = _laserBuffer[index];

      int fieldIndex = 0;

      // Convert spherical coordinates to Cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      *reinterpret_cast<float*>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::cos(inclination) * std::cos(azimuth);
      *reinterpret_cast<float*>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::cos(inclination) * std::sin(azimuth);
      *reinterpret_cast<float*>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::sin(inclination);

      // Move the index to the next point.
      msgBufferIndex += this->pointMsg.point_step();

      azimuth += angleStep;
    }
    inclination += verticleAngleStep;
  }
}

IGN_SENSORS_REGISTER_SENSOR(Lidar)
