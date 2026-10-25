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

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>

#if defined(_MSC_VER)
  #pragma warning(push)
  #pragma warning(disable: 4005)
  #pragma warning(disable: 4251)
#endif
#include <google/protobuf/io/coded_stream.h>
#include <gz/msgs/laserscan.pb.h>
#if defined(_MSC_VER)
  #pragma warning(pop)
#endif

#include <gz/common/Console.hh>
#include <gz/common/Event.hh>
#include <gz/common/Profiler.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>
#include <sdf/Lidar.hh>

#include "gz/sensors/GaussianNoiseModel.hh"
#include "gz/sensors/Lidar.hh"
#include "gz/sensors/Noise.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/SensorTypes.hh"

using namespace gz::sensors;

/// \brief Private data for Lidar class
class gz::sensors::Lidar::Implementation
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub;

  /// \brief Laser message to publish data.
  public: gz::msgs::LaserScan laserMsg;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Sdf sensor.
  public: sdf::Lidar sdfLidar;

  /// \brief Number of channels of the raw lidar buffer
  public: const unsigned int kChannelCount = 3u;
};

namespace
{
using google::protobuf::io::CodedOutputStream;

/// \brief Lidar::Clamp with the range limits given.
/// \return _rangeMax for NaN, a finite _range clamped to the limits, +-inf
/// unchanged.
inline double ClampRange(double _range, double _rangeMin, double _rangeMax)
{
  if (gz::math::isnan(_range))
    return _rangeMax;

  if (std::isfinite(_range))
    return std::clamp(_range, _rangeMin, _rangeMax);

  return _range;
}
}  // namespace

//////////////////////////////////////////////////
Lidar::Lidar()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
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
    gzerr << "Attempting to a load a Lidar sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.LidarSensor() == nullptr)
  {
    gzerr << "Attempting to a load a Lidar sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  // Register publisher
  if (this->Topic().empty())
    this->SetTopic("/lidar");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<gz::msgs::LaserScan>(
        this->Topic());
  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  gzdbg << "Laser scans for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  // Load ray atributes
  this->dataPtr->sdfLidar = *_sdf.LidarSensor();

  if (this->RayCount() == 0 || this->VerticalRayCount() == 0)
  {
    gzerr << "Lidar: Image has 0 size!\n";
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
      // Skip applying noise if gaussian noise params are all 0s
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
      gzwarn << "The lidar sensor only supports Gaussian noise. "
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
gz::common::ConnectionPtr Lidar::ConnectNewLidarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _heighti, unsigned int _channels,
                  const std::string &/*_format*/)> /*_subscriber*/)
{
  return nullptr;
}

//////////////////////////////////////////////////
bool Lidar::Update(const std::chrono::steady_clock::duration &/*_now*/)
{
  gzerr << "No lidar data being updated.\n";
  return false;
}

//////////////////////////////////////////////////
void Lidar::ApplyNoise()
{
  if (this->dataPtr->noises.find(LIDAR_NOISE) != this->dataPtr->noises.end())
  {
    // Range() and Ranges() read the buffer under the same lock.
    std::lock_guard<std::mutex> lock(this->lidarMutex);
    for (unsigned int j = 0; j < this->VerticalRayCount(); ++j)
    {
      for (unsigned int i = 0; i < this->RayCount(); ++i)
      {
        int index = j * this->RayCount() + i;
        double range = this->laserBuffer[index * this->dataPtr->kChannelCount];
        range = this->dataPtr->noises[LIDAR_NOISE]->Apply(range);
        this->laserBuffer[index * this->dataPtr->kChannelCount] =
            this->Clamp(range);
      }
    }
  }
}

//////////////////////////////////////////////////
bool Lidar::PublishLidarScan(const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("Lidar::PublishLidarScan");

  if (!this->dataPtr->pub.HasConnections())
  {
    return false;
  }

  if (!this->laserBuffer)
    return false;

  std::lock_guard<std::mutex> lock(this->lidarMutex);

  *this->dataPtr->laserMsg.mutable_header()->mutable_stamp() =
    msgs::Convert(_now);
  // Remove 'data' entries before adding new ones
  this->dataPtr->laserMsg.mutable_header()->clear_data();
  auto frame = this->dataPtr->laserMsg.mutable_header()->add_data();
  frame->set_key("frame_id");
  // keeping here the sensor name instead of frame_id because the visualizeLidar
  // plugin relies on this value to get the position of the lidar.
  // the ros_gz plugin is using the laserscan.proto 'frame' field
  frame->add_value(this->FrameId());
  this->dataPtr->laserMsg.set_frame(this->FrameId());

  // Store the latest laser scans into laserMsg
  msgs::Set(this->dataPtr->laserMsg.mutable_world_pose(),
      this->Pose());

  // \todo(iche033) interpolate if ray count != range count, i.e. resolution > 1
  // Rows of RangeCount() ranges, VerticalRangeCount() rows, as many as the
  // message holds. The sensor's parameters are read once, not per range:
  // each accessor goes through sdf::Lidar.
  // The message declares that layout (count RangeCount(), vertical_count
  // VerticalRangeCount()), and its rows are the buffer's rows, but it holds
  // one value per ray: with resolution > 1 that is the first rows of the
  // buffer, whole, and the last rows are missing, as with Ranges(). Sizing
  // the message for every range would change what is published.
  const int numRays = this->RayCount() * this->VerticalRayCount();
  const int count = std::min(numRays, static_cast<int>(
      this->RangeCount() * this->VerticalRangeCount()));
  const double rangeMin = this->RangeMin();
  const double rangeMax = this->RangeMax();
  const unsigned int channels = this->dataPtr->kChannelCount;
  const float *buffer = this->laserBuffer;
  const auto rangeAt = [&](int _index)
  {
    return ClampRange(buffer[_index * channels], rangeMin, rangeMax);
  };
  const auto intensityAt = [&](int _index)
  {
    return static_cast<double>(buffer[_index * channels + 1]);
  };
  auto &msg = this->dataPtr->laserMsg;

  if (count < numRays)
  {
    // Fewer ranges than rays (resolution < 1): the message keeps the rays
    // past `count` from earlier scans, so it is updated and published as a
    // whole. Never past its end, though.
    if (msg.ranges_size() != numRays)
    {
      msg.clear_ranges();
      msg.clear_intensities();
      for (int i = 0; i < numRays; ++i)
      {
        msg.add_ranges(gz::math::NAN_F);
        msg.add_intensities(gz::math::NAN_F);
      }
    }
    for (int index = 0; index < count; ++index)
    {
      msg.set_ranges(index, rangeAt(index));
      msg.set_intensities(index, intensityAt(index));
    }
    this->AddSequence(msg.mutable_header());
    this->dataPtr->pub.Publish(msg);
    return true;
  }

  // publish: every range is written, so laserMsg holds everything but the
  // ranges and intensities (fields 14 and 15, the last two), which are
  // written once, straight into the transport's buffer as packed doubles
  // after the rest of the message. laserMsg has no unknown fields (it is
  // never parsed into), which SerializeAsString would put last.
  msg.clear_ranges();
  msg.clear_intensities();
  this->AddSequence(msg.mutable_header());
  const std::string front = msg.SerializeAsString();
  const std::size_t arrayBytes =
      static_cast<std::size_t>(std::max(numRays, 0)) * sizeof(double);
  static_assert(msgs::LaserScan::kIntensitiesFieldNumber < 16,
      "one-byte tags");
  const std::size_t fieldBytes = arrayBytes == 0 ? 0 :
      1 + CodedOutputStream::VarintSize64(arrayBytes) + arrayBytes;
  this->dataPtr->pub.PublishInPlace(front.size() + 2 * fieldBytes,
      [&](char *_buffer)
      {
        memcpy(_buffer, front.data(), front.size());
        if (arrayBytes == 0)
          return true;
        auto *out = reinterpret_cast<uint8_t *>(_buffer + front.size());
        const auto writeField = [&](int _field, auto _value)
        {
          out = CodedOutputStream::WriteTagToArray(
              (static_cast<uint32_t>(_field) << 3) | 2, out);
          out = CodedOutputStream::WriteVarint64ToArray(arrayBytes, out);
          for (int index = 0; index < numRays; ++index)
          {
            const double v = _value(index);
            uint64_t bits;
            memcpy(&bits, &v, sizeof(bits));
            out = CodedOutputStream::WriteLittleEndian64ToArray(bits, out);
          }
        };
        writeField(msgs::LaserScan::kRangesFieldNumber, rangeAt);
        writeField(msgs::LaserScan::kIntensitiesFieldNumber, intensityAt);
        return true;
      });

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
gz::math::Angle Lidar::AngleMin() const
{
  return this->dataPtr->sdfLidar.HorizontalScanMinAngle();
}

//////////////////////////////////////////////////
void Lidar::SetAngleMin(double _angle)
{
  this->dataPtr->sdfLidar.SetHorizontalScanMinAngle(_angle);
}

//////////////////////////////////////////////////
gz::math::Angle Lidar::AngleMax() const
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
  return static_cast<unsigned int>(this->RayCount() *
    this->dataPtr->sdfLidar.HorizontalScanResolution());
}

//////////////////////////////////////////////////
unsigned int Lidar::VerticalRayCount() const
{
  return this->dataPtr->sdfLidar.VerticalScanSamples();
}

//////////////////////////////////////////////////
unsigned int Lidar::VerticalRangeCount() const
{
  unsigned int rows = static_cast<unsigned int>(this->VerticalRayCount() *
    this->dataPtr->sdfLidar.VerticalScanResolution());
  if (rows > 1)
    return rows;
  else
    return 1u;
}

//////////////////////////////////////////////////
void Lidar::SetParent(const std::string &_parent)
{
  Sensor::SetParent(_parent);
}

//////////////////////////////////////////////////
gz::math::Angle Lidar::VerticalAngleMin() const
{
  return this->dataPtr->sdfLidar.VerticalScanMinAngle();
}

//////////////////////////////////////////////////
void Lidar::SetVerticalAngleMin(const double _angle)
{
  this->dataPtr->sdfLidar.SetVerticalScanMinAngle(_angle);
}

//////////////////////////////////////////////////
gz::math::Angle Lidar::VerticalAngleMax() const
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
  if (!this->laserBuffer)
  {
    gzwarn << "Lidar ranges not available" << std::endl;
    return;
  }

  // \todo(iche033) interpolate if ray count != range count, i.e. resolution > 1
  unsigned int size = this->RayCount() * this->VerticalRayCount();
  _ranges.resize(size);

  for (unsigned int i = 0; i < size; ++i)
  {
    _ranges[i] = this->Clamp(
    this->laserBuffer[i * this->dataPtr->kChannelCount]);
  }
}

//////////////////////////////////////////////////
double Lidar::Range(const int _index) const
{
  std::lock_guard<std::mutex> lock(this->lidarMutex);

  // \todo(iche033) interpolate if ray count != range count, i.e. resolution > 1
  if (!this->laserBuffer || _index >= static_cast<int>(
      this->RayCount() * this->VerticalRayCount() *
      this->dataPtr->kChannelCount))
  {
    gzwarn << "Lidar range not available for index: " << _index << std::endl;
    return 0.0;
  }

  return this->Clamp(this->laserBuffer[_index * this->dataPtr->kChannelCount]);
}

//////////////////////////////////////////////////
double Lidar::Retro(const int _index) const
{
  std::lock_guard<std::mutex> lock(this->lidarMutex);

  // \todo(iche033) interpolate if ray count != range count, i.e. resolution > 1
  if (!this->laserBuffer || _index >= static_cast<int>(
      this->RayCount() * this->VerticalRayCount() *
      this->dataPtr->kChannelCount))
  {
    gzwarn << "Lidar retro not available for index: " << _index << std::endl;
    return 0.0;
  }
  return this->laserBuffer[_index * this->dataPtr->kChannelCount + 1];
}

//////////////////////////////////////////////////
int Lidar::Fiducial(const unsigned int /*_index*/) const
{
  return -1;
}

//////////////////////////////////////////////////
uint32_t Lidar::VisibilityMask() const
{
  return this->dataPtr->sdfLidar.VisibilityMask();
}

//////////////////////////////////////////////////
bool Lidar::IsActive() const
{
//  return Sensor::IsActive() ||
//    (this->dataPtr->pub && this->dataPtr->pub->HasConnections());
  return true;
}

//////////////////////////////////////////////////
bool Lidar::HasConnections() const
{
  return this->dataPtr->pub && this->dataPtr->pub.HasConnections();
}

//////////////////////////////////////////////////
double Lidar::Clamp(double _range) const
{
  return ClampRange(_range, this->RangeMin(), this->RangeMax());
}
