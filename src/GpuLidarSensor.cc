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
#if defined(_MSC_VER)
  #pragma warning(push)
  #pragma warning(disable: 4005)
  #pragma warning(disable: 4251)
#endif
#include <google/protobuf/io/coded_stream.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#if defined(_MSC_VER)
  #pragma warning(pop)
#endif

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/GpuLidarSensor.hh"
#include "gz/sensors/SensorFactory.hh"

using namespace gz::sensors;

namespace
{
using google::protobuf::io::CodedOutputStream;

/// \brief Publish a message in place: _front and _back are its serialized
/// fields before and after the bytes field number _field, whose _size bytes
/// _fill writes straight into the transport's buffer (false cancels the
/// message). Protobuf serializes fields in field number order and omits an
/// empty bytes field, so the bytes are those Publish() sends for the whole
/// message.
/// \return The result of PublishInPlace
bool PublishWithData(gz::transport::Node::Publisher &_pub,
    const std::string &_front, int _field, std::size_t _size,
    const std::string &_back, const std::function<bool(char *)> &_fill)
{
  // tag (length-delimited) and varint length: at most 5 + 10 bytes
  uint8_t key[15];
  uint8_t *keyEnd = key;
  if (_size > 0)
  {
    keyEnd = CodedOutputStream::WriteTagToArray(
        (static_cast<uint32_t>(_field) << 3) | 2, keyEnd);
    keyEnd = CodedOutputStream::WriteVarint64ToArray(_size, keyEnd);
  }
  const std::size_t keySize = keyEnd - key;
  return _pub.PublishInPlace(
      _front.size() + keySize + _size + _back.size(), [&](char *_buffer)
      {
        memcpy(_buffer, _front.data(), _front.size());
        _buffer += _front.size();
        memcpy(_buffer, key, keySize);
        _buffer += keySize;
        if (!_fill(_buffer))
          return false;
        memcpy(_buffer + _size, _back.data(), _back.size());
        return true;
      });
}
}  // namespace

/// \brief Private data for the GpuLidar class
class gz::sensors::GpuLidarSensor::Implementation
{
  /// \brief Write the data of the point cloud packed message
  /// \param[in] _laserBuffer Lidar data buffer.
  /// \param[out] _out The message's data, row_step * height bytes.
  public: void FillPointCloudData(const float *_laserBuffer, char *_out);

  /// \brief Rendering camera
  public: gz::rendering::GpuRaysPtr gpuRays;

  /// \brief Connection to the Manager's scene change event.
  public: gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief Event that is used to trigger callbacks when a new
  /// lidar frame is available
  public: gz::common::EventT<
          void(const float *_scan, unsigned int _width,
               unsigned int _height, unsigned int _channels,
               const std::string &_format)> lidarEvent;

  /// \brief Callback when new lidar frame is received
  public: void OnNewLidarFrame(const float *_scan, unsigned int _width,
               unsigned int _height, unsigned int _channels,
               const std::string &_format);

  /// \brief Connection to gpuRays new lidar frame event
  public: gz::common::ConnectionPtr lidarFrameConnection;

  /// \brief The point cloud message.
  public: msgs::PointCloudPacked pointMsg;

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief Publisher for the publish point cloud message.
  public: transport::Node::Publisher pointPub;

  /// \brief Cosine and sine of each column's azimuth, FillPointCloudData's
  /// tables.
  public: std::vector<float> azimuthCos;

  /// \brief See azimuthCos.
  public: std::vector<float> azimuthSin;
};

//////////////////////////////////////////////////
GpuLidarSensor::GpuLidarSensor()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
GpuLidarSensor::~GpuLidarSensor()
{
  this->RemoveGpuRays(this->Scene());

  this->dataPtr->sceneChangeConnection.reset();

  if (this->laserBuffer)
  {
    delete [] this->laserBuffer;
    this->laserBuffer = nullptr;
  }
}

/////////////////////////////////////////////////
void GpuLidarSensor::SetScene(gz::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->lidarMutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    this->RemoveGpuRays(this->Scene());
    RenderingSensor::SetScene(_scene);

    if (this->initialized)
      this->CreateLidar();
  }
}

//////////////////////////////////////////////////
void GpuLidarSensor::RemoveGpuRays(
    gz::rendering::ScenePtr _scene)
{
  if (_scene)
  {
    _scene->DestroySensor(this->dataPtr->gpuRays);
  }
  this->dataPtr->gpuRays.reset();
  this->dataPtr->gpuRays = nullptr;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Load(const sdf::Sensor &_sdf)
{
  // Check if this is being loaded via "builtin" or via another sensor
  if (!Lidar::Load(_sdf))
  {
    return false;
  }

  // Initialize the point message.
  msgs::InitPointCloudPacked(this->dataPtr->pointMsg, this->FrameId(), false,
      {{"xyz", msgs::PointCloudPacked::Field::FLOAT32},
      {"intensity", msgs::PointCloudPacked::Field::FLOAT32},
      {"ring", msgs::PointCloudPacked::Field::UINT16}});

  if (this->Scene())
    this->CreateLidar();

  this->dataPtr->sceneChangeConnection =
    RenderingEvents::ConnectSceneChangeCallback(
        std::bind(&GpuLidarSensor::SetScene, this, std::placeholders::_1));

  // Create the point cloud publisher
  this->SetTopic(this->Topic() + "/points");

  this->dataPtr->pointPub =
      this->dataPtr->node.Advertise<gz::msgs::PointCloudPacked>(
          this->Topic());

  if (!this->dataPtr->pointPub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  gzdbg << "Lidar points for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  this->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool GpuLidarSensor::CreateLidar()
{
  this->dataPtr->gpuRays = this->Scene()->CreateGpuRays(
      this->Name());

  if (!this->dataPtr->gpuRays)
  {
    gzerr << "Unable to create gpu laser sensor\n";
    return false;
  }

  this->dataPtr->gpuRays->SetNearClipPlane(this->RangeMin());
  this->dataPtr->gpuRays->SetFarClipPlane(this->RangeMax());

  // Mask ranges outside of min/max to +/- inf, as per REP 117
  this->dataPtr->gpuRays->SetClamp(false);

  this->dataPtr->gpuRays->SetAngleMin(this->AngleMin().Radian());
  this->dataPtr->gpuRays->SetAngleMax(this->AngleMax().Radian());

  this->dataPtr->gpuRays->SetVerticalAngleMin(
      this->VerticalAngleMin().Radian());
  this->dataPtr->gpuRays->SetVerticalAngleMax(
      this->VerticalAngleMax().Radian());

  this->dataPtr->gpuRays->SetRayCount(this->RayCount());
  this->dataPtr->gpuRays->SetVerticalRayCount(
      this->VerticalRayCount());
  this->dataPtr->gpuRays->SetLocalPose(this->Pose());

  this->Scene()->RootVisual()->AddChild(
      this->dataPtr->gpuRays);

  // Set the values on the point message.
  this->dataPtr->pointMsg.set_width(this->dataPtr->gpuRays->RangeCount());
  this->dataPtr->pointMsg.set_height(
      this->dataPtr->gpuRays->VerticalRangeCount());
  this->dataPtr->pointMsg.set_row_step(
      this->dataPtr->pointMsg.point_step() *
      this->dataPtr->pointMsg.width());
  this->dataPtr->gpuRays->SetVisibilityMask(this->VisibilityMask());

  this->dataPtr->lidarFrameConnection =
      this->dataPtr->gpuRays->ConnectNewGpuRaysFrame(
      std::bind(&GpuLidarSensor::OnNewLidarFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  this->AddSensor(this->dataPtr->gpuRays);

  return true;
}

/////////////////////////////////////////////////
void GpuLidarSensor::OnNewLidarFrame(const float *_scan,
    unsigned int _width, unsigned int _height, unsigned int _channels,
    const std::string &_format)
{
  GZ_PROFILE("GpuLidarSensor::OnNewLidarFrame");
  std::lock_guard<std::mutex> lock(this->lidarMutex);

  unsigned int samples = _width * _height * _channels;
  unsigned int lidarBufferSize = samples * sizeof(float);

  if (!this->laserBuffer)
    this->laserBuffer = new float[samples];

  memcpy(this->laserBuffer, _scan, lidarBufferSize);

  if (this->dataPtr->lidarEvent.ConnectionCount() > 0)
  {
    this->dataPtr->lidarEvent(_scan, _width, _height, _channels, _format);
  }
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("GpuLidarSensor::Update");
  if (!this->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->gpuRays)
  {
    gzerr << "GpuRays doesn't exist.\n";
    return false;
  }

  this->Render();

  // Apply noise before publishing the data.
  this->ApplyNoise();

  this->PublishLidarScan(_now);

  if (this->dataPtr->pointPub.HasConnections())
  {
    // is_dense and the points must come from the same frame.
    std::lock_guard<std::mutex> lock(this->lidarMutex);

    // Set the time stamp
    *this->dataPtr->pointMsg.mutable_header()->mutable_stamp() =
      msgs::Convert(_now);
    // Set frame_id
    for (auto i = 0;
         i < this->dataPtr->pointMsg.mutable_header()->data_size();
         ++i)
    {
      if (this->dataPtr->pointMsg.mutable_header()->data(i).key() == "frame_id"
          && this->dataPtr->pointMsg.mutable_header()->data(i).value_size() > 0)
      {
        this->dataPtr->pointMsg.mutable_header()->mutable_data(i)->set_value(
              0,
              this->FrameId());
      }
    }

    // is_dense follows the data in the message, but the message size
    // depends on it, so find it first: a point is invalid if its depth is
    // NaN or +-inf. Most scans stop at their first ray into the sky.
    msgs::PointCloudPacked &msg = this->dataPtr->pointMsg;
    const uint64_t numPoints = static_cast<uint64_t>(msg.width()) *
        msg.height();
    bool isDense = true;
    for (uint64_t i = 0; i < numPoints && isDense; ++i)
    {
      const float depth = this->laserBuffer[i * 3];
      isDense = std::isfinite(depth);
    }

    // The message serialized around the data, which is written once,
    // straight into the transport's buffer. pointMsg itself keeps empty data
    // and is_dense false, and has no unknown fields (it is never parsed
    // into), which SerializeAsString would put last.
    this->AddSequence(msg.mutable_header());
    msgs::PointCloudPacked tail;
    tail.set_is_dense(isDense);

    GZ_PROFILE("GpuLidarSensor::Update Publish point cloud");
    PublishWithData(this->dataPtr->pointPub, msg.SerializeAsString(),
        msgs::PointCloudPacked::kDataFieldNumber,
        static_cast<std::size_t>(msg.row_step()) * msg.height(),
        tail.SerializeAsString(), [&](char *_out)
        {
          this->dataPtr->FillPointCloudData(this->laserBuffer, _out);
          return true;
        });
  }
  return true;
}

/////////////////////////////////////////////////
gz::common::ConnectionPtr GpuLidarSensor::ConnectNewLidarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _height, unsigned int _channels,
                  const std::string &/*_format*/)> _subscriber)
{
  return this->dataPtr->lidarEvent.Connect(_subscriber);
}

/////////////////////////////////////////////////
gz::rendering::GpuRaysPtr GpuLidarSensor::GpuRays() const
{
  return this->dataPtr->gpuRays;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::IsHorizontal() const
{
  return this->dataPtr->gpuRays->IsHorizontal();
}

//////////////////////////////////////////////////
gz::math::Angle GpuLidarSensor::HFOV() const
{
  return this->dataPtr->gpuRays->HFOV();
}

//////////////////////////////////////////////////
gz::math::Angle GpuLidarSensor::VFOV() const
{
  return this->dataPtr->gpuRays->VFOV();
}

//////////////////////////////////////////////////
bool GpuLidarSensor::HasConnections() const
{
  return Lidar::HasConnections() ||
     (this->dataPtr->pointPub && this->dataPtr->pointPub.HasConnections()) ||
     this->dataPtr->lidarEvent.ConnectionCount() > 0u;
}

//////////////////////////////////////////////////
void GpuLidarSensor::Implementation::FillPointCloudData(
    const float *_laserBuffer, char *_out)
{
  GZ_PROFILE("GpuLidarSensorPrivate::FillPointCloudData");
  uint32_t width = this->pointMsg.width();
  uint32_t height = this->pointMsg.height();
  unsigned int channels = 3;

  float angleStep =
    (this->gpuRays->AngleMax() - this->gpuRays->AngleMin()).Radian() /
    (this->gpuRays->RangeCount()-1);

  float verticleAngleStep = (this->gpuRays->VerticalAngleMax() -
      this->gpuRays->VerticalAngleMin()).Radian() /
    (this->gpuRays->VerticalRangeCount()-1);

  // Angles of ray currently processing, azimuth is horizontal, inclination
  // is vertical
  float inclination = this->gpuRays->VerticalAngleMin().Radian();

  // Every row sweeps the same azimuths, accumulated in float as the rows go:
  // take their cosine and sine once per scan instead of once per point.
  this->azimuthCos.resize(width);
  this->azimuthSin.resize(width);
  float azimuth = this->gpuRays->AngleMin().Radian();
  for (uint32_t i = 0; i < width; ++i)
  {
    this->azimuthCos[i] = std::cos(azimuth);
    this->azimuthSin[i] = std::sin(azimuth);
    azimuth += angleStep;
  }

  const uint32_t pointStep = this->pointMsg.point_step();
  const uint32_t offsetX = this->pointMsg.field(0).offset();
  const uint32_t offsetY = this->pointMsg.field(1).offset();
  const uint32_t offsetZ = this->pointMsg.field(2).offset();
  const uint32_t offsetIntensity = this->pointMsg.field(3).offset();
  const uint32_t offsetRing = this->pointMsg.field(4).offset();

  char *msgBufferIndex = _out;
  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < height; ++j)
  {
    const float cosInclination = std::cos(inclination);
    const float sinInclination = std::sin(inclination);
    const uint16_t ring = j;
    const float *point = _laserBuffer + j * width * channels;

    for (uint32_t i = 0; i < width; ++i)
    {
      float depth = point[0];

      // Convert spherical coordinates to Cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      // Same products, in the same order, as depth * cos(inclination) *
      // cos(azimuth) per point.
      const float x = depth * cosInclination * this->azimuthCos[i];
      const float y = depth * cosInclination * this->azimuthSin[i];
      const float z = depth * sinInclination;
      memcpy(msgBufferIndex + offsetX, &x, sizeof(x));
      memcpy(msgBufferIndex + offsetY, &y, sizeof(y));
      memcpy(msgBufferIndex + offsetZ, &z, sizeof(z));
      memcpy(msgBufferIndex + offsetIntensity, point + 1, sizeof(float));
      memcpy(msgBufferIndex + offsetRing, &ring, sizeof(ring));

      // Move the index to the next point.
      msgBufferIndex += pointStep;
      point += channels;
    }
    inclination += verticleAngleStep;
  }
}
