/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>
#include <string>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include <gz/common/Image.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/Util.hh>
#include <gz/math/Helpers.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/DepthCamera.hh>

#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>
#include <gz/rendering/Utils.hh>
#include <gz/transport/Node.hh>

#include <sdf/Sensor.hh>

#include "gz/sensors/ImageGaussianNoiseModel.hh"
#include "gz/sensors/ImageNoise.hh"
#include "gz/sensors/RgbdCameraSensor.hh"
#include "gz/sensors/RenderingEvents.hh"
#include "gz/sensors/SensorFactory.hh"

#include "CameraSensorUtil.hh"
#include "PublishInPlace.hh"

/// \brief Private data for RgbdCameraSensor
class gz::sensors::RgbdCameraSensor::Implementation
{
  /// \brief Depth data callback used to get the data from the sensor
  /// \param[in] _scan pointer to the data from the sensor
  /// \param[in] _width width of the depth image
  /// \param[in] _height height of the depth image
  /// \param[in] _channel bytes used for the depth data
  /// \param[in] _format string with the format
  public: void OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &_format);

  /// \brief Point cloud data callback used to get the data from the sensor
  /// \param[in] _scan pointer to the data from the sensor
  /// \param[in] _width width of the point cloud image
  /// \param[in] _height height of the point cloud image
  /// \param[in] _channel bytes used for the point cloud data
  /// \param[in] _format string with the format
  public: void OnNewRgbPointCloud(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int _channels,
                    const std::string &_format);

  /// \brief Publish an image of the frame Update() renders, as Update()
  /// built it, its data written in place by _fill
  /// \param[in] _pub Publisher
  /// \param[in] _seqKey Sequence key (AddSequence)
  /// \param[in] _format Pixel format of the data
  /// \param[in] _type Pixel format type of the message
  /// \param[in] _width Image width
  /// \param[in] _height Image height
  /// \param[in] _fill Writes the data, see PublishWithData
  public: void PublishImage(transport::Node::Publisher &_pub,
              const std::string &_seqKey, rendering::PixelFormat _format,
              msgs::PixelFormatType _type, unsigned int _width,
              unsigned int _height, const std::function<bool(char *)> &_fill);

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher imagePub;

  /// \brief publisher to publish depth images
  public: transport::Node::Publisher depthPub;

  /// \brief publisher to publish point cloud
  public: transport::Node::Publisher pointPub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Rendering camera
  public: gz::rendering::DepthCameraPtr depthCamera;

  /// \brief The depth frame with the depth clip applied, which the point
  /// cloud reads too. Only with a depth clip.
  public: float *depthBuffer = nullptr;

  /// \brief True if a depth far clipping value has been set.
  public: bool hasDepthFarClip = false;

  /// \brief True if a depth near clipping value has been set.
  public: bool hasDepthNearClip = false;

  /// \brief Depth camera far clipping distance in meters.
  public: double depthFarClip = 10.0;

  /// \brief Depth camera near clipping distance in meters.
  public: double depthNearClip = 0.1;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Connection from depth camera with new depth data
  public: gz::common::ConnectionPtr depthConnection;

  /// \brief Connection from depth camera with new point cloud data
  public: gz::common::ConnectionPtr pointCloudConnection;

  /// \brief Connection to the Manager's scene change event.
  public: gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief SDF Sensor DOM object.
  public: sdf::Sensor sdfSensor;

  /// \brief The point cloud message: header and layout, the data is
  /// written in place (OnNewRgbPointCloud)
  public: msgs::PointCloudPacked pointMsg;

  /// \brief is_dense of the last point cloud published
  public: bool pointsDense = false;

  /// \brief The sensor, for the frame callbacks, which publish
  public: RgbdCameraSensor *sensor = nullptr;

  /// \brief Stamp of the frame Update() renders, set only while it
  /// renders, the only time the frame callbacks publish
  public: std::optional<std::chrono::steady_clock::duration> frameStamp;
};

using namespace gz;
using namespace sensors;

//////////////////////////////////////////////////
RgbdCameraSensor::RgbdCameraSensor()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
  this->dataPtr->sensor = this;
}

//////////////////////////////////////////////////
RgbdCameraSensor::~RgbdCameraSensor()
{
  this->dataPtr->depthConnection.reset();
  this->dataPtr->pointCloudConnection.reset();
  if (this->dataPtr->depthBuffer)
    delete [] this->dataPtr->depthBuffer;
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::Load(const sdf::Sensor &_sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!Sensor::Load(_sdf))
  {
    return false;
  }

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::RGBD_CAMERA)
  {
    gzerr << "Attempting to a load a RGBD Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    gzerr << "Attempting to a load an RGBD Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  // Create the 2d image publisher
  this->dataPtr->imagePub =
      this->dataPtr->node.Advertise<msgs::Image>(
          this->Topic() + "/image");
  if (!this->dataPtr->imagePub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() + "/image" << "].\n";
    return false;
  }

  gzdbg << "RGB images for [" << this->Name() << "] advertised on ["
         << this->Topic() << "/image]" << std::endl;

  // Create the depth image publisher
  this->dataPtr->depthPub =
      this->dataPtr->node.Advertise<msgs::Image>(
          this->Topic() + "/depth_image");
  if (!this->dataPtr->depthPub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() + "/depth_image" << "].\n";
    return false;
  }

  gzdbg << "Depth images for [" << this->Name() << "] advertised on ["
         << this->Topic() << "/depth_image]" << std::endl;

  // Create the point cloud publisher
  this->dataPtr->pointPub =
      this->dataPtr->node.Advertise<msgs::PointCloudPacked>(
          this->Topic() + "/points");
  if (!this->dataPtr->pointPub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() + "/points" << "].\n";
    return false;
  }

  gzdbg << "Points for [" << this->Name() << "] advertised on ["
         << this->Topic() << "/points]" << std::endl;

  if (_sdf.CameraSensor()->Triggered())
  {
    std::string triggerTopic = _sdf.CameraSensor()->TriggerTopic();
    if (triggerTopic.empty())
    {
      triggerTopic = transport::TopicUtils::AsValidTopic(this->Topic() +
                                                         "/trigger");
    }
    this->SetTriggered(true, triggerTopic);
  }

  if (!_sdf.CameraSensor()->CameraInfoTopic().empty())
  {
    this->SetInfoTopic(_sdf.CameraSensor()->CameraInfoTopic());
  }
  else
  {
    this->SetInfoTopic(this->Topic() + "/camera_info");
  }
  if (!this->AdvertiseInfo())
    return false;

  if (this->Scene())
  {
    this->CreateCameras();
  }

  this->dataPtr->sceneChangeConnection =
      RenderingEvents::ConnectSceneChangeCallback(
      std::bind(&RgbdCameraSensor::SetScene, this, std::placeholders::_1));

  this->dataPtr->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::CreateCameras()
{
  sdf::Camera *cameraSdf = this->dataPtr->sdfSensor.CameraSensor();

  if (!cameraSdf)
  {
    gzerr << "Unable to access camera SDF element\n";
    return false;
  }

  unsigned int width = cameraSdf->ImageWidth();
  unsigned int height = cameraSdf->ImageHeight();

  if (width == 0u || height == 0u)
  {
    gzerr << "Unable to create an RGBD camera sensor with 0 width or height."
          << std::endl;
    return false;
  }

  this->dataPtr->depthCamera =
      this->Scene()->CreateDepthCamera(this->Name());
  this->dataPtr->depthCamera->SetImageWidth(width);
  this->dataPtr->depthCamera->SetImageHeight(height);
  this->dataPtr->depthCamera->SetNearClipPlane(cameraSdf->NearClip());
  this->dataPtr->depthCamera->SetFarClipPlane(cameraSdf->FarClip());

  // Depth camera clip params are new and only override the camera clip
  // params if specified.
  if (cameraSdf->HasDepthCamera())
  {
    if (cameraSdf->HasDepthFarClip())
    {
      this->dataPtr->hasDepthFarClip = true;
      this->dataPtr->depthFarClip = cameraSdf->DepthFarClip();
    }
    if (cameraSdf->HasDepthNearClip())
    {
      this->dataPtr->hasDepthNearClip = true;
      this->dataPtr->depthNearClip = cameraSdf->DepthNearClip();
    }
  }

  this->dataPtr->depthCamera->SetVisibilityMask(cameraSdf->VisibilityMask());
  this->dataPtr->depthCamera->SetLocalPose(this->Pose());

  this->AddSensor(this->dataPtr->depthCamera);

  const std::map<SensorNoiseType, sdf::Noise> noises = {
    {CAMERA_NOISE, cameraSdf->ImageNoise()},
  };

  for (const auto & [noiseType, noiseSdf] : noises)
  {
    // Add gaussian noise to camera sensor
    if (noiseSdf.Type() == sdf::NoiseType::GAUSSIAN)
    {
      this->dataPtr->noises[noiseType] =
        ImageNoiseFactory::NewNoiseModel(noiseSdf, "rgbd_camera");

      std::dynamic_pointer_cast<ImageGaussianNoiseModel>(
           this->dataPtr->noises[noiseType])->SetCamera(
             this->dataPtr->depthCamera);
    }
    else if (noiseSdf.Type() != sdf::NoiseType::NONE)
    {
      gzwarn << "The depth camera sensor only supports Gaussian noise. "
       << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
       << "] is not supported." << std::endl;
    }
  }

  // \todo(nkoeng) these parameters via sdf
  this->dataPtr->depthCamera->SetAntiAliasing(2);

  math::Angle angle = cameraSdf->HorizontalFov();
  // todo(anyone) verify that rgb pixels align with d for angles >90 degrees.
  if (angle < 0.01 || angle > GZ_PI * 2)
  {
    gzerr << "Invalid horizontal field of view [" << angle << "]\n";

    return false;
  }

  this->dataPtr->depthCamera->SetAspectRatio(static_cast<double>(width)/height);
  this->dataPtr->depthCamera->SetHFOV(angle);

  // Create depth texture when the camera is reconfigured from default values
  this->dataPtr->depthCamera->CreateDepthTexture();

  // \todo(nkoenig) Port Distortion class
  // This->dataPtr->distortion.reset(new Distortion());
  // This->dataPtr->distortion->Load(this->dataPtr->sdf->GetElement("distortion"));

  this->Scene()->RootVisual()->AddChild(this->dataPtr->depthCamera);

  this->UpdateLensIntrinsicsAndProjection(this->dataPtr->depthCamera,
      *cameraSdf);

  this->PopulateInfo(cameraSdf);

  this->dataPtr->depthConnection =
      this->dataPtr->depthCamera->ConnectNewDepthFrame(
        std::bind(&RgbdCameraSensor::Implementation::OnNewDepthFrame,
        this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  // Initialize the point message.
  msgs::InitPointCloudPacked(this->dataPtr->pointMsg, this->OpticalFrameId(),
      false,
      {{"xyz", msgs::PointCloudPacked::Field::FLOAT32},
       {"rgb", msgs::PointCloudPacked::Field::FLOAT32}});

  // Set the values of the point message based on the camera information.
  this->dataPtr->pointMsg.set_width(this->ImageWidth());
  this->dataPtr->pointMsg.set_height(this->ImageHeight());
  this->dataPtr->pointMsg.set_row_step(
      this->dataPtr->pointMsg.point_step() * this->ImageWidth());

  return true;
}

/////////////////////////////////////////////////
void RgbdCameraSensor::SetScene(rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    // TODO(anyone) Remove cameras from current scene
    this->dataPtr->depthCamera = nullptr;
    RenderingSensor::SetScene(_scene);

    if (this->dataPtr->initialized)
      this->CreateCameras();
  }
}

/////////////////////////////////////////////////
void RgbdCameraSensor::Implementation::OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &/*_format*/)
{
  GZ_PROFILE("RgbdCameraSensorPrivate::OnNewDepthFrame");
  std::lock_guard<std::mutex> lock(this->mutex);

  unsigned int depthSamples = _width * _height;
  unsigned int depthBufferSize = depthSamples * sizeof(float);

  // The depth image is published from the frame, written once, straight
  // into the transport's buffer. Only the depth clip needs a copy, which
  // the point cloud reads too.
  const bool clip = this->hasDepthNearClip || this->hasDepthFarClip;
  const float *depth = _scan;
  if (clip)
  {
    if (!this->depthBuffer)
      this->depthBuffer = new float[depthSamples];
    memcpy(this->depthBuffer, _scan, depthBufferSize);
    depth = this->depthBuffer;
  }

  if (!this->frameStamp || !this->sensor->HasDepthConnections())
    return;

  // The following code is a work around since gz-rendering's depth camera
  // does not support 2 different clipping distances. An assumption is made
  // that the depth clipping distances are within bounds of the rgb clipping
  // distances, if not, the rgb clipping values will take priority.
  if (clip)
  {
    for (unsigned int i = 0; i < depthSamples; i++)
    {
      if (this->hasDepthFarClip &&
          (this->depthBuffer[i] > this->depthFarClip))
      {
        this->depthBuffer[i] = math::INF_D;
      }
      if (this->hasDepthNearClip &&
          (this->depthBuffer[i] < this->depthNearClip))
      {
        this->depthBuffer[i] = -math::INF_D;
      }
    }
  }

  GZ_PROFILE("RgbdCameraSensor::Update Publish depth image");
  this->PublishImage(this->depthPub, "depthImage", rendering::PF_FLOAT32_R,
      msgs::PixelFormatType::R_FLOAT32, _width, _height, [&](char *_out)
      {
        memcpy(_out, depth, depthBufferSize);
        return true;
      });
}

/////////////////////////////////////////////////
void RgbdCameraSensor::Implementation::PublishImage(
    transport::Node::Publisher &_pub, const std::string &_seqKey,
    rendering::PixelFormat _format, msgs::PixelFormatType _type,
    unsigned int _width, unsigned int _height,
    const std::function<bool(char *)> &_fill)
{
  // The fields before and after the data
  msgs::Image head;
  head.set_width(_width);
  head.set_height(_height);
  head.set_step(_width * rendering::PixelUtil::BytesPerPixel(_format));
  *head.mutable_header()->mutable_stamp() = msgs::Convert(*this->frameStamp);
  auto frame = head.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->sensor->FrameId());
  this->sensor->AddSequence(head.mutable_header(), _seqKey);
  msgs::Image tail;
  tail.set_pixel_format_type(_type);

  PublishWithData(_pub, head.SerializeAsString(),
      msgs::Image::kDataFieldNumber,
      rendering::PixelUtil::MemorySize(_format, _width, _height),
      tail.SerializeAsString(), _fill);
}

/////////////////////////////////////////////////
void RgbdCameraSensor::Implementation::OnNewRgbPointCloud(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int _channels,
                    const std::string &/*_format*/)
{
  GZ_PROFILE("RgbdCameraSensorPrivate::OnNewRgbPointCloud");
  std::lock_guard<std::mutex> lock(this->mutex);

  if (!this->frameStamp)
    return;
  const bool points = this->sensor->HasPointConnections();
  const bool color = this->sensor->HasColorConnections();
  const uint32_t n = _width * _height;
  msgs::PointCloudPacked &msg = this->pointMsg;
  if (!points && !color)
    return;
  if (_channels != 4u)
  {
    gzerr << "RgbdCameraSensor: unexpected point cloud frame ("
          << _channels << " channels)" << std::endl;
    return;
  }

  // The point cloud and the image are published from the frame, each
  // written once, straight into the transport's buffer: the bytes are
  // those of the messages Update built (PointCloudUtil::FillMsg,
  // RGBFromPointCloud). With a depth clip, points whose depth is +-inf take
  // it as x, y and z.
  // (the depth camera delivers the depth frame first: OnNewDepthFrame
  // clipped this frame's depth into depthBuffer)
  const float *depth = (this->hasDepthNearClip || this->hasDepthFarClip) ?
      this->depthBuffer : nullptr;
  // PointCloudUtil::DecodeRGBAFromFloat, inline: one colour byte of point _i
  auto rgb = [_scan](uint32_t _i, int _shift)
  {
    uint32_t bits;
    memcpy(&bits, &_scan[_i * 4 + 3], sizeof(bits));
    return static_cast<char>(bits >> _shift & 0xFF);
  };

  if (points && n != msg.width() * msg.height())
  {
    gzerr << "RgbdCameraSensor: point cloud frame of " << _width << "x"
          << _height << " for a " << msg.width() << "x" << msg.height()
          << " message" << std::endl;
  }
  else if (points)
  {
    // The layout InitPointCloudPacked gives (see CreateCameras): x, y, z,
    // then rgb as b, g, r and a zero byte (little endian)
    GZ_ASSERT(msg.point_step() == 16u && msg.field_size() == 4 &&
        msg.field(3).offset() == 12u && !msg.is_bigendian() &&
        msg.data().empty() && !msg.is_dense(),
        "RgbdCameraSensor: unexpected point cloud layout");
    *msg.mutable_header()->mutable_stamp() = msgs::Convert(*this->frameStamp);
    this->sensor->AddSequence(msg.mutable_header(), "pointMsg");
    // the fields before the data
    const std::string front = msg.SerializeAsString();

    GZ_PROFILE("RgbdCameraSensor::Update Publish point cloud");
    // is_dense follows the data, but the message size depends on it:
    // serialize with the last cloud's, find this one's while writing the
    // data, and only when it changed write again.
    bool dense = this->pointsDense;
    for (int attempt = 0; attempt < 2; ++attempt)
    {
      msgs::PointCloudPacked tail;
      tail.set_is_dense(dense);
      bool isDense = true;
      bool filled = false;
      PublishWithData(this->pointPub, front,
          msgs::PointCloudPacked::kDataFieldNumber,
          static_cast<std::size_t>(msg.row_step()) * msg.height(),
          tail.SerializeAsString(), [&](char *_out)
          {
            for (uint32_t i = 0; i < n; ++i, _out += 16)
            {
              float xyz[3] = {_scan[i * 4], _scan[i * 4 + 1], _scan[i * 4 + 2]};
              if (depth && std::isinf(depth[i]))
                xyz[0] = xyz[1] = xyz[2] = depth[i];
              if (isDense)
              {
                isDense = std::isfinite(xyz[0]) && std::isfinite(xyz[1]) &&
                    std::isfinite(xyz[2]);
              }
              memcpy(_out, xyz, sizeof(xyz));
              _out[12] = rgb(i, 8);
              _out[13] = rgb(i, 16);
              _out[14] = rgb(i, 24);
              _out[15] = 0;
            }
            filled = true;
            return isDense == dense;
          });
      if (!filled || isDense == dense)
        break;
      dense = isDense;
    }
    this->pointsDense = dense;
  }

  if (color)
  {
    GZ_PROFILE("RgbdCameraSensor::Update Publish RGB image");
    this->PublishImage(this->imagePub, "rgbdImage", rendering::PF_R8G8B8,
        msgs::PixelFormatType::RGB_INT8, _width, _height, [&](char *_out)
        {
          for (uint32_t i = 0; i < n; ++i, _out += 3)
          {
            _out[0] = rgb(i, 24);
            _out[1] = rgb(i, 16);
            _out[2] = rgb(i, 8);
          }
          return true;
        });
  }
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("RgbdCameraSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->depthCamera)
  {
    gzerr << "Depth or image cameras don't exist.\n";
    return false;
  }

  if (this->HasInfoConnections())
  {
    // publish the camera info message
    this->PublishInfo(_now);
  }

  // don't render if there are no subscribers
  if (!this->HasColorConnections() && !this->HasDepthConnections() &&
    !this->HasPointConnections())
  {
    return false;
  }

  if ((this->HasPointConnections() || HasColorConnections()) &&
      !this->dataPtr->pointCloudConnection)
  {
    this->dataPtr->pointCloudConnection =
        this->dataPtr->depthCamera->ConnectNewRgbPointCloud(
        std::bind(&RgbdCameraSensor::Implementation::OnNewRgbPointCloud,
        this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));
  }
  else if (!this->HasPointConnections() && !this->HasColorConnections() &&
      this->dataPtr->pointCloudConnection)
  {
    this->dataPtr->pointCloudConnection.reset();
  }

  // generate sensor data; OnNewDepthFrame publishes the depth image,
  // OnNewRgbPointCloud the point cloud and the image
  this->dataPtr->frameStamp = _now;
  this->Render();
  this->dataPtr->frameStamp.reset();

  return true;
}

//////////////////////////////////////////////////
unsigned int RgbdCameraSensor::ImageWidth() const
{
  return this->dataPtr->depthCamera->ImageWidth();
}

//////////////////////////////////////////////////
unsigned int RgbdCameraSensor::ImageHeight() const
{
  return this->dataPtr->depthCamera->ImageHeight();
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::HasConnections() const
{
  return this->HasColorConnections() || this->HasDepthConnections() ||
         this->HasPointConnections() || this->HasInfoConnections();
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::HasColorConnections() const
{
  return this->dataPtr->imagePub && this->dataPtr->imagePub.HasConnections();
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::HasDepthConnections() const
{
  return this->dataPtr->depthPub && this->dataPtr->depthPub.HasConnections();
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::HasPointConnections() const
{
  return this->dataPtr->pointPub && this->dataPtr->pointPub.HasConnections();
}
