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

#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include <gz/common/Image.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/DepthCamera.hh>

#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include <sdf/Sensor.hh>

#include <cstddef>
#include <cstring>
#include <string>

#include "gz/sensors/ImageGaussianNoiseModel.hh"
#include "gz/sensors/ImageNoise.hh"
#include "gz/sensors/RgbdCameraSensor.hh"
#include "gz/sensors/RenderingEvents.hh"
#include "gz/sensors/SensorFactory.hh"

#include "PointCloudUtil.hh"
#include "SensorTailPool.hh"

/// \brief Private data for RgbdCameraSensor
class gz::sensors::RgbdCameraSensorPrivate
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
                    const std::string &/*_format*/);

  /// \brief Point cloud data callback used to get the data from the sensor
  /// \param[in] _scan pointer to the data from the sensor
  /// \param[in] _width width of the point cloud image
  /// \param[in] _height height of the point cloud image
  /// \param[in] _channel bytes used for the point cloud data
  /// \param[in] _format string with the format
  public: void OnNewRgbPointCloud(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int _channels,
                    const std::string &/*_format*/);

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

  /// \brief True if a depth far clipping value has been set.
  public: bool hasDepthFarClip = false;

  /// \brief True if a depth near clipping value has been set.
  public: bool hasDepthNearClip = false;

  /// \brief Depth camera far clipping distance in meters.
  public: double depthFarClip = 10.0;

  /// \brief Depth camera near clipping distance in meters.
  public: double depthNearClip = 0.1;

  /// \brief Frame ID the camera_info message header is expressed.
  public: std::string opticalFrameId{""};

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

  /// \brief Helper class that can fill a msgs::PointCloudPacked
  /// image and depth data.
  public: PointCloudUtil pointsUtil;

  /// \brief Back-pointer to the owning sensor (for AddSequence/FrameId).
  public: RgbdCameraSensor *self = nullptr;

  /// \brief Everything one frame's publish "tail" needs.
  public: struct FrameSlot
  {
    float *depthBuffer = nullptr;       std::size_t depthCap = 0;   // # floats
    float *pointCloudBuffer = nullptr;  std::size_t cloudCap = 0;   // # floats
    unsigned int channels = 4;
    gz::msgs::PointCloudPacked pointMsg;
    bool pointMsgReady = false;
    unsigned int pmWidth = 0, pmHeight = 0;
    gz::rendering::Image image;
    // per-frame context captured on the render thread:
    unsigned int width = 0, height = 0;
    std::chrono::steady_clock::duration now{};
    bool hasDepth = false, hasPoint = false, hasColor = false;
    std::string frameId, opticalFrameId;
  };

  /// \brief Slot the OnNew*Frame callbacks write to during Render().
  public: FrameSlot *writeSlot = nullptr;
  /// \brief Render-thread-only slot used when offload is disabled.
  public: FrameSlot inlineSlot;

  /// \brief (Re)allocate a slot's depth buffer to hold >= _samples floats.
  public: void EnsureDepthCapacity(FrameSlot &_s, std::size_t _samples)
  {
    if (_s.depthCap < _samples)
    {
      delete [] _s.depthBuffer;
      _s.depthBuffer = new float[_samples];
      _s.depthCap = _samples;
    }
  }

  /// \brief (Re)allocate a slot's point-cloud buffer to hold >= _floats floats.
  public: void EnsureCloudCapacity(FrameSlot &_s, std::size_t _floats)
  {
    if (_s.cloudCap < _floats)
    {
      delete [] _s.pointCloudBuffer;
      _s.pointCloudBuffer = new float[_floats];
      _s.cloudCap = _floats;
    }
  }

  /// \brief Initialise a slot's pointMsg layout for its captured dimensions.
  public: void InitSlotPointMsg(FrameSlot &_s);

  /// \brief Run the publish tail (depth msg, point cloud, rgb image) using
  /// only the slot's own buffers/scratch + this->publishers + self->AddSequence.
  public: void PublishTail(FrameSlot &_s);
};

using namespace gz;
using namespace sensors;

//////////////////////////////////////////////////
RgbdCameraSensor::RgbdCameraSensor()
  : dataPtr(new RgbdCameraSensorPrivate())
{
  this->dataPtr->self = this;
}

//////////////////////////////////////////////////
RgbdCameraSensor::~RgbdCameraSensor()
{
  this->dataPtr->depthConnection.reset();
  this->dataPtr->pointCloudConnection.reset();
  delete [] this->dataPtr->inlineSlot.depthBuffer;
  delete [] this->dataPtr->inlineSlot.pointCloudBuffer;
}

//////////////////////////////////////////////////
void RgbdCameraSensor::Implementation::InitSlotPointMsg(FrameSlot &_s)
{
  msgs::InitPointCloudPacked(_s.pointMsg, _s.opticalFrameId, false,
      {{"xyz", msgs::PointCloudPacked::Field::FLOAT32},
       {"rgb", msgs::PointCloudPacked::Field::FLOAT32}});
  _s.pointMsg.set_width(_s.width);
  _s.pointMsg.set_height(_s.height);
  _s.pointMsg.set_row_step(_s.pointMsg.point_step() * _s.width);
  _s.pmWidth = _s.width;
  _s.pmHeight = _s.height;
  _s.pointMsgReady = true;
}

//////////////////////////////////////////////////
void RgbdCameraSensor::Implementation::PublishTail(FrameSlot &_s)
{
  const unsigned int width = _s.width;
  const unsigned int height = _s.height;
  const unsigned int depthSamples = width * height;

  // create and publish the depth message
  if (_s.hasDepth && _s.depthBuffer)
  {
    msgs::Image msg;
    msg.set_width(width);
    msg.set_height(height);
    msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
               rendering::PF_FLOAT32_R));
    msg.set_pixel_format_type(msgs::PixelFormatType::R_FLOAT32);
    *msg.mutable_header()->mutable_stamp() = msgs::Convert(_s.now);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(_s.frameId);

    // The following code is a work around since gz-rendering's depth camera
    // does not support 2 different clipping distances. An assumption is made
    // that the depth clipping distances are within bounds of the rgb clipping
    // distances, if not, the rgb clipping values will take priority.
    if (this->hasDepthNearClip || this->hasDepthFarClip)
    {
      for (unsigned int i = 0; i < depthSamples; i++)
      {
        if (this->hasDepthFarClip &&
            (_s.depthBuffer[i] > this->depthFarClip))
          _s.depthBuffer[i] = math::INF_D;
        if (this->hasDepthNearClip &&
            (_s.depthBuffer[i] < this->depthNearClip))
          _s.depthBuffer[i] = -math::INF_D;
      }
    }
    msg.set_data(_s.depthBuffer,
        rendering::PixelUtil::MemorySize(rendering::PF_FLOAT32_R,
        width, height));

    this->self->AddSequence(msg.mutable_header(), "depthImage");
    GZ_PROFILE("RgbdCameraSensor::Update Publish depth image");
    this->depthPub.Publish(msg);
  }

  if (_s.pointCloudBuffer)
  {
    bool filledImgData = false;
    if (_s.image.Width() != width || _s.image.Height() != height)
      _s.image = rendering::Image(width, height, rendering::PF_R8G8B8);

    if (!_s.pointMsgReady || _s.pmWidth != width || _s.pmHeight != height)
      this->InitSlotPointMsg(_s);

    // publish point cloud msg
    if (_s.hasPoint)
    {
      *_s.pointMsg.mutable_header()->mutable_stamp() = msgs::Convert(_s.now);

      if ((this->hasDepthNearClip || this->hasDepthFarClip) && _s.depthBuffer)
      {
        for (unsigned int i = 0; i < depthSamples; i++)
        {
          float depthValue = _s.depthBuffer[i];
          if (std::isinf(depthValue))
          {
            unsigned int index = i * _s.channels;
            _s.pointCloudBuffer[index] = depthValue;
            _s.pointCloudBuffer[index + 1] = depthValue;
            _s.pointCloudBuffer[index + 2] = depthValue;
          }
        }
      }

      {
        GZ_PROFILE("RgbdCameraSensor::Update Fill Point Cloud");
        this->pointsUtil.FillMsg(_s.pointMsg, _s.pointCloudBuffer, true,
            _s.image.Data<unsigned char>());
        filledImgData = true;
      }

      this->self->AddSequence(_s.pointMsg.mutable_header(), "pointMsg");
      GZ_PROFILE("RgbdCameraSensor::Update Publish point cloud");
      this->pointPub.Publish(_s.pointMsg);
    }

    // publish the 2d image message
    if (_s.hasColor)
    {
      if (!filledImgData)
      {
        GZ_PROFILE("RgbdCameraSensor::Update Fill RGB Image");
        this->pointsUtil.RGBFromPointCloud(_s.image.Data<unsigned char>(),
            _s.pointCloudBuffer, width, height);
      }

      unsigned char *data = _s.image.Data<unsigned char>();

      msgs::Image msg;
      msg.set_width(width);
      msg.set_height(height);
      msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
          rendering::PF_R8G8B8));
      msg.set_pixel_format_type(msgs::PixelFormatType::RGB_INT8);
      *msg.mutable_header()->mutable_stamp() = msgs::Convert(_s.now);
      auto frame = msg.mutable_header()->add_data();
      frame->set_key("frame_id");
      frame->add_value(_s.frameId);
      msg.set_data(data, rendering::PixelUtil::MemorySize(rendering::PF_R8G8B8,
        width, height));

      this->self->AddSequence(msg.mutable_header(), "rgbdImage");
      GZ_PROFILE("RgbdCameraSensor::Update Publish RGB image");
      this->imagePub.Publish(msg);
    }
  }
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

  if (!this->AdvertiseInfo(this->Topic() + "/camera_info"))
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

  // Note: while Gazebo interprets the camera frame to be looking towards +X,
  // other tools, such as ROS, may interpret this frame as looking towards +Z.
  // To make this configurable the user has the option to set an optical frame.
  // If the user has set <optical_frame_id> in the cameraSdf use it,
  // otherwise fall back to the sensor frame.
  if (cameraSdf->OpticalFrameId().empty())
  {
   this->dataPtr->opticalFrameId = this->FrameId();
  }
  else
  {
   this->dataPtr->opticalFrameId = cameraSdf->OpticalFrameId();
  }

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
        std::bind(&RgbdCameraSensorPrivate::OnNewDepthFrame,
        this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

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
void RgbdCameraSensorPrivate::OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &/*_format*/)
{
  GZ_PROFILE("RgbdCameraSensorPrivate::OnNewDepthFrame");
  std::lock_guard<std::mutex> lock(this->mutex);
  if (!this->writeSlot)
    return;
  const std::size_t depthSamples =
      static_cast<std::size_t>(_width) * _height;
  this->EnsureDepthCapacity(*this->writeSlot, depthSamples);
  std::memcpy(this->writeSlot->depthBuffer, _scan,
      depthSamples * sizeof(float));
}

/////////////////////////////////////////////////
void RgbdCameraSensorPrivate::OnNewRgbPointCloud(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int _channels,
                    const std::string &/*_format*/)
{
  GZ_PROFILE("RgbdCameraSensorPrivate::OnNewRgbPointCloud");
  std::lock_guard<std::mutex> lock(this->mutex);
  if (!this->writeSlot)
    return;
  const std::size_t samples = static_cast<std::size_t>(_width) * _height;
  this->writeSlot->channels = _channels;
  this->EnsureCloudCapacity(*this->writeSlot, samples * _channels);
  std::memcpy(this->writeSlot->pointCloudBuffer, _scan,
      samples * _channels * sizeof(float));
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
        std::bind(&RgbdCameraSensorPrivate::OnNewRgbPointCloud,
        this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));
  }
  else if (!this->HasPointConnections() && !this->HasColorConnections() &&
      this->dataPtr->pointCloudConnection)
  {
    this->dataPtr->pointCloudConnection.reset();
  }

  unsigned int width = this->dataPtr->depthCamera->ImageWidth();
  unsigned int height = this->dataPtr->depthCamera->ImageHeight();

  // Choose this frame's slot. (Offload dispatch is added in Task 4; for now,
  // always use the render-thread-only inline slot.)
  RgbdCameraSensor::Implementation::FrameSlot *slot = &this->dataPtr->inlineSlot;

  // Capture per-frame context on the render thread.
  slot->width = width;
  slot->height = height;
  slot->now = _now;
  slot->hasDepth = this->HasDepthConnections();
  slot->hasPoint = this->HasPointConnections();
  slot->hasColor = this->HasColorConnections();
  slot->frameId = this->FrameId();
  slot->opticalFrameId = this->OpticalFrameId();

  this->dataPtr->writeSlot = slot;
  this->Render();
  this->dataPtr->writeSlot = nullptr;

  this->dataPtr->PublishTail(*slot);
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
