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

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <ignition/msgs/image.pb.h>
#include <ignition/msgs/pointcloud_packed.pb.h>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <ignition/common/Image.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Helpers.hh>

// TODO(louise) Remove these pragmas once ign-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/DepthCamera.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <ignition/transport/Node.hh>

#include <sdf/Sensor.hh>

#include "ignition/sensors/ImageGaussianNoiseModel.hh"
#include "ignition/sensors/ImageNoise.hh"
#include "ignition/sensors/RgbdCameraSensor.hh"
#include "ignition/sensors/RenderingEvents.hh"
#include "ignition/sensors/SensorFactory.hh"

#include "PointCloudUtil.hh"

/// \brief Private data for RgbdCameraSensor
class ignition::sensors::RgbdCameraSensorPrivate
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
  public: ignition::rendering::DepthCameraPtr depthCamera;

  /// \brief Depth data buffer.
  public: float *depthBuffer = nullptr;

  /// \brief Point cloud data buffer.
  public: float *pointCloudBuffer = nullptr;

  /// \brief True if a depth far clipping value has been set.
  public: bool hasDepthFarClip = false;

  /// \brief True if a depth near clipping value has been set.
  public: bool hasDepthNearClip = false;

  /// \brief Depth camera far clipping distance in meters.
  public: double depthFarClip = 10.0;

  /// \brief Depth camera near clipping distance in meters.
  public: double depthNearClip = 0.1;

  /// \brief The number of channels (x, y, z, rgba, ...) in the
  /// point cloud.
  public: unsigned int channels = 4;

  /// \brief Pointer to an image to be published
  public: ignition::rendering::Image image;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Connection from depth camera with new depth data
  public: ignition::common::ConnectionPtr depthConnection;

  /// \brief Connection from depth camera with new point cloud data
  public: ignition::common::ConnectionPtr pointCloudConnection;

  /// \brief Connection to the Manager's scene change event.
  public: ignition::common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief SDF Sensor DOM object.
  public: sdf::Sensor sdfSensor;

  /// \brief The point cloud message.
  public: msgs::PointCloudPacked pointMsg;

  /// \brief Helper class that can fill a msgs::PointCloudPacked
  /// image and depth data.
  public: PointCloudUtil pointsUtil;
};

using namespace ignition;
using namespace sensors;

//////////////////////////////////////////////////
RgbdCameraSensor::RgbdCameraSensor()
  : dataPtr(new RgbdCameraSensorPrivate())
{
}

//////////////////////////////////////////////////
RgbdCameraSensor::~RgbdCameraSensor()
{
  this->dataPtr->depthConnection.reset();
  this->dataPtr->pointCloudConnection.reset();
  if (this->dataPtr->depthBuffer)
    delete [] this->dataPtr->depthBuffer;
  if (this->dataPtr->pointCloudBuffer)
    delete [] this->dataPtr->pointCloudBuffer;
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::Init()
{
  return this->Sensor::Init();
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
    ignerr << "Attempting to a load a RGBD Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    ignerr << "Attempting to a load an RGBD Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  // Create the 2d image publisher
  this->dataPtr->imagePub =
      this->dataPtr->node.Advertise<ignition::msgs::Image>(
          this->Topic() + "/image");
  if (!this->dataPtr->imagePub)
  {
    ignerr << "Unable to create publisher on topic["
      << this->Topic() + "/image" << "].\n";
    return false;
  }

  // Create the depth image publisher
  this->dataPtr->depthPub =
      this->dataPtr->node.Advertise<ignition::msgs::Image>(
          this->Topic() + "/depth_image");
  if (!this->dataPtr->depthPub)
  {
    ignerr << "Unable to create publisher on topic["
      << this->Topic() + "/depth_image" << "].\n";
    return false;
  }

  // Create the point cloud publisher
  this->dataPtr->pointPub =
      this->dataPtr->node.Advertise<ignition::msgs::PointCloudPacked>(
          this->Topic() + "/points");
  if (!this->dataPtr->pointPub)
  {
    ignerr << "Unable to create publisher on topic["
      << this->Topic() + "/points" << "].\n";
    return false;
  }

  if (!this->AdvertiseInfo(this->Topic() + "/camera_info"))
    return false;

  // Initialize the point message.
  // \todo(anyone) The true value in the following function call forces
  // the xyz and rgb fields to be aligned to memory boundaries. This is need
  // by ROS1: https://github.com/ros/common_msgs/pull/77. Ideally, memory
  // alignment should be configured.
  msgs::InitPointCloudPacked(this->dataPtr->pointMsg, this->Name(), true,
      {{"xyz", msgs::PointCloudPacked::Field::FLOAT32},
       {"rgb", msgs::PointCloudPacked::Field::FLOAT32}});

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
  const sdf::Camera *cameraSdf = this->dataPtr->sdfSensor.CameraSensor();

  if (!cameraSdf)
  {
    ignerr << "Unable to access camera SDF element\n";
    return false;
  }

  this->PopulateInfo(cameraSdf);

  int width = cameraSdf->ImageWidth();
  int height = cameraSdf->ImageHeight();

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
      ignwarn << "The depth camera sensor only supports Gaussian noise. "
       << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
       << "] is not supported." << std::endl;
    }
  }

  // \todo(nkoeng) these parameters via sdf
  this->dataPtr->depthCamera->SetAntiAliasing(2);

  math::Angle angle = cameraSdf->HorizontalFov();
  // todo(anyone) verify that rgb pixels align with d for angles >90 degrees.
  if (angle < 0.01 || angle > IGN_PI * 2)
  {
    ignerr << "Invalid horizontal field of view [" << angle << "]\n";

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

  this->dataPtr->depthConnection =
      this->dataPtr->depthCamera->ConnectNewDepthFrame(
        std::bind(&RgbdCameraSensorPrivate::OnNewDepthFrame,
        this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  this->dataPtr->pointCloudConnection =
      this->dataPtr->depthCamera->ConnectNewRgbPointCloud(
        std::bind(&RgbdCameraSensorPrivate::OnNewRgbPointCloud,
        this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  // Set the values of the point message based on the camera information.
  this->dataPtr->pointMsg.set_width(this->ImageWidth());
  this->dataPtr->pointMsg.set_height(this->ImageHeight());
  this->dataPtr->pointMsg.set_row_step(
      this->dataPtr->pointMsg.point_step() * this->ImageWidth());

  return true;
}

/////////////////////////////////////////////////
void RgbdCameraSensor::SetScene(ignition::rendering::ScenePtr _scene)
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
  std::lock_guard<std::mutex> lock(this->mutex);

  unsigned int depthSamples = _width * _height;
  unsigned int depthBufferSize = depthSamples * sizeof(float);

  if (!this->depthBuffer)
    this->depthBuffer = new float[depthSamples];

  memcpy(this->depthBuffer, _scan, depthBufferSize);
}

/////////////////////////////////////////////////
void RgbdCameraSensorPrivate::OnNewRgbPointCloud(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int _channels,
                    const std::string &/*_format*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  unsigned int pointCloudSamples = _width * _height;
  unsigned int pointCloudBufferSize = pointCloudSamples * _channels *
      sizeof(float);
  this->channels = _channels;

  if (!this->pointCloudBuffer)
    this->pointCloudBuffer = new float[pointCloudSamples * _channels];

  memcpy(this->pointCloudBuffer, _scan, pointCloudBufferSize);
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::Update(const ignition::common::Time &_now)
{
  return this->Update(math::secNsecToDuration(_now.sec, _now.nsec));
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  IGN_PROFILE("RgbdCameraSensor::Update");
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->depthCamera)
  {
    ignerr << "Depth or image cameras don't exist.\n";
    return false;
  }

  unsigned int width = this->dataPtr->depthCamera->ImageWidth();
  unsigned int height = this->dataPtr->depthCamera->ImageHeight();
  unsigned int depthSamples = height * width;

  // generate sensor data
  this->Render();

  // create and publish the depthmessage
  if (this->dataPtr->depthPub.HasConnections())
  {
    ignition::msgs::Image msg;
    msg.set_width(width);
    msg.set_height(height);
    msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
               rendering::PF_FLOAT32_R));
    msg.set_pixel_format_type(msgs::PixelFormatType::R_FLOAT32);
    *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->Name());

    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    // The following code is a work around since ign-rendering's depth camera
    // does not support 2 different clipping distances. An assumption is made
    // that the depth clipping distances are within bounds of the rgb clipping
    // distances, if not, the rgb clipping values will take priority.
    if (this->dataPtr->hasDepthNearClip || this->dataPtr->hasDepthFarClip)
    {
      for (unsigned int i = 0; i < depthSamples; i++)
      {
        if (this->dataPtr->hasDepthFarClip &&
            (this->dataPtr->depthBuffer[i] > this->dataPtr->depthFarClip))
        {
          this->dataPtr->depthBuffer[i] = ignition::math::INF_D;
        }
        if (this->dataPtr->hasDepthNearClip &&
            (this->dataPtr->depthBuffer[i] < this->dataPtr->depthNearClip))
        {
          this->dataPtr->depthBuffer[i] = -ignition::math::INF_D;
        }
      }
    }
    msg.set_data(this->dataPtr->depthBuffer,
        rendering::PixelUtil::MemorySize(rendering::PF_FLOAT32_R,
        width, height));

    // publish
    {
      this->AddSequence(msg.mutable_header(), "depthImage");
      IGN_PROFILE("RgbdCameraSensor::Update Publish depth image");
      this->dataPtr->depthPub.Publish(msg);
    }
  }

  if (this->dataPtr->pointCloudBuffer)
  {
    bool filledImgData = false;
    if (this->dataPtr->image.Width() != width
        || this->dataPtr->image.Height() != height)
    {
      this->dataPtr->image =
          rendering::Image(width, height, rendering::PF_R8G8B8);
    }

    // publish point cloud msg
    if (this->dataPtr->pointPub.HasConnections())
    {
      *this->dataPtr->pointMsg.mutable_header()->mutable_stamp() =
        msgs::Convert(_now);
      // Set the time stamp
      this->dataPtr->pointMsg.set_is_dense(true);

      if ((this->dataPtr->hasDepthNearClip || this->dataPtr->hasDepthFarClip)
          && this->dataPtr->depthBuffer)
      {
        for (unsigned int i = 0; i < depthSamples; i++)
        {
          float depthValue = this->dataPtr->depthBuffer[i];
          if (std::isinf(depthValue))
          {
            unsigned int index = i * this->dataPtr->channels;

            this->dataPtr->pointCloudBuffer[index] = depthValue;
            this->dataPtr->pointCloudBuffer[index + 1] = depthValue;
            this->dataPtr->pointCloudBuffer[index + 2] = depthValue;
          }
        }
      }

      {
        IGN_PROFILE("RgbdCameraSensor::Update Fill Point Cloud");
        // fill point cloud msg and image data
        this->dataPtr->pointsUtil.FillMsg(this->dataPtr->pointMsg,
            this->dataPtr->pointCloudBuffer, true,
            this->dataPtr->image.Data<unsigned char>());
        filledImgData = true;
      }

      // publish
      {
        this->AddSequence(this->dataPtr->pointMsg.mutable_header(), "pointMsg");
        IGN_PROFILE("RgbdCameraSensor::Update Publish point cloud");
        this->dataPtr->pointPub.Publish(this->dataPtr->pointMsg);
      }
    }

    // publish the 2d image message
    if (this->dataPtr->imagePub.HasConnections())
    {
      if (!filledImgData)
      {
        IGN_PROFILE("RgbdCameraSensor::Update Fill RGB Image");
        // extract image data from point cloud data
        this->dataPtr->pointsUtil.RGBFromPointCloud(
            this->dataPtr->image.Data<unsigned char>(),
            this->dataPtr->pointCloudBuffer,
            width, height);
      }

      unsigned char *data = this->dataPtr->image.Data<unsigned char>();

      ignition::msgs::Image msg;
      msg.set_width(width);
      msg.set_height(height);
      msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
          rendering::PF_R8G8B8));
      msg.set_pixel_format_type(msgs::PixelFormatType::RGB_INT8);
      *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
      auto frame = msg.mutable_header()->add_data();
      frame->set_key("frame_id");
      frame->add_value(this->Name());
      msg.set_data(data, rendering::PixelUtil::MemorySize(rendering::PF_R8G8B8,
        width, height));

      // publish the image message
      {
        this->AddSequence(msg.mutable_header(), "rgbdImage");
        IGN_PROFILE("RgbdCameraSensor::Update Publish RGB image");
        this->dataPtr->imagePub.Publish(msg);
      }
    }
  }

  // publish the camera info message
  this->PublishInfo(_now);

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

IGN_SENSORS_REGISTER_SENSOR(RgbdCameraSensor)
