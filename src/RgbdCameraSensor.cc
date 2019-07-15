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

#include <ignition/msgs/image.pb.h>
#include <ignition/msgs/pointcloud_packed.pb.h>

#include <ignition/common/Image.hh>
#include <ignition/math/Helpers.hh>

#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/DepthCamera.hh>

#include <sdf/Sensor.hh>

#include "ignition/sensors/RgbdCameraSensor.hh"
#include "ignition/sensors/SensorFactory.hh"

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

  /// \brief Fill the point cloud message.
  public: void FillMsg();

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
  public: ignition::rendering::CameraPtr camera;

  /// \brief Rendering camera
  public: ignition::rendering::DepthCameraPtr depthCamera;

  /// \brief Depth data buffer.
  public: float *depthBuffer = nullptr;

  /// \brief Pointer to an image to be published
  public: ignition::rendering::Image image;

  /// \brief Connection from depth camera with a new image
  public: ignition::common::ConnectionPtr connection;

  /// \brief Connection to the Manager's scene change event.
  public: ignition::common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief SDF Sensor DOM object.
  public: sdf::Sensor sdfSensor;

  /// \brief The point cloud message.
  public: msgs::PointCloudPacked pointMsg;
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
  this->dataPtr->connection.reset();
  if (this->dataPtr->depthBuffer)
    delete [] this->dataPtr->depthBuffer;
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
    return false;

  // Create the depth image publisher
  this->dataPtr->depthPub =
      this->dataPtr->node.Advertise<ignition::msgs::Image>(
          this->Topic() + "/depth_image");
  if (!this->dataPtr->depthPub)
    return false;

  // Create the point cloud publisher
  this->dataPtr->pointPub =
      this->dataPtr->node.Advertise<ignition::msgs::PointCloudPacked>(
          this->Topic() + "/points");
  if (!this->dataPtr->pointPub)
    return false;

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
      this->Scene()->CreateDepthCamera(this->Name() + "_depth");
  this->dataPtr->depthCamera->SetImageWidth(width);
  this->dataPtr->depthCamera->SetImageHeight(height);
  // TODO(anyone) Specify different clipping for each camera on SDF.
  this->dataPtr->depthCamera->SetNearClipPlane(cameraSdf->NearClip());
  this->dataPtr->depthCamera->SetFarClipPlane(cameraSdf->FarClip());

  this->dataPtr->camera = this->Scene()->CreateCamera(this->Name());
  this->dataPtr->camera->SetImageWidth(width);
  this->dataPtr->camera->SetImageHeight(height);
  this->dataPtr->camera->SetNearClipPlane(cameraSdf->NearClip());
  this->dataPtr->camera->SetFarClipPlane(cameraSdf->FarClip());

  // \todo(nkoeng) these parameters via sdf
  this->dataPtr->depthCamera->SetAntiAliasing(2);
  this->dataPtr->camera->SetAntiAliasing(2);

  math::Angle angle = cameraSdf->HorizontalFov();
  // todo(anyone) verify that rgb pixels align with d for angles >90 degrees.
  if (angle < 0.01 || angle > IGN_PI * 2)
  {
    ignerr << "Invalid horizontal field of view [" << angle << "]\n";

    return false;
  }

  this->dataPtr->depthCamera->SetAspectRatio(static_cast<double>(width)/height);
  this->dataPtr->depthCamera->SetHFOV(angle);
  this->dataPtr->camera->SetAspectRatio(static_cast<double>(width)/height);
  this->dataPtr->camera->SetHFOV(angle);

  // Create depth texture when the camera is reconfigured from default values
  this->dataPtr->depthCamera->CreateDepthTexture();

  // \todo(nkoenig) Port Distortion class
  // This->dataPtr->distortion.reset(new Distortion());
  // This->dataPtr->distortion->Load(this->dataPtr->sdf->GetElement("distortion"));

  this->dataPtr->depthCamera->SetImageFormat(ignition::rendering::PF_FLOAT32_R);

  this->dataPtr->camera->SetImageFormat(ignition::rendering::PF_R8G8B8);
  this->dataPtr->image = this->dataPtr->camera->CreateImage();

  this->Scene()->RootVisual()->AddChild(this->dataPtr->depthCamera);
  this->Scene()->RootVisual()->AddChild(this->dataPtr->camera);

  this->dataPtr->connection = this->dataPtr->depthCamera->ConnectNewDepthFrame(
      std::bind(&RgbdCameraSensorPrivate::OnNewDepthFrame, this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  // Set the values of the point message based on the camera information.
  this->dataPtr->pointMsg.set_width(this->ImageWidth());
  this->dataPtr->pointMsg.set_height(this->ImageHeight());
  this->dataPtr->pointMsg.set_row_step(
      this->dataPtr->pointMsg.point_step() * this->ImageWidth()
      * this->ImageHeight());

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
    this->dataPtr->camera = nullptr;
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

  for (unsigned int i = 0; i < depthSamples; ++i)
  {
    // Mask ranges outside of min/max to +/- inf, as per REP 117
    if (this->depthBuffer[i] >= this->sdfSensor.CameraSensor()->FarClip())
    {
      this->depthBuffer[i] = ignition::math::INF_D;
    }
    else if (this->depthBuffer[i] <= this->sdfSensor.CameraSensor()->NearClip())
    {
      this->depthBuffer[i] = -ignition::math::INF_D;
    }
  }
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::Update(const ignition::common::Time &_now)
{
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->depthCamera || !this->dataPtr->camera)
  {
    ignerr << "Depth or image cameras don't exist.\n";
    return false;
  }

  // move the depth camera to the current pose
  // \todo(nkoenig) Ignition Gazebo attaches the rendering camera with
  // this->Name() to the appropriate model link. When physics updates the
  // link's pose, the visual is also moved. This in turn moves the
  // rendering camera. The problem is that the RGBD sensor has two cameras,
  // one with a name that equals 'this->Name()' and one with a name that
  // equal 'this->Name() + "_depth"'. In order to make the depth camera
  // move, we have to manually set the world pose of the depth camera to
  // match this->dataPtr->camera.
  //
  // It would be nice if the whole sensor + rendering + gazebo pipeline
  // could handle this use-case more elegantly.
  this->dataPtr->depthCamera->SetWorldPose(this->dataPtr->camera->WorldPose());

  unsigned int width = this->dataPtr->camera->ImageWidth();
  unsigned int height = this->dataPtr->camera->ImageHeight();

  // create and publish the 2d image message
  {
    // TODO(anyone) Capture calls render functions, so this is inefficient for
    // multi-camera worlds.
    this->dataPtr->camera->Capture(this->dataPtr->image);
    unsigned char *data = this->dataPtr->image.Data<unsigned char>();

    ignition::msgs::Image msg;
    msg.set_width(width);
    msg.set_height(height);
    msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
          this->dataPtr->camera->ImageFormat()));
    msg.set_pixel_format(ignition::common::Image::RGB_INT8);
    msg.set_pixel_format_type(msgs::PixelFormatType::RGB_INT8);
    msg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
    msg.mutable_header()->mutable_stamp()->set_nsec(_now.nsec);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->Name());
    msg.set_data(data, this->dataPtr->camera->ImageMemorySize());

    // publish the image message
    this->dataPtr->imagePub.Publish(msg);
  }

  // create and publish the depthmessage
  {
    // generate sensor data
    this->dataPtr->depthCamera->Update();

    ignition::msgs::Image msg;
    msg.set_width(width);
    msg.set_height(height);
    msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
          this->dataPtr->depthCamera->ImageFormat()));
    msg.set_pixel_format(ignition::common::Image::R_FLOAT32);
    msg.set_pixel_format_type(msgs::PixelFormatType::R_FLOAT32);
    msg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
    msg.mutable_header()->mutable_stamp()->set_nsec(_now.nsec);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->Name());

    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    msg.set_data(this->dataPtr->depthBuffer,
        this->dataPtr->depthCamera->ImageMemorySize());

    // publish
    this->dataPtr->depthPub.Publish(msg);
  }

  {
    // Set the time stamp
    this->dataPtr->pointMsg.mutable_header()->mutable_stamp()->set_sec(
        _now.sec);
    this->dataPtr->pointMsg.mutable_header()->mutable_stamp()->set_nsec(
        _now.nsec);
    this->dataPtr->pointMsg.set_is_dense(true);

    if (this->dataPtr->depthBuffer)
      this->dataPtr->FillMsg();

    this->dataPtr->pointPub.Publish(this->dataPtr->pointMsg);
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

//////////////////////////////////////////////////
void RgbdCameraSensorPrivate::FillMsg()
{
  // Fill message. Logic borrowed from
  // https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_depth_camera.cpp

  uint32_t width = this->pointMsg.width();
  uint32_t height = this->pointMsg.height();

  std::string *msgBuffer = this->pointMsg.mutable_data();
  msgBuffer->resize(this->pointMsg.row_step());
  char *msgBufferIndex = msgBuffer->data();

  // For depth calculation from image
  double fl = width /
    (2.0 * std::tan(this->depthCamera->HFOV().Radian() / 2.0));

  // Image and depth buffers.
  unsigned char *imageData = this->image.Data<unsigned char>();

  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < height; ++j)
  {
    float pAngle = 0.0;
    if (fl > 0 && height > 1)
      pAngle = std::atan2((height-j) - 0.5 * (height - 1), fl);

    for (uint32_t i = 0; i < width; ++i)
    {
      int fieldIndex = 0;

      // Current point depth
      float depth = this->depthBuffer[j * width + i];

      float yAngle = 0.0;
      if (fl > 0 && width > 1)
        yAngle = std::atan2(i - 0.5 * (width - 1), fl);

      *reinterpret_cast<float*>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) = depth;
      *reinterpret_cast<float*>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::tan(yAngle);
      *reinterpret_cast<float*>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::tan(pAngle);

      int imgIndex = i * 3 + j * width * 3;
      int fieldOffset = this->pointMsg.field(fieldIndex).offset();
      // Put image color data for each point, check endianess first.
      if (this->pointMsg.is_bigendian())
      {
        *(msgBufferIndex + fieldOffset + 0) = imageData[imgIndex + 0];
        *(msgBufferIndex + fieldOffset + 1) = imageData[imgIndex + 1];
        *(msgBufferIndex + fieldOffset + 2) = imageData[imgIndex + 2];
      }
      else
      {
        *(msgBufferIndex + fieldOffset + 0) = imageData[imgIndex + 2];
        *(msgBufferIndex + fieldOffset + 1) = imageData[imgIndex + 1];
        *(msgBufferIndex + fieldOffset + 2) = imageData[imgIndex + 0];
      }

      // Add any padding
      msgBufferIndex += this->pointMsg.point_step();
    }
  }
}
IGN_SENSORS_REGISTER_SENSOR(RgbdCameraSensor)
