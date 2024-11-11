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

#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include <mutex>

#include <gz/common/Console.hh>
#include <gz/common/Image.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/SystemPaths.hh>

#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>

#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/DepthCameraSensor.hh"
#include "gz/sensors/Manager.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/ImageGaussianNoiseModel.hh"
#include "gz/sensors/ImageNoise.hh"
#include "gz/sensors/RenderingEvents.hh"

#include "PointCloudUtil.hh"

// undefine near and far macros from windows.h
#ifdef _WIN32
  #undef near
  #undef far
#endif

/// \brief Private data for DepthCameraSensor
class gz::sensors::DepthCameraSensorPrivate
{
  /// \brief Save an image
  /// \param[in] _data the image data to be saved
  /// \param[in] _width width of image in pixels
  /// \param[in] _height height of image in pixels
  /// \param[in] _format The format the data is in
  /// \return True if the image was saved successfully. False can mean
  /// that the path provided to the constructor does exist and creation
  /// of the path was not possible.
  /// \sa ImageSaver
  public: bool SaveImage(const float *_data, unsigned int _width,
    unsigned int _height, gz::common::Image::PixelFormatType _format);

  /// \brief Helper function to convert depth data to depth image
  /// \param[in] _data depth data
  /// \param[out] _imageBuffer resulting depth image data
  /// \param[in] _width width of image
  /// \param[in] _height height of image
  public: bool ConvertDepthToImage(const float *_data,
    unsigned char *_imageBuffer, unsigned int _width, unsigned int _height);

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

    /// \brief Rendering camera
  public: gz::rendering::DepthCameraPtr depthCamera;

  /// \brief Depth data buffer.
  public: float *depthBuffer = nullptr;

  /// \brief point cloud data buffer.
  public: float *pointCloudBuffer = nullptr;

  /// \brief xyz data buffer.
  public: float *xyzBuffer = nullptr;

  /// \brief Near clip distance.
  public: float near = 0.0;

  /// \brief Pointer to an image to be published
  public: gz::rendering::Image image;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: gz::common::EventT<
          void(const gz::msgs::Image &)> imageEvent;

  /// \brief Connection from depth camera with new depth data
  public: gz::common::ConnectionPtr depthConnection;

  /// \brief Connection from depth camera with new point cloud data
  public: gz::common::ConnectionPtr pointCloudConnection;

  /// \brief Connection to the Manager's scene change event.
  public: gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief True to save images
  public: bool saveImage = false;

  /// \brief path directory to where images are saved
  public: std::string saveImagePath = "./";

  /// \prefix of an image name
  public: std::string saveImagePrefix = "./";

  /// \brief counter used to set the image filename
  public: std::uint64_t saveImageCounter = 0;

  /// \brief SDF Sensor DOM object.
  public: sdf::Sensor sdfSensor;

  /// \brief The point cloud message.
  public: msgs::PointCloudPacked pointMsg;

  /// \brief Helper class that can fill a msgs::PointCloudPacked
  /// image and depth data.
  public: PointCloudUtil pointsUtil;

  /// \brief publisher to publish point cloud
  public: transport::Node::Publisher pointPub;
};

using namespace gz;
using namespace sensors;

//////////////////////////////////////////////////
bool DepthCameraSensorPrivate::ConvertDepthToImage(
    const float *_data,
    unsigned char *_imageBuffer,
    unsigned int _width, unsigned int _height)
{
  float maxDepth = 0;
  for (unsigned int i = 0; i < _height * _width; ++i)
  {
    if (_data[i] > maxDepth && !std::isinf(_data[i]))
    {
      maxDepth = _data[i];
    }
  }
  double factor = 255 / maxDepth;
  for (unsigned int j = 0; j < _height * _width; ++j)
  {
    unsigned char d = static_cast<unsigned char>(255 - (_data[j] * factor));
    _imageBuffer[j * 3] = d;
    _imageBuffer[j * 3 + 1] = d;
    _imageBuffer[j * 3 + 2] = d;
  }
  return true;
}

//////////////////////////////////////////////////
bool DepthCameraSensorPrivate::SaveImage(const float *_data,
    unsigned int _width, unsigned int _height,
    common::Image::PixelFormatType /*_format*/)
{
  // Attempt to create the directory if it doesn't exist
  if (!common::isDirectory(this->saveImagePath))
  {
    if (!common::createDirectories(this->saveImagePath))
      return false;
  }

  if (_width == 0 || _height == 0)
    return false;

  common::Image localImage;

  unsigned int depthSamples = _width * _height;
  unsigned int depthBufferSize = depthSamples * 3;

  unsigned char * imgDepthBuffer = new unsigned char[depthBufferSize];

  this->ConvertDepthToImage(_data, imgDepthBuffer, _width, _height);

  std::string filename = this->saveImagePrefix +
                         std::to_string(this->saveImageCounter) + ".png";
  ++this->saveImageCounter;

  localImage.SetFromData(imgDepthBuffer, _width, _height,
      common::Image::RGB_INT8);
  localImage.SavePNG(
      common::joinPaths(this->saveImagePath, filename));

  delete[] imgDepthBuffer;
  return true;
}

//////////////////////////////////////////////////
DepthCameraSensor::DepthCameraSensor()
  : CameraSensor(), dataPtr(new DepthCameraSensorPrivate())
{
}

//////////////////////////////////////////////////
DepthCameraSensor::~DepthCameraSensor()
{
  this->dataPtr->depthConnection.reset();
  this->dataPtr->pointCloudConnection.reset();
  if (this->dataPtr->depthBuffer)
    delete [] this->dataPtr->depthBuffer;
  if (this->dataPtr->pointCloudBuffer)
    delete [] this->dataPtr->pointCloudBuffer;
  if (this->dataPtr->xyzBuffer)
    delete [] this->dataPtr->xyzBuffer;
}

//////////////////////////////////////////////////
bool DepthCameraSensor::Init()
{
  return this->CameraSensor::Init();
}

//////////////////////////////////////////////////
bool DepthCameraSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool DepthCameraSensor::Load(const sdf::Sensor &_sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!Sensor::Load(_sdf))
  {
    return false;
  }

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::DEPTH_CAMERA)
  {
    gzerr << "Attempting to a load a Depth Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    gzerr << "Attempting to a load a Depth Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  if (this->Topic().empty())
    this->SetTopic("/camera/depth");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<msgs::Image>(
          this->Topic());
  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  gzdbg << "Depth images for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

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

  if (!this->AdvertiseInfo())
    return false;

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

  if (this->Scene())
  {
    this->CreateCamera();
  }

  this->dataPtr->sceneChangeConnection =
      RenderingEvents::ConnectSceneChangeCallback(
      std::bind(&DepthCameraSensor::SetScene, this, std::placeholders::_1));

  this->dataPtr->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool DepthCameraSensor::CreateCamera()
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
    gzerr << "Unable to create a depth camera sensor with 0 width or height."
          << std::endl;
    return false;
  }

  double far = cameraSdf->FarClip();
  double near = cameraSdf->NearClip();


  this->dataPtr->depthCamera = this->Scene()->CreateDepthCamera(
      this->Name());
  this->dataPtr->depthCamera->SetImageWidth(width);
  this->dataPtr->depthCamera->SetImageHeight(height);
  this->dataPtr->depthCamera->SetNearClipPlane(near);
  this->dataPtr->depthCamera->SetFarClipPlane(far);
  this->dataPtr->depthCamera->SetVisibilityMask(
      cameraSdf->VisibilityMask());
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
      // Skip applying noise if mean and stddev are 0 - this avoids
      // doing an extra render pass in gz-rendering
      // Note ImageGaussianNoiseModel only uses mean and stddev and does not
      // use bias parameters.
      if (!math::equal(noiseSdf.Mean(), 0.0) ||
          !math::equal(noiseSdf.StdDev(), 0.0))
      {
        this->dataPtr->noises[noiseType] =
          ImageNoiseFactory::NewNoiseModel(noiseSdf, "depth");

        std::dynamic_pointer_cast<ImageGaussianNoiseModel>(
             this->dataPtr->noises[noiseType])->SetCamera(
               this->dataPtr->depthCamera);
      }
    }
    else if (noiseSdf.Type() != sdf::NoiseType::NONE)
    {
      gzwarn << "The depth camera sensor only supports Gaussian noise. "
       << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
       << "] is not supported." << std::endl;
    }
  }

  // Near clip plane not set because we need to be able to detect occlusion
  // from objects before near clip plane
  this->dataPtr->near = near;

  // \todo(nkoeng) these parameters via sdf
  this->dataPtr->depthCamera->SetAntiAliasing(2);

  math::Angle angle = cameraSdf->HorizontalFov();
  if (angle < 0.01 || angle > GZ_PI*2)
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
  // This->dataPtr->distortion->Load(this->sdf->GetElement("distortion"));

  this->Scene()->RootVisual()->AddChild(this->dataPtr->depthCamera);

  this->UpdateLensIntrinsicsAndProjection(this->dataPtr->depthCamera,
      *cameraSdf);

  // Create the directory to store frames
  if (cameraSdf->SaveFrames())
  {
    this->dataPtr->saveImagePath = cameraSdf->SaveFramesPath();
    this->dataPtr->saveImagePrefix = this->Name() + "_";
    this->dataPtr->saveImage = true;
  }

  this->PopulateInfo(cameraSdf);

  this->dataPtr->depthConnection =
      this->dataPtr->depthCamera->ConnectNewDepthFrame(
      std::bind(&DepthCameraSensor::OnNewDepthFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  // Initialize the point message.
  // \todo(anyone) The true value in the following function call forces
  // the xyz and rgb fields to be aligned to memory boundaries. This is need
  // by ROS1: https://github.com/ros/common_msgs/pull/77. Ideally, memory
  // alignment should be configured.
  msgs::InitPointCloudPacked(
        this->dataPtr->pointMsg,
        this->OpticalFrameId(),
        true,
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
void DepthCameraSensor::OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &_format)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  unsigned int depthSamples = _width * _height;
  unsigned int depthBufferSize = depthSamples * sizeof(float);

  common::Image::PixelFormatType format =
    common::Image::ConvertPixelFormat(_format);

  if (!this->dataPtr->depthBuffer)
    this->dataPtr->depthBuffer = new float[depthSamples];

  memcpy(this->dataPtr->depthBuffer, _scan, depthBufferSize);

  // Save image
  if (this->dataPtr->saveImage)
  {
    this->dataPtr->SaveImage(_scan, _width, _height,
        format);
  }
}

/////////////////////////////////////////////////
void DepthCameraSensor::OnNewRgbPointCloud(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int _channels,
                    const std::string &/*_format*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  unsigned int pointCloudSamples = _width * _height;
  unsigned int pointCloudBufferSize = pointCloudSamples * _channels *
      sizeof(float);

  if (!this->dataPtr->pointCloudBuffer)
    this->dataPtr->pointCloudBuffer = new float[pointCloudSamples * _channels];

  memcpy(this->dataPtr->pointCloudBuffer, _scan, pointCloudBufferSize);
}

/////////////////////////////////////////////////
rendering::DepthCameraPtr DepthCameraSensor::DepthCamera() const
{
  return this->dataPtr->depthCamera;
}

/////////////////////////////////////////////////
common::ConnectionPtr DepthCameraSensor::ConnectImageCallback(
    std::function<void(const msgs::Image &)> _callback)
{
  return this->dataPtr->imageEvent.Connect(_callback);
}

/////////////////////////////////////////////////
void DepthCameraSensor::SetScene(rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    // TODO(anyone) Remove camera from scene
    this->dataPtr->depthCamera = nullptr;
    RenderingSensor::SetScene(_scene);

    if (this->dataPtr->initialized)
      this->CreateCamera();
  }
}

//////////////////////////////////////////////////
bool DepthCameraSensor::Update(
  const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("DepthCameraSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->depthCamera)
  {
    gzerr << "Camera doesn't exist.\n";
    return false;
  }

  if (this->HasInfoConnections())
  {
    // publish the camera info message
    this->PublishInfo(_now);
  }

  if (!this->HasDepthConnections() && !this->HasPointConnections())
  {
    return false;
  }

  if (this->HasPointConnections() && !this->dataPtr->pointCloudConnection)
  {
    this->dataPtr->pointCloudConnection =
        this->dataPtr->depthCamera->ConnectNewRgbPointCloud(
        std::bind(&DepthCameraSensor::OnNewRgbPointCloud, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  }
  else if (!this->HasPointConnections() && this->dataPtr->pointCloudConnection)
  {
    this->dataPtr->pointCloudConnection.reset();
  }

  // generate sensor data
  this->Render();

  unsigned int width = this->dataPtr->depthCamera->ImageWidth();
  unsigned int height = this->dataPtr->depthCamera->ImageHeight();

  auto msgsFormat = msgs::PixelFormatType::R_FLOAT32;

  // create message
  msgs::Image msg;
  msg.set_width(width);
  msg.set_height(height);
  msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
               rendering::PF_FLOAT32_R));
  msg.set_pixel_format_type(msgsFormat);
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);

  auto* frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->OpticalFrameId());

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  msg.set_data(this->dataPtr->depthBuffer,
      rendering::PixelUtil::MemorySize(rendering::PF_FLOAT32_R,
      width, height));

  this->AddSequence(msg.mutable_header(), "default");
  this->dataPtr->pub.Publish(msg);


  if (this->dataPtr->imageEvent.ConnectionCount() > 0u)
  {
    // Trigger callbacks.
    try
    {
      this->dataPtr->imageEvent(msg);
    }
    catch(...)
    {
      gzerr << "Exception thrown in an image callback.\n";
    }
  }

  if (this->HasPointConnections() &&
      this->dataPtr->pointCloudBuffer)
  {
    // Set the time stamp
    *this->dataPtr->pointMsg.mutable_header()->mutable_stamp() =
      msgs::Convert(_now);
    this->dataPtr->pointMsg.set_is_dense(true);

    if (!this->dataPtr->xyzBuffer)
      this->dataPtr->xyzBuffer = new float[width*height*3];

    if (this->dataPtr->image.Width() != width
        || this->dataPtr->image.Height() != height)
    {
      this->dataPtr->image =
          rendering::Image(width, height, rendering::PF_R8G8B8);
    }

    // extract image data from point cloud data
    this->dataPtr->pointsUtil.XYZFromPointCloud(
        this->dataPtr->xyzBuffer,
        this->dataPtr->pointCloudBuffer,
        width, height);

    // convert depth to grayscale rgb image
    this->dataPtr->ConvertDepthToImage(this->dataPtr->depthBuffer,
        this->dataPtr->image.Data<unsigned char>(), width, height);

    // fill the point cloud msg with data from xyz and rgb buffer
    this->dataPtr->pointsUtil.FillMsg(this->dataPtr->pointMsg,
        this->dataPtr->xyzBuffer,
        this->dataPtr->image.Data<unsigned char>());

    this->AddSequence(this->dataPtr->pointMsg.mutable_header(), "pointMsg");
    this->dataPtr->pointPub.Publish(this->dataPtr->pointMsg);
  }
  return true;
}

//////////////////////////////////////////////////
unsigned int DepthCameraSensor::ImageWidth() const
{
  return this->dataPtr->depthCamera->ImageWidth();
}

//////////////////////////////////////////////////
unsigned int DepthCameraSensor::ImageHeight() const
{
  return this->dataPtr->depthCamera->ImageHeight();
}

//////////////////////////////////////////////////
double DepthCameraSensor::FarClip() const
{
  return this->dataPtr->depthCamera->FarClipPlane();
}

//////////////////////////////////////////////////
double DepthCameraSensor::NearClip() const
{
  return this->dataPtr->near;
}

//////////////////////////////////////////////////
bool DepthCameraSensor::HasConnections() const
{
  return this->HasDepthConnections() || this->HasPointConnections() ||
      this->HasInfoConnections();
}

//////////////////////////////////////////////////
bool DepthCameraSensor::HasDepthConnections() const
{
  return (this->dataPtr->pub && this->dataPtr->pub.HasConnections())
         || this->dataPtr->imageEvent.ConnectionCount() > 0u;
}

//////////////////////////////////////////////////
bool DepthCameraSensor::HasPointConnections() const
{
  return this->dataPtr->pointPub && this->dataPtr->pointPub.HasConnections();
}
