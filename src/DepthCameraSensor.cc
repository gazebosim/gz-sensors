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

#include <ignition/math/Helpers.hh>

#include "ignition/sensors/DepthCameraSensor.hh"
#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/GaussianNoiseModel.hh"

// undefine near and far macros from windows.h
#ifdef _WIN32
  #undef near
  #undef far
#endif

/// \brief Private data for DepthCameraSensor
class ignition::sensors::DepthCameraSensorPrivate
{
  /// \brief Remove a camera from a scene
  public: void RemoveCamera(ignition::rendering::ScenePtr _scene);

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
    unsigned int _height, ignition::common::Image::PixelFormatType _format);

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub;

  /// \brief Camera info publisher to publish images
  public: transport::Node::Publisher infoPub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

    /// \brief Rendering camera
  public: ignition::rendering::DepthCameraPtr depthCamera;

  /// \brief Depth data buffer.
  public: float *depthBuffer = nullptr;

  /// \brief Near clip distance.
  public: float near = 0.0;

  /// \brief Pointer to an image to be published
  public: ignition::rendering::Image image;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: ignition::common::EventT<
          void(const ignition::msgs::Image &)> imageEvent;

  /// \brief Connection from depth camera with a new image
  public: ignition::common::ConnectionPtr connection;

  /// \brief Connection to the Manager's scene change event.
  public: ignition::common::ConnectionPtr sceneChangeConnection;

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

  /// \brief Camera information message.
  public: msgs::CameraInfo infoMsg;
};

using namespace ignition;
using namespace sensors;

//////////////////////////////////////////////////
void DepthCameraSensorPrivate::RemoveCamera(
    rendering::ScenePtr _scene)
{
  if (_scene)
  {
    // \todo(nkoenig) Remove camera from scene!
  }
  // this->depthCamera = nullptr;
}

//////////////////////////////////////////////////
bool DepthCameraSensorPrivate::SaveImage(const float *_data,
    unsigned int _width, unsigned int _height,
    ignition::common::Image::PixelFormatType /*_format*/)
{
  // Attempt to create the directory if it doesn't exist
  if (!ignition::common::isDirectory(this->saveImagePath))
  {
    if (!ignition::common::createDirectories(this->saveImagePath))
      return false;
  }

  if (_width == 0 || _height == 0)
    return false;

  ignition::common::Image localImage;

  unsigned int depthSamples = _width * _height;
  unsigned int depthBufferSize = depthSamples * 3;

  unsigned char * imgDepthBuffer = new unsigned char[depthBufferSize];

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
    unsigned char d = 255 - (_data[j] * factor);
    imgDepthBuffer[j * 3] = d;
    imgDepthBuffer[j * 3 + 1] = d;
    imgDepthBuffer[j * 3 + 2] = d;
  }

  std::string filename = this->saveImagePrefix +
                         std::to_string(this->saveImageCounter) + ".png";
  ++this->saveImageCounter;

  localImage.SetFromData(imgDepthBuffer, _width, _height,
      common::Image::RGB_INT8);
  localImage.SavePNG(
      ignition::common::joinPaths(this->saveImagePath, filename));

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
  this->dataPtr->connection.reset();
  if (this->dataPtr->depthBuffer)
    delete [] this->dataPtr->depthBuffer;
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
    ignerr << "Attempting to a load a Depth Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    ignerr << "Attempting to a load a Depth Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::Image>(
          this->Topic());
  if (!this->dataPtr->pub)
    return false;

  this->dataPtr->infoPub =
      this->dataPtr->node.Advertise<ignition::msgs::CameraInfo>(
          this->Topic() + "/camera_info");
  if (!this->dataPtr->infoPub)
    return false;

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
  const sdf::Camera *cameraSdf = this->dataPtr->sdfSensor.CameraSensor();

  if (!cameraSdf)
  {
    ignerr << "Unable to access camera SDF element\n";
    return false;
  }

  int width = cameraSdf->ImageWidth();
  int height = cameraSdf->ImageHeight();

  double far = cameraSdf->FarClip();
  double near = cameraSdf->NearClip();

  // Set some values of the camera info message.
  {
    msgs::CameraInfo::Distortion *distortion =
      this->dataPtr->infoMsg.mutable_distortion();

    distortion->set_distortion_model(msgs::CameraInfo::Distortion::PLUMB_BOB);
    distortion->set_k1(cameraSdf->DistortionK1());
    distortion->set_k2(cameraSdf->DistortionK2());
    distortion->set_k3(cameraSdf->DistortionK3());
    distortion->set_t1(cameraSdf->DistortionP1());
    distortion->set_t2(cameraSdf->DistortionP2());

    msgs::CameraInfo::Intrinsics *intrinsics =
      this->dataPtr->infoMsg.mutable_intrinsics();
    intrinsics->set_fx(cameraSdf->LensIntrinsicsFx());
    intrinsics->set_fy(cameraSdf->LensIntrinsicsFy());
    intrinsics->set_cx(cameraSdf->LensIntrinsicsCx());
    intrinsics->set_cy(cameraSdf->LensIntrinsicsCy());

    msgs::CameraInfo::Projection *proj =
      this->dataPtr->infoMsg.mutable_projection();
    proj->set_fx(cameraSdf->LensIntrinsicsFx());
    proj->set_fy(cameraSdf->LensIntrinsicsFy());
    proj->set_cx(cameraSdf->LensIntrinsicsCx());
    proj->set_cy(cameraSdf->LensIntrinsicsCy());

    // Set the rectifcation matrix to identity
    this->dataPtr->infoMsg.add_rectification_matrix(1.0);
    this->dataPtr->infoMsg.add_rectification_matrix(0.0);
    this->dataPtr->infoMsg.add_rectification_matrix(0.0);

    this->dataPtr->infoMsg.add_rectification_matrix(0.0);
    this->dataPtr->infoMsg.add_rectification_matrix(1.0);
    this->dataPtr->infoMsg.add_rectification_matrix(0.0);

    this->dataPtr->infoMsg.add_rectification_matrix(0.0);
    this->dataPtr->infoMsg.add_rectification_matrix(0.0);
    this->dataPtr->infoMsg.add_rectification_matrix(1.0);

    auto infoFrame = this->dataPtr->infoMsg.mutable_header()->add_data();
    infoFrame->set_key("frame_id");
    infoFrame->add_value(this->Name());

    this->dataPtr->infoMsg.set_width(width);
    this->dataPtr->infoMsg.set_height(height);
  }

  this->dataPtr->depthCamera = this->Scene()->CreateDepthCamera(
      this->Name());
  this->dataPtr->depthCamera->SetImageWidth(width);
  this->dataPtr->depthCamera->SetImageHeight(height);
  this->dataPtr->depthCamera->SetFarClipPlane(far);

  const std::map<SensorNoiseType, sdf::Noise> noises = {
    {CAMERA_NOISE, cameraSdf->ImageNoise()},
  };

  for (const auto & [noiseType, noiseSdf] : noises)
  {
    // Add gaussian noise to camera sensor
    if (noiseSdf.Type() == sdf::NoiseType::GAUSSIAN)
    {
      this->dataPtr->noises[noiseType] =
        NoiseFactory::NewNoiseModel(noiseSdf, "depth");

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

  // Near clip plane not set because we need to be able to detect occlusion
  // from objects before near clip plane
  this->dataPtr->near = near;

  // \todo(nkoeng) these parameters via sdf
  this->dataPtr->depthCamera->SetAntiAliasing(2);

  math::Angle angle = cameraSdf->HorizontalFov();
  if (angle < 0.01 || angle > IGN_PI*2)
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
  // This->dataPtr->distortion->Load(this->sdf->GetElement("distortion"));

  sdf::PixelFormatType pixelFormat = cameraSdf->PixelFormat();
  switch (pixelFormat)
  {
    case sdf::PixelFormatType::R_FLOAT32:
      this->dataPtr->depthCamera->SetImageFormat(
          ignition::rendering::PF_FLOAT32_R);
      break;
    default:
      ignerr << "Unsupported pixel format ["
        << static_cast<int>(pixelFormat) << "]\n";
      break;
  }

  this->dataPtr->image = this->dataPtr->depthCamera->CreateImage();

  this->Scene()->RootVisual()->AddChild(this->dataPtr->depthCamera);

  // Create the directory to store frames
  if (cameraSdf->SaveFrames())
  {
    this->dataPtr->saveImagePath = cameraSdf->SaveFramesPath();
    this->dataPtr->saveImagePrefix = this->Name() + "_";
    this->dataPtr->saveImage = true;
  }

  this->dataPtr->connection = this->dataPtr->depthCamera->ConnectNewDepthFrame(
      std::bind(&DepthCameraSensor::OnNewDepthFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  return true;
}

/////////////////////////////////////////////////
void DepthCameraSensor::OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &_format)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  float near = this->NearClip();
  float far = this->FarClip();
  unsigned int depthSamples = _width * _height;
  unsigned int depthBufferSize = depthSamples * sizeof(float);

  ignition::common::Image::PixelFormatType format =
    ignition::common::Image::ConvertPixelFormat(_format);

  if (!this->dataPtr->depthBuffer)
    this->dataPtr->depthBuffer = new float[depthSamples];

  memcpy(this->dataPtr->depthBuffer, _scan, depthBufferSize);

  for (unsigned int i = 0; i < depthSamples; ++i)
  {
    // Mask ranges outside of min/max to +/- inf, as per REP 117
    if (this->dataPtr->depthBuffer[i] >= far)
    {
      this->dataPtr->depthBuffer[i] = ignition::math::INF_D;
    }
    else if (this->dataPtr->depthBuffer[i] <= near)
    {
      this->dataPtr->depthBuffer[i] = -ignition::math::INF_D;
    }
  }

  // Save image
  if (this->dataPtr->saveImage)
  {
    this->dataPtr->SaveImage(_scan, _width, _height,
        format);
  }
}

/////////////////////////////////////////////////
ignition::rendering::DepthCameraPtr DepthCameraSensor::DepthCamera()
{
  return this->dataPtr->depthCamera;
}

/////////////////////////////////////////////////
ignition::common::ConnectionPtr DepthCameraSensor::ConnectImageCallback(
    std::function<void(const ignition::msgs::Image &)> _callback)
{
  return this->dataPtr->imageEvent.Connect(_callback);
}

/////////////////////////////////////////////////
void DepthCameraSensor::SetScene(ignition::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    this->dataPtr->RemoveCamera(this->Scene());
    RenderingSensor::SetScene(_scene);

    if (this->dataPtr->initialized)
      this->CreateCamera();
  }
}

//////////////////////////////////////////////////
bool DepthCameraSensor::Update(const ignition::common::Time &_now)
{
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->depthCamera)
  {
    ignerr << "Camera doesn't exist.\n";
    return false;
  }

  // generate sensor data
  this->dataPtr->depthCamera->Update();

  unsigned int width = this->dataPtr->depthCamera->ImageWidth();
  unsigned int height = this->dataPtr->depthCamera->ImageHeight();

  auto commonFormat = common::Image::R_FLOAT32;
  auto msgsFormat = msgs::PixelFormatType::R_FLOAT32;

  // create message
  ignition::msgs::Image msg;
  msg.set_width(width);
  msg.set_height(height);
  msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
               this->dataPtr->depthCamera->ImageFormat()));
  // TODO(anyone) Deprecated in ign-msgs4, will be removed on ign-msgs5
  // in favor of set_pixel_format_type.
  msg.set_pixel_format(commonFormat);
  msg.set_pixel_format_type(msgsFormat);
  msg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
  msg.mutable_header()->mutable_stamp()->set_nsec(_now.nsec);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  msg.set_data(this->dataPtr->depthBuffer,
      this->dataPtr->depthCamera->ImageMemorySize());

  // publish
  this->dataPtr->pub.Publish(msg);

  // publish the camera info message
  this->dataPtr->infoMsg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
  this->dataPtr->infoMsg.mutable_header()->mutable_stamp()->set_nsec(
      _now.nsec);
  this->dataPtr->infoPub.Publish(this->dataPtr->infoMsg);

  // Trigger callbacks.
  try
  {
    this->dataPtr->imageEvent(msg);
  }
  catch(...)
  {
    ignerr << "Exception thrown in an image callback.\n";
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

IGN_SENSORS_REGISTER_SENSOR(DepthCameraSensor)
