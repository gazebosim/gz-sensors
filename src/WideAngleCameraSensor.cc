/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gz/msgs/camera_info.pb.h>

#include <mutex>

#include <gz/common/Console.hh>
#include <gz/common/Event.hh>
#include <gz/common/Image.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/StringUtils.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/WideAngleCameraSensor.hh"
#include "gz/sensors/ImageGaussianNoiseModel.hh"
#include "gz/sensors/ImageNoise.hh"
#include "gz/sensors/Manager.hh"
#include "gz/sensors/RenderingEvents.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/SensorTypes.hh"

using namespace gz;
using namespace sensors;

/// \brief Private data for WideAngleCameraSensor
class gz::sensors::WideAngleCameraSensorPrivate
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
  public: bool SaveImage(const unsigned char *_data, unsigned int _width,
    unsigned int _height, gz::common::Image::PixelFormatType _format);

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub;

  /// \brief WideAngleCamera info publisher to publish images
  public: transport::Node::Publisher infoPub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Rendering camera
  public: gz::rendering::WideAngleCameraPtr camera;

  /// \brief Image data buffer.
  public: unsigned char *imageBuffer = nullptr;

  /// \brief Pointer to an image to be published
  // public: gz::rendering::Image image;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: gz::common::EventT<
          void(const gz::msgs::Image &)> imageEvent;

  /// \brief Connection from wide angle camera with image data
  public: gz::common::ConnectionPtr imageConnection;

  /// \brief Connection to the Manager's scene change event.
  public: gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief True to save images
  public: bool saveImage = false;

  /// \brief path directory to where images are saved
  public: std::string saveImagePath = "";

  /// \prefix of an image name
  public: std::string saveImagePrefix = "";

  /// \brief counter used to set the image filename
  public: std::uint64_t saveImageCounter = 0;

  /// \brief SDF Sensor DOM object.
  public: sdf::Sensor sdfSensor;

  /// \brief Flag to indicate if sensor is generating data
  public: bool generatingData = false;
};

//////////////////////////////////////////////////
WideAngleCameraSensor::WideAngleCameraSensor()
  : dataPtr(new WideAngleCameraSensorPrivate())
{
}

//////////////////////////////////////////////////
WideAngleCameraSensor::~WideAngleCameraSensor()
{
  this->dataPtr->imageConnection.reset();
  if (this->dataPtr->imageBuffer)
  {
    delete [] this->dataPtr->imageBuffer;
    this->dataPtr->imageBuffer = nullptr;
  }
}

//////////////////////////////////////////////////
bool WideAngleCameraSensor::Init()
{
  return this->CameraSensor::Init();
}

//////////////////////////////////////////////////
bool WideAngleCameraSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}


//////////////////////////////////////////////////
bool WideAngleCameraSensor::Load(const sdf::Sensor &_sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!Sensor::Load(_sdf))
  {
    return false;
  }

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::WIDE_ANGLE_CAMERA)
  {
    gzerr << "Attempting to a load a Wide Angle Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    gzerr << "Attempting to a load a Wide Angle Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  // Create the image publisher
  this->dataPtr->pub =
      this->dataPtr->node.Advertise<gz::msgs::Image>(
          this->Topic());

  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  gzdbg << "Wide angle camera images for [" << this->Name()
         << "] advertised on [" << this->Topic() << "]" << std::endl;

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

  if (this->Scene())
  {
    this->CreateCamera();
  }

  this->dataPtr->sceneChangeConnection =
      RenderingEvents::ConnectSceneChangeCallback(
      std::bind(&WideAngleCameraSensor::SetScene, this, std::placeholders::_1));

  this->dataPtr->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool WideAngleCameraSensor::CreateCamera()
{
  const sdf::Camera *cameraSdf = this->dataPtr->sdfSensor.CameraSensor();
  if (!cameraSdf)
  {
    gzerr << "Unable to access camera SDF element.\n";
    return false;
  }

  this->PopulateInfo(cameraSdf);

  unsigned int width = cameraSdf->ImageWidth();
  unsigned int height = cameraSdf->ImageHeight();

  if (width == 0u || height == 0u)
  {
    gzerr << "Unable to create a wide angle camera sensor with 0 width or "
          << "height." << std::endl;
    return false;
  }

  this->dataPtr->camera = this->Scene()->CreateWideAngleCamera(this->Name());

  if (!this->dataPtr->camera)
  {
    gzerr << "Failed to create wide angle camera" << std::endl;
    return false;
  }
  this->dataPtr->camera->SetImageWidth(width);
  this->dataPtr->camera->SetImageHeight(height);
  this->dataPtr->camera->SetNearClipPlane(cameraSdf->NearClip());
  this->dataPtr->camera->SetFarClipPlane(cameraSdf->FarClip());
  this->dataPtr->camera->SetVisibilityMask(cameraSdf->VisibilityMask());
  this->dataPtr->camera->SetLocalPose(this->Pose());

  rendering::CameraLens lens;
  std::string lensType = cameraSdf->LensType();
  if (lensType  == "custom")
  {
    std::string funStr = cameraSdf->LensFunction();
    rendering::AngleFunctionType fun;
    if (funStr == "sin")
      fun = rendering::AFT_SIN;
    else if (funStr == "tan")
      fun = rendering::AFT_TAN;
    else
      fun = rendering::AFT_IDENTITY;

    lens.SetCustomMappingFunction(
        cameraSdf->LensC1(),
        cameraSdf->LensC2(),
        fun,
        cameraSdf->LensFocalLength(),
        cameraSdf->LensC3());
  }
  else if (lensType  == "equidistant")
  {
    lens.SetType(rendering::MFT_EQUIDISTANT);
  }
  else if (lensType  == "equisolid_angle")
  {
    lens.SetType(rendering::MFT_EQUISOLID_ANGLE);
  }
  else if (lensType  == "orthographic")
  {
    lens.SetType(rendering::MFT_ORTHOGRAPHIC);
  }
  else if (lensType  == "gnomonical" || lensType  == "gnomonic")
  {
    lens.SetType(rendering::MFT_GNOMONIC);
  }
  else if (lensType  == "stereographic")
  {
    lens.SetType(rendering::MFT_STEREOGRAPHIC);
  }

  this->dataPtr->camera->SetLens(lens);


  this->AddSensor(this->dataPtr->camera);

  const std::map<SensorNoiseType, sdf::Noise> noises = {
    {CAMERA_NOISE, cameraSdf->ImageNoise()},
  };

  for (const auto & [noiseType, noiseSdf] : noises)
  {
    // Add gaussian noise to camera sensor
    if (noiseSdf.Type() == sdf::NoiseType::GAUSSIAN)
    {
      this->dataPtr->noises[noiseType] =
        ImageNoiseFactory::NewNoiseModel(noiseSdf, "wide_angle_camera");

      std::dynamic_pointer_cast<ImageGaussianNoiseModel>(
           this->dataPtr->noises[noiseType])->SetCamera(
             this->dataPtr->camera);
    }
    else if (noiseSdf.Type() != sdf::NoiseType::NONE)
    {
      gzwarn << "The camera sensor only supports Gaussian noise. "
       << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
       << "] is not supported." << std::endl;
    }
  }

  // \todo(nkoeng) these parameters via sdf
  this->dataPtr->camera->SetAntiAliasing(2);

  math::Angle angle = cameraSdf->HorizontalFov();

  this->dataPtr->camera->SetAspectRatio(static_cast<double>(width)/height);
  this->dataPtr->camera->SetHFOV(angle);

  sdf::PixelFormatType pixelFormat = cameraSdf->PixelFormat();
  switch (pixelFormat)
  {
    case sdf::PixelFormatType::RGB_INT8:
      this->dataPtr->camera->SetImageFormat(gz::rendering::PF_R8G8B8);
      break;
    default:
      gzerr << "Unsupported pixel format ["
        << static_cast<int>(pixelFormat) << "]\n";
      break;
  }

  this->Scene()->RootVisual()->AddChild(this->dataPtr->camera);

  // Create the directory to store frames
  if (cameraSdf->SaveFrames())
  {
    this->dataPtr->saveImagePath = cameraSdf->SaveFramesPath();
    this->dataPtr->saveImagePrefix = this->Name() + "_";
    this->dataPtr->saveImage = true;
  }

  this->dataPtr->imageConnection =
      this->dataPtr->camera->ConnectNewWideAngleFrame(
      std::bind(&WideAngleCameraSensor::OnNewWideAngleFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  return true;
}

/////////////////////////////////////////////////
void WideAngleCameraSensor::OnNewWideAngleFrame(
    const unsigned char *_data,
    unsigned int _width, unsigned int _height,
    unsigned int _channels,
    const std::string &/*_format*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  unsigned int len = _width * _height * _channels;
  unsigned int bufferSize = len * sizeof(unsigned char);

  if (!this->dataPtr->imageBuffer)
    this->dataPtr->imageBuffer = new unsigned char[len];

  memcpy(this->dataPtr->imageBuffer, _data, bufferSize);
}

/////////////////////////////////////////////////
gz::common::ConnectionPtr WideAngleCameraSensor::ConnectImageCallback(
    std::function<void(const gz::msgs::Image &)> _callback)
{
  return this->dataPtr->imageEvent.Connect(_callback);
}

/////////////////////////////////////////////////
rendering::WideAngleCameraPtr WideAngleCameraSensor::WideAngleCamera() const
{
  return this->dataPtr->camera;
}

/////////////////////////////////////////////////
void WideAngleCameraSensor::SetScene(gz::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    // TODO(anyone) Remove camera from scene
    this->dataPtr->camera = nullptr;
    RenderingSensor::SetScene(_scene);
    if (this->dataPtr->initialized)
      this->CreateCamera();
  }
}

//////////////////////////////////////////////////
bool WideAngleCameraSensor::Update(
    const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("WideAngleCameraSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->camera)
  {
    gzerr << "WideAngleCamera doesn't exist.\n";
    return false;
  }

  // render only if necessary
  if (!this->dataPtr->pub.HasConnections() &&
      this->dataPtr->imageEvent.ConnectionCount() <= 0 &&
      !this->dataPtr->saveImage)
  {
    if (this->dataPtr->generatingData)
    {
      gzdbg << "Disabling camera sensor: '" << this->Name() << "' data "
             << "generation. " << std::endl;;
      this->dataPtr->generatingData = false;
    }

    return true;
  }
  else
  {
    if (!this->dataPtr->generatingData)
    {
      gzdbg << "Enabling camera sensor: '" << this->Name() << "' data "
             << "generation." << std::endl;;
      this->dataPtr->generatingData = true;
    }
  }

  // generate sensor data
  this->Render();

  if (!this->dataPtr->imageBuffer)
    return false;

  unsigned int width = this->dataPtr->camera->ImageWidth();
  unsigned int height = this->dataPtr->camera->ImageHeight();

  gz::common::Image::PixelFormatType
      format{common::Image::UNKNOWN_PIXEL_FORMAT};
  msgs::PixelFormatType msgsPixelFormat =
    msgs::PixelFormatType::UNKNOWN_PIXEL_FORMAT;


  auto renderingFormat = this->dataPtr->camera->ImageFormat();
  switch (renderingFormat)
  {
    case gz::rendering::PF_R8G8B8:
      format = gz::common::Image::RGB_INT8;
      msgsPixelFormat = msgs::PixelFormatType::RGB_INT8;
      break;
    default:
      gzerr << "Unsupported pixel format ["
        << this->dataPtr->camera->ImageFormat() << "]\n";
      break;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // create message
  gz::msgs::Image msg;
  {
    GZ_PROFILE("WideAngleCameraSensor::Update Message");
    msg.set_width(width);
    msg.set_height(height);
    msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
                 this->dataPtr->camera->ImageFormat()));
    msg.set_pixel_format_type(msgsPixelFormat);
    *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->Name());
    msg.set_data(this->dataPtr->imageBuffer,
        rendering::PixelUtil::MemorySize(renderingFormat,
        width, height));
  }

  // publish the image message
  {
    this->AddSequence(msg.mutable_header());
    GZ_PROFILE("WideAngleCameraSensor::Update Publish");
    this->dataPtr->pub.Publish(msg);

    // publish the camera info message
    this->PublishInfo(_now);
  }

  // Trigger callbacks.
  if (this->dataPtr->imageEvent.ConnectionCount() > 0)
  {
    try
    {
      this->dataPtr->imageEvent(msg);
    }
    catch(...)
    {
      gzerr << "Exception thrown in an image callback.\n";
    }
  }

  // Save image
  if (this->dataPtr->saveImage)
  {
    this->dataPtr->SaveImage(this->dataPtr->imageBuffer, width, height, format);
  }

  return true;
}

//////////////////////////////////////////////////
bool WideAngleCameraSensorPrivate::SaveImage(const unsigned char *_data,
    unsigned int _width, unsigned int _height,
    gz::common::Image::PixelFormatType _format)
{
  // Attempt to create the directory if it doesn't exist
  if (!gz::common::isDirectory(this->saveImagePath))
  {
    if (!gz::common::createDirectories(this->saveImagePath))
      return false;
  }

  std::string filename = this->saveImagePrefix +
                         std::to_string(this->saveImageCounter) + ".png";
  ++this->saveImageCounter;

  gz::common::Image localImage;
  localImage.SetFromData(_data, _width, _height, _format);

  localImage.SavePNG(
      gz::common::joinPaths(this->saveImagePath, filename));
  return true;
}

//////////////////////////////////////////////////
unsigned int WideAngleCameraSensor::ImageWidth() const
{
  if (this->dataPtr->camera)
    return this->dataPtr->camera->ImageWidth();
  return 0;
}

//////////////////////////////////////////////////
unsigned int WideAngleCameraSensor::ImageHeight() const
{
  if (this->dataPtr->camera)
    return this->dataPtr->camera->ImageHeight();
  return 0;
}

//////////////////////////////////////////////////
rendering::CameraPtr WideAngleCameraSensor::RenderingCamera() const
{
  return this->dataPtr->camera;
}

//////////////////////////////////////////////////
bool WideAngleCameraSensor::HasConnections() const
{
  return (this->dataPtr->pub && this->dataPtr->pub.HasConnections()) ||
      this->dataPtr->imageEvent.ConnectionCount() > 0u;
}
