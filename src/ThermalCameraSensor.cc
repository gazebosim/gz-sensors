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
#include <gz/msgs/image.pb.h>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <algorithm>
#include <mutex>

#include <gz/common/Console.hh>
#include <gz/common/Image.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/SystemPaths.hh>

#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>

#include <gz/transport/Node.hh>

#include "gz/sensors/ThermalCameraSensor.hh"
#include "gz/sensors/ImageGaussianNoiseModel.hh"
#include "gz/sensors/RenderingEvents.hh"
#include "gz/sensors/SensorFactory.hh"

/// \brief Private data for ThermalCameraSensor
class gz::sensors::ThermalCameraSensorPrivate
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
  public: bool SaveImage(const uint16_t *_data, unsigned int _width,
    unsigned int _height, gz::common::Image::PixelFormatType _format);

  /// \brief Helper function to convert temperature data to thermal image
  /// \param[in] _data temperature data
  /// \param[out] _imageBuffer resulting thermal image data
  /// \param[in] _width width of image
  /// \param[in] _height height of image
  public: bool ConvertTemperatureToImage(const uint16_t *_data,
    unsigned char *_imageBuffer, unsigned int _width, unsigned int _height);

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Rendering camera
  public: gz::rendering::ThermalCameraPtr thermalCamera;

  /// \brief Thermal data buffer.
  public: uint16_t *thermalBuffer = nullptr;

  /// \brief Thermal data buffer used when saving image.
  public: unsigned char *imgThermalBuffer = nullptr;

  /// \brief Thermal data buffer used when saving image.
  public: math::Vector2i imgThermalBufferSize =
      math::Vector2i::Zero;

  /// \brief Pointer to an image to be published
  public: gz::rendering::Image image;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: gz::common::EventT<
          void(const gz::msgs::Image &)> imageEvent;

  /// \brief Connection from thermal camera with thermal data
  public: gz::common::ConnectionPtr thermalConnection;

  /// \brief Connection to the Manager's scene change event.
  public: gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief True to save images
  public: bool saveImage = false;

  /// \brief path directory to where images are saved
  public: std::string saveImagePath = "./";

  /// \brief Prefix of an image name
  public: std::string saveImagePrefix = "./";

  /// \brief counter used to set the image filename
  public: std::uint64_t saveImageCounter = 0;

  /// \brief SDF Sensor DOM object.
  public: sdf::Sensor sdfSensor;

  /// \brief The point cloud message.
  public: msgs::Image thermalMsg;

  /// \brief publisher to publish thermal image
  public: transport::Node::Publisher thermalPub;

  /// \brief Ambient temperature of the environment
  public: float ambient = 0.0;

  /// \brief Range of ambient temperature.
  public: float ambientRange = 0.0;

  /// \brief Min temperature the sensor can detect
  public: float minTemp = -gz::math::INF_F;

  /// \brief Max temperature the sensor can detect
  public: float maxTemp = gz::math::INF_F;

  /// \brief Linear resolution. Defaults to 10mK
  public: float resolution = 0.01f;
};

using namespace gz;
using namespace sensors;

//////////////////////////////////////////////////
ThermalCameraSensor::ThermalCameraSensor()
  : CameraSensor(), dataPtr(new ThermalCameraSensorPrivate())
{
}

//////////////////////////////////////////////////
ThermalCameraSensor::~ThermalCameraSensor()
{
  this->dataPtr->thermalConnection.reset();
  if (this->dataPtr->thermalBuffer)
    delete [] this->dataPtr->thermalBuffer;

  if (this->dataPtr->imgThermalBuffer)
    delete[] this->dataPtr->imgThermalBuffer;
}

//////////////////////////////////////////////////
bool ThermalCameraSensor::Init()
{
  return this->CameraSensor::Init();
}

//////////////////////////////////////////////////
bool ThermalCameraSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool ThermalCameraSensor::Load(const sdf::Sensor &_sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!Sensor::Load(_sdf))
  {
    return false;
  }

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::THERMAL_CAMERA)
  {
    ignerr << "Attempting to a load a Thermal Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    ignerr << "Attempting to a load a Thermal Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  // Create the thermal image publisher
  this->dataPtr->thermalPub =
      this->dataPtr->node.Advertise<msgs::Image>(
          this->Topic());

  if (!this->dataPtr->thermalPub)
  {
    ignerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  igndbg << "Thermal images for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  if (!this->AdvertiseInfo())
    return false;

  if (this->Scene())
  {
    this->CreateCamera();
  }

  this->dataPtr->sceneChangeConnection =
      RenderingEvents::ConnectSceneChangeCallback(
      std::bind(&ThermalCameraSensor::SetScene, this, std::placeholders::_1));

  this->dataPtr->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool ThermalCameraSensor::CreateCamera()
{
  const sdf::Camera *cameraSdf = this->dataPtr->sdfSensor.CameraSensor();

  if (!cameraSdf)
  {
    ignerr << "Unable to access camera SDF element\n";
    return false;
  }

  int width = cameraSdf->ImageWidth();
  int height = cameraSdf->ImageHeight();

  double farPlane = cameraSdf->FarClip();
  double nearPlane = cameraSdf->NearClip();

  this->PopulateInfo(cameraSdf);

  this->dataPtr->thermalCamera = this->Scene()->CreateThermalCamera(
      this->Name());
  this->dataPtr->thermalCamera->SetImageWidth(width);
  this->dataPtr->thermalCamera->SetImageHeight(height);
  this->dataPtr->thermalCamera->SetNearClipPlane(nearPlane);
  this->dataPtr->thermalCamera->SetFarClipPlane(farPlane);

  this->dataPtr->thermalCamera->SetAmbientTemperature(this->dataPtr->ambient);
  this->dataPtr->thermalCamera->SetMinTemperature(this->dataPtr->minTemp);
  this->dataPtr->thermalCamera->SetMaxTemperature(this->dataPtr->maxTemp);
  this->dataPtr->thermalCamera->SetLinearResolution(this->dataPtr->resolution);

  this->AddSensor(this->dataPtr->thermalCamera);

  const std::map<SensorNoiseType, sdf::Noise> noises = {
    {CAMERA_NOISE, cameraSdf->ImageNoise()},
  };

  for (const auto & [noiseType, noiseSdf] : noises)
  {
    // Add gaussian noise to camera sensor
    if (noiseSdf.Type() == sdf::NoiseType::GAUSSIAN)
    {
      this->dataPtr->noises[noiseType] =
        NoiseFactory::NewNoiseModel(noiseSdf, "camera");

      std::dynamic_pointer_cast<ImageGaussianNoiseModel>(
           this->dataPtr->noises[noiseType])->SetCamera(
             this->dataPtr->thermalCamera);
    }
    else if (noiseSdf.Type() != sdf::NoiseType::NONE)
    {
      ignwarn << "The thermal camera sensor only supports Gaussian noise. "
       << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
       << "] is not supported." << std::endl;
    }
  }

  math::Angle angle = cameraSdf->HorizontalFov();
  if (angle < 0.01 || angle > IGN_PI*2)
  {
    ignerr << "Invalid horizontal field of view [" << angle << "]\n";

    return false;
  }
  this->dataPtr->thermalCamera->SetAspectRatio(
      static_cast<double>(width)/height);
  this->dataPtr->thermalCamera->SetHFOV(angle);

  // \todo(nkoenig) Port Distortion class
  // This->dataPtr->distortion.reset(new Distortion());
  // This->dataPtr->distortion->Load(this->sdf->GetElement("distortion"));

  this->Scene()->RootVisual()->AddChild(this->dataPtr->thermalCamera);

  // Create the directory to store frames
  if (cameraSdf->SaveFrames())
  {
    this->dataPtr->saveImagePath = cameraSdf->SaveFramesPath();
    this->dataPtr->saveImagePrefix = this->Name() + "_";
    this->dataPtr->saveImage = true;
  }

  this->dataPtr->thermalConnection =
      this->dataPtr->thermalCamera->ConnectNewThermalFrame(
      std::bind(&ThermalCameraSensor::OnNewThermalFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  return true;
}

/////////////////////////////////////////////////
void ThermalCameraSensor::OnNewThermalFrame(const uint16_t *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &/*_format*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  unsigned int samples = _width * _height;
  unsigned int thermalBufferSize = samples * sizeof(uint16_t);

  if (!this->dataPtr->thermalBuffer)
    this->dataPtr->thermalBuffer = new uint16_t[samples];

  memcpy(this->dataPtr->thermalBuffer, _scan, thermalBufferSize);
}

/////////////////////////////////////////////////
rendering::ThermalCameraPtr ThermalCameraSensor::ThermalCamera()
{
  return this->dataPtr->thermalCamera;
}

/////////////////////////////////////////////////
common::ConnectionPtr ThermalCameraSensor::ConnectImageCallback(
    std::function<void(const msgs::Image &)> _callback)
{
  return this->dataPtr->imageEvent.Connect(_callback);
}

/////////////////////////////////////////////////
void ThermalCameraSensor::SetScene(rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    // TODO(anyone) Remove camera from scene
    this->dataPtr->thermalCamera = nullptr;
    RenderingSensor::SetScene(_scene);

    if (this->dataPtr->initialized)
      this->CreateCamera();
  }
}

//////////////////////////////////////////////////
bool ThermalCameraSensor::Update(const common::Time &_now)
{
  IGN_PROFILE("ThermalCameraSensor::Update");
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->thermalCamera)
  {
    ignerr << "Camera doesn't exist.\n";
    return false;
  }

  if (!this->dataPtr->thermalPub.HasConnections() &&
      this->dataPtr->imageEvent.ConnectionCount() == 0u)
    return false;

  // generate sensor data - this triggers image callback
  this->Render();

  if (!this->dataPtr->thermalBuffer)
    return false;

  unsigned int width = this->dataPtr->thermalCamera->ImageWidth();
  unsigned int height = this->dataPtr->thermalCamera->ImageHeight();

  auto commonFormat = common::Image::L_INT16;
  auto msgsFormat = msgs::PixelFormatType::L_INT16;

  // create message
  this->dataPtr->thermalMsg.set_width(width);
  this->dataPtr->thermalMsg.set_height(height);
  this->dataPtr->thermalMsg.set_step(
      width * rendering::PixelUtil::BytesPerPixel(rendering::PF_L16));
  this->dataPtr->thermalMsg.set_pixel_format_type(msgsFormat);
  auto stamp = this->dataPtr->thermalMsg.mutable_header()->mutable_stamp();
  stamp->set_sec(_now.sec);
  stamp->set_nsec(_now.nsec);
  auto frame = this->dataPtr->thermalMsg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->FrameId());

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->thermalMsg.set_data(this->dataPtr->thermalBuffer,
      rendering::PixelUtil::MemorySize(rendering::PF_L16,
      width, height));

  // publish the camera info message
  this->PublishInfo(_now);

  this->dataPtr->thermalPub.Publish(this->dataPtr->thermalMsg);

  // Trigger callbacks.
  try
  {
    this->dataPtr->imageEvent(this->dataPtr->thermalMsg);
  }
  catch(...)
  {
    ignerr << "Exception thrown in an image callback.\n";
  }

  // Save image
  if (this->dataPtr->saveImage)
  {
    this->dataPtr->SaveImage(this->dataPtr->thermalBuffer, width, height,
        commonFormat);
  }

  return true;
}

//////////////////////////////////////////////////
unsigned int ThermalCameraSensor::ImageWidth() const
{
  if (!this->dataPtr->thermalCamera)
    return 0u;
  return this->dataPtr->thermalCamera->ImageWidth();
}

//////////////////////////////////////////////////
unsigned int ThermalCameraSensor::ImageHeight() const
{
  if (!this->dataPtr->thermalCamera)
    return 0u;
  return this->dataPtr->thermalCamera->ImageHeight();
}

//////////////////////////////////////////////////
void ThermalCameraSensor::SetAmbientTemperature(float _ambient)
{
  this->dataPtr->ambient = _ambient;
  if (this->dataPtr->thermalCamera)
  {
    this->dataPtr->thermalCamera->SetAmbientTemperature(this->dataPtr->ambient);
  }
}

//////////////////////////////////////////////////
void ThermalCameraSensor::SetAmbientTemperatureRange(float _range)
{
  this->dataPtr->ambientRange = _range;
  if (this->dataPtr->thermalCamera)
  {
    this->dataPtr->thermalCamera->SetAmbientTemperatureRange(
        this->dataPtr->ambientRange);
  }
}

//////////////////////////////////////////////////
void ThermalCameraSensor::SetMinTemperature(float _min)
{
  this->dataPtr->minTemp = _min;
  if (this->dataPtr->thermalCamera)
  {
    this->dataPtr->thermalCamera->SetMinTemperature(this->dataPtr->minTemp);
  }
}

//////////////////////////////////////////////////
void ThermalCameraSensor::SetMaxTemperature(float _max)
{
  this->dataPtr->maxTemp = _max;
  if (this->dataPtr->thermalCamera)
  {
    this->dataPtr->thermalCamera->SetMaxTemperature(this->dataPtr->maxTemp);
  }
}

//////////////////////////////////////////////////
void ThermalCameraSensor::SetLinearResolution(float _resolution)
{
  this->dataPtr->resolution = _resolution;
  if (this->dataPtr->thermalCamera)
  {
    this->dataPtr->thermalCamera->SetLinearResolution(
        this->dataPtr->resolution);
  }
}

//////////////////////////////////////////////////
bool ThermalCameraSensorPrivate::ConvertTemperatureToImage(
    const uint16_t *_data,
    unsigned char *_imageBuffer,
    unsigned int _width, unsigned int _height)
{
  // get min and max of temperature values
  const auto [min, max] = // NOLINT
      std::minmax_element(&_data[0], &_data[_height * _width]);

  // convert temperature to grayscale image
  double range = static_cast<double>(max - min);
  if (math::equal(range, 0.0))
    range = 1.0;
  for (unsigned int i = 0; i < _height; ++i)
  {
    for (unsigned int j = 0; j < _width; ++j)
    {
      uint16_t temp = _data[i*_width + j];
      double t = static_cast<double>(temp-*min) / range;
      int r = static_cast<int>(255*t);
      int g = r;
      int b = r;
      int index = i*_width*3 + j*3;
      _imageBuffer[index] = r;
      _imageBuffer[index+1] = g;
      _imageBuffer[index+2] = b;
    }
  }

  return true;
}

//////////////////////////////////////////////////
bool ThermalCameraSensorPrivate::SaveImage(const uint16_t *_data,
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

  if (static_cast<int>(_width) != this->imgThermalBufferSize.X() ||
      static_cast<int>(_height) != this->imgThermalBufferSize.Y())
  {
    delete [] this->imgThermalBuffer;
    unsigned int bufferSize = _width * _height * 3;
    this->imgThermalBuffer = new unsigned char[bufferSize];
    this->imgThermalBufferSize = math::Vector2i(_width, _height);
  }

  this->ConvertTemperatureToImage(_data, this->imgThermalBuffer,
      _width, _height);

  std::string filename = this->saveImagePrefix +
                         std::to_string(this->saveImageCounter) + ".png";
  ++this->saveImageCounter;

  localImage.SetFromData(this->imgThermalBuffer, _width, _height,
      common::Image::RGB_INT8);
  localImage.SavePNG(
      common::joinPaths(this->saveImagePath, filename));

  return true;
}

IGN_SENSORS_REGISTER_SENSOR(ThermalCameraSensor)
