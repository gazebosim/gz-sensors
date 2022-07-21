/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <ignition/msgs/camera_info.pb.h>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <mutex>

#include <ignition/common/Console.hh>
#include <ignition/common/Event.hh>
#include <ignition/common/Image.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include "ignition/sensors/CameraSensor.hh"
#include "ignition/sensors/ImageGaussianNoiseModel.hh"
#include "ignition/sensors/ImageNoise.hh"
#include "ignition/sensors/Manager.hh"
#include "ignition/sensors/RenderingEvents.hh"
#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/SensorTypes.hh"

using namespace ignition;
using namespace sensors;

/// \brief Private data for CameraSensor
class ignition::sensors::CameraSensorPrivate
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
  public: ignition::rendering::CameraPtr camera;

  /// \brief Pointer to an image to be published
  public: ignition::rendering::Image image;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: ignition::common::EventT<
          void(const ignition::msgs::Image &)> imageEvent;

  /// \brief Connection to the Manager's scene change event.
  public: ignition::common::ConnectionPtr sceneChangeConnection;

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

  /// \brief Camera information message.
  public: msgs::CameraInfo infoMsg;

  /// \brief Topic for info message.
  public: std::string infoTopic{""};

  /// \brief Baseline for stereo cameras.
  public: double baseline{0.0};

  /// \brief Flag to indicate if sensor is generating data
  public: bool generatingData = false;
};

//////////////////////////////////////////////////
bool CameraSensor::CreateCamera()
{
  const sdf::Camera *cameraSdf = this->dataPtr->sdfSensor.CameraSensor();
  if (!cameraSdf)
  {
    ignerr << "Unable to access camera SDF element.\n";
    return false;
  }

  this->PopulateInfo(cameraSdf);

  unsigned int width = cameraSdf->ImageWidth();
  unsigned int height = cameraSdf->ImageHeight();

  this->dataPtr->camera = this->Scene()->CreateCamera(this->Name());
  this->dataPtr->camera->SetImageWidth(width);
  this->dataPtr->camera->SetImageHeight(height);
  this->dataPtr->camera->SetNearClipPlane(cameraSdf->NearClip());
  this->dataPtr->camera->SetFarClipPlane(cameraSdf->FarClip());
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
        ImageNoiseFactory::NewNoiseModel(noiseSdf, "camera");

      std::dynamic_pointer_cast<ImageGaussianNoiseModel>(
           this->dataPtr->noises[noiseType])->SetCamera(
             this->dataPtr->camera);
    }
    else if (noiseSdf.Type() != sdf::NoiseType::NONE)
    {
      ignwarn << "The camera sensor only supports Gaussian noise. "
       << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
       << "] is not supported." << std::endl;
    }
  }

  // \todo(nkoeng) these parameters via sdf
  this->dataPtr->camera->SetAntiAliasing(2);

  math::Angle angle = cameraSdf->HorizontalFov();
  if (angle < 0.01 || angle > IGN_PI*2)
  {
    ignerr << "Invalid horizontal field of view [" << angle << "]\n";

    return false;
  }
  this->dataPtr->camera->SetAspectRatio(static_cast<double>(width)/height);
  this->dataPtr->camera->SetHFOV(angle);

  // \todo(nkoenig) Port Distortion class
  // This->dataPtr->distortion.reset(new Distortion());
  // This->dataPtr->distortion->Load(this->sdf->GetElement("distortion"));

  sdf::PixelFormatType pixelFormat = cameraSdf->PixelFormat();
  switch (pixelFormat)
  {
    case sdf::PixelFormatType::RGB_INT8:
      this->dataPtr->camera->SetImageFormat(ignition::rendering::PF_R8G8B8);
      break;
    default:
      ignerr << "Unsupported pixel format ["
        << static_cast<int>(pixelFormat) << "]\n";
      break;
  }

  this->dataPtr->image = this->dataPtr->camera->CreateImage();

  this->Scene()->RootVisual()->AddChild(this->dataPtr->camera);

  // Create the directory to store frames
  if (cameraSdf->SaveFrames())
  {
    this->dataPtr->saveImagePath = cameraSdf->SaveFramesPath();
    this->dataPtr->saveImagePrefix = this->Name() + "_";
    this->dataPtr->saveImage = true;
  }

  return true;
}

//////////////////////////////////////////////////
CameraSensor::CameraSensor()
  : dataPtr(new CameraSensorPrivate())
{
}

//////////////////////////////////////////////////
CameraSensor::~CameraSensor()
{
}

//////////////////////////////////////////////////
bool CameraSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool CameraSensor::Load(const sdf::Sensor &_sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!Sensor::Load(_sdf))
  {
    return false;
  }

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::CAMERA)
  {
    ignerr << "Attempting to a load a Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    ignerr << "Attempting to a load a Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  if (this->Topic().empty())
    this->SetTopic("/camera");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::Image>(
          this->Topic());
  if (!this->dataPtr->pub)
  {
    ignerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  igndbg << "Camera images for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  if (!this->AdvertiseInfo())
    return false;

  if (this->Scene())
    this->CreateCamera();

  this->dataPtr->sceneChangeConnection =
      RenderingEvents::ConnectSceneChangeCallback(
      std::bind(&CameraSensor::SetScene, this, std::placeholders::_1));

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool CameraSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

/////////////////////////////////////////////////
ignition::common::ConnectionPtr CameraSensor::ConnectImageCallback(
    std::function<void(const ignition::msgs::Image &)> _callback)
{
  return this->dataPtr->imageEvent.Connect(_callback);
}

/////////////////////////////////////////////////
void CameraSensor::SetScene(ignition::rendering::ScenePtr _scene)
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
bool CameraSensor::Update(const ignition::common::Time &_now)
{
  IGN_PROFILE("CameraSensor::Update");
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->camera)
  {
    ignerr << "Camera doesn't exist.\n";
    return false;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // move the camera to the current pose
  this->dataPtr->camera->SetLocalPose(this->Pose());

  // render only if necessary
  if (!this->dataPtr->pub.HasConnections() &&
      this->dataPtr->imageEvent.ConnectionCount() <= 0 &&
      !this->dataPtr->saveImage)
  {
    if (this->dataPtr->generatingData)
    {
      igndbg << "Disabling camera sensor: '" << this->Name() << "' data "
             << "generation. " << std::endl;;
      this->dataPtr->generatingData = false;
    }

    return true;
  }
  else
  {
    if (!this->dataPtr->generatingData)
    {
      igndbg << "Enabling camera sensor: '" << this->Name() << "' data "
             << "generation." << std::endl;;
      this->dataPtr->generatingData = true;
    }
  }

  // generate sensor data
  this->Render();
  {
    IGN_PROFILE("CameraSensor::Update Copy image");
    this->dataPtr->camera->Copy(this->dataPtr->image);
  }

  unsigned int width = this->dataPtr->camera->ImageWidth();
  unsigned int height = this->dataPtr->camera->ImageHeight();
  unsigned char *data = this->dataPtr->image.Data<unsigned char>();

  ignition::common::Image::PixelFormatType
      format{common::Image::UNKNOWN_PIXEL_FORMAT};
  msgs::PixelFormatType msgsPixelFormat =
    msgs::PixelFormatType::UNKNOWN_PIXEL_FORMAT;

  switch (this->dataPtr->camera->ImageFormat())
  {
    case ignition::rendering::PF_R8G8B8:
      format = ignition::common::Image::RGB_INT8;
      msgsPixelFormat = msgs::PixelFormatType::RGB_INT8;
      break;
    default:
      ignerr << "Unsupported pixel format ["
        << this->dataPtr->camera->ImageFormat() << "]\n";
      break;
  }

  // create message
  ignition::msgs::Image msg;
  {
    IGN_PROFILE("CameraSensor::Update Message");
    msg.set_width(width);
    msg.set_height(height);
    msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
                 this->dataPtr->camera->ImageFormat()));
    msg.set_pixel_format_type(msgsPixelFormat);
    msg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
    msg.mutable_header()->mutable_stamp()->set_nsec(_now.nsec);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->FrameId());
    msg.set_data(data, this->dataPtr->camera->ImageMemorySize());
  }

  // publish the image message
  {
    this->AddSequence(msg.mutable_header());
    IGN_PROFILE("CameraSensor::Update Publish");
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
      ignerr << "Exception thrown in an image callback.\n";
    }
  }

  // Save image
  if (this->dataPtr->saveImage)
  {
    this->dataPtr->SaveImage(data, width, height, format);
  }

  return true;
}

//////////////////////////////////////////////////
bool CameraSensorPrivate::SaveImage(const unsigned char *_data,
    unsigned int _width, unsigned int _height,
    ignition::common::Image::PixelFormatType _format)
{
  // Attempt to create the directory if it doesn't exist
  if (!ignition::common::isDirectory(this->saveImagePath))
  {
    if (!ignition::common::createDirectories(this->saveImagePath))
      return false;
  }

  std::string filename = this->saveImagePrefix +
                         std::to_string(this->saveImageCounter) + ".png";
  ++this->saveImageCounter;

  ignition::common::Image localImage;
  localImage.SetFromData(_data, _width, _height, _format);

  localImage.SavePNG(
      ignition::common::joinPaths(this->saveImagePath, filename));
  return true;
}

//////////////////////////////////////////////////
unsigned int CameraSensor::ImageWidth() const
{
  if (this->dataPtr->camera)
    return this->dataPtr->camera->ImageWidth();
  return 0;
}

//////////////////////////////////////////////////
unsigned int CameraSensor::ImageHeight() const
{
  if (this->dataPtr->camera)
    return this->dataPtr->camera->ImageHeight();
  return 0;
}

//////////////////////////////////////////////////
rendering::CameraPtr CameraSensor::RenderingCamera() const
{
  return this->dataPtr->camera;
}

//////////////////////////////////////////////////
std::string CameraSensor::InfoTopic() const
{
  return this->dataPtr->infoTopic;
}

//////////////////////////////////////////////////
bool CameraSensor::AdvertiseInfo()
{
  // TODO(anyone) Make info topic configurable from SDF
  // Info topic must be at same level as image topic
  auto parts = common::Split(this->Topic(), '/');
  parts.pop_back();

  for (const auto &part : parts)
  {
    if (!part.empty())
      this->dataPtr->infoTopic += "/" + part;
  }
  this->dataPtr->infoTopic += "/camera_info";

  return this->AdvertiseInfo(this->dataPtr->infoTopic);
}

//////////////////////////////////////////////////
bool CameraSensor::AdvertiseInfo(const std::string &_topic)
{
  this->dataPtr->infoTopic = _topic;

  this->dataPtr->infoPub =
      this->dataPtr->node.Advertise<ignition::msgs::CameraInfo>(
      this->dataPtr->infoTopic);
  if (!this->dataPtr->infoPub)
  {
    ignerr << "Unable to create publisher on topic ["
      << this->dataPtr->infoTopic << "].\n";
  }
  else
  {
    igndbg << "Camera info for [" << this->Name() << "] advertised on ["
           << this->dataPtr->infoTopic << "]" << std::endl;
  }

  return this->dataPtr->infoPub;
}

//////////////////////////////////////////////////
void CameraSensor::PublishInfo(const ignition::common::Time &_now)
{
  this->dataPtr->infoMsg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
  this->dataPtr->infoMsg.mutable_header()->mutable_stamp()->set_nsec(
      _now.nsec);
  this->dataPtr->infoPub.Publish(this->dataPtr->infoMsg);
}

//////////////////////////////////////////////////
void CameraSensor::PopulateInfo(const sdf::Camera *_cameraSdf)
{
  unsigned int width = _cameraSdf->ImageWidth();
  unsigned int height = _cameraSdf->ImageHeight();

  msgs::CameraInfo::Distortion *distortion =
    this->dataPtr->infoMsg.mutable_distortion();

  distortion->set_model(msgs::CameraInfo::Distortion::PLUMB_BOB);
  distortion->add_k(_cameraSdf->DistortionK1());
  distortion->add_k(_cameraSdf->DistortionK2());
  distortion->add_k(_cameraSdf->DistortionP1());
  distortion->add_k(_cameraSdf->DistortionP2());
  distortion->add_k(_cameraSdf->DistortionK3());

  msgs::CameraInfo::Intrinsics *intrinsics =
    this->dataPtr->infoMsg.mutable_intrinsics();

  intrinsics->add_k(_cameraSdf->LensIntrinsicsFx());
  intrinsics->add_k(0.0);
  intrinsics->add_k(_cameraSdf->LensIntrinsicsCx());

  intrinsics->add_k(0.0);
  intrinsics->add_k(_cameraSdf->LensIntrinsicsFy());
  intrinsics->add_k(_cameraSdf->LensIntrinsicsCy());

  intrinsics->add_k(0.0);
  intrinsics->add_k(0.0);
  intrinsics->add_k(1.0);

  msgs::CameraInfo::Projection *proj =
    this->dataPtr->infoMsg.mutable_projection();

  proj->add_p(_cameraSdf->LensProjectionFx());
  proj->add_p(0.0);
  proj->add_p(_cameraSdf->LensProjectionCx());
  proj->add_p(_cameraSdf->LensProjectionTx());

  proj->add_p(0.0);
  proj->add_p(_cameraSdf->LensProjectionFy());
  proj->add_p(_cameraSdf->LensProjectionCy());
  proj->add_p(_cameraSdf->LensProjectionTy());

  proj->add_p(0.0);
  proj->add_p(0.0);
  proj->add_p(1.0);
  proj->add_p(0.0);

  // Set the rectification matrix to identity
  this->dataPtr->infoMsg.add_rectification_matrix(1.0);
  this->dataPtr->infoMsg.add_rectification_matrix(0.0);
  this->dataPtr->infoMsg.add_rectification_matrix(0.0);

  this->dataPtr->infoMsg.add_rectification_matrix(0.0);
  this->dataPtr->infoMsg.add_rectification_matrix(1.0);
  this->dataPtr->infoMsg.add_rectification_matrix(0.0);

  this->dataPtr->infoMsg.add_rectification_matrix(0.0);
  this->dataPtr->infoMsg.add_rectification_matrix(0.0);
  this->dataPtr->infoMsg.add_rectification_matrix(1.0);

  // Note: while Gazebo interprets the camera frame to be looking towards +X,
  // other tools, such as ROS, may interpret this frame as looking towards +Z.
  // TODO(anyone) Expose the `frame_id` as an SDF parameter so downstream users
  // can populate it with arbitrary frames.
  auto infoFrame = this->dataPtr->infoMsg.mutable_header()->add_data();
  infoFrame->set_key("frame_id");
  infoFrame->add_value(this->FrameId());

  this->dataPtr->infoMsg.set_width(width);
  this->dataPtr->infoMsg.set_height(height);
}

//////////////////////////////////////////////////
void CameraSensor::SetBaseline(double _baseline)
{
  this->dataPtr->baseline = _baseline;

  // Also update message
  if (this->dataPtr->infoMsg.has_projection() &&
      this->dataPtr->infoMsg.projection().p_size() == 12)
  {
    auto fx = this->dataPtr->infoMsg.projection().p(0);
    this->dataPtr->infoMsg.mutable_projection()->set_p(3, -fx * _baseline);
  }
}

//////////////////////////////////////////////////
double CameraSensor::Baseline() const
{
  return this->dataPtr->baseline;
}

IGN_SENSORS_REGISTER_SENSOR(CameraSensor)
