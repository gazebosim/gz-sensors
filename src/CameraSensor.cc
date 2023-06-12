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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>

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

#include "gz/sensors/CameraSensor.hh"
#include "gz/sensors/ImageBrownDistortionModel.hh"
#include "gz/sensors/ImageDistortion.hh"
#include "gz/sensors/ImageGaussianNoiseModel.hh"
#include "gz/sensors/ImageNoise.hh"
#include "gz/sensors/Manager.hh"
#include "gz/sensors/RenderingEvents.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/SensorTypes.hh"

#include "gz/rendering/Utils.hh"

using namespace gz;
using namespace sensors;

/// \brief Private data for CameraSensor
class gz::sensors::CameraSensorPrivate
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

  /// \brief Computes the OpenGL NDC matrix
  /// \param[in] _left Left vertical clipping plane
  /// \param[in] _right Right vertical clipping plane
  /// \param[in] _bottom Bottom horizontal clipping plane
  /// \param[in] _top Top horizontal clipping plane
  /// \param[in] _near Distance to the nearer depth clipping plane
  ///            This value is negative if the plane is to be behind
  ///            the camera
  /// \param[in] _far Distance to the farther depth clipping plane
  ///            This value is negative if the plane is to be behind
  ///            the camera
  /// \return OpenGL NDC (Normalized Device Coordinates) matrix
  public: static math::Matrix4d BuildNDCMatrix(
          double _left, double _right,
          double _bottom, double _top,
          double _near, double _far);

  /// \brief Computes the OpenGL perspective matrix
  /// \param[in] _intrinsicsFx Horizontal focal length (in pixels)
  /// \param[in] _intrinsicsFy Vertical focal length (in pixels)
  /// \param[in] _intrinsicsCx X coordinate of principal point in pixels
  /// \param[in] _intrinsicsCy Y coordinate of principal point in pixels
  /// \param[in] _intrinsicsS Skew coefficient defining the angle between
  ///            the x and y pixel axes
  /// \param[in] _clipNear Distance to the nearer depth clipping plane
  ///            This value is negative if the plane is to be behind
  ///            the camera
  /// \param[in] _clipFar Distance to the farther depth clipping plane
  ///            This value is negative if the plane is to be behind
  ///            the camera
  /// \return OpenGL perspective matrix
  public: static math::Matrix4d BuildPerspectiveMatrix(
          double _intrinsicsFx, double _intrinsicsFy,
          double _intrinsicsCx, double _intrinsicsCy,
          double _intrinsicsS,
          double _clipNear, double _clipFar);

  /// \brief Computes the OpenGL projection matrix by multiplying
  ///        the OpenGL Normalized Device Coordinates matrix (NDC) with
  ///        the OpenGL perspective matrix
  ///        openglProjectionMatrix = ndcMatrix * perspectiveMatrix
  /// \param[in] _imageWidth Image width (in pixels)
  /// \param[in] _imageHeight Image height (in pixels)
  /// \param[in] _intrinsicsFx Horizontal focal length (in pixels)
  /// \param[in] _intrinsicsFy Vertical focal length (in pixels)
  /// \param[in] _intrinsicsCx X coordinate of principal point in pixels
  /// \param[in] _intrinsicsCy Y coordinate of principal point in pixels
  /// \param[in] _intrinsicsS Skew coefficient defining the angle between
  ///             the x and y pixel axes
  /// \param[in] _clipNear Distance to the nearer depth clipping plane
  ///            This value is negative if the plane is to be behind
  ///            the camera
  /// \param[in] _clipFar Distance to the farther depth clipping plane
  ///            This value is negative if the plane is to be behind
  ///            the camera
  /// \return OpenGL projection matrix
  public: static math::Matrix4d BuildProjectionMatrix(
          double _imageWidth, double _imageHeight,
          double _intrinsicsFx, double _intrinsicsFy,
          double _intrinsicsCx, double _intrinsicsCy,
          double _intrinsicsS,
          double _clipNear, double _clipFar);

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub;

  /// \brief Camera info publisher to publish images
  public: transport::Node::Publisher infoPub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Rendering camera
  public: gz::rendering::CameraPtr camera;

  /// \brief Pointer to an image to be published
  public: gz::rendering::Image image;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Distortion added to sensor data
  public: DistortionPtr distortion;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: gz::common::EventT<
          void(const gz::msgs::Image &)> imageEvent;

  /// \brief Connection to the Manager's scene change event.
  public: gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief True if camera is triggered by a topic
  public: bool isTriggeredCamera = false;

  /// \brief True if camera has been triggered by a topic
  public: bool isTriggered = false;

  /// \brief Topic for camera trigger
  public: std::string triggerTopic = "";

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

  /// \brief The frame this camera uses in its camera_info topic.
  public: std::string opticalFrameId{""};

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
  sdf::Camera *cameraSdf = this->dataPtr->sdfSensor.CameraSensor();
  if (!cameraSdf)
  {
    gzerr << "Unable to access camera SDF element.\n";
    return false;
  }

  unsigned int width = cameraSdf->ImageWidth();
  unsigned int height = cameraSdf->ImageHeight();

  this->dataPtr->camera = this->Scene()->CreateCamera(this->Name());
  this->dataPtr->camera->SetImageWidth(width);
  this->dataPtr->camera->SetImageHeight(height);
  this->dataPtr->camera->SetNearClipPlane(cameraSdf->NearClip());
  this->dataPtr->camera->SetFarClipPlane(cameraSdf->FarClip());
  this->dataPtr->camera->SetVisibilityMask(cameraSdf->VisibilityMask());
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
      gzwarn << "The camera sensor only supports Gaussian noise. "
       << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
       << "] is not supported." << std::endl;
    }
  }

  // \todo(nkoeng) these parameters via sdf
  this->dataPtr->camera->SetAntiAliasing(cameraSdf->AntiAliasingValue());

  math::Angle angle = cameraSdf->HorizontalFov();
  if (angle < 0.01 || angle > GZ_PI*2)
  {
    gzerr << "Invalid horizontal field of view [" << angle << "]\n";

    return false;
  }
  this->dataPtr->camera->SetAspectRatio(static_cast<double>(width)/height);
  this->dataPtr->camera->SetHFOV(angle);

  if (cameraSdf->Element()->HasElement("distortion")) {
    this->dataPtr->distortion =
        ImageDistortionFactory::NewDistortionModel(*cameraSdf, "camera");
    this->dataPtr->distortion->Load(*cameraSdf);

    std::dynamic_pointer_cast<ImageBrownDistortionModel>(
        this->dataPtr->distortion)->SetCamera(this->dataPtr->camera);
  }

  sdf::PixelFormatType pixelFormat = cameraSdf->PixelFormat();
  switch (pixelFormat)
  {
    case sdf::PixelFormatType::RGB_INT8:
      this->dataPtr->camera->SetImageFormat(rendering::PF_R8G8B8);
      break;
    case sdf::PixelFormatType::L_INT8:
      this->dataPtr->camera->SetImageFormat(rendering::PF_L8);
      break;
    case sdf::PixelFormatType::L_INT16:
      this->dataPtr->camera->SetImageFormat(rendering::PF_L16);
      break;
    case sdf::PixelFormatType::BAYER_RGGB8:
      this->dataPtr->camera->SetImageFormat(rendering::PF_BAYER_RGGB8);
      break;
    case sdf::PixelFormatType::BAYER_BGGR8:
      this->dataPtr->camera->SetImageFormat(rendering::PF_BAYER_BGGR8);
      break;
    case sdf::PixelFormatType::BAYER_GBRG8:
      this->dataPtr->camera->SetImageFormat(rendering::PF_BAYER_GBRG8);
      break;
    case sdf::PixelFormatType::BAYER_GRBG8:
      this->dataPtr->camera->SetImageFormat(rendering::PF_BAYER_GRBG8);
      break;
    default:
      gzerr << "Unsupported pixel format ["
        << static_cast<int>(pixelFormat) << "]\n";
      break;
  }

  // Update the DOM object intrinsics to have consistent
  // intrinsics between ogre camera and camera_info msg
  if(!cameraSdf->HasLensIntrinsics())
  {
    auto intrinsicMatrix =
      gz::rendering::projectionToCameraIntrinsic(
        this->dataPtr->camera->ProjectionMatrix(),
        this->dataPtr->camera->ImageWidth(),
        this->dataPtr->camera->ImageHeight()
      );

    cameraSdf->SetLensIntrinsicsFx(intrinsicMatrix(0, 0));
    cameraSdf->SetLensIntrinsicsFy(intrinsicMatrix(1, 1));
    cameraSdf->SetLensIntrinsicsCx(intrinsicMatrix(0, 2));
    cameraSdf->SetLensIntrinsicsCy(intrinsicMatrix(1, 2));
  }
  // set custom projection matrix based on intrinsics param specified in sdf
  else
  {
    double fx = cameraSdf->LensIntrinsicsFx();
    double fy = cameraSdf->LensIntrinsicsFy();
    double cx = cameraSdf->LensIntrinsicsCx();
    double cy = cameraSdf->LensIntrinsicsCy();
    double s = cameraSdf->LensIntrinsicsSkew();
    auto projectionMatrix = CameraSensorPrivate::BuildProjectionMatrix(
        this->dataPtr->camera->ImageWidth(),
        this->dataPtr->camera->ImageHeight(),
        fx, fy, cx, cy, s,
        this->dataPtr->camera->NearClipPlane(),
        this->dataPtr->camera->FarClipPlane());
    this->dataPtr->camera->SetProjectionMatrix(projectionMatrix);
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

  // Update the DOM object intrinsics to have consistent
  // projection matrix values between ogre camera and camera_info msg
  // If these values are not defined in the SDF then we need to update
  // these values to something reasonable. The projection matrix is
  // the cumulative effect of intrinsic and extrinsic parameters
  if(!cameraSdf->HasLensProjection())
  {
    // Note that the matrix from Ogre via camera->ProjectionMatrix() has a
    // different format than the projection matrix used in SDFormat.
    // This is why they are converted using projectionToCameraIntrinsic.
    // The resulting matrix is the intrinsic matrix, but since the user has
    // not overridden the values, this is also equal to the projection matrix.
    auto intrinsicMatrix =
      gz::rendering::projectionToCameraIntrinsic(
        this->dataPtr->camera->ProjectionMatrix(),
        this->dataPtr->camera->ImageWidth(),
        this->dataPtr->camera->ImageHeight()
      );
    cameraSdf->SetLensProjectionFx(intrinsicMatrix(0, 0));
    cameraSdf->SetLensProjectionFy(intrinsicMatrix(1, 1));
    cameraSdf->SetLensProjectionCx(intrinsicMatrix(0, 2));
    cameraSdf->SetLensProjectionCy(intrinsicMatrix(1, 2));
  }
  // set custom projection matrix based on projection param specified in sdf
  else
  {
    // tx and ty are not used
    double fx = cameraSdf->LensProjectionFx();
    double fy = cameraSdf->LensProjectionFy();
    double cx = cameraSdf->LensProjectionCx();
    double cy = cameraSdf->LensProjectionCy();
    double s = 0;

    auto projectionMatrix = CameraSensorPrivate::BuildProjectionMatrix(
        this->dataPtr->camera->ImageWidth(),
        this->dataPtr->camera->ImageHeight(),
        fx, fy, cx, cy, s,
        this->dataPtr->camera->NearClipPlane(),
        this->dataPtr->camera->FarClipPlane());
    this->dataPtr->camera->SetProjectionMatrix(projectionMatrix);
  }

  // Populate camera info topic
  this->PopulateInfo(cameraSdf);

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
  if (this->Scene() && this->dataPtr->camera)
  {
    this->Scene()->DestroySensor(this->dataPtr->camera);
  }
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
    gzerr << "Attempting to a load a Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    gzerr << "Attempting to a load a Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  if (this->Topic().empty())
    this->SetTopic("/camera");

  if (!_sdf.CameraSensor()->CameraInfoTopic().empty())
  {
    this->dataPtr->infoTopic = _sdf.CameraSensor()->CameraInfoTopic();
  }

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<gz::msgs::Image>(
          this->Topic());
  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  gzdbg << "Camera images for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  if (_sdf.CameraSensor()->Triggered())
  {
    if (!_sdf.CameraSensor()->TriggerTopic().empty())
    {
      this->dataPtr->triggerTopic = _sdf.CameraSensor()->TriggerTopic();
    }
    else
    {
      this->dataPtr->triggerTopic =
          transport::TopicUtils::AsValidTopic(
          this->Topic() + "/trigger");

      if (this->dataPtr->triggerTopic.empty())
      {
        gzerr << "Invalid trigger topic name" << std::endl;
        return false;
      }
    }

    this->dataPtr->node.Subscribe(this->dataPtr->triggerTopic,
        &CameraSensor::OnTrigger, this);

    gzdbg << "Camera trigger messages for [" << this->Name() << "] subscribed"
           << " on [" << this->dataPtr->triggerTopic << "]" << std::endl;
    this->dataPtr->isTriggeredCamera = true;
  }

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
gz::common::ConnectionPtr CameraSensor::ConnectImageCallback(
    std::function<void(const gz::msgs::Image &)> _callback)
{
  return this->dataPtr->imageEvent.Connect(_callback);
}

/////////////////////////////////////////////////
void CameraSensor::SetScene(gz::rendering::ScenePtr _scene)
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
bool CameraSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("CameraSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->camera)
  {
    gzerr << "Camera doesn't exist.\n";
    return false;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // move the camera to the current pose
  this->dataPtr->camera->SetLocalPose(this->Pose());

  if (this->HasInfoConnections())
  {
    // publish the camera info message
    this->PublishInfo(_now);
  }

  // render only if necessary
  if (this->dataPtr->isTriggeredCamera &&
      !this->dataPtr->isTriggered)
  {
    return true;
  }

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

  if (this->HasImageConnections() || this->dataPtr->saveImage)
  {
    // generate sensor data
    this->Render();
    {
      GZ_PROFILE("CameraSensor::Update Copy image");
      this->dataPtr->camera->Copy(this->dataPtr->image);
    }

    unsigned int width = this->dataPtr->camera->ImageWidth();
    unsigned int height = this->dataPtr->camera->ImageHeight();
    unsigned char *data = this->dataPtr->image.Data<unsigned char>();

    gz::common::Image::PixelFormatType
        format{common::Image::UNKNOWN_PIXEL_FORMAT};
    msgs::PixelFormatType msgsPixelFormat =
      msgs::PixelFormatType::UNKNOWN_PIXEL_FORMAT;

    switch (this->dataPtr->camera->ImageFormat())
    {
      case rendering::PF_R8G8B8:
        format = common::Image::RGB_INT8;
        msgsPixelFormat = msgs::PixelFormatType::RGB_INT8;
        break;
      case rendering::PF_L8:
        format = common::Image::L_INT8;
        msgsPixelFormat = msgs::PixelFormatType::L_INT8;
        break;
      case rendering::PF_L16:
        format = common::Image::L_INT16;
        msgsPixelFormat = msgs::PixelFormatType::L_INT16;
        break;
      case rendering::PF_BAYER_RGGB8:
        format = common::Image::BAYER_RGGB8;
        msgsPixelFormat = msgs::PixelFormatType::BAYER_RGGB8;
        break;
      case rendering::PF_BAYER_BGGR8:
        format = common::Image::BAYER_BGGR8;
        msgsPixelFormat = msgs::PixelFormatType::BAYER_BGGR8;
        break;
      case rendering::PF_BAYER_GBRG8:
        format = common::Image::BAYER_GBRG8;
        msgsPixelFormat = msgs::PixelFormatType::BAYER_GBRG8;
        break;
      case rendering::PF_BAYER_GRBG8:
        format = common::Image::BAYER_GRBG8;
        msgsPixelFormat = msgs::PixelFormatType::BAYER_GRBG8;
        break;
      default:
        gzerr << "Unsupported pixel format ["
          << this->dataPtr->camera->ImageFormat() << "]\n";
        break;
    }

    // create message
    msgs::Image msg;
    {
      GZ_PROFILE("CameraSensor::Update Message");
      msg.set_width(width);
      msg.set_height(height);
      msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
                   this->dataPtr->camera->ImageFormat()));
      msg.set_pixel_format_type(msgsPixelFormat);
      *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
      auto frame = msg.mutable_header()->add_data();
      frame->set_key("frame_id");
      frame->add_value(this->dataPtr->opticalFrameId);
      msg.set_data(data, this->dataPtr->camera->ImageMemorySize());
    }

    // publish the image message
    {
      this->AddSequence(msg.mutable_header());
      GZ_PROFILE("CameraSensor::Update Publish");
      this->dataPtr->pub.Publish(msg);
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
  }

  if (this->dataPtr->isTriggeredCamera)
  {
    return this->dataPtr->isTriggered = false;
  }

  return true;
}

//////////////////////////////////////////////////
void CameraSensor::OnTrigger(const gz::msgs::Boolean &/*_msg*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->isTriggered = true;
}

//////////////////////////////////////////////////
bool CameraSensorPrivate::SaveImage(const unsigned char *_data,
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
  if (this->dataPtr->infoTopic.empty())
  {
    auto parts = common::Split(this->Topic(), '/');
    parts.pop_back();
    for (const auto &part : parts)
    {
      if (!part.empty())
        this->dataPtr->infoTopic += "/" + part;
    }
    this->dataPtr->infoTopic += "/camera_info";
  }

  return this->AdvertiseInfo(this->dataPtr->infoTopic);
}

//////////////////////////////////////////////////
bool CameraSensor::AdvertiseInfo(const std::string &_topic)
{
  this->dataPtr->infoTopic = _topic;

  this->dataPtr->infoPub =
      this->dataPtr->node.Advertise<gz::msgs::CameraInfo>(
      this->dataPtr->infoTopic);
  if (!this->dataPtr->infoPub)
  {
    gzerr << "Unable to create publisher on topic ["
      << this->dataPtr->infoTopic << "].\n";
  }
  else
  {
    gzdbg << "Camera info for [" << this->Name() << "] advertised on ["
           << this->dataPtr->infoTopic << "]" << std::endl;
  }

  return this->dataPtr->infoPub;
}

//////////////////////////////////////////////////
void CameraSensor::PublishInfo(
  const std::chrono::steady_clock::duration &_now)
{
  *this->dataPtr->infoMsg.mutable_header()->mutable_stamp() =
    msgs::Convert(_now);
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
  // To make this configurable the user has the option to set an optical frame.
  // If the user has set <optical_frame_id> in the cameraSdf use it,
  // otherwise fall back to the sensor frame.
  if (_cameraSdf->OpticalFrameId().empty())
  {
   this->dataPtr->opticalFrameId = this->FrameId();
  }
  else
  {
   this->dataPtr->opticalFrameId = _cameraSdf->OpticalFrameId();
  }
  auto infoFrame = this->dataPtr->infoMsg.mutable_header()->add_data();
  infoFrame->set_key("frame_id");
  infoFrame->add_value(this->dataPtr->opticalFrameId);

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

//////////////////////////////////////////////////
bool CameraSensor::HasConnections() const
{
  return this->HasImageConnections() || this->HasInfoConnections();
}

//////////////////////////////////////////////////
bool CameraSensor::HasImageConnections() const
{
  return (this->dataPtr->pub && this->dataPtr->pub.HasConnections()) ||
         this->dataPtr->imageEvent.ConnectionCount() > 0u;
}

//////////////////////////////////////////////////
bool CameraSensor::HasInfoConnections() const
{
  return this->dataPtr->infoPub && this->dataPtr->infoPub.HasConnections();
}

//////////////////////////////////////////////////
math::Matrix4d CameraSensorPrivate::BuildProjectionMatrix(
    double _imageWidth, double _imageHeight,
    double _intrinsicsFx, double _intrinsicsFy,
    double _intrinsicsCx, double _intrinsicsCy,
    double _intrinsicsS,
    double _clipNear, double _clipFar)
{
  return CameraSensorPrivate::BuildNDCMatrix(
           0, _imageWidth, 0, _imageHeight, _clipNear, _clipFar) *
           CameraSensorPrivate::BuildPerspectiveMatrix(
             _intrinsicsFx, _intrinsicsFy,
             _intrinsicsCx, _imageHeight - _intrinsicsCy,
             _intrinsicsS, _clipNear, _clipFar);
}

//////////////////////////////////////////////////
math::Matrix4d CameraSensorPrivate::BuildNDCMatrix(
    double _left, double _right,
    double _bottom, double _top,
    double _near, double _far)
{
  double inverseWidth = 1.0 / (_right - _left);
  double inverseHeight = 1.0 / (_top - _bottom);
  double inverseDistance = 1.0 / (_far - _near);

  return math::Matrix4d(
           2.0 * inverseWidth,
           0.0,
           0.0,
           -(_right + _left) * inverseWidth,
           0.0,
           2.0 * inverseHeight,
           0.0,
           -(_top + _bottom) * inverseHeight,
           0.0,
           0.0,
           -2.0 * inverseDistance,
           -(_far + _near) * inverseDistance,
           0.0,
           0.0,
           0.0,
           1.0);
}

//////////////////////////////////////////////////
math::Matrix4d CameraSensorPrivate::BuildPerspectiveMatrix(
    double _intrinsicsFx, double _intrinsicsFy,
    double _intrinsicsCx, double _intrinsicsCy,
    double _intrinsicsS,
    double _clipNear, double _clipFar)
{
  return math::Matrix4d(
           _intrinsicsFx,
           _intrinsicsS,
           -_intrinsicsCx,
           0.0,
           0.0,
           _intrinsicsFy,
           -_intrinsicsCy,
           0.0,
           0.0,
           0.0,
           _clipNear + _clipFar,
           _clipNear * _clipFar,
           0.0,
           0.0,
           -1.0,
           0.0);
}
