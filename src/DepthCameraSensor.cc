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

#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <typeinfo>

#include <gz/common/Console.hh>
#include <gz/common/Image.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/SystemPaths.hh>

#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>

#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>
#include <gz/rendering/Utils.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/DepthCameraSensor.hh"
#include "gz/sensors/Manager.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/ImageGaussianNoiseModel.hh"
#include "gz/sensors/ImageNoise.hh"
#include "gz/sensors/RenderingEvents.hh"

#include "CameraSensorUtil.hh"
#include "PublishInPlace.hh"

// undefine near and far macros from windows.h
#ifdef _WIN32
  #undef near
  #undef far
#endif

/// \brief Private data for DepthCameraSensor
class gz::sensors::DepthCameraSensor::Implementation
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

  /// \brief Write the data of pointMsg from the camera's point cloud in a
  /// single pass: xyz from the first three channels and a grayscale colour
  /// from the depth (channel 0), the same bytes as XYZFromPointCloud +
  /// ConvertDepthToImage + PointCloudUtil::FillMsg without their buffers.
  /// \param[in] _pointCloud Float point cloud, pointMsg size
  /// \param[in] _channels Floats per point of _pointCloud, at least 3
  /// \param[in] _maxDepth Largest finite depth of the frame
  /// \param[out] _out pointMsg's data, row_step * height bytes
  /// \return True if every x, y and z is finite (the message's is_dense)
  public: bool FillPointData(const float *_pointCloud, unsigned int _channels,
      float _maxDepth, char *_out) const;

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

    /// \brief Rendering camera
  public: gz::rendering::DepthCameraPtr depthCamera;

  /// \brief The depth image message. OnNewDepthFrame copies the frame
  /// straight into its data, which is reused frame to frame.
  public: msgs::Image depthMsg;

  /// \brief Largest finite depth of the last depth frame, the colour scale
  /// of its point cloud.
  public: float cloudMaxDepth = 0;

  /// \brief True if the last Render() delivered a depth frame.
  public: bool newFrame = false;

  /// \brief is_dense of the last point cloud, which the next one is
  /// serialized with first (see OnNewRgbPointCloud).
  public: bool pointsDense = false;

  /// \brief True if OnNewRgbPointCloud filled pointMsg for Update() to
  /// publish (image callbacks: the cloud follows the image and its
  /// callbacks, as without in-place publishing).
  public: bool newPoints = false;

  /// \brief True while Update() renders a frame that OnNewDepthFrame
  /// publishes itself, serialized straight into the transport's buffer
  /// (no image callbacks need the message).
  public: bool publishInPlace = false;

  /// \brief True if OnNewDepthFrame published the frame of the last
  /// Render().
  public: bool publishedInPlace = false;

  /// \brief Stamp of the frame the current Render() delivers.
  public: std::chrono::steady_clock::duration frameStamp{0};

  /// \brief Near clip distance.
  public: float near = 0.0;

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

  /// \brief publisher to publish point cloud
  public: transport::Node::Publisher pointPub;
};

using namespace gz;
using namespace sensors;

namespace
{

/// \brief Largest finite depth of a depth frame, 0 if it has none.
/// \param[in] _data Depth frame
/// \param[in] _count Number of depths
float MaxFiniteDepth(const float *_data, unsigned int _count)
{
  float maxDepth = 0;
  for (unsigned int i = 0; i < _count; ++i)
  {
    if (_data[i] > maxDepth && !std::isinf(_data[i]))
    {
      maxDepth = _data[i];
    }
  }
  return maxDepth;
}

/// \brief Fill a depth image message as Update() builds it, all but its
/// data and pixel format type (the fields serialized after the data).
/// \param[in] _sensor The sensor, for the frame id and the sequence
/// \param[out] _msg The message
/// \param[in] _width Image width
/// \param[in] _height Image height
/// \param[in] _stamp Frame stamp
void FillDepthMsg(DepthCameraSensor &_sensor, msgs::Image &_msg,
    unsigned int _width, unsigned int _height,
    const std::chrono::steady_clock::duration &_stamp)
{
  _msg.clear_header();
  _msg.set_width(_width);
  _msg.set_height(_height);
  _msg.set_step(_width * rendering::PixelUtil::BytesPerPixel(
      rendering::PF_FLOAT32_R));
  *_msg.mutable_header()->mutable_stamp() = msgs::Convert(_stamp);
  auto *frame = _msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(_sensor.OpticalFrameId());
  _sensor.AddSequence(_msg.mutable_header(), "default");
}
}  // namespace

//////////////////////////////////////////////////
bool DepthCameraSensor::Implementation::ConvertDepthToImage(
    const float *_data,
    unsigned char *_imageBuffer,
    unsigned int _width, unsigned int _height)
{
  double factor = 255 / MaxFiniteDepth(_data, _height * _width);
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
bool DepthCameraSensor::Implementation::FillPointData(
    const float *_pointCloud, unsigned int _channels, float _maxDepth,
    char *_out) const
{
  const uint32_t step = this->pointMsg.point_step();
  const uint32_t n = this->pointMsg.width() * this->pointMsg.height();
  const uint32_t ox = this->pointMsg.field(0).offset();
  const uint32_t oy = this->pointMsg.field(1).offset();
  const uint32_t oz = this->pointMsg.field(2).offset();
  const uint32_t oc = this->pointMsg.field(3).offset();

  // A float is finite unless all its exponent bits are set.
  constexpr uint32_t kExponent = 0x7f800000u;
  uint32_t nonFinite = 0u;

  // Same expression as ConvertDepthToImage, so the bytes match.
  double factor = 255 / _maxDepth;
  for (uint32_t i = 0; i < n; ++i, _out += step)
  {
    const float *p = _pointCloud + i * _channels;
    uint32_t x, y, z;
    memcpy(&x, p, sizeof(x));
    memcpy(&y, p + 1, sizeof(y));
    memcpy(&z, p + 2, sizeof(z));
    nonFinite |= ((x & kExponent) == kExponent) |
        ((y & kExponent) == kExponent) | ((z & kExponent) == kExponent);
    memcpy(_out + ox, &x, sizeof(x));
    memcpy(_out + oy, &y, sizeof(y));
    memcpy(_out + oz, &z, sizeof(z));
    // Grayscale: equal r, g and b, so byte order does not matter. The depth
    // frame is channel 0 of the point cloud. The fourth byte of the rgb
    // float is 0, as in a message buffer resized from empty.
    const char d = static_cast<char>(
        static_cast<unsigned char>(255 - (p[0] * factor)));
    const char rgb[4] = {d, d, d, 0};
    memcpy(_out + oc, rgb, sizeof(rgb));
  }
  return nonFinite == 0u;
}

//////////////////////////////////////////////////
bool DepthCameraSensor::Implementation::SaveImage(const float *_data,
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
  : CameraSensor(), dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
DepthCameraSensor::~DepthCameraSensor()
{
  this->dataPtr->depthConnection.reset();
  this->dataPtr->pointCloudConnection.reset();
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

  if (!_sdf.CameraSensor()->CameraInfoTopic().empty())
  {
    this->SetInfoTopic(_sdf.CameraSensor()->CameraInfoTopic());
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
  msgs::InitPointCloudPacked(
        this->dataPtr->pointMsg,
        this->OpticalFrameId(),
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
void DepthCameraSensor::OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &_format)
{
  GZ_PROFILE("DepthCameraSensor::OnNewDepthFrame");
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  unsigned int depthSamples = _width * _height;
  unsigned int depthBufferSize = depthSamples * sizeof(float);

  common::Image::PixelFormatType format =
    common::Image::ConvertPixelFormat(_format);

  if (this->dataPtr->publishInPlace)
  {
    // The message as Update() would build it, serialized around the data,
    // which is written once, from the frame straight into the transport's
    // buffer.
    msgs::Image head;
    FillDepthMsg(*this, head, _width, _height, this->dataPtr->frameStamp);
    msgs::Image tail;
    tail.set_pixel_format_type(msgs::PixelFormatType::R_FLOAT32);

    GZ_PROFILE("DepthCameraSensor::OnNewDepthFrame Publish");
    this->dataPtr->publishedInPlace = PublishWithData(this->dataPtr->pub,
        head.SerializeAsString(), msgs::Image::kDataFieldNumber,
        depthBufferSize, tail.SerializeAsString(), [&](char *_out)
        {
          memcpy(_out, _scan, depthBufferSize);
          return true;
        });
  }
  if (!this->dataPtr->publishedInPlace)
  {
    // Resizing to the same size is a no-op after the first frame.
    std::string *data = this->dataPtr->depthMsg.mutable_data();
    data->resize(depthBufferSize);
    memcpy(data->data(), _scan, depthBufferSize);
  }
  this->dataPtr->newFrame = true;

  // Point cloud colour scale (see ConvertDepthToImage), found here while the
  // depth frame is hot in cache.
  if (this->dataPtr->pointCloudConnection)
    this->dataPtr->cloudMaxDepth = MaxFiniteDepth(_scan, depthSamples);

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
  GZ_PROFILE("DepthCameraSensor::OnNewRgbPointCloud");
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  msgs::PointCloudPacked &msg = this->dataPtr->pointMsg;
  // Only for point cloud subscribers (as Update() published it), and only
  // a cloud of the message's size.
  if (!this->HasPointConnections())
    return;
  if (_channels < 3u)
  {
    gzerr << "DepthCameraSensor: unexpected point cloud frame ("
          << _channels << " channels)" << std::endl;
    return;
  }
  if (_width * _height != msg.width() * msg.height())
  {
    gzerr << "DepthCameraSensor: point cloud frame of " << _width << "x"
          << _height << " for a " << msg.width() << "x" << msg.height()
          << " message" << std::endl;
    return;
  }

  if (!this->dataPtr->publishInPlace)
  {
    // Image callbacks, or a render outside Update(): Update() publishes the
    // cloud after the image. Filled here, while the camera's buffer is
    // valid.
    std::string *data = msg.mutable_data();
    data->resize(static_cast<std::size_t>(msg.row_step()) * msg.height());
    msg.set_is_dense(this->dataPtr->FillPointData(_scan, _channels,
        this->dataPtr->cloudMaxDepth, data->data()));
    this->dataPtr->newPoints = true;
    return;
  }

  // The message as Update() built it, serialized around the data, which is
  // written once, from the frame straight into the transport's buffer.
  // pointMsg itself keeps empty data here.
  *msg.mutable_header()->mutable_stamp() =
      msgs::Convert(this->dataPtr->frameStamp);
  this->AddSequence(msg.mutable_header(), "pointMsg");
  const std::string front = msg.SerializeAsString();
  const std::size_t dataSize =
      static_cast<std::size_t>(msg.row_step()) * msg.height();

  GZ_PROFILE("DepthCameraSensor::OnNewRgbPointCloud Publish");
  PublishPointCloudWithData(this->dataPtr->pointPub, front, dataSize,
      [&](char *_out)
      {
        return this->dataPtr->FillPointData(_scan, _channels,
            this->dataPtr->cloudMaxDepth, _out);
      }, this->dataPtr->pointsDense);
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
    this->DropRenderedAhead();
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

  // A frame rendered ahead is completed whatever the subscribers are now:
  // the subscribers seen by RenderAhead() decide, as if they had changed
  // after this update.
  const bool renderedAhead = this->RenderedAhead();
  this->DropRenderedAhead();
  if (!renderedAhead && !this->HasDepthConnections() &&
      !this->HasPointConnections())
  {
    return false;
  }

  // Without image callbacks OnNewDepthFrame publishes the depth image
  // itself; OnNewRgbPointCloud publishes the point cloud.
  this->dataPtr->newFrame = false;
  this->dataPtr->frameStamp = _now;
  this->dataPtr->publishInPlace =
      this->dataPtr->imageEvent.ConnectionCount() == 0u;
  this->dataPtr->publishedInPlace = false;
  this->dataPtr->newPoints = false;
  // (the in-place path serializes pointMsg without data and is_dense, see
  // OnNewRgbPointCloud)
  this->dataPtr->pointMsg.clear_data();
  this->dataPtr->pointMsg.clear_is_dense();
  if (renderedAhead)
  {
    this->dataPtr->depthCamera->PostRender();
  }
  else
  {
    this->UpdatePointCloudConnection();

    // generate sensor data
    this->Render();
  }
  this->dataPtr->publishInPlace = false;
  if (!this->dataPtr->newFrame)
    return false;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->publishedInPlace)
    return true;

  // create message; OnNewDepthFrame filled its data
  msgs::Image &msg = this->dataPtr->depthMsg;
  FillDepthMsg(*this, msg, this->dataPtr->depthCamera->ImageWidth(),
      this->dataPtr->depthCamera->ImageHeight(), _now);
  msg.set_pixel_format_type(msgs::PixelFormatType::R_FLOAT32);

  {
    GZ_PROFILE("DepthCameraSensor::Update Publish");
    this->dataPtr->pub.Publish(msg);
  }

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

  // Only a frame rendered with the point cloud connected has a cloud;
  // OnNewRgbPointCloud published it in place unless there are image
  // callbacks.
  if (this->HasPointConnections() && this->dataPtr->newPoints)
  {
    msgs::PointCloudPacked &pointMsg = this->dataPtr->pointMsg;
    *pointMsg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
    this->AddSequence(pointMsg.mutable_header(), "pointMsg");
    this->dataPtr->pointPub.Publish(pointMsg);
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
void DepthCameraSensor::UpdatePointCloudConnection()
{
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
}

//////////////////////////////////////////////////
bool DepthCameraSensor::RenderAhead(
    const std::chrono::steady_clock::duration &/*_now*/)
{
  // Only a DepthCameraSensor: a derived sensor's Update() may run code
  // before it renders. Only when Update(_now) would render, and through
  // RenderingSensor::Render() only the camera (the scene update is manual).
  // Image callbacks run user code between the serial renders, which may
  // change the scene or draw random numbers that the next sensor's render
  // draws: serial then.
  if (typeid(*this) != typeid(DepthCameraSensor) ||
      !this->dataPtr->initialized || !this->dataPtr->depthCamera ||
      !this->ManualSceneUpdate() ||
      this->dataPtr->imageEvent.ConnectionCount() > 0u ||
      (!this->HasDepthConnections() && !this->HasPointConnections()))
  {
    return false;
  }
  GZ_PROFILE("DepthCameraSensor::RenderAhead");
  this->UpdatePointCloudConnection();
  this->dataPtr->depthCamera->Render();
  this->dataPtr->depthCamera->IssueReadback();
  return true;
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
