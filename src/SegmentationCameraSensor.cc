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

#include <memory>
#include <mutex>

#include <ignition/common/Console.hh>
#include <ignition/common/Image.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/msgs.hh>
#include <ignition/rendering/SegmentationCamera.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include "ignition/sensors/RenderingEvents.hh"
#include "ignition/sensors/SegmentationCameraSensor.hh"
#include "ignition/sensors/SensorFactory.hh"

using namespace ignition;
using namespace sensors;

class ignition::sensors::SegmentationCameraSensorPrivate
{
  /// \brief Save an image
  /// \param[in] _coloredBuffer buffer of colored map
  /// \param[in] _labelsBuffer buffer of labels map
  /// \param[in] _width width of image in pixels
  /// \param[in] _height height of image in pixels
  /// \return True if the image was saved successfully. False can mean
  /// that the image save path does not exist and creation
  /// of the path was not possible.
  public: bool SaveImage(const uint8_t *_coloredBuffer,
    const uint8_t *_labelsBuffer, unsigned int _width, unsigned int _height);

  /// \brief SDF Sensor DOM Object
  public: sdf::Sensor sdfSensor;

  /// \brief True if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Rendering Segmentation Camera
  public: rendering::SegmentationCameraPtr camera {nullptr};

  /// \brief Node to create publisher
  public: transport::Node node;

  /// \brief Publisher to publish segmentation colored image
  public: transport::Node::Publisher coloredMapPublisher;

  /// \brief Publisher to publish segmentation labels image
  public: transport::Node::Publisher labelsMapPublisher;

  /// \brief Segmentation colored image message
  public: msgs::Image coloredMapMsg;

  /// \brief Segmentation labels image message
  public: msgs::Image labelsMapMsg;

  /// \brief Topic suffix to publish the segmentation colored map
  public: const std::string topicColoredMapSuffix = "/colored_map";

  /// \brief Topic suffix to publish the segmentation labels map
  public: const std::string topicLabelsMapSuffix = "/labels_map";

  /// \brief Buffer contains the segmentation colored map data
  public: uint8_t *segmentationColoredBuffer {nullptr};

  /// \brief Buffer contains the segmentation labels map data
  public: uint8_t *segmentationLabelsBuffer {nullptr};

  /// \brief Segmentation type (Semantic / Instance)
  public: rendering::SegmentationType type
    {rendering::SegmentationType::SEMANTIC};

  /// \brief Connection to the new segmentation frames data
  public: common::ConnectionPtr newSegmentationConnection {nullptr};

  /// \brief Connection to the Manager's scene change event.
  public: common::ConnectionPtr sceneChangeConnection {nullptr};

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief True to save images
  public: bool saveImage = false;

  /// \brief Path directory to where images are saved
  public: std::string saveImagePath = "./";

  /// \brief Prefix of an image name
  public: std::string saveImagePrefix = "./";

  /// \brief Counter used to set the image filename
  public: std::uint64_t saveImageCounter = 0;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: ignition::common::EventT<
          void(const ignition::msgs::Image &)> imageEvent;
};

//////////////////////////////////////////////////
SegmentationCameraSensor::SegmentationCameraSensor()
  : CameraSensor(), dataPtr(new SegmentationCameraSensorPrivate)
{
}

/////////////////////////////////////////////////
SegmentationCameraSensor::~SegmentationCameraSensor()
{
  if (this->dataPtr->segmentationColoredBuffer)
    delete this->dataPtr->segmentationColoredBuffer;

  if (this->dataPtr->segmentationLabelsBuffer)
    delete this->dataPtr->segmentationLabelsBuffer;
}

/////////////////////////////////////////////////
bool SegmentationCameraSensor::Init()
{
  return CameraSensor::Init();
}

/////////////////////////////////////////////////
bool SegmentationCameraSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

/////////////////////////////////////////////////
bool SegmentationCameraSensor::Load(const sdf::Sensor &_sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!Sensor::Load(_sdf))
  {
    return false;
  }

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::SEGMENTATION_CAMERA)
  {
    ignerr << "Attempting to a load a Segmentation Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    ignerr << "Attempting to a load a Segmentation Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  // Create the segmentation colored map image publisher
  this->dataPtr->coloredMapPublisher =
      this->dataPtr->node.Advertise<ignition::msgs::Image>(
          this->Topic() + this->dataPtr->topicColoredMapSuffix);

  // Create the segmentation labels map image publisher
  this->dataPtr->labelsMapPublisher =
      this->dataPtr->node.Advertise<ignition::msgs::Image>(
          this->Topic() + this->dataPtr->topicLabelsMapSuffix);

  if (!this->dataPtr->labelsMapPublisher)
  {
    ignerr << "Unable to create publisher on topic ["
      << this->Topic() + this->dataPtr->topicLabelsMapSuffix << "].\n";
    return false;
  }
  if (!this->dataPtr->coloredMapPublisher)
  {
    ignerr << "Unable to create publisher on topic ["
      << this->Topic() + this->dataPtr->topicColoredMapSuffix << "].\n";
    return false;
  }

  // TODO(anyone) Access the info topic from the parent class
  if (!this->AdvertiseInfo(this->Topic() + "/camera_info"))
    return false;

  if (this->Scene())
  {
    if (!this->CreateCamera())
    {
      ignerr << "Unable to create segmentation camera sensor\n";
      return false;
    }
  }

  this->dataPtr->sceneChangeConnection =
    RenderingEvents::ConnectSceneChangeCallback(
    std::bind(&SegmentationCameraSensor::SetScene, this,
    std::placeholders::_1));

  this->dataPtr->initialized = true;

  return true;
}

/////////////////////////////////////////////////
void SegmentationCameraSensor::SetScene(
  ignition::rendering::ScenePtr _scene)
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

/////////////////////////////////////////////////
bool SegmentationCameraSensor::CreateCamera()
{
  const auto sdfCamera = this->dataPtr->sdfSensor.CameraSensor();
  if (!sdfCamera)
  {
    ignerr << "Unable to access camera SDF element\n";
    return false;
  }

  // Segmentation type
  if (sdfCamera->HasSegmentationType())
  {
    auto type = sdfCamera->SegmentationType();

    // convert type to lowercase
    std::for_each(type.begin(), type.end(), [](char & c){
      c = std::tolower(c);
    });

    if (type == "semantic")
      this->dataPtr->type = rendering::SegmentationType::SEMANTIC;
    else if (type == "instance" || type == "panoptic")
      this->dataPtr->type = rendering::SegmentationType::PANOPTIC;
    else
    {
      igndbg << "Wrong type [" << type <<
        "], type should be semantic or instance or panoptic" << std::endl;
      return false;
    }
  }

  // Camera Info Msg
  this->PopulateInfo(sdfCamera);

  if (!this->dataPtr->camera)
  {
    // Create rendering camera
    this->dataPtr->camera = this->Scene()->CreateSegmentationCamera(
      this->Name());
  }

  // Segmentation properties
  this->dataPtr->camera->SetSegmentationType(this->dataPtr->type);
  // Must be true to generate the colored map first then convert it
  this->dataPtr->camera->EnableColoredMap(true);

  auto width = sdfCamera->ImageWidth();
  auto height = sdfCamera->ImageHeight();

  // Set Camera Properties
  this->dataPtr->camera->SetImageWidth(width);
  this->dataPtr->camera->SetImageHeight(height);
  this->dataPtr->camera->SetVisibilityMask(sdfCamera->VisibilityMask());
  this->dataPtr->camera->SetNearClipPlane(sdfCamera->NearClip());
  this->dataPtr->camera->SetFarClipPlane(sdfCamera->FarClip());
  math::Angle angle = sdfCamera->HorizontalFov();
  if (angle < 0.01 || angle > IGN_PI*2)
  {
    ignerr << "Invalid horizontal field of view [" << angle << "]\n";
    return false;
  }
  this->dataPtr->camera->SetAspectRatio(static_cast<double>(width)/height);
  this->dataPtr->camera->SetHFOV(angle);

  // Add the camera to the scene
  this->Scene()->RootVisual()->AddChild(this->dataPtr->camera);

  // Add the rendering sensor to handle its render
  this->AddSensor(this->dataPtr->camera);

  // Connection to receive the segmentation buffer
  this->dataPtr->newSegmentationConnection =
      this->dataPtr->camera->ConnectNewSegmentationFrame(
      std::bind(&SegmentationCameraSensor::OnNewSegmentationFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  // Create the directory to store frames
  if (sdfCamera->SaveFrames())
  {
    this->dataPtr->saveImagePath = sdfCamera->SaveFramesPath();
    this->dataPtr->saveImagePrefix = this->Name() + "_";
    this->dataPtr->saveImage = true;
  }

  return true;
}

/////////////////////////////////////////////////
rendering::SegmentationCameraPtr SegmentationCameraSensor::SegmentationCamera()
  const
{
  return this->dataPtr->camera;
}

/////////////////////////////////////////////////
void SegmentationCameraSensor::OnNewSegmentationFrame(const uint8_t * _data,
  unsigned int _width, unsigned int _height, unsigned int _channles,
  const std::string &/*_format*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  unsigned int bufferSize = _width * _height * _channles;

  if (!this->dataPtr->segmentationColoredBuffer)
    this->dataPtr->segmentationColoredBuffer = new uint8_t[bufferSize];

  if (!this->dataPtr->segmentationLabelsBuffer)
    this->dataPtr->segmentationLabelsBuffer = new uint8_t[bufferSize];

  memcpy(this->dataPtr->segmentationColoredBuffer, _data, bufferSize);

  // Convert the colored map to labels map
  this->dataPtr->camera->LabelMapFromColoredBuffer(
    this->dataPtr->segmentationLabelsBuffer);
}

//////////////////////////////////////////////////
bool SegmentationCameraSensor::Update(
  const std::chrono::steady_clock::duration &_now)
{
  IGN_PROFILE("SegmentationCameraSensor::Update");
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

  // don't render if there are no subscribers
  if (!this->dataPtr->coloredMapPublisher.HasConnections() &&
    !this->dataPtr->labelsMapPublisher.HasConnections() &&
    this->dataPtr->imageEvent.ConnectionCount() <= 0u)
  {
    return false;
  }

  // Actual render
  this->Render();

  if (!this->dataPtr->segmentationColoredBuffer ||
    !this->dataPtr->segmentationLabelsBuffer)
    return false;

  auto width = this->dataPtr->camera->ImageWidth();
  auto height = this->dataPtr->camera->ImageHeight();

  // create colored map message
  this->dataPtr->coloredMapMsg.set_width(width);
  this->dataPtr->coloredMapMsg.set_height(height);
  // format
  this->dataPtr->coloredMapMsg.set_step(
    width * rendering::PixelUtil::BytesPerPixel(rendering::PF_R8G8B8));
  this->dataPtr->coloredMapMsg.set_pixel_format_type(
    msgs::PixelFormatType::RGB_INT8);
  // time stamp
  auto stamp = this->dataPtr->coloredMapMsg.mutable_header()->mutable_stamp();
  *stamp = msgs::Convert(_now);
  auto frame = this->dataPtr->coloredMapMsg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  this->dataPtr->labelsMapMsg.CopyFrom(this->dataPtr->coloredMapMsg);

  // Protect the data being modified by the segmentation buffers
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // segmentation colored map data
  this->dataPtr->coloredMapMsg.set_data(
    this->dataPtr->segmentationColoredBuffer,
    rendering::PixelUtil::MemorySize(rendering::PF_R8G8B8,
    width, height));

  // segmentation labels map data
  this->dataPtr->labelsMapMsg.set_data(this->dataPtr->segmentationLabelsBuffer,
      rendering::PixelUtil::MemorySize(rendering::PF_R8G8B8,
      width, height));

  // Publish
  this->PublishInfo(_now);
  this->dataPtr->coloredMapPublisher.Publish(this->dataPtr->coloredMapMsg);
  this->dataPtr->labelsMapPublisher.Publish(this->dataPtr->labelsMapMsg);

  // Trigger callbacks.
  if (this->dataPtr->imageEvent.ConnectionCount() > 0u)
  {
    try
    {
      this->dataPtr->imageEvent(this->dataPtr->coloredMapMsg);
    }
    catch(...)
    {
      ignerr << "Exception thrown in an image callback.\n";
    }
  }

  // Save image
  if (this->dataPtr->saveImage)
  {
    this->dataPtr->SaveImage(this->dataPtr->segmentationColoredBuffer,
      this->dataPtr->segmentationLabelsBuffer, width, height);
  }

  return true;
}

/////////////////////////////////////////////////
unsigned int SegmentationCameraSensor::ImageHeight() const
{
  if (!this->dataPtr->camera)
    return 0u;
  return this->dataPtr->camera->ImageHeight();
}

/////////////////////////////////////////////////
unsigned int SegmentationCameraSensor::ImageWidth() const
{
  if (!this->dataPtr->camera)
    return 0u;
  return this->dataPtr->camera->ImageWidth();
}

/////////////////////////////////////////////////
common::ConnectionPtr SegmentationCameraSensor::ConnectImageCallback(
    std::function<void(const msgs::Image &)> _callback)
{
  return this->dataPtr->imageEvent.Connect(_callback);
}

//////////////////////////////////////////////////
bool SegmentationCameraSensorPrivate::SaveImage(
  const uint8_t *_coloredBuffer, const uint8_t *_labelsBuffer,
  unsigned int _width, unsigned int _height)
{
  // Attempt to create the directory if it doesn't exist
  if (!ignition::common::isDirectory(this->saveImagePath))
  {
    if (!ignition::common::createDirectories(this->saveImagePath))
      return false;
  }

  std::string coloredName = this->saveImagePrefix + "colored_" +
                         std::to_string(this->saveImageCounter) + ".png";
  std::string labelsName = this->saveImagePrefix + "labels_" +
                         std::to_string(this->saveImageCounter) + ".png";

  ++this->saveImageCounter;

  // save colored map
  ignition::common::Image localColoredImage;
  localColoredImage.SetFromData(_coloredBuffer, _width, _height,
    ignition::common::Image::RGB_INT8);

  localColoredImage.SavePNG(
      ignition::common::joinPaths(this->saveImagePath, coloredName));

  // save labels map
  ignition::common::Image localLabelsImage;
  localLabelsImage.SetFromData(_labelsBuffer, _width, _height,
    ignition::common::Image::RGB_INT8);

  localLabelsImage.SavePNG(
      ignition::common::joinPaths(this->saveImagePath, labelsName));

  return true;
}
