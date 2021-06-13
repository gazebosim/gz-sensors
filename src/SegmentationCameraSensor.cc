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

#include <mutex>
#include <memory>

#include "ignition/common/Console.hh"
#include "ignition/common/Profiler.hh"
#include "ignition/common/Image.hh"
#include "ignition/sensors/RenderingEvents.hh"
#include "ignition/sensors/SensorFactory.hh"

#include "ignition/sensors/SegmentationCameraSensor.hh"
#include "ignition/rendering/SegmentationCamera.hh"

#include "ignition/transport/Node.hh"
#include "ignition/transport/Publisher.hh"
#include "ignition/msgs.hh"


using namespace ignition;
using namespace sensors;

class ignition::sensors::SegmentationCameraSensorPrivate
{
  /// \brief Save an image
  /// \param[in] _data the image data to be saved
  /// \param[in] _width width of image in pixels
  /// \param[in] _height height of image in pixels
  /// \return True if the image was saved successfully. False can mean
  /// that the path provided to the constructor does exist and creation
  /// of the path was not possible.
  /// \sa ImageSaver
  public: bool SaveImage(const uint8_t *_data, unsigned int _width,
    unsigned int _height);

  /// \brief SDF Sensor DOM Object
  public: sdf::Sensor sdfSensor;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief rendering Segmentation Camera
  public: rendering::SegmentationCameraPtr camera {nullptr};

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish segmentation image
  public: transport::Node::Publisher publisher;

  /// \brief Segmentation Image Msg
  public: msgs::Image segmentationMsg;

  /// \brief Topic to publish the segmentation image
  public: std::string topicSegmentation = "";

  /// \brief Buffer contains the segmentation map data
  public: uint8_t *segmentationBuffer {nullptr};

  /// \brief Segmentation type (Semantic / Instance)
  public: SegmentationType type {SegmentationType::Semantic};

  /// \brief True if camera generates a colored map image
  /// False if camera generates labels ids image
  public: bool isColoredMap {false};

  /// \brief Connection to the new segmentation frames data
  public: common::ConnectionPtr newSegmentationConnection;

  /// \brief Connection to the Manager's scene change event.
  public: common::ConnectionPtr sceneChangeConnection;

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
  if (this->dataPtr->segmentationBuffer)
    delete this->dataPtr->segmentationBuffer;
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

  // Segmentation Type & Colored Properties
  sdf::ElementPtr sdfElement = _sdf.Element();
  if (sdfElement->HasElement("segmentation_type"))
  {
    std::string type = sdfElement->Get<std::string>("segmentation_type");

    // convert type to lowercase
    std::for_each(type.begin(), type.end(), [](char & c){
      c = std::tolower(c);
    });

    if (type == "semantic")
      this->dataPtr->type = SegmentationType::Semantic;
    else if (type == "instance" || type == "panoptic")
      this->dataPtr->type = SegmentationType::Panoptic;
  }

  if (sdfElement->HasElement("colored"))
  {
    this->dataPtr->isColoredMap = sdfElement->Get<bool>("colored");
  }

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

  // Create the thermal image publisher
  this->dataPtr->publisher =
      this->dataPtr->node.Advertise<ignition::msgs::Image>(
          this->Topic());

  if (!this->dataPtr->publisher)
  {
    ignerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  if (!this->AdvertiseInfo())
    return false;

  if (this->Scene())
  {
    this->CreateCamera();
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
  auto sdfCamera = this->dataPtr->sdfSensor.CameraSensor();
  if (!sdfCamera)
  {
    ignerr << "Unable to access camera SDF element\n";
    return false;
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
  this->dataPtr->camera->EnableColoredMap(this->dataPtr->isColoredMap);

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
  // this->dataPtr->camera->SetLocalPose(sdfCamera->RawPose());
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

  if (!this->dataPtr->segmentationBuffer)
    this->dataPtr->segmentationBuffer = new uint8_t[bufferSize];

  memcpy(this->dataPtr->segmentationBuffer, _data, bufferSize);
}

//////////////////////////////////////////////////
bool SegmentationCameraSensor::Update(const ignition::common::Time &_now)
{
  return this->Update(math::secNsecToDuration(_now.sec, _now.nsec));
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

  // don't render if there is no subscribers
  if (!this->dataPtr->publisher.HasConnections() &&
    this->dataPtr->imageEvent.ConnectionCount() <= 0 &&
    !this->dataPtr->saveImage)
  {
    return false;
  }

  // Actual render
  this->Render();

  if (!this->dataPtr->segmentationBuffer)
    return false;


  auto width = this->dataPtr->camera->ImageWidth();
  auto height = this->dataPtr->camera->ImageHeight();

  // create message
  this->dataPtr->segmentationMsg.set_width(width);
  this->dataPtr->segmentationMsg.set_height(height);
  // format
  this->dataPtr->segmentationMsg.set_step(
    width * rendering::PixelUtil::BytesPerPixel(rendering::PF_R8G8B8));
  this->dataPtr->segmentationMsg.set_pixel_format_type(
    msgs::PixelFormatType::RGB_INT8);
  // time stamp
  auto stamp = this->dataPtr->segmentationMsg.mutable_header()->mutable_stamp();
  *stamp = msgs::Convert(_now);
  auto frame = this->dataPtr->segmentationMsg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());
  // segmentation data
  this->dataPtr->segmentationMsg.set_data(this->dataPtr->segmentationBuffer,
      rendering::PixelUtil::MemorySize(rendering::PF_R8G8B8,
      width, height));

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Publish
  this->PublishInfo(_now);
  this->dataPtr->publisher.Publish(this->dataPtr->segmentationMsg);

  // Trigger callbacks.
  if (this->dataPtr->imageEvent.ConnectionCount() > 0)
  {
    try
    {
      this->dataPtr->imageEvent(this->dataPtr->segmentationMsg);
    }
    catch(...)
    {
      ignerr << "Exception thrown in an image callback.\n";
    }
  }

  // Save image
  if (this->dataPtr->saveImage)
  {
    this->dataPtr->SaveImage(this->dataPtr->segmentationBuffer, width, height);
  }

  return true;
}

/////////////////////////////////////////////////
unsigned int SegmentationCameraSensor::ImageHeight() const
{
  return this->dataPtr->camera->ImageHeight();
}

/////////////////////////////////////////////////
unsigned int SegmentationCameraSensor::ImageWidth() const
{
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
  const uint8_t *_data, unsigned int _width, unsigned int _height)
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
  localImage.SetFromData(_data, _width, _height,
    ignition::common::Image::RGB_INT8);

  localImage.SavePNG(
      ignition::common::joinPaths(this->saveImagePath, filename));
  return true;
}

IGN_SENSORS_REGISTER_SENSOR(SegmentationCameraSensor)
