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

#include <gz/msgs/image.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Image.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/Utility.hh>
#include <gz/rendering/SegmentationCamera.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

#include "gz/sensors/RenderingEvents.hh"
#include "gz/sensors/SegmentationCameraSensor.hh"
#include "gz/sensors/SensorFactory.hh"

using namespace gz;
using namespace sensors;

class gz::sensors::SegmentationCameraSensorPrivate
{
  /// \brief Save a sample for the dataset (image & colored map & labels map)
  /// \return True if the image was saved successfully. False can mean
  /// that the image save path does not exist and creation
  /// of the path was not possible.
  public: bool SaveSample();

  /// \brief SDF Sensor DOM Object
  public: sdf::Sensor sdfSensor;

  /// \brief True if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Rendering Segmentation Camera
  public: rendering::SegmentationCameraPtr camera {nullptr};

  /// \brief Rendering RGB Camera to save rgb images for dataset generation
  public: rendering::CameraPtr rgbCamera {nullptr};

  /// \brief RGB Image to load the rgb camera data
  public: rendering::Image image;

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

  /// \brief Buffer contains the image data to be saved
  public: unsigned char *saveImageBuffer {nullptr};

  /// \brief Segmentation type (Semantic / Instance)
  public: rendering::SegmentationType type
    {rendering::SegmentationType::ST_SEMANTIC};

  /// \brief Connection to the new segmentation frames data
  public: common::ConnectionPtr newSegmentationConnection {nullptr};

  /// \brief Connection to the Manager's scene change event.
  public: common::ConnectionPtr sceneChangeConnection {nullptr};

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief True to save samples
  public: bool saveSamples = false;

  /// \brief Folder to save the image
  public: std::string saveImageFolder = "/images";

  /// \brief Folder to save the segmentation colored maps
  public: std::string saveColoredMapsFolder = "/colored_maps";

  /// \brief Folder to save the segmentation labels maps
  public: std::string saveLabelsMapsFolder = "/labels_maps";

  /// \brief Path directory to where images are saved
  public: std::string savePath = "./";

  /// \brief Prefix of an image name
  public: std::string saveImagePrefix = "./";

  /// \brief Counter used to set the image filename
  public: std::uint64_t saveCounter = 0;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: gz::common::EventT<
          void(const gz::msgs::Image &)> imageEvent;
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
  {
    delete [] this->dataPtr->segmentationColoredBuffer;
    this->dataPtr->segmentationColoredBuffer = nullptr;
  }

  if (this->dataPtr->segmentationLabelsBuffer)
  {
    delete [] this->dataPtr->segmentationLabelsBuffer;
    this->dataPtr->segmentationLabelsBuffer = nullptr;
  }
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
    gzerr << "Attempting to a load a Segmentation Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    gzerr << "Attempting to a load a Segmentation Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  // Create the segmentation colored map image publisher
  this->dataPtr->coloredMapPublisher =
      this->dataPtr->node.Advertise<gz::msgs::Image>(
          this->Topic() + this->dataPtr->topicColoredMapSuffix);

  if (!this->dataPtr->coloredMapPublisher)
  {
    gzerr << "Unable to create publisher on topic ["
      << this->Topic() << this->dataPtr->topicColoredMapSuffix << "].\n";
    return false;
  }

  gzdbg << "Colored map image for [" << this->Name() << "] advertised on ["
    << this->Topic() << this->dataPtr->topicColoredMapSuffix << "]\n";

  // Create the segmentation labels map image publisher
  this->dataPtr->labelsMapPublisher =
      this->dataPtr->node.Advertise<gz::msgs::Image>(
          this->Topic() + this->dataPtr->topicLabelsMapSuffix);

  if (!this->dataPtr->labelsMapPublisher)
  {
    gzerr << "Unable to create publisher on topic ["
      << this->Topic() << this->dataPtr->topicLabelsMapSuffix << "].\n";
    return false;
  }

  gzdbg << "Segmentation labels map image for [" << this->Name()
    << "] advertised on [" << this->Topic()
    << this->dataPtr->topicLabelsMapSuffix << "]\n";

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

  // TODO(anyone) Access the info topic from the parent class
  if (!this->AdvertiseInfo(this->Topic() + "/camera_info"))
    return false;

  if (this->Scene())
  {
    if (!this->CreateCamera())
    {
      gzerr << "Unable to create segmentation camera sensor\n";
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
  gz::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->Scene() != _scene)
  {
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
    gzerr << "Unable to access camera SDF element\n";
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
      this->dataPtr->type = rendering::SegmentationType::ST_SEMANTIC;
    else if (type == "instance" || type == "panoptic")
      this->dataPtr->type = rendering::SegmentationType::ST_PANOPTIC;
    else
    {
      gzdbg << "Wrong type [" << type <<
        "], type should be semantic or instance or panoptic" << std::endl;
      return false;
    }
  }

  // Save frames properties
  if (sdfCamera->SaveFrames())
  {
    this->dataPtr->savePath = sdfCamera->SaveFramesPath();
    this->dataPtr->saveImagePrefix = this->Name() + "_";
    this->dataPtr->saveSamples = true;

    // Folders paths
    this->dataPtr->saveImageFolder =
      this->dataPtr->savePath + this->dataPtr->saveImageFolder;
    this->dataPtr->saveColoredMapsFolder =
      this->dataPtr->savePath + this->dataPtr->saveColoredMapsFolder;
    this->dataPtr->saveLabelsMapsFolder =
      this->dataPtr->savePath + this->dataPtr->saveLabelsMapsFolder;

    // Set the save counter to be equal number of images in the folder + 1
    // to continue adding to the images in the folder (multi scene datasets)
    if (gz::common::isDirectory(this->dataPtr->saveImageFolder))
    {
      common::DirIter endIter;
      for (common::DirIter dirIter(this->dataPtr->saveImageFolder);
        dirIter != endIter; ++dirIter)
      {
        this->dataPtr->saveCounter++;
      }
    }
  }

  if (!this->dataPtr->camera)
  {
    // Create rendering camera
    this->dataPtr->camera = this->Scene()->CreateSegmentationCamera(
      this->Name());

    if (this->dataPtr->saveSamples)
    {
      this->dataPtr->rgbCamera = this->Scene()->CreateCamera(
        this->Name() + "_rgbCamera");
    }
  }

  // Segmentation properties
  this->dataPtr->camera->SetSegmentationType(this->dataPtr->type);
  // Must be true to generate the colored map first then convert it
  this->dataPtr->camera->EnableColoredMap(true);

  auto width = sdfCamera->ImageWidth();
  auto height = sdfCamera->ImageHeight();

  if (width == 0u || height == 0u)
  {
    gzerr << "Unable to create a segmentation camera sensor with 0 width or "
          << "height." << std::endl;
    return false;
  }

  math::Angle angle = sdfCamera->HorizontalFov();
  if (angle < 0.01 || angle > GZ_PI*2)
  {
    gzerr << "Invalid horizontal field of view [" << angle << "]\n";
    return false;
  }
  double aspectRatio = static_cast<double>(width)/height;

  // Set segmentation camera properties
  this->dataPtr->camera->SetImageWidth(width);
  this->dataPtr->camera->SetImageHeight(height);
  this->dataPtr->camera->SetVisibilityMask(sdfCamera->VisibilityMask());
  this->dataPtr->camera->SetNearClipPlane(sdfCamera->NearClip());
  this->dataPtr->camera->SetFarClipPlane(sdfCamera->FarClip());
  this->dataPtr->camera->SetAspectRatio(aspectRatio);
  this->dataPtr->camera->SetHFOV(angle);
  this->dataPtr->camera->SetLocalPose(this->Pose());

  // Add the camera to the scene
  this->Scene()->RootVisual()->AddChild(this->dataPtr->camera);

  // Add the rendering sensor to handle its render
  this->AddSensor(this->dataPtr->camera);

  this->UpdateLensIntrinsicsAndProjection(this->dataPtr->camera,
      *sdfCamera);

  // Add the rgb camera only if we want to generate dataset / save samples
  if (this->dataPtr->saveSamples)
  {
    // Set rgb camera properties
    this->dataPtr->rgbCamera->SetImageFormat(rendering::PF_R8G8B8);
    this->dataPtr->rgbCamera->SetImageWidth(width);
    this->dataPtr->rgbCamera->SetImageHeight(height);
    this->dataPtr->rgbCamera->SetVisibilityMask(sdfCamera->VisibilityMask());
    this->dataPtr->rgbCamera->SetNearClipPlane(sdfCamera->NearClip());
    this->dataPtr->rgbCamera->SetFarClipPlane(sdfCamera->FarClip());
    this->dataPtr->rgbCamera->SetAspectRatio(aspectRatio);
    this->dataPtr->rgbCamera->SetHFOV(angle);

    // Add the rgb camera to the rendering pipeline
    this->Scene()->RootVisual()->AddChild(this->dataPtr->rgbCamera);
    this->AddSensor(this->dataPtr->rgbCamera);
    this->dataPtr->image = this->dataPtr->rgbCamera->CreateImage();

    this->UpdateLensIntrinsicsAndProjection(this->dataPtr->rgbCamera,
      *sdfCamera);
  }

  // Camera Info Msg
  this->PopulateInfo(sdfCamera);

  // Connection to receive the segmentation buffer
  this->dataPtr->newSegmentationConnection =
      this->dataPtr->camera->ConnectNewSegmentationFrame(
      std::bind(&SegmentationCameraSensor::OnNewSegmentationFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

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
  unsigned int _width, unsigned int _height, unsigned int _channels,
  const std::string &/*_format*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  unsigned int bufferSize = _width * _height * _channels;

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
  GZ_PROFILE("SegmentationCameraSensor::Update");
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

  if (this->HasInfoConnections())
  {
    // publish the camera info message
    this->PublishInfo(_now);
  }

  // don't render if there are no subscribers nor saving
  if (!this->dataPtr->coloredMapPublisher.HasConnections() &&
    !this->dataPtr->labelsMapPublisher.HasConnections() &&
    !this->dataPtr->saveSamples)
  {
    return false;
  }

  if (this->dataPtr->saveSamples)
  {
    // The sensor updates only the segmentation camera with its pose
    // as it has the same name, so make rgb camera with the same pose
    this->dataPtr->rgbCamera->SetWorldPose(
      this->dataPtr->camera->WorldPose());
  }

  // Actual render
  this->Render();

  if (this->dataPtr->saveSamples)
  {
    // Copy the rgb camera image data
    this->dataPtr->rgbCamera->Copy(this->dataPtr->image);
    this->dataPtr->saveImageBuffer = this->dataPtr->image.Data<unsigned char>();
  }

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
  frame->add_value(this->FrameId());

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
      gzerr << "Exception thrown in an image callback.\n";
    }
  }

  // Save a sample (image & colored map & labels map)
  if (this->dataPtr->saveSamples)
    this->dataPtr->SaveSample();

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
bool SegmentationCameraSensor::HasConnections() const
{
  return (this->dataPtr->coloredMapPublisher &&
      this->dataPtr->coloredMapPublisher.HasConnections()) ||
      (this->dataPtr->labelsMapPublisher &&
      this->dataPtr->labelsMapPublisher.HasConnections()) ||
      this->dataPtr->imageEvent.ConnectionCount() > 0u ||
      this->HasInfoConnections();
}

//////////////////////////////////////////////////
bool SegmentationCameraSensorPrivate::SaveSample()
{
  // Attempt to create the directories if they don't exist
  if (!gz::common::isDirectory(this->savePath))
  {
    if (!gz::common::createDirectories(this->savePath))
      return false;
  }
  if (!gz::common::isDirectory(this->saveImageFolder))
  {
    if (!gz::common::createDirectories(this->saveImageFolder))
      return false;
  }
  if (!gz::common::isDirectory(this->saveColoredMapsFolder))
  {
    if (!gz::common::createDirectories(this->saveColoredMapsFolder))
      return false;
  }
  if (!gz::common::isDirectory(this->saveLabelsMapsFolder))
  {
    if (!gz::common::createDirectories(this->saveLabelsMapsFolder))
      return false;
  }

  auto width = this->camera->ImageWidth();
  auto height = this->camera->ImageHeight();

  // Save the images in format of 0000001, 0000002 .. etc
  // Useful in sorting them in python
  std::stringstream ss;
  ss << std::setw(7) << std::setfill('0') << this->saveCounter;
  std::string saveCounterString = ss.str();

  std::string coloredName = "colored_" + saveCounterString + ".png";
  std::string labelsName = "labels_" + saveCounterString + ".png";
  std::string rgbImageName = "image_" + saveCounterString + ".png";

  // Save rgb image
  gz::common::Image rgbImage;
  rgbImage.SetFromData(this->saveImageBuffer,
    width, height, gz::common::Image::RGB_INT8);

  rgbImage.SavePNG(
      gz::common::joinPaths(this->saveImageFolder, rgbImageName));

  // Save colored map
  gz::common::Image localColoredImage;
  localColoredImage.SetFromData(this->segmentationColoredBuffer,
    width, height, gz::common::Image::RGB_INT8);

  localColoredImage.SavePNG(
      gz::common::joinPaths(this->saveColoredMapsFolder, coloredName));

  // Save labels map
  gz::common::Image localLabelsImage;
  localLabelsImage.SetFromData(this->segmentationLabelsBuffer,
    width, height, gz::common::Image::RGB_INT8);

  localLabelsImage.SavePNG(
      gz::common::joinPaths(this->saveLabelsMapsFolder, labelsName));

  ++this->saveCounter;
  return true;
}
