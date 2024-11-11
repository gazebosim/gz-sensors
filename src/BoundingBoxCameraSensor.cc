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
#include <ostream>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/annotated_axis_aligned_2d_box.pb.h>
#include <gz/msgs/annotated_axis_aligned_2d_box_v.pb.h>
#include <gz/msgs/annotated_oriented_3d_box.pb.h>
#include <gz/msgs/annotated_oriented_3d_box_v.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Image.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/Utility.hh>
#include <gz/rendering/BoundingBoxCamera.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>
#include <gz/transport/TopicUtils.hh>

#include "gz/sensors/BoundingBoxCameraSensor.hh"
#include "gz/sensors/RenderingEvents.hh"
#include "gz/sensors/SensorFactory.hh"

using namespace gz;
using namespace sensors;

class gz::sensors::BoundingBoxCameraSensorPrivate
{
  /// \brief Save an image of rgb camera
  public: void SaveImage();

  /// \brief Save the bounding boxes
  public: void SaveBoxes();

  /// \brief SDF Sensor DOM Object
  public: sdf::Sensor sdfSensor;

  /// \brief True if Load() has been called and was successful
  public: bool initialized{false};

  /// \brief Rendering BoundingBox Camera
  public: rendering::BoundingBoxCameraPtr boundingboxCamera{nullptr};

  /// \brief Rendering RGB Camera to draw boxes on it and publish
  /// its image (just for visualization)
  public: rendering::CameraPtr rgbCamera{nullptr};

  /// \brief Node to create publisher
  public: transport::Node node;

  /// \brief Publisher to publish Image msg with drawn boxes
  public: transport::Node::Publisher imagePublisher;

  /// \brief Publisher to publish BoundingBoxes msg
  public: transport::Node::Publisher boxesPublisher;

  /// \brief Vector to receive boxes from the rendering camera
  public: std::vector<rendering::BoundingBox> boundingBoxes;

  /// \brief RGB Image to draw boxes on it
  public: rendering::Image image;

  /// \brief Buffer contains the image data to be saved
  public: unsigned char *saveImageBuffer{nullptr};

  /// \brief Connection to the new BoundingBox frames data
  public: common::ConnectionPtr newBoundingBoxConnection;

  /// \brief Connection to the Manager's scene change event.
  public: common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief BoundingBoxes type
  public: rendering::BoundingBoxType type
    {rendering::BoundingBoxType::BBT_VISIBLEBOX2D};

  /// \brief True to save images & boxes
  public: bool saveSample{false};

  /// \brief path directory to where images & boxes are saved
  public: std::string savePath{"./"};

  /// \brief Folder to save the image
  public: std::string saveImageFolder{"/images"};

  /// \brief Folder to save the bounding boxes
  public: std::string saveBoxesFolder{"/boxes"};

  /// \brief counter used to set the sample filename
  public: std::uint64_t saveCounter{0};
};

//////////////////////////////////////////////////
BoundingBoxCameraSensor::BoundingBoxCameraSensor()
  : CameraSensor(), dataPtr(std::make_unique<BoundingBoxCameraSensorPrivate>())
{
}

/////////////////////////////////////////////////
BoundingBoxCameraSensor::~BoundingBoxCameraSensor()
{
}

/////////////////////////////////////////////////
bool BoundingBoxCameraSensor::Init()
{
  return CameraSensor::Init();
}

/////////////////////////////////////////////////
bool BoundingBoxCameraSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

/////////////////////////////////////////////////
bool BoundingBoxCameraSensor::Load(const sdf::Sensor &_sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  auto sdfCamera = _sdf.CameraSensor();
  if (!sdfCamera)
    return false;

  // BoundingBox Type
  if (sdfCamera->HasBoundingBoxType())
  {
    std::string type = sdfCamera->BoundingBoxType();

    if (type == "full_2d" || type == "full_box_2d")
      this->dataPtr->type = rendering::BoundingBoxType::BBT_FULLBOX2D;
    else if (type == "2d" || type == "visible_2d"
      || type == "visible_box_2d")
      this->dataPtr->type = rendering::BoundingBoxType::BBT_VISIBLEBOX2D;
    else if (type == "3d")
      this->dataPtr->type = rendering::BoundingBoxType::BBT_BOX3D;
    else
    {
      gzerr << "Unknown bounding box type " << type << std::endl;
      return false;
    }
  }

  if (!Sensor::Load(_sdf))
  {
    return false;
  }

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::BOUNDINGBOX_CAMERA)
  {
    gzerr << "Attempting to a load a BoundingBox Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    gzerr << "Attempting to a load a BoundingBox Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  auto topicBoundingBoxes = this->Topic();
  auto topicImage = this->Topic() + "_image";

  this->dataPtr->imagePublisher =
    this->dataPtr->node.Advertise<msgs::Image>(topicImage);

  if (!this->dataPtr->imagePublisher)
  {
    gzerr << "Unable to create publisher on topic ["
      << topicImage << "].\n";
    return false;
  }

  gzdbg << "Camera images for [" << this->Name() << "] advertised on ["
    << topicImage << "]" << std::endl;

  if (this->dataPtr->type == rendering::BoundingBoxType::BBT_BOX3D)
  {
    this->dataPtr->boxesPublisher = this->dataPtr->node.Advertise<
      msgs::AnnotatedOriented3DBox_V>(topicBoundingBoxes);
  }
  else
  {
    this->dataPtr->boxesPublisher = this->dataPtr->node.Advertise<
      msgs::AnnotatedAxisAligned2DBox_V>(topicBoundingBoxes);
  }

  if (!this->dataPtr->boxesPublisher)
  {
    gzerr << "Unable to create publisher on topic ["
      << topicBoundingBoxes << "].\n";
    return false;
  }

  gzdbg << "Bounding boxes for [" << this->Name() << "] advertised on ["
    << topicBoundingBoxes << std::endl;

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
    std::bind(&BoundingBoxCameraSensor::SetScene, this,
    std::placeholders::_1));

  this->dataPtr->initialized = true;

  return true;
}

/////////////////////////////////////////////////
void BoundingBoxCameraSensor::SetScene(
  rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    this->dataPtr->boundingboxCamera = nullptr;
    this->dataPtr->rgbCamera = nullptr;
    RenderingSensor::SetScene(_scene);

    if (this->dataPtr->initialized)
      this->CreateCamera();
  }
}

/////////////////////////////////////////////////
bool BoundingBoxCameraSensor::CreateCamera()
{
  auto sdfCamera = this->dataPtr->sdfSensor.CameraSensor();
  if (!sdfCamera)
  {
    gzerr << "Unable to access camera SDF element\n";
    return false;
  }

  if (!this->dataPtr->rgbCamera)
  {
    // Create rendering camera
    this->dataPtr->boundingboxCamera =
      this->Scene()->CreateBoundingBoxCamera(this->Name());

    this->dataPtr->rgbCamera = this->Scene()->CreateCamera(
      this->Name() + "_rgbCamera");
  }

  auto width = sdfCamera->ImageWidth();
  auto height = sdfCamera->ImageHeight();

  if (width == 0u || height == 0u)
  {
    gzerr << "Unable to create a bounding box camera sensor with 0 width or "
          << "height. " << std::endl;
    return false;
  }

  // Set Camera Properties
  this->dataPtr->rgbCamera->SetImageFormat(rendering::PF_R8G8B8);
  this->dataPtr->rgbCamera->SetImageWidth(width);
  this->dataPtr->rgbCamera->SetImageHeight(height);
  this->dataPtr->rgbCamera->SetVisibilityMask(sdfCamera->VisibilityMask());
  this->dataPtr->rgbCamera->SetNearClipPlane(sdfCamera->NearClip());
  this->dataPtr->rgbCamera->SetFarClipPlane(sdfCamera->FarClip());
  math::Angle angle = sdfCamera->HorizontalFov();
  if (angle < 0.01 || angle > GZ_PI*2)
  {
    gzerr << "Invalid horizontal field of view [" << angle << "]\n";
    return false;
  }
  double aspectRatio = static_cast<double>(width)/height;
  this->dataPtr->rgbCamera->SetAspectRatio(aspectRatio);
  this->dataPtr->rgbCamera->SetHFOV(angle);

  this->dataPtr->boundingboxCamera->SetImageWidth(width);
  this->dataPtr->boundingboxCamera->SetImageHeight(height);
  this->dataPtr->boundingboxCamera->SetNearClipPlane(sdfCamera->NearClip());
  this->dataPtr->boundingboxCamera->SetFarClipPlane(sdfCamera->FarClip());
  this->dataPtr->boundingboxCamera->SetImageFormat(
    rendering::PixelFormat::PF_R8G8B8);
  this->dataPtr->boundingboxCamera->SetAspectRatio(aspectRatio);
  this->dataPtr->boundingboxCamera->SetHFOV(angle);
  this->dataPtr->boundingboxCamera->SetVisibilityMask(
    sdfCamera->VisibilityMask());
  this->dataPtr->boundingboxCamera->SetBoundingBoxType(this->dataPtr->type);
  this->dataPtr->boundingboxCamera->SetLocalPose(this->Pose());

  // Add the camera to the scene
  this->Scene()->RootVisual()->AddChild(this->dataPtr->rgbCamera);
  this->Scene()->RootVisual()->AddChild(this->dataPtr->boundingboxCamera);

  // Add the rendering sensors to handle its render
  this->AddSensor(this->dataPtr->boundingboxCamera);
  this->AddSensor(this->dataPtr->rgbCamera);

  // use a copy so we do not modify the original sdfCamera
  // when updating bounding box camera
  auto sdfCameraCopy = *sdfCamera;
  this->UpdateLensIntrinsicsAndProjection(this->dataPtr->rgbCamera,
      sdfCameraCopy);
  this->UpdateLensIntrinsicsAndProjection(this->dataPtr->boundingboxCamera,
      *sdfCamera);
  // Camera Info Msg
  this->PopulateInfo(sdfCamera);

  // Create the directory to store frames
  if (sdfCamera->SaveFrames())
  {
    this->dataPtr->savePath = sdfCamera->SaveFramesPath();
    this->dataPtr->saveImageFolder =
      this->dataPtr->savePath + this->dataPtr->saveImageFolder;
    this->dataPtr->saveBoxesFolder =
      this->dataPtr->savePath + this->dataPtr->saveBoxesFolder;
    this->dataPtr->saveSample = true;

    // Set the save counter to be equal number of images in the folder + 1
    // to continue adding to the images in the folder (multi scene datasets)
    if (common::isDirectory(this->dataPtr->saveImageFolder))
    {
      common::DirIter endIter;
      for (common::DirIter dirIter(this->dataPtr->saveImageFolder);
        dirIter != endIter; ++dirIter)
      {
        this->dataPtr->saveCounter++;
      }
    }
  }

  // Connection to receive the BoundingBox buffer
  this->dataPtr->newBoundingBoxConnection =
    this->dataPtr->boundingboxCamera->ConnectNewBoundingBoxes(
      std::bind(&BoundingBoxCameraSensor::OnNewBoundingBoxes, this,
        std::placeholders::_1));

  this->dataPtr->image = this->dataPtr->rgbCamera->CreateImage();

  return true;
}

/////////////////////////////////////////////////
rendering::BoundingBoxCameraPtr
  BoundingBoxCameraSensor::BoundingBoxCamera() const
{
  return this->dataPtr->boundingboxCamera;
}

/////////////////////////////////////////////////
void BoundingBoxCameraSensor::OnNewBoundingBoxes(
  const std::vector<rendering::BoundingBox> &_boxes)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->boundingBoxes.clear();
  for (const auto &box : _boxes)
    this->dataPtr->boundingBoxes.push_back(box);
}

//////////////////////////////////////////////////
bool BoundingBoxCameraSensor::Update(
  const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("BoundingBoxCameraSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->boundingboxCamera || !this->dataPtr->rgbCamera)
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
  if (!this->dataPtr->imagePublisher.HasConnections() &&
    !this->dataPtr->boxesPublisher.HasConnections() &&
    !this->dataPtr->saveSample)
  {
    return false;
  }

  // The sensor updates only the bounding box camera with its pose
  // as it has the same name, so make rgb camera with the same pose
  this->dataPtr->rgbCamera->SetWorldPose(
    this->dataPtr->boundingboxCamera->WorldPose());

  // Render the bounding box camera
  this->Render();

  // Render the rgb camera
  this->dataPtr->rgbCamera->Copy(this->dataPtr->image);

  auto imageBuffer = this->dataPtr->image.Data<unsigned char>();

  if (this->dataPtr->saveSample)
  {
    auto bufferSize = this->dataPtr->image.MemorySize();
    if (!this->dataPtr->saveImageBuffer)
      this->dataPtr->saveImageBuffer = new uint8_t[bufferSize];

    memcpy(this->dataPtr->saveImageBuffer, imageBuffer,
      bufferSize);
  }

  // Draw bounding boxes
  for (const auto &box : this->dataPtr->boundingBoxes)
  {
    this->dataPtr->boundingboxCamera->DrawBoundingBox(
      imageBuffer, math::Color::Green, box);
  }

  auto width = this->dataPtr->rgbCamera->ImageWidth();
  auto height = this->dataPtr->rgbCamera->ImageHeight();

  // Create Image message
  msgs::Image imageMsg;
  imageMsg.set_width(width);
  imageMsg.set_height(height);
  // Format
  imageMsg.set_step(
    width * rendering::PixelUtil::BytesPerPixel(rendering::PF_R8G8B8));
  imageMsg.set_pixel_format_type(
    msgs::PixelFormatType::RGB_INT8);
  // Time stamp
  auto stamp = imageMsg.mutable_header()->mutable_stamp();
  *stamp = msgs::Convert(_now);
  auto frame = imageMsg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());
  // Image data
  imageMsg.set_data(imageBuffer,
      rendering::PixelUtil::MemorySize(rendering::PF_R8G8B8,
      width, height));

  // Publish
  this->AddSequence(imageMsg.mutable_header(), "rgbImage");
  this->dataPtr->imagePublisher.Publish(imageMsg);

  msgs::AnnotatedAxisAligned2DBox_V boxes2DMsg;
  msgs::AnnotatedOriented3DBox_V boxes3DMsg;

  if (this->dataPtr->type == rendering::BoundingBoxType::BBT_BOX3D)
  {
    // Create 3D boxes message
    for (const auto &box : this->dataPtr->boundingBoxes)
    {
      // box data
      auto annotatedBox = boxes3DMsg.add_annotated_box();
      annotatedBox->set_label(box.Label());

      auto oriented3DBox = annotatedBox->mutable_box();
      msgs::Set(oriented3DBox->mutable_center(), box.Center());
      msgs::Set(oriented3DBox->mutable_boxsize(), box.Size());
      msgs::Set(oriented3DBox->mutable_orientation(), box.Orientation());
    }
    // time stamp
    auto stampBoxes =
      boxes3DMsg.mutable_header()->mutable_stamp();
    *stampBoxes = msgs::Convert(_now);
    auto frameBoxes = boxes3DMsg.mutable_header()->add_data();
    frameBoxes->set_key("frame_id");
    frameBoxes->add_value(this->Name());
  }
  else
  {
    // Create 2D boxes message
    for (const auto &box : this->dataPtr->boundingBoxes)
    {
      // box data
      auto annotatedBox = boxes2DMsg.add_annotated_box();
      annotatedBox->set_label(box.Label());

      auto minCorner = box.Center() - box.Size() * 0.5;
      auto maxCorner = box.Center() + box.Size() * 0.5;

      auto axisAlignedBox = annotatedBox->mutable_box();
      msgs::Set(axisAlignedBox->mutable_min_corner(),
          {minCorner.X(), minCorner.Y()});
      msgs::Set(axisAlignedBox->mutable_max_corner(),
          {maxCorner.X(), maxCorner.Y()});
    }
    // time stamp
    auto stampBoxes = boxes2DMsg.mutable_header()->mutable_stamp();
    *stampBoxes = msgs::Convert(_now);
    auto frameBoxes = boxes2DMsg.mutable_header()->add_data();
    frameBoxes->set_key("frame_id");
    frameBoxes->add_value(this->Name());
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Publish
  if (this->dataPtr->type == rendering::BoundingBoxType::BBT_BOX3D)
  {
    this->AddSequence(boxes3DMsg.mutable_header(), "boundingboxes");
    this->dataPtr->boxesPublisher.Publish(boxes3DMsg);
  }
  else
  {
    this->AddSequence(boxes2DMsg.mutable_header(), "boundingboxes");
    this->dataPtr->boxesPublisher.Publish(boxes2DMsg);
  }

  // Save a sample (image & its bounding boxes)
  if (this->dataPtr->saveSample)
  {
    this->dataPtr->SaveImage();
    this->dataPtr->SaveBoxes();
    ++this->dataPtr->saveCounter;
  }

  return true;
}

/////////////////////////////////////////////////
unsigned int BoundingBoxCameraSensor::ImageHeight() const
{
  return this->dataPtr->rgbCamera->ImageHeight();
}

/////////////////////////////////////////////////
unsigned int BoundingBoxCameraSensor::ImageWidth() const
{
  return this->dataPtr->rgbCamera->ImageWidth();
}

//////////////////////////////////////////////////
void BoundingBoxCameraSensorPrivate::SaveImage()
{
  // Attempt to create the save directory if it doesn't exist
  if (!common::isDirectory(this->savePath))
  {
    if (!common::createDirectories(this->savePath))
    {
      gzerr << "Failed to create directory [" << this->savePath << "]"
             << std::endl;
      return;
    }
  }
  // Attempt to create the image directory if it doesn't exist
  if (!common::isDirectory(this->saveImageFolder))
  {
    if (!common::createDirectories(this->saveImageFolder))
    {
      gzerr << "Failed to create directory [" << this->saveImageFolder << "]"
             << std::endl;
      return;
    }
  }

  auto width = this->rgbCamera->ImageWidth();
  auto height = this->rgbCamera->ImageHeight();
  if (width == 0 || height == 0)
    return;

  common::Image localImage;

  // Save the images in format of 0000001, 0000002 .. etc
  // Useful in sorting them in python
  std::stringstream ss;
  ss << std::setw(7) << std::setfill('0') << this->saveCounter;
  std::string saveCounterString = ss.str();

  std::string filename = "image_" + saveCounterString + ".png";

  localImage.SetFromData(this->saveImageBuffer, width, height,
      common::Image::RGB_INT8);
  localImage.SavePNG(
      common::joinPaths(this->saveImageFolder, filename));
}

//////////////////////////////////////////////////
void BoundingBoxCameraSensorPrivate::SaveBoxes()
{
  // Attempt to create the save directory if it doesn't exist
  if (!common::isDirectory(this->savePath))
  {
    if (!common::createDirectories(this->savePath))
    {
      gzerr << "Failed to create directory [" << this->savePath << "]"
             << std::endl;
      return;
    }
  }
  // Attempt to create the boxes directory if it doesn't exist
  if (!common::isDirectory(this->saveBoxesFolder))
  {
    if (!common::createDirectories(this->saveBoxesFolder))
    {
      gzerr << "Failed to create directory [" << this->saveBoxesFolder << "]"
             << std::endl;
      return;
    }
  }

  // Save the images in format of 0000001, 0000002 .. etc
  // Useful in sorting them in python
  std::stringstream ss;
  ss << std::setw(7) << std::setfill('0') << this->saveCounter;
  std::string saveCounterString = ss.str();

  std::string filename = this->saveBoxesFolder + "/boxes_" +
    saveCounterString + ".csv";
  std::ofstream file(filename);

  if (this->type == rendering::BoundingBoxType::BBT_BOX3D)
  {
    file << "label,x,y,z,w,h,l,roll,pitch,yaw\n";
    for (const auto &box : this->boundingBoxes)
    {
      auto label = std::to_string(box.Label());

      auto x = std::to_string(box.Center().X());
      auto y = std::to_string(box.Center().Y());
      auto z = std::to_string(box.Center().Z());

      auto w = std::to_string(box.Size().X());
      auto h = std::to_string(box.Size().Y());
      auto l = std::to_string(box.Size().Z());

      auto roll = std::to_string(box.Orientation().Roll());
      auto pitch = std::to_string(box.Orientation().Pitch());
      auto yaw = std::to_string(box.Orientation().Yaw());

      // label x y z w h l roll pitch yaw
      std::string sep = ",";
      std::string boxString = label + sep + x + sep + y + sep + z + sep +
        w + sep + h + sep + l + sep + roll + sep + pitch + sep + yaw;

      file << boxString + '\n';
    }
  }
  else
  {
    file << "label,x_center,y_center,width,height\n";
    for (const auto &box : this->boundingBoxes)
    {
      auto label = std::to_string(box.Label());

      auto x = std::to_string(box.Center().X());
      auto y = std::to_string(box.Center().Y());
      auto width = std::to_string(box.Size().X());
      auto height = std::to_string(box.Size().Y());

      // label x y width height
      std::string sep = ",";
      std::string boxString = label + sep +
        x + sep + y + sep + width + sep + height;

      file << boxString + '\n';
    }
  }
  file.close();
}

//////////////////////////////////////////////////
bool BoundingBoxCameraSensor::HasConnections() const
{
  return (this->dataPtr->imagePublisher &&
      this->dataPtr->imagePublisher.HasConnections()) ||
      (this->dataPtr->boxesPublisher &&
      this->dataPtr->boxesPublisher.HasConnections()) ||
      this->HasInfoConnections();
}
