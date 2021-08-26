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

#include "ignition/sensors/BoundingBoxCameraSensor.hh"

#include <filesystem>
#include <memory>
#include <mutex>

#include <ignition/common/Console.hh>
#include <ignition/common/Image.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/msgs.hh>
#include <ignition/rendering/BoundingBoxCamera.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include "ignition/sensors/RenderingEvents.hh"
#include "ignition/sensors/SensorFactory.hh"

using namespace ignition;
using namespace sensors;

class ignition::sensors::BoundingBoxCameraSensorPrivate
{
  /// \brief Save an image of rgb camera
  public: void SaveImage();

  /// \brief Save the bounding boxes
  public: void SaveBoxes();

  /// \brief SDF Sensor DOM Object
  public: sdf::Sensor sdfSensor;

  /// \brief True if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Rendering BoundingBox Camera
  public: rendering::BoundingBoxCameraPtr boundingboxCamera {nullptr};

  /// \brief Rendering RGB Camera to draw boxes on it and publish
  /// its image (just for visualization)
  public: rendering::CameraPtr rgbCamera {nullptr};

  /// \brief Node to create publisher
  public: transport::Node node;

  /// \brief Publisher to publish Image msg with drawn boxes
  public: transport::Node::Publisher imagePublisher;

  /// \brief Publisher to publish BoundingBoxes msg
  public: transport::Node::Publisher boxesPublisher;

  /// \brief Image msg with drawn boxes on it
  public: msgs::Image imageMsg;

  /// \brief 2D axis aligned bounding boxes msg
  public: msgs::AnnotatedAxisAligned2DBox_V boxes2DMsg;

  /// \brief 3D oreinted bounding boxes msg
  public: msgs::AnnotatedOriented3DBox_V boxes3DMsg;

  /// \brief Topic to publish the bounding box msg
  public: std::string topicBoundingBoxes = "";

  /// \brief Topic to publish the image with drawn boxes
  public: std::string topicImage = "";

  /// \brief Vector to receive boxes from the rendering camera
  public: std::vector<rendering::BoundingBox> boundingBoxes;

  /// \brief RGB Image to draw boxes on it
  public: rendering::Image image;

  /// \brief Buffer contains the image data with drawn boxes
  public: unsigned char *imageBuffer {nullptr};

  /// \brief Buffer contains the image data to be saved
  public: unsigned char *saveImageBuffer {nullptr};

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
  public: bool saveSample = false;

  /// \brief path directory to where images & boxes are saved
  public: std::string savePath = "./";

  /// \brief Folder to save the image
  public: std::string saveImageFolder = "/images";

  /// \brief Folder to save the bounding boxes
  public: std::string saveBoxesFolder = "/boxes";

  /// \brief counter used to set the sample filename
  public: std::uint64_t saveCounter = 0;
};

//////////////////////////////////////////////////
BoundingBoxCameraSensor::BoundingBoxCameraSensor()
  : CameraSensor(), dataPtr(new BoundingBoxCameraSensorPrivate)
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
      ignerr << "Unknown bounding box type " << type << std::endl;
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
    ignerr << "Attempting to a load a BoundingBox Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    ignerr << "Attempting to a load a BoundingBox Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  this->dataPtr->topicBoundingBoxes = this->Topic();
  this->dataPtr->topicImage = this->Topic() + "_image";

  this->dataPtr->imagePublisher =
    this->dataPtr->node.Advertise<ignition::msgs::Image>(
      this->dataPtr->topicImage);

  if (!this->dataPtr->imagePublisher)
  {
    ignerr << "Unable to create publisher on topic ["
      << this->dataPtr->topicImage << "].\n";
    return false;
  }

  igndbg << "Camera images for [" << this->Name() << "] advertised on ["
    << this->dataPtr->topicImage << "]" << std::endl;

  if (this->dataPtr->type == rendering::BoundingBoxType::BBT_BOX3D)
  {
    this->dataPtr->boxesPublisher =
      this->dataPtr->node.Advertise<
      ignition::msgs::AnnotatedOriented3DBox_V>(
      this->dataPtr->topicBoundingBoxes);
  }
  else
  {
    this->dataPtr->boxesPublisher =
      this->dataPtr->node.Advertise<
      ignition::msgs::AnnotatedAxisAligned2DBox_V>(
      this->dataPtr->topicBoundingBoxes);
  }

  if (!this->dataPtr->boxesPublisher)
  {
    ignerr << "Unable to create publisher on topic ["
      << this->dataPtr->topicBoundingBoxes << "].\n";
    return false;
  }

  igndbg << "Bounding boxes for [" << this->Name() << "] advertised on ["
    << this->dataPtr->topicBoundingBoxes << std::endl;

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
  ignition::rendering::ScenePtr _scene)
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
    ignerr << "Unable to access camera SDF element\n";
    return false;
  }

  // Camera Info Msg
  this->PopulateInfo(sdfCamera);

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

  // Set Camera Properties
  this->dataPtr->rgbCamera->SetImageFormat(rendering::PF_R8G8B8);
  this->dataPtr->rgbCamera->SetImageWidth(width);
  this->dataPtr->rgbCamera->SetImageHeight(height);
  this->dataPtr->rgbCamera->SetVisibilityMask(sdfCamera->VisibilityMask());
  this->dataPtr->rgbCamera->SetNearClipPlane(sdfCamera->NearClip());
  this->dataPtr->rgbCamera->SetFarClipPlane(sdfCamera->FarClip());
  math::Angle angle = sdfCamera->HorizontalFov();
  if (angle < 0.01 || angle > IGN_PI*2)
  {
    ignerr << "Invalid horizontal field of view [" << angle << "]\n";
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

  // Add the camera to the scene
  this->Scene()->RootVisual()->AddChild(this->dataPtr->rgbCamera);
  this->Scene()->RootVisual()->AddChild(this->dataPtr->boundingboxCamera);

  // Add the rendering sensors to handle its render
  this->AddSensor(this->dataPtr->boundingboxCamera);
  this->AddSensor(this->dataPtr->rgbCamera);

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
    if (ignition::common::isDirectory(this->dataPtr->saveImageFolder))
    {
      for (const auto &entry : std::filesystem::directory_iterator(
        this->dataPtr->saveImageFolder))
      {
        // To disable unused warning
        (void)entry;
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
  const std::vector<rendering::BoundingBox> &boxes)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->boundingBoxes.clear();
  for (const auto &box : boxes)
    this->dataPtr->boundingBoxes.push_back(box);
}

//////////////////////////////////////////////////
bool BoundingBoxCameraSensor::Update(
  const std::chrono::steady_clock::duration &_now)
{
  IGN_PROFILE("BoundingBoxCameraSensor::Update");
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->boundingboxCamera || !this->dataPtr->rgbCamera)
  {
    ignerr << "Camera doesn't exist.\n";
    return false;
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

  this->dataPtr->imageBuffer = this->dataPtr->image.Data<unsigned char>();

  if (this->dataPtr->saveSample)
  {
    auto bufferSize = this->dataPtr->image.MemorySize();
    if (!this->dataPtr->saveImageBuffer)
      this->dataPtr->saveImageBuffer = new uint8_t[bufferSize];

    memcpy(this->dataPtr->saveImageBuffer, this->dataPtr->imageBuffer,
      bufferSize);
  }

  // Draw bounding boxes
  for (const auto &box : this->dataPtr->boundingBoxes)
  {
    this->dataPtr->boundingboxCamera->DrawBoundingBox(
      this->dataPtr->imageBuffer, box);
  }

  auto width = this->dataPtr->rgbCamera->ImageWidth();
  auto height = this->dataPtr->rgbCamera->ImageHeight();

  // Create Image message
  this->dataPtr->imageMsg.set_width(width);
  this->dataPtr->imageMsg.set_height(height);
  // Format
  this->dataPtr->imageMsg.set_step(
    width * rendering::PixelUtil::BytesPerPixel(rendering::PF_R8G8B8));
  this->dataPtr->imageMsg.set_pixel_format_type(
    msgs::PixelFormatType::RGB_INT8);
  // Time stamp
  auto stamp = this->dataPtr->imageMsg.mutable_header()->mutable_stamp();
  *stamp = msgs::Convert(_now);
  auto frame = this->dataPtr->imageMsg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());
  // Image data
  this->dataPtr->imageMsg.set_data(this->dataPtr->imageBuffer,
      rendering::PixelUtil::MemorySize(rendering::PF_R8G8B8,
      width, height));

  // Publish
  this->AddSequence(this->dataPtr->imageMsg.mutable_header(), "rgbImage");
  this->dataPtr->imagePublisher.Publish(this->dataPtr->imageMsg);

  if (this->dataPtr->type == rendering::BoundingBoxType::BBT_BOX3D)
  {
    this->dataPtr->boxes3DMsg.Clear();

    // Create 3D boxes message
    for (const auto &box : this->dataPtr->boundingBoxes)
    {
      // box data
      auto annotated_box = this->dataPtr->boxes3DMsg.add_annotated_box();

      auto oriented3DBox = new msgs::Oriented3DBox();
      auto center = new msgs::Vector3d();
      auto size = new msgs::Vector3d();
      auto rotation = new msgs::Quaternion();

      center->set_x(box.center.X());
      center->set_y(box.center.Y());
      center->set_z(box.center.Z());

      size->set_x(box.size.X());
      size->set_y(box.size.Y());
      size->set_z(box.size.Z());

      rotation->set_x(box.orientation.X());
      rotation->set_y(box.orientation.Y());
      rotation->set_z(box.orientation.Z());
      rotation->set_w(box.orientation.W());

      oriented3DBox->set_allocated_center(center);
      oriented3DBox->set_allocated_orientation(rotation);
      oriented3DBox->set_allocated_boxsize(size);

      annotated_box->set_allocated_box(oriented3DBox);
      annotated_box->set_label(box.label);
    }
    // time stamp
    auto stampBoxes =
      this->dataPtr->boxes3DMsg.mutable_header()->mutable_stamp();
    *stampBoxes = msgs::Convert(_now);
    auto frameBoxes = this->dataPtr->boxes3DMsg.mutable_header()->add_data();
    frameBoxes->set_key("frame_id");
    frameBoxes->add_value(this->Name());
  }
  else
  {
    this->dataPtr->boxes2DMsg.Clear();

    // Create 2D boxes message
    for (const auto &box : this->dataPtr->boundingBoxes)
    {
      // box data
      auto annotated_box = this->dataPtr->boxes2DMsg.add_annotated_box();

      auto axisAlignedBox = new msgs::AxisAligned2DBox();
      auto min_corner = new msgs::Vector2d();
      auto max_corner = new msgs::Vector2d();

      min_corner->set_x(box.center.X() - box.size.X() / 2);
      min_corner->set_y(box.center.Y() - box.size.Y() / 2);

      max_corner->set_x(box.center.X() + box.size.X() / 2);
      max_corner->set_y(box.center.Y() + box.size.Y() / 2);

      axisAlignedBox->set_allocated_min_corner(min_corner);
      axisAlignedBox->set_allocated_max_corner(max_corner);
      annotated_box->set_allocated_box(axisAlignedBox);
      annotated_box->set_label(box.label);
    }
    // time stamp
    auto stampBoxes =
      this->dataPtr->boxes2DMsg.mutable_header()->mutable_stamp();
    *stampBoxes = msgs::Convert(_now);
    auto frameBoxes = this->dataPtr->boxes2DMsg.mutable_header()->add_data();
    frameBoxes->set_key("frame_id");
    frameBoxes->add_value(this->Name());
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Publish
  this->PublishInfo(_now);
  if (this->dataPtr->type == rendering::BoundingBoxType::BBT_BOX3D)
  {
    this->AddSequence(
      this->dataPtr->boxes3DMsg.mutable_header(), "boundingboxes");
    this->dataPtr->boxesPublisher.Publish(this->dataPtr->boxes3DMsg);
  }
  else
  {
    this->AddSequence(
      this->dataPtr->boxes2DMsg.mutable_header(), "boundingboxes");
    this->dataPtr->boxesPublisher.Publish(this->dataPtr->boxes2DMsg);
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
  if (!ignition::common::isDirectory(this->savePath))
  {
    if (!ignition::common::createDirectories(this->savePath))
      return;
  }
  // Attempt to create the image directory if it doesn't exist
  if (!ignition::common::isDirectory(this->saveImageFolder))
  {
    if (!ignition::common::createDirectories(this->saveImageFolder))
      return;
  }

  auto width = this->rgbCamera->ImageWidth();
  auto height = this->rgbCamera->ImageHeight();
  if (width == 0 || height == 0)
    return;

  ignition::common::Image localImage;

  // Save the images in format of 0000001, 0000002 .. etc
  // Useful in sorting them in python
  std::stringstream ss;
  ss << std::setw(7) << std::setfill('0') << this->saveCounter;
  std::string saveCounterString = ss.str();

  std::string filename = "image_" + saveCounterString + ".png";

  localImage.SetFromData(this->saveImageBuffer, width, height,
      common::Image::RGB_INT8);
  localImage.SavePNG(
      ignition::common::joinPaths(this->saveImageFolder, filename));
}

//////////////////////////////////////////////////
void BoundingBoxCameraSensorPrivate::SaveBoxes()
{
  // Attempt to create the save directory if it doesn't exist
  if (!ignition::common::isDirectory(this->savePath))
  {
    if (!ignition::common::createDirectories(this->savePath))
      return;
  }
  // Attempt to create the boxes directory if it doesn't exist
  if (!ignition::common::isDirectory(this->saveBoxesFolder))
  {
    if (!ignition::common::createDirectories(this->saveBoxesFolder))
      return;
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
    for (auto box : this->boundingBoxes)
    {
      auto label = std::to_string(box.label);

      auto x = std::to_string(box.center.X());
      auto y = std::to_string(box.center.Y());
      auto z = std::to_string(box.center.Z());

      auto w = std::to_string(box.size.X());
      auto h = std::to_string(box.size.Y());
      auto l = std::to_string(box.size.Z());

      auto roll = std::to_string(box.orientation.Roll());
      auto pitch = std::to_string(box.orientation.Pitch());
      auto yaw = std::to_string(box.orientation.Yaw());

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
    for (auto box : this->boundingBoxes)
    {
      auto label = std::to_string(box.label);

      auto x = std::to_string(box.center.X());
      auto y = std::to_string(box.center.Y());
      auto width = std::to_string(box.size.X());
      auto height = std::to_string(box.size.Y());

      // label x y width height
      std::string sep = ",";
      std::string boxString = label + sep +
        x + sep + y + sep + width + sep + height;

      file << boxString + '\n';
    }
  }
  file.close();
}
