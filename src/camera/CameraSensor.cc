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

#include <ignition/sensors/CameraSensor.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>
#include <ignition/math/Angle.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/transport.hh>

#include "src/camera/ImageSaver.hh"

using namespace ignition::sensors;


class ignition::sensors::CameraSensorPrivate
{
  /// \brief Set fields in this class from an sdf element
  public: bool PopulateFromSDF(sdf::ElementPtr _sdf);

  /// \brief Create a camera in a scene
  public: void CreateCamera(ignition::rendering::ScenePtr _scene);

  /// \brief Remove a camera from a scene
  public: void RemoveCamera(ignition::rendering::ScenePtr _scene);

  /// \brief Callback to call when an image is created
  public: std::function<
           void(const ignition::msgs::ImageStamped &)> callback = nullptr;

  /// \brief width of the image in pixels
  public: int imageWidth;

  /// \brief height of the image in pixels
  public: int imageHeight;

  /// \brief the horizontal field of view of the camera
  public: double horizontalFieldOfView;

  /// \brief Pixel format used by the camera
  public: ignition::common::Image::PixelFormatType format;

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief A scene the camera is capturing
  public: ignition::rendering::ScenePtr scene;

  /// \brief Rendering camera
  public: ignition::rendering::CameraPtr camera;

  /// \brief Pointer to an image to be published
  public: ignition::rendering::ImagePtr image;

  /// \brief A class to manage saving images to disk
  public: std::unique_ptr<ImageSaver> saver;

  /// \brief Pointer to CameraSensor
  public: CameraSensor *pThis = nullptr;
};

//////////////////////////////////////////////////
bool CameraSensorPrivate::PopulateFromSDF(sdf::ElementPtr _sdf)
{
  // Check if this is being loaded via "builtin" or via a plugin
  if (_sdf->GetName() == "sensor")
    _sdf = _sdf->GetElement("camera");

  if (!_sdf)
  {
    ignerr << "Null SDF element\n";
    return false;
  }

  sdf::ElementPtr imgElem = _sdf->GetElement("image");
  if (!imgElem)
  {
    ignerr << "Failed to find <image> element\n";
    return false;
  }

  this->imageWidth = imgElem->Get<int>("width");
  this->imageHeight = imgElem->Get<int>("height");
  std::string format = imgElem->Get<std::string>("format");
  this->format = ignition::common::Image::ConvertPixelFormat(format);
  if (ignition::common::Image::UNKNOWN_PIXEL_FORMAT == this->format)
  {
    ignerr << "Unknown pixel format [" << format << "]\n";
    return false;
  }

  // Create the directory to store frames
  if (_sdf->HasElement("save") &&
      _sdf->GetElement("save")->Get<bool>("enabled"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("save");
    std::string path = elem->Get<std::string>("path");
    std::string prefix = this->pThis->Name() + "_";
    this->saver.reset(new ImageSaver(path, prefix));
  }

  auto angle = _sdf->Get<double>("horizontal_fov", 0);
  if (angle.first < 0.01 || angle.first > M_PI*2)
  {
    ignerr << "Invalid horizontal field of view [" << angle.first << "]\n";
    return false;
  }
  this->horizontalFieldOfView = angle.first;

  if (_sdf->HasElement("distortion"))
  {
    // TODO Port Distortion class
    // This->dataPtr->distortion.reset(new Distortion());
    // This->dataPtr->distortion->Load(this->sdf->GetElement("distortion"));
  }
  return true;
}

//////////////////////////////////////////////////
void CameraSensorPrivate::CreateCamera(ignition::rendering::ScenePtr _scene)
{
  if (!_scene)
    return;

  this->camera = _scene->CreateCamera(this->pThis->Name());
  this->camera->SetImageWidth(this->imageWidth);
  this->camera->SetImageHeight(this->imageHeight);
  // TODO these parameters via sdf
  this->camera->SetAntiAliasing(2);
  this->camera->SetAspectRatio(1.333);
  this->camera->SetHFOV(this->horizontalFieldOfView);
  // TODO other camera parameters via sdf

  switch (this->format)
  {
    case ignition::common::Image::RGB_INT8:
      this->camera->SetImageFormat(ignition::rendering::PF_R8G8B8);
      break;
    default:
      ignerr << "Unsupported pixel format [" << this->format << "]\n";
      break;
  }

  this->image = std::make_shared<ignition::rendering::Image>(
        this->camera->CreateImage());

  ignition::rendering::VisualPtr root = _scene->RootVisual();
  root->AddChild(camera);
}

//////////////////////////////////////////////////
void CameraSensorPrivate::RemoveCamera(ignition::rendering::ScenePtr _scene)
{
  if (_scene)
  {
    // TODO Remove camera from scene!
  }
  this->camera = nullptr;
}

//////////////////////////////////////////////////
CameraSensor::CameraSensor()
{
  this->dataPtr.reset(new CameraSensorPrivate());
  this->dataPtr->pThis = this;
}

//////////////////////////////////////////////////
CameraSensor::~CameraSensor()
{
}

//////////////////////////////////////////////////
void CameraSensor::Init(
    ignition::sensors::Manager *_mgr, SensorId _id)
{
  this->Sensor::Init(_mgr, _id);
}

//////////////////////////////////////////////////
bool CameraSensor::Load(sdf::ElementPtr _sdf)
{
  if (!this->Sensor::Load(_sdf))
    return false;

  if (!this->dataPtr->PopulateFromSDF(_sdf))
    return false;

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::ImageStamped>(
          this->Topic());
  if (!this->dataPtr->pub)
    return false;

  this->dataPtr->scene = this->Manager()->RenderingScene();
  if (this->dataPtr->scene)
  {
    this->dataPtr->CreateCamera(this->dataPtr->scene);
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool CameraSensor::SetImageCallback(std::function<
    void(const ignition::msgs::ImageStamped &)> _callback)
{
  if (!this->dataPtr->initialized)
    return false;
  this->dataPtr->callback = _callback;
  return true;
}

//////////////////////////////////////////////////
bool CameraSensor::Update(const common::Time &_now)
{
  if (!this->dataPtr->initialized)
    return false;

  // APIs make it possible for the scene pointer to change
  auto newScene = this->Manager()->RenderingScene();
  if (this->dataPtr->scene != newScene)
  {
    this->dataPtr->RemoveCamera(this->dataPtr->scene);
    this->dataPtr->CreateCamera(newScene);
    this->dataPtr->scene = newScene;
  }

  if (!this->dataPtr->camera)
    return false;

  // move the camera to the current pose
  auto pose = this->Pose();
  auto euler = pose.Rot().Euler();
  this->dataPtr->camera->SetLocalPosition(
      pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
  this->dataPtr->camera->SetLocalRotation(euler.X(), euler.Y(), euler.Z());

  // generate sensor data
  this->dataPtr->camera->Capture(*this->dataPtr->image);
  unsigned char *data = this->dataPtr->image->Data<unsigned char>();

  // Save image
  if (this->dataPtr->saver)
  {
    this->dataPtr->saver->SaveImage(data, this->dataPtr->imageWidth,
        this->dataPtr->imageHeight, this->dataPtr->format);
  }

  // create message
  ignition::msgs::ImageStamped msg;
  msg.mutable_image()->set_width(this->dataPtr->camera->ImageWidth());
  msg.mutable_image()->set_height(this->dataPtr->camera->ImageHeight());
  msg.mutable_image()->set_step(this->dataPtr->camera->ImageWidth() *
      rendering::PixelUtil::BytesPerPixel(
        this->dataPtr->camera->ImageFormat()));
  msg.mutable_image()->set_pixel_format(this->dataPtr->format);
  msg.mutable_image()->set_data(data, this->dataPtr->camera->ImageMemorySize());
  msg.mutable_time()->set_sec(_now.sec);
  msg.mutable_time()->set_nsec(_now.nsec);

  // publish
  if (this->dataPtr->callback)
    this->dataPtr->callback(msg);
  this->dataPtr->pub.Publish(msg);

  return true;
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(
    ignition::sensors::CameraSensor,
    ignition::sensors::Sensor)
