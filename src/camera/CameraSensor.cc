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
  /// \brief Remove a camera from a scene
  public: void RemoveCamera(ignition::rendering::ScenePtr _scene);

  /// \brief Callback to call when an image is created
  public: std::function<
           void(const ignition::msgs::ImageStamped &)> callback = nullptr;

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
};

//////////////////////////////////////////////////
bool CameraSensor::CreateCamera(ignition::rendering::ScenePtr _scene)
{
  if (!_scene)
    return false;

  sdf::ElementPtr cameraElem = this->SDF()->GetElement("camera");

  this->dataPtr->camera = _scene->CreateCamera(this->Name());

  sdf::ElementPtr imgElem = cameraElem->GetElement("image");
  if (!imgElem)
    return false;

  this->dataPtr->camera->SetImageWidth(imgElem->Get<int>("width"));
  this->dataPtr->camera->SetImageHeight(imgElem->Get<int>("height"));

  auto angle = cameraElem->Get<double>("horizontal_fov", 0);
  if (angle.first < 0.01 || angle.first > M_PI*2)
  {
    ignerr << "...";
    return false;
  }
  this->dataPtr->camera->SetHFOV(angle.first);

  if (cameraElem->HasElement("distortion"))
  {
    // TODO Port Distortion class
    // This->dataPtr->distortion.reset(new Distortion());
    // This->dataPtr->distortion->Load(this->sdf->GetElement("distortion"));
  }

  // TODO these parameters via sdf
  this->dataPtr->camera->SetAntiAliasing(2);
  this->dataPtr->camera->SetAspectRatio(1.333);

  // TODO other camera parameters via sdf

  std::string formatStr = imgElem->Get<std::string>("format");
  ignition::common::Image::PixelFormatType format =
    ignition::common::Image::ConvertPixelFormat(formatStr);
  switch (format)
  {
    case ignition::common::Image::RGB_INT8:
      this->dataPtr->camera->SetImageFormat(ignition::rendering::PF_R8G8B8);
      break;
    default:
      ignerr << "Unsupported pixel format [" << formatStr << "]\n";
      break;
  }

  this->dataPtr->image = std::make_shared<ignition::rendering::Image>(
        this->dataPtr->camera->CreateImage());

  _scene->RootVisual()->AddChild(this->dataPtr->camera);

  // Create the directory to store frames
  if (cameraElem->HasElement("save") &&
      cameraElem->GetElement("save")->Get<bool>("enabled"))
  {
    sdf::ElementPtr elem = cameraElem->GetElement("save");
    std::string path = elem->Get<std::string>("path");
    std::string prefix = this->Name() + "_";
    this->dataPtr->saver.reset(new ImageSaver(path, prefix));
  }
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
bool CameraSensor::Load(sdf::ElementPtr _sdf)
{
  // Check if this is being loaded via "builtin" or via a plugin
  if (_sdf->GetName() == "sensor")
  {
    if (!_sdf->GetElement("camera"))
    {
      ignerr << "<sensor><camera> SDF element not found while attempting to "
        << "load a ignition::sensors::CameraSensor\n";
      return false;
    }
  }

  if (!this->Sensor::Load(_sdf))
  {
    return false;
  }

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::ImageStamped>(
          this->Topic());
  if (!this->dataPtr->pub)
    return false;

  if (this->Iface()->RenderingScene())
  {
    this->CreateCamera(this->Iface()->RenderingScene());
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
  {
    ignerr << "Not initialized\n";
    return false;
  }

  // APIs make it possible for the scene pointer to change
  /* auto newScene = this->Manager()->RenderingScene();
  if (this->dataPtr->scene != newScene)
  {
    this->dataPtr->RemoveCamera(this->dataPtr->scene);
    this->CreateCamera(newScene);
    this->dataPtr->scene = newScene;
  }*/

  if (!this->dataPtr->camera)
  {
    ignerr << "No camera\n";
    return false;
  }

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
    // this->dataPtr->saver->SaveImage(data);
  }

  // create message
  ignition::msgs::ImageStamped msg;
  msg.mutable_image()->set_width(this->dataPtr->camera->ImageWidth());
  msg.mutable_image()->set_height(this->dataPtr->camera->ImageHeight());
  msg.mutable_image()->set_step(this->dataPtr->camera->ImageWidth() *
      rendering::PixelUtil::BytesPerPixel(
        this->dataPtr->camera->ImageFormat()));
  // msg.mutable_image()->set_pixel_format(this->dataPtr->format);
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
