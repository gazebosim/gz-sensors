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
  public: ignition::rendering::Image image;

  /// \brief A class to manage saving images to disk
  public: std::unique_ptr<ImageSaver> saver;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: ignition::common::EventT<void(const ignition::msgs::Image &)>
          imageEvent;

  /// \brief Connection to the Manager's scene change event.
  public: ignition::common::ConnectionPtr sceneChangeConnection;
};

//////////////////////////////////////////////////
bool CameraSensor::CreateCamera()
{
  sdf::ElementPtr cameraElem = this->SDF()->GetElement("camera");
  if (!cameraElem)
  {
    ignerr << "Unable to find <camera> SDF element\n";
    return false;
  }

  sdf::ElementPtr imgElem = cameraElem->GetElement("image");

  if (!imgElem)
  {
    ignerr << "Unable to find <camera><image> SDF element\n";
    return false;
  }

  int width = imgElem->Get<int>("width");
  int height = imgElem->Get<int>("height");

  this->dataPtr->camera = this->dataPtr->scene->CreateCamera(this->Name());
  this->dataPtr->camera->SetImageWidth(width);
  this->dataPtr->camera->SetImageHeight(height);

  // TODO these parameters via sdf
  this->dataPtr->camera->SetAntiAliasing(2);

  auto angle = cameraElem->Get<double>("horizontal_fov", 0);
  if (angle.first < 0.01 || angle.first > M_PI*2)
  {
    ignerr << "Invalid horizontal field of view [" << angle.first << "]\n";

    return false;
  }
  this->dataPtr->camera->SetAspectRatio(static_cast<double>(width)/height);
  this->dataPtr->camera->SetHFOV(angle.first);

  if (cameraElem->HasElement("distortion"))
  {
    // TODO Port Distortion class
    // This->dataPtr->distortion.reset(new Distortion());
    // This->dataPtr->distortion->Load(this->sdf->GetElement("distortion"));
  }

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

  this->dataPtr->image = this->dataPtr->camera->CreateImage();

  this->dataPtr->scene->RootVisual()->AddChild(this->dataPtr->camera);

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

  if (!Sensor::Load(_sdf))
  {
    return false;
  }

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::Image>(
          this->Topic());
  if (!this->dataPtr->pub)
    return false;

  if (this->dataPtr->scene)
  {
    this->CreateCamera();
  }

  // this->dataPtr->sceneChangeConnection = ;

  this->dataPtr->initialized = true;
  return true;
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
  // APIs make it possible for the scene pointer to change
  if (this->dataPtr->scene != _scene)
  {
    this->dataPtr->RemoveCamera(this->dataPtr->scene);
    this->dataPtr->scene = _scene;
    if (this->dataPtr->initialized)
      this->CreateCamera();
  }
}

//////////////////////////////////////////////////
bool CameraSensor::Update(const common::Time &_now)
{
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

  // move the camera to the current pose
  this->dataPtr->camera->SetLocalPose(this->Pose());

  // generate sensor data
  this->dataPtr->camera->Capture(this->dataPtr->image);
  unsigned char *data = this->dataPtr->image.Data<unsigned char>();

  unsigned int width = this->dataPtr->camera->ImageWidth();
  unsigned int height = this->dataPtr->camera->ImageHeight();
  ignition::common::Image::PixelFormatType format;

  switch (this->dataPtr->camera->ImageFormat())
  {
    case ignition::rendering::PF_R8G8B8:
      format = ignition::common::Image::RGB_INT8;
      break;
    default:
      ignerr << "Unsupported pixel format ["
             << this->dataPtr->camera->ImageFormat() << "]\n";
      break;
  }

  // Save image
  if (this->dataPtr->saver)
  {
    this->dataPtr->saver->SaveImage(data, width, height, format);
  }

  // create message
  ignition::msgs::Image msg;
  msg.set_width(width);
  msg.set_height(height);
  msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
        this->dataPtr->camera->ImageFormat()));

  msg.set_pixel_format(format);
  msg.set_data(data, this->dataPtr->camera->ImageMemorySize());
  msg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
  msg.mutable_header()->mutable_stamp()->set_nsec(_now.nsec);

  // publish
  this->dataPtr->pub.Publish(msg);

  // Call all registered callbacks.
  this->dataPtr->imageEvent(msg);

  return true;
}


IGN_COMMON_REGISTER_SINGLE_PLUGIN(
    ignition::sensors::CameraSensor,
    ignition::sensors::Sensor)
