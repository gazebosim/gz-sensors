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

#include <ignition/common/PluginMacros.hh>
#include <ignition/math/Angle.hh>
#include <ignition/transport.hh>

using namespace ignition::sensors;


class ignition::sensors::CameraSensorPrivate
{
  public: bool PopulateFromSDF(sdf::ElementPtr _sdf);

  /// \brief Callback to call when an image is created
  public: std::function<
           void(const ignition::msgs::ImageStamped &)> callback = nullptr;

  /// \brief width of the image in pixels
  public: int imageWidth;

  /// \brief height of the image in pixels
  public: int imageHeight;

  /// \brief the horizontal field of view of the camera
  public: double horizontalFieldOfView;

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;
};

//////////////////////////////////////////////////
bool CameraSensorPrivate::PopulateFromSDF(sdf::ElementPtr _sdf)
{
  // Check if this is being loaded via "builtin" or via a plugin
  if (_sdf->GetName() == "sensor")
    _sdf = _sdf->GetElement("camera");

  if (!_sdf)
    return false;

  sdf::ElementPtr imgElem = _sdf->GetElement("image");
  if (!imgElem)
    return false;

  this->imageWidth = imgElem->Get<int>("width");
  this->imageHeight = imgElem->Get<int>("height");
  // TODO Pixel format

  // Create the directory to store frames
  if (_sdf->HasElement("save") &&
      _sdf->GetElement("save")->Get<bool>("enabled"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("save");
    std::string path = elem->Get<std::string>("path");
    // TODO make directory, or delegate saving images to another class
  }

  if (_sdf->HasElement("horizontal_fov"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("horizontal_fov");
    ignition::math::Angle angle = elem->Get<double>();
    if (angle < 0.01 || angle > M_PI*2)
      return false;
    this->horizontalFieldOfView = angle.Radian();
  }

  if (_sdf->HasElement("distortion"))
  {
    // TODO Port Distortion class
    // This->dataPtr->distortion.reset(new Distortion());
    // This->dataPtr->distortion->Load(this->sdf->GetElement("distortion"));
  }
  return true;
}

//////////////////////////////////////////////////
CameraSensor::CameraSensor()
{
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
  this->dataPtr = std::make_shared<CameraSensorPrivate>();
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

  // TODO create rendering scene

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
void CameraSensor::Update(const common::Time &_now)
{
  // TODO generate sensor data
  ignition::msgs::ImageStamped msg;

  if (this->dataPtr->callback)
    this->dataPtr->callback(msg);
  this->dataPtr->pub.Publish(msg);
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(
    ignition::sensors::CameraSensor,
    ignition::sensors::Sensor)
