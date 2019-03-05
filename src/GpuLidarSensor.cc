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


#include <ignition/common/Console.hh>
#include "ignition/sensors/GpuLidarSensor.hh"
#include "ignition/sensors/Register.hh"

using namespace ignition::sensors;

/// \brief Private data for the GpuLidar class
class ignition::sensors::GpuLidarSensorPrivate
{
  /// \brief A scene the camera is capturing
  public: ignition::rendering::ScenePtr scene;

  /// \brief Rendering camera
  public: ignition::rendering::GpuRaysPtr gpuRays;

  /// \brief Connection to the Manager's scene change event.
  public: ignition::common::ConnectionPtr sceneChangeConnection;
};

//////////////////////////////////////////////////
GpuLidarSensor::GpuLidarSensor()
  : dataPtr(new GpuLidarSensorPrivate())
{
}

//////////////////////////////////////////////////
GpuLidarSensor::~GpuLidarSensor()
{
  this->RemoveGpuRays(this->dataPtr->scene);

  this->dataPtr->sceneChangeConnection.reset();

  if (this->laserBuffer)
  {
    delete [] this->laserBuffer;
    this->laserBuffer = nullptr;
  }
}

/////////////////////////////////////////////////
void GpuLidarSensor::SetScene(ignition::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->lidarMutex);
  // APIs make it possible for the scene pointer to change
  if (this->dataPtr->scene != _scene)
  {
    this->RemoveGpuRays(this->dataPtr->scene);
    this->dataPtr->scene = _scene;

    if (this->initialized)
      this->CreateLidar();
  }
}

//////////////////////////////////////////////////
void GpuLidarSensor::RemoveGpuRays(
    ignition::rendering::ScenePtr _scene)
{
  if (_scene)
  {
    _scene->DestroySensor(this->dataPtr->gpuRays);
  }
  this->dataPtr->gpuRays.reset();
  this->dataPtr->gpuRays = nullptr;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Load(sdf::ElementPtr _sdf)
{
  // Check if this is being loaded via "builtin" or via a plugin
  if (!Lidar::Load(_sdf))
  {
    return false;
  }

  if (_sdf->GetName() == "sensor")
  {
    if (!_sdf->GetElement("ray"))
    {
      ignerr << "<sensor><camera> SDF element not found while attempting to "
        << "load a ignition::sensors::GpuLidarSensor\n";
      return false;
    }
  }

  if (this->dataPtr->scene)
  {
    this->CreateLidar();
  }

  this->dataPtr->sceneChangeConnection =
      RenderingEvents::ConnectSceneChangeCallback(
      std::bind(&GpuLidarSensor::SetScene, this, std::placeholders::_1));

  this->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool GpuLidarSensor::CreateLidar()
{
  this->dataPtr->gpuRays = this->dataPtr->scene->CreateGpuRays(
      this->Name());

  if (!this->dataPtr->gpuRays)
  {
    ignerr << "Unable to create gpu laser sensor\n";
    return false;
  }

  this->dataPtr->gpuRays->SetWorldPosition(this->Pose().Pos());
  this->dataPtr->gpuRays->SetWorldRotation(this->Pose().Rot());

  this->dataPtr->gpuRays->SetNearClipPlane(this->RangeMin());
  this->dataPtr->gpuRays->SetFarClipPlane(this->RangeMax());

  // Mask ranges outside of min/max to +/- inf, as per REP 117
  this->dataPtr->gpuRays->SetClamp(false);

  this->dataPtr->gpuRays->SetAngleMin(this->AngleMin().Radian());
  this->dataPtr->gpuRays->SetAngleMax(this->AngleMax().Radian());

  this->dataPtr->gpuRays->SetVerticalAngleMin(
      this->VerticalAngleMin().Radian());
  this->dataPtr->gpuRays->SetVerticalAngleMax(
      this->VerticalAngleMax().Radian());

  this->dataPtr->gpuRays->SetRayCount(this->RayCount());
  this->dataPtr->gpuRays->SetVerticalRayCount(
      this->VerticalRayCount());

  this->dataPtr->scene->RootVisual()->AddChild(
      this->dataPtr->gpuRays);

  return true;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Update(const ignition::common::Time &_now)
{
  if (!this->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->gpuRays)
  {
    ignerr << "GpuRays doesn't exist.\n";
    return false;
  }

  int len = this->dataPtr->gpuRays->RayCount() *
    this->dataPtr->gpuRays->VerticalRayCount() * 3;

  if (this->laserBuffer == nullptr)
  {
    this->laserBuffer = new float[len];
  }

  this->dataPtr->gpuRays->Update();
  this->dataPtr->gpuRays->Copy(this->laserBuffer);

  this->PublishLidarScan(_now);

  return true;
}

/////////////////////////////////////////////////
ignition::common::ConnectionPtr GpuLidarSensor::ConnectNewLidarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _heighti, unsigned int _channels,
                  const std::string &/*_format*/)> _subscriber)
{
  return this->dataPtr->gpuRays->ConnectNewGpuRaysFrame(_subscriber);
}

/////////////////////////////////////////////////
ignition::rendering::GpuRaysPtr GpuLidarSensor::GpuRays() const
{
  return this->dataPtr->gpuRays;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::IsHorizontal() const
{
  return this->dataPtr->gpuRays->IsHorizontal();
}

//////////////////////////////////////////////////
ignition::math::Angle GpuLidarSensor::HFOV() const
{
  return this->dataPtr->gpuRays->HFOV();
}

//////////////////////////////////////////////////
ignition::math::Angle GpuLidarSensor::VFOV() const
{
  return this->dataPtr->gpuRays->VFOV();
}

IGN_SENSORS_REGISTER_STATIC_SENSOR("gpu_lidar", GpuLidarSensor)
