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

#include <ignition/sensors/GpuLidarSensor.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

#include <ignition/sensors/Events.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/transport.hh>


#include "ignition/rendering/RenderTypes.hh"


namespace ignition
{
  namespace common
  {
    class Mesh;
  }
}

namespace ignition
{
  namespace sensors
  {
    /// \internal
    /// \brief Private data for the GpuLidar class
    class GpuLidarSensorPrivate
    {
      /// \brief constructor
      public: GpuLidarSensorPrivate();

      /// \brief destructor
      public: ~GpuLidarSensorPrivate();

      /// \brief A scene the camera is capturing
      public: ignition::rendering::ScenePtr scene;

      /// \brief Rendering camera
      public: ignition::rendering::GpuRaysPtr gpuRays;

      /// \brief Connection to the Manager's scene change event.
      public: ignition::common::ConnectionPtr sceneChangeConnection;
    };
}
}

using namespace ignition::sensors;

//////////////////////////////////////////////////
GpuLidarSensorPrivate::GpuLidarSensorPrivate()
{
}

//////////////////////////////////////////////////
GpuLidarSensorPrivate::~GpuLidarSensorPrivate()
{
}

//////////////////////////////////////////////////
GpuLidarSensor::GpuLidarSensor()
  : gpuLidarDataPtr(new GpuLidarSensorPrivate())
{
}

//////////////////////////////////////////////////
GpuLidarSensor::~GpuLidarSensor()
{
  this->gpuLidarDataPtr->gpuRays.reset();
  this->gpuLidarDataPtr->gpuRays = nullptr;

  this->gpuLidarDataPtr->sceneChangeConnection.reset();

  if (this->dataPtr->laserBuffer)
  {
    delete [] this->dataPtr->laserBuffer;
    this->dataPtr->laserBuffer = nullptr;
  }
}

/////////////////////////////////////////////////
void GpuLidarSensor::SetScene(ignition::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // APIs make it possible for the scene pointer to change
  if (this->gpuLidarDataPtr->scene != _scene)
  {
    this->RemoveGpuRays(this->gpuLidarDataPtr->scene);
    this->gpuLidarDataPtr->scene = _scene;

    if (this->dataPtr->initialized)
      this->CreateLidar();
  }
}

//////////////////////////////////////////////////
void GpuLidarSensor::RemoveGpuRays(
    ignition::rendering::ScenePtr _scene)
{
  if (_scene)
  {
    // \todo(nkoenig) Remove camera from scene!
  }
  // this->gpuLidarDataPtr->gpuRays = nullptr;
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

  if (this->gpuLidarDataPtr->scene)
  {
    this->CreateLidar();
  }

  this->gpuLidarDataPtr->sceneChangeConnection =
    Events::ConnectSceneChangeCallback(std::bind(&GpuLidarSensor::SetScene,
          this, std::placeholders::_1));

  this->dataPtr->initialized = true;

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
  this->gpuLidarDataPtr->gpuRays = this->gpuLidarDataPtr->scene->CreateGpuRays(
      this->Name() + "_gpu_rays");

  if (!this->gpuLidarDataPtr->gpuRays)
  {
    ignerr << "Unable to create gpu laser sensor\n";
    return false;
  }

  this->gpuLidarDataPtr->gpuRays->SetWorldPosition(this->Pose().Pos());
  this->gpuLidarDataPtr->gpuRays->SetWorldRotation(this->Pose().Rot());

  this->gpuLidarDataPtr->gpuRays->SetNearClipPlane(this->RangeMin());
  this->gpuLidarDataPtr->gpuRays->SetFarClipPlane(this->RangeMax());

  // Mask ranges outside of min/max to +/- inf, as per REP 117
  this->gpuLidarDataPtr->gpuRays->SetClamp(false);

  this->gpuLidarDataPtr->gpuRays->SetAngleMin(this->AngleMin().Radian());
  this->gpuLidarDataPtr->gpuRays->SetAngleMax(this->AngleMax().Radian());

  this->gpuLidarDataPtr->gpuRays->SetVerticalAngleMin(
      this->VerticalAngleMin().Radian());
  this->gpuLidarDataPtr->gpuRays->SetVerticalAngleMax(
      this->VerticalAngleMax().Radian());

  this->gpuLidarDataPtr->gpuRays->SetRayCount(this->RayCount());
  this->gpuLidarDataPtr->gpuRays->SetVerticalRayCount(
      this->VerticalRayCount());

  this->gpuLidarDataPtr->scene->RootVisual()->AddChild(
      this->gpuLidarDataPtr->gpuRays);

  return true;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Update(const common::Time &_now)
{
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->gpuLidarDataPtr->gpuRays)
  {
    ignerr << "GpuRays doesn't exist.\n";
    return false;
  }

  int len = this->gpuLidarDataPtr->gpuRays->RayCount() *
    this->gpuLidarDataPtr->gpuRays->VerticalRayCount() * 3;

  if (this->dataPtr->laserBuffer == nullptr)
  {
    this->dataPtr->laserBuffer = new float[len];
  }

  this->gpuLidarDataPtr->gpuRays->SetWorldPosition(this->Pose().Pos());
  this->gpuLidarDataPtr->gpuRays->SetWorldRotation(this->Pose().Rot());
  this->gpuLidarDataPtr->gpuRays->Update();
  this->gpuLidarDataPtr->gpuRays->Copy(this->dataPtr->laserBuffer);

  this->PublishLidarScan(_now);

  return true;
}

/////////////////////////////////////////////////
ignition::common::ConnectionPtr GpuLidarSensor::ConnectNewLidarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _heighti, unsigned int _channels,
                  const std::string &/*_format*/)> _subscriber)
{
  return this->gpuLidarDataPtr->gpuRays->ConnectNewGpuRaysFrame(_subscriber);
}

/////////////////////////////////////////////////
ignition::rendering::GpuRaysPtr GpuLidarSensor::GpuRays() const
{
  return this->gpuLidarDataPtr->gpuRays;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::IsHorizontal() const
{
  return this->gpuLidarDataPtr->gpuRays->IsHorizontal();
}

//////////////////////////////////////////////////
ignition::math::Angle GpuLidarSensor::HFOV() const
{
  return this->gpuLidarDataPtr->gpuRays->HFOV();
}

//////////////////////////////////////////////////
ignition::math::Angle GpuLidarSensor::VFOV() const
{
  return this->gpuLidarDataPtr->gpuRays->VFOV();
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(
    ignition::sensors::GpuLidarSensor,
    ignition::sensors::Sensor)
