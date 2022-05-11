/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <ignition/common/Profiler.hh>

// TODO(louise) Remove these pragmas once ign-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <ignition/rendering/Camera.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include "ignition/sensors/RenderingSensor.hh"

/// \brief Private data class for RenderingSensor
class ignition::sensors::RenderingSensorPrivate
{
  /// \brief Pointer to the scene
  public: ignition::rendering::ScenePtr scene;

  /// \brief Manually update the rendering scene graph
  public: bool manualSceneUpdate = false;

  /// \brief Pointer to the internal rendering sensors used for generating
  /// sensor data
  public: std::vector<rendering::SensorPtr::weak_type> sensors;
};

using namespace ignition;
using namespace sensors;

//////////////////////////////////////////////////
RenderingSensor::RenderingSensor() :
  dataPtr(new RenderingSensorPrivate)
{
}

//////////////////////////////////////////////////
RenderingSensor::~RenderingSensor()
{
  if (!this->dataPtr->scene)
    return;
  for (auto &s : this->dataPtr->sensors)
  {
    auto sensor = s.lock();
    if (sensor)
      this->dataPtr->scene->DestroySensor(sensor);
  }
  this->dataPtr->sensors.clear();
}

/////////////////////////////////////////////////
void RenderingSensor::SetScene(rendering::ScenePtr _scene)
{
  this->dataPtr->scene = _scene;
}

/////////////////////////////////////////////////
rendering::ScenePtr RenderingSensor::Scene() const
{
  return this->dataPtr->scene;
}

/////////////////////////////////////////////////
void RenderingSensor::AddSensor(rendering::SensorPtr _sensor)
{
  this->dataPtr->sensors.push_back(_sensor);
}

/////////////////////////////////////////////////
void RenderingSensor::SetManualSceneUpdate(bool _manual)
{
  this->dataPtr->manualSceneUpdate = _manual;
}

/////////////////////////////////////////////////
bool RenderingSensor::ManualSceneUpdate() const
{
  return this->dataPtr->manualSceneUpdate;
}

/////////////////////////////////////////////////
void RenderingSensor::Render()
{
  IGN_PROFILE("RenderingSensor::Render");
  // Skip scene update. The user indicated that they will do this manually.
  // Performance is improved when a global scene update occurs only once per
  // frame, which can be acheived using a manual scene update.
  if (!this->dataPtr->manualSceneUpdate)
    this->dataPtr->scene->PreRender();

  for (auto rs : this->dataPtr->sensors)
  {
    auto s = rs.lock();
    if (!s)
      continue;
    rendering::CameraPtr rc =
        std::dynamic_pointer_cast<rendering::Camera>(s);
    if (rc)
    {
      rc->Render();
      rc->PostRender();
    }
  }

  if (!this->dataPtr->manualSceneUpdate &&
      !this->dataPtr->scene->LegacyAutoGpuFlush())
  {
    // When LegacyAutoGpuFlush = true, that function gets
    // called per sensor, so we don't have to do anything here
    this->dataPtr->scene->PostRender();
  }
}
