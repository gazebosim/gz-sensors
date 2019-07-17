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
#ifndef IGNITION_SENSORS_RENDERINGSENSOR_HH_
#define IGNITION_SENSORS_RENDERINGSENSOR_HH_

#include <memory>

#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Sensor.hh>

#include "ignition/sensors/rendering/Export.hh"
#include "ignition/sensors/Sensor.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief forward declarations
    class RenderingSensorPrivate;

    /// \brief a rendering sensor class
    ///
    ///   This class is a base for all rendering sensor classes. It provides
    /// interface to ignition rendering objects
    class IGNITION_SENSORS_RENDERING_VISIBLE RenderingSensor
        : public Sensor
    {
      /// \brief constructor
      protected: RenderingSensor();

      /// \brief destructor
      public: virtual ~RenderingSensor();

      /// \brief Set the rendering scene.
      ///
      /// \param[in] _scene Pointer to the scene
      public: virtual void SetScene(rendering::ScenePtr _scene);

      /// \brief Get the rendering scene.
      public: rendering::ScenePtr Scene() const;

      /// \brief Render update. This performs the actual render operation.
      public: void Render();

      /// \brief Set whether to update the scene graph manually. If set to true,
      /// it is expected that rendering::Scene::PreRender is called manually
      /// before calling Render()
      public: void SetManualSceneUpdate(bool _manual);

      /// \brief Get whether the scene graph is updated manually. Defaults to
      /// false.
      /// \sa SetManualSceneUpdate
      public: bool ManualSceneUpdate() const;

      /// \brief Add a rendering::Sensor. Its render updates will be handled
      /// by this base class.
      /// \param[in] _sensor Sensor to add.
      protected: void AddSensor(rendering::SensorPtr _sensor);

      /// \internal
      /// \brief Data pointer for private data
      private: std::unique_ptr<RenderingSensorPrivate> dataPtr;
    };
    }
  }
}

#endif
