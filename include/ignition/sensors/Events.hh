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

#ifndef IGNITION_SENSORS_EVENTS_HH_
#define IGNITION_SENSORS_EVENTS_HH_

#include <ignition/common/Event.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/sensors/ign_sensors_export.hh>

namespace ignition
{
  namespace sensors
  {
    class IGN_SENSORS_EXPORT Events
    {
      /// \brief Set a callback to be called when the scene is changed.
      ///
      /// \param[in] _callback  This callback will be called every time the
      /// scene is changed.
      /// \remark Do not block inside of the callback.
      /// \return A connection pointer that must remain in scope. When the
      /// connection pointer falls out of scope, the connection is broken.
      public: static ignition::common::ConnectionPtr ConnectSceneChangeCallback(
                  std::function<void (const ignition::rendering::ScenePtr &)>
                  _callback);

      /// \brief Event that is used to trigger callbacks when the scene
      /// is changed
      public: static ignition::common::EventT<
              void(const ignition::rendering::ScenePtr &)> sceneEvent;
    };
  }
}
#endif
