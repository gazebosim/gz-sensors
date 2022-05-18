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

#ifndef GZ_SENSORS_RENDERINGEVENTS_HH_
#define GZ_SENSORS_RENDERINGEVENTS_HH_

#include <gz/common/Event.hh>
#include <gz/utils/SuppressWarning.hh>

// TODO(louise) Remove these pragmas once ign-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <gz/rendering/RenderTypes.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <gz/sensors/config.hh>
#include <gz/sensors/rendering/Export.hh>

namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    class IGNITION_SENSORS_RENDERING_VISIBLE RenderingEvents
    {
      /// \brief Set a callback to be called when the scene is changed.
      ///
      /// \param[in] _callback  This callback will be called every time the
      /// scene is changed.
      /// \remark Do not block inside of the callback.
      /// \return A connection pointer that must remain in scope. When the
      /// connection pointer falls out of scope, the connection is broken.
      public: static gz::common::ConnectionPtr ConnectSceneChangeCallback(
                  std::function<void(const gz::rendering::ScenePtr &)>
                  _callback);

      IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Event that is used to trigger callbacks when the scene
      /// is changed
      public: static gz::common::EventT<
              void(const gz::rendering::ScenePtr &)> sceneEvent;
      IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}
#endif
