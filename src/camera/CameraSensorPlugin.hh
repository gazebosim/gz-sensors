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
#ifndef IGNITION_SENSORS_CAMERA_CAMERASENSOR_PLUGIN_HH_
#define IGNITION_SENSORS_CAMERA_CAMERASENSOR_PLUGIN_HH_

#include <ignition/sensors/Sensor.hh>

namespace ignition
{
  namespace sensors
  {
    /// \brief a plugin for a camera sensor
    /// \internal
    class CameraSensorPlugin : public Sensor
    {
      /// \brief constructor
      public: CameraSensorPlugin();

      /// \brief destructor
      public: virtual ~CameraSensorPlugin();

      // inherited
      public: virtual bool Load(sdf::ElementPtr _sdf) override;

      // inherited
      protected: virtual void Update(const common::Time &_now) override;

      /// \brief width of the image in pixels
      private: int imageWidth;

      /// \brief height of the image in pixels
      private: int imageHeight;

      /// \brief the horizontal field of view of the camera
      private: double horizontalFieldOfView;
    };
  }
}

#endif



