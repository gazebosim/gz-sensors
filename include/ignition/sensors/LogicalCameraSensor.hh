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
#ifndef IGNITION_SENSORS_LOGICALCAMERASENSOR_HH_
#define IGNITION_SENSORS_LOGICALCAMERASENSOR_HH_

#include <map>
#include <memory>
#include <string>

#include <sdf/sdf.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>
#include <ignition/common/Time.hh>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>

#include <ignition/msgs.hh>

#include <ignition/sensors/config.hh>
#include <ignition/sensors/Export.hh>

#include "ignition/sensors/Sensor.hh"

#ifndef _WIN32
#  define LogicalCameraSensor_EXPORTS_API
#else
#  if (defined(LogicalCameraSensor_EXPORTS))
#    define LogicalCameraSensor_EXPORTS_API __declspec(dllexport)
#  else
#    define LogicalCameraSensor_EXPORTS_API __declspec(dllimport)
#  endif
#endif

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief forward declarations
    class LogicalCameraSensorPrivate;

    /// \brief Logical Camera Sensor Class
    ///
    /// A logical camera reports locations of objects. This camera finds models
    /// within the sensor's frustum and publishes information about the models
    /// on the sensor's topic.
    class LogicalCameraSensor_EXPORTS_API LogicalCameraSensor : public Sensor
    {
      /// \brief constructor
      public: LogicalCameraSensor();

      /// \brief destructor
      public: virtual ~LogicalCameraSensor();

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(sdf::ElementPtr _sdf) override;

      /// \brief Initialize values in the sensor
      /// \return True on success
      public: virtual bool Init() override;

      /// \brief Force the sensor to generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(const common::Time &_now) override;

      /// \brief Get the near distance. This is the distance from the
      /// frustum's vertex to the closest plane.
      /// \return Near distance.
      public: double Near() const;

      /// \brief Get the far distance. This is the distance from the
      /// frustum's vertex to the farthest plane.
      /// \return Far distance.
      public: double Far() const;

      /// \brief Set the models currently in the world
      /// \param[in] _models A map of model names to their world pose.
      public: void SetModelPoses(std::map<std::string, math::Pose3d> &&_models);

      /// \brief Get the horizontal field of view. The field of view is the
      /// angle between the frustum's vertex and the edges of the near or far
      /// plane. This value represents the horizontal angle.
      /// \return The field of view.
      public: ignition::math::Angle HorizontalFOV() const;

      /// \brief Get the aspect ratio, which is the width divided by height
      /// of the near or far planes.
      /// \return The frustum's aspect ratio.
      public: double AspectRatio() const;

      /// \brief Get the latest image. An image is an instance of
      /// msgs::LogicalCameraImage, which contains a list of detected models.
      /// \return List of detected models.
      public: msgs::LogicalCameraImage Image() const;

      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<LogicalCameraSensorPrivate> dataPtr;
    };
    }
  }
}

#endif
