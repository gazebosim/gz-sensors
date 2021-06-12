/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef IGNITION_SENSORS_SEGMENTATIONCAMERASENSOR_HH_
#define IGNITION_SENSORS_SEGMENTATIONCAMERASENSOR_HH_

#include <string>
#include <memory>
#include <sdf/sdf.hh>

#include <ignition/common/Event.hh>
#include <ignition/common/PluginMacros.hh>
#include <ignition/common/SuppressWarning.hh>
#include <ignition/common/Time.hh>

#include "ignition/msgs.hh"
#include "ignition/transport/Node.hh"
#include "ignition/transport/Publisher.hh"

#include "ignition/sensors/CameraSensor.hh"
#include "ignition/sensors/Export.hh"
#include "ignition/sensors/Sensor.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    // forward declarations
    class SegmentationCameraSensorPrivate;

    /// \brief Segmentation camera sensor class.
    ///
    /// This class creates Segmentation image from an ignition rendering scene.
    /// The scene must be created in advance and given to Manager::Init().
    /// It offers both an ignition-transport interface and a direct C++ API
    /// to access the image data. The API works by setting a callback to be
    /// called with image data.
    class SegmentationCameraSensor : public CameraSensor
    {
      /// \brief constructor
      public: SegmentationCameraSensor();

      /// \brief destructor
      public: virtual ~SegmentationCameraSensor();

      /// \brief Load the sensor based on data from an sdf::Sensor object.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(const sdf::Sensor &_sdf) override;

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
      public: virtual bool IGN_DEPRECATED(4) Update(
        const ignition::common::Time &_now) override;

      /// \brief Force the sensor to generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;

      /// \brief Force the sensor to generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual rendering::SegmentationCameraPtr SegmentationCamera();

      /// \brief Segmentation data callback used to get the data from the sensor
      /// \param[in] _scan pointer to the data from the sensor
      /// \param[in] _width width of the Segmentation image
      /// \param[in] _height height of the Segmentation image
      /// \param[in] _channel bytes used for the Segmentation data
      /// \param[in] _format string with the format
      public: void OnNewSegmentationFrame(const uint8_t *, unsigned int,
        unsigned int, unsigned int, const std::string &);

      /// \brief Set the rendering scene.
      /// \param[in] _scene Pointer to the scene
      public: virtual void SetScene(
                  ignition::rendering::ScenePtr _scene) override;

      /// \brief Get image width.
      /// \return width of the image
      public: virtual unsigned int ImageWidth() const override;

      /// \brief Get image height.
      /// \return height of the image
      public: virtual unsigned int ImageHeight() const override;

     /// \brief Create a camera in a scene
      /// \return True on success.
      private: bool CreateCamera();

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<SegmentationCameraSensorPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
