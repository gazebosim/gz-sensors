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

#ifndef GZ_SENSORS_SEGMENTATIONCAMERASENSOR_HH_
#define GZ_SENSORS_SEGMENTATIONCAMERASENSOR_HH_

#include <memory>
#include <string>

#include <gz/common/Event.hh>
#include <gz/utils/SuppressWarning.hh>
#include <gz/msgs.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>
#include <sdf/sdf.hh>

#include "gz/sensors/CameraSensor.hh"
#include "gz/sensors/Export.hh"
#include "gz/sensors/Sensor.hh"

#include "gz/sensors/segmentation_camera/Export.hh"

namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {
    // forward declarations
    class SegmentationCameraSensorPrivate;

    /// \brief Segmentation camera sensor class.
    ///
    /// This class creates segmentation images from an ignition rendering scene.
    /// The scene must be created in advance and given to Manager::Init().
    /// It offers both an ignition-transport interface and a direct C++ API
    /// to access the image data. The API works by setting a callback to be
    /// called with image data.
    class GZ_SENSORS_SEGMENTATION_CAMERA_VISIBLE
      SegmentationCameraSensor : public CameraSensor
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
      /// \return true if the update was successful
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;

      /// \brief Get the rendering segmentation camera
      /// \return Segmentation camera pointer
      public: rendering::SegmentationCameraPtr SegmentationCamera() const;

      /// \brief Segmentation data callback used to get the data from the sensor
      /// \param[in] _data pointer to the data from the sensor
      /// \param[in] _width width of the segmentation image
      /// \param[in] _height height of the segmentation image
      /// \param[in] _channels num of channels
      /// \param[in] _format string with the format
      public: void OnNewSegmentationFrame(const uint8_t * _data,
        unsigned int _width, unsigned int _height, unsigned int _channels,
        const std::string &_format);

      /// \brief Set a callback to be called when image frame data is
      /// generated.
      /// \param[in] _callback This callback will be called every time the
      /// camera produces image data. The Update function will be blocked
      /// while the callbacks are executed.
      /// \remark Do not block inside of the callback.
      /// \return A connection pointer that must remain in scope. When the
      /// connection pointer falls out of scope, the connection is broken.
      public: gz::common::ConnectionPtr ConnectImageCallback(
                  std::function<void(const gz::msgs::Image &)> _callback);

      /// \brief Set the rendering scene.
      /// \param[in] _scene Pointer to the scene
      public: virtual void SetScene(
                  gz::rendering::ScenePtr _scene) override;

      /// \brief Get image width.
      /// \return width of the image
      public: virtual unsigned int ImageWidth() const override;

      /// \brief Get image height.
      /// \return height of the image
      public: virtual unsigned int ImageHeight() const override;

      // Documentation inherited.
      public: virtual bool HasConnections() const override;

      /// \brief Create a camera in a scene
      /// \return True on success.
      private: bool CreateCamera();

      IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<SegmentationCameraSensorPrivate> dataPtr;
      IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
