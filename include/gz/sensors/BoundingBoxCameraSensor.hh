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

#ifndef GZ_SENSORS_BOUNDINGBOXCAMERASENSOR_HH_
#define GZ_SENSORS_BOUNDINGBOXCAMERASENSOR_HH_

#include <memory>
#include <vector>

#include <gz/rendering/BoundingBoxCamera.hh>
#include <sdf/Sensor.hh>

#include "gz/sensors/CameraSensor.hh"
#include "gz/sensors/boundingbox_camera/Export.hh"

namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {
    // forward declarations
    class BoundingBoxCameraSensorPrivate;

    /// \brief BoundingBox camera sensor class.
    ///
    /// This class creates a BoundingBox image from an gz rendering scene.
    /// The scene must be created in advance and given to Manager::Init().
    /// It offers both an gz-transport interface and a direct C++ API
    /// to access the image data. The API works by setting a callback to be
    /// called with image data.
    class GZ_SENSORS_BOUNDINGBOX_CAMERA_VISIBLE
      BoundingBoxCameraSensor : public CameraSensor
    {
      /// \brief constructor
      public: BoundingBoxCameraSensor();

      /// \brief destructor
      public: virtual ~BoundingBoxCameraSensor();

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

      /// \brief Get the rendering BoundingBox camera
      /// \return BoundingBox camera pointer
      public: virtual rendering::BoundingBoxCameraPtr
        BoundingBoxCamera() const;

      /// \brief Callback on new bounding boxes from bounding boxes camera
      /// \param[in] _boxes Detected bounding boxes from the camera
      public: void OnNewBoundingBoxes(
        const std::vector<rendering::BoundingBox> &_boxes);

      /// \brief Set the rendering scene.
      /// \param[in] _scene Pointer to the scene
      public: virtual void SetScene(
                  rendering::ScenePtr _scene) override;

      /// \brief Get image width.
      /// \return width of the image
      public: virtual unsigned int ImageWidth() const override;

      /// \brief Get image height.
      /// \return height of the image
      public: virtual unsigned int ImageHeight() const override;

      /// \brief Check if there are any subscribers
      /// \return True if there are subscribers, false otherwise
      public: virtual bool HasConnections() const override;

      /// \brief Create a camera in a scene
      /// \return True on success.
      private: bool CreateCamera();

      GZ_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<BoundingBoxCameraSensorPrivate> dataPtr;
      GZ_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
