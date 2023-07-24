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
#ifndef GZ_SENSORS_DEPTHCAMERASENSOR_HH_
#define GZ_SENSORS_DEPTHCAMERASENSOR_HH_

#include <memory>
#include <cstdint>
#include <string>

#include <sdf/sdf.hh>

#include <gz/common/Event.hh>
#include <gz/utils/SuppressWarning.hh>

#include <gz/msgs/image.pb.h>

// TODO(louise) Remove these pragmas once gz-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <gz/rendering/DepthCamera.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include "gz/sensors/depth_camera/Export.hh"
#include "gz/sensors/CameraSensor.hh"
#include "gz/sensors/Export.hh"
#include "gz/sensors/Sensor.hh"

namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {
    // forward declarations
    class DepthCameraSensorPrivate;

    /// \brief Depth camera sensor class.
    ///
    /// This class creates depth image from a Gazebo Rendering scene.
    /// The scene  must be created in advance and given to Manager::Init().
    /// It offers both a gz-transport interface and a direct C++ API
    /// to access the image data. The API works by setting a callback to be
    /// called with image data.
    class GZ_SENSORS_DEPTH_CAMERA_VISIBLE DepthCameraSensor
      : public CameraSensor
    {
      /// \brief constructor
      public: DepthCameraSensor();

      /// \brief destructor
      public: virtual ~DepthCameraSensor();

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
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;

      /// \brief Get a pointer to the rendering depth camera
      /// \return Rendering depth camera
      public: virtual rendering::DepthCameraPtr DepthCamera() const;

      /// \brief Depth data callback used to get the data from the sensor
      /// \param[in] _scan pointer to the data from the sensor
      /// \param[in] _width width of the depth image
      /// \param[in] _height height of the depth image
      /// \param[in] _channel bytes used for the depth data
      /// \param[in] _format string with the format
      public: void OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int _channel,
                    const std::string &_format);

      /// \brief Point cloud data callback used to get the data from the sensor
      /// \param[in] _scan pointer to the data from the sensor
      /// \param[in] _width width of the point cloud image
      /// \param[in] _height height of the point cloud image
      /// \param[in] _channels bytes used for the point cloud data
      /// \param[in] _format string with the format
      public: void OnNewRgbPointCloud(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int _channels,
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
                  std::function<
                  void(const gz::msgs::Image &)> _callback);

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

      /// \brief Get image width.
      /// \return width of the image
      public: virtual double FarClip() const;

      /// \brief Get image height.
      /// \return height of the image
      public: virtual double NearClip() const;

      // Documentation inherited.
      public: virtual bool HasConnections() const override;

      /// \brief Check if there are any depth subscribers
      /// \return True if there are subscribers, false otherwise
      public: virtual bool HasDepthConnections() const;

      /// \brief Check if there are any point subscribers
      /// \return True if there are subscribers, false otherwise
      public: virtual bool HasPointConnections() const;

      /// \brief Create a camera in a scene
      /// \return True on success.
      private: bool CreateCamera();

      /// \brief Callback that is triggered when the scene changes on
      /// the Manager.
      /// \param[in] _scene Pointer to the new scene.
      private: void OnSceneChange(gz::rendering::ScenePtr /*_scene*/)
              { }

      GZ_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<DepthCameraSensorPrivate> dataPtr;
      GZ_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
