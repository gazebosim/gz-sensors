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
#ifndef IGNITION_SENSORS_THERMALCAMERASENSOR_HH_
#define IGNITION_SENSORS_THERMALCAMERASENSOR_HH_

#include <memory>
#include <cstdint>
#include <string>

#include <sdf/sdf.hh>

#include <ignition/common/Event.hh>
#include <ignition/utils/SuppressWarning.hh>

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <ignition/msgs.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

// TODO(louise) Remove these pragmas once ign-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <ignition/rendering/ThermalCamera.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include "ignition/sensors/thermal_camera/Export.hh"
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
    class ThermalCameraSensorPrivate;

    /// \brief Thermal camera sensor class.
    ///
    /// This class creates thermal image from an ignition rendering scene.
    /// The scene must be created in advance and given to Manager::Init().
    /// It offers both an ignition-transport interface and a direct C++ API
    /// to access the image data. The API works by setting a callback to be
    /// called with image data.
    class IGNITION_SENSORS_THERMAL_CAMERA_VISIBLE ThermalCameraSensor
      : public CameraSensor
    {
      /// \brief constructor
      public: ThermalCameraSensor();

      /// \brief destructor
      public: virtual ~ThermalCameraSensor();

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

      /// \brief Get a pointer to the rendering thermal camera
      /// \return Rendering thermal camera
      public: virtual rendering::ThermalCameraPtr ThermalCamera() const;

      /// \brief Thermal data callback used to get the data from the sensor
      /// \param[in] _scan pointer to the data from the sensor
      /// \param[in] _width width of the thermal image
      /// \param[in] _height height of the thermal image
      /// \param[in] _channel bytes used for the thermal data
      /// \param[in] _format string with the format
      public: void OnNewThermalFrame(const uint16_t *_scan,
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
      public: common::ConnectionPtr ConnectImageCallback(
                  std::function<void(const msgs::Image &)> _callback);

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

      /// \brief Set the ambient temperature of the environment
      /// \param[in] _ambient Ambient temperature in kelvin
      public: virtual void SetAmbientTemperature(float _ambient);

      /// \brief Set the range of ambient temperature
      /// \param[in] _range The ambient temperature ranges from
      /// (ambient - range/2) to (ambient + range/2).
      public: virtual void SetAmbientTemperatureRange(float _range);

      /// \brief Set the minimum temperature the sensor can detect
      /// \param[in] _min Min temperature in kelvin
      public: virtual void SetMinTemperature(float _min);

      /// \brief Set the maximum temperature the sensor can detect
      /// \param[in] _max Max temperature in kelvin
      public: virtual void SetMaxTemperature(float _max);

      /// \brief Set the temperature linear resolution. The thermal image data
      /// returned will be temperature in kelvin / resolution.
      /// Typical values are 0.01 (10mK), 0.1 (100mK), or 0.04 to simulate
      /// 14 bit format.
      /// \param[in] resolution Temperature linear resolution
      public: virtual void SetLinearResolution(float _resolution);

     /// \brief Create a camera in a scene
      /// \return True on success.
      private: bool CreateCamera();

      /// \brief Callback that is triggered when the scene changes on
      /// the Manager.
      /// \param[in] _scene Pointer to the new scene.
      private: void OnSceneChange(ignition::rendering::ScenePtr /*_scene*/)
              { }

      IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<ThermalCameraSensorPrivate> dataPtr;
      IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
