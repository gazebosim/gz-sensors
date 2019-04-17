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
#ifndef IGNITION_SENSORS_SENSOR_HH_
#define IGNITION_SENSORS_SENSOR_HH_

#include <memory>
#include <string>

#include <ignition/common/Time.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/sensors/config.hh>
#include <ignition/sensors/Export.hh>
#include <sdf/sdf.hh>

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief A string used to identify a sensor
    using SensorId = std::size_t;
    const SensorId NO_SENSOR = 0;

    /// \brief forward declarations
    class SensorPrivate;

    /// \brief a base sensor class
    ///
    ///   This class is a base for all sensor classes. It parses some common
    ///   SDF elements in the <sensor> tag and is responsible for making sure
    ///   sensors update at the right time.
    class IGNITION_SENSORS_VISIBLE Sensor
    {
      /// \brief constructor
      protected: Sensor();

      /// \brief destructor
      public: virtual ~Sensor();

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF <sensor> or <plugin> inside of <sensor>
      /// \return true if loading was successful
      public: virtual bool Load(const sdf::Sensor &_sdf);

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF <sensor> or <plugin> inside of <sensor>
      /// \return true if loading was successful
      public: virtual bool Load(sdf::ElementPtr _sdf);

      /// \brief Initialize values in the sensor
      public: virtual bool Init();

      /// \brief Force the sensor to generate data
      ///
      ///   This method must be overridden by sensors. Subclasses should not
      ///   not make a decision about whether or not they need to update. The
      ///   Sensor class will make sure Update() is called at the correct time.
      ///
      ///   If a subclass wants to have a variable update rate it should call
      ///   SetUpdateRate().
      ///
      ///   A subclass should return false if there was an error while updating
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      /// \sa SetUpdateRate()
      public: virtual bool Update(const common::Time &_now) = 0;

      /// \brief Return the next time the sensor will generate data
      public: common::Time NextUpdateTime() const;

      /// \brief Update the sensor.
      ///
      ///   This is called by the manager, and is responsible for determining
      ///   if this sensor needs to generate data at this time. If so, the
      ///   subclasses' Update() method will be called.
      /// \param[in] _now The current time
      /// \param[in] _force Force the update to happen even if it's not time
      /// \return True if the update was triggered (_force was true or _now
      /// >= next_update_time) and the sensor's
      /// bool Sensor::Update(const common::Time &_now) function returned true.
      /// False otherwise.
      /// \remarks If forced the NextUpdateTime() will be unchanged.
      /// \sa virtual bool Update(const common::Time &_name) = 0
      public: bool Update(const common::Time &_now, const bool _force);

      /// \brief Get the update rate of the sensor.
      ///
      ///   The update rate is the number of times per second a sensor should
      ///   generate and output data.
      /// \return _hz update rate of sensor.
      public: double UpdateRate() const;

      /// \brief Set the update rate of the sensor.
      /// \param[in] _hz update rate of sensor.
      public: void SetUpdateRate(const double _hz);

      /// \brief Get the current pose.
      /// \return Current pose of the sensor.
      public: ignition::math::Pose3d Pose() const;

      /// \brief Update the pose of the sensor
      public: void SetPose(const ignition::math::Pose3d &_pose);

      /// \brief Set the parent of the sensor
      public: void SetParent(const std::string &_parent);

      /// \brief Get name.
      /// \return Name of sensor.
      public: std::string Name() const;

      /// \brief Get topic
      /// \return Topic sensor publishes data to
      public: std::string Topic() const;

      /// \brief Get parent link of the sensor.
      /// \return Parent link of sensor.
      public: std::string Parent() const;

      /// \brief Get the sensor's ID.
      /// \return The sensor's ID.
      public: SensorId Id() const;

      /// \brief Get the SDF used to load this sensor.
      /// \return Pointer to an SDF element that contains initialization
      /// information for this sensor.
      public: sdf::ElementPtr SDF() const;

      /// \brief Set the rendering scene.
      ///
      /// A sensor subclass should override this function if the subclass
      /// needs a pointer to the scene.
      /// \param[in] _scene Pointer to the scene
      /// \deprecated See RenderingSensor::SetScene
      public: virtual void IGN_DEPRECATED(1) SetScene(
          ignition::rendering::ScenePtr _scene);

      /// \internal
      /// \brief Data pointer for private data
      private: std::unique_ptr<SensorPrivate> dataPtr;
    };
    }
  }
}

#endif
