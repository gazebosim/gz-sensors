/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef GZ_SENSORS_DOPPLERVELOCITYLOG_HH_
#define GZ_SENSORS_DOPPLERVELOCITYLOG_HH_

#include <chrono>
#include <memory>
#include <unordered_map>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sensors/EnvironmentalData.hh>
#include <gz/sensors/RenderingSensor.hh>

namespace gz
{
  namespace sensors
  {
    /// \brief Kinematic state for an entity in the world.
    ///
    /// All quantities are defined w.r.t. the world frame.
    struct EntityKinematicState
    {
      gz::math::Pose3d pose;
      gz::math::Vector3d linearVelocity;
      gz::math::Vector3d angularVelocity;
    };

    /// \brief Kinematic state for all entities in the world.
    using WorldKinematicState = std::unordered_map<
      uint64_t, EntityKinematicState>;

    /// \brief Kinematic state for all entities in the world.
    struct WorldState
    {
      WorldKinematicState kinematics;
      gz::math::SphericalCoordinates origin;
    };

    /// \brief Doppler velocity log (DVL) sensor, built as a custom
    /// rendering sensor to leverage GPU shaders for speed.
    ///
    /// By default, this sensor uses an x-forward, y-left, z-up
    /// coordinate frame to report all measurements. All acoustic
    /// beams' lobes are expected to face downwards.
    ///
    /// \verbatim
    /// <sensor type="custom" gz:type="dvl">
    ///   <gz:dvl>
    ///     <arrangement degrees="true">
    ///       <beam id="1">
    ///         <aperture></aperture>
    ///         <rotation></rotation>
    ///         <tilt></tilt>
    ///       </beam>
    ///     </arrangement>
    ///     <tracking>
    ///       <bottom_mode>
    ///         <when></when>
    ///         <noise type="gaussian">
    ///           <stddev></stddev>
    ///         </noise>
    ///         <visualize></visualize>
    ///       </bottom_mode>
    ///       <water_mass_mode>
    ///         <when></when>
    ///         <water_velocity>
    ///           <x></x>
    ///           <y></y>
    ///           <z></z>
    ///         </water_velocity>
    ///         <boundaries>
    ///           <near>20.</near>
    ///           <far>60.</far>
    ///         </boundaries>
    ///         <bins>10</bins>
    ///         <noise type="gaussian">
    ///           <stddev></stddev>
    ///         </noise>
    ///         <visualize></visualize>
    ///       </water_mass_mode>
    ///     <minimum_range></minimum_range>
    ///     <maximum_range></maximum_range>
    ///     <resolution></resolution>
    ///     <reference_frame></reference_frame>
    ///   </gz:dvl>
    /// </sensor>
    /// \endverbatim
    ///
    /// - `<arrangement>` describes the arrangement of acoustic beams
    /// in the DVL sensor frame. It may include a `degrees` attribute
    /// to signal use of degrees instead of radians for all angles
    /// within, defaulting to radians if left unspecified.
    /// - `<arrangement><beam>` describes one acoustic beam in the
    /// arrangement. May include an `id` attribute, defaulting to the
    /// last specified id plus 1 if left unspecified (or 0 if it's the
    /// first).
    /// - `<arrangement><beam><aperture>` sets the aperture angle for
    /// the acoustic beam's main lobe (modelled as a cone). Defaults
    /// to 90 degrees if left unspecified.
    /// - `<arrangement><beam><rotation>` sets the rotation angle of
    /// the acoustic beam's symmetry axis about the sensor frame z
    /// axis. Defaults to 0 degrees if left unspecified.
    /// - `<arrangement><beam><tilt>` sets the inclination angle of
    /// the acoustic beam's symmetry axis w.r.t. the sensor frame -z
    /// axis (ie. rotation about the -y axis). Defaults to 0 degrees
    /// if left unspecified.
    /// - `<arrangement><visualize>` enables visual aids to evaluate
    /// acoustic beam arrangements. Beam lobes' are rendered and adjusted
    /// in size to match range measurements.
    /// - `<tracking>` configures velocity tracking modes for the DVL.
    /// - `<tracking><bottom_mode>` configures the bottom tracking mode.
    /// - `<tracking><bottom_mode><when>` enables (or disables) the bottom
    /// tracking mode. Supported values are 'never', to disable it completely
    /// (as if no <bottom_mode> configuration had been specified), 'always' to
    /// enable it at all times, and 'best' to track at all times but only
    /// publish estimates when it performs best among all configured modes.
    /// Defaults to 'always' if left unspecified.
    /// - `<tracking><bottom_mode><noise>` sets the noise model for velocity
    /// estimates. Only 'gaussian' noise is currently supported. Defaults to
    /// none if left unspecified.
    /// - `<tracking><bottom_mode><visualize>` enables visual aids to validate
    /// bottom tracking. Acoustic beam reflection paths are depicted, where the
    /// color scales linearly in hue with measured speed and low opacity sections
    /// depict range uncertainty (+/- 2 standard deviations).
    /// - `<tracking><water_mass_mode>` configures the water-mass tracking mode.
    /// - `<tracking><water_mass_mode><when>` enables (or disables) the water-mass
    /// tracking mode. Supported values are 'never', to disable it completely
    /// (as if no <water_mass_mode> configuration had been specified), 'always'
    /// to enable it at all times, and 'best' to track at all times but only
    /// publish estimates when it performs best among all configured modes.
    /// Defaults to 'always' if left unspecified.
    /// - `<tracking><water_mass_mode><water_velocity>` set the variables in world
    /// environmental data to be used to sample water velocity w.r.t. the world frame
    /// in each axis. At least one axis must be specified.
    /// - `<tracking><water_mass_mode><water_velocity><x>` set the variable in world
    /// environmental data to be used to sample water velocity w.r.t. the world frame
    /// along the x-axis (that is, towards east). Defaults to none (and thus zero
    /// water velocity in this axis) if left unspecified.
    /// - `<tracking><water_mass_mode><water_velocity><y>` set the variable in world
    /// environmental data to be used to sample water velocity w.r.t. the world frame
    /// along the y-axis (that is, towards north). Defaults to none (and thus zero
    /// water velocity in this axis) if left unspecified.
    /// - `<tracking><water_mass_mode><water_velocity><z>` set the variable in world
    /// environmental data to be used to sample water velocity w.r.t. the world frame
    /// along the z-axis (that is, upwards). Defaults to none (and thus zero
    /// water velocity in this axis) if left unspecified.
    /// - `<tracking><water_mass_mode><boundaries>` sets water-mass layer boundaries.
    /// These boundaries are planar at given z-offsets in the sensor frame.
    /// - `<tracking><water_mass_mode><boundaries><near>` sets the water-mass layer
    /// boundary that is the closest to the sensor.
    /// - `<tracking><water_mass_mode><boundaries><far>` sets the water-mass layer
    /// boundary that is the farthest to the sensor.
    /// - `<tracking><water_mass_mode><bins>` sets the number of bins to use for
    /// water-mass velocity sampling. Each bin is a slab of water between boundaries.
    /// - `<tracking><water_mass_mode><noise>` sets the noise model for velocity
    /// estimates. Only 'gaussian' noise is currently supported. Defaults to
    /// none if left unspecified.
    /// - `<tracking><water_mass_mode><visualize>` enables visual aids to validate
    /// bottom tracking. Acoustic beam reflection paths are depicted, where the
    /// color scales linearly in hue with measured speed and low opacity sections
    /// depict range uncertainty (+/- 2 standard deviations).
    /// - `<type>` sets the sensor type, either 'piston' or 'phased_array'.
    /// Defaults to unspecified.
    /// - `<resolution>` sets the resolution of the beam for bottom
    /// tracking at a 1 m distance. Defaults to 1 cm if left unspecified.
    /// - `<minimum_range>` sets a lower bound for range measurements.
    /// Defaults to 1 cm if left unspecified.
    /// - `<maximum_range>` sets an upper bound for range measurements.
    /// Defaults to 100 m if left unspecified.
    /// - `<reference_frame>` sets a transform from the sensor frame to the
    /// reference frame in which all measurements are reported. Defaults to
    /// the identity transform.
    ///
    /// Note the tethys::DopplerVelocityLogSystem plugin must be
    /// loaded for these custom sensors to be picked up and setup.
    class DopplerVelocityLog : public gz::sensors::RenderingSensor
    {
      public: DopplerVelocityLog();
    
      public: ~DopplerVelocityLog();
    
      /// Inherits documentation from parent class
      public: virtual bool Load(const sdf::Sensor &_sdf) override;
    
      /// Inherits documentation from parent class
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;
    
      /// Perform any sensor updates after the rendering pass
      public: virtual void PostUpdate(
        const std::chrono::steady_clock::duration &_now);
    
      /// Inherits documentation from parent class
      public: void SetScene(gz::rendering::ScenePtr _scene) override;
    
      /// \brief Set this sensor's entity ID (for world state lookup).
      public: void SetEntity(uint64_t entity);
    
      /// \brief Set world `_state` to support DVL water and bottom-tracking.
      public: void SetWorldState(const WorldState &_state);
    
      /// \brief Set environmental `_data` to support DVL water-tracking.
      public: void SetEnvironmentalData(const EnvironmentalData &_data);
    
      /// \brief Yield rendering sensors that underpin the implementation.
      ///
      /// \internal
      public: std::vector<gz::rendering::SensorPtr> RenderingSensors() const;
    
      private: class Implementation;
    
      private: std::unique_ptr<Implementation> dataPtr;
    };    
    
  }  // namespace sensors
}  // namespace gz
   
#endif // GZ_SENSORS_DOPPLERVELOCITYLOG_HH_
