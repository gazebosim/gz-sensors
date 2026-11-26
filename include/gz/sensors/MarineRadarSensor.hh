/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#ifndef GZ_SENSORS_MARINERADARSENSOR_HH_
#define GZ_SENSORS_MARINERADARSENSOR_HH_

#include <cstdint>
#include <string>

#include <sdf/sdf.hh>

#include <gz/utils/ImplPtr.hh>

#include <gz/sensors/config.hh>
#include <gz/sensors/marine_radar/Export.hh>

#include "gz/sensors/RenderingSensor.hh"

namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief Rotating 2-D marine (X/S band) navigation radar sensor.
    ///
    /// The antenna rotates clockwise seen from above (the usual marine radar
    /// convention) and transmits one "spoke" per angular step. Every spoke is
    /// sampled with a fan of gravity-levelled rays, one per entry of
    /// `<slice_heights>`, cast with gz::rendering::RayQuery against the
    /// rendering scene. The geometric hits are turned into radar video by a
    /// simplified radar equation (SNR proportional to |cos incidence| / R^4
    /// with a Gaussian two-way beam pattern), a detection threshold, range
    /// quantisation and the configured noise, missed-echo and false-echo
    /// statistics. Echo cells are then clustered into targets with persistent
    /// IDs, and scan-converted into a Plan Position Indicator (PPI) image.
    ///
    /// SDFormat has no native `radar` sensor type, so this sensor is loaded
    /// as a custom sensor:
    ///
    /// \verbatim
    /// <sensor name="radar" type="custom" gz:type="marine_radar">
    ///   <topic>/marine_radar</topic>
    ///   <update_rate>60</update_rate>
    ///   <gz:marine_radar>
    ///     <range_min>10</range_min>
    ///     <range_max>1000</range_max>
    ///     <range_resolution>2</range_resolution>
    ///     <horizontal_fov_deg>360</horizontal_fov_deg>
    ///     <angular_resolution_deg>0.087890625</angular_resolution_deg>
    ///     <rotation_rate_rpm>24</rotation_rate_rpm>
    ///     <beamwidth_horizontal_deg>1.2</beamwidth_horizontal_deg>
    ///     <beamwidth_vertical_deg>20</beamwidth_vertical_deg>
    ///     <slice_heights>0.5 1.5 3.0 6.0 12.0 24.0</slice_heights>
    ///     <range_noise>
    ///       <noise type="gaussian">
    ///         <stddev>0.5</stddev>
    ///       </noise>
    ///     </range_noise>
    ///     <image_size>512</image_size>
    ///     <image_rate>5</image_rate>
    ///   </gz:marine_radar>
    /// </sensor>
    /// \endverbatim
    ///
    /// Every `<gz:marine_radar>` child that is not listed below is forwarded
    /// to the radar configuration parser. Angles are given in degrees (keys
    /// end in `_deg`), distances in metres and times in seconds. The full
    /// parameter set, with defaults and validation rules, is documented in
    /// the Marine Radar Sensor section of the README and in the sensor's
    /// source (src/marine_radar/RadarConfig.hh).
    ///
    /// Children handled by the sensor itself rather than by that parser:
    /// - `<publish_image>`: publish the PPI image. Default `true`.
    /// - `<publish_returns>`: publish the per-cell returns cloud. Default
    ///   `true`.
    /// - `<normal_estimation>`: `probe` (default) casts two extra probe rays
    ///   per hit so the surface normal, and therefore the incidence angle of
    ///   the echo, can be recovered from the rendering engine (which does
    ///   not report normals); `none` is cheaper but assumes normal
    ///   incidence for every hit.
    /// - `<normal_probe_angle_deg>`: angular offset of those probe rays.
    ///   Default 0.05 degrees.
    /// - `<range_noise>`, `<bearing_noise>`, `<intensity_noise>`: each holds
    ///   a nested standard SDFormat `<noise>` element, the way
    ///   `//altimeter/vertical_position` does. Only `gaussian` is supported;
    ///   the standard deviation feeds the radar echo model, which applies
    ///   range, bearing and SNR noise at different stages of the pipeline.
    ///   The bearing standard deviation is in degrees, the intensity one in
    ///   dB.
    ///
    /// Data is published on three topics derived from `<topic>` (default
    /// `/marine_radar`):
    /// - `<topic>/image`: gz::msgs::Image, the RGB_INT8 PPI display.
    /// - `<topic>/targets`: gz::msgs::PointCloudPacked, one point per
    ///   extracted target.
    /// - `<topic>/returns`: gz::msgs::PointCloudPacked, one point per echo
    ///   cell.
    ///
    /// The maximum range can be changed while the simulation runs by
    /// publishing a gz::msgs::Double on `<topic>/cmd/range_max`.
    class GZ_SENSORS_MARINE_RADAR_VISIBLE MarineRadarSensor :
      public RenderingSensor
    {
      /// \brief constructor
      public: MarineRadarSensor();

      /// \brief destructor
      public: virtual ~MarineRadarSensor();

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

      using Sensor::Update;

      /// \brief Update the sensor and generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;

      /// \brief Set the rendering scene. Ray queries are created from it.
      /// \param[in] _scene Pointer to the scene
      public: virtual void SetScene(rendering::ScenePtr _scene) override;

      /// \brief Check if there are any subscribers
      /// \return True if there are subscribers, false otherwise
      public: virtual bool HasConnections() const override;

      /// \brief Get the minimum (blind) range of the radar.
      /// \return Minimum range in meters
      public: double RangeMin() const;

      /// \brief Get the maximum instrumented range of the radar.
      /// \return Maximum range in meters
      public: double RangeMax() const;

      /// \brief Set the maximum instrumented range of the radar. The radar
      /// pipeline is restarted when the range changes. This is the same
      /// operation the `<topic>/cmd/range_max` topic performs.
      /// \param[in] _range New maximum range in meters
      /// \return True if the new range is valid and was applied
      public: bool SetRangeMax(double _range);

      /// \brief Get the range-bin (pulse length cell) size.
      /// \return Range resolution in meters
      public: double RangeResolution() const;

      /// \brief Get the transmitting sector width, centered on +X.
      /// \return Horizontal field of view in radians
      public: double HorizontalFOV() const;

      /// \brief Get the spoke spacing.
      /// \return Angular resolution in radians
      public: double AngularResolution() const;

      /// \brief Get the antenna rotation speed.
      /// \return Rotation rate in revolutions per minute
      public: double RotationRateRpm() const;

      /// \brief Get the number of spokes transmitted per scan.
      /// \return Spokes per scan
      public: unsigned int SpokesPerScan() const;

      /// \brief Get the number of range bins per spoke.
      /// \return Range bins per spoke
      public: unsigned int RangeBins() const;

      /// \brief Get the width and height of the PPI image.
      /// \return Image size in pixels
      public: unsigned int ImageSize() const;

      /// \brief Get the number of scans (antenna revolutions) completed
      /// since the sensor was loaded or last reset.
      /// \return Number of completed scans
      public: uint64_t ScanCount() const;

      /// \brief Get the number of targets extracted from the last completed
      /// scan.
      /// \return Number of targets
      public: unsigned int TargetCount() const;

      /// \brief Get the number of rays cast during the last update.
      /// \return Number of rays, probe rays included
      public: unsigned int RayCount() const;

      /// \brief Get the topic the PPI image is published on.
      /// \return Image topic
      public: std::string ImageTopic() const;

      /// \brief Get the topic the extracted targets are published on.
      /// \return Targets topic
      public: std::string TargetsTopic() const;

      /// \brief Get the topic the per-cell returns are published on.
      /// \return Returns topic
      public: std::string ReturnsTopic() const;

      /// \brief Forget all accumulated radar state, e.g. after a world
      /// reset. The configuration is kept.
      public: void Reset();

      GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
    };
    }
  }
}

#endif
