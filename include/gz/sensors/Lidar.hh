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
#ifndef GZ_SENSORS_LIDAR_HH_
#define GZ_SENSORS_LIDAR_HH_

#include <memory>
#include <string>
#include <vector>

#include <gz/common/SuppressWarning.hh>
#include <gz/common/Event.hh>

#include "gz/sensors/lidar/Export.hh"
#include "gz/sensors/RenderingSensor.hh"

namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief forward declarations
    class LidarPrivate;

    /// \brief Lidar Sensor Class
    ///
    ///   This class creates laser scans using. It's measures the range
    ///   from the origin of the center to points on the visual geometry in the
    ///   scene.
    ///
    ///   It offers both an ignition-transport interface and a direct C++ API
    ///   to access the image data. The API works by setting a callback to be
    ///   called with image data.
    class IGNITION_SENSORS_LIDAR_VISIBLE Lidar : public RenderingSensor
    {
      /// \brief constructor
      public: Lidar();

      /// \brief destructor
      public: virtual ~Lidar();

      /// \brief Force the sensor to generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(const common::Time &_now) override;

      /// \brief Apply noise to the laser buffer, if noise has been
      /// configured. This should be called before PublishLidarScan if you
      /// want the scan data to contain noise.
      public: void ApplyNoise();

      /// \brief Publish LaserScan message
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool PublishLidarScan(const common::Time &_now);

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

      /// \brief Initialize values in the sensor
      /// \return True on success
      public: void SetParent(const std::string &_parent) override;

      /// \brief Create Lidar sensor
      public: virtual bool CreateLidar();

      /// \brief Finalize the ray
      protected: virtual void Fini();

      /// \brief Get the minimum angle
      /// \return The minimum angle
      public: gz::math::Angle AngleMin() const;

      /// \brief Set the scan minimum angle
      /// \param[in] _angle The minimum angle
      public: void SetAngleMin(const double _angle);

      /// \brief Get the maximum angle
      /// \return the maximum angle
      public: gz::math::Angle AngleMax() const;

      /// \brief Set the scan maximum angle
      /// \param[in] _angle The maximum angle
      public: void SetAngleMax(const double _angle);

      /// \brief Get radians between each range
      /// \return Return angle resolution
      public: double AngleResolution() const;

      /// \brief Get the minimum range
      /// \return The minimum range
      public: double RangeMin() const;

      /// \brief Get the maximum range
      /// \return The maximum range
      public: double RangeMax() const;

      /// \brief Get the range resolution
      ///      If RangeResolution is 1, the number of simulated rays is equal
      ///      to the number of returned range readings. If it's less than 1,
      ///      fewer simulated rays than actual returned range readings are
      ///      used, the results are interpolated from two nearest neighbors,
      ///      and vice versa.
      /// \return The Range Resolution
      public: double RangeResolution() const;

      /// \brief Get the ray count
      /// \return The number of rays
      public: unsigned int RayCount() const;

      /// \brief Get the range count
      /// \return The number of ranges
      public: unsigned int RangeCount() const;

      /// \brief Get the vertical scan line count
      /// \return The number of scan lines vertically
      public: unsigned int VerticalRayCount() const;

      /// \brief Get the vertical scan line count
      /// \return The number of scan lines vertically
      public: unsigned int VerticalRangeCount() const;

      /// \brief Get the vertical scan bottom angle
      /// \return The minimum angle of the scan block
      public: gz::math::Angle VerticalAngleMin() const;

      /// \brief Set the vertical scan bottom angle
      /// \param[in] _angle The minimum angle of the scan block
      public: void SetVerticalAngleMin(const double _angle);

      /// \brief Get the vertical scan line top angle
      /// \return The Maximum angle of the scan block
      public: gz::math::Angle VerticalAngleMax() const;

      /// \brief Set the vertical scan line top angle
      /// \param[in] _angle The Maximum angle of the scan block
      public: void SetVerticalAngleMax(const double _angle);

      /// \brief Get the vertical angle in radians between each range
      /// \return Resolution of the angle
      public: double VerticalAngleResolution() const;

      /// \brief Get detected range for a ray.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \param[in] _index Index of specific ray
      /// \return Returns RangeMax for no detection.
      public: double Range(const int _index) const;

      /// \brief Get all the ranges
      /// \param[out] _range A vector that will contain all the range data
      public: void Ranges(std::vector<double> &_ranges) const;

      /// \brief Get detected retro (intensity) value for a ray.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \param[in] _index Index of specific ray
      /// \return Intensity value of ray
      public: double Retro(const int _index) const;

      /// \brief Get detected fiducial value for a ray.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \param[in] _index Index of specific ray
      /// \return Fiducial value of ray
      public: int Fiducial(const unsigned int _index) const;

      /// \brief Gets if sensor is horizontal
      /// \return True if horizontal, false if not
      public: bool IsHorizontal() const;

      /// \brief Return the ratio of horizontal ray count to vertical ray
      /// count.
      ///
      /// A ray count is the number of simulated rays. Whereas a range count
      /// is the total number of data points returned. When range count
      /// != ray count, then values are interpolated between rays.
      public: double RayCountRatio() const;

      /// \brief Return the ratio of horizontal range count to vertical
      /// range count.
      ///
      /// A ray count is the number of simulated rays. Whereas a range count
      /// is the total number of data points returned. When range count
      /// != ray count, then values are interpolated between rays.
      public: double RangeCountRatio() const;

      /// \brief Get the horizontal field of view of the laser sensor.
      /// \return The horizontal field of view of the laser sensor.
      public: double HorzFOV() const;

      /// \brief Get the vertical field-of-view.
      /// \return Vertical field of view.
      public: double VertFOV() const;

      // Documentation inherited
      public: virtual bool IsActive() const;

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Just a mutex for thread safety
      public: mutable std::mutex lidarMutex;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

      /// \brief Raw buffer of laser data.
      public: float *laserBuffer = nullptr;

      /// \brief true if Load() has been called and was successful
      public: bool initialized = false;

      /// \brief Set a callback to be called when data is generated.
      /// \param[in] _callback This callback will be called every time the
      /// sensor generates data. The Update function will be blocked while the
      /// callbacks are executed.
      /// \remark Do not block inside of the callback.
      /// \return A connection pointer that must remain in scope. When the
      /// connection pointer falls out of scope, the connection is broken.
      public: virtual gz::common::ConnectionPtr ConnectNewLidarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _heighti, unsigned int _channels,
                  const std::string &/*_format*/)> _subscriber);

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<LidarPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
