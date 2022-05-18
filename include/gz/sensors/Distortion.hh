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

#ifndef GZ_SENSORS_DISTORTION_HH_
#define GZ_SENSORS_DISTORTION_HH_

#include <functional>
#include <string>
#include <vector>

#include <gz/sensors/Export.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/sensors/config.hh>
#include <gz/utils/ImplPtr.hh>

#include <sdf/sdf.hh>

namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    // Forward declarations
    class DistortionPrivate;

    /// \class DistortionFactory Distortion.hh gz/sensors/Distortion.hh
    /// \brief Use this distortion manager for creating and loading distortion
    /// models.
    class IGNITION_SENSORS_VISIBLE DistortionFactory
    {
      /// \brief Load a distortion model based on the input sdf parameters and
      /// sensor type.
      /// \param[in] _sdf Distortion sdf parameters.
      /// \param[in] _sensorType Type of sensor. This is currently used to
      /// distinguish between image and non image sensors in order to create
      /// the appropriate distortion model.
      /// \return Pointer to the distortion model created.
      public: static DistortionPtr NewDistortionModel(sdf::ElementPtr _sdf,
          const std::string &_sensorType = "");

      /// \brief Load a distortion model based on the input sdf parameters and
      /// sensor type.
      /// \param[in] _sdf Distortion sdf parameters.
      /// \param[in] _sensorType Type of sensor. This is currently used to
      /// distinguish between image and non image sensors in order to create
      /// the appropriate distortion model.
      /// \return Pointer to the distortion model created.
      public: static DistortionPtr NewDistortionModel(
                  const sdf::Camera &_sdf,
                  const std::string &_sensorType = "");
    };

    /// \brief Which distortion types we support
    enum class IGNITION_SENSORS_VISIBLE DistortionType : int
    {
      NONE = 0,
      CUSTOM = 1,
      BROWN = 2
    };

    /// \class Distortion Distortion.hh gz/sensors/Distortion.hh
    /// \brief Distortion models for sensor output signals.
    class IGNITION_SENSORS_VISIBLE Distortion
    {
      /// \brief Constructor. This should not be called directly unless creating
      /// an empty distortion model. Use DistortionFactory::NewDistortionModel
      /// to instantiate a new distortion model.
      /// \param[in] _type Type of distortion model.
      /// \sa DistortionFactory::NewDistortionModel
      public: explicit Distortion(DistortionType _type);

      /// \brief Destructor.
      public: virtual ~Distortion();

      /// \brief Load distortion parameters from sdf.
      /// \param[in] _sdf SDF Distortion DOM object.
      public: virtual void Load(const sdf::Camera &_sdf);

      /// \brief Accessor for DistortionType.
      /// \return Type of distortion currently in use.
      public: DistortionType Type() const;

      /// \brief Output information about the distortion model.
      /// \param[in] _out Output stream
      public: virtual void Print(std::ostream &_out) const;

      /// \brief Private data pointer
      IGN_UTILS_IMPL_PTR(dataPtr)
    };
    }
  }
}
#endif
