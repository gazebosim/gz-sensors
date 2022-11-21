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
#ifndef GZ_SENSORS_ENVIRONMENTAL_DATA_HH_
#define GZ_SENSORS_ENVIRONMENTAL_DATA_HH_

#include <memory>

#include <gz/common/DataFrame.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/TimeVaryingVolumetricGrid.hh>

#include <gz/sensors/Export.hh>

namespace gz
{
  namespace sensors
  {
    /// \brief Environment data across time and space. This is useful to
    /// introduce physical quantities that may be of interest even if not
    /// modelled in simulation.
    struct GZ_SENSORS_VISIBLE EnvironmentalData
    {
      using T = math::InMemoryTimeVaryingVolumetricGrid<double>;
      using FrameT = common::DataFrame<std::string, T>;
      using ReferenceT = math::SphericalCoordinates::CoordinateType;
  
      /// \brief Reference units
      enum class ReferenceUnits {
        RADIANS = 0,
        DEGREES
      };
  
      /// \brief Instantiate environmental data.
      ///
      /// An std::make_shared equivalent that ensures
      /// dynamically loaded call sites use a template
      /// instantiation that is guaranteed to outlive
      /// them.
      static std::shared_ptr<EnvironmentalData>
      MakeShared(FrameT _frame, ReferenceT _reference,
        ReferenceUnits _units = ReferenceUnits::RADIANS,
        bool _ignoreTimeStep = false);
  
      /// \brief Environmental data frame.
      FrameT frame;
  
      /// \brief Spatial reference for data coordinates.
      ReferenceT reference;
  
      /// \brief The units to be used (only for spherical coordinates)
      ReferenceUnits units;
  
      /// \brief Use time axis or not.
      bool staticTime;
    };
  }  // namespace sensors
}  // namespace gz

#endif 