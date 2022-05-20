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

#ifndef GZ_SENSORS_BROWNDISTORTIONMODEL_HH_
#define GZ_SENSORS_BROWNDISTORTIONMODEL_HH_

#include <sdf/sdf.hh>

#include "gz/sensors/Distortion.hh"
#include "gz/sensors/Export.hh"
#include "gz/sensors/config.hh"
#include "gz/utils/ImplPtr.hh"

namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {
    //
    // Forward declarations
    class BrownDistortionModelPrivate;

    /** \class BrownDistortionModel BrownDistortionModel.hh \
    gz/sensors/BrownDistortionModel.hh
    **/
    /// \brief Brown Distortion Model class
    class IGNITION_SENSORS_VISIBLE BrownDistortionModel : public Distortion
    {
      /// \brief Constructor.
      public: BrownDistortionModel();

      /// \brief Destructor.
      public: virtual ~BrownDistortionModel();

      // Documentation inherited.
      public: virtual void Load(const sdf::Camera &_sdf) override;

      /// \brief Get the radial distortion coefficient k1.
      /// \return Distortion coefficient k1.
      public: double K1() const;

      /// \brief Get the radial distortion coefficient k2.
      /// \return Distortion coefficient k2.
      public: double K2() const;

      /// \brief Get the radial distortion coefficient k3.
      /// \return Distortion coefficient k3.
      public: double K3() const;

      /// \brief Get the tangential distortion coefficient p1.
      /// \return Distortion coefficient p1.
      public: double P1() const;

      /// \brief Get the tangential distortion coefficient p2.
      /// \return Distortion coefficient p2.
      public: double P2() const;

      /// \brief Get the distortion center.
      /// \return Distortion center.
      public: math::Vector2d Center() const;

      /// Documentation inherited
      public: virtual void Print(std::ostream &_out) const override;

      /// \brief Private data pointer.
      IGN_UTILS_IMPL_PTR(dataPtr)
    };
  }
  }
}

#endif
