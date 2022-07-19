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

#ifndef GZ_SENSORS_IMAGEBROWNDISTORTIONMODEL_HH_
#define GZ_SENSORS_IMAGEBROWNDISTORTIONMODEL_HH_

#include <sdf/sdf.hh>

// TODO(WilliamLewww): Remove these pragmas once gz-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <gz/rendering/Camera.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include "gz/sensors/config.hh"
#include "gz/sensors/BrownDistortionModel.hh"
#include "gz/sensors/rendering/Export.hh"

namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {
    // Forward declarations
    class ImageBrownDistortionModelPrivate;

    /** \class ImageBrownDistortionModel ImageBrownDistortionModel.hh \
    gz/sensors/ImageBrownDistortionModel.hh
    **/
    /// \brief Distortion Model class for image sensors
    class GZ_SENSORS_RENDERING_VISIBLE ImageBrownDistortionModel :
      public BrownDistortionModel
    {
      /// \brief Constructor.
      public: ImageBrownDistortionModel();

      /// \brief Destructor.
      public: virtual ~ImageBrownDistortionModel();

      // Documentation inherited.
      public: virtual void Load(const sdf::Camera &_sdf) override;

      // Documentation inherited.
      public: virtual void SetCamera(rendering::CameraPtr _camera);

      /// Documentation inherited
      public: virtual void Print(std::ostream &_out) const override;

      /// \brief Private data pointer.
      private: ImageBrownDistortionModelPrivate *dataPtr = nullptr;
    };
  }
  }
}

#endif
