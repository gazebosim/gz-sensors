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

#ifndef IGNITION_SENSORS_DEPTHIMAGE2POINTS_HH_
#define IGNITION_SENSORS_DEPTHIMAGE2POINTS_HH_

#include <ignition/msgs/pointcloud_packed.pb.h>
#include <ignition/math/Angle.hh>

#include "ignition/sensors/config.hh"
#include "ignition/sensors/Export.hh"

#ifndef _WIN32
#  define DepthImage2Points_EXPORTS_API
#else
#  if (defined(DepthPoints_EXPORTS))
#    define DepthImage2Points_EXPORTS_API __declspec(dllexport)
#  else
#    define DepthImage2Points_EXPORTS_API __declspec(dllimport)
#  endif
#endif

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {

    /// \brief Helper class the fills a msgs::PointCloudPacked message using
    /// image and depth data. The RgbdCameraSensor and DepthCameraSensor
    /// class use this.
    class DepthImage2Points_EXPORTS_API DepthImage2Points
    {
      /// \brief Fill a msgs::PointCloudPacked.
      /// \param[in,out] _msg Point cloud message to fill. This message
      /// should be initialized. See example usage in either
      /// RgbdCameraSensor and DepthCameraSensor
      /// \param[in] _hfov Horizontal field of view of the camera that
      /// generated the image and depth data.
      /// \param[in] _imageData RGB Image data.
      /// \param[in] _depthData Depth image data.
      public: void FillMsg(msgs::PointCloudPacked &_msg,
          const math::Angle &_hfov, const unsigned char *_imageData,
          const float *_depthData) const;
    };
    }
  }
}
#endif
