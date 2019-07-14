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
#ifndef IGNITION_SENSORS_POINTCLOUD_HH_
#define IGNITION_SENSORS_POINTCLOUD_HH_

#include <ignition/msgs/pointcloud_packed.pb.h>
#include <ignition/math/Angle.hh>

#include "ignition/sensors/config.hh"
#include "ignition/sensors/Export.hh"

#ifndef _WIN32
#  define PointCloud_EXPORTS_API
#else
#  if (defined(PointCloud_EXPORTS))
#    define PointCloud_EXPORTS_API __declspec(dllexport)
#  else
#    define PointCloud_EXPORTS_API __declspec(dllimport)
#  endif
#endif


namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
      class PointCloud_EXPORTS_API PointCloud
      {
        public: void FillMsg(msgs::PointCloudPacked &_msg,
                    const math::Angle &_hfov, unsigned char *_imageData,
                    float *_depthBuffer);

        /// \brief ROS1 requires the XYZ and RGB data in the point cloud
        // message to occur at 16 byte boundaries.
        /// See: https://github.com/ros/common_msgs/pull/77
        /// \todo(anyone) This value should be set from SDF, and the default
        /// should be false. The message size will be roughly have the size if
        /// the point cloud message has no padding.
        public: bool ros1ByteBoundary{true};
      };
    }
  }
}

#endif
