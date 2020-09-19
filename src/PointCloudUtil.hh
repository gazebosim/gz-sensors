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

#ifndef IGNITION_SENSORS_POINTCLOUDUTIL_HH_
#define IGNITION_SENSORS_POINTCLOUDUTIL_HH_

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4005)
#endif
#include <ignition/msgs/pointcloud_packed.pb.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif
#include <ignition/math/Angle.hh>

#include "ignition/sensors/config.hh"
#include "ignition/sensors/Export.hh"

#ifndef _WIN32
#  define PointCloudUtil_EXPORTS_API
#else
#  if (defined(DepthPoints_EXPORTS))
#    define PointCloudUtil_EXPORTS_API __declspec(dllexport)
#  else
#    define PointCloudUtil_EXPORTS_API __declspec(dllimport)
#  endif
#endif

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief Helper class that fills a msgs::PointCloudPacked message using
    /// image and depth data. The RgbdCameraSensor and DepthCameraSensor
    /// class use this.
    class PointCloudUtil_EXPORTS_API PointCloudUtil
    {
      /// \brief Fill a msgs::PointCloudPacked.
      /// \param[in,out] _msg Point cloud message to fill. This message
      /// should be initialized. See example usage in either
      /// RgbdCameraSensor and DepthCameraSensor.
      /// \param[in] _hfov Horizontal field of view of the camera that
      /// generated the image and depth data.
      /// \param[in] _imageData RGB Image data.
      /// \param[in] _depthData Depth image data.
      public: void FillMsg(msgs::PointCloudPacked &_msg,
          const math::Angle &_hfov, const unsigned char *_imageData,
          const float *_depthData) const;

      /// \brief Fill a msgs::PointCloudPacked.
      /// \param[in,out] _msg Point cloud message to fill. This message
      /// should be initialized. See example usage in either
      /// RgbdCameraSensor and DepthCameraSensor.
      /// \param[in] _xyzData XYZ data.
      /// \param[in] _imageData RGB data.
      public: void FillMsg(msgs::PointCloudPacked &_msg,
          const float *_xyzData, const unsigned char *_imageData) const;

      /// \brief Fill a msgs::PointCloudPacked.
      /// \param[in,out] _msg Point cloud message to fill. This message
      /// should be initialized. See example usage in either
      /// RgbdCameraSensor and DepthCameraSensor.
      /// \param[in] _pointCloudData Point cloud XYZ RGB data.
      /// \param[in] _writeToBuffers If true, writes data to
      /// RGB (_imageData) and XYZ (_xyzData) buffers.
      /// \param[out] _imageData Fill _imageData wth RGB data extracted
      /// from _pointCloudData.
      /// \param[out] _xyzData Fill _xyzData wth XYZ data extracted
      /// from _pointCloudData.
      public: void FillMsg(msgs::PointCloudPacked &_msg,
          const float *_pointCloudData, bool _writeToBuffers = false,
          unsigned char *_imageData = 0, float *_xyzData = 0) const;

      /// \brief Extract RGB data from point cloud data
      /// \param[out] _imageData RGB Image buffer to be filled.
      /// \param[in] _pointCloudData Point cloud XYZ data.
      /// \param[in] _width Image width
      /// \param[in] _height Image height
      public: void RGBFromPointCloud(unsigned char *_imageData,
          const float *_pointCloudData, unsigned int _width,
          unsigned int _height) const;

      /// \brief Extract XYZ data from point cloud data
      /// \param[out] _xyzData XYZ buffer to be filled.
      /// \param[in] _pointCloudData Point cloud XYZ data.
      /// \param[in] _width Image width
      /// \param[in] _height Image height
      public: void XYZFromPointCloud(float *_xyzData,
          const float *_pointCloudData, unsigned int _width,
          unsigned int _height) const;

      /// \brief Decode/unpack RGBA values from a floating point value.
      /// Point cloud data is encoded as [X, Y, Z, RGBA], with all four fields
      /// in 32 bit float format. This function helps to unpack the last field
      /// to individual 8 bit unsigned int R, G, B, A values.
      /// \param[in] _rgba floating point representation of rgba
      /// \param[out] _r Red [0-255]
      /// \param[out] _g Green [0-255]
      /// \param[out] _b Blue [0-255]
      /// \param[out] _a Alpha [0-255]
      public: void DecodeRGBAFromFloat(float _rgba, uint8_t &_r, uint8_t &_g,
          uint8_t &_b, uint8_t &_a) const;
    };
    }
  }
}
#endif
