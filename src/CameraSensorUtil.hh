/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#ifndef GZ_SENSORS_CAMERASENSORUTIL_HH_
#define GZ_SENSORS_CAMERASENSORUTIL_HH_

#include <gz/math/Matrix4.hh>

#include "gz/sensors/config.hh"
#include "gz/sensors/Export.hh"

namespace gz
{
namespace sensors
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SENSORS_VERSION_NAMESPACE {
/// \brief Computes the OpenGL NDC matrix
/// \param[in] _left Left vertical clipping plane
/// \param[in] _right Right vertical clipping plane
/// \param[in] _bottom Bottom horizontal clipping plane
/// \param[in] _top Top horizontal clipping plane
/// \param[in] _near Distance to the nearer depth clipping plane
///            This value is negative if the plane is to be behind
///            the camera
/// \param[in] _far Distance to the farther depth clipping plane
///            This value is negative if the plane is to be behind
///            the camera
/// \return OpenGL NDC (Normalized Device Coordinates) matrix
math::Matrix4d buildNDCMatrix(double _left, double _right, double _bottom,
                              double _top, double _near, double _far);

/// \brief Computes the OpenGL perspective matrix
/// \param[in] _intrinsicsFx Horizontal focal length (in pixels)
/// \param[in] _intrinsicsFy Vertical focal length (in pixels)
/// \param[in] _intrinsicsCx X coordinate of principal point in pixels
/// \param[in] _intrinsicsCy Y coordinate of principal point in pixels
/// \param[in] _intrinsicsS Skew coefficient defining the angle between
///            the x and y pixel axes
/// \param[in] _clipNear Distance to the nearer depth clipping plane
///            This value is negative if the plane is to be behind
///            the camera
/// \param[in] _clipFar Distance to the farther depth clipping plane
///            This value is negative if the plane is to be behind
///            the camera
/// \return OpenGL perspective matrix
math::Matrix4d buildPerspectiveMatrix(
    double _intrinsicsFx, double _intrinsicsFy, double _intrinsicsCx,
    double _intrinsicsCy, double _intrinsicsS, double _clipNear,
    double _clipFar);

/// \brief Computes the OpenGL projection matrix by multiplying
///        the OpenGL Normalized Device Coordinates matrix (NDC) with
///        the OpenGL perspective matrix
///        openglProjectionMatrix = ndcMatrix * perspectiveMatrix
/// \param[in] _imageWidth Image width (in pixels)
/// \param[in] _imageHeight Image height (in pixels)
/// \param[in] _intrinsicsFx Horizontal focal length (in pixels)
/// \param[in] _intrinsicsFy Vertical focal length (in pixels)
/// \param[in] _intrinsicsCx X coordinate of principal point in pixels
/// \param[in] _intrinsicsCy Y coordinate of principal point in pixels
/// \param[in] _intrinsicsS Skew coefficient defining the angle between
///             the x and y pixel axes
/// \param[in] _clipNear Distance to the nearer depth clipping plane
///            This value is negative if the plane is to be behind
///            the camera
/// \param[in] _clipFar Distance to the farther depth clipping plane
///            This value is negative if the plane is to be behind
///            the camera
/// \return OpenGL projection matrix
math::Matrix4d GZ_SENSORS_VISIBLE buildProjectionMatrix(
    double _imageWidth, double _imageHeight,
    double _intrinsicsFx, double _intrinsicsFy,
    double _intrinsicsCx, double _intrinsicsCy,
    double _intrinsicsS, double _clipNear,
    double _clipFar);
}
}
}

#endif
