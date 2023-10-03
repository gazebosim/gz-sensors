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

#include "CameraSensorUtil.hh"

#include <gz/math/Matrix4.hh>

//////////////////////////////////////////////////
gz::math::Matrix4d gz::sensors::buildNDCMatrix(
    double _left, double _right,
    double _bottom, double _top,
    double _near, double _far)
{
  double inverseWidth = 1.0 / (_right - _left);
  double inverseHeight = 1.0 / (_top - _bottom);
  double inverseDistance = 1.0 / (_far - _near);

  return math::Matrix4d(
           2.0 * inverseWidth,
           0.0,
           0.0,
           -(_right + _left) * inverseWidth,
           0.0,
           2.0 * inverseHeight,
           0.0,
           -(_top + _bottom) * inverseHeight,
           0.0,
           0.0,
           -2.0 * inverseDistance,
           -(_far + _near) * inverseDistance,
           0.0,
           0.0,
           0.0,
           1.0);
}

//////////////////////////////////////////////////
gz::math::Matrix4d gz::sensors::buildPerspectiveMatrix(
    double _intrinsicsFx, double _intrinsicsFy,
    double _intrinsicsCx, double _intrinsicsCy,
    double _intrinsicsS,
    double _clipNear, double _clipFar)
{
  return math::Matrix4d(
           _intrinsicsFx,
           _intrinsicsS,
           -_intrinsicsCx,
           0.0,
           0.0,
           _intrinsicsFy,
           -_intrinsicsCy,
           0.0,
           0.0,
           0.0,
           _clipNear + _clipFar,
           _clipNear * _clipFar,
           0.0,
           0.0,
           -1.0,
           0.0);
}

//////////////////////////////////////////////////
gz::math::Matrix4d gz::sensors::buildProjectionMatrix(
    double _imageWidth, double _imageHeight,
    double _intrinsicsFx, double _intrinsicsFy,
    double _intrinsicsCx, double _intrinsicsCy,
    double _intrinsicsS,
    double _clipNear, double _clipFar)
{
  return buildNDCMatrix(
           0, _imageWidth, 0, _imageHeight, _clipNear, _clipFar) *
         buildPerspectiveMatrix(
           _intrinsicsFx, _intrinsicsFy,
           _intrinsicsCx, _imageHeight - _intrinsicsCy,
           _intrinsicsS, _clipNear, _clipFar);
}
