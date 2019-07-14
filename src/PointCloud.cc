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

#include "PointCloud.hh"

using namespace ignition;
using namespace sensors;

//////////////////////////////////////////////////
void PointCloud::FillMsg(msgs::PointCloudPacked &_msg,
    const math::Angle &_hfov, unsigned char *_imageData, float *_depthBuffer)
{
  // Fill message. Logic borrowed from
  // https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_depth_camera.cpp

  uint32_t width = _msg.width();
  uint32_t height = _msg.height();

  std::string *msgBuffer = _msg.mutable_data();
  msgBuffer->resize(_msg.row_step());
  char *msgBufferIndex = msgBuffer->data();

  // For depth calculation from image
  double fl = width / (2.0 * std::tan(_hfov.Radian() / 2.0));

  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < height; ++j)
  {
    float pAngle = 0.0;
    if (fl > 0 && height > 1)
      pAngle = std::atan2((height-j) - 0.5 * (height - 1), fl);

    for (uint32_t i = 0; i < width; ++i)
    {
      // Current point depth
      float depth = _depthBuffer[j * width + i];

      float yAngle = 0.0;
      if (fl > 0 && width > 1)
        yAngle = std::atan2(i - 0.5 * (width - 1), fl);

      // cppcheck-suppress invalidPointerCast
      *reinterpret_cast<float*>(msgBufferIndex) = depth;
      msgBufferIndex += 4;
      // cppcheck-suppress invalidPointerCast
      *reinterpret_cast<float*>(msgBufferIndex) = depth * std::tan(yAngle);
      msgBufferIndex += 4;
      // cppcheck-suppress invalidPointerCast
      *reinterpret_cast<float*>(msgBufferIndex) =  depth * std::tan(pAngle);
      msgBufferIndex += 4;

      // Add an extra 4 bytes for ROS1
      if (this->ros1ByteBoundary)
        msgBufferIndex += 4;

      int imgIndex = i * 3 + j * width * 3;
      // Put image color data for each point, check endianess first.
      if (_msg.is_bigendian())
      {
        *(msgBufferIndex++) = _imageData[imgIndex + 0];
        *(msgBufferIndex++) = _imageData[imgIndex + 1];
        *(msgBufferIndex++) = _imageData[imgIndex + 2];
      }
      else
      {
        *(msgBufferIndex++) = _imageData[imgIndex + 2];
        *(msgBufferIndex++) = _imageData[imgIndex + 1];
        *(msgBufferIndex++) = _imageData[imgIndex + 2];
      }

      // Pad the end for ROS1
      if (this->ros1ByteBoundary)
        msgBufferIndex += 13;
    }
  }
}
