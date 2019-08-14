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

#include "PointCloudUtil.hh"

using namespace ignition;
using namespace sensors;

//////////////////////////////////////////////////
void PointCloudUtil::FillMsg(msgs::PointCloudPacked &_msg,
    const math::Angle &_hfov, const unsigned char *_imageData,
    const float *_depthData) const
{
  // Fill message. Logic borrowed from
  // https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_depth_camera.cpp

  uint32_t width = _msg.width();
  uint32_t height = _msg.height();

  std::string *msgBuffer = _msg.mutable_data();
  msgBuffer->resize(_msg.row_step() * _msg.height());
  char *msgBufferIndex = msgBuffer->data();

  // For depth calculation from image
  double fl = width / (2.0 * std::tan(_hfov.Radian() / 2.0));

  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < height; ++j)
  {
    float pAngle = 0.0;
    if (fl > 0 && height > 1)
      pAngle = std::atan2((height-j-1) - 0.5 * (height - 1), fl);

    for (uint32_t i = 0; i < width; ++i)
    {
      int fieldIndex = 0;

      // Current point depth
      float depth = _depthData[j * width + i];

      float yAngle = 0.0;
      if (fl > 0 && width > 1)
        yAngle = std::atan2(0.5 * (width - 1) - i, fl);

      *reinterpret_cast<float*>(msgBufferIndex +
          _msg.field(fieldIndex++).offset()) = depth;
      *reinterpret_cast<float*>(msgBufferIndex +
          _msg.field(fieldIndex++).offset()) =
        depth * std::tan(yAngle);
      *reinterpret_cast<float*>(msgBufferIndex +
          _msg.field(fieldIndex++).offset()) =
        depth * std::tan(pAngle);

      int imgIndex = i * 3 + j * width * 3;
      int fieldOffset = _msg.field(fieldIndex).offset();
      // Put image color data for each point, check endianess first.
      if (_msg.is_bigendian())
      {
        *(msgBufferIndex + fieldOffset + 0) = _imageData[imgIndex + 0];
        *(msgBufferIndex + fieldOffset + 1) = _imageData[imgIndex + 1];
        *(msgBufferIndex + fieldOffset + 2) = _imageData[imgIndex + 2];
      }
      else
      {
        *(msgBufferIndex + fieldOffset + 0) = _imageData[imgIndex + 2];
        *(msgBufferIndex + fieldOffset + 1) = _imageData[imgIndex + 1];
        *(msgBufferIndex + fieldOffset + 2) = _imageData[imgIndex + 0];
      }

      // Add any padding
      msgBufferIndex += _msg.point_step();
    }
  }
}

//////////////////////////////////////////////////
void PointCloudUtil::FillMsg(msgs::PointCloudPacked &_msg,
    const float *_pointCloudData,
    unsigned char *_imageData,
    float *_xyzData) const
{
  uint32_t width = _msg.width();
  uint32_t height = _msg.height();

  std::string *msgBuffer = _msg.mutable_data();
  msgBuffer->resize(_msg.row_step() * _msg.height());
  char *msgBufferIndex = msgBuffer->data();

  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < height; ++j)
  {
    int pcStep = j*width*4;
    int imgStep = j*width*3;
    for (uint32_t i = 0; i < width; ++i)
    {
      int pcIndex = pcStep + i*4;
      float x = _pointCloudData[pcIndex];
      float y = _pointCloudData[pcIndex + 1];
      float z = _pointCloudData[pcIndex + 2];
      float rgba = _pointCloudData[pcIndex + 3];

      int fieldIndex = 0;
      *reinterpret_cast<float*>(msgBufferIndex +
          _msg.field(fieldIndex++).offset()) = x;
      *reinterpret_cast<float*>(msgBufferIndex +
          _msg.field(fieldIndex++).offset()) = y;
      *reinterpret_cast<float*>(msgBufferIndex +
          _msg.field(fieldIndex++).offset()) = z;

      uint8_t r = 0u;
      uint8_t g = 0u;
      uint8_t b = 0u;
      uint8_t a = 255u;
      this->DecodeRGBAFromFloat(rgba, r, g, b, a);

      int fieldOffset = _msg.field(fieldIndex).offset();
      // Put image color data for each point, check endianess first.
      if (_msg.is_bigendian())
      {
        *(msgBufferIndex + fieldOffset + 0) = r;
        *(msgBufferIndex + fieldOffset + 1) = g;
        *(msgBufferIndex + fieldOffset + 2) = b;
      }
      else
      {
        *(msgBufferIndex + fieldOffset + 0) = b;
        *(msgBufferIndex + fieldOffset + 1) = g;
        *(msgBufferIndex + fieldOffset + 2) = r;
      }

      // Add any padding
      msgBufferIndex += _msg.point_step();

      // Fill buffers
      int imgIndex = imgStep + i * 3;
      if (_xyzData)
      {
        _xyzData[imgIndex + 0] = x;
        _xyzData[imgIndex + 1] = y;
        _xyzData[imgIndex + 2] = z;
      }
      if (_imageData)
      {
        _imageData[imgIndex + 0] = static_cast<unsigned char>(r);
        _imageData[imgIndex + 1] = static_cast<unsigned char>(g);
        _imageData[imgIndex + 2] = static_cast<unsigned char>(b);
      }
    }
  }
}


//////////////////////////////////////////////////
void PointCloudUtil::RGBImageFromPointCloud(unsigned char *_imageData,
    const float *_pointCloudData, unsigned int _width, unsigned int _height)
    const
{
  // Iterate over scan and populate image data
  for (uint32_t j = 0; j < _height; ++j)
  {
    int pcStep = j*_width*4;
    int imgStep = j*_width*3;
    for (uint32_t i = 0; i < _width; ++i)
    {
      int pcIndex = pcStep + i*4;
      float rgba = _pointCloudData[pcIndex + 3];
      uint8_t r = 0u;
      uint8_t g = 0u;
      uint8_t b = 0u;
      uint8_t a = 255u;
      this->DecodeRGBAFromFloat(rgba, r, g, b, a);
      int imgIndex = imgStep + i * 3;
      _imageData[imgIndex + 0] = static_cast<unsigned char>(r);
      _imageData[imgIndex + 1] = static_cast<unsigned char>(g);
      _imageData[imgIndex + 2] = static_cast<unsigned char>(b);
    }
  }
}

//////////////////////////////////////////////////
void PointCloudUtil::DecodeRGBAFromFloat(float _rgba,
  uint8_t &_r, uint8_t &_g, uint8_t &_b, uint8_t &_a) const
{
  uint32_t *rgba = reinterpret_cast<uint32_t *>(&_rgba);
  _r = static_cast<uint8_t>(*rgba >> 24 & 0xFF);
  _g = static_cast<uint8_t>(*rgba >> 16 & 0xFF);
  _b = static_cast<uint8_t>(*rgba >> 8 & 0xFF);
  _a = static_cast<uint8_t>(*rgba >> 0 & 0xFF);
}
