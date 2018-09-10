/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef IGNITION_SENSORS_CAMERACONFIG_HH_
#define IGNITION_SENSORS_CAMERACONFIG_HH_

#include <memory>
#include <string>

#include <ignition/common/Image.hh>
#include <ignition/msgs.hh>
#include <sdf/sdf.hh>
#include <ignition/sensors/Export.hh>

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE
    {
    /// \brief Forward declaration
    class CameraConfigPrivate;

    /// TODO link to save_image example, since that uses CameraConfig
    /// \brief Hold information for configuring camera sensors
    ///
    ///   This class is meant to make it easier to create a camera sensor.
    ///   To use it, populate the class with the desired configuration. Then
    ///   call ToSDF() to get sdformat representing that configuration. Finally
    ///   pass the sdf element to Manager::Load() to create a camera sensor.
    class IGNITION_SENSORS_VISIBLE CameraConfig
    {
      /// \brief Constructor with initial values
      /// \param[in] _name Name of the camera sensor
      /// \param[in] _topic ignition-transport topic the camera will publish to
      /// \param[in] _hz Rate in Hz that the camera should generate images
      /// \param[in] _width Width of the image in pixels
      /// \param[in] _height Height of the image in pixels
      /// \param[in] _hfov Horizontal field of view in radians
      /// \param[in] _near Near clip plane distance
      /// \param[in] _far Far clip plane distance
      /// \param[in] _format How pixel data is formatted
      public: CameraConfig(const std::string &_name,
        const std::string &_topic, double _hz, std::size_t _width,
        std::size_t _height, double _hfov, double _near, double _far,
        common::Image::PixelFormatType _format);

      /// \brief Constructor
      public: CameraConfig();

      /// \brief Destructor
      public: ~CameraConfig();

      /// \brief Set the name of the camera
      /// \param[in] _name a name for the camera
      /// \return true if the name is legal
      /// \sa Name()
      public: bool SetName(const std::string &_name);

      /// \brief Get the name of the camera
      ///
      ///   The name is used to create a topic if the topic is not set.
      /// \return name or empty string if it is not set
      //// \sa SetName()
      public: const std::string &Name() const;

      /// \brief Set the topic the camera will publish to
      /// \param[in] _topic an ignition transport topic name.
      /// \return true if the topic name is legal
      /// \sa Topic()
      public: bool SetTopic(const std::string &_topic);

      /// \brief Get the topic the camera publishes to
      ///
      ///   The topic is an ignition tranpsport topic to which the camera will
      ///   publish images.
      /// \return the topic the camera will publish to
      /// \sa SetTopic()
      public: const std::string &Topic() const;

      /// \brief Set number of frames per second
      /// \param[in] _hz images per second to generate.
      /// \return true if the value is > 0
      /// \sa Hz()
      public: bool SetHz(double _hz);

      /// \brief The number of frames per second
      ///
      ///   This is the rate at which the camera sensor will generate images.
      ///   The maximum rate is limited by the rate of calls to
      ///   ignition::sensors::Manager::Update(). A camera with a Hz of 2 will
      ///   only generate 1 frame per second if the manager is only updated
      ///   once every simulated second.
      /// \sa SetHz()
      public: double Hz() const;

      /// \brief Set the width of the output image
      /// \param[in] The width of the output image in pixels
      /// \returns true if the width is > 0
      /// \sa Width()
      public: bool SetWidth(std::size_t _width);

      /// \brief Get the width of the output image
      /// \returns width of the image in pixels
      /// \sa SetWidth(), Height(), SetHeight()
      public: std::size_t Width() const;

      /// \brief Set the height of the output image
      /// \param[in] The height of the output image in pixels
      /// \returns true if the height is > 0
      /// \sa Height()
      public: bool SetHeight(std::size_t _height);

      /// \brief Get the height of the output image
      /// \returns height of the image in pixels
      /// \sa SetHeight(), Width(), SetWidth()
      public: std::size_t Height() const;

      /// \brief set the horizontal field of view
      /// \param[in] _hfov horizontal field of view in radians
      /// \returns true if the provided value is between 0 and PI/2
      /// \sa HorizontalFOV()
      public: bool SetHorizontalFOV(double _hfov);

      /// \brief Get the horizontal field of view
      ///
      ///   The horizontal field is an angle in radians that defines how wide
      ///   of an area the camera can see.
      /// \returns angle in radians
      /// \sa SetHorizontalFOV()
      public: double HorizontalFOV() const;

      /// \brief Get the near clip plane distance
      ///
      ///   The near clip plane distance is the distance from the origin of the
      ///   camera to the closest point on the near clip plane. Any object
      ///   between the camera origin and this plane will be culled. Very small
      ///   values for near clip distance may result in "z-fighting" or
      ///   "stitching". How severe this will be depends on how close the
      ///   fighting objects are to each other, how far away the objects are
      ///   from the camera, and the number of bits the z-buffer has on a given
      ///   graphics card.
      ///
      ///   This effect gets worse the further away the objects are from the
      ///   camera, so the far clip plane distance should be set to cull any
      ///   objects at a range where this becomes a problem.
      /// \returns distance in meters
      /// \sa SetClip(), Far()
      public: double Near() const;

      /// \brief Set clip plane distances
      /// \param[in] _near distance from camera origin in meters to the closest
      ///   point on the near clip plane.
      /// \param[in] _far distance from camera origin in meters to the closest
      ///   point on the far clip plane
      /// \returns true if the provided values are legal
      /// \sa Far(), Near()
      public: bool SetClip(double _near, double _far);

      /// \brief Get the far clip plane distance
      /// \returns distance in meters
      /// \sa SetClip(), Near()
      public: double Far() const;

      /// \brief Get the vertical field of view
      ///
      ///   The vertical field of view is an angle in radians that defines how
      ///   tall of an area the camera can see.
      /// \remarks The vertical FOV is calculated from width, height, hfov
      /// \returns angle in radians
      public: double VerticalFOV() const;

      /// \brief Set the pixel format output by the camera
      /// \param[in] _format the pixel format
      /// \return true if the format is valid
      public: bool SetFormat(common::Image::PixelFormatType _format);

      /// \brief Get the pixel format output by the camera
      /// \return pixel format enum
      public: common::Image::PixelFormatType Format() const;

      /// \brief Convert camera configuration to SDFormat
      ///
      ///   This populates a `<sensor>` element with a `<camera>` tag. These can
      ///   then be passed to ignition::sensors::Manager::Load() to create a
      ///   camera sensor with the given settings.
      /// \returns An SDF Element pointer with camera data
      public: sdf::ElementPtr ToSDF();

      /// \internal
      private: std::unique_ptr<CameraConfigPrivate> dataPtr;
    };
    }
  }
}

#endif
