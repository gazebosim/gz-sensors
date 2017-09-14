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
#ifndef IGNITION_SENSORS_CAMERA_IMAGESAVER_HH_
#define IGNITION_SENSORS_CAMERA_IMAGESAVER_HH_

#include <cstdint>
#include <memory>
#include <string>

#include <ignition/common/Image.hh>
#include <ignition/msgs.hh>

namespace ignition
{
  namespace sensors
  {
    /// \internal
    /// \brief Manage saving images to disk
    ///
    ///   This class saves camera images to disk. It creates a directory to
    ///   contain the images, and then saves each image with a prefix and
    ///   increasing counter in the file name.
    class ImageSaver
    {
      /// \brief Constructor.
      /// \param[in] _path The path to save images.
      /// \param[in] _prefix Beginning of the saved image filename.
      public: ImageSaver(const std::string &_path, const std::string &_prefix);

      /// \brief destructor
      public: ~ImageSaver();

      /// \brief Save an image
      /// \param[in] _data the image data to be saved
      /// \param[in] _width width of image in pixels
      /// \param[in] _height height of image in pixels
      /// \param[in] _format The format the data is in
      /// \return True if the image was saved successfully. False can mean
      /// that the path provided to the constructor does exist and creation
      /// of the path was not possible.
      /// \sa ImageSaver
      public: bool SaveImage(const unsigned char *_data, unsigned int _width,
          unsigned int _height, common::Image::PixelFormatType _format);

      /// \brief counter used to set the image filename
      private: std::uint64_t counter = 0;

      /// \brief path directory to where images are saved
      private: std::string path;

      /// \prefix of an image name
      private: std::string prefix;
    };
  }
}

#endif

