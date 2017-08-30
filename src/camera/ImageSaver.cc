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

#include "src/camera/ImageSaver.hh"

#include <string>

#include <ignition/common/Filesystem.hh>

using namespace ignition::sensors;

//////////////////////////////////////////////////
ImageSaver::ImageSaver(const std::string &_path, const std::string &_prefix)
  : path(_path), prefix(_prefix)
{
}

//////////////////////////////////////////////////
ImageSaver::~ImageSaver()
{
}

//////////////////////////////////////////////////
bool ImageSaver::SaveImage(const unsigned char *_data, unsigned int _width,
    unsigned int _height, common::Image::PixelFormatType _format)
{
  // Attempt to create the directory if it doesn't exist
  if (!ignition::common::isDirectory(this->path))
  {
    if (!ignition::common::createDirectories(this->path))
      return false;
  }

  std::string filename = this->prefix + std::to_string(this->counter) + ".png";
  ++counter;

  ignition::common::Image image;
  image.SetFromData(_data, _width, _height, _format);

  image.SavePNG(ignition::common::joinPaths(this->path, filename));
}
