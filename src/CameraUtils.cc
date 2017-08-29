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

#include <ignition/sensors/CameraUtils.hh>

#include <sstream>

using namespace ignition::sensors;

//////////////////////////////////////////////////
sdf::ElementPtr CameraConfig(const std::string &_name,
    const std::string &_topic, double _hz, std::size_t _width,
    std::size_t _height, double _hfov, double _near, double _far)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << "  <model name='m1'>"
    << "    <link name='link1'>"
    << "      <sensor name='" << _name << "' type='camera'>"
    << "        <update_rate>" << _hz << "</update_rate>"
    << "        <topic>" << _topic << "</topic>"
    << "        <camera>"
    << "          <horizontal_fov>" << _hfov << "</horizontal_fov>"
    << "          <image>"
    << "            <width>" << _width << "</width>"
    << "            <height>" << _height << "</height>"
    << "          </image>"
    << "          <clip>"
    << "            <near>" << _near << "</near>"
    << "            <far>" << _far << "</far>"
    << "          </clip>"
    << "        </camera>"
    << "      </sensor>"
    << "    </link>"
    << "  </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  if (!sdf::readString(stream.str(), sdfParsed))
    return sdf::ElementPtr();

  return sdfParsed->Root()->GetElement("model")->GetElement("link")
    ->GetElement("sensor");
}
