/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gz/common/Console.hh>

#include "gz/sensors/Util.hh"

//////////////////////////////////////////////////
std::string gz::sensors::customType(const sdf::Sensor &_sdf)
{
  if (nullptr == _sdf.Element())
    return std::string();

  return customType(_sdf.Element());
}

//////////////////////////////////////////////////
std::string gz::sensors::customType(sdf::ElementPtr _sdf)
{
  if (_sdf == nullptr)
    return std::string();

  if (!_sdf->HasAttribute("type"))
  {
    gzerr << "Sensor missing `type` attribute." << std::endl;
    return std::string();
  }
  auto type = _sdf->Get<std::string>("type");
  if ("custom" != type)
  {
    gzerr << "Sensor `type` is not [custom]; it's [" << type << "]."
           << std::endl;
    return std::string();
  }

  if (!_sdf->HasAttribute("gz:type"))
  {
    // TODO(CH3): Deprecated. Remove on tock.
    // Try deprecated ignition:type attribute if gz:type attribute is missing
    if (_sdf->HasAttribute("ignition:type"))
    {
      gzwarn << "The `ignition:type` attribute is deprecated. Please use "
             << "`gz:type` instead." << std::endl;
      return _sdf->Get<std::string>("ignition:type");
    }
    else
    {
      gzerr << "Custom sensor missing `gz:type` attribute." << std::endl;
      return std::string();
    }
  }

  return _sdf->Get<std::string>("gz:type");
}
