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

#include "gz/sensors/SensorTypes.hh"

using namespace gz;
using namespace sensors;

// Initialize enum iterator, and string converter
IGN_ENUM(sensorNoiseIface, SensorNoiseType,
    SENSOR_NOISE_TYPE_BEGIN,
    SENSOR_NOISE_TYPE_END,
    "NO_NOISE",
    "CAMERA_NOISE",
    "SENSOR_NOISE_TYPE_END")
