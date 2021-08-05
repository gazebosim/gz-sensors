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

#include <vector>

#include <sdf/Sensor.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/SignalHandler.hh>
#include <ignition/sensors/Manager.hh>

// Include all supported sensors
#include <ignition/sensors/AltimeterSensor.hh>
#include "../custom_sensor/DoubleSensor.hh"

using namespace std::literals::chrono_literals;

int main(int argc,  char **argv)
{
  ignition::common::Console::SetVerbosity(4);

  if (argc < 2)
  {
    ignerr << "Missing path to SDF file" << std::endl;
    return 1;
  }

  // Load file
  std::string sdfFile(argv[1]);

  sdf::Root root;
  auto errors = root.Load(sdfFile);

  for (const auto &error : errors)
  {
    ignerr << error << std::endl;
  }

  auto world = root.WorldByIndex(0);
  if (nullptr == world)
  {
    ignerr << "Failed to load world from [" << sdfFile << "]" << std::endl;
    return 1;
  }

  // Initiate sensor manager
  ignition::sensors::Manager mgr;

  // Loop thorough SDF file and add all supported sensors to manager
  std::vector<ignition::sensors::Sensor *> sensors;
  for (auto m = 0; m < world->ModelCount(); ++m)
  {
    auto model = world->ModelByIndex(m);
    for (auto l = 0; l < model->LinkCount(); ++l)
    {
      auto link = model->LinkByIndex(l);
      for (auto s = 0; s < link->SensorCount(); ++s)
      {
        auto sensor = link->SensorByIndex(s);

        ignition::sensors::Sensor *sensorPtr;
        if (sensor->Type() == sdf::SensorType::ALTIMETER)
        {
          sensorPtr = mgr.CreateSensor<ignition::sensors::AltimeterSensor>(
              *sensor);
        }
        else if (sensor->Type() == sdf::SensorType::CUSTOM)
        {
          sensorPtr = mgr.CreateSensor<custom::DoubleSensor>(*sensor);
        }
        else
        {
          ignerr << "Sensor type [" << static_cast<int>(sensor->Type())
                 << "] not supported." << std::endl;
        }

        if (nullptr == sensorPtr)
        {
          ignerr << "Failed to create sensor [" << sensor->Name() << "]"
                 << std::endl;
          continue;
        }

        sensors.push_back(sensorPtr);

        ignmsg << "Added sensor [" << sensor->Name() << "] to manager."
               << std::endl;
      }
    }
  }

  if (sensors.empty())
  {
    ignerr << "No sensors have been added to the manager." << std::endl;
    return 1;
  }

  // Stop when user presses Ctrl+C
  bool signaled{false};
  ignition::common::SignalHandler sigHandler;
  sigHandler.AddCallback([&] (int)
  {
    signaled = true;
  });

  ignmsg << "Looping sensor manager. Press Ctrl + C to stop." << std::endl;

  auto time = 0s;
  while (!signaled)
  {
    // Update each sensor using their specific APIs
    for (const auto &sensorPtr : sensors)
    {
      if (auto altimeter = dynamic_cast<ignition::sensors::AltimeterSensor *>(
          sensorPtr))
      {
        altimeter->SetVerticalVelocity(altimeter->VerticalVelocity() + 0.1);
      }
      else if (auto custom = dynamic_cast<custom::DoubleSensor *>(sensorPtr))
      {
        custom->data += 0.1;
      }
    }

    // Call all sensor's own update functions, which will apply noise, publish
    // data, etc.
    mgr.RunOnce(time, true);
    time += 1s;
    std::this_thread::sleep_for(1s);
  }

  return 0;
}
