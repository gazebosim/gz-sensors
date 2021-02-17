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

#include <fstream>
#include <iostream>

#include <sdf/Sensor.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/SignalHandler.hh>

#include <ignition/sensors/Manager.hh>

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
  std::ifstream fileStream(sdfFile, std::ifstream::in);

  std::string sdfString((std::istreambuf_iterator<char>(fileStream)),
                         std::istreambuf_iterator<char>());
  fileStream.close();

  // Read SDF
  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  if (!sdf::readString(sdfString, sdfParsed))
  {
    ignerr << "Failed to parse SDF: " << std::endl << sdfString << std::endl;
    return 1;
  }
  if (!sdfParsed ||
      !sdfParsed->Root() ||
      !sdfParsed->Root()->GetElement("model") ||
      !sdfParsed->Root()->GetElement("model")->GetElement("link") ||
      !sdfParsed->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"))
  {
    ignerr << "Failed to find sensor in SDF" << std::endl;
    return 1;
  }

  auto sdfElem = sdfParsed->Root()->GetElement("model")->GetElement("link")
    ->GetElement("sensor");

  // Create sensor
  ignition::sensors::Manager mgr;
  auto sensor = mgr.CreateSensor(sdfElem);

  if (!sensor)
  {
    ignerr << "Unable to load sensor" << std::endl;;
    return 1;
  }

  // Stop when user presses Ctrl+C
  bool signaled{false};
  ignition::common::SignalHandler sigHandler;
  sigHandler.AddCallback([&] (int)
  {
    signaled = true;
  });

  auto time = 0s;
  while (!signaled)
  {
    mgr.RunOnce(time, true);
    time += 1s;
    std::this_thread::sleep_for(1s);
  }

  return 0;
}
