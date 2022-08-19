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

#include <gtest/gtest.h>
#include <ignition/sensors/Manager.hh>

/// \brief Test sensor manager
class Manager_TEST : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);
  }
};

//////////////////////////////////////////////////
TEST(Manager, construct)
{
  gz::sensors::Manager mgr;
  EXPECT_TRUE(mgr.Init());

  sdf::ElementPtr ptr;
  gz::sensors::SensorId id = mgr.CreateSensor(ptr);
  EXPECT_EQ(id, gz::sensors::NO_SENSOR);

  gz::sensors::Sensor *sensor = mgr.Sensor(0);
  EXPECT_EQ(sensor, nullptr);

  EXPECT_FALSE(mgr.Remove(gz::sensors::NO_SENSOR));
}

//////////////////////////////////////////////////
TEST(Manager, removeSensor)
{
  gz::sensors::Manager mgr;
  EXPECT_TRUE(mgr.Init());

  EXPECT_FALSE(mgr.Remove(gz::sensors::NO_SENSOR));

  // \todo(nkoenig) Add a sensor, then remove it
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
