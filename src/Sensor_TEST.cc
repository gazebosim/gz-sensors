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
#include <gtest/gtest.h>

#include <ignition/sensors/Export.hh>
#include <ignition/sensors/Sensor.hh>

using namespace ignition;
using namespace sensors;

class TestSensor : public Sensor
{
  public: bool Update(const common::Time &) override
  {
    updateCount++;
    return true;
  }

  public: unsigned int updateCount{0};
};

//////////////////////////////////////////////////
TEST(Sensor_TEST, Sensor)
{
  TestSensor sensor;

  EXPECT_EQ(math::Pose3d::Zero, sensor.Pose());
  sensor.SetPose(math::Pose3d(1, 2, 3, 0, 0, 0));
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0, 0, 0), sensor.Pose());

  EXPECT_DOUBLE_EQ(0.0, sensor.UpdateRate());
  sensor.SetUpdateRate(0.5);
  EXPECT_DOUBLE_EQ(0.5, sensor.UpdateRate());
  sensor.SetUpdateRate(0);
  EXPECT_DOUBLE_EQ(0, sensor.UpdateRate());
  sensor.SetUpdateRate(-123);
  EXPECT_DOUBLE_EQ(0, sensor.UpdateRate());

  EXPECT_EQ("", sensor.Parent());
  sensor.SetParent("banana");
  EXPECT_EQ("banana", sensor.Parent());

  EXPECT_EQ("", sensor.Name());

  EXPECT_EQ("", sensor.Topic());

  EXPECT_EQ(1u, sensor.Id());

  EXPECT_EQ(nullptr, sensor.SDF());
}

TEST(Sensor_TEST, AddSequence)
{
  TestSensor sensor;
  ignition::msgs::Header header;
  sensor.AddSequence(&header);
  EXPECT_EQ("seq", header.data(0).key());
  EXPECT_EQ("0", header.data(0).value(0));

  sensor.AddSequence(&header);
  EXPECT_EQ("seq", header.data(0).key());
  EXPECT_EQ("1", header.data(0).value(0));

  EXPECT_EQ(1, header.data_size());
  EXPECT_EQ(1, header.data(0).value_size());

  for (int i = 0; i < 100; i++)
    sensor.AddSequence(&header);
  EXPECT_EQ(1, header.data_size());
  EXPECT_EQ(1, header.data(0).value_size());
  EXPECT_EQ("101", header.data(0).value(0));
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
