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
#include <gz/sensors/Manager.hh>

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
class DummySensor : public gz::sensors::Sensor
{
   public: virtual bool Update(
     const std::chrono::steady_clock::duration &) override
   {
     return true;
   }
};

//////////////////////////////////////////////////
TEST_F(Manager_TEST, Construct)
{
  gz::sensors::Manager mgr;
  EXPECT_TRUE(mgr.Init());

  sdf::ElementPtr ptr{nullptr};
  auto createdSensor = mgr.CreateSensor<DummySensor>(ptr);
  EXPECT_EQ(nullptr, createdSensor);

  gz::sensors::Sensor *sensor = mgr.Sensor(0);
  EXPECT_EQ(sensor, nullptr);

  EXPECT_FALSE(mgr.Remove(gz::sensors::NO_SENSOR));
}

//////////////////////////////////////////////////
TEST_F(Manager_TEST, AddSensor)
{
  gz::sensors::Manager mgr;

  // Fail
  std::unique_ptr<DummySensor> dummyNull{nullptr};
  EXPECT_EQ(gz::sensors::NO_SENSOR, mgr.AddSensor(std::move(dummyNull)));

  // Succeed
  sdf::Sensor sdfSensor;
  sdfSensor.SetTopic("/some/topic");
  sdfSensor.SetType(sdf::SensorType::CUSTOM);

  auto dummyGood = std::make_unique<DummySensor>();
  EXPECT_TRUE(dummyGood->Load(sdfSensor));
  EXPECT_TRUE(dummyGood->Init());
  EXPECT_NE(gz::sensors::NO_SENSOR, mgr.AddSensor(std::move(dummyGood)));
}

//////////////////////////////////////////////////
TEST_F(Manager_TEST, RemoveSensor)
{
  gz::sensors::Manager mgr;
  EXPECT_TRUE(mgr.Init());

  // Fail
  EXPECT_FALSE(mgr.Remove(gz::sensors::NO_SENSOR));

  // Succeed
  sdf::Sensor sdfSensor;
  sdfSensor.SetTopic("/some/topic");
  sdfSensor.SetType(sdf::SensorType::CUSTOM);

  auto createdSensor = mgr.CreateSensor<DummySensor>(sdfSensor);
  ASSERT_NE(nullptr, createdSensor);
  EXPECT_NE(nullptr, mgr.Sensor(createdSensor->Id()));

  EXPECT_TRUE(mgr.Remove(createdSensor->Id()));
}
