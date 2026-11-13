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

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

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

//////////////////////////////////////////////////
/// \brief Sensor that logs its calls, renders ahead if asked to, and runs
/// a callback from Update().
class RenderAheadSensor : public gz::sensors::Sensor
{
  public: RenderAheadSensor(std::vector<std::string> &_log, bool _ahead)
    : log(_log), ahead(_ahead)
  {
  }

  public: ~RenderAheadSensor() override
  {
    this->log.push_back("~" + this->Name());
  }

  protected: bool RenderAhead(
                 const std::chrono::steady_clock::duration &) override
  {
    if (this->ahead)
      this->log.push_back("R" + this->Name());
    return this->ahead;
  }

  public: bool Update(const std::chrono::steady_clock::duration &) override
  {
    this->log.push_back(
        "U" + this->Name() + (this->RenderedAhead() ? "*" : ""));
    if (this->onUpdate)
      this->onUpdate();
    return true;
  }

  public: std::vector<std::string> &log;
  public: bool ahead;
  public: std::function<void()> onUpdate;
};

//////////////////////////////////////////////////
TEST_F(Manager_TEST, RenderAheadOrder)
{
  std::vector<std::string> log;
  gz::sensors::Manager mgr;
  std::map<std::string, RenderAheadSensor *> s;
  auto add = [&](const std::string &_name, bool _ahead)
  {
    auto sensor = std::make_unique<RenderAheadSensor>(log, _ahead);
    sdf::Sensor sdfSensor;
    sdfSensor.SetName(_name);
    sdfSensor.SetType(sdf::SensorType::CUSTOM);
    EXPECT_TRUE(sensor->Load(sdfSensor));
    s[_name] = sensor.get();
    mgr.AddSensor(std::move(sensor));
  };
  // Sensors run in id order, i.e. in this order.
  for (const std::string name : {"A", "B", "C", "D", "E", "F"})
    add(name, name != "C" && name != "F");
  // Not due: neither rendered ahead nor updated.
  s["E"]->SetUpdateRate(1.0);
  s["E"]->SetNextDataUpdateTime(std::chrono::seconds(1));

  // A run of sensors that render ahead renders first, then updates; a
  // sensor that doesn't completes the run before its own update.
  mgr.RunOnce(std::chrono::milliseconds(10));
  EXPECT_EQ((std::vector<std::string>{
      "RA", "RB", "UA*", "UB*", "UC", "RD", "UD*", "UF"}), log);

  // A callback removes a sensor whose frame is pending (B), and the sensor
  // the manager is at (C): neither is updated, and the rest still is.
  log.clear();
  s["A"]->onUpdate = [&]()
  {
    mgr.Remove(s["B"]->Id());
    mgr.Remove(s["C"]->Id());
  };
  mgr.RunOnce(std::chrono::milliseconds(20));
  EXPECT_EQ((std::vector<std::string>{
      "RA", "RB", "UA*", "~B", "~C", "RD", "UD*", "UF"}), log);

  // A callback of the last run adds a sensor after it: it is updated in
  // the same RunOnce, as the serial loop would.
  log.clear();
  s["A"]->onUpdate = nullptr;
  s["F"]->ahead = true;
  s["F"]->onUpdate = [&]() { add("G", false); };
  mgr.RunOnce(std::chrono::milliseconds(30));
  EXPECT_EQ((std::vector<std::string>{
      "RA", "RD", "UA*", "UD*", "RF", "UF*", "UG"}), log);

  // A callback deactivates D after D rendered ahead: D's update is skipped,
  // and its frame is not completed by a later update for the same time.
  log.clear();
  s["F"]->ahead = false;
  s["F"]->onUpdate = nullptr;
  s["A"]->onUpdate = [&]() { s["D"]->SetActive(false); };
  mgr.RunOnce(std::chrono::milliseconds(40));
  EXPECT_EQ((std::vector<std::string>{
      "RA", "RD", "UA*", "UF", "UG"}), log);
  log.clear();
  s["A"]->onUpdate = nullptr;
  s["D"]->SetActive(true);
  // Triggered: never rendered ahead, updated by the forced run.
  ASSERT_TRUE(s["D"]->SetTriggered(true, "/D/trigger"));
  mgr.RunOnce(std::chrono::milliseconds(40), true);
  EXPECT_EQ((std::vector<std::string>{
      "RA", "UA*", "UD", "RE", "UE*", "UF", "UG"}), log);
}
