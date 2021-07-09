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
#include <ignition/sensors/Sensor.hh>

#include <atomic>
#include <memory>

#include <gtest/gtest.h>

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <ignition/msgs/performance_sensor_metrics.pb.h>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <ignition/common/Console.hh>
#include <ignition/common/Time.hh>
#include <ignition/sensors/Export.hh>
#include <ignition/transport/Node.hh>

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

/// \brief Test sensor class
class Sensor_TEST : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
  }
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

//////////////////////////////////////////////////
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

  // Add another sequence for this sensor.
  ignition::msgs::Header header2;
  sensor.AddSequence(&header2, "other");
  // The original header shouldn't change
  EXPECT_EQ(1, header.data_size());
  EXPECT_EQ(1, header.data(0).value_size());
  EXPECT_EQ("101", header.data(0).value(0));
  // The new header should be set
  EXPECT_EQ(1, header2.data_size());
  EXPECT_EQ(1, header2.data(0).value_size());
  EXPECT_EQ("0", header2.data(0).value(0));
}

//////////////////////////////////////////////////
TEST(Sensor_TEST, Topic)
{
  TestSensor sensor;
  EXPECT_EQ("", sensor.Topic());

  EXPECT_TRUE(sensor.SetTopic("/topic"));
  EXPECT_EQ("/topic", sensor.Topic());

  EXPECT_TRUE(sensor.SetTopic("/topic with spaces/@~characters//"));
  EXPECT_EQ("/topic_with_spaces/characters", sensor.Topic());

  EXPECT_FALSE(sensor.SetTopic(""));
}

class SensorUpdate : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    node.Subscribe(kPerformanceMetricTopic,
      &SensorUpdate::OnPerformanceMetric, this);
  }

  // Callback function for the performance metric topic.
  protected: void OnPerformanceMetric(
    const ignition::msgs::PerformanceSensorMetrics &_msg)
  {
    EXPECT_EQ(kSensorName, _msg.name());
    performanceMetricsMsgsCount++;
  }

  // Sensor name.
  protected: const std::string kSensorName{"sensor_test"};
  // Sensor topic.
  protected: const std::string kSensorTopic{"/sensor_test"};
  // Topic where performance metrics are published.
  protected: const std::string kPerformanceMetricTopic{
    kSensorTopic + "/performance_metrics"};
  // Number of messages to be published.
  protected: const unsigned int kNumberOfMessages{10};
  // Counter for performance metrics messages published.
  protected: std::atomic<unsigned int> performanceMetricsMsgsCount{0};
  // Transport node.
  protected: transport::Node node;
};

//////////////////////////////////////////////////
TEST_F(SensorUpdate, Update)
{
  // Create sensor.
  sdf::Sensor sdfSensor;
  sdfSensor.SetName(kSensorName);
  sdfSensor.SetTopic(kSensorTopic);
  std::unique_ptr<Sensor> sensor = std::make_unique<TestSensor>();
  sensor->Load(sdfSensor);
  ASSERT_EQ(kSensorTopic, sensor->Topic());

  // Call Update() method kNumberOfMessages times.
  // For each call there is also a call to Sensor::PublishMetrics() that
  // publishes metrics in the correspondant topic.
  for (int sec = 0 ; sec < static_cast<int>(kNumberOfMessages) ; ++sec)
  {
    common::Time now{sec, 0 /*nsec*/};
    sensor->Update(now, true);
  }

  int sleep = 0;
  const int maxSleep = 30;
  while (performanceMetricsMsgsCount < kNumberOfMessages && sleep < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sleep++;
  }
  ASSERT_EQ(kNumberOfMessages, performanceMetricsMsgsCount);

  auto testSensor = dynamic_cast<TestSensor*>(sensor.get());
  EXPECT_EQ(testSensor->updateCount, performanceMetricsMsgsCount);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
