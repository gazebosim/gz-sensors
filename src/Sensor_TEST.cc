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
#if defined(_MSC_VER)
  #pragma warning(push)
  #pragma warning(disable: 4005)
  #pragma warning(disable: 4251)
#endif
#include <gz/msgs/performance_sensor_metrics.pb.h>
#if defined(_MSC_VER)
  #pragma warning(pop)
#endif

#include <atomic>
#include <memory>

#include <gtest/gtest.h>

#include <gz/common/Console.hh>
#include <gz/sensors/Export.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/transport/Node.hh>

using namespace gz;
using namespace sensors;

class TestSensor : public Sensor
{
  public: bool Update(const std::chrono::steady_clock::duration &) override
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
    common::Console::SetVerbosity(4);
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

  EXPECT_EQ(sensor.Name(), sensor.FrameId());
  sensor.SetFrameId("frame_id_12");
  EXPECT_EQ(std::string("frame_id_12"), sensor.FrameId());

  EXPECT_TRUE(sensor.IsActive());
  sensor.SetActive(false);
  EXPECT_FALSE(sensor.IsActive());
}

//////////////////////////////////////////////////
TEST(Sensor_TEST, AddSequence)
{
  TestSensor sensor;
  msgs::Header header;
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
  msgs::Header header2;
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
    common::Console::SetVerbosity(4);
    node.Subscribe(kPerformanceMetricTopic,
      &SensorUpdate::OnPerformanceMetric, this);
  }

  // Callback function for the performance metric topic.
  protected: void OnPerformanceMetric(
    const msgs::PerformanceSensorMetrics &_msg)
  {
    EXPECT_EQ(kSensorName, _msg.name());
    performanceMetricsMsgsCount++;
  }

  // Sensor name.
  protected: const std::string kSensorName{"sensor_test"};
  // Sensor topic.
  protected: const std::string kSensorTopic{"/sensor_test"};
  // Enable metrics flag.
  protected: const bool kEnableMetrics{true};
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
  sdfSensor.SetEnableMetrics(kEnableMetrics);
  std::unique_ptr<Sensor> sensor = std::make_unique<TestSensor>();
  sensor->Load(sdfSensor);
  ASSERT_EQ(kSensorTopic, sensor->Topic());
  ASSERT_EQ(kEnableMetrics, sensor->EnableMetrics());

  // Call Update() method kNumberOfMessages times.
  // For each call there is also a call to Sensor::PublishMetrics() that
  // publishes metrics in the correspondant topic.
  for (int sec = 0 ; sec < static_cast<int>(kNumberOfMessages) ; ++sec)
  {
    std::chrono::steady_clock::duration now = std::chrono::seconds(sec);
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
TEST(Sensor_TEST, SetRateService)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='test' type='altimeter'>"
    << "      <topic>test_topic</topic>"
    << "      <update_rate>10.0</update_rate>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  TestSensor sensor;
  ASSERT_TRUE(sensor.Load(
    sdfParsed->Root()->GetElement("model")->GetElement("link")
      ->GetElement("sensor")));

  EXPECT_EQ("test_topic", sensor.Topic());
  EXPECT_FLOAT_EQ(10.0, sensor.UpdateRate());

  transport::Node node;

  std::vector<std::string> services;
  node.ServiceList(services);
  ASSERT_LT(0u, services.size());

  const auto serviceIt =
    std::find(services.begin(), services.end(), "/test_topic/set_rate");
  ASSERT_NE(services.end(), serviceIt);

  std::vector<transport::ServicePublisher> publishers;
  ASSERT_TRUE(node.ServiceInfo("/test_topic/set_rate", publishers));

  ASSERT_LT(0u, publishers.size());

  msgs::Double msg;
  msgs::Empty rep;
  bool res;

  // can set value lower than in SDF
  msg.set_data(5.0);
  res = false;
  EXPECT_TRUE(node.Request("/test_topic/set_rate", msg, 1000, rep, res));
  EXPECT_TRUE(res);

  EXPECT_FLOAT_EQ(5.0, sensor.UpdateRate());

  // cannot set 0 if SDF has non-zero
  msg.set_data(0.0);
  res = false;
  EXPECT_TRUE(node.Request("/test_topic/set_rate", msg, 1000, rep, res));
  EXPECT_TRUE(res);

  EXPECT_FLOAT_EQ(5.0, sensor.UpdateRate());

  // cannot set higher than SDF value
  msg.set_data(20.0);
  res = false;
  EXPECT_TRUE(node.Request("/test_topic/set_rate", msg, 1000, rep, res));
  EXPECT_TRUE(res);

  EXPECT_FLOAT_EQ(5.0, sensor.UpdateRate());
}

//////////////////////////////////////////////////
TEST(Sensor_TEST, SetRateZeroService)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='test' type='altimeter'>"
    << "      <topic>test_topic2</topic>"
    << "      <update_rate>0.0</update_rate>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  TestSensor sensor;
  ASSERT_TRUE(sensor.Load(
    sdfParsed->Root()->GetElement("model")->GetElement("link")
      ->GetElement("sensor")));

  EXPECT_EQ("test_topic2", sensor.Topic());
  EXPECT_FLOAT_EQ(0.0, sensor.UpdateRate());

  transport::Node node;

  msgs::Double msg;
  msgs::Empty rep;
  bool res;

  // can set any value if SDF has 0
  msg.set_data(5.0);
  res = false;
  EXPECT_TRUE(node.Request("/test_topic2/set_rate", msg, 1000, rep, res));
  EXPECT_TRUE(res);

  EXPECT_FLOAT_EQ(5.0, sensor.UpdateRate());

  // can set 0 if SDF has zero
  msg.set_data(0.0);
  res = false;
  EXPECT_TRUE(node.Request("/test_topic2/set_rate", msg, 1000, rep, res));
  EXPECT_TRUE(res);

  EXPECT_FLOAT_EQ(0.0, sensor.UpdateRate());

  // can set any value if SDF has 0
  msg.set_data(20.0);
  res = false;
  EXPECT_TRUE(node.Request("/test_topic2/set_rate", msg, 1000, rep, res));
  EXPECT_TRUE(res);

  EXPECT_FLOAT_EQ(20.0, sensor.UpdateRate());
}

//////////////////////////////////////////////////
TEST(Sensor_TEST, FrameIdFromSdf)
{
  auto loadSensorWithSdfParam =
      [](TestSensor &_testSensor, const std::string &_sensorParam)
  {
    const std::string sensorSdf = R"(
    <sdf version="1.9">
      <model name="m1">
        <link name="link1">
          <sensor name="test" type="imu">)" +
                                  _sensorParam + R"(
          </sensor>
        </link>
      </model>
    </sdf>
    )";
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(sensorSdf);
    ASSERT_TRUE(errors.empty()) << errors;
    auto *model = root.Model();
    ASSERT_NE(model, nullptr);
    auto *link = model->LinkByIndex(0);
    ASSERT_NE(link, nullptr);
    auto *sensor = link->SensorByIndex(0);
    ASSERT_NE(sensor, nullptr);

    _testSensor.Load(*sensor);
  };

  {
    TestSensor testSensor;
    loadSensorWithSdfParam(testSensor, "");
    EXPECT_EQ("test", testSensor.FrameId());
  }
  {
    TestSensor testSensor;
    loadSensorWithSdfParam(
        testSensor, "<ignition_frame_id>custom_frame_id</ignition_frame_id>");
    EXPECT_EQ("custom_frame_id", testSensor.FrameId());
  }
  {
    TestSensor testSensor;
    loadSensorWithSdfParam(testSensor,
                           "<gz_frame_id>custom_frame_id</gz_frame_id>");
    EXPECT_EQ("custom_frame_id", testSensor.FrameId());
  }
  {
    TestSensor testSensor;
    loadSensorWithSdfParam(testSensor, R"(
      <ignition_frame_id>custom_frame_id</ignition_frame_id>
      <gz_frame_id>other_custom_frame_id</gz_frame_id>)");
    EXPECT_EQ("other_custom_frame_id", testSensor.FrameId());
  }
}
//////////////////////////////////////////////////
TEST_F(SensorUpdate, NextDataUpdateTime)
{
  // Create sensor.
  sdf::Sensor sdfSensor;
  sdfSensor.SetName(kSensorName);
  sdfSensor.SetTopic(kSensorTopic);
  sdfSensor.SetUpdateRate(1);

  std::unique_ptr<Sensor> sensor = std::make_unique<TestSensor>();
  sensor->Load(sdfSensor);

  {
    std::chrono::steady_clock::duration now = std::chrono::seconds(0);
    std::chrono::steady_clock::duration next = std::chrono::seconds(1);
    EXPECT_TRUE(sensor->Update(now, false));
    EXPECT_EQ(next.count(), sensor->NextDataUpdateTime().count());
  }

  {
    std::chrono::steady_clock::duration now = std::chrono::seconds(1);
    std::chrono::steady_clock::duration next = std::chrono::seconds(2);
    EXPECT_TRUE(sensor->Update(now, false));
    EXPECT_EQ(next.count(), sensor->NextDataUpdateTime().count());
  }

  {
    // set the next data update time into the future, so we no longer
    // expect an update to happen
    std::chrono::steady_clock::duration now = std::chrono::seconds(2);
    std::chrono::steady_clock::duration next = std::chrono::seconds(5);
    sensor->SetNextDataUpdateTime(next);
    EXPECT_FALSE(sensor->Update(now, false));
    EXPECT_EQ(next.count(), sensor->NextDataUpdateTime().count());
  }

  {
    // Force has no impact  on the next update time
    std::chrono::steady_clock::duration now = std::chrono::seconds(3);
    std::chrono::steady_clock::duration next = std::chrono::seconds(5);
    EXPECT_TRUE(sensor->Update(now, true));
    EXPECT_EQ(next.count(), sensor->NextDataUpdateTime().count());
  }

  {
    // When that time point is reached, the update happens
    std::chrono::steady_clock::duration now = std::chrono::seconds(5);
    std::chrono::steady_clock::duration next = std::chrono::seconds(6);
    EXPECT_TRUE(sensor->Update(now, false));
    EXPECT_EQ(next.count(), sensor->NextDataUpdateTime().count());
  }

  {
    // Jump backwards in time
    std::chrono::steady_clock::duration now = std::chrono::seconds(5);
    std::chrono::steady_clock::duration next = std::chrono::seconds(1);
    sensor->SetNextDataUpdateTime(next);
    EXPECT_TRUE(sensor->Update(now, false));

    // The next update should be the first dt past the current time
    std::chrono::steady_clock::duration newNext = std::chrono::seconds(6);
    EXPECT_EQ(newNext.count(), sensor->NextDataUpdateTime().count());
  }
}
