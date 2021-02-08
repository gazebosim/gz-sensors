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

#include <ignition/common/Console.hh>
#include <ignition/sensors/Export.hh>
#include <ignition/sensors/Sensor.hh>
#include <ignition/transport/Node.hh>

using namespace ignition;
using namespace sensors;

class TestSensor : public Sensor
{
  public: bool Update(const std::chrono::steady_clock::duration &) override
  {
    updateCount++;
    return true;
  }

  public: bool Update(const common::Time &) override
  {
    return false;
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

  ignition::transport::Node node;

  std::vector<std::string> services;
  node.ServiceList(services);
  ASSERT_LT(0u, services.size());

  const auto serviceIt =
    std::find(services.begin(), services.end(), "/test_topic/set_rate");
  ASSERT_NE(services.end(), serviceIt);

  std::vector<ignition::transport::ServicePublisher> publishers;
  ASSERT_TRUE(node.ServiceInfo("/test_topic/set_rate", publishers));

  ASSERT_LT(0u, publishers.size());

  ignition::msgs::Double msg;
  ignition::msgs::Empty rep;
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

  ignition::transport::Node node;

  ignition::msgs::Double msg;
  ignition::msgs::Empty rep;
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
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
