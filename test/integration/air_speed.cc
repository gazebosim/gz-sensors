/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <sdf/sdf.hh>

#include <gz/math/Helpers.hh>

#include <gz/msgs/air_speed.pb.h>

#include <gz/sensors/AirSpeedSensor.hh>
#include <gz/sensors/SensorFactory.hh>

#include "test_config.hh"  // NOLINT(build/include)
#include "TransportTestTools.hh"

/// \brief Helper function to create an air speed sdf element
sdf::ElementPtr AirSpeedToSdf(const std::string &_name,
    const gz::math::Pose3d &_pose, const double _updateRate,
    const std::string &_topic, const bool _alwaysOn,
    const bool _visualize)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='" << _name << "' type='air_speed'>"
    << "      <pose>" << _pose << "</pose>"
    << "      <topic>" << _topic << "</topic>"
    << "      <update_rate>"<< _updateRate <<"</update_rate>"
    << "      <alwaysOn>" << _alwaysOn <<"</alwaysOn>"
    << "      <visualize>" << _visualize << "</visualize>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  if (!sdf::readString(stream.str(), sdfParsed))
    return sdf::ElementPtr();

  return sdfParsed->Root()->GetElement("model")->GetElement("link")
    ->GetElement("sensor");
}

/// \brief Helper function to create an air speed sdf element with noise
sdf::ElementPtr AirSpeedToSdfWithNoise(const std::string &_name,
    const gz::math::Pose3d &_pose, const double _updateRate,
    const std::string &_topic, const bool _alwaysOn,
    const bool _visualize, double _mean, double _stddev, double _bias)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='" << _name << "' type='air_speed'>"
    << "      <pose>" << _pose << "</pose>"
    << "      <topic>" << _topic << "</topic>"
    << "      <update_rate>"<< _updateRate <<"</update_rate>"
    << "      <alwaysOn>" << _alwaysOn <<"</alwaysOn>"
    << "      <visualize>" << _visualize << "</visualize>"
    << "      <air_speed>"
    << "        <pressure>"
    << "          <noise type='gaussian'>"
    << "            <mean>" << _mean << "</mean>"
    << "            <stddev>" << _stddev << "</stddev>"
    << "            <bias_mean>" << _bias << "</bias_mean>"
    << "          </noise>"
    << "        </pressure>"
    << "      </air_speed>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  if (!sdf::readString(stream.str(), sdfParsed))
    return sdf::ElementPtr();

  return sdfParsed->Root()->GetElement("model")->GetElement("link")
    ->GetElement("sensor");
}

/// \brief Test air speed sensor
class AirSpeedSensorTest: public testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);
  }
};

/////////////////////////////////////////////////
TEST_F(AirSpeedSensorTest, CreateAirSpeed)
{
  // Create SDF describing an air_pressure sensor
  const std::string name = "TestAirSpeed";
  const std::string topic = "/gz/sensors/test/air_speed";
  const std::string topicNoise = "/gz/sensors/test/air_speed_noise";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  gz::math::Pose3d sensorPose(gz::math::Vector3d(0.25, 0.0, 0.5),
      gz::math::Quaterniond::Identity);
  sdf::ElementPtr airSpeedSdf = AirSpeedToSdf(name, sensorPose,
        updateRate, topic, alwaysOn, visualize);

  sdf::ElementPtr airSpeedSdfNoise = AirSpeedToSdfWithNoise(name,
      sensorPose, updateRate, topicNoise, alwaysOn, visualize, 1.0, 0.2, 10.0);

  // create the sensor using sensor factory
  gz::sensors::SensorFactory sf;
  std::unique_ptr<gz::sensors::AirSpeedSensor> sensor =
      sf.CreateSensor<gz::sensors::AirSpeedSensor>(airSpeedSdf);
  ASSERT_NE(nullptr, sensor);

  EXPECT_EQ(name, sensor->Name());
  EXPECT_EQ(topic, sensor->Topic());
  EXPECT_DOUBLE_EQ(updateRate, sensor->UpdateRate());

  std::unique_ptr<gz::sensors::AirSpeedSensor> sensorNoise =
      sf.CreateSensor<gz::sensors::AirSpeedSensor>(
          airSpeedSdfNoise);
  ASSERT_NE(nullptr, sensorNoise);

  EXPECT_EQ(name, sensorNoise->Name());
  EXPECT_EQ(topicNoise, sensorNoise->Topic());
  EXPECT_DOUBLE_EQ(updateRate, sensorNoise->UpdateRate());
}

/////////////////////////////////////////////////
TEST_F(AirSpeedSensorTest, SensorReadings)
{
  // Create SDF describing an air_pressure sensor
  const std::string name = "TestAirSpeed";
  const std::string topic = "/gz/sensors/test/air_pressure";
  const std::string topicNoise = "/gz/sensors/test/air_pressure_noise";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  gz::math::Pose3d sensorPose(gz::math::Vector3d(0.25, 0.0, 0.5),
      gz::math::Quaterniond::Identity);
  sdf::ElementPtr airSpeedSdf = AirSpeedToSdf(name, sensorPose,
        updateRate, topic, alwaysOn, visualize);

  sdf::ElementPtr airSpeedSdfNoise = AirSpeedToSdfWithNoise(name,
      sensorPose, updateRate, topicNoise, alwaysOn, visualize, 1.0, 0.2, 10.0);

  // create the sensor using sensor factory
  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::AirSpeedSensor>(
      airSpeedSdf);
  ASSERT_NE(nullptr, sensor);
  EXPECT_FALSE(sensor->HasConnections());

  auto sensorNoise = sf.CreateSensor<gz::sensors::AirSpeedSensor>(
      airSpeedSdfNoise);
  ASSERT_NE(nullptr, sensorNoise);

  sensor->SetPose(
      gz::math::Pose3d(0, 0, 1.5, 0, 0, 0) * sensorNoise->Pose());

  // verify msg received on the topic
  WaitForMessageTestHelper<gz::msgs::AirSpeed> msgHelper(topic);
  EXPECT_TRUE(sensor->HasConnections());
  sensor->Update(std::chrono::steady_clock::duration(std::chrono::seconds(1)));
  EXPECT_TRUE(msgHelper.WaitForMessage()) << msgHelper;
  auto msg = msgHelper.Message();
  EXPECT_EQ(1, msg.header().stamp().sec());
  EXPECT_EQ(0, msg.header().stamp().nsec());
  EXPECT_DOUBLE_EQ(0.0, msg.diff_pressure());
  EXPECT_DOUBLE_EQ(288.1369934082031, msg.temperature());

  // verify msg with noise received on the topic
  WaitForMessageTestHelper<gz::msgs::AirSpeed>
    msgHelperNoise(topicNoise);
  sensorNoise->Update(std::chrono::steady_clock::duration(
      std::chrono::seconds(1)), false);
  EXPECT_TRUE(msgHelperNoise.WaitForMessage()) << msgHelperNoise;
  auto msgNoise = msgHelperNoise.Message();
  EXPECT_EQ(1, msg.header().stamp().sec());
  EXPECT_EQ(0, msg.header().stamp().nsec());
  EXPECT_FALSE(gz::math::equal(0.0, msgNoise.diff_pressure()));
  EXPECT_DOUBLE_EQ(288.14675903320312, msgNoise.temperature());
}

/////////////////////////////////////////////////
TEST_F(AirSpeedSensorTest, Topic)
{
  const std::string name = "TestAirSpeed";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;
  auto sensorPose = gz::math::Pose3d();

  // Factory
  gz::sensors::SensorFactory factory;

  // Default topic
  {
    const std::string topic;
    auto airSpeedSdf = AirSpeedToSdf(name, sensorPose,
          updateRate, topic, alwaysOn, visualize);

    auto airSpeed = factory.CreateSensor<
        gz::sensors::AirSpeedSensor>(airSpeedSdf);
    ASSERT_NE(nullptr, airSpeed);

    EXPECT_EQ("/air_speed", airSpeed->Topic());
  }

  // Convert to valid topic
  {
    const std::string topic = "/topic with spaces/@~characters//";
    auto airSpeedSdf = AirSpeedToSdf(name, sensorPose,
          updateRate, topic, alwaysOn, visualize);

    auto airSpeed = factory.CreateSensor<
        gz::sensors::AirSpeedSensor>(airSpeedSdf);
    ASSERT_NE(nullptr, airSpeed);

    EXPECT_EQ("/topic_with_spaces/characters", airSpeed->Topic());
  }

  // Invalid topic
  {
    const std::string topic = "@@@";
    auto airSpeedSdf = AirSpeedToSdf(name, sensorPose,
          updateRate, topic, alwaysOn, visualize);

    auto sensor = factory.CreateSensor<
        gz::sensors::AirSpeedSensor>(airSpeedSdf);
    ASSERT_EQ(nullptr, sensor);
  }
}
