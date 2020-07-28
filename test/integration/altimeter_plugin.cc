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

#include <sdf/sdf.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/sensors/AltimeterSensor.hh>
#include <ignition/sensors/SensorFactory.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "TransportTestTools.hh"

/// \brief Helper function to create an altimeter sdf element
sdf::ElementPtr AltimeterToSdf(const std::string &_name,
    const ignition::math::Pose3d &_pose, const double _updateRate,
    const std::string &_topic, const bool _alwaysOn,
    const bool _visualize)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='" << _name << "' type='altimeter'>"
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

/// \brief Helper function to create an altimeter sdf element with noise
sdf::ElementPtr AltimeterToSdfWithNoise(const std::string &_name,
    const ignition::math::Pose3d &_pose, const double _updateRate,
    const std::string &_topic, const bool _alwaysOn,
    const bool _visualize, double _mean, double _stddev, double _bias)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='" << _name << "' type='altimeter'>"
    << "      <pose>" << _pose << "</pose>"
    << "      <topic>" << _topic << "</topic>"
    << "      <update_rate>"<< _updateRate <<"</update_rate>"
    << "      <alwaysOn>" << _alwaysOn <<"</alwaysOn>"
    << "      <visualize>" << _visualize << "</visualize>"
    << "      <altimeter>"
    << "        <vertical_position>"
    << "          <noise type='gaussian'>"
    << "            <mean>" << _mean << "</mean>"
    << "            <stddev>" << _stddev << "</stddev>"
    << "            <bias_mean>" << _bias << "</bias_mean>"
    << "          </noise>"
    << "        </vertical_position>"
    << "        <vertical_velocity>"
    << "          <noise type='gaussian'>"
    << "            <mean>" << _mean << "</mean>"
    << "            <stddev>" << _stddev << "</stddev>"
    << "            <bias_mean>" << _bias << "</bias_mean>"
    << "          </noise>"
    << "        </vertical_velocity>"
    << "      </altimeter>"
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

class AltimeterSensorTest: public testing::Test
{
};

/////////////////////////////////////////////////
TEST_F(AltimeterSensorTest, CreateAltimeter)
{
  // Create SDF describing an altimeter sensor
  const std::string name = "TestAltimeter";
  const std::string topic = "/ignition/sensors/test/altimeter";
  const std::string topicNoise = "/ignition/sensors/test/altimeter_noise";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d sensorPose(ignition::math::Vector3d(0.25, 0.0, 0.5),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr altimeterSdf = AltimeterToSdf(name, sensorPose,
        updateRate, topic, alwaysOn, visualize);

  sdf::ElementPtr altimeterSdfNoise = AltimeterToSdfWithNoise(name, sensorPose,
        updateRate, topicNoise, alwaysOn, visualize, 1.0, 0.2, 10.0);

  // create the sensor using sensor factory
  ignition::sensors::SensorFactory sf;
  sf.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_PATH, "lib"));
  std::unique_ptr<ignition::sensors::AltimeterSensor> sensor =
      sf.CreateSensor<ignition::sensors::AltimeterSensor>(altimeterSdf);
  EXPECT_TRUE(sensor != nullptr);

  EXPECT_EQ(name, sensor->Name());
  EXPECT_EQ(topic, sensor->Topic());
  EXPECT_DOUBLE_EQ(updateRate, sensor->UpdateRate());

  std::unique_ptr<ignition::sensors::AltimeterSensor> sensorNoise =
      sf.CreateSensor<ignition::sensors::AltimeterSensor>(altimeterSdfNoise);
  EXPECT_TRUE(sensorNoise != nullptr);

  EXPECT_EQ(name, sensorNoise->Name());
  EXPECT_EQ(topicNoise, sensorNoise->Topic());
  EXPECT_DOUBLE_EQ(updateRate, sensorNoise->UpdateRate());
}

/////////////////////////////////////////////////
TEST_F(AltimeterSensorTest, SensorReadings)
{
  // Create SDF describing an altimeter sensor
  const std::string name = "TestAltimeter";
  const std::string topic = "/ignition/sensors/test/altimeter";
  const std::string topicNoise = "/ignition/sensors/test/altimeter_noise";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d sensorPose(ignition::math::Vector3d(0.25, 0.0, 0.5),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr altimeterSdf = AltimeterToSdf(name, sensorPose,
        updateRate, topic, alwaysOn, visualize);

  sdf::ElementPtr altimeterSdfNoise = AltimeterToSdfWithNoise(name, sensorPose,
        updateRate, topicNoise, alwaysOn, visualize, 1.0, 0.2, 10.0);

  // create the sensor using sensor factory
  // try creating without specifying the sensor type and then cast it
  ignition::sensors::SensorFactory sf;
  sf.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_PATH, "lib"));
  std::unique_ptr<ignition::sensors::Sensor> s =
      sf.CreateSensor(altimeterSdf);
  std::unique_ptr<ignition::sensors::AltimeterSensor> sensor(
      dynamic_cast<ignition::sensors::AltimeterSensor *>(s.release()));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  std::unique_ptr<ignition::sensors::Sensor> sNoise =
      sf.CreateSensor(altimeterSdfNoise);
  std::unique_ptr<ignition::sensors::AltimeterSensor> sensorNoise(
      dynamic_cast<ignition::sensors::AltimeterSensor *>(sNoise.release()));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensorNoise != nullptr);

  // verify initial readings
  EXPECT_DOUBLE_EQ(0.0, sensor->VerticalReference());
  EXPECT_DOUBLE_EQ(0.0, sensor->VerticalVelocity());
  EXPECT_DOUBLE_EQ(0.0, sensor->VerticalPosition());

  // verify initial readings
  EXPECT_DOUBLE_EQ(0.0, sensorNoise->VerticalReference());
  EXPECT_DOUBLE_EQ(0.0, sensorNoise->VerticalVelocity());
  EXPECT_DOUBLE_EQ(0.0, sensorNoise->VerticalPosition());

  // set state and verify readings
  double vertRef = 1.0;
  sensor->SetVerticalReference(vertRef);
  sensorNoise->SetVerticalReference(vertRef);
  EXPECT_DOUBLE_EQ(vertRef, sensor->VerticalReference());
  EXPECT_DOUBLE_EQ(vertRef, sensorNoise->VerticalReference());

  double pos = 2.0;
  sensor->SetPosition(pos);
  sensorNoise->SetPosition(pos);
  EXPECT_DOUBLE_EQ(pos - vertRef, sensor->VerticalPosition());
  EXPECT_DOUBLE_EQ(pos - vertRef, sensorNoise->VerticalPosition());

  double vertVel = 3.0;
  sensor->SetVerticalVelocity(vertVel);
  sensorNoise->SetVerticalVelocity(vertVel);
  EXPECT_DOUBLE_EQ(vertVel, sensor->VerticalVelocity());
  EXPECT_DOUBLE_EQ(vertVel, sensorNoise->VerticalVelocity());

  pos = -6.0;
  sensor->SetPosition(pos);
  sensorNoise->SetPosition(pos);
  EXPECT_DOUBLE_EQ(pos - vertRef, sensor->VerticalPosition());
  EXPECT_DOUBLE_EQ(pos - vertRef, sensorNoise->VerticalPosition());

  // verify msg received on the topic
  WaitForMessageTestHelper<ignition::msgs::Altimeter> msgHelper(topic);
  sensor->Update(ignition::common::Time(1, 0));
  EXPECT_TRUE(msgHelper.WaitForMessage()) << msgHelper;
  auto msg = msgHelper.Message();
  EXPECT_EQ(1, msg.header().stamp().sec());
  EXPECT_EQ(0, msg.header().stamp().nsec());
  EXPECT_DOUBLE_EQ(vertRef, msg.vertical_reference());
  EXPECT_DOUBLE_EQ(pos - vertRef, msg.vertical_position());
  EXPECT_DOUBLE_EQ(vertVel, msg.vertical_velocity());

  // verify msg with noise received on the topic
  WaitForMessageTestHelper<ignition::msgs::Altimeter>
    msgHelperNoise(topicNoise);
  sensorNoise->Update(ignition::common::Time(1, 0));
  EXPECT_TRUE(msgHelperNoise.WaitForMessage()) << msgHelperNoise;
  auto msgNoise = msgHelperNoise.Message();
  EXPECT_EQ(1, msg.header().stamp().sec());
  EXPECT_EQ(0, msg.header().stamp().nsec());
  EXPECT_DOUBLE_EQ(vertRef, msgNoise.vertical_reference());
  EXPECT_FALSE(ignition::math::equal(pos - vertRef,
        msgNoise.vertical_position()));
  EXPECT_FALSE(ignition::math::equal(vertVel, msgNoise.vertical_velocity()));
}

/////////////////////////////////////////////////
TEST_F(AltimeterSensorTest, Topic)
{
  const std::string name = "TestAltimeter";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;
  auto sensorPose = ignition::math::Pose3d();

  // Factory
  ignition::sensors::SensorFactory factory;
  factory.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_PATH,
      "lib"));

  // Default topic
  {
    const std::string topic;
    auto altimeterSdf = AltimeterToSdf(name, sensorPose,
          updateRate, topic, alwaysOn, visualize);

    auto sensor = factory.CreateSensor(altimeterSdf);
    EXPECT_NE(nullptr, sensor);

    auto altimeter =
        dynamic_cast<ignition::sensors::AltimeterSensor *>(sensor.release());
    ASSERT_NE(nullptr, altimeter);

    EXPECT_EQ("/altimeter", altimeter->Topic());
  }

  // Convert to valid topic
  {
    const std::string topic = "/topic with  spaces/@~characters//";
    auto altimeterSdf = AltimeterToSdf(name, sensorPose,
          updateRate, topic, alwaysOn, visualize);

    auto sensor = factory.CreateSensor(altimeterSdf);
    EXPECT_NE(nullptr, sensor);

    auto altimeter =
        dynamic_cast<ignition::sensors::AltimeterSensor *>(sensor.release());
    ASSERT_NE(nullptr, altimeter);

    EXPECT_EQ("/topic_with__spaces/characters", altimeter->Topic());
  }

  // Invalid topic
  {
    const std::string topic = "@@@";
    auto altimeterSdf = AltimeterToSdf(name, sensorPose,
          updateRate, topic, alwaysOn, visualize);

    auto sensor = factory.CreateSensor(altimeterSdf);
    ASSERT_EQ(nullptr, sensor);
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
