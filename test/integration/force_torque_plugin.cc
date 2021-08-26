/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/sensors/ForceTorqueSensor.hh>
#include <ignition/sensors/SensorFactory.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "TransportTestTools.hh"

/// \brief Helper function to create a force torque sdf element.
sdf::ElementPtr ForceTorqueToSdf(const std::string &_name,
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
    << "    <sensor name='" << _name << "' type='force_torque'>"
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

// /// \brief Helper function to create an altimeter sdf element with noise
// sdf::ElementPtr AltimeterToSdfWithNoise(const std::string &_name,
//     const ignition::math::Pose3d &_pose, const double _updateRate,
//     const std::string &_topic, const bool _alwaysOn,
//     const bool _visualize, double _mean, double _stddev, double _bias)
// {
//   std::ostringstream stream;
//   stream
//     << "<?xml version='1.0'?>"
//     << "<sdf version='1.6'>"
//     << " <model name='m1'>"
//     << "  <link name='link1'>"
//     << "    <sensor name='" << _name << "' type='altimeter'>"
//     << "      <pose>" << _pose << "</pose>"
//     << "      <topic>" << _topic << "</topic>"
//     << "      <update_rate>"<< _updateRate <<"</update_rate>"
//     << "      <alwaysOn>" << _alwaysOn <<"</alwaysOn>"
//     << "      <visualize>" << _visualize << "</visualize>"
//     << "      <altimeter>"
//     << "        <vertical_position>"
//     << "          <noise type='gaussian'>"
//     << "            <mean>" << _mean << "</mean>"
//     << "            <stddev>" << _stddev << "</stddev>"
//     << "            <bias_mean>" << _bias << "</bias_mean>"
//     << "          </noise>"
//     << "        </vertical_position>"
//     << "        <vertical_velocity>"
//     << "          <noise type='gaussian'>"
//     << "            <mean>" << _mean << "</mean>"
//     << "            <stddev>" << _stddev << "</stddev>"
//     << "            <bias_mean>" << _bias << "</bias_mean>"
//     << "          </noise>"
//     << "        </vertical_velocity>"
//     << "      </altimeter>"
//     << "    </sensor>"
//     << "  </link>"
//     << " </model>"
//     << "</sdf>";

//   sdf::SDFPtr sdfParsed(new sdf::SDF());
//   sdf::init(sdfParsed);
//   if (!sdf::readString(stream.str(), sdfParsed))
//     return sdf::ElementPtr();

//   return sdfParsed->Root()->GetElement("model")->GetElement("link")
//     ->GetElement("sensor");
// }

/// \brief Test force torque sensor
class ForceTorqueSensorTest: public testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
  }
};

/////////////////////////////////////////////////
TEST_F(ForceTorqueSensorTest, CreateForceTorqueSensor)
{
  // Create SDF describing a force torque sensor
  const std::string name = "TestForceTorqueSensor";
  const std::string topic = "/ignition/sensors/test/force_torque";
  
  // const std::string topicNoise = "/ignition/sensors/test/altimeter_noise";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d sensorPose(ignition::math::Vector3d(0.25, 0.0, 0.5),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr forcetorqueSdf = ForceTorqueToSdf(name, sensorPose,
        updateRate, topic, alwaysOn, visualize);

  // sdf::ElementPtr altimeterSdfNoise = AltimeterToSdfWithNoise(name, sensorPose,
  //       updateRate, topicNoise, alwaysOn, visualize, 1.0, 0.2, 10.0);

  // create the sensor using sensor factory
  ignition::sensors::SensorFactory sf;
  std::unique_ptr<ignition::sensors::ForceTorqueSensor> sensor =
      sf.CreateSensor<ignition::sensors::ForceTorqueSensor>(forcetorqueSdf);
  ASSERT_NE(nullptr, sensor);

  EXPECT_EQ(name, sensor->Name());
  EXPECT_EQ(topic, sensor->Topic());
  EXPECT_DOUBLE_EQ(updateRate, sensor->UpdateRate());

  // std::unique_ptr<ignition::sensors::AltimeterSensor> sensorNoise =
  //     sf.CreateSensor<ignition::sensors::AltimeterSensor>(altimeterSdfNoise);
  // ASSERT_NE(nullptr, sensorNoise);

  // EXPECT_EQ(name, sensorNoise->Name());
  // EXPECT_EQ(topicNoise, sensorNoise->Topic());
  // EXPECT_DOUBLE_EQ(updateRate, sensorNoise->UpdateRate());
}

// /////////////////////////////////////////////////
TEST_F(ForceTorqueSensorTest, SensorReadings)
{
  // Create SDF describing a a force torque sensor
  const std::string name = "TestForceTorqueSensor";
  const std::string topic = "/ignition/sensors/test/force_torque";
  // const std::string topicNoise = "/ignition/sensors/test/altimeter_noise";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d sensorPose(ignition::math::Vector3d(0.25, 0.0, 0.5),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr forcetorqueSdf = ForceTorqueToSdf(name, sensorPose,
        updateRate, topic, alwaysOn, visualize);

  // sdf::ElementPtr altimeterSdfNoise = AltimeterToSdfWithNoise(name, sensorPose,
  //       updateRate, topicNoise, alwaysOn, visualize, 1.0, 0.2, 10.0);

  // create the sensor using sensor factory
  // try creating without specifying the sensor type and then cast it
  ignition::sensors::SensorFactory sf;
  std::unique_ptr<ignition::sensors::Sensor> s =
      sf.CreateSensor(forcetorqueSdf);
  std::unique_ptr<ignition::sensors::ForceTorqueSensor> sensor(
      dynamic_cast<ignition::sensors::ForceTorqueSensor *>(s.release()));

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);

  // std::unique_ptr<ignition::sensors::Sensor> sNoise =
  //     sf.CreateSensor(altimeterSdfNoise);
  // std::unique_ptr<ignition::sensors::AltimeterSensor> sensorNoise(
  //     dynamic_cast<ignition::sensors::AltimeterSensor *>(sNoise.release()));

  // Make sure the above dynamic cast worked.
  // ASSERT_NE(nullptr, sensorNoise);

  // verify initial readings
  EXPECT_EQ(ignition::math::Vector3d::Zero, sensor->Force());
  EXPECT_EQ(ignition::math::Vector3d::Zero, sensor->Torque());

  // verify initial readings
  // EXPECT_DOUBLE_EQ(0.0, sensorNoise->force());
  // EXPECT_DOUBLE_EQ(0.0, sensorNoise->torque());

  // set state and verify readings

  // TODO : Add Noise tests
  ignition::math::Vector3d force(0,0,1);
  sensor->SetForce(force);
  EXPECT_EQ(force, sensor->Force());

  ignition::math::Vector3d torque(0,0,1);
  sensor->SetTorque(torque);
  EXPECT_EQ(torque, sensor->Torque());

  // verify msg received on the topic
  WaitForMessageTestHelper<ignition::msgs::Wrench> msgHelper(topic);
  EXPECT_TRUE(sensor->Update(ignition::common::Time(1, 0)));
  EXPECT_TRUE(msgHelper.WaitForMessage()) << msgHelper;
  auto msg = msgHelper.Message();
  EXPECT_EQ(1, msg.header().stamp().sec());
  EXPECT_EQ(0, msg.header().stamp().nsec());

  EXPECT_EQ(ignition::math::Vector3d(0, 0, 1), ignition::msgs::Convert(msg.force()));
  EXPECT_EQ(ignition::math::Vector3d(0, 0, 1), ignition::msgs::Convert(msg.torque()));

  // // verify msg with noise received on the topic
  // WaitForMessageTestHelper<ignition::msgs::Altimeter>
  //   msgHelperNoise(topicNoise);
  // sensorNoise->Update(std::chrono::steady_clock::duration(
  //   std::chrono::seconds(1)));
  // EXPECT_TRUE(msgHelperNoise.WaitForMessage()) << msgHelperNoise;
  // auto msgNoise = msgHelperNoise.Message();
  // EXPECT_EQ(1, msg.header().stamp().sec());
  // EXPECT_EQ(0, msg.header().stamp().nsec());
  // EXPECT_DOUBLE_EQ(vertRef, msgNoise.vertical_reference());
  // EXPECT_FALSE(ignition::math::equal(pos - vertRef,
  //       msgNoise.vertical_position()));
  // EXPECT_FALSE(ignition::math::equal(vertVel, msgNoise.vertical_velocity()));
}

/////////////////////////////////////////////////
TEST_F(ForceTorqueSensorTest, Topic)
{
  const std::string name = "TestForceTorqueSensor";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;
  auto sensorPose = ignition::math::Pose3d();

  // Factory
  ignition::sensors::SensorFactory factory;

  // Default topic
  {
    const std::string topic;
    auto forcetorqueSdf = ForceTorqueToSdf(name, sensorPose,
          updateRate, topic, alwaysOn, visualize);

    auto sensor = factory.CreateSensor(forcetorqueSdf);
    EXPECT_NE(nullptr, sensor);

    auto force_torque =
        dynamic_cast<ignition::sensors::ForceTorqueSensor *>(sensor.release());
    ASSERT_NE(nullptr, force_torque);

    EXPECT_EQ("/force_torque", force_torque->Topic());
  }

  // Convert to valid topic
  {
    const std::string topic = "/topic with spaces/@~characters//";
    auto forcetorqueSdf = ForceTorqueToSdf(name, sensorPose,
          updateRate, topic, alwaysOn, visualize);

    auto sensor = factory.CreateSensor(forcetorqueSdf);
    EXPECT_NE(nullptr, sensor);

    auto force_torque =
        dynamic_cast<ignition::sensors::ForceTorqueSensor *>(sensor.release());
    ASSERT_NE(nullptr, force_torque);

    EXPECT_EQ("/topic_with_spaces/characters", force_torque->Topic());
  }

  // Invalid topic
  {
    const std::string topic = "@@@";
    auto forcetorqueSdf = ForceTorqueToSdf(name, sensorPose,
          updateRate, topic, alwaysOn, visualize);

    auto sensor = factory.CreateSensor(forcetorqueSdf);
    ASSERT_EQ(nullptr, sensor);
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
