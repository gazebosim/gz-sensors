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

#include <sdf/ForceTorque.hh>

#include <ignition/math/Helpers.hh>

#include <ignition/sensors/ForceTorqueSensor.hh>
#include <ignition/sensors/SensorFactory.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "TransportTestTools.hh"

/// \brief Helper function to create a force torque sensor sdf element.
void CreateForceTorqueToSdf(const std::string &_name,
                            const ignition::math::Pose3d &_pose,
                            const double _updateRate, const std::string &_topic,
                            const bool _alwaysOn, const bool _visualize,
                            const ignition::math::Vector3d &_forceNoiseMean,
                            const ignition::math::Vector3d &_forceNoiseStddev,
                            const ignition::math::Vector3d &_torqueNoiseMean,
                            const ignition::math::Vector3d &_torqueNoiseStddev,
                            sdf::ElementPtr &_sensorSdf)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.7'>"
    << "  <model name='m1'>"
    << "    <link name='link1'/>"
    << "    <link name='link2'/>"
    << "    <joint name='joint1' type='fixed'>"
    << "      <parent>link1</parent>"
    << "      <child>link2</child>"
    << "      <sensor name='" << _name << "' type='force_torque'>"
    << "        <pose>" << _pose << "</pose>"
    << "        <topic>" << _topic << "</topic>"
    << "        <update_rate>"<< _updateRate <<"</update_rate>"
    << "        <alwaysOn>" << _alwaysOn <<"</alwaysOn>"
    << "        <visualize>" << _visualize << "</visualize>"
    << "        <force_torque>"
    << "          <frame>child</frame>"
    << "          <measure_direction>child_to_parent</measure_direction>"
    << "          <force>"
    << "            <x>"
    << "              <noise type='gaussian'>"
    << "                <mean>" << _forceNoiseMean.X() << "</mean>"
    << "                <stddev>" << _forceNoiseStddev.X() << "</stddev>"
    << "              </noise>"
    << "            </x>"
    << "            <y>"
    << "              <noise type='gaussian'>"
    << "                <mean>" << _forceNoiseMean.Y() << "</mean>"
    << "                <stddev>" << _forceNoiseStddev.Y() << "</stddev>"
    << "              </noise>"
    << "            </y>"
    << "            <z>"
    << "              <noise type='gaussian'>"
    << "                <mean>" << _forceNoiseMean.Z() << "</mean>"
    << "                <stddev>" << _forceNoiseStddev.Z() << "</stddev>"
    << "              </noise>"
    << "            </z>"
    << "          </force>"
    << "          <torque>"
    << "            <x>"
    << "              <noise type='gaussian'>"
    << "                <mean>" << _torqueNoiseMean.X() << "</mean>"
    << "                <stddev>" << _torqueNoiseStddev.X() << "</stddev>"
    << "              </noise>"
    << "            </x>"
    << "            <y>"
    << "              <noise type='gaussian'>"
    << "                <mean>" << _torqueNoiseMean.Y() << "</mean>"
    << "                <stddev>" << _torqueNoiseStddev.Y() << "</stddev>"
    << "              </noise>"
    << "            </y>"
    << "            <z>"
    << "              <noise type='gaussian'>"
    << "                <mean>" << _torqueNoiseMean.Z() << "</mean>"
    << "                <stddev>" << _torqueNoiseStddev.Z() << "</stddev>"
    << "              </noise>"
    << "            </z>"
    << "          </torque>"
    << "        </force_torque>"
    << "      </sensor>"
    << "    </joint>"
    << "  </model>"
    << "</sdf>";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(stream.str());
  if (errors.empty())
  {
    auto model = root.Model();
    ASSERT_NE(nullptr, model);
    auto joint = model->JointByIndex(0);
    ASSERT_NE(nullptr, joint);
    auto sensor = joint->SensorByIndex(0);
    ASSERT_NE(nullptr, sensor);
    _sensorSdf = sensor->Element();
  }
  else
  {
    _sensorSdf = nullptr;
  }
}

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
  sdf::ElementPtr forcetorqueSdf;
  CreateForceTorqueToSdf(name, sensorPose, updateRate, topic, alwaysOn,
                         visualize, {}, {}, {}, {}, forcetorqueSdf);

  ASSERT_NE(nullptr, forcetorqueSdf);

  // create the sensor using sensor factory
  ignition::sensors::SensorFactory sf;
  auto sensor =
      sf.CreateSensor<ignition::sensors::ForceTorqueSensor>(forcetorqueSdf);
  ASSERT_NE(nullptr, sensor);

  EXPECT_EQ(name, sensor->Name());
  EXPECT_EQ(topic, sensor->Topic());
  EXPECT_DOUBLE_EQ(updateRate, sensor->UpdateRate());
}

/////////////////////////////////////////////////
TEST_F(ForceTorqueSensorTest, SensorReadings)
{
  namespace math = ignition::math;

  // Create SDF describing a a force torque sensor
  const std::string name = "TestForceTorqueSensor";
  const std::string topic = "/ignition/sensors/test/force_torque";
  // const std::string topicNoise = "/ignition/sensors/test/altimeter_noise";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  const math::Vector3d forceNoiseMean{0.1, 0.2, 0.3};
  // Set the stddev to 0s so we can test the mean values
  const math::Vector3d forceNoiseStddev{0.0, 0.0, 0.0};

  const math::Vector3d torqueNoiseMean{0.5, 0.6, 0.7};
  // Set the stddev to 0s so we can test the mean values
  const math::Vector3d torqueNoiseStddev{0.0, 0.0, 0.0};

  // Create sensor SDF
  math::Pose3d sensorPose{math::Vector3d(0.25, 0.0, 0.5),
                          math::Quaterniond::Identity};
  sdf::ElementPtr forcetorqueSdf;
  CreateForceTorqueToSdf(name, sensorPose, updateRate, topic, alwaysOn,
                         visualize, forceNoiseMean, forceNoiseStddev,
                         torqueNoiseMean, torqueNoiseStddev, forcetorqueSdf);

  ASSERT_NE(nullptr, forcetorqueSdf);

  // create the sensor using sensor factory
  // try creating without specifying the sensor type and then cast it
  ignition::sensors::SensorFactory sf;
  auto sensor =
      sf.CreateSensor<ignition::sensors::ForceTorqueSensor>(forcetorqueSdf);

  ASSERT_NE(nullptr, sensor);

  // verify initial readings
  EXPECT_EQ(math::Vector3d::Zero, sensor->Force());
  EXPECT_EQ(math::Vector3d::Zero, sensor->Torque());
  EXPECT_EQ(math::Quaterniond::Identity, sensor->RotationParentInSensor());
  EXPECT_EQ(math::Quaterniond::Identity, sensor->RotationChildInSensor());

  // set state and verify readings
  math::Vector3d force{0, 0, 1};
  sensor->SetForce(force);
  EXPECT_EQ(force, sensor->Force());

  math::Vector3d torque{0, 1, 0};
  sensor->SetTorque(torque);
  EXPECT_EQ(torque, sensor->Torque());

  // verify msg received on the topic
  WaitForMessageTestHelper<ignition::msgs::Wrench> msgHelper(topic);
  auto dt = std::chrono::steady_clock::duration(std::chrono::seconds(1));
  {
    sensor->Update(dt);
    EXPECT_TRUE(msgHelper.WaitForMessage()) << msgHelper;
    auto msg = msgHelper.Message();
    EXPECT_EQ(1, msg.header().stamp().sec());
    EXPECT_EQ(0, msg.header().stamp().nsec());

    EXPECT_EQ(force + forceNoiseMean, ignition::msgs::Convert(msg.force()));
    EXPECT_EQ(torque + torqueNoiseMean, ignition::msgs::Convert(msg.torque()));

    // The Force() and Torque() functions return the noise-free forces and 
    // torques in the sensor frame
    EXPECT_EQ(force, sensor->Force());
    EXPECT_EQ(torque, sensor->Torque());
  }

  // Set rotation of child
  const math::Quaterniond rotChildInSensor{
      math::Vector3d{IGN_PI_4, IGN_PI_2, 0}};
  sensor->SetRotationChildInSensor(rotChildInSensor);
  EXPECT_EQ(rotChildInSensor, sensor->RotationChildInSensor());
  {
    sensor->Update(dt);
    EXPECT_TRUE(msgHelper.WaitForMessage()) << msgHelper;
    auto msg = msgHelper.Message();

    EXPECT_EQ((rotChildInSensor.Inverse() * force) + forceNoiseMean,
              ignition::msgs::Convert(msg.force()));
    EXPECT_EQ((rotChildInSensor.Inverse() * torque) + torqueNoiseMean,
              ignition::msgs::Convert(msg.torque()));
    // The Force() and Torque() functions return the noise-free forces and 
    // torques in the sensor frame
    EXPECT_EQ(force, sensor->Force());
    EXPECT_EQ(torque, sensor->Torque());
  }

  // Set rotation of parent. This should not affect the ouptut because the 
  // measurement frame is "child".
  const math::Quaterniond rotParentInSensor{math::Vector3d{0.1, 0.2, 0.3}};
  sensor->SetRotationParentInSensor(rotParentInSensor);
  EXPECT_EQ(rotParentInSensor, sensor->RotationParentInSensor());
  {
    sensor->Update(dt);
    EXPECT_TRUE(msgHelper.WaitForMessage()) << msgHelper;
    auto msg = msgHelper.Message();

    // Using rotChildInSensor since the measurement frame is still "child"
    EXPECT_EQ((rotChildInSensor.Inverse() * force) + forceNoiseMean,
              ignition::msgs::Convert(msg.force()));
    EXPECT_EQ((rotChildInSensor.Inverse() * torque) + torqueNoiseMean,
              ignition::msgs::Convert(msg.torque()));
    // The Force() and Torque() functions return the noise-free forces and 
    // torques in the sensor frame
    EXPECT_EQ(force, sensor->Force());
    EXPECT_EQ(torque, sensor->Torque());
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
