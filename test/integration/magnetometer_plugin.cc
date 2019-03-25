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

#include <ignition/sensors/MagnetometerSensor.hh>
#include <ignition/sensors/SensorFactory.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "TransportTestTools.hh"

/// \brief Helper function to create an magnetometer sdf element
sdf::ElementPtr MagnetometerToSDF(const std::string &_name,
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
    << "    <sensor name='" << _name << "' type='magnetometer'>"
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

class MagnetometerSensorTest: public testing::Test
{
};

/////////////////////////////////////////////////
TEST_F(MagnetometerSensorTest, CreateMagnetometer)
{
  // Create SDF describing an magnetometer sensor
  const std::string name = "TestMagnetometer";
  const std::string topic = "/ignition/sensors/test/magnetometer";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d sensorPose(ignition::math::Vector3d(0.25, 0.0, 0.5),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr magnetometerSDF = MagnetometerToSDF(name, sensorPose,
        updateRate, topic, alwaysOn, visualize);

  // create the sensor using sensor factory
  ignition::sensors::SensorFactory sf;
  sf.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_PATH, "lib"));
  std::unique_ptr<ignition::sensors::MagnetometerSensor> sensor =
      sf.CreateSensor<ignition::sensors::MagnetometerSensor>(magnetometerSDF);
  EXPECT_TRUE(sensor != nullptr);

  EXPECT_EQ(name, sensor->Name());
  EXPECT_EQ(topic, sensor->Topic());
  EXPECT_DOUBLE_EQ(updateRate, sensor->UpdateRate());
}

/////////////////////////////////////////////////
TEST_F(MagnetometerSensorTest, SensorReadings)
{
  // Create SDF describing an magnetometer sensor
  const std::string name = "TestMagnetometer";
  const std::string topic = "/ignition/sensors/test/magnetometer";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d sensorPose(ignition::math::Vector3d(0.25, 0.0, 0.5),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr magnetometerSDF = MagnetometerToSDF(name, sensorPose,
        updateRate, topic, alwaysOn, visualize);

  // create the sensor using sensor factory
  // try creating without specifying the sensor type and then cast it
  ignition::sensors::SensorFactory sf;
  sf.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_PATH, "lib"));
  std::unique_ptr<ignition::sensors::Sensor> s =
      sf.CreateSensor(magnetometerSDF);
  std::unique_ptr<ignition::sensors::MagnetometerSensor> sensor(
      dynamic_cast<ignition::sensors::MagnetometerSensor *>(s.release()));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  // subscribe to the topic
  WaitForMessageTestHelper<ignition::msgs::Magnetometer> msgHelper(topic);

  // verify initial readings
  EXPECT_EQ(ignition::math::Pose3d::Zero, sensor->WorldPose());
  EXPECT_EQ(ignition::math::Vector3d::Zero, sensor->WorldMagneticField());
  EXPECT_EQ(ignition::math::Vector3d::Zero, sensor->MagneticField());

  // set magnetic field and verify
  ignition::math::Vector3d worldField(1, 2, -4);
  sensor->SetWorldMagneticField(worldField);
  EXPECT_EQ(worldField, sensor->WorldMagneticField());

  // set world pose and verify
  ignition::math::Vector3d position(1, 0, 3);
  ignition::math::Quaterniond orientation =
      ignition::math::Quaterniond::Identity;
  ignition::math::Pose3d pose(position, orientation);
  sensor->SetWorldPose(pose);
  EXPECT_EQ(pose, sensor->WorldPose());

  // update sensor and verify new readings
  // there are not sensor rotations so the magnetic fields in body frame and
  // world frame should be the same
  EXPECT_TRUE(sensor->Update(ignition::common::Time(1, 0)));
  EXPECT_EQ(pose, sensor->WorldPose());
  EXPECT_EQ(worldField, sensor->WorldMagneticField());
  EXPECT_EQ(worldField, sensor->MagneticField());

  // verify msg received on the topic
  EXPECT_TRUE(msgHelper.WaitForMessage()) << msgHelper;
  auto msg = msgHelper.Message();
  EXPECT_EQ(1, msg.header().stamp().sec());
  EXPECT_EQ(0, msg.header().stamp().nsec());
  EXPECT_EQ(worldField, ignition::msgs::Convert(msg.field_tesla()));

  // Rotate the magnetometer
  ignition::math::Quaterniond newOrientation(0, 3.14, 1.57);
  ignition::math::Pose3d newPose(position, newOrientation);
  sensor->SetWorldPose(newPose);
  EXPECT_EQ(newPose, sensor->WorldPose());

  // update sensor and verify new readings
  EXPECT_TRUE(sensor->Update(ignition::common::Time(2, 0)));
  EXPECT_EQ(worldField, sensor->WorldMagneticField());
  ignition::math::Vector3d localField =
      newOrientation.RotateVectorReverse(worldField);
  EXPECT_EQ(localField, sensor->MagneticField());

  // verify updated msg
  EXPECT_TRUE(msgHelper.WaitForMessage()) << msgHelper;
  msg = msgHelper.Message();
  EXPECT_EQ(2, msg.header().stamp().sec());
  EXPECT_EQ(0, msg.header().stamp().nsec());
  EXPECT_EQ(localField, ignition::msgs::Convert(msg.field_tesla()));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
