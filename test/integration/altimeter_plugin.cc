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

#include <ignition/sensors/AltimeterSensor.hh>
#include <ignition/sensors/SensorFactory.hh>

#include "test_config.h"  // NOLINT(build/include)

/// \brief Helper function to create an altimeter sdf element
sdf::ElementPtr AltimeterToSDF(const std::string &_name,
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

class AltimeterSensorTest: public testing::Test
{
};

/////////////////////////////////////////////////
TEST_F(AltimeterSensorTest, CreateAltimeter)
{
  // Create SDF describing an altimeter sensor
  const std::string name = "TestAltimeter";
  const std::string topic = "/ignition/sensors/test/altimeter";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d sensorPose(ignition::math::Vector3d(0.25, 0.0, 0.5),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr altimeterSDF = AltimeterToSDF(name, sensorPose,
        updateRate, topic, alwaysOn, visualize);

  // create the sensor using sensor factory
  ignition::sensors::SensorFactory sf;
  sf.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_PATH, "lib"));
  std::unique_ptr<ignition::sensors::AltimeterSensor> sensor =
      sf.CreateSensor<ignition::sensors::AltimeterSensor>(altimeterSDF);
  EXPECT_TRUE(sensor != nullptr);

  EXPECT_EQ(name, sensor->Name());
  EXPECT_EQ(topic, sensor->Topic());
  EXPECT_DOUBLE_EQ(updateRate, sensor->UpdateRate());
}

/////////////////////////////////////////////////
TEST_F(AltimeterSensorTest, SensorReadings)
{
  // Create SDF describing an altimeter sensor
  const std::string name = "TestAltimeter";
  const std::string topic = "/ignition/sensors/test/altimeter";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d sensorPose(ignition::math::Vector3d(0.25, 0.0, 0.5),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr altimeterSDF = AltimeterToSDF(name, sensorPose,
        updateRate, topic, alwaysOn, visualize);

  // create the sensor using sensor factory
  // try creating without specifying the sensor type and then cast it
  ignition::sensors::SensorFactory sf;
  sf.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_PATH, "lib"));
  std::unique_ptr<ignition::sensors::Sensor> s =
      sf.CreateSensor(altimeterSDF);
  std::unique_ptr<ignition::sensors::AltimeterSensor> sensor(
      dynamic_cast<ignition::sensors::AltimeterSensor *>(s.release()));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  // verify initial readings
  EXPECT_DOUBLE_EQ(0.0, sensor->VerticalReference());
  EXPECT_DOUBLE_EQ(0.0, sensor->VerticalVelocity());
  EXPECT_DOUBLE_EQ(0.0, sensor->VerticalPosition());

  // set state and verify readings
  double vertRef = 1.0;
  sensor->SetVerticalReference(vertRef);
  EXPECT_DOUBLE_EQ(vertRef, sensor->VerticalReference());

  double pos = 2.0;
  sensor->SetPosition(pos);
  EXPECT_DOUBLE_EQ(pos - vertRef, sensor->VerticalPosition());

  double vertVel = 3.0;
  sensor->SetVerticalVelocity(vertVel);
  EXPECT_DOUBLE_EQ(vertVel, sensor->VerticalVelocity());

  pos = -6.0;
  sensor->SetPosition(pos);
  EXPECT_DOUBLE_EQ(pos - vertRef, sensor->VerticalPosition());
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
