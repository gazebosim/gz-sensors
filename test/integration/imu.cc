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

#include <gz/sensors/ImuSensor.hh>
#include <gz/sensors/SensorFactory.hh>

#include "test_config.hh"  // NOLINT(build/include)
#include "TransportTestTools.hh"

/// \brief Helper function to create an imu sdf element
sdf::ElementPtr ImuToSdf(const std::string &_name,
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
    << "    <sensor name='" << _name << "' type='imu'>"
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

/// \brief Test IMU sensor
class ImuSensorTest: public testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);
  }
};

/////////////////////////////////////////////////
TEST_F(ImuSensorTest, CreateImu)
{
  // Create SDF describing an imu sensor
  const std::string name = "TestImu";
  const std::string topic = "/ignition/sensors/test/imu";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  gz::math::Pose3d sensorPose(gz::math::Vector3d(0.25, 0.0, 0.5),
      gz::math::Quaterniond::Identity);
  sdf::ElementPtr imuSdf = ImuToSdf(name, sensorPose,
        updateRate, topic, alwaysOn, visualize);

  // create the sensor using sensor factory
  gz::sensors::SensorFactory sf;
  std::unique_ptr<gz::sensors::ImuSensor> sensor =
      sf.CreateSensor<gz::sensors::ImuSensor>(imuSdf);
  ASSERT_NE(nullptr, sensor);

  EXPECT_EQ(name, sensor->Name());
  EXPECT_EQ(topic, sensor->Topic());
  EXPECT_DOUBLE_EQ(updateRate, sensor->UpdateRate());
}

/////////////////////////////////////////////////
TEST_F(ImuSensorTest, SensorReadings)
{
  // Create SDF describing an imu sensor
  const std::string name = "TestImu";
  const std::string topic = "/ignition/sensors/test/imu";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  gz::math::Pose3d sensorPose(gz::math::Vector3d(0.25, 0.0, 0.5),
      gz::math::Quaterniond::Identity);
  sdf::ElementPtr imuSdf = ImuToSdf(name, sensorPose,
        updateRate, topic, alwaysOn, visualize);

  // create the sensor using sensor factory
  gz::sensors::SensorFactory sf;
  auto sensor = sf.CreateSensor<gz::sensors::ImuSensor>(imuSdf);
  ASSERT_NE(nullptr, sensor);
  EXPECT_FALSE(sensor->HasConnections());

  // subscribe to the topic
  WaitForMessageTestHelper<gz::msgs::IMU> msgHelper(topic);
  EXPECT_TRUE(sensor->HasConnections());

  // verify initial readings
  EXPECT_EQ(gz::math::Pose3d::Zero, sensor->WorldPose());
  EXPECT_EQ(gz::math::Vector3d::Zero, sensor->LinearAcceleration());
  EXPECT_EQ(gz::math::Vector3d::Zero, sensor->AngularVelocity());
  EXPECT_EQ(gz::math::Vector3d::Zero, sensor->Gravity());
  EXPECT_EQ(gz::math::Quaterniond::Identity,
      sensor->OrientationReference());
  EXPECT_EQ(gz::math::Quaterniond::Identity, sensor->Orientation());


  // 1. Verify imu readings at rest

  // set orientation reference and verify readings
  // this sets the initial imu rotation to be +90 degrees in z
  gz::math::Quaterniond orientRef(
    gz::math::Vector3d(0, 0, 1.57));
  sensor->SetOrientationReference(orientRef);
  EXPECT_EQ(orientRef, sensor->OrientationReference());

  // set gravity and verify
  gz::math::Vector3d gravity(0, 0, -4);
  sensor->SetGravity(gravity);
  EXPECT_EQ(gravity, sensor->Gravity());

  // set world pose and verify
  gz::math::Vector3d position(1, 0, 3);
  gz::math::Quaterniond orientation =
      gz::math::Quaterniond::Identity;
  gz::math::Pose3d pose(position, orientation);
  sensor->SetWorldPose(pose);
  EXPECT_EQ(pose, sensor->WorldPose());

  // orientation should still be identity before update
  EXPECT_EQ(gz::math::Quaterniond::Identity, sensor->Orientation());

  // update sensor and verify new readings
  EXPECT_TRUE(sensor->Update(std::chrono::steady_clock::duration(
    std::chrono::seconds(1))));
  EXPECT_EQ(orientRef, sensor->OrientationReference());
  EXPECT_EQ(gravity, sensor->Gravity());
  EXPECT_EQ(pose, sensor->WorldPose());
  EXPECT_EQ(gz::math::Vector3d::Zero, sensor->AngularVelocity());
  EXPECT_EQ(-gravity, sensor->LinearAcceleration());
  EXPECT_EQ(gz::math::Quaterniond(gz::math::Vector3d(0, 0, -1.57)),
      sensor->Orientation());

  // verify msg received on the topic
  EXPECT_TRUE(msgHelper.WaitForMessage()) << msgHelper;
  auto msg = msgHelper.Message();
  EXPECT_EQ(1, msg.header().stamp().sec());
  EXPECT_EQ(0, msg.header().stamp().nsec());
  EXPECT_EQ(gz::math::Vector3d::Zero,
      gz::msgs::Convert(msg.angular_velocity()));
  EXPECT_EQ(-gravity, gz::msgs::Convert(msg.linear_acceleration()));
  EXPECT_EQ(gz::math::Quaterniond(gz::math::Vector3d(0, 0, -1.57)),
      gz::msgs::Convert(msg.orientation()));

  // 2. Turn imu upside down, give it some linear acc and angular velocity and
  // verify readings

  // set angular velocity and verify
  gz::math::Vector3d angularVel(1.0, 2.0, 3.0);
  sensor->SetAngularVelocity(angularVel);
  EXPECT_EQ(angularVel, sensor->AngularVelocity());

  // set linear acceleration and verify
  gz::math::Vector3d linearAcc(0, 0, 3);
  sensor->SetLinearAcceleration(linearAcc);
  EXPECT_EQ(linearAcc, sensor->LinearAcceleration());

  // set orientation and verify
  gz::math::Quaterniond newOrientation(0, 3.14, 0);
  gz::math::Pose3d newPose(position, newOrientation);
  sensor->SetWorldPose(newPose);
  EXPECT_EQ(newPose, sensor->WorldPose());

  // update sensor and verify new readings
  EXPECT_TRUE(sensor->Update(std::chrono::steady_clock::duration(
    std::chrono::seconds(2)), false));
  EXPECT_EQ(orientRef, sensor->OrientationReference());
  EXPECT_EQ(gravity, sensor->Gravity());
  EXPECT_EQ(angularVel, sensor->AngularVelocity());
  EXPECT_EQ(newPose, sensor->WorldPose());
  gz::math::Vector3d expectedLinAcc = linearAcc + gravity;
  EXPECT_NEAR(expectedLinAcc.X(), sensor->LinearAcceleration().X(), 1e-2);
  EXPECT_NEAR(expectedLinAcc.Y(), sensor->LinearAcceleration().Y(), 1e-6);
  EXPECT_NEAR(expectedLinAcc.Z(), sensor->LinearAcceleration().Z(), 1e-5);
  EXPECT_EQ(
      gz::math::Quaterniond(gz::math::Vector3d(0, 3.14, -1.57)),
      sensor->Orientation());

  // verify updated msg
  EXPECT_TRUE(msgHelper.WaitForMessage()) << msgHelper;
  msg = msgHelper.Message();
  EXPECT_EQ(2, msg.header().stamp().sec());
  EXPECT_EQ(0, msg.header().stamp().nsec());
  EXPECT_EQ(angularVel,
      gz::msgs::Convert(msg.angular_velocity()));
  gz::math::Vector3d actualLinAcc =
      gz::msgs::Convert(msg.linear_acceleration());
  EXPECT_NEAR(expectedLinAcc.X(), actualLinAcc.X(), 1e-2);
  EXPECT_NEAR(expectedLinAcc.Y(), actualLinAcc.Y(), 1e-6);
  EXPECT_NEAR(expectedLinAcc.Z(), actualLinAcc.Z(), 1e-5);
  EXPECT_EQ(
      gz::math::Quaterniond(gz::math::Vector3d(0, 3.14, -1.57)),
      gz::msgs::Convert(msg.orientation()));
}

/////////////////////////////////////////////////
TEST_F(ImuSensorTest, Topic)
{
  const std::string name = "TestImu";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;
  auto sensorPose = gz::math::Pose3d();

  // Factory
  gz::sensors::SensorFactory factory;

  // Default topic
  {
    const std::string topic;
    auto imuSdf = ImuToSdf(name, sensorPose,
          updateRate, topic, alwaysOn, visualize);

    auto imu = factory.CreateSensor<gz::sensors::ImuSensor>(imuSdf);
    ASSERT_NE(nullptr, imu);

    EXPECT_EQ("/imu", imu->Topic());
  }

  // Convert to valid topic
  {
    const std::string topic = "/topic with spaces/@~characters//";
    auto imuSdf = ImuToSdf(name, sensorPose,
          updateRate, topic, alwaysOn, visualize);

    auto imu = factory.CreateSensor<gz::sensors::ImuSensor>(imuSdf);
    ASSERT_NE(nullptr, imu);

    EXPECT_EQ("/topic_with_spaces/characters", imu->Topic());
  }

  // Invalid topic
  {
    const std::string topic = "@@@";
    auto imuSdf = ImuToSdf(name, sensorPose,
          updateRate, topic, alwaysOn, visualize);

    auto sensor = factory.CreateSensor<gz::sensors::ImuSensor>(imuSdf);
    ASSERT_EQ(nullptr, sensor);
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
