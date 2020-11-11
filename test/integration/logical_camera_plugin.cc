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

#include <ignition/common/Console.hh>
#include <ignition/common/Time.hh>

#include <ignition/sensors/LogicalCameraSensor.hh>
#include <ignition/sensors/SensorFactory.hh>
#include <ignition/sensors/Export.hh>

#include <ignition/math/Helpers.hh>
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <ignition/msgs.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif
#include <ignition/transport/Node.hh>

#include "test_config.h"  // NOLINT(build/include)

// undefine near and far macros from windows.h
#ifdef _WIN32
  #undef near
  #undef far
#endif

/// \brief Helper function to create a logical camera sdf element
sdf::ElementPtr LogicalCameraToSdf(const std::string &_name,
    const ignition::math::Pose3d &_pose, const double _updateRate,
    const std::string &_topic, const double _near,
    const double _far, const double _horzFov,
    const double _aspectRatio, const bool _alwaysOn,
    const bool _visualize)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='" << _name << "' type='logical_camera'>"
    << "      <pose>" << _pose << "</pose>"
    << "      <topic>" << _topic << "</topic>"
    << "      <update_rate>"<< _updateRate <<"</update_rate>"
    << "      <logical_camera>"
    << "        <near>" << _near << "</near>"
    << "        <far>" << _far << "</far>"
    << "        <horizontal_fov>" << _horzFov << "</horizontal_fov>"
    << "        <aspect_ratio>" << _aspectRatio << "</aspect_ratio>"
    << "      </logical_camera>"
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

class LogicalCameraSensorTest: public testing::Test
{
};

/////////////////////////////////////////////////
TEST_F(LogicalCameraSensorTest, CreateLogicalCamera)
{
  // Create SDF describing a logical camera sensor
  const std::string name = "TestLogicalCamera";
  const std::string topic = "/ignition/sensors/test/logical_camera";
  const double updateRate = 30;
  const double near = 0.55;
  const double far = 5;
  const double horzFov = 1.04719755;
  const double aspectRatio = 1.778;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d sensorPose(ignition::math::Vector3d(0.25, 0.0, 0.5),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr logicalCameraSdf = LogicalCameraToSdf(name, sensorPose,
        updateRate, topic, near, far, horzFov, aspectRatio, alwaysOn,
        visualize);

  // create the sensor using sensor factory
  ignition::sensors::SensorFactory sf;
  sf.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_PATH, "lib"));
  std::unique_ptr<ignition::sensors::LogicalCameraSensor> sensor =
      sf.CreateSensor<ignition::sensors::LogicalCameraSensor>(logicalCameraSdf);
  EXPECT_TRUE(sensor != nullptr);

  EXPECT_EQ(name, sensor->Name());
  EXPECT_EQ(topic, sensor->Topic());
  EXPECT_DOUBLE_EQ(updateRate, sensor->UpdateRate());
  EXPECT_DOUBLE_EQ(near, sensor->Near());
  EXPECT_DOUBLE_EQ(far, sensor->Far());
  EXPECT_NEAR(horzFov, sensor->HorizontalFOV().Radian(), 1e-3);
  EXPECT_DOUBLE_EQ(aspectRatio, sensor->AspectRatio());
}

/////////////////////////////////////////////////
/// \brief Test detecting one box
TEST_F(LogicalCameraSensorTest, DetectBox)
{
  // Create SDF describing a logical camera sensor
  const std::string name = "TestLogicalCamera";
  const std::string topic = "/ignition/sensors/test/logical_camera";
  const double updateRate = 30;
  const double near = 0.55;
  const double far = 5;
  const double horzFov = 1.04719755;
  const double aspectRatio = 1.778;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d sensorPose(ignition::math::Vector3d(0.25, 0.0, 0.5),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr logicalCameraSdf = LogicalCameraToSdf(name, sensorPose,
        updateRate, topic, near, far, horzFov, aspectRatio, alwaysOn,
        visualize);

  // create the sensor using sensor factory
  // try creating without specifying the sensor type and then cast it
  ignition::sensors::SensorFactory sf;
  sf.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_PATH, "lib"));
  std::unique_ptr<ignition::sensors::Sensor> s =
      sf.CreateSensor(logicalCameraSdf);
  std::unique_ptr<ignition::sensors::LogicalCameraSensor> sensor(
      dynamic_cast<ignition::sensors::LogicalCameraSensor *>(s.release()));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  // verify initial image
  auto img = sensor->Image();
  EXPECT_EQ(0, img.model().size());

  // Create testing boxes
  // 1. box in the center
  std::string boxName = "TestBox";
  ignition::math::Pose3d boxPose(ignition::math::Vector3d(2, 0, 0.5),
      ignition::math::Quaterniond::Identity);

  std::map<std::string, ignition::math::Pose3d> modelPoses;
  modelPoses[boxName] = boxPose;
  sensor->SetModelPoses(std::move(modelPoses));

  // update
  sensor->Update(std::chrono::steady_clock::duration::zero());

  // verify box is in image
  img = sensor->Image();
  EXPECT_EQ(sensorPose, ignition::msgs::Convert(img.pose()));
  EXPECT_EQ(1, img.model().size());
  EXPECT_EQ(boxName, img.model(0).name());
  ignition::math::Pose3d boxPoseCameraFrame = boxPose - sensorPose;
  EXPECT_EQ(boxPoseCameraFrame, ignition::msgs::Convert(img.model(0).pose()));

  // 2. test box outside of frustum
  std::map<std::string, ignition::math::Pose3d> modelPoses2;
  ignition::math::Pose3d boxPose2(ignition::math::Vector3d(8, 0, 0.5),
      ignition::math::Quaterniond::Identity);
  modelPoses2[boxName] = boxPose2;
  sensor->SetModelPoses(std::move(modelPoses2));

  // update
  sensor->Update(std::chrono::steady_clock::duration::zero());

  // verify box is not in the image
  img = sensor->Image();
  EXPECT_EQ(sensorPose, ignition::msgs::Convert(img.pose()));
  EXPECT_EQ(0, img.model().size());

  // 3. test with different sensor pose
  // camera now on y, orientated to face box
  std::map<std::string, ignition::math::Pose3d> modelPoses3;
  ignition::math::Pose3d sensorPose3(ignition::math::Vector3d(2, 2, 0.5),
      ignition::math::Quaterniond(0, 0, -1.57));
  sensor->SetPose(sensorPose3);

  ignition::math::Pose3d boxPose3(ignition::math::Vector3d(2, 0, 0.5),
      ignition::math::Quaterniond(0, 0, 1.57));
  modelPoses3[boxName] = boxPose3;
  sensor->SetModelPoses(std::move(modelPoses3));

  // update
  sensor->Update(std::chrono::steady_clock::duration::zero());

  // verify box is in image
  img = sensor->Image();
  EXPECT_EQ(sensorPose3, ignition::msgs::Convert(img.pose()));
  EXPECT_EQ(1, img.model().size());
  EXPECT_EQ(boxName, img.model(0).name());
  ignition::math::Pose3d boxPose3CameraFrame = boxPose3 - sensorPose3;
  EXPECT_EQ(boxPose3CameraFrame, ignition::msgs::Convert(img.model(0).pose()));

  // 4. rotate camera away and image should be empty
  ignition::math::Pose3d sensorPose4(ignition::math::Vector3d(2, 2, 0.5),
      ignition::math::Quaterniond(0, 0, 0));
  sensor->SetPose(sensorPose4);

  // update
  sensor->Update(std::chrono::steady_clock::duration::zero());

  // verify box is no longer in the image
  img = sensor->Image();
  EXPECT_EQ(sensorPose4, ignition::msgs::Convert(img.pose()));
  EXPECT_EQ(0, img.model().size());
}

/////////////////////////////////////////////////
TEST_F(LogicalCameraSensorTest, Topic)
{
  const std::string name = "TestLogicalCamera";
  const double updateRate = 30;
  const double near = 0.55;
  const double far = 5;
  const double horzFov = 1.04719755;
  const double aspectRatio = 1.778;
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
    auto logicalCameraSdf = LogicalCameraToSdf(name, sensorPose,
        updateRate, topic, near, far, horzFov, aspectRatio, alwaysOn,
        visualize);

    auto sensor = factory.CreateSensor(logicalCameraSdf);
    EXPECT_NE(nullptr, sensor);

    auto logicalCamera =
        dynamic_cast<ignition::sensors::LogicalCameraSensor *>(
        sensor.release());
    ASSERT_NE(nullptr, logicalCamera);

    EXPECT_EQ("/logical_camera", logicalCamera->Topic());
  }

  // Convert to valid topic
  {
    const std::string topic = "/topic with spaces/@~characters//";
    auto logicalCameraSdf = LogicalCameraToSdf(name, sensorPose,
        updateRate, topic, near, far, horzFov, aspectRatio, alwaysOn,
        visualize);

    auto sensor = factory.CreateSensor(logicalCameraSdf);
    EXPECT_NE(nullptr, sensor);

    auto logicalCamera =
        dynamic_cast<ignition::sensors::LogicalCameraSensor *>(
        sensor.release());
    ASSERT_NE(nullptr, logicalCamera);

    EXPECT_EQ("/topic_with_spaces/characters", logicalCamera->Topic());
  }

  // Invalid topic
  {
    const std::string topic = "@@@";
    auto logicalCameraSdf = LogicalCameraToSdf(name, sensorPose,
        updateRate, topic, near, far, horzFov, aspectRatio, alwaysOn,
        visualize);

    auto sensor = factory.CreateSensor(logicalCameraSdf);
    ASSERT_EQ(nullptr, sensor);
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
