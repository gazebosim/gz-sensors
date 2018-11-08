/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include "test/test_config.hh"
#include <sdf/sdf.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/Event.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/msgs.hh>
#include "TransportTestTools.hh"

#include <ignition/math/Angle.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/msgs.hh>
#include <ignition/rendering.hh>
#include <ignition/sensors/Export.hh>
#include <ignition/sensors/Manager.hh>

#include <ignition/sensors/GpuLidarSensor.hh>

#define LASER_TOL 1e-4
#define DOUBLE_TOL 1e-6

// vertical range values seem to be less accurate
#define VERTICAL_LASER_TOL 1e-3
#define WAIT_TIME 0.02

sdf::ElementPtr GpuLidarToSDF(std::string name, ignition::math::Pose3d &pose, double update_rate, std::string topic,
    double horz_samples, double horz_resolution, double horz_min_angle, double horz_max_angle,
    double vert_samples, double vert_resolution, double vert_min_angle, double vert_max_angle,
    double range_resolution, double range_min, double range_max, bool always_on, bool visualize)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='" << name << "' type='gpu_lidar'>"
    << "      <pose>" << pose << "</pose>"
    << "      <topic>" << topic << "</topic>"
    << "      <update_rate>"<< update_rate <<"</update_rate>"
    << "      <ray>"
    << "        <scan>"
    << "          <horizontal>"
    << "            <samples>" << horz_samples << "</samples>"
    << "            <resolution>" << horz_resolution << "</resolution>"
    << "            <min_angle>" << horz_min_angle << "</min_angle>"
    << "            <max_angle>" << horz_max_angle << "</max_angle>"
    << "          </horizontal>"
    << "          <vertical>"
    << "            <samples>" << vert_samples << "</samples>"
    << "            <resolution>" << vert_resolution << "</resolution>"
    << "            <min_angle>" << vert_min_angle << "</min_angle>"
    << "            <max_angle>" << vert_max_angle << "</max_angle>"
    << "          </vertical>"
    << "        </scan>"
    << "        <range>"
    << "          <min>" << range_min << "</min>"
    << "          <max>" << range_max << "</max>"
    << "          <resolution>" << range_resolution << "</resolution>"
    << "        </range>"
    << "      </ray>"
    << "      <always_on>"<< always_on <<"</always_on>"
    << "      <visualize>" << visualize << "</visualize>"
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

unsigned int g_laserCounter = 0;

void OnNewLidarFrame(const float * /*_scan*/, unsigned int /*_width*/,
    unsigned int /*_height*/, unsigned int /*_channels*/,
    const std::string &/*_format*/)
{
  g_laserCounter++;
}

/////////////////////////////////////////////////
/// \brief Test Creation of a Ray sensor
TEST(GpuLidarSensor_TEST, CreateLidar)
{
  // // Setup ign-rendering with a scene
  // auto *engine = ignition::rendering::engine("ogre");
  // EXPECT_TRUE(engine != nullptr);
  //
  // ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  // ignition::rendering::VisualPtr root = scene->RootVisual();
  //
  // // Create a sensor manager
  // ignition::sensors::Manager mgr;
  // mgr.SetRenderingScene(scene);
  // mgr.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_DIR, "lib"));
  //
  // // Create SDF describing a camera sensor
  // const std::string name = "TestGpuLidar";
  // const std::string topic = "/ignition/sensors/test/lidar";
  // const double update_rate = 30;
  // const double horz_samples = 640;
  // const double horz_resolution = 1;
  // const double horz_min_angle = -1.396263;
  // const double horz_max_angle = 1.396263;
  // const double vert_resolution = 1;
  // const double vert_samples = 1;
  // const double vert_min_angle = 0;
  // const double vert_max_angle = 0;
  // const double range_resolution = 0.01;
  // const double range_min = 0.08;
  // const double range_max = 10.0;
  // const bool always_on = 1;
  // const bool visualize = 1;
  //
  // // Create an scene with a box in it
  // scene->SetAmbientLight(0.3, 0.3, 0.3);
  //
  // ignition::math::Pose3d test_pose(ignition::math::Vector3d(0, 0, 0.1),
  //     ignition::math::Quaterniond::Identity);
  //
  // // Create sensor description in SDF
  // sdf::ElementPtr lidarSDF = GpuLidarToSDF(name, test_pose, update_rate, topic,
  //   horz_samples, horz_resolution, horz_min_angle, horz_max_angle,
  //   vert_samples, vert_resolution, vert_min_angle, vert_max_angle,
  //   range_resolution, range_min, range_max, always_on, visualize);
  //
  // // Create a GpuLidarSensor
  // auto sensor = mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(
  //     lidarSDF);
  //
  // // Make sure the above dynamic cast worked.
  // EXPECT_TRUE(sensor != nullptr);
  //
  // // Set a callback on the lidar sensor to get a scan
  // ignition::common::ConnectionPtr c =
  //   sensor->ConnectNewLidarFrame(
  //       std::bind(&::OnNewLidarFrame,
  //         std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
  //         std::placeholders::_4, std::placeholders::_5));
  //
  // EXPECT_TRUE(c != nullptr);
  //
  // double angleRes = (sensor->AngleMax() - sensor->AngleMin()).Radian() /
  //                   sensor->RayCount();
  // EXPECT_EQ(sensor->AngleMin(), ignition::math::Angle(horz_min_angle));
  // EXPECT_EQ(sensor->AngleMax(), ignition::math::Angle(horz_max_angle));
  // EXPECT_NEAR(sensor->RangeMin(), range_min, 1e-6);
  // EXPECT_NEAR(sensor->RangeMax(), range_max, 1e-6);
  // EXPECT_NEAR(sensor->AngleResolution(), angleRes, 1e-3);
  // EXPECT_NEAR(sensor->RangeResolution(), range_resolution, 1e-3);
  // EXPECT_EQ(sensor->RayCount(), horz_samples);
  // EXPECT_EQ(sensor->RangeCount(), horz_samples);
  //
  // EXPECT_EQ(sensor->VerticalRayCount(), vert_samples);
  // EXPECT_EQ(sensor->VerticalRangeCount(), vert_samples);
  // EXPECT_EQ(sensor->VerticalAngleMin(), vert_min_angle);
  // EXPECT_EQ(sensor->VerticalAngleMax(), vert_max_angle);
  //
  // EXPECT_TRUE(sensor->IsActive());
  //
  // g_laserCounter = 0;
  // WaitForMessageTestHelper<ignition::msgs::LaserScan> helper(topic);
  //
  // // Update once to verify that a message is sent
  // mgr.RunOnce(ignition::common::Time::Zero);
  //
  // EXPECT_TRUE(helper.WaitForMessage()) << helper;
  //
  // // Verify that the callback is called
  // EXPECT_EQ(g_laserCounter, 1);
  //
  // // Get all the range values
  // std::vector<double> ranges;
  // sensor->Ranges(ranges);
  // EXPECT_EQ(ranges.size(), static_cast<size_t>(horz_samples * vert_samples));
  //
  // // Check that all the range values are +inf
  // for (unsigned int i = 0; i < ranges.size(); ++i)
  // {
  //   EXPECT_DOUBLE_EQ(ranges[i], ignition::math::INF_D);
  //   EXPECT_DOUBLE_EQ(sensor->Range(i), ranges[i]);
  //   EXPECT_NEAR(sensor->Retro(i), 0, 1e-6);
  //   EXPECT_EQ(sensor->Fiducial(i), -1);
  // }
  //
  // // Clean up
  // c.reset();
  // engine->DestroyScene(scene);
}

/////////////////////////////////////////////////
/// \brief Test detect three different boxes
TEST(GpuLidarSensor_TEST, DetectBoxes)
{
  // Create SDF describing a camera sensor
  const std::string name = "TestGpuLidar";
  const std::string topic = "/ignition/sensors/test/lidar";
  const double update_rate = 30;
  const double horz_samples = 320;
  const double horz_resolution = 1;
  const double horz_min_angle = -M_PI/2.0;
  const double horz_max_angle = M_PI/2.0;
  const double vert_resolution = 1;
  const double vert_samples = 1;
  const double vert_min_angle = 0;
  const double vert_max_angle = 0;
  const double range_resolution = 0.01;
  const double range_min = 0.08;
  const double range_max = 10.0;
  const bool always_on = 1;
  const bool visualize = 1;

  ignition::math::Pose3d test_pose(ignition::math::Vector3d(0.3, 0.1, 0.1),
      ignition::math::Quaterniond::Identity);

  // Create sensor SDF
  sdf::ElementPtr lidarSDF = GpuLidarToSDF(name, test_pose, update_rate, topic,
    horz_samples, horz_resolution, horz_min_angle, horz_max_angle,
    vert_samples, vert_resolution, vert_min_angle, vert_max_angle,
    range_resolution, range_min, range_max, always_on, visualize);

  // Setup ign-rendering with a scene
  auto *engine = ignition::rendering::engine("ogre");
  EXPECT_TRUE(engine != nullptr);

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  ignition::rendering::VisualPtr root = scene->RootVisual();

  scene->SetAmbientLight(0.3, 0.3, 0.3);

  // Create testing boxes
  // box in the center
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(1, 0, 0.5),
      ignition::math::Quaterniond::Identity);
  ignition::rendering::VisualPtr visualBox1 = scene->CreateVisual("VerticalTestBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetWorldPosition(box01Pose.Pos());
  visualBox1->SetWorldRotation(box01Pose.Rot());
  root->AddChild(visualBox1);

  // Create a sensor manager
  ignition::sensors::Manager mgr;
  mgr.SetRenderingScene(scene);
  mgr.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_DIR, "lib"));

  // Create a GpuLidarSensor
  auto sensor = mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(
      lidarSDF);

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  // Set a callback on the lidar sensor to get a scan
  ignition::common::ConnectionPtr c =
    sensor->ConnectNewLidarFrame(
        std::bind(&::OnNewLidarFrame,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  EXPECT_TRUE(c != nullptr);

  // Verify that the callback is called
  g_laserCounter = 0;
  mgr.RunOnce(ignition::common::Time::Zero);
  EXPECT_EQ(g_laserCounter, 1);

  // Get all the range values
  std::vector<double> ranges;
  sensor->Ranges(ranges);
  EXPECT_EQ(ranges.size(), static_cast<size_t>(horz_samples * vert_samples));

  int mid = horz_samples / 2;
  int last = (horz_samples - 1);
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPointBox1 = abs(box01Pose.Pos().X()) - unitBoxSize/2;

  // rays caster 1 should see box01 and box02
  EXPECT_NEAR(sensor->Range(mid), expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_DOUBLE_EQ(sensor->Range(last), ignition::math::INF_D);

  // Clean up
  c.reset();
  engine->DestroyScene(scene);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
