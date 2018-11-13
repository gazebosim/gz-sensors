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
#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/Event.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/sensors/Export.hh>
#include <ignition/sensors/GpuLidarSensor.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/msgs.hh>
#include <ignition/rendering.hh>
#include <sdf/sdf.hh>

#include "test/test_config.hh"  // NOLINT(build/include)
#include "TransportTestTools.hh"

#define LASER_TOL 1e-4
#define DOUBLE_TOL 1e-6

// vertical range values seem to be less accurate
#define VERTICAL_LASER_TOL 1e-3
#define WAIT_TIME 0.02

sdf::ElementPtr GpuLidarToSDF(const std::string &name,
    const ignition::math::Pose3d &pose, const double updateRate,
    const std::string &topic, const double horzSamples,
    const double horzResolution, const double horzMinAngle,
    const double horzMaxAngle, const double vertSamples,
    const double vertResolution, const double vertMinAngle,
    const double vertMaxAngle, const double rangeResolution,
    const double rangeMin, const double rangeMax, const bool alwaysOn,
    const bool visualize)
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
    << "      <updateRate>"<< updateRate <<"</updateRate>"
    << "      <ray>"
    << "        <scan>"
    << "          <horizontal>"
    << "            <samples>" << horzSamples << "</samples>"
    << "            <resolution>" << horzResolution << "</resolution>"
    << "            <min_angle>" << horzMinAngle << "</min_angle>"
    << "            <max_angle>" << horzMaxAngle << "</max_angle>"
    << "          </horizontal>"
    << "          <vertical>"
    << "            <samples>" << vertSamples << "</samples>"
    << "            <resolution>" << vertResolution << "</resolution>"
    << "            <min_angle>" << vertMinAngle << "</min_angle>"
    << "            <max_angle>" << vertMaxAngle << "</max_angle>"
    << "          </vertical>"
    << "        </scan>"
    << "        <range>"
    << "          <min>" << rangeMin << "</min>"
    << "          <max>" << rangeMax << "</max>"
    << "          <resolution>" << rangeResolution << "</resolution>"
    << "        </range>"
    << "      </ray>"
    << "      <alwaysOn>"<< alwaysOn <<"</alwaysOn>"
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

class GpuLidarSensorTest: public testing::Test,
                  public testing::WithParamInterface<const char *>
{
  // Test and verify gpu rays properties setters and getters
  public: void CreateGpuLidar(const std::string &_renderEngine);

  // Test single box detection
  public: void DetectBox(const std::string &_renderEngine);

  // Test vertical measurements
  public: void TestThreeBoxes(const std::string &_renderEngine);

  // Test vertical measurements
  public: void VerticalLidar(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
/// \brief Test Creation of a GPU Lidar sensor
void GpuLidarSensorTest::CreateGpuLidar(const std::string &_renderEngine)
{
  // Create SDF describing a camera sensor
  const std::string name = "TestGpuLidar";
  const std::string topic = "/ignition/sensors/test/lidar";
  const double updateRate = 30;
  const double horzSamples = 640;
  const double horzResolution = 1;
  const double horzMinAngle = -1.396263;
  const double horzMaxAngle = 1.396263;
  const double vertResolution = 1;
  const double vertSamples = 1;
  const double vertMinAngle = 0;
  const double vertMaxAngle = 0;
  const double rangeResolution = 0.01;
  const double rangeMin = 0.08;
  const double rangeMax = 10.0;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor description in SDF
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr lidarSDF = GpuLidarToSDF(name, testPose, updateRate, topic,
    horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
    vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
    rangeResolution, rangeMin, rangeMax, alwaysOn, visualize);

  // Create and populate scene
  ignition::rendering::RenderEngine *engine =
    ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  ignition::rendering::VisualPtr root = scene->RootVisual();

  // Create a sensor manager
  ignition::sensors::Manager mgr;
  mgr.SetRenderingScene(scene);
  mgr.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_DIR, "lib"));

  // Create an scene with a box in it
  scene->SetAmbientLight(0.3, 0.3, 0.3);

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

  double angleRes = (sensor->AngleMax() - sensor->AngleMin()).Radian() /
                    sensor->RayCount();
  EXPECT_EQ(sensor->AngleMin(), ignition::math::Angle(horzMinAngle));
  EXPECT_EQ(sensor->AngleMax(), ignition::math::Angle(horzMaxAngle));
  EXPECT_NEAR(sensor->RangeMin(), rangeMin, 1e-6);
  EXPECT_NEAR(sensor->RangeMax(), rangeMax, 1e-6);
  EXPECT_NEAR(sensor->AngleResolution(), angleRes, 1e-3);
  EXPECT_NEAR(sensor->RangeResolution(), rangeResolution, 1e-3);
  EXPECT_EQ(sensor->RayCount(), horzSamples);
  EXPECT_EQ(sensor->RangeCount(), horzSamples);

  EXPECT_EQ(sensor->VerticalRayCount(), vertSamples);
  EXPECT_EQ(sensor->VerticalRangeCount(), vertSamples);
  EXPECT_EQ(sensor->VerticalAngleMin(), vertMinAngle);
  EXPECT_EQ(sensor->VerticalAngleMax(), vertMaxAngle);

  EXPECT_TRUE(sensor->IsActive());

  g_laserCounter = 0;
  WaitForMessageTestHelper<ignition::msgs::LaserScan> helper(topic);

  // Update once to verify that a message is sent
  mgr.RunOnce(ignition::common::Time::Zero);

  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  // Verify that the callback is called
  EXPECT_EQ(g_laserCounter, 1);

  // Get all the range values
  std::vector<double> ranges;
  sensor->Ranges(ranges);
  EXPECT_EQ(ranges.size(), static_cast<size_t>(horzSamples * vertSamples));

  // Check that all the range values are +inf
  for (unsigned int i = 0; i < ranges.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(ranges[i], ignition::math::INF_D);
    EXPECT_DOUBLE_EQ(sensor->Range(i), ranges[i]);
    EXPECT_NEAR(sensor->Retro(i), 0, 1e-6);
    EXPECT_EQ(sensor->Fiducial(i), -1);
  }

  // Clean up
  c.reset();
  engine->DestroyScene(scene);
}

/////////////////////////////////////////////////
/// \brief Test detect one box
void GpuLidarSensorTest::DetectBox(const std::string &_renderEngine)
{
  // Create SDF describing a camera sensor
  const std::string name = "TestGpuLidar";
  const std::string topic = "/ignition/sensors/test/lidar";
  const double updateRate = 30;
  const double horzSamples = 320;
  const double horzResolution = 1;
  const double horzMinAngle = -M_PI/2.0;
  const double horzMaxAngle = M_PI/2.0;
  const double vertResolution = 1;
  const double vertSamples = 1;
  const double vertMinAngle = 0;
  const double vertMaxAngle = 0;
  const double rangeResolution = 0.01;
  const double rangeMin = 0.08;
  const double rangeMax = 10.0;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0.0, 0.0, 0.1),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr lidarSDF = GpuLidarToSDF(name, testPose, updateRate, topic,
    horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
    vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
    rangeResolution, rangeMin, rangeMax, alwaysOn, visualize);

  // Create and populate scene
  ignition::rendering::RenderEngine *engine =
    ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  ignition::rendering::VisualPtr root = scene->RootVisual();

  scene->SetAmbientLight(0.3, 0.3, 0.3);

  // Create testing box
  // box in the center
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(1, 0, 0.5),
      ignition::math::Quaterniond::Identity);
  ignition::rendering::VisualPtr visualBox1 = scene->CreateVisual("TestBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetLocalPosition(box01Pose.Pos());
  visualBox1->SetLocalRotation(box01Pose.Rot());
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

  // Update sensor
  mgr.RunOnce(ignition::common::Time::Zero);

  // TODO(anyone): verify that the box is detected properly
  // (For cleaning reasons they don't seem to be updated properly)
  // int mid = horzSamples / 2;
  // int last = (horzSamples - 1);
  // double unitBoxSize = 1.0;
  // double expectedRangeAtMidPointBox1 =
  //   abs(box01Pose.Pos().X()) - unitBoxSize/2;
  //
  // sensor 1 should see TestBox1
  // EXPECT_DOUBLE_EQ(sensor->Range(0), ignition::math::INF_D);
  // EXPECT_NEAR(sensor->Range(mid), expectedRangeAtMidPointBox1, LASER_TOL);
  // EXPECT_DOUBLE_EQ(sensor->Range(last), ignition::math::INF_D);

  // Clean up
  engine->DestroyScene(scene);
}

/////////////////////////////////////////////////
/// \brief Test detection of different boxes
void GpuLidarSensorTest::TestThreeBoxes(const std::string &_renderEngine)
{
  // Test GPU lidar sensor with 3 boxes in the world.
  // First sensor at identity orientation, second at 90 degree roll
  // First place 2 of 3 boxes within range and verify range values.
  // then move all 3 boxes out of range and verify range values

  // Create SDF describing a gpu lidar sensors sensor
  const std::string name1 = "TestGpuLidar1";
  const std::string name2 = "TestGpuLidar2";
  const std::string topic1 = "/ignition/sensors/test/lidar1";
  const std::string topic2 = "/ignition/sensors/test/lidar2";
  const double updateRate = 30;
  const double horzSamples = 320;
  const double horzResolution = 1;
  const double horzMinAngle = -M_PI/2.0;
  const double horzMaxAngle = M_PI/2.0;
  const double vertResolution = 1;
  const double vertSamples = 1;
  const double vertMinAngle = 0;
  const double vertMaxAngle = 0;
  const double rangeResolution = 0.01;
  const double rangeMin = 0.1;
  const double rangeMax = 10.0;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d testPose1(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr lidarSDF1 = GpuLidarToSDF(name1, testPose1, updateRate,
      topic1, horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize);

  // Create a second sensor SDF rotated
  ignition::math::Pose3d testPose2(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond(M_PI/2.0, 0, 0));
  sdf::ElementPtr lidarSDF2 = GpuLidarToSDF(name2, testPose2, updateRate,
      topic2, horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize);

  // Create and populate scene
  ignition::rendering::RenderEngine *engine =
    ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  ignition::rendering::VisualPtr root = scene->RootVisual();

  scene->SetAmbientLight(0.3, 0.3, 0.3);

  // Create a sensor manager
  ignition::sensors::Manager mgr;
  mgr.SetRenderingScene(scene);
  mgr.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_DIR, "lib"));

  // Create a GpuLidarSensors
  auto sensor1 = mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(
      lidarSDF1);
  // Create second GpuLidarSensor
  auto sensor2 = mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(
      lidarSDF2);

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor1 != nullptr);
  EXPECT_TRUE(sensor2 != nullptr);

  // Create testing boxes
  // Box in the center
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(3, 0, 0.5),
                                   ignition::math::Quaterniond::Identity);
  ignition::rendering::VisualPtr visualBox1 = scene->CreateVisual("UnitBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetLocalPosition(box01Pose.Pos());
  visualBox1->SetLocalRotation(box01Pose.Rot());
  root->AddChild(visualBox1);

  // Box on the right of the first sensor
  ignition::math::Pose3d box02Pose(ignition::math::Vector3d(0, -5, 0.5),
                                   ignition::math::Quaterniond::Identity);
  ignition::rendering::VisualPtr visualBox2 = scene->CreateVisual("UnitBox2");
  visualBox2->AddGeometry(scene->CreateBox());
  visualBox2->SetLocalPosition(box02Pose.Pos());
  visualBox2->SetLocalRotation(box02Pose.Rot());
  root->AddChild(visualBox2);

  // Box on the left of the sensor 1 but out of range
  ignition::math::Pose3d box03Pose(
      ignition::math::Vector3d(0, rangeMax + 1, 0.5),
      ignition::math::Quaterniond::Identity);
  ignition::rendering::VisualPtr visualBox3 = scene->CreateVisual("UnitBox3");
  visualBox3->AddGeometry(scene->CreateBox());
  visualBox3->SetLocalPosition(box03Pose.Pos());
  visualBox3->SetLocalRotation(box03Pose.Rot());
  root->AddChild(visualBox3);

  // Update sensors
  mgr.RunOnce(ignition::common::Time::Zero);

  int mid = horzSamples / 2;
  int last = (horzSamples - 1);
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPointBox1 = abs(box01Pose.Pos().X()) - unitBoxSize/2;
  double expectedRangeAtMidPointBox2 = abs(box02Pose.Pos().Y()) - unitBoxSize/2;

  // Sensor 1 should see box01 and box02
  EXPECT_NEAR(sensor1->Range(0), expectedRangeAtMidPointBox2, LASER_TOL);
  EXPECT_NEAR(sensor1->Range(mid), expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_DOUBLE_EQ(sensor1->Range(last), ignition::math::INF_D);

  // Only box01 should be visible to sensor 2
  EXPECT_DOUBLE_EQ(sensor2->Range(0), ignition::math::INF_D);
  EXPECT_NEAR(sensor2->Range(mid), expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_DOUBLE_EQ(sensor2->Range(last), ignition::math::INF_D);

  // Move all boxes out of range
  root->RemoveChild(visualBox1);
  root->RemoveChild(visualBox2);
  visualBox1->SetLocalPosition(
      ignition::math::Vector3d(rangeMax + 1, 0, 0));
  visualBox1->SetLocalRotation(box01Pose.Rot());
  visualBox2->SetLocalPosition(
      ignition::math::Vector3d(0, -(rangeMax + 1), 0));
  visualBox2->SetLocalRotation(box02Pose.Rot());
  root->AddChild(visualBox1);
  root->AddChild(visualBox2);

  // Update sensors
  mgr.RunOnce(ignition::common::Time::Zero);

  // TODO(anyone): verify that boxes are not detected when moved
  // (For cleaning reasons they don't seem to be updated properly)
  // for (int i = 0; i < sensor1->RayCount(); ++i)
  //   EXPECT_DOUBLE_EQ(sensor1->Range(i), ignition::math::INF_D);
  //
  // for (int i = 0; i < sensor1->RayCount(); ++i)
  //   EXPECT_DOUBLE_EQ(sensor2->Range(i), ignition::math::INF_D);

  // Clean up
  engine->DestroyScene(scene);
}

/////////////////////////////////////////////////
/// \brief Test detect one box
void GpuLidarSensorTest::VerticalLidar(const std::string &_renderEngine)
{
  // Create SDF describing a camera sensor
  const std::string name = "TestGpuLidar";
  const std::string topic = "/ignition/sensors/test/lidar";
  const double updateRate = 30;
  const double horzSamples = 640;
  const double horzResolution = 1;
  const double horzMinAngle = -M_PI/2.0;
  const double horzMaxAngle = M_PI/2.0;
  const double vertResolution = 1;
  const double vertSamples = 4;
  const double vertMinAngle = -M_PI/4.0;
  const double vertMaxAngle = M_PI/4.0;
  const double rangeResolution = 0.01;
  const double rangeMin = 0.08;
  const double rangeMax = 10.0;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0.25, 0.0, 0.5),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr lidarSDF = GpuLidarToSDF(name, testPose, updateRate, topic,
    horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
    vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
    rangeResolution, rangeMin, rangeMax, alwaysOn, visualize);

  // Create and populate scene
  ignition::rendering::RenderEngine *engine =
    ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  ignition::rendering::VisualPtr root = scene->RootVisual();

  scene->SetAmbientLight(0.3, 0.3, 0.3);

  // Create testing boxes
  // box in the center
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(1, 0, 0.5),
      ignition::math::Quaterniond::Identity);
  ignition::rendering::VisualPtr visualBox1 =
    scene->CreateVisual("VerticalTestBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetLocalPosition(box01Pose.Pos());
  visualBox1->SetLocalRotation(box01Pose.Rot());
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

  // Update sensor
  mgr.RunOnce(ignition::common::Time::Zero);

  // TODO(anyone): verify that boxes are not detected when moved
  // (For cleaning reasons they don't seem to be updated properly)
  // unsigned int mid = horzSamples / 2;
  // double unitBoxSize = 1.0;
  // double expectedRangeAtMidPoint = box01Pose.Pos().X() - unitBoxSize/2
  //     - testPose.Pos().X();
  //
  // double vAngleStep = (vertMaxAngle - vertMinAngle) / (vertSamples - 1);
  // double angleStep = vertMinAngle;
  //
  // // all vertical laser planes should sense box
  // for (unsigned int i = 0; i < vertSamples; ++i)
  // {
  //   double expectedRange = expectedRangeAtMidPoint / cos(angleStep);
  //
  //   EXPECT_NEAR(sensor->Range(i * horzSamples + mid),
  //       expectedRange, VERTICAL_LASER_TOL);
  //
  //   angleStep += vAngleStep;
  //
  //   // check that the values in the extremes are infinity
  //   EXPECT_DOUBLE_EQ(sensor->Range(i * horzSamples),
  //       ignition::math::INF_D);
  //   EXPECT_DOUBLE_EQ(sensor->Range(i * horzSamples + (horzSamples - 1)),
  //       ignition::math::INF_D);
  // }

  // Move box out of range
  root->RemoveChild(visualBox1);
  visualBox1->SetLocalPosition(
      ignition::math::Vector3d(rangeMax + 1, 0, 0));
  visualBox1->SetLocalRotation(
      ignition::math::Quaterniond::Identity);
  root->AddChild(visualBox1);

  // wait for a few more laser scans
  mgr.RunOnce(ignition::common::Time::Zero);

  for (int j = 0; j < sensor->VerticalRayCount(); ++j)
  {
    for (int i = 0; i < sensor->RayCount(); ++i)
    {
      // TODO(anyone): verify that boxes are not detected when moved
      // (For cleaning reasons they don't seem to be updated properly)
      // EXPECT_DOUBLE_EQ(sensor->Range(j * sensor->RayCount() + i),
      //     ignition::math::INF_D);
    }
  }

  // Clean up
  engine->DestroyScene(scene);
}

TEST_P(GpuLidarSensorTest, CreateGpuLidar)
{
  CreateGpuLidar(GetParam());
}

TEST_P(GpuLidarSensorTest, DetectBox)
{
  DetectBox(GetParam());
}

TEST_P(GpuLidarSensorTest, TestThreeBoxes)
{
  TestThreeBoxes(GetParam());
}

TEST_P(GpuLidarSensorTest, VerticalLidar)
{
  VerticalLidar(GetParam());
}

INSTANTIATE_TEST_CASE_P(GpuLidarSensor, GpuLidarSensorTest,
    ::testing::Values("ogre"));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
