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

// TODO(louise) Remove these pragmas once ign-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <sdf/sdf.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "TransportTestTools.hh"

#define LASER_TOL 1e-4
#define DOUBLE_TOL 1e-6

// vertical range values seem to be less accurate
#define VERTICAL_LASER_TOL 1e-3
#define WAIT_TIME 0.02

sdf::ElementPtr GpuLidarToSdf(const std::string &name,
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

int g_laserCounter = 0;
std::vector<ignition::msgs::LaserScan> laserMsgs;
std::vector<ignition::msgs::PointCloudPacked> pointMsgs;

void OnNewLidarFrame(const float * /*_scan*/, unsigned int /*_width*/,
    unsigned int /*_height*/, unsigned int /*_channels*/,
    const std::string &/*_format*/)
{
  g_laserCounter++;
}

/////////////////////////////////////////////////
void laserCb(const ignition::msgs::LaserScan &_msg)
{
  laserMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void pointCb(const ignition::msgs::PointCloudPacked &_msg)
{
  pointMsgs.push_back(_msg);
}

class GpuLidarSensorTest: public testing::Test,
                  public testing::WithParamInterface<const char *>
{
  // Test and verify gpu rays properties setters and getters
  public: void CreateGpuLidar(const std::string &_renderEngine);

  // Test single box detection
  public: void DetectBox(const std::string &_renderEngine);

  // Test detection of three boxes placed at different locations
  public: void TestThreeBoxes(const std::string &_renderEngine);

  // Test vertical measurements
  public: void VerticalLidar(const std::string &_renderEngine);

  // Test manually updating sensors
  public: void ManualUpdate(const std::string &_renderEngine);

  // Test topics
  public: void Topic(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
/// \brief Test Creation of a GPU Lidar sensor
void GpuLidarSensorTest::CreateGpuLidar(const std::string &_renderEngine)
{
  // Create SDF describing a camera sensor
  const std::string name = "TestGpuLidar";
  const std::string parent = "parent_link";
  const std::string topic = "/ignition/sensors/test/lidar";
  const double updateRate = 30;
  const unsigned int horzSamples = 640;
  const double horzResolution = 1;
  const double horzMinAngle = -1.396263;
  const double horzMaxAngle = 1.396263;
  const double vertResolution = 1;
  const unsigned int vertSamples = 1;
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
  sdf::ElementPtr lidarSdf = GpuLidarToSdf(name, testPose, updateRate, topic,
    horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
    vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
    rangeResolution, rangeMin, rangeMax, alwaysOn, visualize);

  // Setup ign-rendering with an empty scene
  auto *engine = ignition::rendering::engine(_renderEngine);
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

  // Create an scene with a box in it
  scene->SetAmbientLight(0.3, 0.3, 0.3);

  // Create a GpuLidarSensor
  ignition::sensors::GpuLidarSensor *sensor =
      mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(lidarSdf);
  sensor->SetParent(parent);
  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);
  sensor->SetScene(scene);
  EXPECT_FALSE(sensor->HasConnections());

  // Set a callback on the lidar sensor to get a scan
  ignition::common::ConnectionPtr c =
    sensor->ConnectNewLidarFrame(
        std::bind(&::OnNewLidarFrame,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  EXPECT_TRUE(c != nullptr);
  EXPECT_TRUE(sensor->HasConnections());

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

  EXPECT_EQ(sensor->Parent(), parent);
  EXPECT_EQ(sensor->VerticalRayCount(), vertSamples);
  EXPECT_EQ(sensor->VerticalRangeCount(), vertSamples);
  EXPECT_EQ(sensor->VerticalAngleMin(), vertMinAngle);
  EXPECT_EQ(sensor->VerticalAngleMax(), vertMaxAngle);

  EXPECT_TRUE(sensor->IsActive());

  g_laserCounter = 0;
  WaitForMessageTestHelper<ignition::msgs::LaserScan> helper(topic);

  // Update once to verify that a message is sent
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

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
  ignition::rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
/// \brief Test detect one box
void GpuLidarSensorTest::DetectBox(const std::string &_renderEngine)
{
  // Create SDF describing a camera sensor
  const std::string name = "TestGpuLidar";
  const std::string parent = "parent_link";
  const std::string topic = "/ignition/sensors/test/lidar";
  const double updateRate = 30;
  const int horzSamples = 320;
  const double horzResolution = 1;
  const double horzMinAngle = -IGN_PI/2.0;
  const double horzMaxAngle = IGN_PI/2.0;
  const double vertResolution = 1;
  const int vertSamples = 1;
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
  sdf::ElementPtr lidarSdf = GpuLidarToSdf(name, testPose, updateRate, topic,
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

  // Create a GpuLidarSensor
  ignition::sensors::GpuLidarSensor *sensor =
      mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(lidarSdf);
  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);
  sensor->SetParent(parent);
  sensor->SetScene(scene);

  // subscribe to gpu lidar topic
  ignition::transport::Node node;
  node.Subscribe(topic, &::laserCb);
  node.Subscribe(topic + "/points", &::pointCb);

  WaitForMessageTestHelper<ignition::msgs::LaserScan> helper(topic);
  // Update sensor
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());
  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  int mid = horzSamples / 2;
  int last = (horzSamples - 1);
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPointBox1 =
    abs(box01Pose.Pos().X()) - unitBoxSize/2;

  // Sensor 1 should see TestBox1
  EXPECT_DOUBLE_EQ(sensor->Range(0), ignition::math::INF_D);
  EXPECT_NEAR(sensor->Range(mid), expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_DOUBLE_EQ(sensor->Range(last), ignition::math::INF_D);

  // Make sure to wait to receive the message
  auto waitTime = std::chrono::duration_cast< std::chrono::milliseconds >(
      std::chrono::duration< double >(0.01));
  int i = 0;
  while ((laserMsgs.empty() || pointMsgs.empty()) && i < 300)
  {
    std::this_thread::sleep_for(waitTime);
    i++;
  }
  EXPECT_LT(i, 300);

  double angleRes = (sensor->AngleMax() - sensor->AngleMin()).Radian() /
                    sensor->RayCount();

  // Check we have the same values than using the sensors methods
  EXPECT_DOUBLE_EQ(laserMsgs.back().ranges(0), ignition::math::INF_D);
  EXPECT_NEAR(laserMsgs.back().ranges(mid), expectedRangeAtMidPointBox1,
      LASER_TOL);
  EXPECT_DOUBLE_EQ(laserMsgs.back().ranges(last), ignition::math::INF_D);

  EXPECT_EQ(laserMsgs.back().frame(), name);
  EXPECT_NEAR(laserMsgs.back().angle_min(), horzMinAngle, 1e-4);
  EXPECT_NEAR(laserMsgs.back().angle_max(), horzMaxAngle, 1e-4);
  EXPECT_NEAR(laserMsgs.back().count(), horzSamples, 1e-4);
  EXPECT_NEAR(laserMsgs.back().angle_step(), angleRes, 1e-4);
  EXPECT_NEAR(laserMsgs.back().vertical_angle_min(), vertMinAngle, 1e-4);
  EXPECT_NEAR(laserMsgs.back().vertical_angle_max(), vertMaxAngle, 1e-4);
  EXPECT_NEAR(laserMsgs.back().vertical_count(), vertSamples, 1e-4);
  EXPECT_NEAR(laserMsgs.back().range_min(), rangeMin, 1e-4);
  EXPECT_NEAR(laserMsgs.back().range_max(), rangeMax, 1e-4);

  ASSERT_TRUE(!pointMsgs.empty());
  EXPECT_EQ(5, pointMsgs.back().field_size());
  EXPECT_EQ("x", pointMsgs.back().field(0).name());
  EXPECT_EQ("y", pointMsgs.back().field(1).name());
  EXPECT_EQ("z", pointMsgs.back().field(2).name());
  EXPECT_EQ("intensity", pointMsgs.back().field(3).name());
  EXPECT_EQ("ring", pointMsgs.back().field(4).name());
  EXPECT_EQ(static_cast<uint32_t>(vertSamples), pointMsgs.back().height());
  EXPECT_EQ(static_cast<uint32_t>(horzSamples), pointMsgs.back().width());
  EXPECT_FALSE(pointMsgs.back().is_bigendian());
  EXPECT_EQ(32u, pointMsgs.back().point_step());
  EXPECT_EQ(32u * horzSamples, pointMsgs.back().row_step());
  EXPECT_FALSE(pointMsgs.back().is_dense());
  EXPECT_EQ(32u * horzSamples * vertSamples, pointMsgs.back().data().size());

  // Clean up
  //
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
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
  const int horzSamples = 320;
  const double horzResolution = 1;
  const double horzMinAngle = -IGN_PI/2.0;
  const double horzMaxAngle = IGN_PI/2.0;
  const double vertResolution = 1;
  const int vertSamples = 1;
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
  sdf::ElementPtr lidarSdf1 = GpuLidarToSdf(name1, testPose1, updateRate,
      topic1, horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize);

  // Create a second sensor SDF rotated
  ignition::math::Pose3d testPose2(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond(IGN_PI/2.0, 0, 0));
  sdf::ElementPtr lidarSdf2 = GpuLidarToSdf(name2, testPose2, updateRate,
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

  // Create a GpuLidarSensors
  ignition::sensors::GpuLidarSensor *sensor1 =
      mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(lidarSdf1);

  // Create second GpuLidarSensor
  ignition::sensors::GpuLidarSensor *sensor2 =
      mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(lidarSdf2);

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor1);
  ASSERT_NE(nullptr, sensor2);
  sensor1->SetScene(scene);
  sensor2->SetScene(scene);

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
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  int mid = horzSamples / 2;
  int last = (horzSamples - 1);
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPointBox1 = abs(box01Pose.Pos().X()) - unitBoxSize/2;
  double expectedRangeAtMidPointBox2 = abs(box02Pose.Pos().Y()) - unitBoxSize/2;

  // Sensor 1 should see box01 and box02
  // ign-rendering uses lower resolution textures for lidars with low sample
  // count after: https://github.com/ignitionrobotics/ign-rendering/pull/296
  // Side effect is the loss of precision in the depth buffer data so we relax
  // tolerance for this check in order for test to pass.
  EXPECT_NEAR(sensor1->Range(0), expectedRangeAtMidPointBox2, LASER_TOL + 1e-5);
  EXPECT_NEAR(sensor1->Range(mid), expectedRangeAtMidPointBox1, LASER_TOL);
#ifndef __APPLE__
  // See https://github.com/ignitionrobotics/ign-sensors/issues/66
  EXPECT_DOUBLE_EQ(sensor1->Range(last), ignition::math::INF_D);
#endif

  // Only box01 should be visible to sensor 2
  EXPECT_DOUBLE_EQ(sensor2->Range(0), ignition::math::INF_D);
  EXPECT_NEAR(sensor2->Range(mid), expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_DOUBLE_EQ(sensor2->Range(last), ignition::math::INF_D);

  // Move all boxes out of range
  ignition::math::Vector3d box1PositionFar(
      rangeMax + 1, 0, 0);
  ignition::math::Vector3d box2PositionFar(
      0, -(rangeMax + 1), 0);

  visualBox1->SetLocalPosition(box1PositionFar);
  visualBox2->SetLocalPosition(box2PositionFar);

  // Update sensors
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // Verify values out of range
  for (unsigned int i = 0; i < sensor1->RayCount(); ++i)
    EXPECT_DOUBLE_EQ(sensor1->Range(i), ignition::math::INF_D);

  for (unsigned int i = 0; i < sensor1->RayCount(); ++i)
    EXPECT_DOUBLE_EQ(sensor2->Range(i), ignition::math::INF_D);

  // Clean up
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
/// \brief Test detect one box
void GpuLidarSensorTest::VerticalLidar(const std::string &_renderEngine)
{
  // Create SDF describing a camera sensor
  const std::string name = "TestGpuLidar";
  const std::string topic = "/ignition/sensors/test/lidar";
  const double updateRate = 30;
  const unsigned int horzSamples = 640;
  const double horzResolution = 1;
  const double horzMinAngle = -IGN_PI/2.0;
  const double horzMaxAngle = IGN_PI/2.0;
  const double vertResolution = 1;
  const unsigned int vertSamples = 4;
  const double vertMinAngle = -IGN_PI/4.0;
  const double vertMaxAngle = IGN_PI/4.0;
  const double rangeResolution = 0.01;
  const double rangeMin = 0.08;
  const double rangeMax = 10.0;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0.25, 0.0, 0.5),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr lidarSdf = GpuLidarToSdf(name, testPose, updateRate, topic,
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

  // Create a GpuLidarSensor
  ignition::sensors::GpuLidarSensor *sensor =
      mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(lidarSdf);

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);
  sensor->SetScene(scene);

  // Update sensor
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  unsigned int mid = horzSamples / 2;
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPoint = box01Pose.Pos().X() - unitBoxSize/2
      - testPose.Pos().X();

  double vAngleStep = (vertMaxAngle - vertMinAngle) / (vertSamples - 1);
  double angleStep = vertMinAngle;

  // all vertical laser planes should sense box
  for (unsigned int i = 0; i < vertSamples; ++i)
  {
    double expectedRange = expectedRangeAtMidPoint / cos(angleStep);

#ifndef __APPLE__
    // See https://github.com/ignitionrobotics/ign-sensors/issues/66
    EXPECT_NEAR(sensor->Range(i * horzSamples + mid),
        expectedRange, VERTICAL_LASER_TOL);
#endif

    angleStep += vAngleStep;

    // check that the values in the extremes are infinity
    EXPECT_DOUBLE_EQ(sensor->Range(i * horzSamples),
        ignition::math::INF_D);
    EXPECT_DOUBLE_EQ(sensor->Range(i * horzSamples + (horzSamples - 1)),
        ignition::math::INF_D);
  }

  // Move box out of range
  visualBox1->SetLocalPosition(
      ignition::math::Vector3d(rangeMax + 1, 0, 0));

  // Wait for a few more laser scans
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // Verify all values are out of range
  for (unsigned int j = 0; j < sensor->VerticalRayCount(); ++j)
  {
    for (unsigned int i = 0; i < sensor->RayCount(); ++i)
    {
      EXPECT_DOUBLE_EQ(sensor->Range(j * sensor->RayCount() + i),
          ignition::math::INF_D);
    }
  }

  // Clean up
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
/// \brief Test manually updating the scene with Scene::PreRender before
/// updating the sensors
void GpuLidarSensorTest::ManualUpdate(const std::string &_renderEngine)
{
  // Create SDF describing a gpu lidar sensors sensor
  const std::string name1 = "TestGpuLidar1";
  const std::string name2 = "TestGpuLidar2";
  const std::string topic1 = "/ignition/sensors/test/lidar1";
  const std::string topic2 = "/ignition/sensors/test/lidar2";
  const double updateRate = 30;
  const int horzSamples = 320;
  const double horzResolution = 1;
  const double horzMinAngle = -IGN_PI/2.0;
  const double horzMaxAngle = IGN_PI/2.0;
  const double vertResolution = 1;
  const int vertSamples = 1;
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
  sdf::ElementPtr lidarSdf1 = GpuLidarToSdf(name1, testPose1, updateRate,
      topic1, horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize);

  // Create a second sensor SDF at an xy offset of 1
  ignition::math::Pose3d testPose2(ignition::math::Vector3d(1, 1, 0.1),
      ignition::math::Quaterniond::Identity);
  sdf::ElementPtr lidarSdf2 = GpuLidarToSdf(name2, testPose2, updateRate,
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
  scene->SetCameraPassCountPerGpuFlush(6u);

  // Create a sensor manager
  ignition::sensors::Manager mgr;

  // Create a GpuLidarSensors
  ignition::sensors::GpuLidarSensor *sensor1 =
      mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(lidarSdf1);

  // Create second GpuLidarSensor
  ignition::sensors::GpuLidarSensor *sensor2 =
      mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(lidarSdf2);

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor1);
  ASSERT_NE(nullptr, sensor2);
  sensor1->SetScene(scene);
  sensor2->SetScene(scene);

  // Create testing box
  // box in the center of lidar1 and right of lidar2
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(1, 0, 0.5),
      ignition::math::Quaterniond::Identity);
  ignition::rendering::VisualPtr visualBox1 = scene->CreateVisual("TestBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetLocalPosition(box01Pose.Pos());
  visualBox1->SetLocalRotation(box01Pose.Rot());
  root->AddChild(visualBox1);

  // Set sensors to manual update mode
  sensor1->SetManualSceneUpdate(true);
  EXPECT_TRUE(sensor1->ManualSceneUpdate());
  sensor2->SetManualSceneUpdate(true);
  EXPECT_TRUE(sensor2->ManualSceneUpdate());

  // manually update scene
  scene->PreRender();

  // Render and update
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  // manually finish update scene
  scene->PostRender();

  int mid = horzSamples / 2;
  int last = (horzSamples - 1);
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPointBox1 =
    abs(box01Pose.Pos().X()) - unitBoxSize/2;

  // Sensor 1 should see box01 in front of it
  EXPECT_DOUBLE_EQ(sensor1->Range(0), ignition::math::INF_D);
  EXPECT_NEAR(sensor1->Range(mid), expectedRangeAtMidPointBox1, LASER_TOL);
#ifndef __APPLE__
  // See https://github.com/ignitionrobotics/ign-sensors/issues/66
  EXPECT_DOUBLE_EQ(sensor1->Range(last), ignition::math::INF_D);
#endif

  // Sensor 2 should see box01 to the right of it
  EXPECT_NEAR(sensor2->Range(0), expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_DOUBLE_EQ(sensor2->Range(mid), ignition::math::INF_D);
#ifndef __APPLE__
  // See https://github.com/ignitionrobotics/ign-sensors/issues/66
  EXPECT_DOUBLE_EQ(sensor2->Range(last), ignition::math::INF_D);
#endif

  // Clean up
  //
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void GpuLidarSensorTest::Topic(const std::string &_renderEngine)
{
  const std::string name = "TestGpuLidar";
  const double updateRate = 30;
  const unsigned int horzSamples = 640;
  const double horzResolution = 1;
  const double horzMinAngle = -1.396263;
  const double horzMaxAngle = 1.396263;
  const double vertResolution = 1;
  const unsigned int vertSamples = 1;
  const double vertMinAngle = 0;
  const double vertMaxAngle = 0;
  const double rangeResolution = 0.01;
  const double rangeMin = 0.08;
  const double rangeMax = 10.0;
  const bool alwaysOn = 1;
  const bool visualize = 1;
  auto testPose = ignition::math::Pose3d();

  // Scene
  auto engine = ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }
  auto scene = engine->CreateScene("scene");
  EXPECT_NE(nullptr, scene);

  // Create a GpuLidarSensor
  ignition::sensors::Manager mgr;


  // Default topic
  {
    const std::string topic;
    auto lidarSdf = GpuLidarToSdf(name, testPose, updateRate, topic,
      horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize);

    auto lidar = mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(lidarSdf);
    ASSERT_NE(nullptr, lidar);
    EXPECT_NE(ignition::sensors::NO_SENSOR, lidar->Id());
    EXPECT_EQ("/lidar/points", lidar->Topic());
  }

  // Convert to valid topic
  {
    const std::string topic = "/topic with spaces/@~characters//";
    auto lidarSdf = GpuLidarToSdf(name, testPose, updateRate, topic,
      horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize);

    auto lidar = mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(lidarSdf);
    ASSERT_NE(nullptr, lidar);
    EXPECT_NE(ignition::sensors::NO_SENSOR, lidar->Id());

    EXPECT_EQ("/topic_with_spaces/characters/points", lidar->Topic());
  }

  // Invalid topic
  {
    const std::string topic = "@@@";
    auto lidarSdf = GpuLidarToSdf(name, testPose, updateRate, topic,
      horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize);

    auto sensor = mgr.CreateSensor<ignition::sensors::GpuLidarSensor>(lidarSdf);
    EXPECT_EQ(nullptr, sensor);
  }
}

/////////////////////////////////////////////////
#ifdef __APPLE__
TEST_P(GpuLidarSensorTest, DISABLED_CreateGpuLidar)
#else
TEST_P(GpuLidarSensorTest, CreateGpuLidar)
#endif
{
  CreateGpuLidar(GetParam());
}

/////////////////////////////////////////////////
#ifdef __APPLE__
TEST_P(GpuLidarSensorTest, DISABLED_DetectBox)
#else
TEST_P(GpuLidarSensorTest, DetectBox)
#endif
{
  DetectBox(GetParam());
}

/////////////////////////////////////////////////
#ifdef __APPLE__
TEST_P(GpuLidarSensorTest, DISABLED_TestThreeBoxes)
#else
TEST_P(GpuLidarSensorTest, TestThreeBoxes)
#endif
{
  TestThreeBoxes(GetParam());
}

/////////////////////////////////////////////////
#ifdef __APPLE__
TEST_P(GpuLidarSensorTest, DISABLED_VerticalLidar)
#else
TEST_P(GpuLidarSensorTest, VerticalLidar)
#endif
{
  VerticalLidar(GetParam());
}

/////////////////////////////////////////////////
#ifdef __APPLE__
TEST_P(GpuLidarSensorTest, DISABLED_ManualUpdate)
#else
TEST_P(GpuLidarSensorTest, ManualUpdate)
#endif
{
  ManualUpdate(GetParam());
}

/////////////////////////////////////////////////
TEST_P(GpuLidarSensorTest, Topic)
{
  Topic(GetParam());
}

INSTANTIATE_TEST_CASE_P(GpuLidarSensor, GpuLidarSensorTest,
    RENDER_ENGINE_VALUES, ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ignition::common::Console::SetVerbosity(4);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
