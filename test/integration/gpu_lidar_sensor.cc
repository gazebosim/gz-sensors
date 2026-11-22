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

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <gtest/gtest.h>

#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Event.hh>
#include <gz/sensors/Manager.hh>
#include <gz/sensors/Export.hh>
#include <gz/sensors/GpuLidarSensor.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Rand.hh>
#include <gz/transport/Node.hh>

// TODO(louise) Remove these pragmas once gz-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <sdf/sdf.hh>

#include "test_config.hh"  // NOLINT(build/include)
#include "TransportTestTools.hh"

#define LASER_TOL 1e-4
#define DOUBLE_TOL 1e-6

// vertical range values seem to be less accurate
#define VERTICAL_LASER_TOL 1e-3
#define WAIT_TIME 0.02

sdf::ElementPtr GpuLidarToSdf(const std::string &name,
    const gz::math::Pose3d &pose, const double updateRate,
    const std::string &topic, const double horzSamples,
    const double horzResolution, const double horzMinAngle,
    const double horzMaxAngle, const double vertSamples,
    const double vertResolution, const double vertMinAngle,
    const double vertMaxAngle, const double rangeResolution,
    const double rangeMin, const double rangeMax, const bool alwaysOn,
    const bool visualize, const std::string &frameId)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='" << name << "' type='gpu_lidar'>"
    << "      <frame_id>" << frameId << "</frame_id>"
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
std::vector<gz::msgs::LaserScan> laserMsgs;
std::vector<gz::msgs::PointCloudPacked> pointMsgs;

void OnNewLidarFrame(const float * /*_scan*/, unsigned int /*_width*/,
    unsigned int /*_height*/, unsigned int /*_channels*/,
    const std::string &/*_format*/)
{
  g_laserCounter++;
}

/////////////////////////////////////////////////
void laserCb(const gz::msgs::LaserScan &_msg)
{
  laserMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void pointCb(const gz::msgs::PointCloudPacked &_msg)
{
  pointMsgs.push_back(_msg);
}

class GpuLidarSensorTest: public testing::Test,
                  public testing::WithParamInterface<const char *>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    // Disable Ogre tests on windows. See
    // https://github.com/gazebosim/gz-sensors/issues/284
#ifdef _WIN32
    if (strcmp(GetParam(), "ogre") == 0)
    {
      GTEST_SKIP() << "Ogre tests disabled on windows. See #284.";
    }
#endif
  }

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

  // Test the published bytes against the original formulas
  public: void BitExactOutput(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
/// \brief Test Creation of a GPU Lidar sensor
void GpuLidarSensorTest::CreateGpuLidar(const std::string &_renderEngine)
{
  // Create SDF describing a camera sensor
  const std::string name = "TestGpuLidar";
  const std::string parent = "parent_link";
  const std::string topic = "/gz/sensors/test/lidar";
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
  const std::string frameId = "TestGpuLidar_frame";

  // Create sensor description in SDF
  gz::math::Pose3d testPose(gz::math::Vector3d(0, 0, 0.1),
      gz::math::Quaterniond::Identity);
  sdf::ElementPtr lidarSdf = GpuLidarToSdf(name, testPose, updateRate, topic,
    horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
    vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
    rangeResolution, rangeMin, rangeMax, alwaysOn, visualize, frameId);

  // Setup gz-rendering with an empty scene
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");
  gz::rendering::VisualPtr root = scene->RootVisual();

  // Create a sensor manager
  gz::sensors::Manager mgr;

  // Create an scene with a box in it
  scene->SetAmbientLight(0.3, 0.3, 0.3);

  // Create a GpuLidarSensor
  gz::sensors::GpuLidarSensor *sensor =
      mgr.CreateSensor<gz::sensors::GpuLidarSensor>(lidarSdf);
  sensor->SetParent(parent);
  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);
  sensor->SetScene(scene);
  EXPECT_FALSE(sensor->HasConnections());

  // Set a callback on the lidar sensor to get a scan
  gz::common::ConnectionPtr c =
    sensor->ConnectNewLidarFrame(
        std::bind(&::OnNewLidarFrame,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  EXPECT_TRUE(c != nullptr);
  EXPECT_TRUE(sensor->HasConnections());

  double angleRes = (sensor->AngleMax() - sensor->AngleMin()).Radian() /
                    sensor->RayCount();
  EXPECT_EQ(sensor->AngleMin(), gz::math::Angle(horzMinAngle));
  EXPECT_EQ(sensor->AngleMax(), gz::math::Angle(horzMaxAngle));
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
  WaitForMessageTestHelper<gz::msgs::LaserScan> helper(topic);

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
    EXPECT_DOUBLE_EQ(ranges[i], gz::math::INF_D);
    EXPECT_DOUBLE_EQ(sensor->Range(i), ranges[i]);
    EXPECT_NEAR(sensor->Retro(i), 0, 1e-6);
    EXPECT_EQ(sensor->Fiducial(i), -1);
  }

  // Clean up
  c.reset();
  mgr.Remove(sensor->Id());
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
/// \brief Test detect one box
void GpuLidarSensorTest::DetectBox(const std::string &_renderEngine)
{
  // Create SDF describing a camera sensor
  const std::string name = "TestGpuLidar";
  const std::string parent = "parent_link";
  const std::string topic = "/gz/sensors/test/lidar";
  const double updateRate = 30;
  const int horzSamples = 320;
  const double horzResolution = 1;
  const double horzMinAngle = -GZ_PI/2.0;
  const double horzMaxAngle = GZ_PI/2.0;
  const double vertResolution = 1;
  const int vertSamples = 1;
  const double vertMinAngle = 0;
  const double vertMaxAngle = 0;
  const double rangeResolution = 0.01;
  const double rangeMin = 0.08;
  const double rangeMax = 10.0;
  const bool alwaysOn = 1;
  const bool visualize = 1;
  const std::string frameId = "TestGpuLidar_frame";

  // Create sensor SDF
  gz::math::Pose3d testPose(gz::math::Vector3d(0.0, 0.0, 0.1),
      gz::math::Quaterniond::Identity);
  sdf::ElementPtr lidarSdf = GpuLidarToSdf(name, testPose, updateRate, topic,
    horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
    vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
    rangeResolution, rangeMin, rangeMax, alwaysOn, visualize, frameId);

  // Create and populate scene
  gz::rendering::RenderEngine *engine =
    gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");
  gz::rendering::VisualPtr root = scene->RootVisual();

  scene->SetAmbientLight(0.3, 0.3, 0.3);

  // Create testing box
  // box in the center
  gz::math::Pose3d box01Pose(gz::math::Vector3d(1, 0, 0.5),
      gz::math::Quaterniond::Identity);
  gz::rendering::VisualPtr visualBox1 = scene->CreateVisual("TestBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetLocalPosition(box01Pose.Pos());
  visualBox1->SetLocalRotation(box01Pose.Rot());
  visualBox1->SetUserData("laser_retro", 123);
  root->AddChild(visualBox1);

  // Create a sensor manager
  gz::sensors::Manager mgr;

  // Create a GpuLidarSensor
  gz::sensors::GpuLidarSensor *sensor =
      mgr.CreateSensor<gz::sensors::GpuLidarSensor>(lidarSdf);
  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);
  sensor->SetParent(parent);
  sensor->SetScene(scene);

  // subscribe to gpu lidar topic
  gz::transport::Node node;
  node.Subscribe(topic, &::laserCb);
  node.Subscribe(topic + "/points", &::pointCb);

  WaitForMessageTestHelper<gz::msgs::LaserScan> helper(topic);
  // Update sensor
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  int mid = horzSamples / 2;
  int last = (horzSamples - 1);
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPointBox1 =
    abs(box01Pose.Pos().X()) - unitBoxSize/2;

  // Sensor 1 should see TestBox1
  EXPECT_DOUBLE_EQ(sensor->Range(0), gz::math::INF_D);
  EXPECT_NEAR(sensor->Range(mid), expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_NEAR(123, sensor->Retro(mid), 1e-1);
  EXPECT_DOUBLE_EQ(sensor->Range(last), gz::math::INF_D);

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
  EXPECT_DOUBLE_EQ(laserMsgs.back().ranges(0), gz::math::INF_D);
  EXPECT_NEAR(laserMsgs.back().ranges(mid), expectedRangeAtMidPointBox1,
      LASER_TOL);
  EXPECT_DOUBLE_EQ(laserMsgs.back().ranges(last), gz::math::INF_D);

  EXPECT_EQ(laserMsgs.back().frame(), frameId);
  EXPECT_NEAR(laserMsgs.back().angle_min(), horzMinAngle, 1e-4);
  EXPECT_NEAR(laserMsgs.back().angle_max(), horzMaxAngle, 1e-4);
  EXPECT_NEAR(laserMsgs.back().count(), horzSamples, 1e-4);
  EXPECT_NEAR(laserMsgs.back().angle_step(), angleRes, 1e-4);
  EXPECT_NEAR(laserMsgs.back().vertical_angle_min(), vertMinAngle, 1e-4);
  EXPECT_NEAR(laserMsgs.back().vertical_angle_max(), vertMaxAngle, 1e-4);
  EXPECT_NEAR(laserMsgs.back().vertical_count(), vertSamples, 1e-4);
  EXPECT_NEAR(laserMsgs.back().range_min(), rangeMin, 1e-4);
  EXPECT_NEAR(laserMsgs.back().range_max(), rangeMax, 1e-4);

  EXPECT_TRUE(laserMsgs.back().has_header());
  EXPECT_LT(1, laserMsgs.back().header().data().size());
  EXPECT_EQ("frame_id", laserMsgs.back().header().data(0).key());
  ASSERT_EQ(1, laserMsgs.back().header().data(0).value().size());
  EXPECT_EQ(frameId, laserMsgs.back().header().data(0).value(0));

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
  EXPECT_EQ(18u, pointMsgs.back().point_step());
  EXPECT_EQ(18u * horzSamples, pointMsgs.back().row_step());
  EXPECT_FALSE(pointMsgs.back().is_dense());
  EXPECT_EQ(18u * horzSamples * vertSamples, pointMsgs.back().data().size());

  EXPECT_TRUE(pointMsgs.back().has_header());
  EXPECT_LT(1, pointMsgs.back().header().data().size());
  EXPECT_EQ("frame_id", pointMsgs.back().header().data(0).key());
  ASSERT_EQ(1, pointMsgs.back().header().data(0).value().size());
  EXPECT_EQ(frameId, pointMsgs.back().header().data(0).value(0));

  // Clean up rendering ptrs
  visualBox1.reset();

  // Clean up
  mgr.Remove(sensor->Id());
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
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
  const std::string topic1 = "/gz/sensors/test/lidar1";
  const std::string topic2 = "/gz/sensors/test/lidar2";
  const double updateRate = 30;
  const int horzSamples = 320;
  const double horzResolution = 1;
  const double horzMinAngle = -GZ_PI/2.0;
  const double horzMaxAngle = GZ_PI/2.0;
  const double vertResolution = 1;
  const int vertSamples = 1;
  const double vertMinAngle = 0;
  const double vertMaxAngle = 0;
  const double rangeResolution = 0.01;
  const double rangeMin = 0.1;
  const double rangeMax = 10.0;
  const bool alwaysOn = 1;
  const bool visualize = 1;
  const std::string frameId = "TestGpuLidar_frame";

  // Create sensor SDF
  gz::math::Pose3d testPose1(gz::math::Vector3d(0, 0, 0.1),
      gz::math::Quaterniond::Identity);
  sdf::ElementPtr lidarSdf1 = GpuLidarToSdf(name1, testPose1, updateRate,
      topic1, horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize, frameId);

  // Create a second sensor SDF rotated
  gz::math::Pose3d testPose2(gz::math::Vector3d(0, 0, 0.1),
      gz::math::Quaterniond(GZ_PI/2.0, 0, 0));
  sdf::ElementPtr lidarSdf2 = GpuLidarToSdf(name2, testPose2, updateRate,
      topic2, horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize, frameId);

  // Create and populate scene
  gz::rendering::RenderEngine *engine =
    gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");
  gz::rendering::VisualPtr root = scene->RootVisual();

  scene->SetAmbientLight(0.3, 0.3, 0.3);

  // Create a sensor manager
  gz::sensors::Manager mgr;

  // Create a GpuLidarSensors
  gz::sensors::GpuLidarSensor *sensor1 =
      mgr.CreateSensor<gz::sensors::GpuLidarSensor>(lidarSdf1);

  // Create second GpuLidarSensor
  gz::sensors::GpuLidarSensor *sensor2 =
      mgr.CreateSensor<gz::sensors::GpuLidarSensor>(lidarSdf2);

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor1);
  ASSERT_NE(nullptr, sensor2);
  sensor1->SetScene(scene);
  sensor2->SetScene(scene);

  // Create testing boxes
  // Box in the center
  gz::math::Pose3d box01Pose(gz::math::Vector3d(3, 0, 0.5),
                                   gz::math::Quaterniond::Identity);
  gz::rendering::VisualPtr visualBox1 = scene->CreateVisual("UnitBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetLocalPosition(box01Pose.Pos());
  visualBox1->SetLocalRotation(box01Pose.Rot());
  root->AddChild(visualBox1);

  // Box on the right of the first sensor
  gz::math::Pose3d box02Pose(gz::math::Vector3d(0, -5, 0.5),
                                   gz::math::Quaterniond::Identity);
  gz::rendering::VisualPtr visualBox2 = scene->CreateVisual("UnitBox2");
  visualBox2->AddGeometry(scene->CreateBox());
  visualBox2->SetLocalPosition(box02Pose.Pos());
  visualBox2->SetLocalRotation(box02Pose.Rot());
  root->AddChild(visualBox2);

  // Box on the left of the sensor 1 but out of range
  gz::math::Pose3d box03Pose(
      gz::math::Vector3d(0, rangeMax + 1, 0.5),
      gz::math::Quaterniond::Identity);
  gz::rendering::VisualPtr visualBox3 = scene->CreateVisual("UnitBox3");
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
  // gz-rendering uses lower resolution textures for lidars with low sample
  // count after: https://github.com/gazebosim/gz-rendering/pull/296
  // Side effect is the loss of precision in the depth buffer data so we relax
  // tolerance for this check in order for test to pass.
  EXPECT_NEAR(sensor1->Range(0), expectedRangeAtMidPointBox2, LASER_TOL + 1e-5);
  EXPECT_NEAR(sensor1->Range(mid), expectedRangeAtMidPointBox1, LASER_TOL);
#ifndef __APPLE__
  // See https://github.com/gazebosim/gz-sensors/issues/66
  EXPECT_DOUBLE_EQ(sensor1->Range(last), gz::math::INF_D);
#endif

  // Only box01 should be visible to sensor 2
  EXPECT_DOUBLE_EQ(sensor2->Range(0), gz::math::INF_D);
  EXPECT_NEAR(sensor2->Range(mid), expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_DOUBLE_EQ(sensor2->Range(last), gz::math::INF_D);

  // Move all boxes out of range
  gz::math::Vector3d box1PositionFar(
      rangeMax + 1, 0, 0);
  gz::math::Vector3d box2PositionFar(
      0, -(rangeMax + 1), 0);

  visualBox1->SetLocalPosition(box1PositionFar);
  visualBox2->SetLocalPosition(box2PositionFar);

  // Update sensors
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // Verify values out of range
  for (unsigned int i = 0; i < sensor1->RayCount(); ++i)
    EXPECT_DOUBLE_EQ(sensor1->Range(i), gz::math::INF_D);

  for (unsigned int i = 0; i < sensor1->RayCount(); ++i)
    EXPECT_DOUBLE_EQ(sensor2->Range(i), gz::math::INF_D);

  // Clean up rendering ptrs
  visualBox1.reset();
  visualBox2.reset();
  visualBox3.reset();

  // Clean up
  mgr.Remove(sensor1->Id());
  mgr.Remove(sensor2->Id());
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
/// \brief Test detect one box
void GpuLidarSensorTest::VerticalLidar(const std::string &_renderEngine)
{
  // Create SDF describing a camera sensor
  const std::string name = "TestGpuLidar";
  const std::string topic = "/gz/sensors/test/lidar";
  const double updateRate = 30;
  const unsigned int horzSamples = 640;
  const double horzResolution = 1;
  const double horzMinAngle = -GZ_PI/2.0;
  const double horzMaxAngle = GZ_PI/2.0;
  const double vertResolution = 1;
  const unsigned int vertSamples = 4;
  const double vertMinAngle = -GZ_PI/4.0;
  const double vertMaxAngle = GZ_PI/4.0;
  const double rangeResolution = 0.01;
  const double rangeMin = 0.08;
  const double rangeMax = 10.0;
  const bool alwaysOn = 1;
  const bool visualize = 1;
  const std::string frameId = "TestGpuLidar_frame";

  // Create sensor SDF
  gz::math::Pose3d testPose(gz::math::Vector3d(0.25, 0.0, 0.5),
      gz::math::Quaterniond::Identity);
  sdf::ElementPtr lidarSdf = GpuLidarToSdf(name, testPose, updateRate, topic,
    horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
    vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
    rangeResolution, rangeMin, rangeMax, alwaysOn, visualize, frameId);

  // Create and populate scene
  gz::rendering::RenderEngine *engine =
    gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");
  gz::rendering::VisualPtr root = scene->RootVisual();

  scene->SetAmbientLight(0.3, 0.3, 0.3);

  // Create testing boxes
  // box in the center
  gz::math::Pose3d box01Pose(gz::math::Vector3d(1, 0, 0.5),
      gz::math::Quaterniond::Identity);
  gz::rendering::VisualPtr visualBox1 =
    scene->CreateVisual("VerticalTestBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetLocalPosition(box01Pose.Pos());
  visualBox1->SetLocalRotation(box01Pose.Rot());
  root->AddChild(visualBox1);

  // Create a sensor manager
  gz::sensors::Manager mgr;

  // Create a GpuLidarSensor
  gz::sensors::GpuLidarSensor *sensor =
      mgr.CreateSensor<gz::sensors::GpuLidarSensor>(lidarSdf);

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
    // See https://github.com/gazebosim/gz-sensors/issues/66
    EXPECT_NEAR(sensor->Range(i * horzSamples + mid),
        expectedRange, VERTICAL_LASER_TOL);
#endif

    angleStep += vAngleStep;

    // check that the values in the extremes are infinity
    EXPECT_DOUBLE_EQ(sensor->Range(i * horzSamples),
        gz::math::INF_D);
    EXPECT_DOUBLE_EQ(sensor->Range(i * horzSamples + (horzSamples - 1)),
        gz::math::INF_D);
  }

  // Move box out of range
  visualBox1->SetLocalPosition(
      gz::math::Vector3d(rangeMax + 1, 0, 0));

  // Wait for a few more laser scans
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // Verify all values are out of range
  for (unsigned int j = 0; j < sensor->VerticalRayCount(); ++j)
  {
    for (unsigned int i = 0; i < sensor->RayCount(); ++i)
    {
      EXPECT_DOUBLE_EQ(sensor->Range(j * sensor->RayCount() + i),
          gz::math::INF_D);
    }
  }

  // Clean up rendering ptrs
  visualBox1.reset();

  // Clean up
  mgr.Remove(sensor->Id());
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
/// \brief Test manually updating the scene with Scene::PreRender before
/// updating the sensors
void GpuLidarSensorTest::ManualUpdate(const std::string &_renderEngine)
{
  // Create SDF describing a gpu lidar sensors sensor
  const std::string name1 = "TestGpuLidar1";
  const std::string name2 = "TestGpuLidar2";
  const std::string topic1 = "/gz/sensors/test/lidar1";
  const std::string topic2 = "/gz/sensors/test/lidar2";
  const double updateRate = 30;
  const int horzSamples = 320;
  const double horzResolution = 1;
  const double horzMinAngle = -GZ_PI/2.0;
  const double horzMaxAngle = GZ_PI/2.0;
  const double vertResolution = 1;
  const int vertSamples = 1;
  const double vertMinAngle = 0;
  const double vertMaxAngle = 0;
  const double rangeResolution = 0.01;
  const double rangeMin = 0.1;
  const double rangeMax = 10.0;
  const bool alwaysOn = 1;
  const bool visualize = 1;
  const std::string frameId = "TestGpuLidar_frame";

  // Create sensor SDF
  gz::math::Pose3d testPose1(gz::math::Vector3d(0, 0, 0.1),
      gz::math::Quaterniond::Identity);
  sdf::ElementPtr lidarSdf1 = GpuLidarToSdf(name1, testPose1, updateRate,
      topic1, horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize, frameId);

  // Create a second sensor SDF at an xy offset of 1
  gz::math::Pose3d testPose2(gz::math::Vector3d(1, 1, 0.1),
      gz::math::Quaterniond::Identity);
  sdf::ElementPtr lidarSdf2 = GpuLidarToSdf(name2, testPose2, updateRate,
      topic2, horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize, frameId);

  // Create and populate scene
  gz::rendering::RenderEngine *engine =
    gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");
  gz::rendering::VisualPtr root = scene->RootVisual();

  scene->SetAmbientLight(0.3, 0.3, 0.3);
  scene->SetCameraPassCountPerGpuFlush(6u);

  // Create a sensor manager
  gz::sensors::Manager mgr;

  // Create a GpuLidarSensors
  gz::sensors::GpuLidarSensor *sensor1 =
      mgr.CreateSensor<gz::sensors::GpuLidarSensor>(lidarSdf1);

  // Create second GpuLidarSensor
  gz::sensors::GpuLidarSensor *sensor2 =
      mgr.CreateSensor<gz::sensors::GpuLidarSensor>(lidarSdf2);

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor1);
  ASSERT_NE(nullptr, sensor2);
  sensor1->SetScene(scene);
  sensor2->SetScene(scene);

  // Create testing box
  // box in the center of lidar1 and right of lidar2
  gz::math::Pose3d box01Pose(gz::math::Vector3d(1, 0, 0.5),
      gz::math::Quaterniond::Identity);
  gz::rendering::VisualPtr visualBox1 = scene->CreateVisual("TestBox1");
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
  EXPECT_DOUBLE_EQ(sensor1->Range(0), gz::math::INF_D);
  EXPECT_NEAR(sensor1->Range(mid), expectedRangeAtMidPointBox1, LASER_TOL);
#ifndef __APPLE__
  // See https://github.com/gazebosim/gz-sensors/issues/66
  EXPECT_DOUBLE_EQ(sensor1->Range(last), gz::math::INF_D);
#endif

  // Sensor 2 should see box01 to the right of it
  EXPECT_NEAR(sensor2->Range(0), expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_DOUBLE_EQ(sensor2->Range(mid), gz::math::INF_D);
#ifndef __APPLE__
  // See https://github.com/gazebosim/gz-sensors/issues/66
  EXPECT_DOUBLE_EQ(sensor2->Range(last), gz::math::INF_D);
#endif

  // Clean up rendering ptrs
  visualBox1.reset();

  // Clean up
  mgr.Remove(sensor1->Id());
  mgr.Remove(sensor2->Id());
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
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
  const std::string frameId = "TestGpuLidar_frame";
  auto testPose = gz::math::Pose3d();

  // Scene
  auto engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
  }
  auto scene = engine->CreateScene("scene");
  EXPECT_NE(nullptr, scene);

  // Create a GpuLidarSensor
  gz::sensors::Manager mgr;

  // Default topic
  {
    const std::string topic;
    auto lidarSdf = GpuLidarToSdf(name, testPose, updateRate, topic,
      horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize, frameId);

    auto lidar = mgr.CreateSensor<gz::sensors::GpuLidarSensor>(lidarSdf);
    ASSERT_NE(nullptr, lidar);
    EXPECT_NE(gz::sensors::NO_SENSOR, lidar->Id());
    EXPECT_EQ("/lidar/points", lidar->Topic());
  }

  // Convert to valid topic
  {
    const std::string topic = "/topic with spaces/@~characters//";
    auto lidarSdf = GpuLidarToSdf(name, testPose, updateRate, topic,
      horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize, frameId);

    auto lidar = mgr.CreateSensor<gz::sensors::GpuLidarSensor>(lidarSdf);
    ASSERT_NE(nullptr, lidar);
    EXPECT_NE(gz::sensors::NO_SENSOR, lidar->Id());

    EXPECT_EQ("/topic_with_spaces/characters/points", lidar->Topic());
  }

  // Invalid topic
  {
    const std::string topic = "@@@";
    auto lidarSdf = GpuLidarToSdf(name, testPose, updateRate, topic,
      horzSamples, horzResolution, horzMinAngle, horzMaxAngle,
      vertSamples, vertResolution, vertMinAngle, vertMaxAngle,
      rangeResolution, rangeMin, rangeMax, alwaysOn, visualize, frameId);

    auto sensor = mgr.CreateSensor<gz::sensors::GpuLidarSensor>(lidarSdf);
    EXPECT_EQ(nullptr, sensor);
  }
}

/////////////////////////////////////////////////
// Reference copies of the original (gz-sensors 11.0.0~pre2) Lidar::Clamp and
// GpuLidarSensor::FillPointCloudMsg. The published messages must match what
// these produce bit for bit.
double RefClamp(double _range, double _rangeMin, double _rangeMax)
{
  if (gz::math::isnan(_range))
    return _rangeMax;

  if (std::isfinite(_range))
    return std::clamp(_range, _rangeMin, _rangeMax);

  return _range;
}

/////////////////////////////////////////////////
void RefFillPointCloudMsg(gz::msgs::PointCloudPacked &_pointMsg,
    const gz::rendering::GpuRaysPtr &_gpuRays, const float *_laserBuffer)
{
  uint32_t width = _pointMsg.width();
  uint32_t height = _pointMsg.height();
  unsigned int channels = 3;

  float angleStep =
    (_gpuRays->AngleMax() - _gpuRays->AngleMin()).Radian() /
    (_gpuRays->RangeCount()-1);

  float verticleAngleStep = (_gpuRays->VerticalAngleMax() -
      _gpuRays->VerticalAngleMin()).Radian() /
    (_gpuRays->VerticalRangeCount()-1);

  float inclination = _gpuRays->VerticalAngleMin().Radian();

  std::string *msgBuffer = _pointMsg.mutable_data();
  msgBuffer->resize(_pointMsg.row_step() * _pointMsg.height());
  char *msgBufferIndex = msgBuffer->data();
  bool isDense { true };
  for (uint32_t j = 0; j < height; ++j)
  {
    float azimuth = _gpuRays->AngleMin().Radian();

    for (uint32_t i = 0; i < width; ++i)
    {
      auto index = j * width * channels + i * channels;
      float depth = _laserBuffer[index];
      if (isDense)
        isDense = !(gz::math::isnan(depth) || std::isinf(depth));

      float intensity = _laserBuffer[index + 1];
      uint16_t ring = j;

      int fieldIndex = 0;
      const float v[4] = {
        depth * std::cos(inclination) * std::cos(azimuth),
        depth * std::cos(inclination) * std::sin(azimuth),
        depth * std::sin(inclination),
        intensity};
      for (float f : v)
      {
        memcpy(msgBufferIndex + _pointMsg.field(fieldIndex++).offset(), &f,
            sizeof(f));
      }
      memcpy(msgBufferIndex + _pointMsg.field(fieldIndex++).offset(), &ring,
          sizeof(ring));

      msgBufferIndex += _pointMsg.point_step();

      azimuth += angleStep;
    }
    inclination += verticleAngleStep;
  }
  _pointMsg.set_is_dense(isDense);
}

/////////////////////////////////////////////////
/// \brief The published LaserScan and PointCloudPacked messages must equal,
/// byte for byte, messages built independently: the metadata from the
/// configured sensor and the requested times, the payload with the reference
/// formulas above from the sensor's laser buffer, including NaN, +-inf and
/// out of range values. Three sensors: 641 x 13 rays; one vertical ray;
/// Gaussian noise with a fixed seed.
void GpuLidarSensorTest::BitExactOutput(const std::string &_renderEngine)
{
  gz::rendering::RenderEngine *engine =
    gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
  }
  gz::rendering::ScenePtr scene = engine->CreateScene("scene_bits");
  ASSERT_NE(nullptr, scene);
  gz::rendering::VisualPtr box = scene->CreateVisual("TestBox");
  box->AddGeometry(scene->CreateBox());
  box->SetLocalPosition(1, 0, 0.5);
  scene->RootVisual()->AddChild(box);

  const double rangeMin = 0.3;
  const double rangeMax = 8.0;
  const double hMin = -2.6, hMax = 2.9;
  const std::string frameId = "bits_frame";
  const gz::math::Pose3d pose(0, 0, 0.1, 0, 0, 0);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();

  struct Config
  {
    int horz;
    int vert;
    double vMin;
    double vMax;
    bool noise;
  };
  for (const Config &cfg : {Config{641, 13, -0.35, 0.3, false},
                            Config{257, 1, -0.1, 0.1, false},
                            Config{301, 7, -0.35, 0.3, true}})
  {
    SCOPED_TRACE("rays " + std::to_string(cfg.horz) + " x " +
        std::to_string(cfg.vert) + (cfg.noise ? ", noise" : ""));
    const std::string topic = "/gz/sensors/test/lidar_bits_" +
        std::to_string(cfg.horz) + "_" + std::to_string(cfg.vert);
    sdf::ElementPtr lidarSdf = GpuLidarToSdf("TestGpuLidarBits", pose, 10,
        topic, cfg.horz, 1, hMin, hMax, cfg.vert, 1, cfg.vMin, cfg.vMax,
        0.01, rangeMin, rangeMax, true, false, frameId);
    ASSERT_NE(nullptr, lidarSdf);
    if (cfg.noise)
    {
      sdf::ElementPtr noise = lidarSdf->GetElement("ray")->GetElement("noise");
      noise->GetElement("type")->Set<std::string>("gaussian");
      noise->GetElement("mean")->Set<double>(0.01);
      noise->GetElement("stddev")->Set<double>(0.2);
      gz::math::Rand::Seed(7);
    }

    gz::sensors::Manager mgr;
    gz::sensors::GpuLidarSensor *sensor =
        mgr.CreateSensor<gz::sensors::GpuLidarSensor>(lidarSdf);
    ASSERT_NE(nullptr, sensor);
    sensor->SetScene(scene);

    // Overwrite the laser buffer the sensor publishes from: frame 0 gets
    // finite ranges first, then NaN, +-inf, zero, values on and beyond both
    // clamps and NaN intensities between the rendered ones; frame 1 finite
    // ranges only (a dense cloud without noise).
    int frame = 0;
    std::vector<float> injected;
    auto conn = sensor->ConnectNewLidarFrame(
        [&](const float *, unsigned int _w, unsigned int _h, unsigned int _c,
            const std::string &)
        {
          float *buf = sensor->laserBuffer;
          for (unsigned int k = 0; k < _w * _h; ++k)
          {
            float &range = buf[k * _c];
            float &retro = buf[k * _c + 1];
            if (frame == 0)
            {
              const float special[] = {nan, inf, -inf, 0.0f, -2.5f,
                  std::nextafter(0.3f, 0.0f), 0.3f, 8.0f,
                  std::nextafter(8.0f, 9.0f), 1e30f, 1e-30f};
              // Even rays: the special values, the first invalid one at
              // ray 16; odd rays finite.
              range = k % 2 == 0 ?
                  special[(k / 2 + 3) % std::size(special)] : 1.5f;
              if (k % 7 == 0)
                retro = nan;
            }
            else
            {
              range = 0.35f + 0.0073f * static_cast<float>(
                  (k * 2654435761u) % 1009u);
              retro = static_cast<float>(k % 251u) * 0.5f;
            }
          }
          // Before noise: the noise must change most in-range values.
          injected.assign(buf, buf + _w * _h * _c);
        });

    // Owned with the subscriber callbacks, which may still run after a
    // failed assertion has left this scope.
    struct Received
    {
      std::mutex mutex;
      std::vector<std::string> scans;
      std::vector<std::string> clouds;
    };
    auto received = std::make_shared<Received>();
    std::mutex &mutex = received->mutex;
    std::vector<std::string> &scans = received->scans;
    std::vector<std::string> &clouds = received->clouds;
    gz::transport::Node node;
    ASSERT_TRUE(node.SubscribeRaw(topic,
        [received](const char *_data, const size_t _size,
            const gz::transport::MessageInfo &)
        {
          std::lock_guard<std::mutex> lock(received->mutex);
          received->scans.emplace_back(_data, _size);
        }, "gz.msgs.LaserScan"));
    ASSERT_TRUE(node.SubscribeRaw(topic + "/points",
        [received](const char *_data, const size_t _size,
            const gz::transport::MessageInfo &)
        {
          std::lock_guard<std::mutex> lock(received->mutex);
          received->clouds.emplace_back(_data, _size);
        }, "gz.msgs.PointCloudPacked"));

    const int numRays = cfg.horz * cfg.vert;
    for (frame = 0; frame < 2; ++frame)
    {
      SCOPED_TRACE("frame " + std::to_string(frame));
      const int64_t sec = frame + 1;
      mgr.RunOnce(std::chrono::seconds(sec), true);
      for (int i = 0; i < 300; ++i)
      {
        {
          std::lock_guard<std::mutex> lock(mutex);
          if (static_cast<int>(scans.size()) > frame &&
              static_cast<int>(clouds.size()) > frame)
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      std::lock_guard<std::mutex> lock(mutex);
      ASSERT_EQ(frame + 1, static_cast<int>(scans.size()));
      ASSERT_EQ(frame + 1, static_cast<int>(clouds.size()));
      const float *buf = sensor->laserBuffer;
      // Scan and cloud share the sensor's sequence: scan first.
      const std::string scanSeq = std::to_string(2 * frame);
      const std::string cloudSeq = std::to_string(2 * frame + 1);

      // LaserScan, as gz-sensors 10 fills it.
      gz::msgs::LaserScan scan;
      scan.mutable_header()->mutable_stamp()->set_sec(sec);
      auto *data = scan.mutable_header()->add_data();
      data->set_key("frame_id");
      data->add_value(frameId);
      data = scan.mutable_header()->add_data();
      data->set_key("seq");
      data->add_value(scanSeq);
      scan.set_frame(frameId);
      gz::msgs::Set(scan.mutable_world_pose(), pose);
      scan.set_angle_min(hMin);
      scan.set_angle_max(hMax);
      scan.set_angle_step((hMax - hMin) / (cfg.horz - 1));
      scan.set_range_min(rangeMin);
      scan.set_range_max(rangeMax);
      scan.set_count(cfg.horz);
      scan.set_vertical_angle_min(cfg.vMin);
      scan.set_vertical_angle_max(cfg.vMax);
      scan.set_vertical_angle_step((cfg.vMax - cfg.vMin) / (cfg.vert - 1));
      scan.set_vertical_count(cfg.vert);
      for (int k = 0; k < numRays; ++k)
      {
        scan.add_ranges(RefClamp(buf[k * 3], rangeMin, rangeMax));
        scan.add_intensities(buf[k * 3 + 1]);
      }
      gz::msgs::LaserScan got;
      ASSERT_TRUE(got.ParseFromString(scans.back()));
      got.clear_ranges();
      got.clear_intensities();
      gz::msgs::LaserScan meta = scan;
      meta.clear_ranges();
      meta.clear_intensities();
      EXPECT_EQ(meta.DebugString(), got.DebugString());
      EXPECT_TRUE(scan.SerializeAsString() == scans.back());

      // PointCloudPacked, as gz-sensors 10 fills it.
      gz::msgs::PointCloudPacked cloud;
      gz::msgs::InitPointCloudPacked(cloud, frameId, false,
          {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32},
          {"intensity", gz::msgs::PointCloudPacked::Field::FLOAT32},
          {"ring", gz::msgs::PointCloudPacked::Field::UINT16}});
      cloud.set_width(cfg.horz);
      cloud.set_height(cfg.vert);
      cloud.set_row_step(cloud.point_step() * cfg.horz);
      cloud.mutable_header()->mutable_stamp()->set_sec(sec);
      data = cloud.mutable_header()->add_data();
      data->set_key("seq");
      data->add_value(cloudSeq);
      RefFillPointCloudMsg(cloud, sensor->GpuRays(), buf);
      if (!cfg.noise)
      {
        EXPECT_EQ(frame == 1, cloud.is_dense());
        // Ray 1 is finite, ray 16 the first invalid one.
        if (frame == 0)
        {
          EXPECT_TRUE(std::isfinite(buf[1 * 3]));
          EXPECT_TRUE(std::isnan(buf[16 * 3]));
        }
      }
      else
      {
        // The published ranges are the noisy ones.
        int inRange = 0, changed = 0;
        for (int k = 0; k < numRays; ++k)
        {
          const float before = injected[k * 3];
          if (before > rangeMin + 1 && before < rangeMax - 1)
          {
            ++inRange;
            const double noisy = scan.ranges(k);
            const double clean = RefClamp(before, rangeMin, rangeMax);
            changed += memcmp(&noisy, &clean, sizeof(double)) != 0;
          }
        }
        EXPECT_GT(inRange, numRays / 4);
        EXPECT_GT(changed, inRange * 9 / 10);
      }

      gz::msgs::PointCloudPacked gotCloud;
      ASSERT_TRUE(gotCloud.ParseFromString(clouds.back()));
      ASSERT_EQ(cloud.data().size(), gotCloud.data().size());
      int pointDiffs = 0;
      for (size_t p = 0; p < cloud.data().size(); p += cloud.point_step())
      {
        pointDiffs += memcmp(gotCloud.data().data() + p,
            cloud.data().data() + p, cloud.point_step()) != 0;
      }
      EXPECT_EQ(0, pointDiffs);
      gotCloud.clear_data();
      gz::msgs::PointCloudPacked cloudMeta = cloud;
      cloudMeta.clear_data();
      EXPECT_EQ(cloudMeta.DebugString(), gotCloud.DebugString());
      EXPECT_TRUE(cloud.SerializeAsString() == clouds.back());
    }

    conn.reset();
    mgr.Remove(sensor->Id());
  }

  box.reset();
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
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
  gz::common::Console::SetVerbosity(4);
  Topic(GetParam());
}

/////////////////////////////////////////////////
#ifdef __APPLE__
TEST_P(GpuLidarSensorTest, DISABLED_BitExactOutput)
#else
TEST_P(GpuLidarSensorTest, BitExactOutput)
#endif
{
  BitExactOutput(GetParam());
}

INSTANTIATE_TEST_SUITE_P(GpuLidarSensor, GpuLidarSensorTest,
    RENDER_ENGINE_VALUES, gz::rendering::PrintToStringParam());
