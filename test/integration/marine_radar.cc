/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <cmath>
#include <cstring>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include <sdf/Element.hh>
#include <sdf/parser.hh>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/sensors/Manager.hh>
#include <gz/sensors/MarineRadarSensor.hh>

#include "test_config.hh"  // NOLINT(build/include)
#include "TransportTestTools.hh"

/// \brief Build a marine radar SDF element. The radar is deliberately coarse
/// and fast (64 spokes at 600 rpm) so a whole revolution fits into a short
/// test, and every random imperfection is switched off so the expectations
/// below are deterministic.
sdf::ElementPtr MarineRadarToSdf(const std::string &_name,
    const gz::math::Pose3d &_pose, const std::string &_topic,
    double _rangeMax)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.9'>"
    << " <model name='usv'>"
    << "  <link name='radar_link'>"
    << "    <sensor name='" << _name
    << "' type='custom' gz:type='marine_radar'>"
    << "      <pose>" << _pose << "</pose>"
    << "      <topic>" << _topic << "</topic>"
    << "      <update_rate>0</update_rate>"
    << "      <always_on>1</always_on>"
    << "      <gz:marine_radar>"
    << "        <range_min>10</range_min>"
    << "        <range_max>" << _rangeMax << "</range_max>"
    << "        <range_resolution>5</range_resolution>"
    << "        <samples>64</samples>"
    << "        <rotation_rate_rpm>600</rotation_rate_rpm>"
    << "        <beamwidth_horizontal_deg>6</beamwidth_horizontal_deg>"
    << "        <beamwidth_vertical_deg>20</beamwidth_vertical_deg>"
    << "        <slice_heights>1 3 6</slice_heights>"
    << "        <intensity_noise_stddev_db>0</intensity_noise_stddev_db>"
    << "        <range_noise_stddev>0</range_noise_stddev>"
    << "        <bearing_noise_stddev_deg>0</bearing_noise_stddev_deg>"
    << "        <echo_detection_probability>1</echo_detection_probability>"
    << "        <false_alarm_rate>0</false_alarm_rate>"
    << "        <min_target_cells>1</min_target_cells>"
    << "        <image_size>64</image_size>"
    << "        <image_rate>50</image_rate>"
    << "      </gz:marine_radar>"
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

/// \brief Read one named FLOAT32 field of the first point of a packed cloud.
bool FirstPointField(const gz::msgs::PointCloudPacked &_msg,
    const std::string &_name, float &_value)
{
  for (const auto &field : _msg.field())
  {
    if (field.name() != _name)
      continue;
    if (_msg.data().size() < field.offset() + sizeof(float))
      return false;
    std::memcpy(&_value, _msg.data().data() + field.offset(), sizeof(float));
    return true;
  }
  return false;
}

class MarineRadarSensorTest : public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);
  }
};

/////////////////////////////////////////////////
/// \brief An empty sea must produce scans with no targets at all.
void EmptyScene(const std::string &_renderEngine)
{
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
    GTEST_SKIP() << "Engine '" << _renderEngine << "' is not supported";

  auto scene = engine->CreateScene("marine_radar_empty");
  ASSERT_NE(nullptr, scene);

  auto radarSdf = MarineRadarToSdf("radar", gz::math::Pose3d::Zero,
      "/marine_radar_empty", 500);
  ASSERT_NE(nullptr, radarSdf);

  gz::sensors::Manager mgr;
  auto *sensor = mgr.CreateSensor<gz::sensors::MarineRadarSensor>(radarSdf);
  ASSERT_NE(nullptr, sensor);
  sensor->SetScene(scene);

  using namespace std::chrono_literals;
  auto simTime = std::chrono::steady_clock::duration::zero();
  for (int i = 0; i < 210; ++i)
  {
    sensor->SetPose(gz::math::Pose3d(gz::math::Vector3d(0, 0, 5),
        gz::math::Quaterniond::Identity));
    mgr.RunOnce(simTime, true);
    simTime += 1ms;
  }

  // Rays were cast (64 spokes x 3 slices per revolution, probes included).
  EXPECT_GT(sensor->RayCount(), 0u);
  EXPECT_GE(sensor->ScanCount(), 1u);
  EXPECT_EQ(0u, sensor->TargetCount());

  engine->DestroyScene(scene);
}

/////////////////////////////////////////////////
/// \brief A box ahead of the vessel must be detected at the right range and
/// bearing, and must appear on every output topic.
void DetectBox(const std::string &_renderEngine)
{
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
    GTEST_SKIP() << "Engine '" << _renderEngine << "' is not supported";

  auto scene = engine->CreateScene("marine_radar_box");
  ASSERT_NE(nullptr, scene);
  scene->SetAmbientLight(0.3, 0.3, 0.3);
  auto root = scene->RootVisual();

  // A 40 x 40 x 8 m block 300 m ahead: its near face is 280 m away and its
  // top is 8 m above the waterline, so the 1, 3 and 6 m ray slices all hit
  // it.
  auto box = scene->CreateVisual("obstacle");
  ASSERT_NE(nullptr, box);
  box->AddGeometry(scene->CreateBox());
  box->SetLocalScale(40, 40, 8);
  box->SetLocalPosition(300, 0, 4);
  root->AddChild(box);

  auto radarSdf = MarineRadarToSdf("radar", gz::math::Pose3d::Zero,
      "/marine_radar_box", 500);
  ASSERT_NE(nullptr, radarSdf);

  gz::sensors::Manager mgr;
  auto *sensor = mgr.CreateSensor<gz::sensors::MarineRadarSensor>(radarSdf);
  ASSERT_NE(nullptr, sensor);
  sensor->SetScene(scene);
  EXPECT_EQ("/marine_radar_box/targets", sensor->TargetsTopic());

  WaitForMessageTestHelper<gz::msgs::PointCloudPacked> targetsHelper(
      "/marine_radar_box/targets");
  WaitForMessageTestHelper<gz::msgs::PointCloudPacked> returnsHelper(
      "/marine_radar_box/returns");
  WaitForMessageTestHelper<gz::msgs::Image> imageHelper(
      "/marine_radar_box/image");
  EXPECT_TRUE(sensor->HasConnections());

  using namespace std::chrono_literals;
  auto simTime = std::chrono::steady_clock::duration::zero();
  for (int i = 0; i < 310; ++i)
  {
    sensor->SetPose(gz::math::Pose3d(gz::math::Vector3d(0, 0, 5),
        gz::math::Quaterniond::Identity));
    mgr.RunOnce(simTime, true);
    simTime += 1ms;
  }

  EXPECT_GE(sensor->ScanCount(), 2u);
  EXPECT_GE(sensor->TargetCount(), 1u);

  // The PPI image is published with the configured size and pixel format.
  ASSERT_TRUE(imageHelper.WaitForMessage(std::chrono::seconds(5)));
  auto imageMsg = imageHelper.Message();
  EXPECT_EQ(64u, imageMsg.width());
  EXPECT_EQ(64u, imageMsg.height());
  EXPECT_EQ(gz::msgs::PixelFormatType::RGB_INT8,
      imageMsg.pixel_format_type());
  EXPECT_EQ(64u * 64u * 3u, imageMsg.data().size());

  // One point per echo cell.
  ASSERT_TRUE(returnsHelper.WaitForMessage(std::chrono::seconds(5)));
  EXPECT_GT(returnsHelper.Message().width(), 0u);

  // One point per target, with the expected range and bearing. The near face
  // of the block is 280 m from the antenna; the echo is quantised into 5 m
  // range bins and the centroid is intensity weighted over the visible face,
  // so a generous tolerance is used.
  ASSERT_TRUE(targetsHelper.WaitForMessage(std::chrono::seconds(5)));
  auto targetsMsg = targetsHelper.Message();
  ASSERT_GT(targetsMsg.width(), 0u);

  float range = 0.0f;
  float bearing = 0.0f;
  float intensity = 0.0f;
  ASSERT_TRUE(FirstPointField(targetsMsg, "range", range));
  ASSERT_TRUE(FirstPointField(targetsMsg, "bearing", bearing));
  ASSERT_TRUE(FirstPointField(targetsMsg, "intensity", intensity));
  EXPECT_NEAR(280.0, range, 20.0);
  EXPECT_NEAR(0.0, bearing, 0.2);
  // Well above the 8 dB detection threshold at 280 m.
  EXPECT_GT(intensity, 8.0f);

  engine->DestroyScene(scene);
}

/////////////////////////////////////////////////
/// \brief An obstacle beyond range_max must not be reported, and must come
/// back once the range is extended at run time.
void RangeLimit(const std::string &_renderEngine)
{
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
    GTEST_SKIP() << "Engine '" << _renderEngine << "' is not supported";

  auto scene = engine->CreateScene("marine_radar_range");
  ASSERT_NE(nullptr, scene);
  auto root = scene->RootVisual();

  // Near face 280 m away, outside a 100 m radar.
  auto box = scene->CreateVisual("obstacle");
  box->AddGeometry(scene->CreateBox());
  box->SetLocalScale(40, 40, 8);
  box->SetLocalPosition(300, 0, 4);
  root->AddChild(box);

  auto radarSdf = MarineRadarToSdf("radar", gz::math::Pose3d::Zero,
      "/marine_radar_range", 100);
  ASSERT_NE(nullptr, radarSdf);

  gz::sensors::Manager mgr;
  auto *sensor = mgr.CreateSensor<gz::sensors::MarineRadarSensor>(radarSdf);
  ASSERT_NE(nullptr, sensor);
  sensor->SetScene(scene);

  using namespace std::chrono_literals;
  auto simTime = std::chrono::steady_clock::duration::zero();
  auto spin = [&](int _steps)
  {
    for (int i = 0; i < _steps; ++i)
    {
      sensor->SetPose(gz::math::Pose3d(gz::math::Vector3d(0, 0, 5),
          gz::math::Quaterniond::Identity));
      mgr.RunOnce(simTime, true);
      simTime += 1ms;
    }
  };

  spin(310);
  EXPECT_GE(sensor->ScanCount(), 2u);
  EXPECT_EQ(0u, sensor->TargetCount());

  // Extend the range past the obstacle: the pipeline restarts, so give it
  // two more revolutions.
  ASSERT_TRUE(sensor->SetRangeMax(500.0));
  EXPECT_DOUBLE_EQ(500.0, sensor->RangeMax());
  spin(310);
  EXPECT_GE(sensor->TargetCount(), 1u);

  engine->DestroyScene(scene);
}

/////////////////////////////////////////////////
TEST_P(MarineRadarSensorTest, EmptyScene)
{
  EmptyScene(GetParam());
}

/////////////////////////////////////////////////
TEST_P(MarineRadarSensorTest, DetectBox)
{
  DetectBox(GetParam());
}

/////////////////////////////////////////////////
TEST_P(MarineRadarSensorTest, RangeLimit)
{
  RangeLimit(GetParam());
}

INSTANTIATE_TEST_SUITE_P(MarineRadarSensor, MarineRadarSensorTest,
    RENDER_ENGINE_VALUES, gz::rendering::PrintToStringParam());
