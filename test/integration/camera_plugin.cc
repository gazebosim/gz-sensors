/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <gz/common/Filesystem.hh>
#include <gz/common/Time.hh>
#include <gz/sensors/Manager.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/transport/Node.hh>

// TODO(louise) Remove these pragmas once ign-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <ignition/msgs.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include "test_config.h"  // NOLINT(build/include)
#include "TransportTestTools.hh"


std::mutex g_infoMutex;
unsigned int g_infoCounter = 0;
gz::msgs::CameraInfo g_infoMsg;

//////////////////////////////////////////////////
void OnCameraInfo(const gz::msgs::CameraInfo & _msg)
{
  g_infoMutex.lock();
  g_infoCounter++;
  g_infoMsg.CopyFrom(_msg);
  g_infoMutex.unlock();
}

//////////////////////////////////////////////////
class CameraSensorTest: public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Create a Camera sensor from a SDF and gets a image message
  public: void ImagesWithBuiltinSDF(const std::string &_renderEngine);
};

//////////////////////////////////////////////////
void CameraSensorTest::ImagesWithBuiltinSDF(const std::string &_renderEngine)
{
  // get the darn test data
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "integration", "camera_sensor_builtin.sdf");
  sdf::SDFPtr doc(new sdf::SDF());
  sdf::init(doc);
  ASSERT_TRUE(sdf::readFile(path, doc));
  ASSERT_NE(nullptr, doc->Root());
  ASSERT_TRUE(doc->Root()->HasElement("model"));
  auto modelPtr = doc->Root()->GetElement("model");
  ASSERT_TRUE(modelPtr->HasElement("link"));
  auto linkPtr = modelPtr->GetElement("link");
  ASSERT_TRUE(linkPtr->HasElement("sensor"));
  auto sensorPtr = linkPtr->GetElement("sensor");

  // Setup ign-rendering with an empty scene
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");

  // do the test
  gz::sensors::Manager mgr;

  gz::sensors::CameraSensor *sensor =
      mgr.CreateSensor<gz::sensors::CameraSensor>(sensorPtr);
  ASSERT_NE(sensor, nullptr);
  sensor->SetScene(scene);

  ASSERT_NE(sensor->RenderingCamera(), nullptr);
  EXPECT_NE(sensor->Id(), sensor->RenderingCamera()->Id());
  EXPECT_EQ(256u, sensor->ImageWidth());
  EXPECT_EQ(257u, sensor->ImageHeight());

  EXPECT_EQ(std::string("base_camera"), sensor->FrameId());

  // subscribe to the camera info topic
  std::string infoTopic = sensor->InfoTopic();
  gz::transport::Node node;
  node.Subscribe(infoTopic, &OnCameraInfo);
  WaitForMessageTestHelper<gz::msgs::CameraInfo> helperInfo(infoTopic);

  std::string topic = "/test/integration/CameraPlugin_imagesWithBuiltinSDF";
  WaitForMessageTestHelper<gz::msgs::Image> helper(topic);

  // Update once to create image
  mgr.RunOnce(gz::common::Time::Zero);

  EXPECT_TRUE(helper.WaitForMessage()) << helper;
  EXPECT_TRUE(helperInfo.WaitForMessage()) << helperInfo;

  // Check CameraInfo properties
  gz::msgs::CameraInfo infoMsg;
  g_infoMutex.lock();
  EXPECT_EQ(1u, g_infoCounter);
  infoMsg.CopyFrom(g_infoMsg);
  g_infoMutex.unlock();

  {
    auto intrinsics = infoMsg.intrinsics();
    EXPECT_EQ(9, intrinsics.k_size());
    EXPECT_DOUBLE_EQ(280.0, intrinsics.k(0));
    EXPECT_DOUBLE_EQ(0.0, intrinsics.k(1));
    EXPECT_DOUBLE_EQ(162.0, intrinsics.k(2));
    EXPECT_DOUBLE_EQ(0.0, intrinsics.k(3));
    EXPECT_DOUBLE_EQ(281.0, intrinsics.k(4));
    EXPECT_DOUBLE_EQ(124.0, intrinsics.k(5));
    EXPECT_DOUBLE_EQ(0.0, intrinsics.k(6));
    EXPECT_DOUBLE_EQ(0.0, intrinsics.k(7));
    EXPECT_DOUBLE_EQ(1.0, intrinsics.k(8));
  }

  {
    auto projection = infoMsg.projection();
    EXPECT_EQ(12, projection.p_size());
    EXPECT_DOUBLE_EQ(282.0, projection.p(0));
    EXPECT_DOUBLE_EQ(0.0, projection.p(1));
    EXPECT_DOUBLE_EQ(163.0, projection.p(2));
    EXPECT_DOUBLE_EQ(1.0, projection.p(3));
    EXPECT_DOUBLE_EQ(0.0, projection.p(4));
    EXPECT_DOUBLE_EQ(283.0, projection.p(5));
    EXPECT_DOUBLE_EQ(125.0, projection.p(6));
    EXPECT_DOUBLE_EQ(2.0, projection.p(7));
    EXPECT_DOUBLE_EQ(0.0, projection.p(8));
    EXPECT_DOUBLE_EQ(0.0, projection.p(9));
    EXPECT_DOUBLE_EQ(1.0, projection.p(10));
    EXPECT_DOUBLE_EQ(0.0, projection.p(11));
  }

  // test removing sensor
  // first make sure the sensor objects do exist
  auto sensorId = sensor->Id();
  auto cameraId = sensor->RenderingCamera()->Id();
  EXPECT_EQ(sensor, mgr.Sensor(sensorId));
  EXPECT_EQ(sensor->RenderingCamera(), scene->SensorById(cameraId));
  // remove and check sensor objects no longer exist in both sensors and
  // rendering
  EXPECT_TRUE(mgr.Remove(sensorId));
  EXPECT_EQ(nullptr, mgr.Sensor(sensorId));
  EXPECT_EQ(nullptr, scene->SensorById(cameraId));

  // Clean up
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(CameraSensorTest, ImagesWithBuiltinSDF)
{
  ImagesWithBuiltinSDF(GetParam());
}

INSTANTIATE_TEST_CASE_P(CameraSensor, CameraSensorTest,
    RENDER_ENGINE_VALUES, gz::rendering::PrintToStringParam());

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  gz::common::Console::SetVerbosity(4);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
