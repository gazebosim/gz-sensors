/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <cstring>
#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/image.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/sensors/Manager.hh>

// TODO(louise) Remove these pragmas once gz-rendering is disabling the
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

#include "test_config.hh"  // NOLINT(build/include)
#include "TransportTestTools.hh"

using namespace std::chrono_literals;

class TriggeredCameraTest: public testing::Test,
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
    gz::common::Console::SetVerbosity(4);
  }

  // Create a Camera sensor from a SDF and gets a image message
  public: void ImagesWithBuiltinSDF(const std::string &_renderEngine);

  // Create a Camera sensor from a SDF with empty trigger topic
  public: void EmptyTriggerTopic(const std::string &_renderEngine);
};

void TriggeredCameraTest::ImagesWithBuiltinSDF(const std::string &_renderEngine)
{
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "triggered_camera_sensor_builtin.sdf");
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

  // Setup gz-rendering with an empty scene
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    gzdbg << "Engine '" << _renderEngine
           << "' is not supported" << std::endl;
    return;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");

  gz::sensors::Manager mgr;

  gz::sensors::CameraSensor *sensor =
      mgr.CreateSensor<gz::sensors::CameraSensor>(sensorPtr);
  ASSERT_NE(sensor, nullptr);
  EXPECT_FALSE(sensor->HasConnections());
  sensor->SetScene(scene);

  sdf::Sensor sdfSensor;
  sdfSensor.Load(sensorPtr);
  EXPECT_EQ(true, sdfSensor.CameraSensor()->Triggered());

  ASSERT_NE(sensor->RenderingCamera(), nullptr);
  EXPECT_NE(sensor->Id(), sensor->RenderingCamera()->Id());
  EXPECT_EQ(256u, sensor->ImageWidth());
  EXPECT_EQ(257u, sensor->ImageHeight());

  // check camera image before trigger
  {
    std::string imageTopic =
        "/test/integration/TriggeredCameraPlugin_imagesWithBuiltinSDF";
    WaitForMessageTestHelper<gz::msgs::Image> helper(imageTopic);
    EXPECT_TRUE(sensor->HasConnections());
    mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
    EXPECT_FALSE(helper.WaitForMessage(1s)) << helper;
  }

  // trigger camera through topic
  gz::transport::Node triggerNode;
  std::string triggerTopic =
      "/test/integration/TriggeredCameraPlugin_imagesWithBuiltinSDF/trigger";

  auto pub = triggerNode.Advertise<gz::msgs::Boolean>(triggerTopic);
  gz::msgs::Boolean msg;
  msg.set_data(true);
  pub.Publish(msg);

  // sleep to wait for trigger msg to be received before calling mgr.RunOnce
  std::this_thread::sleep_for(2s);

  // check camera image after trigger
  {
    std::string imageTopic =
        "/test/integration/TriggeredCameraPlugin_imagesWithBuiltinSDF";
    WaitForMessageTestHelper<gz::msgs::Image> helper(imageTopic);
    mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
    EXPECT_TRUE(helper.WaitForMessage(10s)) << helper;
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

void TriggeredCameraTest::EmptyTriggerTopic(const std::string &_renderEngine)
{
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "triggered_camera_sensor_topic_builtin.sdf");
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

  // Setup gz-rendering with an empty scene
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    gzdbg << "Engine '" << _renderEngine
           << "' is not supported" << std::endl;
    return;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");

  gz::sensors::Manager mgr;

  gz::sensors::CameraSensor *sensor =
      mgr.CreateSensor<gz::sensors::CameraSensor>(sensorPtr);
  ASSERT_NE(sensor, nullptr);
  EXPECT_FALSE(sensor->HasConnections());
  sensor->SetScene(scene);

  sdf::Sensor sdfSensor;
  sdfSensor.Load(sensorPtr);
  EXPECT_EQ(true, sdfSensor.CameraSensor()->Triggered());

  // trigger camera through default generated topic
  gz::transport::Node triggerNode;
  std::string triggerTopic =
      "/test/integration/triggered_camera/trigger";

  auto pub = triggerNode.Advertise<gz::msgs::Boolean>(triggerTopic);
  gz::msgs::Boolean msg;
  msg.set_data(true);
  pub.Publish(msg);

  // check camera image after trigger
  {
    std::string imageTopic =
        "/test/integration/triggered_camera";
    WaitForMessageTestHelper<gz::msgs::Image> helper(imageTopic);
    mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
    EXPECT_TRUE(helper.WaitForMessage(10s)) << helper;
  }

  // Clean up
  auto sensorId = sensor->Id();
  EXPECT_TRUE(mgr.Remove(sensorId));
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(TriggeredCameraTest, ImagesWithBuiltinSDF)
{
  gz::common::Console::SetVerbosity(4);
  ImagesWithBuiltinSDF(GetParam());
}

//////////////////////////////////////////////////
TEST_P(TriggeredCameraTest, EmptyTriggerTopic)
{
  gz::common::Console::SetVerbosity(4);
  EmptyTriggerTopic(GetParam());
}

INSTANTIATE_TEST_SUITE_P(CameraSensor, TriggeredCameraTest,
    RENDER_ENGINE_VALUES, gz::rendering::PrintToStringParam());
