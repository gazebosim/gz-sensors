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

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/sensors/CameraSensor.hh>

// TODO(louise) Remove these pragmas once ign-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
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

std::mutex g_mutex;
unsigned int g_imgCounter1 = 0;
unsigned int g_imgCounter2 = 0;

void OnGrayscaleImageL8(const ignition::msgs::Image &_msg)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  EXPECT_EQ(ignition::msgs::PixelFormatType::L_INT8, _msg.pixel_format_type());
  EXPECT_EQ(256u, _msg.width());
  EXPECT_EQ(256u, _msg.height());
  g_imgCounter1++;
}

void OnGrayscaleImageL16(const ignition::msgs::Image &_msg)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  EXPECT_EQ(ignition::msgs::PixelFormatType::L_INT16, _msg.pixel_format_type());
  EXPECT_EQ(512u, _msg.width());
  EXPECT_EQ(512u, _msg.height());
  g_imgCounter2++;
}

class CameraSensorTest: public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
  }

  // Create a Camera sensor from a SDF and gets a image message
  public: void ImagesWithBuiltinSDF(const std::string &_renderEngine);

  // Create 8 bit and 16 bit grayscale camera sensors and verify image format
  public: void ImageFormatLInt8LInt16(const std::string &_renderEngine);
};

void CameraSensorTest::ImagesWithBuiltinSDF(const std::string &_renderEngine)
{
  // get the darn test data
  std::string path = ignition::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "camera_sensor_builtin.sdf");
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
  auto *engine = ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");

  // do the test
  ignition::sensors::Manager mgr;

  ignition::sensors::CameraSensor *sensor =
      mgr.CreateSensor<ignition::sensors::CameraSensor>(sensorPtr);
  ASSERT_NE(sensor, nullptr);
  EXPECT_FALSE(sensor->HasConnections());
  sensor->SetScene(scene);

  ASSERT_NE(sensor->RenderingCamera(), nullptr);
  EXPECT_NE(sensor->Id(), sensor->RenderingCamera()->Id());
  EXPECT_EQ(256u, sensor->ImageWidth());
  EXPECT_EQ(257u, sensor->ImageHeight());

  EXPECT_EQ(std::string("base_camera"), sensor->FrameId());

  std::string topic = "/test/integration/CameraPlugin_imagesWithBuiltinSDF";
  WaitForMessageTestHelper<ignition::msgs::Image> helper(topic);

  EXPECT_TRUE(sensor->HasConnections());

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  // verify sensor does not update / publish data when not active
  sensor->SetActive(false);
  EXPECT_FALSE(sensor->IsActive());
  mgr.RunOnce(std::chrono::seconds(1));
  EXPECT_FALSE(helper.WaitForMessage(std::chrono::seconds(3))) << helper;

  // sensor should update when forced even if it is not active
  mgr.RunOnce(std::chrono::seconds(1), true);
  EXPECT_TRUE(helper.WaitForMessage(std::chrono::seconds(3))) << helper;

  // make the sensor active again and verify data is published
  sensor->SetActive(true);
  EXPECT_TRUE(sensor->IsActive());
  mgr.RunOnce(std::chrono::seconds(2));
  EXPECT_TRUE(helper.WaitForMessage(std::chrono::seconds(3))) << helper;

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
  ignition::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(CameraSensorTest, ImagesWithBuiltinSDF)
{
  ImagesWithBuiltinSDF(GetParam());
}

//////////////////////////////////////////////////
void CameraSensorTest::ImageFormatLInt8LInt16(const std::string &_renderEngine)
{
  // get the darn test data
  std::string path = ignition::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "camera_sensor_l8_l16_builtin.sdf");
  sdf::SDFPtr doc(new sdf::SDF());
  sdf::init(doc);
  ASSERT_TRUE(sdf::readFile(path, doc));
  ASSERT_NE(nullptr, doc->Root());
  ASSERT_TRUE(doc->Root()->HasElement("model"));
  auto modelPtr = doc->Root()->GetElement("model");
  ASSERT_TRUE(modelPtr->HasElement("link"));
  auto linkPtr = modelPtr->GetElement("link");
  ASSERT_TRUE(linkPtr->HasElement("sensor"));
  auto sensorPtr1 = linkPtr->GetElement("sensor");
  auto sensorPtr2 = linkPtr->GetElement("sensor")->GetNextElement();

  // Setup ign-rendering with an empty scene
  auto *engine = ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");

  // do the test
  ignition::sensors::Manager mgr;

  ignition::sensors::CameraSensor *sensor1 =
      mgr.CreateSensor<ignition::sensors::CameraSensor>(sensorPtr1);
  ignition::sensors::CameraSensor *sensor2 =
      mgr.CreateSensor<ignition::sensors::CameraSensor>(sensorPtr2);
  ASSERT_NE(sensor1, nullptr);
  ASSERT_NE(sensor2, nullptr);
  sensor1->SetScene(scene);
  sensor2->SetScene(scene);

  ASSERT_NE(sensor1->RenderingCamera(), nullptr);
  EXPECT_NE(sensor1->Id(), sensor1->RenderingCamera()->Id());
  EXPECT_EQ(256u, sensor1->ImageWidth());
  EXPECT_EQ(256u, sensor1->ImageHeight());

  ASSERT_NE(sensor2->RenderingCamera(), nullptr);
  EXPECT_NE(sensor2->Id(), sensor2->RenderingCamera()->Id());
  EXPECT_EQ(512u, sensor2->ImageWidth());
  EXPECT_EQ(512u, sensor2->ImageHeight());

  std::string topic1 = "/images_l8";
  WaitForMessageTestHelper<ignition::msgs::Image> helper1(topic1);

  std::string topic2 = "/images_l16";
  WaitForMessageTestHelper<ignition::msgs::Image> helper2(topic2);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper1.WaitForMessage()) << helper1;
  EXPECT_TRUE(helper2.WaitForMessage()) << helper2;

  // subscribe to the camera topic
  ignition::transport::Node node;
  node.Subscribe(topic1, &OnGrayscaleImageL8);
  node.Subscribe(topic2, &OnGrayscaleImageL16);

  // wait for a few camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // run to get image and check image format in callback
  bool done = false;
  int sleep = 0;
  int maxSleep = 10;
  while (!done && sleep++ < maxSleep)
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    done = (g_imgCounter1 > 0 && g_imgCounter2 > 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // test removing sensor
  // first make sure the sensor objects do exist
  auto sensorId1 = sensor1->Id();
  auto sensorId2 = sensor2->Id();
  auto cameraId1 = sensor1->RenderingCamera()->Id();
  auto cameraId2 = sensor2->RenderingCamera()->Id();

  EXPECT_EQ(sensor1, mgr.Sensor(sensorId1));
  EXPECT_EQ(sensor1->RenderingCamera(), scene->SensorById(cameraId1));
  EXPECT_EQ(sensor2, mgr.Sensor(sensorId2));
  EXPECT_EQ(sensor2->RenderingCamera(), scene->SensorById(cameraId2));
  // remove and check sensor objects no longer exist in both sensors and
  // rendering
  EXPECT_TRUE(mgr.Remove(sensorId1));
  EXPECT_TRUE(mgr.Remove(sensorId2));
  EXPECT_EQ(nullptr, mgr.Sensor(sensorId1));
  EXPECT_EQ(nullptr, mgr.Sensor(sensorId2));
  EXPECT_EQ(nullptr, scene->SensorById(cameraId1));
  EXPECT_EQ(nullptr, scene->SensorById(cameraId2));

  // Clean up
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(CameraSensorTest, LInt8ImagesWithBuiltinSDF)
{
  ImageFormatLInt8LInt16(GetParam());
}

INSTANTIATE_TEST_CASE_P(CameraSensor, CameraSensorTest,
    RENDER_ENGINE_VALUES, ignition::rendering::PrintToStringParam());

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ignition::common::Console::SetVerbosity(4);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
