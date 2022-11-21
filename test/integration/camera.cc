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
#include <ignition/rendering/Utils.hh>
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
unsigned int g_imgCounter = 0;

void OnGrayscale8bitImage(const ignition::msgs::Image &_msg)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  EXPECT_EQ(ignition::msgs::PixelFormatType::L_INT8, _msg.pixel_format_type());
  EXPECT_EQ(256u, _msg.width());
  EXPECT_EQ(256u, _msg.height());
  g_imgCounter++;
}

void OnGrayscale16bitImage(const ignition::msgs::Image &_msg)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  EXPECT_EQ(ignition::msgs::PixelFormatType::L_INT16, _msg.pixel_format_type());
  EXPECT_EQ(256u, _msg.width());
  EXPECT_EQ(256u, _msg.height());
  g_imgCounter++;
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

  // Create a 8 bit grayscale camera sensor and verify image format
  public: void ImageFormatLInt8(const std::string &_renderEngine);

  // Create a 16 bit grayscale camera sensor and verify image format
  public: void ImageFormatLInt16(const std::string &_renderEngine);

  // Create camera sensors and verify camera intrinsics
  public: void CameraIntrinsics(const std::string &_renderEngine);

  // Create camera sensors and verify camera projection
  public: void CameraProjection(const std::string &_renderEngine);
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
void CameraSensorTest::CameraIntrinsics(const std::string &_renderEngine)
{
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
                                           "sdf", "camera_intrinsics.sdf");
  sdf::SDFPtr doc(new sdf::SDF());
  sdf::init(doc);
  ASSERT_TRUE(sdf::readFile(path, doc));
  ASSERT_NE(nullptr, doc->Root());
  ASSERT_TRUE(doc->Root()->HasElement("model"));
  auto modelPtr = doc->Root()->GetElement("model");
  ASSERT_TRUE(modelPtr->HasElement("link"));
  auto linkPtr = modelPtr->GetElement("link");
  ASSERT_TRUE(linkPtr->HasElement("sensor"));

  // Camera sensor without intrinsics tag
  auto cameraWithoutIntrinsicsTag = linkPtr->GetElement("sensor");

  // Camera sensor with intrinsics tag
  auto cameraWithIntrinsicsTag =
      linkPtr->GetElement("sensor")->GetNextElement();

  // Setup gz-rendering with an empty scene
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
          << "' is not supported" << std::endl;
    return;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");

  // Do the test
  gz::sensors::Manager mgr;

  auto *sensor1 =
      mgr.CreateSensor<gz::sensors::CameraSensor>(cameraWithoutIntrinsicsTag);
  auto *sensor2 =
      mgr.CreateSensor<gz::sensors::CameraSensor>(cameraWithIntrinsicsTag);
  ASSERT_NE(sensor1, nullptr);
  ASSERT_NE(sensor2, nullptr);
  sensor1->SetScene(scene);
  sensor2->SetScene(scene);

  std::string infoTopic1 = "/camera1/camera_info";
  std::string infoTopic2 = "/camera2/camera_info";
  WaitForMessageTestHelper<gz::msgs::Image> helper1("/camera1/image");
  WaitForMessageTestHelper<gz::msgs::CameraInfo> helper2(infoTopic1);
  WaitForMessageTestHelper<gz::msgs::Image> helper3("/camera2/image");
  WaitForMessageTestHelper<gz::msgs::CameraInfo> helper4(infoTopic2);
  EXPECT_TRUE(sensor1->HasConnections());
  EXPECT_TRUE(sensor2->HasConnections());

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper1.WaitForMessage()) << helper1;
  EXPECT_TRUE(helper2.WaitForMessage()) << helper2;
  EXPECT_TRUE(helper3.WaitForMessage()) << helper3;
  EXPECT_TRUE(helper4.WaitForMessage()) << helper4;

  // Subscribe to the camera info topic
  gz::msgs::CameraInfo camera1Info, camera2Info;
  unsigned int camera1Counter = 0;
  unsigned int camera2Counter = 0;

  std::function<void(const gz::msgs::CameraInfo&)> camera1InfoCallback =
      [&camera1Info, &camera1Counter](const gz::msgs::CameraInfo& _msg) {
        camera1Info = _msg;
        camera1Counter++;
  };

  std::function<void(const gz::msgs::CameraInfo&)> camera2InfoCallback =
      [&camera2Info, &camera2Counter](const gz::msgs::CameraInfo& _msg) {
        camera2Info = _msg;
        camera2Counter++;
  };

  // Subscribe to the camera topic
  gz::transport::Node node;
  node.Subscribe(infoTopic1, camera1InfoCallback);
  node.Subscribe(infoTopic2, camera2InfoCallback);

  // Wait for a few camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // Run to get image and check image format in callback
  bool done = false;
  int sleep = 0;
  int maxSleep = 10;
  while (!done && sleep++ < maxSleep)
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    done = (camera1Counter > 0 && camera2Counter > 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Image size, focal length and optical center
  // Camera sensor without intrinsics tag
  double error = 1e-1;
  EXPECT_EQ(camera1Info.width(), 1000u);
  EXPECT_EQ(camera1Info.height(), 1000u);
  EXPECT_NEAR(camera1Info.intrinsics().k(0), 863.2297, error);
  EXPECT_NEAR(camera1Info.intrinsics().k(4), 863.2297, error);
  EXPECT_DOUBLE_EQ(camera1Info.intrinsics().k(2), 500);
  EXPECT_DOUBLE_EQ(camera1Info.intrinsics().k(5), 500);

  // Camera sensor with intrinsics tag
  EXPECT_EQ(camera2Info.width(), 1000u);
  EXPECT_EQ(camera2Info.height(), 1000u);
  EXPECT_DOUBLE_EQ(camera2Info.intrinsics().k(0), 866.23);
  EXPECT_DOUBLE_EQ(camera2Info.intrinsics().k(4), 866.23);
  EXPECT_DOUBLE_EQ(camera2Info.intrinsics().k(2), 500);
  EXPECT_DOUBLE_EQ(camera2Info.intrinsics().k(5), 500);

  // Clean up
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(CameraSensorTest, CameraIntrinsics)
{
  gz::common::Console::SetVerbosity(2);
  CameraIntrinsics(GetParam());
}


//////////////////////////////////////////////////
void CameraSensorTest::CameraProjection(const std::string &_renderEngine)
{
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
                                           "sdf", "camera_projection.sdf");
  sdf::SDFPtr doc(new sdf::SDF());
  sdf::init(doc);
  ASSERT_TRUE(sdf::readFile(path, doc));
  ASSERT_NE(nullptr, doc->Root());
  ASSERT_TRUE(doc->Root()->HasElement("model"));
  auto modelPtr = doc->Root()->GetElement("model");
  ASSERT_TRUE(modelPtr->HasElement("link"));
  auto linkPtr = modelPtr->GetElement("link");
  ASSERT_TRUE(linkPtr->HasElement("sensor"));

  // Camera sensor without intrinsics tag
  auto cameraWithoutIntrinsicsTag = linkPtr->GetElement("sensor");

  // Camera sensor with intrinsics tag
  auto cameraWithIntrinsicsTag =
      linkPtr->GetElement("sensor")->GetNextElement();

  // Setup gz-rendering with an empty scene
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
          << "' is not supported" << std::endl;
    return;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");

  // Do the test
  gz::sensors::Manager mgr;

  auto *sensor1 =
      mgr.CreateSensor<gz::sensors::CameraSensor>(cameraWithoutIntrinsicsTag);
  auto *sensor2 =
      mgr.CreateSensor<gz::sensors::CameraSensor>(cameraWithIntrinsicsTag);
  ASSERT_NE(sensor1, nullptr);
  ASSERT_NE(sensor2, nullptr);
  sensor1->SetScene(scene);
  sensor2->SetScene(scene);

  std::string infoTopic1 = "/camera1/camera_info";
  std::string infoTopic2 = "/camera2/camera_info";
  WaitForMessageTestHelper<gz::msgs::Image> helper1("/camera1/image");
  WaitForMessageTestHelper<gz::msgs::CameraInfo> helper2(infoTopic1);
  WaitForMessageTestHelper<gz::msgs::Image> helper3("/camera2/image");
  WaitForMessageTestHelper<gz::msgs::CameraInfo> helper4(infoTopic2);
  EXPECT_TRUE(sensor1->HasConnections());
  EXPECT_TRUE(sensor2->HasConnections());

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper1.WaitForMessage()) << helper1;
  EXPECT_TRUE(helper2.WaitForMessage()) << helper2;
  EXPECT_TRUE(helper3.WaitForMessage()) << helper3;
  EXPECT_TRUE(helper4.WaitForMessage()) << helper4;

  // Subscribe to the camera info topic
  gz::msgs::CameraInfo camera1Info, camera2Info;
  unsigned int camera1Counter = 0;
  unsigned int camera2Counter = 0;

  std::function<void(const gz::msgs::CameraInfo&)> camera1InfoCallback =
      [&camera1Info, &camera1Counter](const gz::msgs::CameraInfo& _msg) {
        camera1Info = _msg;
        camera1Counter++;
  };

  std::function<void(const gz::msgs::CameraInfo&)> camera2InfoCallback =
      [&camera2Info, &camera2Counter](const gz::msgs::CameraInfo& _msg) {
        camera2Info = _msg;
        camera2Counter++;
  };

  // Subscribe to the camera topic
  gz::transport::Node node;
  node.Subscribe(infoTopic1, camera1InfoCallback);
  node.Subscribe(infoTopic2, camera2InfoCallback);

  // Wait for a few camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // Run to get image and check image format in callback
  bool done = false;
  int sleep = 0;
  int maxSleep = 10;
  while (!done && sleep++ < maxSleep)
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    done = (camera1Counter > 0 && camera2Counter > 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Image size, focal length and optical center
  // Camera sensor without projection tag
  double error = 1e-1;
  EXPECT_EQ(camera1Info.width(), 1000u);
  EXPECT_EQ(camera1Info.height(), 1000u);
  EXPECT_NEAR(camera1Info.projection().p(0), 863.22975158691406, error);
  EXPECT_NEAR(camera1Info.projection().p(5), 863.22975158691406, error);
  EXPECT_DOUBLE_EQ(camera1Info.projection().p(2), 500);
  EXPECT_DOUBLE_EQ(camera1Info.projection().p(6), 500);
  EXPECT_DOUBLE_EQ(camera1Info.projection().p(3), 0);
  EXPECT_DOUBLE_EQ(camera1Info.projection().p(7), 0);

  // Camera sensor with projection tag
  EXPECT_EQ(camera2Info.width(), 1000u);
  EXPECT_EQ(camera2Info.height(), 1000u);
  EXPECT_DOUBLE_EQ(camera2Info.projection().p(0), 866.23);
  EXPECT_DOUBLE_EQ(camera2Info.projection().p(5), 966.23);
  EXPECT_DOUBLE_EQ(camera2Info.projection().p(2), 500);
  EXPECT_DOUBLE_EQ(camera2Info.projection().p(6), 400);
  EXPECT_DOUBLE_EQ(camera2Info.projection().p(3), 300);
  EXPECT_DOUBLE_EQ(camera2Info.projection().p(7), 200);

  // Clean up
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(CameraSensorTest, CameraProjection)
{
  gz::common::Console::SetVerbosity(2);
  CameraProjection(GetParam());
}

//////////////////////////////////////////////////
void CameraSensorTest::ImageFormatLInt8(const std::string &_renderEngine)
{
  // get the darn test data
  std::string path = ignition::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "camera_sensor_l8_builtin.sdf");
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
  sensor->SetScene(scene);

  ASSERT_NE(sensor->RenderingCamera(), nullptr);
  EXPECT_NE(sensor->Id(), sensor->RenderingCamera()->Id());
  EXPECT_EQ(256u, sensor->ImageWidth());
  EXPECT_EQ(256u, sensor->ImageHeight());

  std::string topic = "/images_l8";
  WaitForMessageTestHelper<ignition::msgs::Image> helper(topic);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  g_imgCounter = 0u;

  // subscribe to the camera topic
  ignition::transport::Node node;
  node.Subscribe(topic, &OnGrayscale8bitImage);

  // wait for a few camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // run to get image and check image format in callback
  bool done = false;
  int sleep = 0;
  int maxSleep = 10;
  while (!done && sleep++ < maxSleep)
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    done = (g_imgCounter > 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
  ignition::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(CameraSensorTest, LInt8ImagesWithBuiltinSDF)
{
  ImageFormatLInt8(GetParam());
}

//////////////////////////////////////////////////
void CameraSensorTest::ImageFormatLInt16(const std::string &_renderEngine)
{
  // get the darn test data
  std::string path = ignition::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "camera_sensor_l16_builtin.sdf");
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

  // Setup empty scene
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
  sensor->SetScene(scene);

  ASSERT_NE(sensor->RenderingCamera(), nullptr);
  EXPECT_NE(sensor->Id(), sensor->RenderingCamera()->Id());
  EXPECT_EQ(256u, sensor->ImageWidth());
  EXPECT_EQ(256u, sensor->ImageHeight());

  std::string topic = "/images_l16";
  WaitForMessageTestHelper<ignition::msgs::Image> helper(topic);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  g_imgCounter = 0u;

  // subscribe to the camera topic
  ignition::transport::Node node;
  node.Subscribe(topic, &OnGrayscale16bitImage);

  // wait for a few camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // run to get image and check image format in callback
  bool done = false;
  int sleep = 0;
  int maxSleep = 10;
  while (!done && sleep++ < maxSleep)
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    done = (g_imgCounter > 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
  ignition::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(CameraSensorTest, LInt16ImagesWithBuiltinSDF)
{
  ImageFormatLInt16(GetParam());
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
