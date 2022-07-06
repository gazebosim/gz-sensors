
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

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <ignition/msgs/camera_info.pb.h>
#include <ignition/msgs.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Event.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/sensors/ThermalCameraSensor.hh>
#include <ignition/utils/ExtraTestMacros.hh>

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

#include "test_config.h"  // NOLINT(build/include)
#include "TransportTestTools.hh"

#define DOUBLE_TOL 1e-6

std::mutex g_mutex;
unsigned int g_thermalCounter = 0;
uint16_t *g_thermalBuffer = nullptr;
unsigned char *g_thermalBuffer8Bit = nullptr;

std::mutex g_infoMutex;
unsigned int g_infoCounter = 0;
ignition::msgs::CameraInfo g_infoMsg;

void OnCameraInfo(const ignition::msgs::CameraInfo & _msg)
{
  g_infoMutex.lock();
  g_infoCounter++;
  g_infoMsg.CopyFrom(_msg);
  g_infoMutex.unlock();
}

void OnImage(const ignition::msgs::Image &_msg)
{
  g_mutex.lock();
  unsigned int thermalSamples = _msg.width() * _msg.height();
  unsigned int thermalBufferSize = thermalSamples * sizeof(uint16_t);
  if (!g_thermalBuffer)
    g_thermalBuffer = new uint16_t[thermalSamples];
  memcpy(g_thermalBuffer, _msg.data().c_str(), thermalBufferSize);
  g_thermalCounter++;
  g_mutex.unlock();
}

void OnImage8Bit(const ignition::msgs::Image &_msg)
{
  g_mutex.lock();
  unsigned int thermalSamples = _msg.width() * _msg.height();
  unsigned int thermalBufferSize = thermalSamples * sizeof(unsigned char);
  if (!g_thermalBuffer8Bit)
    g_thermalBuffer8Bit = new unsigned char[thermalSamples];
  memcpy(g_thermalBuffer8Bit, _msg.data().c_str(), thermalBufferSize);
  g_thermalCounter++;
  g_mutex.unlock();
}

class ThermalCameraSensorTest: public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Create a Camera sensor from a SDF and gets a image message
  public: void ImagesWithBuiltinSDF(const std::string &_renderEngine);

  // Create a thermal camera sensor from a SDF with 8 bit image format
  public: void Images8BitWithBuiltinSDF(const std::string &_renderEngine);
};

void ThermalCameraSensorTest::ImagesWithBuiltinSDF(
    const std::string &_renderEngine)
{
  // get the darn test data
  std::string path = ignition::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "thermal_camera_sensor_builtin.sdf");
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
  ASSERT_TRUE(sensorPtr->HasElement("camera"));
  auto cameraPtr = sensorPtr->GetElement("camera");
  ASSERT_TRUE(cameraPtr->HasElement("image"));
  auto imagePtr = cameraPtr->GetElement("image");
  ASSERT_TRUE(cameraPtr->HasElement("clip"));
  auto clipPtr = cameraPtr->GetElement("clip");

  int imgWidth = imagePtr->Get<int>("width");
  int imgHeight = imagePtr->Get<int>("height");
  double far_ = clipPtr->Get<double>("far");
  double near_ = clipPtr->Get<double>("near");

  double unitBoxSize = 1.0;
  ignition::math::Vector3d boxPosition(3.0, 0.0, 0.0);

  // If ogre is not the engine, don't run the test
  if ((_renderEngine.compare("ogre") != 0) &&
      (_renderEngine.compare("ogre2") != 0))
  {
    igndbg << "Engine '" << _renderEngine
              << "' doesn't support thermal cameras" << std::endl;
    return;
  }

  // Setup ign-rendering with an empty scene
  auto *engine = ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");

  // Create an scene with a box in it
  scene->SetAmbientLight(0.3, 0.3, 0.3);
  ignition::rendering::VisualPtr root = scene->RootVisual();

  // create box visual
  ignition::rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(boxPosition);
  box->SetLocalRotation(0, 0, 0);
  box->SetLocalScale(unitBoxSize, unitBoxSize, unitBoxSize);

  // set box temperature
  float boxTemp = 310.0;
  box->SetUserData("temperature", boxTemp);

  root->AddChild(box);

  ignition::sensors::Manager mgr;

  ignition::sensors::ThermalCameraSensor *thermalSensor =
      mgr.CreateSensor<ignition::sensors::ThermalCameraSensor>(sensorPtr);
  ASSERT_NE(thermalSensor, nullptr);
  EXPECT_FALSE(thermalSensor->HasConnections());

  float ambientTemp = 296.0f;
  float ambientTempRange = 4.0f;
  float linearResolution = 0.01f;
  thermalSensor->SetAmbientTemperature(ambientTemp);
  thermalSensor->SetAmbientTemperatureRange(ambientTempRange);
  thermalSensor->SetLinearResolution(linearResolution);
  thermalSensor->SetScene(scene);

  EXPECT_EQ(thermalSensor->ImageWidth(), static_cast<unsigned int>(imgWidth));
  EXPECT_EQ(thermalSensor->ImageHeight(), static_cast<unsigned int>(imgHeight));

  std::string topic =
    "/test/integration/ThermalCameraPlugin_imagesWithBuiltinSDF/image";
  WaitForMessageTestHelper<ignition::msgs::Image> helper(topic);
  EXPECT_TRUE(thermalSensor->HasConnections());

  std::string infoTopic =
    "/test/integration/ThermalCameraPlugin_imagesWithBuiltinSDF/camera_info";
  WaitForMessageTestHelper<ignition::msgs::CameraInfo> infoHelper(infoTopic);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper.WaitForMessage()) << helper;
  EXPECT_TRUE(infoHelper.WaitForMessage()) << infoHelper;

  // subscribe to the thermal camera topic
  ignition::transport::Node node;
  node.Subscribe(topic, &OnImage);

  // subscribe to the thermal camera topic
  node.Subscribe(infoTopic, &OnCameraInfo);

  // wait for a few thermal camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  int midWidth = static_cast<int>(thermalSensor->ImageWidth() * 0.5);
  int midHeight = static_cast<int>(thermalSensor->ImageHeight() * 0.5);
  int mid = midHeight * thermalSensor->ImageWidth() + midWidth -1;
  int left = midHeight * thermalSensor->ImageWidth();
  int right = (midHeight+1) * thermalSensor->ImageWidth() - 1;

  auto waitTime = std::chrono::duration_cast< std::chrono::milliseconds >(
      std::chrono::duration< double >(0.001));
  int counter = 0;
  int infoCounter = 0;
  ignition::msgs::CameraInfo infoMsg;
  for (int sleep = 0;
       sleep < 300 && (counter == 0 || infoCounter == 0); ++sleep)
  {
    g_mutex.lock();
    counter = g_thermalCounter;
    g_mutex.unlock();

    g_infoMutex.lock();
    infoCounter = g_infoCounter;
    infoMsg = g_infoMsg;
    g_infoMutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }
  g_mutex.lock();
  g_infoMutex.lock();
  g_thermalCounter = 0;
  g_infoCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_EQ(counter, infoCounter);
  counter = 0;
  infoCounter = 0;

  // verify temperature
  // Box should be in the middle of image and return box temp
  // Left and right side of the image frame should be ambient temp
  EXPECT_NEAR(ambientTemp, g_thermalBuffer[left] * linearResolution,
      ambientTempRange);
  EXPECT_NEAR(ambientTemp, g_thermalBuffer[right] * linearResolution,
      ambientTempRange);
  EXPECT_FLOAT_EQ(g_thermalBuffer[right], g_thermalBuffer[left]);

  // temp range of visual is hardcoded in ign-rendering shaders
  float boxTempRange = 3.0;
  EXPECT_NEAR(boxTemp, g_thermalBuffer[mid] * linearResolution, boxTempRange);

  g_infoMutex.unlock();
  g_mutex.unlock();

  // Check camera info
  EXPECT_TRUE(infoMsg.has_header());
  ASSERT_EQ(1, infoMsg.header().data().size());
  EXPECT_EQ("frame_id", infoMsg.header().data(0).key());
  ASSERT_EQ(1, infoMsg.header().data(0).value().size());
  EXPECT_EQ("camera1", infoMsg.header().data(0).value(0));
  EXPECT_TRUE(infoMsg.has_distortion());
  EXPECT_EQ(ignition::msgs::CameraInfo::Distortion::PLUMB_BOB,
      infoMsg.distortion().model());
  EXPECT_EQ(5, infoMsg.distortion().k().size());
  EXPECT_TRUE(infoMsg.has_intrinsics());
  EXPECT_EQ(9, infoMsg.intrinsics().k().size());
  EXPECT_TRUE(infoMsg.has_projection());
  EXPECT_EQ(12, infoMsg.projection().p().size());
  EXPECT_EQ(9, infoMsg.rectification_matrix().size());

  // Check that for a box really close it returns box temperature
  root->RemoveChild(box);
  ignition::math::Vector3d boxPositionNear(
      unitBoxSize * 0.5 + near_ * 0.5, 0.0, 0.0);
  box->SetLocalPosition(boxPositionNear);
  root->AddChild(box);

  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  for (int sleep = 0;
       sleep < 300 && (counter == 0 || infoCounter == 0); ++sleep)
  {
    g_mutex.lock();
    counter = g_thermalCounter;
    g_mutex.unlock();

    g_infoMutex.lock();
    infoCounter = g_infoCounter;
    g_infoMutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }

  g_mutex.lock();
  g_infoMutex.lock();
  g_thermalCounter = 0;
  g_infoCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_EQ(counter, infoCounter);
  counter = 0;
  infoCounter = 0;

  EXPECT_NEAR(boxTemp, g_thermalBuffer[mid] * linearResolution, boxTempRange);
  g_infoMutex.unlock();
  g_mutex.unlock();

  // Check that for a box really far it returns ambient temperature
  root->RemoveChild(box);
  ignition::math::Vector3d boxPositionFar(
      unitBoxSize * 0.5 + far_ * 1.5, 0.0, 0.0);
  box->SetLocalPosition(boxPositionFar);
  root->AddChild(box);

  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  for (int sleep = 0;
       sleep < 300 && (counter == 0 || infoCounter == 0); ++sleep)
  {
    g_mutex.lock();
    counter = g_thermalCounter;
    g_mutex.unlock();

    g_infoMutex.lock();
    infoCounter = g_infoCounter;
    g_infoMutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }
  g_mutex.lock();
  g_infoMutex.lock();
  g_thermalCounter = 0;
  g_infoCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_EQ(counter, infoCounter);
  counter = 0;
  infoCounter = 0;

  EXPECT_NEAR(ambientTemp, g_thermalBuffer[mid] * linearResolution,
      ambientTempRange);
  g_infoMutex.unlock();
  g_mutex.unlock();

  delete [] g_thermalBuffer;
  g_thermalBuffer = nullptr;

  // Clean up
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
// See: https://github.com/gazebosim/gz-rendering/issues/654
TEST_P(ThermalCameraSensorTest,
       IGN_UTILS_TEST_DISABLED_ON_MAC(ImagesWithBuiltinSDF))
{
  ImagesWithBuiltinSDF(GetParam());
}

//////////////////////////////////////////////////
void ThermalCameraSensorTest::Images8BitWithBuiltinSDF(
    const std::string &_renderEngine)
{
  // get the darn test data
  std::string path = ignition::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "thermal_camera_sensor_8bit_builtin.sdf");
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
  ASSERT_TRUE(sensorPtr->HasElement("camera"));
  auto cameraPtr = sensorPtr->GetElement("camera");
  ASSERT_TRUE(cameraPtr->HasElement("image"));
  auto imagePtr = cameraPtr->GetElement("image");

  std::string format = imagePtr->Get<std::string>("format");
  EXPECT_EQ("L8", format);

  ASSERT_TRUE(cameraPtr->HasElement("clip"));
  auto clipPtr = cameraPtr->GetElement("clip");
  int imgWidth = imagePtr->Get<int>("width");
  int imgHeight = imagePtr->Get<int>("height");
  double far_ = clipPtr->Get<double>("far");
  double near_ = clipPtr->Get<double>("near");

  double unitBoxSize = 1.0;
  ignition::math::Vector3d boxPosition(3.0, 0.0, 0.0);

  // If ogre2 is not the engine, don't run the test
  if ((_renderEngine.compare("ogre2") != 0))
  {
    igndbg << "Engine '" << _renderEngine
              << "' doesn't support 8 bit thermal cameras" << std::endl;
    return;
  }

  // Setup ign-rendering with an empty scene
  auto *engine = ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");

  // Create an scene with a box in it
  scene->SetAmbientLight(0.3, 0.3, 0.3);
  ignition::rendering::VisualPtr root = scene->RootVisual();

  // create box visual
  ignition::rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(boxPosition);
  box->SetLocalRotation(0, 0, 0);
  box->SetLocalScale(unitBoxSize, unitBoxSize, unitBoxSize);

  // set box temperature
  float boxTemp = 310.0;
  box->SetUserData("temperature", boxTemp);

  root->AddChild(box);

  ignition::sensors::Manager mgr;

  ignition::sensors::ThermalCameraSensor *thermalSensor =
      mgr.CreateSensor<ignition::sensors::ThermalCameraSensor>(sensorPtr);
  ASSERT_NE(thermalSensor, nullptr);

  float ambientTemp = 296.0f;
  float ambientTempRange = 4.0f;
  float linearResolution = 3.0f;
  thermalSensor->SetAmbientTemperature(ambientTemp);
  thermalSensor->SetAmbientTemperatureRange(ambientTempRange);
  thermalSensor->SetLinearResolution(linearResolution);
  thermalSensor->SetScene(scene);

  EXPECT_EQ(thermalSensor->ImageWidth(), static_cast<unsigned int>(imgWidth));
  EXPECT_EQ(thermalSensor->ImageHeight(), static_cast<unsigned int>(imgHeight));

  std::string topicBase =
    "/test/integration/ThermalCameraPlugin_images8BitWithBuiltinSDF/";
  std::string topic = topicBase + "image";
  WaitForMessageTestHelper<ignition::msgs::Image> helper(topic);

  std::string infoTopic = topicBase + "camera_info";
  WaitForMessageTestHelper<ignition::msgs::CameraInfo> infoHelper(infoTopic);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper.WaitForMessage()) << helper;
  EXPECT_TRUE(infoHelper.WaitForMessage()) << infoHelper;

  // subscribe to the thermal camera topic
  ignition::transport::Node node;
  node.Subscribe(topic, &OnImage8Bit);

  // subscribe to the thermal camera info topic
  node.Subscribe(infoTopic, &OnCameraInfo);

  // wait for a few thermal camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  int midWidth = static_cast<int>(thermalSensor->ImageWidth() * 0.5);
  int midHeight = static_cast<int>(thermalSensor->ImageHeight() * 0.5);
  int mid = midHeight * thermalSensor->ImageWidth() + midWidth -1;
  int left = midHeight * thermalSensor->ImageWidth();
  int right = (midHeight+1) * thermalSensor->ImageWidth() - 1;

  auto waitTime = std::chrono::duration_cast< std::chrono::milliseconds >(
      std::chrono::duration< double >(0.001));
  int counter = 0;
  int infoCounter = 0;
  ignition::msgs::CameraInfo infoMsg;
  for (int sleep = 0;
       sleep < 300 && (counter == 0 || infoCounter == 0); ++sleep)
  {
    g_mutex.lock();
    counter = g_thermalCounter;
    g_mutex.unlock();

    g_infoMutex.lock();
    infoCounter = g_infoCounter;
    infoMsg = g_infoMsg;
    g_infoMutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }
  g_mutex.lock();
  g_infoMutex.lock();
  g_thermalCounter = 0;
  g_infoCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_EQ(counter, infoCounter);
  counter = 0;
  infoCounter = 0;

  // verify temperature
  // Box should be in the middle of image and return box temp
  // Left and right side of the image frame should be ambient temp
  EXPECT_NEAR(ambientTemp, g_thermalBuffer8Bit[left] * linearResolution,
      ambientTempRange);
  EXPECT_NEAR(ambientTemp, g_thermalBuffer8Bit[right] * linearResolution,
      ambientTempRange);
  EXPECT_FLOAT_EQ(g_thermalBuffer8Bit[right], g_thermalBuffer8Bit[left]);

  EXPECT_NEAR(boxTemp, g_thermalBuffer8Bit[mid] * linearResolution,
      linearResolution);

  g_infoMutex.unlock();
  g_mutex.unlock();

  // Check camera info
  EXPECT_TRUE(infoMsg.has_header());
  ASSERT_EQ(1, infoMsg.header().data().size());
  EXPECT_EQ("frame_id", infoMsg.header().data(0).key());
  ASSERT_EQ(1, infoMsg.header().data(0).value().size());
  EXPECT_EQ("camera1", infoMsg.header().data(0).value(0));
  EXPECT_TRUE(infoMsg.has_distortion());
  EXPECT_EQ(ignition::msgs::CameraInfo::Distortion::PLUMB_BOB,
      infoMsg.distortion().model());
  EXPECT_EQ(5, infoMsg.distortion().k().size());
  EXPECT_TRUE(infoMsg.has_intrinsics());
  EXPECT_EQ(9, infoMsg.intrinsics().k().size());
  EXPECT_TRUE(infoMsg.has_projection());
  EXPECT_EQ(12, infoMsg.projection().p().size());
  EXPECT_EQ(9, infoMsg.rectification_matrix().size());

  // Check that for a box really close it returns box temperature
  root->RemoveChild(box);
  ignition::math::Vector3d boxPositionNear(
      unitBoxSize * 0.5 + near_ * 0.5, 0.0, 0.0);
  box->SetLocalPosition(boxPositionNear);
  root->AddChild(box);

  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  for (int sleep = 0;
       sleep < 300 && (counter == 0 || infoCounter == 0); ++sleep)
  {
    g_mutex.lock();
    counter = g_thermalCounter;
    g_mutex.unlock();

    g_infoMutex.lock();
    infoCounter = g_infoCounter;
    g_infoMutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }

  g_mutex.lock();
  g_infoMutex.lock();
  g_thermalCounter = 0;
  g_infoCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_EQ(counter, infoCounter);
  counter = 0;
  infoCounter = 0;

  EXPECT_NEAR(boxTemp, g_thermalBuffer8Bit[mid] * linearResolution,
      linearResolution);
  g_infoMutex.unlock();
  g_mutex.unlock();

  // Check that for a box really far it returns ambient temperature
  root->RemoveChild(box);
  ignition::math::Vector3d boxPositionFar(
      unitBoxSize * 0.5 + far_ * 1.5, 0.0, 0.0);
  box->SetLocalPosition(boxPositionFar);
  root->AddChild(box);

  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  for (int sleep = 0;
       sleep < 300 && (counter == 0 || infoCounter == 0); ++sleep)
  {
    g_mutex.lock();
    counter = g_thermalCounter;
    g_mutex.unlock();

    g_infoMutex.lock();
    infoCounter = g_infoCounter;
    g_infoMutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }
  g_mutex.lock();
  g_infoMutex.lock();
  g_thermalCounter = 0;
  g_infoCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_EQ(counter, infoCounter);
  counter = 0;
  infoCounter = 0;

  EXPECT_NEAR(ambientTemp, g_thermalBuffer8Bit[mid] * linearResolution,
      ambientTempRange);
  g_infoMutex.unlock();
  g_mutex.unlock();

  delete [] g_thermalBuffer8Bit;
  g_thermalBuffer8Bit = nullptr;

  // Clean up
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(ThermalCameraSensorTest, Images8BitWithBuiltinSDF)
{
  Images8BitWithBuiltinSDF(GetParam());
}

INSTANTIATE_TEST_CASE_P(ThermalCameraSensor, ThermalCameraSensorTest,
    RENDER_ENGINE_VALUES, ignition::rendering::PrintToStringParam());

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ignition::common::Console::SetVerbosity(4);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
