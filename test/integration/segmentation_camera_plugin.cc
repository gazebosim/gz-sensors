/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/common/Filesystem.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/sensors/SegmentationCameraSensor.hh>

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/SegmentationCamera.hh>
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

using namespace ignition;
using namespace rendering;

class SegmentationCameraSensorTest: public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Create a Segmentation Camera sensor from a SDF and gets a image message
  public: void ImagesWithBuiltinSDF(const std::string &_renderEngine);
};

/// \brief mutex for thread safety
std::mutex g_mutex;

/// \brief Segmentation buffer
unsigned char *g_buffer = nullptr;

/// \brief counter of received segmentation msgs
int g_counter = 0;

/// \brief callback to get the segmentation buffer
void OnNewSegmentationFrame(const msgs::Image &_msg)
{
  g_mutex.lock();
  unsigned int size = _msg.width() * _msg.height() * 3;

  if (!g_buffer)
    g_buffer = new unsigned char[size];

  memcpy(g_buffer, _msg.data().c_str(), size);
  g_counter++;
  g_mutex.unlock();
}

/// \brief wait till you read the published frame
void WaitForNewFrame()
{
  auto waitTime = std::chrono::duration_cast< std::chrono::milliseconds >(
  std::chrono::duration< double >(0.001));
  int counter = 0;
  // wait for the counter to increase
  for (int sleep = 0; sleep < 300 && counter == 0; ++sleep)
  {
    g_mutex.lock();
    counter = g_counter;
    g_mutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }
  EXPECT_GT(counter, 0);
}

/// \brief Build the scene with 3 boxes besides each other
/// the 2 aside boxes have the same label & the middle is different
void BuildScene(rendering::ScenePtr scene)
{
  math::Vector3d leftPosition(3, -1.5, 0);
  math::Vector3d rightPosition(3, 1.5, 0);
  math::Vector3d middlePosition(3, 0, 0);

  rendering::VisualPtr root = scene->RootVisual();

  // create box visual
  rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(leftPosition);
  box->SetLocalRotation(0, 0, 0);
  box->SetUserData("label", 1);
  root->AddChild(box);

  // create box visual of same label
  rendering::VisualPtr box1 = scene->CreateVisual();
  box1->AddGeometry(scene->CreateBox());
  box1->SetOrigin(0.0, 0.0, 0.0);
  box1->SetLocalPosition(rightPosition);
  box1->SetLocalRotation(0, 0, 0);
  box1->SetUserData("label", 1);
  root->AddChild(box1);

  // create box visual of different label
  ignition::rendering::VisualPtr box2 = scene->CreateVisual();
  box2->AddGeometry(scene->CreateBox());
  box2->SetOrigin(0.0, 0.0, 0.0);
  box2->SetLocalPosition(middlePosition);
  box2->SetLocalRotation(0, 0, 0);
  box2->SetUserData("label", 2);
  root->AddChild(box2);
}

/////////////////////////////////////////////////
void SegmentationCameraSensorTest::ImagesWithBuiltinSDF(
  const std::string &_renderEngine)
{
  std::string path = ignition::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "integration", "segmentation_camera_sensor_builtin.sdf");
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

  int width = imagePtr->Get<int>("width");
  int height = imagePtr->Get<int>("height");

  // Setup ign-rendering with an empty scene
  auto *engine = ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  BuildScene(scene);

  ignition::sensors::Manager mgr;

  sdf::Sensor sdfSensor;
  sdfSensor.Load(sensorPtr);

  std::string type = sdfSensor.TypeStr();
  EXPECT_EQ(type, "segmentation_camera");

  ignition::sensors::SegmentationCameraSensor *sensor =
    mgr.CreateSensor<ignition::sensors::SegmentationCameraSensor>(sdfSensor);

  EXPECT_NE(sensor, nullptr);
  sensor->SetScene(scene);

  EXPECT_EQ(width, sensor->ImageWidth());
  EXPECT_EQ(height, sensor->ImageHeight());

  auto camera = sensor->SegmentationCamera();
  EXPECT_NE(camera, nullptr);

  uint32_t backgroundLabel = 23;
  camera->SetSegmentationType(SegmentationType::Semantic);
  camera->EnableColoredMap(false);
  camera->SetBackgroundLabel(backgroundLabel);
  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);

  // Get the Msg
  std::string topic =
    "/test/integration/SegmentationCameraPlugin_imagesWithBuiltinSDF";
  WaitForMessageTestHelper<ignition::msgs::Image> helper(topic);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  // subscribe to the segmentation camera topic
  ignition::transport::Node node;
  node.Subscribe(topic, &OnNewSegmentationFrame);

  // wait for a few segmentation camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // wait for a new frame
  WaitForNewFrame();

  // get the center of each box, the percentages locates the center
  math::Vector2d leftProj(width * 0.25, height * 0.5);
  math::Vector2d rightProj(width * 0.75, height * 0.5);
  math::Vector2d middleProj(width * 0.5, height * 0.5);

  // get their index in the buffer
  uint32_t leftIndex = (leftProj.Y() * width + leftProj.X()) * 3;
  uint32_t rightIndex = (rightProj.Y() * width + rightProj.X()) * 3;
  uint32_t middleIndex = (middleProj.Y() * width + middleProj.X()) * 3;

  // test
  g_mutex.lock();
  g_counter = 0;

  // cast the unsigned char to unsigned int to read it
  unsigned int leftLabel =   g_buffer[leftIndex];
  unsigned int rightLabel =  g_buffer[rightIndex];
  unsigned int middleLabel = g_buffer[middleIndex];

  // check the label
  EXPECT_EQ(leftLabel  , 1);
  EXPECT_EQ(middleLabel, 2);
  EXPECT_EQ(rightLabel , 1);

  // check if the first pixel(background) = the background label
  unsigned int background = g_buffer[0];
  EXPECT_EQ(background, backgroundLabel);

  g_mutex.unlock();

  // Instance/Panoptic Segmentation Test
  camera->SetSegmentationType(SegmentationType::Panoptic);

  // wait for a few segmentation camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  // wait for a new frame
  WaitForNewFrame();

  // the label in the last channel
  leftLabel =   g_buffer[leftIndex   + 2];
  rightLabel =  g_buffer[rightIndex  + 2];
  middleLabel = g_buffer[middleIndex + 2];

  // the instances count in the first channel
  unsigned int leftCount =   g_buffer[leftIndex];
  unsigned int rightCount =  g_buffer[rightIndex];
  unsigned int middleCount = g_buffer[middleIndex];

  // test
  g_mutex.lock();
  g_counter = 0;

  // check the label
  EXPECT_EQ(leftLabel  , 1);
  EXPECT_EQ(middleLabel, 2);
  EXPECT_EQ(rightLabel , 1);

  // instance count
  EXPECT_EQ(middleCount, 1);
  EXPECT_EQ(rightCount, 1);
  EXPECT_EQ(leftCount, 2);

  g_mutex.unlock();

  // Clean up
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}


//////////////////////////////////////////////////
TEST_P(SegmentationCameraSensorTest, ImagesWithBuiltinSDF)
{
  ImagesWithBuiltinSDF(GetParam());
}

INSTANTIATE_TEST_CASE_P(SegmentationCameraSensor, SegmentationCameraSensorTest,
    RENDER_ENGINE_VALUES, ignition::rendering::PrintToStringParam());

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ignition::common::Console::SetVerbosity(4);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
