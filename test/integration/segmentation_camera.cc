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

/// \brief mutex for thread safety
std::mutex g_mutex;

/// \brief Segmentation buffer
uint8_t *g_buffer = nullptr;

/// \brief counter of received segmentation msgs
unsigned int g_counter = 0;

/// \brief Label of the boxes in the scene
const uint8_t hiddenLabel = 4;
const uint8_t leftBoxLabel = 1;
const uint8_t rightBoxLabel = 1;
const uint8_t middleBoxLabel = 2;

/// \brief callback to get the segmentation buffer
void OnNewSegmentationFrame(const msgs::Image &_msg)
{
  g_mutex.lock();

  // the image has 3 channels
  unsigned int size = _msg.width() * _msg.height() * 3;

  if (!g_buffer)
    g_buffer = new uint8_t[size];

  memcpy(g_buffer, _msg.data().c_str(), size);
  g_counter++;
  g_mutex.unlock();
}

/// \brief wait till you read the published frame
void WaitForNewFrame(ignition::sensors::Manager &_mgr)
{
  g_counter = 0;

  // wait for a few segmentation camera frames
  _mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  auto waitTime = std::chrono::duration_cast< std::chrono::milliseconds >(
    std::chrono::duration< double >(0.001));

  unsigned int counter = 0;
  // wait for the counter to increase
  for (int sleep = 0; sleep < 300 && counter == 0; ++sleep)
  {
    g_mutex.lock();
    counter = g_counter;
    g_mutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }
  EXPECT_GT(counter, 0u);
  ASSERT_NE(nullptr, g_buffer);
}

/// \brief Build the scene with 3 boxes besides each other
/// the 2 outer boxes have the same label & the middle is different.
/// There is also another box with a unique label that is hidden
/// behind the middle box
void BuildScene(rendering::ScenePtr _scene)
{
  const math::Vector3d leftPosition(3, 1.5, 0);
  const math::Vector3d rightPosition(3, -1.5, 0);
  const math::Vector3d middlePosition(3, 0, 0);
  const math::Vector3d hiddenPosition(8, 0, 0);

  rendering::VisualPtr root = _scene->RootVisual();

  const double unitBoxSize = 1.0;

  // create box visual
  rendering::VisualPtr box = _scene->CreateVisual("box_left");
  ASSERT_NE(nullptr, box);
  box->AddGeometry(_scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(leftPosition);
  box->SetLocalRotation(0, 0, 0);
  box->SetUserData("label", leftBoxLabel);
  box->SetLocalScale(unitBoxSize, unitBoxSize, unitBoxSize);
  root->AddChild(box);

  // create box visual of same label
  rendering::VisualPtr box1 = _scene->CreateVisual("box_right");
  ASSERT_NE(nullptr, box1);
  box1->AddGeometry(_scene->CreateBox());
  box1->SetOrigin(0.0, 0.0, 0.0);
  box1->SetLocalPosition(rightPosition);
  box1->SetLocalRotation(0, 0, 0);
  box1->SetUserData("label", rightBoxLabel);
  box1->SetLocalScale(unitBoxSize, unitBoxSize, unitBoxSize);
  root->AddChild(box1);

  // create box visual of different label
  ignition::rendering::VisualPtr box2 = _scene->CreateVisual("box_mid");
  ASSERT_NE(nullptr, box2);
  box2->AddGeometry(_scene->CreateBox());
  box2->SetOrigin(0.0, 0.0, 0.0);
  box2->SetLocalPosition(middlePosition);
  box2->SetLocalRotation(0, 0, 0);
  box2->SetUserData("label", middleBoxLabel);
  box2->SetLocalScale(unitBoxSize, unitBoxSize, unitBoxSize);
  root->AddChild(box2);

  // create box visual of the hidden box
  ignition::rendering::VisualPtr hiddenBox = _scene->CreateVisual("box_hidden");
  ASSERT_NE(nullptr, hiddenBox);
  hiddenBox->AddGeometry(_scene->CreateBox());
  hiddenBox->SetOrigin(0.0, 0.0, 0.0);
  hiddenBox->SetLocalPosition(hiddenPosition);
  hiddenBox->SetLocalRotation(0, 0, 0);
  hiddenBox->SetUserData("label", hiddenLabel);
  hiddenBox->SetLocalScale(unitBoxSize, unitBoxSize, unitBoxSize);
  root->AddChild(hiddenBox);
}

/////////////////////////////////////////////////
class SegmentationCameraSensorTest: public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Create a Segmentation Camera sensor from a SDF and gets a image message
  public: void ImagesWithBuiltinSDF(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void SegmentationCameraSensorTest::ImagesWithBuiltinSDF(
  const std::string &_renderEngine)
{
  std::string path = ignition::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "segmentation_camera_sensor_builtin.sdf");
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

  // If ogre2 is not the engine, don't run the test
  if (_renderEngine.compare("ogre2") != 0)
  {
    igndbg << "Engine '" << _renderEngine
      << "' doesn't support segmentation cameras" << std::endl;
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
  ASSERT_NE(nullptr, scene);
  BuildScene(scene);

  ignition::sensors::Manager mgr;

  sdf::Sensor sdfSensor;
  sdfSensor.Load(sensorPtr);

  std::string type = sdfSensor.TypeStr();
  EXPECT_EQ(type, "segmentation_camera");

  ignition::sensors::SegmentationCameraSensor *sensor =
    mgr.CreateSensor<ignition::sensors::SegmentationCameraSensor>(sdfSensor);

  ASSERT_NE(sensor, nullptr);
  EXPECT_FALSE(sensor->HasConnections());
  sensor->SetScene(scene);

  EXPECT_EQ(width, static_cast<int>(sensor->ImageWidth()));
  EXPECT_EQ(height, static_cast<int>(sensor->ImageHeight()));

  auto camera = sensor->SegmentationCamera();
  ASSERT_NE(camera, nullptr);

  camera->SetSegmentationType(rendering::SegmentationType::ST_SEMANTIC);
  camera->EnableColoredMap(true);
  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);

  // 23 is a random number that is not used for boxes
  uint32_t backgroundLabel = 23;
  camera->SetBackgroundLabel(backgroundLabel);

  // Topic
  std::string topic =
    "/test/integration/SegmentationCameraPlugin_imagesWithBuiltinSDF";
  std::string infoTopic = topic + "/camera_info";
  // Get the topic of the labels map
  // TODO(anyone) add test coverage for the colored map topic and its data
  topic += "/labels_map";

  WaitForMessageTestHelper<ignition::msgs::Image> helper(topic);
  EXPECT_TRUE(sensor->HasConnections());
  WaitForMessageTestHelper<ignition::msgs::CameraInfo> infoHelper(infoTopic);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  // subscribe to the segmentation camera topic
  ignition::transport::Node node;
  EXPECT_TRUE(node.Subscribe(topic, &OnNewSegmentationFrame));

  // wait for a new frame
  WaitForNewFrame(mgr);

  // get the center of each box, the percentages locates the center
  math::Vector2d leftProj(width * 0.25, height * 0.5);
  math::Vector2d rightProj(width * 0.75, height * 0.5);
  math::Vector2d middleProj(width * 0.5, height * 0.5);

  // get their index in the buffer
  const uint32_t channels = 3;
  uint32_t leftIndex =
    static_cast<uint32_t>((leftProj.Y() * width + leftProj.X()) * channels);
  uint32_t rightIndex =
    static_cast<uint32_t>((rightProj.Y() * width + rightProj.X()) * channels);
  uint32_t middleIndex =
    static_cast<uint32_t>((middleProj.Y() * width + middleProj.X()) * channels);

  uint8_t leftLabel =   g_buffer[leftIndex];
  uint8_t rightLabel =  g_buffer[rightIndex];
  uint8_t middleLabel = g_buffer[middleIndex];

  // check the label
  EXPECT_EQ(leftLabel, leftBoxLabel);
  EXPECT_EQ(middleLabel, middleBoxLabel);
  EXPECT_EQ(rightLabel, rightBoxLabel);

  // check a pixel between 2 boxes & a pixel below a box
  uint32_t betweenBoxesIndex = (120 * width + 230) * channels;
  uint32_t belowBoxesIndex = (200 * width + 280) * channels;
  uint8_t betweenBoxes = g_buffer[betweenBoxesIndex];
  uint8_t belowBoxes = g_buffer[belowBoxesIndex];
  // check if the first pixel(background) = the background label
  uint8_t background = g_buffer[0];

  EXPECT_EQ(background, backgroundLabel);
  EXPECT_EQ(betweenBoxes, backgroundLabel);
  EXPECT_EQ(belowBoxes, backgroundLabel);

  // search for the hidden box label in all pixels
  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      uint32_t index = (i * width + j) * channels;
      uint8_t label = g_buffer[index];
      EXPECT_NE(label, hiddenLabel);
    }
  }

  // Instance/Panoptic Segmentation Test
  camera->SetSegmentationType(rendering::SegmentationType::ST_PANOPTIC);

  // wait for a new frame
  WaitForNewFrame(mgr);

  const int labelOffset = 2;
  leftLabel =   g_buffer[leftIndex   + labelOffset];
  rightLabel =  g_buffer[rightIndex  + labelOffset];
  middleLabel = g_buffer[middleIndex + labelOffset];

  // the instance count is in the first channel
  uint8_t leftCount =   g_buffer[leftIndex];
  uint8_t rightCount =  g_buffer[rightIndex];
  uint8_t middleCount = g_buffer[middleIndex];

  // check the label
  EXPECT_EQ(leftLabel, leftBoxLabel);
  EXPECT_EQ(middleLabel, middleBoxLabel);
  EXPECT_EQ(rightLabel, rightBoxLabel);

  // instance count
  EXPECT_EQ(middleCount, 1);
  EXPECT_EQ(rightCount, 1);
  EXPECT_EQ(leftCount, 2);

  // search for the hidden box label in all pixels
  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      uint32_t index = (i * width + j) * channels;
      uint8_t label = g_buffer[index];
      EXPECT_NE(label, hiddenLabel);
    }
  }

  // Clean up rendering ptrs
  camera.reset();

  // Clean up
  mgr.Remove(sensor->Id());
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
