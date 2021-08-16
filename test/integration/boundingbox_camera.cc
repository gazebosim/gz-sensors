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
#include <ignition/sensors/BoundingBoxCameraSensor.hh>

#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/BoundingBoxCamera.hh>
#include <ignition/msgs.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "TransportTestTools.hh"

using namespace ignition;
using namespace rendering;

class BoundingBoxCameraSensorTest: public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Create a BoundingBox Camera sensor from a SDF and gets a boxes message
  public: void BoxesWithBuiltinSDF(const std::string &_renderEngine);

  public: void Boxes3DWithBuiltinSDF(const std::string &renderEngine);
};

/// \brief mutex for thread safety
std::mutex g_mutex;

/// \brief bounding boxes from the camera
std::vector<BoundingBox> g_boxes;

/// \brief counter of received boundingboxes msg
int g_counter = 0;

/// \brief callback to receive 2d boxes from the camera
void OnNewBoundingBoxes(const msgs::AnnotatedAxisAligned2DBox_V &boxes)
{
  g_mutex.lock();
  g_boxes.clear();

  int size = boxes.annotated_box_size();
  for (int i = 0; i < size; i++)
  {
    BoundingBox box(BoundingBoxType::VisibleBox2D);
    auto annotated_box = boxes.annotated_box(i);
    auto axisAlignedBox = annotated_box.box();
    auto min_corner = axisAlignedBox.min_corner();
    auto max_corner = axisAlignedBox.max_corner();

    box.size.X(max_corner.x() - min_corner.x());
    box.size.Y(max_corner.y() - min_corner.y());
    box.size.Z(0);

    box.center.X(min_corner.x() + box.size.X() / 2);
    box.center.Y(min_corner.y() + box.size.Y() / 2);
    box.center.Z(0);

    box.orientation.Set(0, 0, 0, 0);
    box.label = annotated_box.label();
    g_boxes.push_back(box);
  }
  g_counter++;
  g_mutex.unlock();
}

/// \brief callback to receive 3D boxes from the camera
void OnNew3DBoundingBoxes(const msgs::AnnotatedOriented3DBox_V &boxes)
{
  g_mutex.lock();
  g_boxes.clear();

  int size = boxes.annotated_box_size();
  for (int i = 0; i < size; i++)
  {
    BoundingBox box(BoundingBoxType::Box3D);
    auto annotated_box = boxes.annotated_box(i);
    auto orientedBox = annotated_box.box();
    // center
    box.center.Set(orientedBox.center().x(), orientedBox.center().y(),
      orientedBox.center().z());
    // orientation
    auto orientation = orientedBox.orientation();
    box.orientation.Set(orientation.w(), orientation.x(),
      orientation.y(), orientation.z());
    // size
    auto boxSize = orientedBox.boxsize();
    box.size.Set(boxSize.x(), boxSize.y(), boxSize.z());

    box.label = annotated_box.label();
    g_boxes.push_back(box);
  }
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

/// \brief Build a scene with 3 boxes 2 overlapping boxes which 1
/// is behind the other, and the 3rd box is invisible behind them
void BuildScene(rendering::ScenePtr scene)
{
  math::Vector3d occludedPosition(4, 1, 0);
  math::Vector3d frontPosition(2, 0, 0);
  math::Vector3d invisiblePosition(5, 0, 0);

  rendering::VisualPtr root = scene->RootVisual();

  // create front box visual (the smaller box)
  rendering::VisualPtr occludedBox = scene->CreateVisual();
  occludedBox->AddGeometry(scene->CreateBox());
  occludedBox->SetOrigin(0.0, 0.0, 0.0);
  occludedBox->SetLocalPosition(occludedPosition);
  occludedBox->SetLocalRotation(0, 0, 0);
  occludedBox->SetUserData("label", 1);
  root->AddChild(occludedBox);

  // create occluded box visual (the bigger box)
  rendering::VisualPtr frontBox = scene->CreateVisual();
  frontBox->AddGeometry(scene->CreateBox());
  frontBox->SetOrigin(0.0, 0.0, 0.0);
  frontBox->SetLocalPosition(frontPosition);
  frontBox->SetLocalRotation(0, 0, 0);
  frontBox->SetUserData("label", 2);
  root->AddChild(frontBox);

  rendering::VisualPtr invisibleBox = scene->CreateVisual();
  invisibleBox->AddGeometry(scene->CreateBox());
  invisibleBox->SetOrigin(0.0, 0.0, 0.0);
  invisibleBox->SetLocalPosition(invisiblePosition);
  invisibleBox->SetLocalRotation(0, 0, 0);
  invisibleBox->SetUserData("label", 2);
  root->AddChild(invisibleBox);
}

/// \brief Build a scene with 3d oriented box
void BuildScene3D(rendering::ScenePtr scene)
{
  rendering::VisualPtr root = scene->RootVisual();

  // create front box visual (the smaller box)
  rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(math::Vector3d(2, 0, 0));
  box->SetLocalRotation(math::Quaternion(1.0, 0.5, 0.5, 0.2));
  box->SetUserData("label", 1);
  root->AddChild(box);
}

/////////////////////////////////////////////////
void BoundingBoxCameraSensorTest::BoxesWithBuiltinSDF(
  const std::string &_renderEngine)
{
  std::string path = ignition::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "boundingbox_camera_sensor_builtin.sdf");
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

  unsigned int width = imagePtr->Get<int>("width");
  unsigned int height = imagePtr->Get<int>("height");

  EXPECT_EQ(width, 320u);
  EXPECT_EQ(height, 240u);

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
  EXPECT_EQ(type, "boundingbox_camera");

  ignition::sensors::BoundingBoxCameraSensor *sensor =
    mgr.CreateSensor<ignition::sensors::BoundingBoxCameraSensor>(sdfSensor);

  EXPECT_NE(sensor, nullptr);
  sensor->SetScene(scene);

  EXPECT_EQ(width, sensor->ImageWidth());
  EXPECT_EQ(height, sensor->ImageHeight());

  auto camera = sensor->BoundingBoxCamera();
  EXPECT_NE(camera, nullptr);

  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  camera->SetBoundingBoxType(BoundingBoxType::VisibleBox2D);

  EXPECT_EQ(camera->Type(), BoundingBoxType::VisibleBox2D);
  EXPECT_EQ(camera->ImageWidth(), width);
  EXPECT_EQ(camera->ImageHeight(), height);

  // Get the Msg
  std::string topic =
    "/test/integration/BoundingBoxCameraPlugin_boxesWithBuiltinSDF";
  WaitForMessageTestHelper<
    ignition::msgs::AnnotatedAxisAligned2DBox_V> helper(topic);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  // subscribe to the BoundingBox camera topic
  ignition::transport::Node node;
  node.Subscribe(topic, &OnNewBoundingBoxes);

  // wait for a few BoundingBox camera boxes
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  // wait for a new boxes
  WaitForNewFrame();

  // accepted error with +/- in pixels in comparing the box coordinates
  double margin_error = 1;

  // Visible box test
  g_mutex.lock();
  g_counter = 0;

  // check if the invisible 3rd box is not exist
  EXPECT_EQ(g_boxes.size(), size_t(2));

  BoundingBox occludedBox = g_boxes[0];
  BoundingBox frontBox = g_boxes[1];

  unsigned int occludedLabel = 1;
  unsigned int frontLabel = 2;

  // hard-coded comparasion with acceptable error
  EXPECT_NEAR(occludedBox.center.X(), 98, margin_error);
  EXPECT_NEAR(occludedBox.center.Y(), 119, margin_error);
  EXPECT_NEAR(occludedBox.size.X(), 15, margin_error);
  EXPECT_NEAR(occludedBox.size.Y(), 45, margin_error);
  EXPECT_EQ(occludedBox.label, occludedLabel);

  EXPECT_NEAR(frontBox.center.X(), 159, margin_error);
  EXPECT_NEAR(frontBox.center.Y(), 119, margin_error);
  EXPECT_NEAR(frontBox.size.X(), 105, margin_error);
  EXPECT_NEAR(frontBox.size.Y(), 105, margin_error);
  EXPECT_EQ(frontBox.label, frontLabel);

  g_mutex.unlock();

  // Full Boxes Type Test
  camera->SetBoundingBoxType(BoundingBoxType::FullBox2D);

  // wait for bounding boxes
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  WaitForNewFrame();

  // test
  g_mutex.lock();
  g_counter = 0;

  // check the hidden box
  EXPECT_EQ(g_boxes.size(), size_t(2));
  BoundingBox occludedFullBox = g_boxes[0];
  BoundingBox frontFullBox = g_boxes[1];

  // coordinates of partially occluded object is bigger
  EXPECT_NEAR(occludedFullBox.center.X(), 116, margin_error);
  EXPECT_NEAR(occludedFullBox.center.Y(), 119, margin_error);
  EXPECT_NEAR(occludedFullBox.size.X(), 51, margin_error);
  EXPECT_NEAR(occludedFullBox.size.Y(), 45, margin_error);
  EXPECT_EQ(occludedFullBox.label, occludedLabel);

  EXPECT_NEAR(frontFullBox.center.X(), 159, margin_error);
  EXPECT_NEAR(frontFullBox.center.Y(), 119, margin_error);
  EXPECT_NEAR(frontFullBox.size.X(), 108, margin_error);
  EXPECT_NEAR(frontFullBox.size.Y(), 108, margin_error);
  EXPECT_EQ(frontFullBox.label, frontLabel);

  g_mutex.unlock();

  // Clean up
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void BoundingBoxCameraSensorTest::Boxes3DWithBuiltinSDF(
  const std::string &_renderEngine)
{
  std::string path = ignition::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "boundingbox_3d_camera_sensor_builtin.sdf");
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

  unsigned int width = imagePtr->Get<int>("width");
  unsigned int height = imagePtr->Get<int>("height");

  EXPECT_EQ(width, 320u);
  EXPECT_EQ(height, 240u);

  // Setup ign-rendering with an empty scene
  auto *engine = ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  BuildScene3D(scene);

  ignition::sensors::Manager mgr;

  sdf::Sensor sdfSensor;
  sdfSensor.Load(sensorPtr);

  std::string type = sdfSensor.TypeStr();
  EXPECT_EQ(type, "boundingbox_camera");

  ignition::sensors::BoundingBoxCameraSensor *sensor =
    mgr.CreateSensor<ignition::sensors::BoundingBoxCameraSensor>(sdfSensor);

  EXPECT_NE(sensor, nullptr);
  sensor->SetScene(scene);

  EXPECT_EQ(width, sensor->ImageWidth());
  EXPECT_EQ(height, sensor->ImageHeight());

  auto camera = sensor->BoundingBoxCamera();
  EXPECT_NE(camera, nullptr);

  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  camera->SetBoundingBoxType(BoundingBoxType::Box3D);

  // Get the Msg
  std::string topic =
    "/test/integration/BoundingBox3DCameraPlugin_boxesWithBuiltinSDF";
  WaitForMessageTestHelper<
    ignition::msgs::AnnotatedOriented3DBox_V> helper(topic);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  // subscribe to the BoundingBox camera topic
  ignition::transport::Node node;
  node.Subscribe(topic, &OnNew3DBoundingBoxes);

  // wait for a few BoundingBox camera boxes
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  // wait for a new boxes
  WaitForNewFrame();

  g_mutex.lock();
  g_counter = 0;

  // check the hidden box
  EXPECT_EQ(g_boxes.size(), size_t(1));
  BoundingBox box3D = g_boxes[0];

  double margin_error = 0.01;
  EXPECT_NEAR(box3D.center.X(), 0, margin_error);
  EXPECT_NEAR(box3D.center.Y(), 0, margin_error);
  EXPECT_NEAR(box3D.center.Z(), -2, margin_error);

  EXPECT_NEAR(box3D.size.X(), 1, margin_error);
  EXPECT_NEAR(box3D.size.Y(), 1, margin_error);
  EXPECT_NEAR(box3D.size.Z(), 1, margin_error);

  EXPECT_NEAR(box3D.orientation.X(), 0.322329, margin_error);
  EXPECT_NEAR(box3D.orientation.Y(), -0.886405, margin_error);
  EXPECT_NEAR(box3D.orientation.Z(), -0.0805823, margin_error);
  EXPECT_NEAR(box3D.orientation.W(), -0.322329, margin_error);

  g_mutex.unlock();

  // Clean up
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(BoundingBoxCameraSensorTest, BoxesWithBuiltinSDF)
{
  BoxesWithBuiltinSDF(GetParam());
}

//////////////////////////////////////////////////
TEST_P(BoundingBoxCameraSensorTest, Boxes3DWithBuiltinSDF)
{
  Boxes3DWithBuiltinSDF(GetParam());
}

INSTANTIATE_TEST_CASE_P(BoundingBoxCameraSensor, BoundingBoxCameraSensorTest,
    RENDER_ENGINE_VALUES, ignition::rendering::PrintToStringParam());

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ignition::common::Console::SetVerbosity(4);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
