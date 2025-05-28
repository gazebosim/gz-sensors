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

#include <ignition/msgs/annotated_axis_aligned_2d_box.pb.h>
#include <ignition/msgs/annotated_oriented_3d_box.pb.h>

#include <ignition/common/Filesystem.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/sensors/BoundingBoxCameraSensor.hh>

#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/BoundingBoxCamera.hh>

#include <gz/utils/ExtraTestMacros.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "TransportTestTools.hh"

using namespace ignition;

class BoundingBoxCameraSensorTest: public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Create a BoundingBox Camera sensor from a SDF and gets a boxes message
  public: void BoxesWithBuiltinSDF(const std::string &_renderEngine);

  // Create a BoundingBox Camera 3D sensor from a SDF and gets a boxes message
  public: void Boxes3DWithBuiltinSDF(const std::string &_renderEngine);
};

/// \brief mutex for thread safety
std::mutex g_mutex;

/// \brief bounding boxes from the camera
std::vector<rendering::BoundingBox> g_boxes;

/// \brief counter of received boundingboxes msg
int g_counter = 0;

/// \brief callback to receive 2d boxes from the camera
void OnNewBoundingBoxes(const msgs::AnnotatedAxisAligned2DBox_V &_boxes)
{
  g_mutex.lock();
  g_boxes.clear();

  int size = _boxes.annotated_box_size();
  for (int i = 0; i < size; ++i)
  {
    rendering::BoundingBox box;
    auto annotated_box = _boxes.annotated_box(i);
    auto axisAlignedBox = annotated_box.box();
    auto min_corner = axisAlignedBox.min_corner();
    auto max_corner = axisAlignedBox.max_corner();

    box.SetSize({max_corner.x() - min_corner.x(),
        max_corner.y() - min_corner.y(),
        0});

    box.SetCenter({min_corner.x() + box.Size().X() / 2,
        min_corner.y() + box.Size().Y() / 2,
        0});

    box.SetOrientation({0, 0, 0});
    box.SetLabel(annotated_box.label());
    g_boxes.push_back(box);
  }
  g_counter++;
  g_mutex.unlock();
}

/// \brief callback to receive 3D boxes from the camera
void OnNew3DBoundingBoxes(const msgs::AnnotatedOriented3DBox_V &_boxes)
{
  g_mutex.lock();
  g_boxes.clear();

  int size = _boxes.annotated_box_size();
  for (int i = 0; i < size; ++i)
  {
    rendering::BoundingBox box;
    auto annotated_box = _boxes.annotated_box(i);
    auto orientedBox = annotated_box.box();
    // center
    box.SetCenter({orientedBox.center().x(), orientedBox.center().y(),
      orientedBox.center().z()});
    // orientation
    auto orientation = orientedBox.orientation();
    box.SetOrientation({orientation.w(), orientation.x(),
      orientation.y(), orientation.z()});
    // size
    auto boxSize = orientedBox.boxsize();
    box.SetSize({boxSize.x(), boxSize.y(), boxSize.z()});

    box.SetLabel(annotated_box.label());
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

/// \brief Build a scene with 3 overlapping boxes - one box will be fully
/// visible, another box will be partially visible, and the last box won't be
/// visible (it's occluded by the other 2 boxes)
void BuildScene2d(rendering::ScenePtr _scene)
{
  math::Vector3d occludedPosition(4, 1, 0);
  math::Vector3d frontPosition(2, 0, 0);
  math::Vector3d invisiblePosition(5, 0, 0);

  rendering::VisualPtr root = _scene->RootVisual();

  // create front box visual (the smaller box)
  rendering::VisualPtr occludedBox = _scene->CreateVisual();
  occludedBox->AddGeometry(_scene->CreateBox());
  occludedBox->SetOrigin(0.0, 0.0, 0.0);
  occludedBox->SetLocalPosition(occludedPosition);
  occludedBox->SetLocalRotation(0, 0, 0);
  occludedBox->SetUserData("label", 1);
  root->AddChild(occludedBox);

  // create occluded box visual (the bigger box)
  rendering::VisualPtr frontBox = _scene->CreateVisual();
  frontBox->AddGeometry(_scene->CreateBox());
  frontBox->SetOrigin(0.0, 0.0, 0.0);
  frontBox->SetLocalPosition(frontPosition);
  frontBox->SetLocalRotation(0, 0, 0);
  frontBox->SetUserData("label", 2);
  root->AddChild(frontBox);

  rendering::VisualPtr invisibleBox = _scene->CreateVisual();
  invisibleBox->AddGeometry(_scene->CreateBox());
  invisibleBox->SetOrigin(0.0, 0.0, 0.0);
  invisibleBox->SetLocalPosition(invisiblePosition);
  invisibleBox->SetLocalRotation(0, 0, 0);
  invisibleBox->SetUserData("label", 2);
  root->AddChild(invisibleBox);
}

/// \brief Build a scene with 3d oriented box
void BuildScene3D(rendering::ScenePtr _scene)
{
  rendering::VisualPtr root = _scene->RootVisual();

  // create front box visual (the smaller box)
  rendering::VisualPtr box = _scene->CreateVisual();
  box->AddGeometry(_scene->CreateBox());
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
  std::string path = common::joinPaths(PROJECT_SOURCE_PATH, "test",
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

  // Skip unsupported engines
  if (_renderEngine != "ogre2")
  {
    igndbg << "Engine '" << _renderEngine
              << "' doesn't support bounding box cameras" << std::endl;
    return;
  }

  // Setup ign-rendering with an empty scene
  auto *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  rendering::ScenePtr scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);
  BuildScene2d(scene);

  sensors::Manager mgr;

  sdf::Sensor sdfSensor;
  sdfSensor.Load(sensorPtr);

  std::string type = sdfSensor.TypeStr();
  EXPECT_EQ(type, "boundingbox_camera");

  auto *sensor =
    mgr.CreateSensor<sensors::BoundingBoxCameraSensor>(sdfSensor);

  ASSERT_NE(sensor, nullptr);
  sensor->SetScene(scene);

  EXPECT_EQ(width, sensor->ImageWidth());
  EXPECT_EQ(height, sensor->ImageHeight());

  auto camera = sensor->BoundingBoxCamera();
  ASSERT_NE(camera, nullptr);

  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  camera->SetBoundingBoxType(rendering::BoundingBoxType::BBT_VISIBLEBOX2D);

  EXPECT_EQ(camera->Type(), rendering::BoundingBoxType::BBT_VISIBLEBOX2D);
  EXPECT_EQ(camera->ImageWidth(), width);
  EXPECT_EQ(camera->ImageHeight(), height);

  // Get the Msg
  std::string topic =
    "/test/integration/BoundingBoxCameraPlugin_boxesWithBuiltinSDF";
  WaitForMessageTestHelper<
    msgs::AnnotatedAxisAligned2DBox_V> helper(topic);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  // subscribe to the BoundingBox camera topic
  transport::Node node;
  node.Subscribe(topic, &OnNewBoundingBoxes);

  // wait for a few BoundingBox camera boxes
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  // wait for a new box
  WaitForNewFrame();

  // accepted error with +/- in pixels in comparing the box coordinates
  double marginError = 2.0;

  // Visible box test
  g_mutex.lock();
  g_counter = 0;

  // check that the invisible 3rd box does not exist
  EXPECT_EQ(g_boxes.size(), size_t(2));

  rendering::BoundingBox occludedBox = g_boxes[0];
  rendering::BoundingBox frontBox = g_boxes[1];

  unsigned int occludedLabel = 1;
  unsigned int frontLabel = 2;

  // hard-coded comparasion with acceptable error
  EXPECT_NEAR(occludedBox.Center().X(), 98, marginError);
  EXPECT_NEAR(occludedBox.Center().Y(), 119, marginError);
  EXPECT_NEAR(occludedBox.Size().X(), 15, marginError);
  EXPECT_NEAR(occludedBox.Size().Y(), 45, marginError);
  EXPECT_EQ(occludedBox.Label(), occludedLabel);

  EXPECT_NEAR(frontBox.Center().X(), 159, marginError);
  EXPECT_NEAR(frontBox.Center().Y(), 119, marginError);
  EXPECT_NEAR(frontBox.Size().X(), 105, marginError);
  EXPECT_NEAR(frontBox.Size().Y(), 105, marginError);
  EXPECT_EQ(frontBox.Label(), frontLabel);

  g_mutex.unlock();

  // Full Boxes Type Test
  camera->SetBoundingBoxType(rendering::BoundingBoxType::BBT_FULLBOX2D);

  // wait for bounding boxes
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  WaitForNewFrame();

  // test
  g_mutex.lock();
  g_counter = 0;

  // check the hidden box
  EXPECT_EQ(g_boxes.size(), size_t(2));
  rendering::BoundingBox occludedFullBox = g_boxes[0];
  rendering::BoundingBox frontFullBox = g_boxes[1];

  // coordinates of partially occluded object is bigger
  EXPECT_NEAR(occludedFullBox.Center().X(), 116, marginError);
  EXPECT_NEAR(occludedFullBox.Center().Y(), 119, marginError);
  EXPECT_NEAR(occludedFullBox.Size().X(), 51, marginError);
  EXPECT_NEAR(occludedFullBox.Size().Y(), 45, marginError);
  EXPECT_EQ(occludedFullBox.Label(), occludedLabel);

  EXPECT_NEAR(frontFullBox.Center().X(), 159, marginError);
  EXPECT_NEAR(frontFullBox.Center().Y(), 119, marginError);
  EXPECT_NEAR(frontFullBox.Size().X(), 108, marginError);
  EXPECT_NEAR(frontFullBox.Size().Y(), 108, marginError);
  EXPECT_EQ(frontFullBox.Label(), frontLabel);

  g_mutex.unlock();

  // Clean up rendering ptrs
  camera.reset();

  // Clean up
  mgr.Remove(sensor->Id());
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void BoundingBoxCameraSensorTest::Boxes3DWithBuiltinSDF(
  const std::string &_renderEngine)
{
  std::string path = common::joinPaths(PROJECT_SOURCE_PATH, "test",
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

  // Skip unsupported engines
  if (_renderEngine != "ogre2")
  {
    igndbg << "Engine '" << _renderEngine
              << "' doesn't support bounding box cameras" << std::endl;
    return;
  }

  // Setup ign-rendering with an empty scene
  auto *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  rendering::ScenePtr scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);
  BuildScene3D(scene);

  sensors::Manager mgr;

  sdf::Sensor sdfSensor;
  sdfSensor.Load(sensorPtr);

  std::string type = sdfSensor.TypeStr();
  EXPECT_EQ(type, "boundingbox_camera");

  auto *sensor =
    mgr.CreateSensor<sensors::BoundingBoxCameraSensor>(sdfSensor);

  ASSERT_NE(sensor, nullptr);
  sensor->SetScene(scene);

  EXPECT_EQ(width, sensor->ImageWidth());
  EXPECT_EQ(height, sensor->ImageHeight());

  auto camera = sensor->BoundingBoxCamera();
  ASSERT_NE(camera, nullptr);

  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  camera->SetBoundingBoxType(rendering::BoundingBoxType::BBT_BOX3D);

  // Get the Msg
  std::string topic =
    "/test/integration/BoundingBox3DCameraPlugin_boxesWithBuiltinSDF";
  WaitForMessageTestHelper<
    msgs::AnnotatedOriented3DBox_V> helper(topic);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  // subscribe to the BoundingBox camera topic
  transport::Node node;
  node.Subscribe(topic, &OnNew3DBoundingBoxes);

  // wait for a few BoundingBox camera boxes
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  // wait for a new boxes
  WaitForNewFrame();

  g_mutex.lock();
  g_counter = 0;

  EXPECT_EQ(g_boxes.size(), size_t(1));
  rendering::BoundingBox box3D = g_boxes[0];

  double marginError = 0.01;
  EXPECT_NEAR(box3D.Center().X(), 0, marginError);
  EXPECT_NEAR(box3D.Center().Y(), 0, marginError);
  EXPECT_NEAR(box3D.Center().Z(), -2, marginError);

  EXPECT_NEAR(box3D.Size().X(), 1, marginError);
  EXPECT_NEAR(box3D.Size().Y(), 1, marginError);
  EXPECT_NEAR(box3D.Size().Z(), 1, marginError);

  EXPECT_NEAR(box3D.Orientation().X(), 0.322329, marginError);
  EXPECT_NEAR(box3D.Orientation().Y(), -0.886405, marginError);
  EXPECT_NEAR(box3D.Orientation().Z(), -0.0805823, marginError);
  EXPECT_NEAR(box3D.Orientation().W(), -0.322329, marginError);

  g_mutex.unlock();

  // Clean up rendering ptrs
  camera.reset();

  // Clean up
  mgr.Remove(sensor->Id());
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(BoundingBoxCameraSensorTest,
       IGN_UTILS_TEST_DISABLED_ON_WIN32(BoxesWithBuiltinSDF))
{
  BoxesWithBuiltinSDF(GetParam());
}

//////////////////////////////////////////////////
TEST_P(BoundingBoxCameraSensorTest,
       IGN_UTILS_TEST_DISABLED_ON_WIN32(Boxes3DWithBuiltinSDF))
{
  Boxes3DWithBuiltinSDF(GetParam());
}

INSTANTIATE_TEST_CASE_P(BoundingBoxCameraSensor, BoundingBoxCameraSensorTest,
    RENDER_ENGINE_VALUES, rendering::PrintToStringParam());

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  common::Console::SetVerbosity(4);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
