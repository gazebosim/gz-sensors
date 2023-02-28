/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#include <gz/sensors/BoundingBoxCameraSensor.hh>
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

class TriggeredBoundingBoxCameraTest: public testing::Test,
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

  // Create a BoundingBox Camera sensor from a SDF and gets a boxes message
  public: void BoxesWithBuiltinSDF(const std::string &_renderEngine);

};

/// \brief mutex for thread safety
std::mutex g_mutex;

/// \brief bounding boxes from the camera
std::vector<gz::msgs::AnnotatedAxisAligned2DBox> g_boxes;

/// \brief callback to receive 2d boxes from the camera
void OnNewBoundingBoxes(const gz::msgs::AnnotatedAxisAligned2DBox_V &_boxes)
{
  g_mutex.lock();
  g_boxes.clear();

  int size = _boxes.annotated_box_size();
  for (int i = 0; i < size; ++i)
  {
    auto annotated_box = _boxes.annotated_box(i);
    g_boxes.push_back(annotated_box);
  }
  g_mutex.unlock();
}

/// \brief Build a scene with 2 visible boxes
void BuildScene(gz::rendering::ScenePtr _scene)
{
  gz::math::Vector3d box1Position(1, -1, 0);
  gz::math::Vector3d box2Position(1, 1, 0);

  gz::rendering::VisualPtr root = _scene->RootVisual();

  gz::rendering::VisualPtr box1 = _scene->CreateVisual();
  box1->AddGeometry(_scene->CreateBox());
  box1->SetOrigin(0.0, 0.0, 0.0);
  box1->SetLocalPosition(box1Position);
  box1->SetLocalRotation(0, 0, 0);
  box1->SetUserData("label", 1);
  root->AddChild(box1);

  gz::rendering::VisualPtr box2 = _scene->CreateVisual();
  box2->AddGeometry(_scene->CreateBox());
  box2->SetOrigin(0.0, 0.0, 0.0);
  box2->SetLocalPosition(box2Position);
  box2->SetLocalRotation(0, 0, 0);
  box2->SetUserData("label", 2);
  root->AddChild(box2);
}

void TriggeredBoundingBoxCameraTest::BoxesWithBuiltinSDF(
  const std::string &_renderEngine)
{
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "triggered_boundingbox_camera_sensor_builtin.sdf");
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

    // Skip unsupported engines
  if (_renderEngine != "ogre2")
  {
    gzdbg << "Engine '" << _renderEngine
           << "' doesn't support bounding box cameras" << std::endl;
    return;
  }

  // Setup gz-rendering with an empty scene
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    gzdbg << "Engine '" << _renderEngine
           << "' is not supported" << std::endl;
    return;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);
  BuildScene(scene);

  gz::sensors::Manager mgr;

  sdf::Sensor sdfSensor;
  sdfSensor.Load(sensorPtr);

  std::string type = sdfSensor.TypeStr();
  EXPECT_EQ(type, "boundingbox_camera");

  gz::sensors::BoundingBoxCameraSensor *sensor =
      mgr.CreateSensor<gz::sensors::BoundingBoxCameraSensor>(sdfSensor);
  ASSERT_NE(sensor, nullptr);
  EXPECT_FALSE(sensor->HasConnections());
  sensor->SetScene(scene);

  EXPECT_EQ(320u, sensor->ImageWidth());
  EXPECT_EQ(240u, sensor->ImageHeight());
  EXPECT_EQ(true, sdfSensor.CameraSensor()->Triggered());

  // subscribe to the BoundingBox camera topic
  gz::transport::Node node;
  std::string boxTopic =
      "/test/integration/TriggeredBBCameraPlugin_imagesWithBuiltinSDF";
  node.Subscribe(boxTopic, &OnNewBoundingBoxes);

  // we should not have image before trigger
  {
    std::string imageTopic =
        "/test/integration/TriggeredBBCameraPlugin_imagesWithBuiltinSDF_image";
    WaitForMessageTestHelper<gz::msgs::Image> helper(imageTopic);
    mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
    EXPECT_FALSE(helper.WaitForMessage(1s)) << helper;
    g_mutex.lock();
    EXPECT_EQ(g_boxes.size(), size_t(0));
    g_mutex.unlock();
  }

  // trigger camera through topic
  std::string triggerTopic =
      "/test/integration/TriggeredBBCameraPlugin_imagesWithBuiltinSDF/trigger";

  auto pub = node.Advertise<gz::msgs::Boolean>(triggerTopic);
  gz::msgs::Boolean msg;
  msg.set_data(true);
  pub.Publish(msg);
  // sleep to wait for trigger msg to be received before calling mgr.RunOnce
  std::this_thread::sleep_for(2s);

  // we should receive images and boxes after trigger
  {
    WaitForMessageTestHelper<gz::msgs::AnnotatedAxisAligned2DBox_V>
        helper(boxTopic);
    mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
    EXPECT_TRUE(helper.WaitForMessage(10s)) << helper;
    g_mutex.lock();
    EXPECT_EQ(g_boxes.size(), size_t(2));
    g_mutex.unlock();
  }

  // Clean up
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(TriggeredBoundingBoxCameraTest, BoxesWithBuiltinSDF)
{
  BoxesWithBuiltinSDF(GetParam());
}

INSTANTIATE_TEST_SUITE_P(BoundingBoxCameraSensor,
    TriggeredBoundingBoxCameraTest,
    RENDER_ENGINE_VALUES, gz::rendering::PrintToStringParam());

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  gz::common::Console::SetVerbosity(4);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
