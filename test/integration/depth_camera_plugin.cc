/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <ignition/common/Event.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/sensors/DepthCameraSensor.hh>
#include <ignition/rendering.hh>
#include <ignition/msgs.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "TransportTestTools.hh"

#define DEPTH_TOL 1e-4
#define DOUBLE_TOL 1e-6

unsigned int g_depthCounter = 0;
float *g_depthBuffer = nullptr;

void OnImage(const ignition::msgs::Image &_msg)
{
  unsigned int depthSamples = _msg.width() * _msg.height();
  unsigned int depthBufferSize = depthSamples * sizeof(float);
  if (!g_depthBuffer)
    g_depthBuffer = new float[depthSamples];
  memcpy(g_depthBuffer, _msg.data().c_str(), depthBufferSize);
  g_depthCounter++;
}

class DepthCameraSensorTest: public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Create a Camera sensor from a SDF and gets a image message
  public: void ImagesWithBuiltinSDF(const std::string &_renderEngine);
};

void DepthCameraSensorTest::ImagesWithBuiltinSDF(
    const std::string &_renderEngine)
{
  // get the darn test data
  std::string path = ignition::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "integration", "depth_camera_sensor_builtin.sdf");
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
  if (_renderEngine.compare("ogre") != 0)
  {
    igndbg << "Engine '" << _renderEngine
              << "' doesn't support depth cameras" << std::endl;
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

  // create blue material
  ignition::rendering::MaterialPtr blue = scene->CreateMaterial();
  blue->SetAmbient(0.0, 0.0, 0.3);
  blue->SetDiffuse(0.0, 0.0, 0.8);
  blue->SetSpecular(0.5, 0.5, 0.5);
  blue->SetShininess(50);
  blue->SetReflectivity(0);

  // create box visual
  ignition::rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(boxPosition);
  box->SetLocalRotation(0, 0, 0);
  box->SetLocalScale(unitBoxSize, unitBoxSize, unitBoxSize);
  box->SetMaterial(blue);
  root->AddChild(box);

  // do the test
  ignition::sensors::Manager mgr;
  mgr.SetRenderingScene(scene);
  mgr.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_PATH, "lib"));

  ignition::sensors::DepthCameraSensor *ds =
      mgr.CreateSensor<ignition::sensors::DepthCameraSensor>(
      sensorPtr);
  ASSERT_NE(ds, nullptr);
  std::unique_ptr<ignition::sensors::DepthCameraSensor> depthSensor(ds);

  EXPECT_EQ(depthSensor->ImageWidth(), static_cast<unsigned int>(imgWidth));
  EXPECT_EQ(depthSensor->ImageHeight(), static_cast<unsigned int>(imgHeight));
  EXPECT_NEAR(depthSensor->FarClip(), far_, DOUBLE_TOL);
  EXPECT_NEAR(depthSensor->NearClip(), near_, DOUBLE_TOL);

  std::string topic =
    "/test/integration/DepthCameraPlugin_imagesWithBuiltinSDF";
  WaitForMessageTestHelper<ignition::msgs::Image> helper(topic);

  // Update once to create image
  mgr.RunOnce(ignition::common::Time::Zero);

  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  // Set a callback on the  camera sensor to get a depth camera frame
  ignition::common::ConnectionPtr connection =
    depthSensor->ConnectImageCallback(&OnImage);

  // wait for a few depth camera frames
  mgr.RunOnce(ignition::common::Time::Zero, true);

  int midWidth = depthSensor->ImageWidth() * 0.5;
  int midHeight = depthSensor->ImageHeight() * 0.5;
  int mid = midHeight * depthSensor->ImageWidth() + midWidth -1;
  double expectedRangeAtMidPoint = boxPosition.X() - unitBoxSize * 0.5;

  // Depth sensor should see box in the middle of the image
  EXPECT_NEAR(g_depthBuffer[mid], expectedRangeAtMidPoint, DEPTH_TOL);

  // The left and right side of the depth frame should be inf
  int left = midHeight * depthSensor->ImageWidth();
  EXPECT_DOUBLE_EQ(g_depthBuffer[left], ignition::math::INF_D);
  int right = (midHeight+1) * depthSensor->ImageWidth() - 1;
  EXPECT_DOUBLE_EQ(g_depthBuffer[right], ignition::math::INF_D);

  // Check that for a box really close it returns -inf
  root->RemoveChild(box);
  ignition::math::Vector3d boxPositionNear(
      unitBoxSize * 0.5 + near_ * 0.5, 0.0, 0.0);
  box->SetLocalPosition(boxPositionNear);
  root->AddChild(box);
  mgr.RunOnce(ignition::common::Time::Zero, true);
  EXPECT_DOUBLE_EQ(g_depthBuffer[mid], -ignition::math::INF_D);

  // Check that for a box really far it returns inf
  root->RemoveChild(box);
  ignition::math::Vector3d boxPositionFar(
      unitBoxSize * 0.5 + far_ * 1.5, 0.0, 0.0);
  box->SetLocalPosition(boxPositionFar);
  root->AddChild(box);
  mgr.RunOnce(ignition::common::Time::Zero, true);
  EXPECT_DOUBLE_EQ(g_depthBuffer[mid], ignition::math::INF_D);

  // Clean up
  connection.reset();
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

TEST_P(DepthCameraSensorTest, ImagesWithBuiltinSDF)
{
  ImagesWithBuiltinSDF(GetParam());
}

INSTANTIATE_TEST_CASE_P(DepthCameraSensor, DepthCameraSensorTest,
    RENDER_ENGINE_VALUES, ignition::rendering::PrintToStringParam());

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
