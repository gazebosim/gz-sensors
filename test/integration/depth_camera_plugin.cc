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
#include <ignition/common/Filesystem.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/sensors/DepthCameraSensor.hh>
#include <ignition/rendering.hh>
#include <ignition/msgs.hh>
#include "test/test_config.hh"
#include "TransportTestTools.hh"

#include <ignition/common/Event.hh>

void BuildScene(ignition::rendering::ScenePtr _scene);

std::mutex g_depthMutex;
unsigned int g_depthCounter = 0;
float *g_depthBuffer = nullptr;


void OnNewDepthFrame(const float * _image,
    unsigned int _width, unsigned int _height,
    unsigned int /*_depth*/, const std::string &/*_format*/)
{

  if (!_image)
    return;
  std::lock_guard<std::mutex> lock(g_depthMutex);
  if (!g_depthBuffer)
    g_depthBuffer = new float[_width * _height];
  memcpy(g_depthBuffer,  _image, _width * _height * sizeof(_image[0]));
  g_depthCounter++;
}


TEST(DepthCameraPlugin, imagesWithBuiltinSDF)
{
  // get the darn test data
  std::string path = ignition::common::joinPaths(PROJECT_SOURCE_DIR, "test",
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

  // Setup ign-rendering with an empty scene
  auto *engine = ignition::rendering::engine("ogre");
  ASSERT_NE(nullptr, engine);
  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");

  // Add stuff to take a picture of
  BuildScene(scene);

  // do the test
  ignition::sensors::Manager mgr;
  mgr.SetRenderingScene(scene);
  mgr.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_DIR, "lib"));

  auto *sensor = mgr.CreateSensor<ignition::sensors::DepthCameraSensor>(sensorPtr);
  ASSERT_NE(sensor, nullptr);
/*
  EXPECT_EQ(sensor->ImageWidth(), 640u);
  EXPECT_EQ(sensor->ImageHeight(), 480u);
  EXPECT_TRUE(sensor->IsActive());
*/
  std::string topic = "/test/integration/DepthCameraPlugin_imagesWithBuiltinSDF";
  WaitForMessageTestHelper<ignition::msgs::Image> helper(topic);

  ignition::rendering::DepthCameraPtr depthCamera = sensor->DepthCamera();
  EXPECT_TRUE(depthCamera != nullptr);

  ignition::common::ConnectionPtr c = depthCamera->ConnectNewDepthFrame(
      std::bind(&::OnNewDepthFrame, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
      std::placeholders::_5));

  // Update once to create image
  mgr.RunOnce(ignition::common::Time::Zero);

  EXPECT_TRUE(helper.WaitForMessage()) << helper;

  // wait for a few depth camera frames
  int j = 0;
  while (g_depthCounter < 10 && j < 300)
  {
    testing::internal::SleepMilliseconds(10);
    mgr.RunOnce(ignition::common::Time::Zero, true);
    j++;
  }
  EXPECT_LT(j, 300);

  unsigned int imageSize = 256 * 256;
//      sensor->ImageWidth() * sensor->ImageHeight();

  std::lock_guard<std::mutex> lock(g_depthMutex);
  // Check that the depth values are valid
  for (unsigned int i = 0; i < imageSize; ++i)
  {
 //   EXPECT_TRUE(g_depthBuffer[i] <= depthCamera->FarClip());
 //   EXPECT_TRUE(g_depthBuffer[i] >= depthCamera->NearClip());
 //   EXPECT_TRUE(!ignition::math::equal(g_depthBuffer[i], 0.0f));
  }

  // sphere with radius 1m is at 2m in front of depth camera
  // so verify depth readings are between 1-2m in the mid row
  unsigned int halfHeight =
    static_cast<unsigned int>(256*0.5)-1;
//    static_cast<unsigned int>(sensor->ImageHeight()*0.5)-1;
//  for (unsigned int i = sensor->ImageWidth()*halfHeight;
//      i < sensor->ImageWidth()*(halfHeight+1); ++i)
  for (unsigned int i = 256*halfHeight;
      i < 256*(halfHeight+1); ++i)
  {
    EXPECT_TRUE(g_depthBuffer[i] < 2.0f);
    EXPECT_TRUE(g_depthBuffer[i] >= 1.0f);
    std::cout << g_depthBuffer[i] << std::endl;
  }
}

// Copy/paste from an ignition-rendering example
void BuildScene(ignition::rendering::ScenePtr _scene)
{
  // initialize _scene
  _scene->SetAmbientLight(0.3, 0.3, 0.3);
  ignition::rendering::VisualPtr root = _scene->RootVisual();

  // create directional light
  ignition::rendering::DirectionalLightPtr light0 =
    _scene->CreateDirectionalLight();
  light0->SetDirection(-0.5, 0.5, -1);
  light0->SetDiffuseColor(0.5, 0.5, 0.5);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  // create point light
  ignition::rendering::PointLightPtr light2 = _scene->CreatePointLight();
  light2->SetDiffuseColor(0.5, 0.5, 0.5);
  light2->SetSpecularColor(0.5, 0.5, 0.5);
  light2->SetLocalPosition(3, 5, 5);
  root->AddChild(light2);

  // create green material
  ignition::rendering::MaterialPtr green = _scene->CreateMaterial();
  green->SetAmbient(0.0, 0.5, 0.0);
  green->SetDiffuse(0.0, 0.7, 0.0);
  green->SetSpecular(0.5, 0.5, 0.5);
  green->SetShininess(50);
  green->SetReflectivity(0);

  // create center visual
  ignition::rendering::VisualPtr center = _scene->CreateVisual();
  center->AddGeometry(_scene->CreateSphere());
  center->SetLocalPosition(3, 0, 0);
  center->SetLocalScale(0.1, 0.1, 0.1);
  center->SetMaterial(green);
  root->AddChild(center);

  // create red material
  ignition::rendering::MaterialPtr red = _scene->CreateMaterial();
  red->SetAmbient(0.5, 0.0, 0.0);
  red->SetDiffuse(1.0, 0.0, 0.0);
  red->SetSpecular(0.5, 0.5, 0.5);
  red->SetShininess(50);
  red->SetReflectivity(0);

  // create sphere visual
  ignition::rendering::VisualPtr sphere = _scene->CreateVisual();
  sphere->AddGeometry(_scene->CreateSphere());
  sphere->SetOrigin(0.0, -0.5, 0.0);
  sphere->SetLocalPosition(3, 0, 0);
  sphere->SetLocalRotation(0, 0, 0);
  sphere->SetLocalScale(1, 2.5, 1);
  sphere->SetMaterial(red);
  root->AddChild(sphere);

  // create blue material
  ignition::rendering::MaterialPtr blue = _scene->CreateMaterial();
  blue->SetAmbient(0.0, 0.0, 0.3);
  blue->SetDiffuse(0.0, 0.0, 0.8);
  blue->SetSpecular(0.5, 0.5, 0.5);
  blue->SetShininess(50);
  blue->SetReflectivity(0);

  // create box visual
  ignition::rendering::VisualPtr box = _scene->CreateVisual();
  box->AddGeometry(_scene->CreateBox());
  box->SetOrigin(0.0, 0.5, 0.0);
  box->SetLocalPosition(3, 0, 0);
  box->SetLocalRotation(M_PI / 4, 0, M_PI / 3);
  box->SetLocalScale(1, 2.5, 1);
  box->SetMaterial(blue);
  root->AddChild(box);

  // create white material
  ignition::rendering::MaterialPtr white = _scene->CreateMaterial();
  white->SetAmbient(0.5, 0.5, 0.5);
  white->SetDiffuse(0.8, 0.8, 0.8);
  white->SetReceiveShadows(true);
  white->SetReflectivity(0);

  // create sphere visual
  ignition::rendering::VisualPtr plane = _scene->CreateVisual();
  plane->AddGeometry(_scene->CreatePlane());
  plane->SetLocalScale(5, 8, 1);
  plane->SetLocalPosition(3, 0, -0.5);
  plane->SetMaterial(white);
  root->AddChild(plane);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
