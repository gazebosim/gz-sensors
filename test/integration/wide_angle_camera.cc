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

#include <cstring>
#include <gtest/gtest.h>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/sensors/Manager.hh>
#include <gz/sensors/WideAngleCameraSensor.hh>

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

std::mutex g_mutex;
unsigned int g_imageCounter = 0;
unsigned char *g_imageBuffer = nullptr;

std::mutex g_infoMutex;
unsigned int g_infoCounter = 0;
gz::msgs::CameraInfo g_infoMsg;

void OnCameraInfo(const gz::msgs::CameraInfo & _msg)
{
  g_infoMutex.lock();
  g_infoCounter++;
  g_infoMsg.CopyFrom(_msg);
  g_infoMutex.unlock();
}

void OnImage(const gz::msgs::Image &_msg)
{
  g_mutex.lock();
  unsigned int channelCount = 3u;
  unsigned int imageSamples = _msg.width() * _msg.height() * channelCount;
  unsigned int imageBufferSize = imageSamples * sizeof(unsigned char);
  if (!g_imageBuffer)
    g_imageBuffer = new unsigned char[imageSamples];
  memcpy(g_imageBuffer, _msg.data().c_str(), imageBufferSize);
  g_imageCounter++;
  g_mutex.unlock();
}

class WideAngleCameraSensorTest: public testing::Test,
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

  // Create a Camera sensor from a SDF and gets a image message
  public: void ImagesWithBuiltinSDF(const std::string &_renderEngine);
};

void WideAngleCameraSensorTest::ImagesWithBuiltinSDF(
    const std::string &_renderEngine)
{
  if (_renderEngine != "ogre")
  {
    gzwarn << "Wide angle cameras are not supported in " << _renderEngine
            << std::endl;
    return;
  }

  // get the darn test data
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "wide_angle_camera_sensor_builtin.sdf");
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

  // Setup gz-rendering with an empty scene
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    gzdbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");
  scene->SetAmbientLight(1.0, 1.0, 1.0);

  gz::rendering::VisualPtr root = scene->RootVisual();
  ASSERT_NE(nullptr, root);

  // create blue material
  gz::rendering::MaterialPtr blue = scene->CreateMaterial();
  blue->SetAmbient(0.0, 0.0, 0.3);
  blue->SetDiffuse(0.0, 0.0, 0.8);
  blue->SetSpecular(0.5, 0.5, 0.5);

  // create box visual
  gz::rendering::VisualPtr box = scene->CreateVisual("box");
  ASSERT_NE(nullptr, box);
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(gz::math::Vector3d(2.0, 0, 0));
  box->SetLocalRotation(0, 0, 0);
  box->SetMaterial(blue);
  root->AddChild(box);

  // do the test
  gz::sensors::Manager mgr;

  gz::sensors::WideAngleCameraSensor *sensor =
      mgr.CreateSensor<gz::sensors::WideAngleCameraSensor>(sensorPtr);
  ASSERT_NE(sensor, nullptr);
  EXPECT_FALSE(sensor->HasConnections());
  sensor->SetScene(scene);

  ASSERT_NE(sensor->RenderingCamera(), nullptr);
  EXPECT_NE(sensor->Id(), sensor->RenderingCamera()->Id());
  EXPECT_EQ(320u, sensor->ImageWidth());
  EXPECT_EQ(240u, sensor->ImageHeight());

  std::string topicBase =
      "/test/integration/WideAngleCamera_imagesWithBuiltinSDF/";
  std::string topic = topicBase + "image";
  WaitForMessageTestHelper<gz::msgs::Image> helper(topic);
  EXPECT_TRUE(sensor->HasConnections());

  std::string infoTopic = topicBase + "camera_info";
  WaitForMessageTestHelper<gz::msgs::CameraInfo> infoHelper(infoTopic);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper.WaitForMessage()) << helper;
  EXPECT_TRUE(infoHelper.WaitForMessage()) << infoHelper;

  // subscribe to the wide angle camera topic
  gz::transport::Node node;
  node.Subscribe(topic, &OnImage);

  // subscribe to the thermal camera topic
  node.Subscribe(infoTopic, &OnCameraInfo);

  // wait for a few camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  auto waitTime = std::chrono::duration_cast< std::chrono::milliseconds >(
      std::chrono::duration< double >(0.001));
  int counter = 0;
  int infoCounter = 0;
  gz::msgs::CameraInfo infoMsg;
  for (int sleep = 0;
       sleep < 300 && (counter == 0 || infoCounter == 0); ++sleep)
  {
    g_mutex.lock();
    counter = g_imageCounter;
    g_mutex.unlock();

    g_infoMutex.lock();
    infoCounter = g_infoCounter;
    infoMsg = g_infoMsg;
    g_infoMutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }
  g_mutex.lock();
  g_infoMutex.lock();
  g_imageCounter = 0;
  g_infoCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_EQ(counter, infoCounter);
  counter = 0;
  infoCounter = 0;

  // Compare image pixels
  unsigned int channelCount = gz::rendering::PixelUtil::ChannelCount(
      sensor->RenderingCamera()->ImageFormat());
  unsigned int step = sensor->ImageWidth() * channelCount;

  // verify camera can see the blue box in the middle
  unsigned int mid = static_cast<unsigned int>(
     sensor->ImageHeight() / 2.0 * step + step / 2.0);
  unsigned int r = g_imageBuffer[mid];
  unsigned int g = g_imageBuffer[mid + 1];
  unsigned int b = g_imageBuffer[mid + 2];
  EXPECT_GT(b, g);
  EXPECT_GT(b, r);

  auto sensorId = sensor->Id();
  EXPECT_TRUE(mgr.Remove(sensorId));
  EXPECT_EQ(nullptr, mgr.Sensor(sensorId));

  // Clean up
  box.reset();
  blue.reset();
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(WideAngleCameraSensorTest, ImagesWithBuiltinSDF)
{
  gz::common::Console::SetVerbosity(4);
  ImagesWithBuiltinSDF(GetParam());
}

INSTANTIATE_TEST_SUITE_P(WideAngleCameraSensor, WideAngleCameraSensorTest,
    RENDER_ENGINE_VALUES, gz::rendering::PrintToStringParam());
