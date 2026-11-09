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

#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <gtest/gtest.h>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/common/Image.hh>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/sensors/Manager.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/transport/Node.hh>
#include <gz/rendering/PixelBuffer.hh>
#include <gz/rendering/PixelFormat.hh>
#include <gz/rendering/Utils.hh>

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
unsigned int g_imgCounter1 = 0;
unsigned int g_imgCounter2 = 0;

void OnGrayscaleImageL8(const gz::msgs::Image &_msg)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  EXPECT_EQ(gz::msgs::PixelFormatType::L_INT8, _msg.pixel_format_type());
  EXPECT_EQ(256u, _msg.width());
  EXPECT_EQ(256u, _msg.height());
  g_imgCounter1++;
}

void OnGrayscaleImageL16(const gz::msgs::Image &_msg)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  EXPECT_EQ(gz::msgs::PixelFormatType::L_INT16, _msg.pixel_format_type());
  EXPECT_EQ(512u, _msg.width());
  EXPECT_EQ(512u, _msg.height());
  g_imgCounter2++;
}

class CameraSensorTest: public testing::Test,
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

  // Create 8 bit and 16 bit grayscale camera sensors and verify image format
  public: void ImageFormatLInt8LInt16(const std::string &_renderEngine);

  // Create camera sensors and verify camera intrinsics
  public: void CameraIntrinsics(const std::string &_renderEngine);

  // Create camera sensors and verify camera projection
  public: void CameraProjection(const std::string &_renderEngine);

  // Create a camera sensor and verify a custom camera info topic
  public: void CustomCameraInfoTopic(const std::string &_renderEngine);

  // Compare the image bytes published in place with those of Publish(),
  // for an image of the SDF format _format (empty: the SDF's)
  public: void PublishedBytesMatchCopy(const std::string &_renderEngine,
      const std::string &_format);
};

void CameraSensorTest::ImagesWithBuiltinSDF(const std::string &_renderEngine)
{
  // get the darn test data
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
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

  // Setup gz-rendering with an empty scene
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");

  // do the test
  gz::sensors::Manager mgr;

  gz::sensors::CameraSensor *sensor =
      mgr.CreateSensor<gz::sensors::CameraSensor>(sensorPtr);
  ASSERT_NE(sensor, nullptr);
  EXPECT_FALSE(sensor->HasConnections());
  sensor->SetScene(scene);

  ASSERT_NE(sensor->RenderingCamera(), nullptr);
  EXPECT_NE(sensor->Id(), sensor->RenderingCamera()->Id());
  EXPECT_EQ(256u, sensor->ImageWidth());
  EXPECT_EQ(257u, sensor->ImageHeight());

  EXPECT_EQ(std::string("base_camera"), sensor->FrameId());

  std::string topic = "/test/integration/CameraPlugin_imagesWithBuiltinSDF";
  WaitForMessageTestHelper<gz::msgs::Image> helper(topic);

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
  gz::rendering::unloadEngine(engine->Name());
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

  // Camera sensor with different intrinsics tag
  auto cameraWithDiffIntrinsicsTag =
      cameraWithIntrinsicsTag->GetNextElement();

  // Setup gz-rendering with an empty scene
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
          << "' is not supported" << std::endl;
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
  box->SetLocalPosition(gz::math::Vector3d(4.0, 1, 0.5));
  box->SetLocalRotation(0, 0, 0);
  box->SetMaterial(blue);
  scene->DestroyMaterial(blue);
  root->AddChild(box);

  // Do the test
  gz::sensors::Manager mgr;

  // there are 3 cameras:
  //     - camera1: no intrinsics params
  //     - camera2: has intrinsics params that are the same as default values
  //     - camera3: has intrinsics params that are different from default values
  // camera1 and camera2 should produce very similar images and camera3 should
  // produce different images from 1 and 2.
  auto *sensor1 =
      mgr.CreateSensor<gz::sensors::CameraSensor>(cameraWithoutIntrinsicsTag);
  auto *sensor2 =
      mgr.CreateSensor<gz::sensors::CameraSensor>(cameraWithIntrinsicsTag);
  auto *sensor3 =
      mgr.CreateSensor<gz::sensors::CameraSensor>(cameraWithDiffIntrinsicsTag);

  ASSERT_NE(sensor1, nullptr);
  ASSERT_NE(sensor2, nullptr);
  ASSERT_NE(sensor3, nullptr);
  sensor1->SetScene(scene);
  sensor2->SetScene(scene);
  sensor3->SetScene(scene);

  std::string infoTopic1 = "/camera1/camera_info";
  std::string infoTopic2 = "/camera2/camera_info";
  std::string infoTopic3 = "/camera3/camera_info";
  std::string imgTopic1 = "/camera1/image";
  std::string imgTopic2 = "/camera2/image";
  std::string imgTopic3 = "/camera3/image";
  WaitForMessageTestHelper<gz::msgs::Image> helper1(imgTopic1);
  WaitForMessageTestHelper<gz::msgs::CameraInfo> helper2(infoTopic1);
  WaitForMessageTestHelper<gz::msgs::Image> helper3(imgTopic2);
  WaitForMessageTestHelper<gz::msgs::CameraInfo> helper4(infoTopic2);
  WaitForMessageTestHelper<gz::msgs::Image> helper5(imgTopic3);
  WaitForMessageTestHelper<gz::msgs::CameraInfo> helper6(infoTopic3);

  EXPECT_TRUE(sensor1->HasConnections());
  EXPECT_TRUE(sensor2->HasConnections());
  EXPECT_TRUE(sensor3->HasConnections());

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper1.WaitForMessage()) << helper1;
  EXPECT_TRUE(helper2.WaitForMessage()) << helper2;
  EXPECT_TRUE(helper3.WaitForMessage()) << helper3;
  EXPECT_TRUE(helper4.WaitForMessage()) << helper4;
  EXPECT_TRUE(helper5.WaitForMessage()) << helper5;
  EXPECT_TRUE(helper6.WaitForMessage()) << helper6;

  // Subscribe to the camera info topic
  gz::msgs::CameraInfo camera1Info, camera2Info, camera3Info;
  unsigned int camera1Counter = 0u;
  unsigned int camera2Counter = 0u;
  unsigned int camera3Counter = 0u;

  std::function<void(const gz::msgs::CameraInfo&)> camera1InfoCallback =
      [&camera1Info, &camera1Counter](const gz::msgs::CameraInfo& _msg)
  {
    camera1Info = _msg;
    camera1Counter++;
  };
  std::function<void(const gz::msgs::CameraInfo&)> camera2InfoCallback =
      [&camera2Info, &camera2Counter](const gz::msgs::CameraInfo& _msg)
  {
    camera2Info = _msg;
    camera2Counter++;
  };
  std::function<void(const gz::msgs::CameraInfo&)> camera3InfoCallback =
      [&camera3Info, &camera3Counter](const gz::msgs::CameraInfo& _msg)
  {
    camera3Info = _msg;
    camera3Counter++;
  };

  unsigned int height = 1000u;
  unsigned int width = 1000u;
  unsigned int bpp = 3u;
  unsigned int imgBufferSize = width * height * bpp;
  unsigned char* img1 = new unsigned char[imgBufferSize];
  unsigned char* img2 = new unsigned char[imgBufferSize];
  unsigned char* img3 = new unsigned char[imgBufferSize];
  unsigned int camera1DataCounter = 0u;
  unsigned int camera2DataCounter = 0u;
  unsigned int camera3DataCounter = 0u;
  std::function<void(const gz::msgs::Image &)> camera1Callback =
      [&img1, &camera1DataCounter, &imgBufferSize](const gz::msgs::Image & _msg)
  {
    memcpy(img1, _msg.data().c_str(), imgBufferSize);
    camera1DataCounter++;
  };

  std::function<void(const gz::msgs::Image &)> camera2Callback =
      [&img2, &camera2DataCounter, &imgBufferSize](const gz::msgs::Image & _msg)
  {
    memcpy(img2, _msg.data().c_str(), imgBufferSize);
    camera2DataCounter++;
  };
  std::function<void(const gz::msgs::Image &)> camera3Callback =
      [&img3, &camera3DataCounter, &imgBufferSize](const gz::msgs::Image & _msg)
  {
    memcpy(img3, _msg.data().c_str(), imgBufferSize);
    camera3DataCounter++;
  };

  // Subscribe to the camera topic
  gz::transport::Node node;
  node.Subscribe(infoTopic1, camera1InfoCallback);
  node.Subscribe(infoTopic2, camera2InfoCallback);
  node.Subscribe(infoTopic3, camera3InfoCallback);
  node.Subscribe(imgTopic1, camera1Callback);
  node.Subscribe(imgTopic2, camera2Callback);
  node.Subscribe(imgTopic3, camera3Callback);

  // Wait for a few camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // Run to get image and check image format in callback
  bool done = false;
  int sleep = 0;
  int maxSleep = 10;
  while (!done && sleep++ < maxSleep)
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    done = (camera1Counter > 0u && camera2Counter > 0u && camera3Counter > 0u &&
        camera1DataCounter > 0u && camera2DataCounter > 0u &&
        camera3DataCounter > 0u);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Image size, focal length and optical center
  // Camera sensor without intrinsics tag
  double error = 3e-1;
  EXPECT_EQ(camera1Info.width(), width);
  EXPECT_EQ(camera1Info.height(), height);
  EXPECT_NEAR(camera1Info.intrinsics().k(0), 866.23, error);
  EXPECT_NEAR(camera1Info.intrinsics().k(4), 866.23, error);
  EXPECT_DOUBLE_EQ(camera1Info.intrinsics().k(2), 500);
  EXPECT_DOUBLE_EQ(camera1Info.intrinsics().k(5), 500);

  // Camera sensor with intrinsics tag
  EXPECT_EQ(camera2Info.width(), width);
  EXPECT_EQ(camera2Info.height(), height);
  EXPECT_DOUBLE_EQ(camera2Info.intrinsics().k(0), 866.23);
  EXPECT_DOUBLE_EQ(camera2Info.intrinsics().k(4), 866.23);
  EXPECT_DOUBLE_EQ(camera2Info.intrinsics().k(2), 500);
  EXPECT_DOUBLE_EQ(camera2Info.intrinsics().k(5), 500);

  // Camera sensor with different intrinsics tag
  EXPECT_EQ(camera3Info.width(), width);
  EXPECT_EQ(camera3Info.height(), height);
  EXPECT_DOUBLE_EQ(camera3Info.intrinsics().k(0), 900);
  EXPECT_DOUBLE_EQ(camera3Info.intrinsics().k(4), 900);
  EXPECT_DOUBLE_EQ(camera3Info.intrinsics().k(2), 501);
  EXPECT_DOUBLE_EQ(camera3Info.intrinsics().k(5), 501);

  unsigned int r1Sum = 0u;
  unsigned int g1Sum = 0u;
  unsigned int b1Sum = 0u;
  unsigned int r2Sum = 0u;
  unsigned int g2Sum = 0u;
  unsigned int b2Sum = 0u;
  unsigned int r3Sum = 0u;
  unsigned int g3Sum = 0u;
  unsigned int b3Sum = 0u;
  unsigned int step = width * bpp;

  // get sum of each color channel
  // all cameras should just see blue colors
  for (unsigned int i = 0u; i < height; ++i)
  {
    for (unsigned int j = 0u; j < step; j+=bpp)
    {
      unsigned int idx = i * step + j;
      unsigned int r1 = img1[idx];
      unsigned int g1 = img1[idx+1];
      unsigned int b1 = img1[idx+2];
      r1Sum += r1;
      g1Sum += g1;
      b1Sum += b1;

      unsigned int r2 = img2[idx];
      unsigned int g2 = img2[idx+1];
      unsigned int b2 = img2[idx+2];
      r2Sum += r2;
      g2Sum += g2;
      b2Sum += b2;

      unsigned int r3 = img3[idx];
      unsigned int g3 = img3[idx+1];
      unsigned int b3 = img3[idx+2];
      r3Sum += r3;
      g3Sum += g3;
      b3Sum += b3;
    }
  }

  // images from camera1 without intrinsics params and
  // camera2 with default intrinsic params should be the same
  EXPECT_EQ(0u, r1Sum);
  EXPECT_EQ(0u, g1Sum);
  EXPECT_LT(0u, b1Sum);
  EXPECT_EQ(r1Sum, r2Sum);
  EXPECT_EQ(g1Sum, g2Sum);
  EXPECT_EQ(b1Sum, b2Sum);

  // images from camera2 and camera3 that have different intrinsics params
  // should be different. Both cameras should still see the blue box but
  // sum of blue pixels should be slightly different
  EXPECT_EQ(0u, r3Sum);
  EXPECT_EQ(0u, g3Sum);
  EXPECT_LT(0u, b3Sum);
  EXPECT_EQ(r2Sum, r3Sum);
  EXPECT_EQ(g2Sum, g3Sum);
  EXPECT_NE(b2Sum, b3Sum);

  delete []  img1;
  delete []  img2;
  delete []  img3;

  // Clean up rendering ptrs
  box.reset();
  blue.reset();

  // Clean up
  mgr.Remove(sensor1->Id());
  mgr.Remove(sensor2->Id());
  mgr.Remove(sensor3->Id());
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

  // Camera sensor without projection tag
  auto cameraWithoutProjectionsTag = linkPtr->GetElement("sensor");

  // Camera sensor with projection tag
  auto cameraWithProjectionsTag =
      linkPtr->GetElement("sensor")->GetNextElement();

  // Camera sensor with different projection tag
  auto cameraWithDiffProjectionsTag =
      cameraWithProjectionsTag->GetNextElement();

  // Setup gz-rendering with an empty scene
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
          << "' is not supported" << std::endl;
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
  box->SetLocalPosition(gz::math::Vector3d(4.0, 1, 0.5));
  box->SetLocalRotation(0, 0, 0);
  box->SetMaterial(blue);
  scene->DestroyMaterial(blue);
  root->AddChild(box);

  // Do the test
  gz::sensors::Manager mgr;

  auto *sensor1 =
      mgr.CreateSensor<gz::sensors::CameraSensor>(cameraWithoutProjectionsTag);
  auto *sensor2 =
      mgr.CreateSensor<gz::sensors::CameraSensor>(cameraWithProjectionsTag);
  auto *sensor3 =
      mgr.CreateSensor<gz::sensors::CameraSensor>(cameraWithDiffProjectionsTag);

  ASSERT_NE(sensor1, nullptr);
  ASSERT_NE(sensor2, nullptr);
  ASSERT_NE(sensor3, nullptr);
  std::string imgTopic1 = "/camera1/image";
  std::string imgTopic2 = "/camera2/image";
  std::string imgTopic3 = "/camera3/image";
  sensor1->SetScene(scene);
  sensor2->SetScene(scene);
  sensor3->SetScene(scene);

  std::string infoTopic1 = "/camera1/camera_info";
  std::string infoTopic2 = "/camera2/camera_info";
  std::string infoTopic3 = "/camera3/camera_info";
  WaitForMessageTestHelper<gz::msgs::Image> helper1("/camera1/image");
  WaitForMessageTestHelper<gz::msgs::CameraInfo> helper2(infoTopic1);
  WaitForMessageTestHelper<gz::msgs::Image> helper3("/camera2/image");
  WaitForMessageTestHelper<gz::msgs::CameraInfo> helper4(infoTopic2);
  WaitForMessageTestHelper<gz::msgs::Image> helper5(imgTopic3);
  WaitForMessageTestHelper<gz::msgs::CameraInfo> helper6(infoTopic3);

  EXPECT_TRUE(sensor1->HasConnections());
  EXPECT_TRUE(sensor2->HasConnections());
  EXPECT_TRUE(sensor3->HasConnections());

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper1.WaitForMessage()) << helper1;
  EXPECT_TRUE(helper2.WaitForMessage()) << helper2;
  EXPECT_TRUE(helper3.WaitForMessage()) << helper3;
  EXPECT_TRUE(helper4.WaitForMessage()) << helper4;
  EXPECT_TRUE(helper5.WaitForMessage()) << helper5;
  EXPECT_TRUE(helper6.WaitForMessage()) << helper6;

  // Subscribe to the camera info topic
  gz::msgs::CameraInfo camera1Info, camera2Info, camera3Info;
  unsigned int camera1Counter = 0u;
  unsigned int camera2Counter = 0u;
  unsigned int camera3Counter = 0u;

  std::function<void(const gz::msgs::CameraInfo&)> camera1InfoCallback =
      [&camera1Info, &camera1Counter](const gz::msgs::CameraInfo& _msg)
  {
    camera1Info = _msg;
    camera1Counter++;
  };
  std::function<void(const gz::msgs::CameraInfo&)> camera2InfoCallback =
      [&camera2Info, &camera2Counter](const gz::msgs::CameraInfo& _msg)
  {
    camera2Info = _msg;
    camera2Counter++;
  };
  std::function<void(const gz::msgs::CameraInfo&)> camera3InfoCallback =
      [&camera3Info, &camera3Counter](const gz::msgs::CameraInfo& _msg)
  {
    camera3Info = _msg;
    camera3Counter++;
  };

  unsigned int height = 1000u;
  unsigned int width = 1000u;
  unsigned int bpp = 3u;
  unsigned int imgBufferSize = width * height * bpp;
  unsigned char* img1 = new unsigned char[imgBufferSize];
  unsigned char* img2 = new unsigned char[imgBufferSize];
  unsigned char* img3 = new unsigned char[imgBufferSize];
  unsigned int camera1DataCounter = 0u;
  unsigned int camera2DataCounter = 0u;
  unsigned int camera3DataCounter = 0u;
  std::function<void(const gz::msgs::Image &)> camera1Callback =
      [&img1, &camera1DataCounter, &imgBufferSize](const gz::msgs::Image & _msg)
  {
    memcpy(img1, _msg.data().c_str(), imgBufferSize);
    camera1DataCounter++;
  };

  std::function<void(const gz::msgs::Image &)> camera2Callback =
      [&img2, &camera2DataCounter, &imgBufferSize](const gz::msgs::Image & _msg)
  {
    memcpy(img2, _msg.data().c_str(), imgBufferSize);
    camera2DataCounter++;
  };
  std::function<void(const gz::msgs::Image &)> camera3Callback =
      [&img3, &camera3DataCounter, &imgBufferSize](const gz::msgs::Image & _msg)
  {
    memcpy(img3, _msg.data().c_str(), imgBufferSize);
    camera3DataCounter++;
  };

  // Subscribe to the camera topic
  gz::transport::Node node;
  node.Subscribe(infoTopic1, camera1InfoCallback);
  node.Subscribe(infoTopic2, camera2InfoCallback);
  node.Subscribe(infoTopic3, camera3InfoCallback);
  node.Subscribe(imgTopic1, camera1Callback);
  node.Subscribe(imgTopic2, camera2Callback);
  node.Subscribe(imgTopic3, camera3Callback);

  // Wait for a few camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // Run to get image and check image format in callback
  bool done = false;
  int sleep = 0;
  int maxSleep = 10;
  while (!done && sleep++ < maxSleep)
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    done = (camera1Counter > 0u && camera2Counter > 0u && camera3Counter > 0u &&
        camera1DataCounter > 0u && camera2DataCounter > 0u &&
        camera3DataCounter > 0u);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Image size, focal length and optical center
  // Camera sensor without projection tag
  // account for error converting gl projection values back to
  // cv projection values
  double error = 1.0;
  EXPECT_EQ(camera1Info.width(), width);
  EXPECT_EQ(camera1Info.height(), width);
  EXPECT_NEAR(camera1Info.projection().p(0), 866.23, error);
  EXPECT_NEAR(camera1Info.projection().p(5), 866.23, error);
  EXPECT_DOUBLE_EQ(camera1Info.projection().p(2), 500.0);
  EXPECT_DOUBLE_EQ(camera1Info.projection().p(6), 500.0);
  EXPECT_DOUBLE_EQ(camera1Info.projection().p(3), 0.0);
  EXPECT_DOUBLE_EQ(camera1Info.projection().p(7), 0.0);

  // Camera sensor with projection tag
  EXPECT_EQ(camera2Info.width(), height);
  EXPECT_EQ(camera2Info.height(), height);
  EXPECT_DOUBLE_EQ(camera2Info.projection().p(0), 866.23);
  EXPECT_DOUBLE_EQ(camera2Info.projection().p(5), 866.23);
  EXPECT_DOUBLE_EQ(camera2Info.projection().p(2), 500.0);
  EXPECT_DOUBLE_EQ(camera2Info.projection().p(6), 500.0);
  EXPECT_DOUBLE_EQ(camera2Info.projection().p(3), 300.0);
  EXPECT_DOUBLE_EQ(camera2Info.projection().p(7), 200.0);

  // Camera sensor with different projection tag
  EXPECT_EQ(camera3Info.width(), width);
  EXPECT_EQ(camera3Info.height(), height);
  EXPECT_DOUBLE_EQ(camera3Info.projection().p(0), 900.0);
  EXPECT_DOUBLE_EQ(camera3Info.projection().p(5), 900.0);
  EXPECT_DOUBLE_EQ(camera3Info.projection().p(2), 501.0);
  EXPECT_DOUBLE_EQ(camera3Info.projection().p(6), 501.0);
  EXPECT_DOUBLE_EQ(camera3Info.projection().p(3), 0.0);
  EXPECT_DOUBLE_EQ(camera3Info.projection().p(7), 0.0);

  unsigned int r1Sum = 0u;
  unsigned int g1Sum = 0u;
  unsigned int b1Sum = 0u;
  unsigned int r2Sum = 0u;
  unsigned int g2Sum = 0u;
  unsigned int b2Sum = 0u;
  unsigned int r3Sum = 0u;
  unsigned int g3Sum = 0u;
  unsigned int b3Sum = 0u;
  unsigned int step = width * bpp;

  // get sum of each color channel
  // all cameras should just see blue colors
  for (unsigned int i = 0u; i < height; ++i)
  {
    for (unsigned int j = 0u; j < step; j+=bpp)
    {
      unsigned int idx = i * step + j;
      unsigned int r1 = img1[idx];
      unsigned int g1 = img1[idx+1];
      unsigned int b1 = img1[idx+2];
      r1Sum += r1;
      g1Sum += g1;
      b1Sum += b1;

      unsigned int r2 = img2[idx];
      unsigned int g2 = img2[idx+1];
      unsigned int b2 = img2[idx+2];
      r2Sum += r2;
      g2Sum += g2;
      b2Sum += b2;

      unsigned int r3 = img3[idx];
      unsigned int g3 = img3[idx+1];
      unsigned int b3 = img3[idx+2];
      r3Sum += r3;
      g3Sum += g3;
      b3Sum += b3;
    }
  }

  // images from camera1 without projections params and
  // camera2 with default projection params should be the same
  EXPECT_EQ(0u, r1Sum);
  EXPECT_EQ(0u, g1Sum);
  EXPECT_LT(0u, b1Sum);
  EXPECT_EQ(r1Sum, r2Sum);
  EXPECT_EQ(g1Sum, g2Sum);
  EXPECT_EQ(b1Sum, b2Sum);

  // images from camera2 and camera3 that have different projections params
  // should be different. Both cameras should still see the blue box but
  // sum of blue pixels should be slightly different
  EXPECT_EQ(0u, r3Sum);
  EXPECT_EQ(0u, g3Sum);
  EXPECT_LT(0u, b3Sum);
  EXPECT_EQ(r2Sum, r3Sum);
  EXPECT_EQ(g2Sum, g3Sum);
  EXPECT_NE(b2Sum, b3Sum);

  delete []  img1;
  delete []  img2;
  delete []  img3;

  // Clean up rendering ptrs
  box.reset();
  blue.reset();

  // Clean up
  mgr.Remove(sensor1->Id());
  mgr.Remove(sensor2->Id());
  mgr.Remove(sensor3->Id());
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
void CameraSensorTest::ImageFormatLInt8LInt16(const std::string &_renderEngine)
{
  // get the darn test data
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "camera_sensor_l8_l16_builtin.sdf");
  sdf::SDFPtr doc(new sdf::SDF());
  sdf::init(doc);
  ASSERT_TRUE(sdf::readFile(path, doc));
  ASSERT_NE(nullptr, doc->Root());
  ASSERT_TRUE(doc->Root()->HasElement("model"));
  auto modelPtr = doc->Root()->GetElement("model");
  ASSERT_TRUE(modelPtr->HasElement("link"));
  auto linkPtr = modelPtr->GetElement("link");
  ASSERT_TRUE(linkPtr->HasElement("sensor"));
  auto sensorPtrCamera8Bit = linkPtr->GetElement("sensor");
  auto sensorPtrCamera16Bit = linkPtr->GetElement("sensor")->GetNextElement();

  // Setup gz-rendering with an empty scene
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");

  // do the test
  gz::sensors::Manager mgr;

  gz::sensors::CameraSensor *sensorCamera8Bit =
      mgr.CreateSensor<gz::sensors::CameraSensor>(sensorPtrCamera8Bit);
  gz::sensors::CameraSensor *sensorCamera16Bit =
      mgr.CreateSensor<gz::sensors::CameraSensor>(sensorPtrCamera16Bit);
  ASSERT_NE(sensorCamera8Bit, nullptr);
  ASSERT_NE(sensorCamera16Bit, nullptr);
  sensorCamera8Bit->SetScene(scene);
  sensorCamera16Bit->SetScene(scene);

  ASSERT_NE(sensorCamera8Bit->RenderingCamera(), nullptr);
  EXPECT_NE(sensorCamera8Bit->Id(), sensorCamera8Bit->RenderingCamera()->Id());
  EXPECT_EQ(256u, sensorCamera8Bit->ImageWidth());
  EXPECT_EQ(256u, sensorCamera8Bit->ImageHeight());

  ASSERT_NE(sensorCamera16Bit->RenderingCamera(), nullptr);
  EXPECT_NE(sensorCamera16Bit->Id(),
            sensorCamera16Bit->RenderingCamera()->Id());
  EXPECT_EQ(512u, sensorCamera16Bit->ImageWidth());
  EXPECT_EQ(512u, sensorCamera16Bit->ImageHeight());

  std::string topicCamera8Bit = "/images_l8";
  WaitForMessageTestHelper<gz::msgs::Image> helper1(topicCamera8Bit);

  std::string topicCamera16Bit = "/images_l16";
  WaitForMessageTestHelper<gz::msgs::Image> helper2(topicCamera16Bit);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper1.WaitForMessage()) << helper1;
  EXPECT_TRUE(helper2.WaitForMessage()) << helper2;

  // subscribe to the camera topic
  gz::transport::Node node;
  node.Subscribe(topicCamera8Bit, &OnGrayscaleImageL8);
  node.Subscribe(topicCamera16Bit, &OnGrayscaleImageL16);

  // wait for a few camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // run to get image and check image format in callback
  bool done = false;
  int sleep = 0;
  int maxSleep = 10;
  while (!done && sleep++ < maxSleep)
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    done = (g_imgCounter1 > 0 && g_imgCounter2 > 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // test removing sensor
  // first make sure the sensor objects do exist
  auto sensorIdCamera8Bit = sensorCamera8Bit->Id();
  auto sensorIdCamera16Bit = sensorCamera16Bit->Id();
  auto cameraId1 = sensorCamera8Bit->RenderingCamera()->Id();
  auto cameraId2 = sensorCamera16Bit->RenderingCamera()->Id();

  EXPECT_EQ(sensorCamera8Bit, mgr.Sensor(sensorIdCamera8Bit));
  EXPECT_EQ(sensorCamera8Bit->RenderingCamera(), scene->SensorById(cameraId1));
  EXPECT_EQ(sensorCamera16Bit, mgr.Sensor(sensorIdCamera16Bit));
  EXPECT_EQ(sensorCamera16Bit->RenderingCamera(), scene->SensorById(cameraId2));
  // remove and check sensor objects no longer exist in both sensors and
  // rendering
  EXPECT_TRUE(mgr.Remove(sensorIdCamera8Bit));
  EXPECT_TRUE(mgr.Remove(sensorIdCamera16Bit));
  EXPECT_EQ(nullptr, mgr.Sensor(sensorIdCamera8Bit));
  EXPECT_EQ(nullptr, mgr.Sensor(sensorIdCamera16Bit));
  EXPECT_EQ(nullptr, scene->SensorById(cameraId1));
  EXPECT_EQ(nullptr, scene->SensorById(cameraId2));

  // Clean up
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(CameraSensorTest, LInt8LInt16ImagesWithBuiltinSDF)
{
  gz::common::Console::SetVerbosity(4);
  ImageFormatLInt8LInt16(GetParam());
}

//////////////////////////////////////////////////
void CameraSensorTest::CustomCameraInfoTopic(
    const std::string &_renderEngine)
{
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "custom_camera_info_topic.sdf");
  sdf::SDFPtr doc(new sdf::SDF());
  sdf::init(doc);
  ASSERT_TRUE(sdf::readFile(path, doc));
  ASSERT_NE(nullptr, doc->Root());
  ASSERT_TRUE(doc->Root()->HasElement("model"));
  auto modelPtr = doc->Root()->GetElement("model");
  ASSERT_TRUE(modelPtr->HasElement("link"));
  auto linkPtr = modelPtr->GetElement("link");
  while (linkPtr)
  {
    if(linkPtr->Get<std::string>("name") == "camera_link")
    {
      break;
    }
    linkPtr = linkPtr->GetNextElement("link");
  }
  ASSERT_NE(nullptr, linkPtr);
  ASSERT_TRUE(linkPtr->HasElement("sensor"));
  auto sensorPtr = linkPtr->GetElement("sensor");
  ASSERT_NE(nullptr, sensorPtr);

  // Setup gz-rendering with an empty scene.
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
                 << "' is not supported" << std::endl;
  }

  auto scene = engine->CreateScene("scene");

  gz::sensors::Manager mgr;
  auto *sensor =
      mgr.CreateSensor<gz::sensors::CameraSensor>(sensorPtr);

  ASSERT_NE(nullptr, sensor);
  sensor->SetScene(scene);

  const std::string customInfoTopic = "/test_camera_info";
  WaitForMessageTestHelper<gz::msgs::CameraInfo> infoHelper(
      customInfoTopic);

  EXPECT_TRUE(sensor->HasConnections());

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  EXPECT_TRUE(infoHelper.WaitForMessage()) << infoHelper;

  // Clean up
  mgr.Remove(sensor->Id());
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(CameraSensorTest, CustomCameraInfoTopic)
{
  gz::common::Console::SetVerbosity(4);
  CustomCameraInfoTopic(GetParam());
}

//////////////////////////////////////////////////
void CameraSensorTest::PublishedBytesMatchCopy(
    const std::string &_renderEngine, const std::string &_format)
{
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "camera_sensor_builtin.sdf");
  sdf::SDFPtr doc(new sdf::SDF());
  sdf::init(doc);
  ASSERT_TRUE(sdf::readFile(path, doc));
  auto sensorPtr = doc->Root()->GetElement("model")->GetElement("link")
      ->GetElement("sensor");
  ASSERT_NE(nullptr, sensorPtr);
  if (!_format.empty())
  {
    sensorPtr->GetElement("camera")->GetElement("image")
        ->GetElement("format")->Set<std::string>(_format);
  }

  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
                 << "' is not supported" << std::endl;
  }
  gz::rendering::ScenePtr scene = engine->CreateScene("scene");
  scene->SetAmbientLight(1, 1, 1);
  scene->SetBackgroundColor(0.2, 0.3, 0.4);
  // A red box in front of the camera (at the origin, looking along +X)
  gz::rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetWorldPosition(2.0, 0.3, 0.0);
  gz::rendering::MaterialPtr red = scene->CreateMaterial();
  red->SetAmbient(1.0, 0.0, 0.0);
  red->SetDiffuse(1.0, 0.0, 0.0);
  box->SetMaterial(red);
  scene->RootVisual()->AddChild(box);

  gz::sensors::Manager mgr;
  auto *sensor = mgr.CreateSensor<gz::sensors::CameraSensor>(sensorPtr);
  ASSERT_NE(nullptr, sensor);
  sensor->SetScene(scene);

  // The serialized messages, as they leave the publisher. Shared with the
  // callback, which runs on a transport thread and may outlive this scope
  // if an assertion returns early.
  struct Received
  {
    std::mutex mutex;
    std::vector<std::string> raw;
  };
  auto received = std::make_shared<Received>();
  gz::transport::Node node;
  ASSERT_TRUE(node.SubscribeRaw(sensor->Topic(),
      [received](const char *_data, std::size_t _size,
          const gz::transport::MessageInfo &)
      {
        std::lock_guard<std::mutex> lock(received->mutex);
        received->raw.emplace_back(_data, _size);
      }, "gz.msgs.Image"));
  auto waitFor = [&](std::size_t _n)
  {
    for (int i = 0; i < 300; ++i)
    {
      {
        std::lock_guard<std::mutex> lock(received->mutex);
        if (received->raw.size() >= _n)
          return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  };
  ASSERT_TRUE(sensor->HasConnections());

  // Frame 0 takes the copying path: an image callback makes the sensor
  // fill the message and Publish() it.
  const auto stamp = std::chrono::seconds(1) + std::chrono::milliseconds(5);
  gz::msgs::Image copied;
  auto connection = sensor->ConnectImageCallback(
      [&copied](const gz::msgs::Image &_msg) { copied = _msg; });
  EXPECT_TRUE(sensor->Update(stamp));
  ASSERT_TRUE(waitFor(1u));
  connection.reset();

  // Frame 1 still takes the copying path: the event drops a disconnected
  // callback only when it next signals, until then it counts it
  EXPECT_TRUE(sensor->Update(stamp));
  ASSERT_TRUE(waitFor(2u));

  // Frame 2: no image callback, so it is published in place
  EXPECT_TRUE(sensor->Update(stamp));
  ASSERT_TRUE(waitFor(3u));
  // frame 2, read again from the render target
  const gz::rendering::PixelFormat format =
      sensor->RenderingCamera()->ImageFormat();
  std::string frame(sensor->RenderingCamera()->ImageMemorySize(), '\0');
  ASSERT_TRUE(sensor->RenderingCamera()->CopyTo(gz::rendering::PixelBuffer(
      256u, 257u, format, frame.data(), frame.size())));

  std::lock_guard<std::mutex> lock(received->mutex);
  const std::vector<std::string> &raw = received->raw;
  ASSERT_EQ(3u, raw.size());
  EXPECT_EQ(copied.SerializeAsString(), raw[0]);
  if (_format == "L8")
  {
    EXPECT_EQ(gz::msgs::PixelFormatType::L_INT8, copied.pixel_format_type());
  }
  // a frame with an image, not a uniform one
  const unsigned int bpp = gz::rendering::PixelUtil::BytesPerPixel(format);
  ASSERT_EQ(256u * 257u * bpp, frame.size());
  bool uniform = true;
  for (std::size_t i = bpp; uniform && i < frame.size(); i += bpp)
    uniform = frame.compare(i, bpp, frame, 0, bpp) == 0;
  EXPECT_FALSE(uniform);

  // Frame 2 is Publish() of frame 0's message with sequence number 2 and
  // frame 2's data
  gz::msgs::Image expected = copied;
  expected.set_data(frame);
  bool seqFound = false;
  for (auto &data : *expected.mutable_header()->mutable_data())
  {
    if (data.key() == "seq")
    {
      EXPECT_EQ("0", data.value(0));
      data.set_value(0, "2");
      seqFound = true;
    }
  }
  EXPECT_TRUE(seqFound);
  const std::string expectedBytes = expected.SerializeAsString();
  ASSERT_EQ(expectedBytes.size(), raw[2].size());
  std::size_t firstDiff = 0;
  while (firstDiff < raw[2].size() &&
         raw[2][firstDiff] == expectedBytes[firstDiff])
  {
    ++firstDiff;
  }
  EXPECT_EQ(raw[2].size(), firstDiff) << "first differing byte";

  mgr.Remove(sensor->Id());
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(CameraSensorTest, PublishedBytesMatchCopy)
{
  PublishedBytesMatchCopy(GetParam(), "");
}

//////////////////////////////////////////////////
// L8: one byte per pixel, and a pixel_format_type other than RGB_INT8 after
// the data
TEST_P(CameraSensorTest, PublishedBytesMatchCopyL8)
{
  PublishedBytesMatchCopy(GetParam(), "L8");
}

INSTANTIATE_TEST_SUITE_P(CameraSensor, CameraSensorTest,
    RENDER_ENGINE_VALUES, gz::rendering::PrintToStringParam());
