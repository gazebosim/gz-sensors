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

#include <cstring>
#include <string>
#include <gtest/gtest.h>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>

#include <gz/common/Filesystem.hh>
#include <gz/common/Event.hh>
#include <gz/sensors/Manager.hh>
#include <gz/sensors/DepthCameraSensor.hh>

// TODO(louise) Remove these pragmas once gz-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include "test_config.hh"  // NOLINT(build/include)
#include "TransportTestTools.hh"

#include "PointCloudUtil.hh"

#define DEPTH_TOL 1e-4
#define DOUBLE_TOL 1e-6

std::mutex g_mutex;
unsigned int g_depthCounter = 0;
float *g_depthBuffer = nullptr;

std::mutex g_infoMutex;
unsigned int g_infoCounter = 0;
gz::msgs::CameraInfo g_infoMsg;

std::mutex g_pcMutex;
unsigned int g_pcCounter = 0;
float *g_pointsXYZBuffer = nullptr;
unsigned char *g_pointsRGBBuffer = nullptr;

void UnpackPointCloudMsg(const gz::msgs::PointCloudPacked &_msg,
  float *_xyzBuffer, unsigned char *_rgbBuffer)
{
  std::string msgBuffer = _msg.data();
  char *msgBufferIndex = msgBuffer.data();

  for (uint32_t j = 0; j < _msg.height(); ++j)
  {
    for (uint32_t i = 0; i < _msg.width(); ++i)
    {
      int fieldIndex = 0;
      int pointIndex = j*_msg.width()*3 + i*3;

      _xyzBuffer[pointIndex] =  *reinterpret_cast<float *>(
        msgBufferIndex + _msg.field(fieldIndex++).offset());
      _xyzBuffer[pointIndex + 1] = *reinterpret_cast<float *>(
        msgBufferIndex + _msg.field(fieldIndex++).offset());
      _xyzBuffer[pointIndex + 2] = *reinterpret_cast<float *>(
        msgBufferIndex + _msg.field(fieldIndex++).offset());

      int fieldOffset = _msg.field(fieldIndex).offset();
      if (_msg.is_bigendian())
      {
        _rgbBuffer[pointIndex] = *(msgBufferIndex + fieldOffset + 0);
        _rgbBuffer[pointIndex + 1] = *(msgBufferIndex + fieldOffset + 1);
        _rgbBuffer[pointIndex + 2] = *(msgBufferIndex + fieldOffset + 2);
      }
      else
      {
        _rgbBuffer[pointIndex] = *(msgBufferIndex + fieldOffset + 2);
        _rgbBuffer[pointIndex + 1] = *(msgBufferIndex + fieldOffset + 1);
        _rgbBuffer[pointIndex + 2] = *(msgBufferIndex + fieldOffset + 0);
      }
      msgBufferIndex += _msg.point_step();
    }
  }
}

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
  unsigned int depthSamples = _msg.width() * _msg.height();
  unsigned int depthBufferSize = depthSamples * sizeof(float);
  if (!g_depthBuffer)
    g_depthBuffer = new float[depthSamples];
  memcpy(g_depthBuffer, _msg.data().c_str(), depthBufferSize);
  g_depthCounter++;
  g_mutex.unlock();
}

void OnPointCloud(const gz::msgs::PointCloudPacked &_msg)
{
  g_pcMutex.lock();

  unsigned int pointCloudSamples = _msg.width() * _msg.height();
  unsigned int pointCloudBufferSize = pointCloudSamples * 3;
  if (!g_pointsXYZBuffer)
    g_pointsXYZBuffer = new float[pointCloudBufferSize];
  if (!g_pointsRGBBuffer)
    g_pointsRGBBuffer = new unsigned char[pointCloudBufferSize];

  UnpackPointCloudMsg(_msg, g_pointsXYZBuffer, g_pointsRGBBuffer);

  g_pcCounter++;
  g_pcMutex.unlock();
}

class DepthCameraSensorTest: public testing::Test,
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
  }
  // Create a Camera sensor from a SDF and gets a image message
  public: void ImagesWithBuiltinSDF(const std::string &_renderEngine);

  // Create depth camera sensors and verify camera intrinsics
  public: void DepthCameraIntrinsics(const std::string &_renderEngine);

  // Create depth camera sensors and verify camera projection
  public: void DepthCameraProjection(const std::string &_renderEngine);
};

void DepthCameraSensorTest::ImagesWithBuiltinSDF(
    const std::string &_renderEngine)
{
  // get the darn test data
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "depth_camera_sensor_builtin.sdf");
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
  gz::math::Vector3d boxPosition(3.0, 0.0, 0.0);

  // If ogre is not the engine, don't run the test
  if ((_renderEngine.compare("ogre") != 0) &&
      (_renderEngine.compare("ogre2") != 0))
  {
    gzdbg << "Engine '" << _renderEngine
              << "' doesn't support depth cameras" << std::endl;
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

  // Create an scene with a box in it
  scene->SetAmbientLight(0.3, 0.3, 0.3);
  gz::rendering::VisualPtr root = scene->RootVisual();

  // create blue material
  gz::rendering::MaterialPtr blue = scene->CreateMaterial();
  blue->SetAmbient(0.0, 0.0, 0.3);
  blue->SetDiffuse(0.0, 0.0, 0.8);
  blue->SetSpecular(0.5, 0.5, 0.5);
  blue->SetShininess(50);
  blue->SetReflectivity(0);

  // create box visual
  gz::rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(boxPosition);
  box->SetLocalRotation(0, 0, 0);
  box->SetLocalScale(unitBoxSize, unitBoxSize, unitBoxSize);
  box->SetMaterial(blue);
  scene->DestroyMaterial(blue);
  root->AddChild(box);

  // do the test
  gz::sensors::Manager mgr;

  gz::sensors::DepthCameraSensor *depthSensor =
      mgr.CreateSensor<gz::sensors::DepthCameraSensor>(sensorPtr);
  ASSERT_NE(depthSensor, nullptr);
  EXPECT_FALSE(depthSensor->HasConnections());
  depthSensor->SetScene(scene);

  EXPECT_EQ(depthSensor->ImageWidth(), static_cast<unsigned int>(imgWidth));
  EXPECT_EQ(depthSensor->ImageHeight(), static_cast<unsigned int>(imgHeight));
  EXPECT_NEAR(depthSensor->FarClip(), far_, DOUBLE_TOL);
  EXPECT_NEAR(depthSensor->NearClip(), near_, DOUBLE_TOL);

  std::string topic =
    "/test/integration/DepthCameraPlugin_imagesWithBuiltinSDF/image";
  WaitForMessageTestHelper<gz::msgs::Image> helper(topic);
  EXPECT_TRUE(depthSensor->HasConnections());

  std::string pointsTopic =
    "/test/integration/DepthCameraPlugin_imagesWithBuiltinSDF/image/points";
  WaitForMessageTestHelper<gz::msgs::PointCloudPacked>
    pointsHelper(pointsTopic);

  std::string infoTopic =
    "/test/integration/DepthCameraPlugin_imagesWithBuiltinSDF/camera_info";
  WaitForMessageTestHelper<gz::msgs::CameraInfo> infoHelper(infoTopic);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(helper.WaitForMessage()) << helper;
  EXPECT_TRUE(pointsHelper.WaitForMessage()) << pointsHelper;
  EXPECT_TRUE(infoHelper.WaitForMessage()) << infoHelper;

  // subscribe to the depth camera topic
  gz::transport::Node node;
  node.Subscribe(topic, &OnImage);

  // subscribe to the depth camera points topic
  node.Subscribe(pointsTopic, &OnPointCloud);

  // subscribe to the depth camera topic
  node.Subscribe(infoTopic, &OnCameraInfo);

  // wait for a few depth camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  int midWidth = static_cast<int>(depthSensor->ImageWidth() * 0.5);
  int midHeight = static_cast<int>(depthSensor->ImageHeight() * 0.5);
  int mid = midHeight * depthSensor->ImageWidth() + midWidth -1;
  double expectedRangeAtMidPoint = boxPosition.X() - unitBoxSize * 0.5;

  auto waitTime = std::chrono::duration_cast< std::chrono::milliseconds >(
      std::chrono::duration< double >(0.001));
  int counter = 0;
  int infoCounter = 0;
  int pcCounter = 0;
  gz::msgs::CameraInfo infoMsg;
  for (int sleep = 0;
       sleep < 300 && (counter == 0 || infoCounter == 0); ++sleep)
  {
    g_mutex.lock();
    counter = g_depthCounter;
    g_mutex.unlock();

    g_infoMutex.lock();
    infoCounter = g_infoCounter;
    infoMsg = g_infoMsg;
    g_infoMutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }
  g_mutex.lock();
  g_infoMutex.lock();
  g_depthCounter = 0;
  g_infoCounter = 0;
  g_pcCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_EQ(counter, infoCounter);
  counter = 0;
  infoCounter = 0;
  pcCounter = 0;

  EXPECT_NEAR(g_depthBuffer[mid], expectedRangeAtMidPoint, DEPTH_TOL);
  // Depth sensor should see box in the middle of the image
  EXPECT_NEAR(g_depthBuffer[mid], expectedRangeAtMidPoint, DEPTH_TOL);

  // The left and right side of the depth frame should be inf
  int left = midHeight * depthSensor->ImageWidth();
  EXPECT_DOUBLE_EQ(g_depthBuffer[left], gz::math::INF_D);
  int right = (midHeight+1) * depthSensor->ImageWidth() - 1;
  EXPECT_DOUBLE_EQ(g_depthBuffer[right], gz::math::INF_D);
  g_infoMutex.unlock();
  g_mutex.unlock();

  // Check camera info
  EXPECT_TRUE(infoMsg.has_header());
  ASSERT_EQ(1, infoMsg.header().data().size());
  EXPECT_EQ("frame_id", infoMsg.header().data(0).key());
  ASSERT_EQ(1, infoMsg.header().data(0).value().size());
  EXPECT_EQ("camera1", infoMsg.header().data(0).value(0));
  EXPECT_TRUE(infoMsg.has_distortion());
  EXPECT_EQ(gz::msgs::CameraInfo::Distortion::PLUMB_BOB,
      infoMsg.distortion().model());
  EXPECT_EQ(5, infoMsg.distortion().k().size());
  EXPECT_TRUE(infoMsg.has_intrinsics());
  EXPECT_EQ(9, infoMsg.intrinsics().k().size());
  EXPECT_TRUE(infoMsg.has_projection());
  EXPECT_EQ(12, infoMsg.projection().p().size());
  EXPECT_EQ(9, infoMsg.rectification_matrix().size());

  // Check that for a box really close it returns -inf
  gz::math::Vector3d boxPositionNear(
      unitBoxSize * 0.5 + near_ * 0.5, 0.0, 0.0);
  box->SetLocalPosition(boxPositionNear);

  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  for (int sleep = 0;
       sleep < 300 && (counter == 0 || infoCounter == 0); ++sleep)
  {
    g_mutex.lock();
    counter = g_depthCounter;
    g_mutex.unlock();

    g_infoMutex.lock();
    infoCounter = g_infoCounter;
    g_infoMutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }

  g_mutex.lock();
  g_infoMutex.lock();
  g_depthCounter = 0;
  g_infoCounter = 0;
  g_pcCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_EQ(counter, infoCounter);
  counter = 0;
  infoCounter = 0;
  pcCounter = 0;

  EXPECT_DOUBLE_EQ(g_depthBuffer[mid], -gz::math::INF_D);
  g_infoMutex.unlock();
  g_mutex.unlock();

  // Check that for a box really far it returns inf
  gz::math::Vector3d boxPositionFar(
      unitBoxSize * 0.5 + far_ * 1.5, 0.0, 0.0);
  box->SetLocalPosition(boxPositionFar);

  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  for (int sleep = 0;
       sleep < 300 && (counter == 0 || infoCounter == 0); ++sleep)
  {
    g_mutex.lock();
    counter = g_depthCounter;
    g_mutex.unlock();

    g_infoMutex.lock();
    infoCounter = g_infoCounter;
    g_infoMutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }
  g_mutex.lock();
  g_infoMutex.lock();
  g_depthCounter = 0;
  g_infoCounter = 0;
  g_pcCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_EQ(counter, infoCounter);
  counter = 0;
  infoCounter = 0;
  pcCounter = 0;

  EXPECT_DOUBLE_EQ(g_depthBuffer[mid], gz::math::INF_D);
  g_infoMutex.unlock();
  g_mutex.unlock();


  // Check that the depth values for a box do not warp.
  gz::math::Vector3d boxPositionFillFrame(
      unitBoxSize * 0.5 + 0.2, 0.0, 0.0);
  box->SetLocalPosition(boxPositionFillFrame);

  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  for (int sleep = 0;
       sleep < 300 && (counter == 0 || infoCounter == 0 || pcCounter == 0);
       ++sleep)
  {
    g_mutex.lock();
    counter = g_depthCounter;
    g_mutex.unlock();

    g_infoMutex.lock();
    infoCounter = g_infoCounter;
    g_infoMutex.unlock();

    g_pcMutex.lock();
    pcCounter = g_pcCounter;
    g_pcMutex.unlock();

    std::this_thread::sleep_for(waitTime);
  }
  g_mutex.lock();
  g_infoMutex.lock();
  g_pcMutex.lock();
  g_depthCounter = 0;
  g_infoCounter = 0;
  g_pcCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_GT(pcCounter, 0);
  EXPECT_EQ(counter, infoCounter);
  EXPECT_LE(std::abs(counter - pcCounter), 1);
  counter = 0;
  infoCounter = 0;
  pcCounter = 0;

  double expectedDepth = boxPositionFillFrame.X() - unitBoxSize * 0.5;
  // Verify Depth
  {
    // all points should have the same depth
    for (unsigned int i = 0; i < depthSensor->ImageHeight(); ++i)
    {
      unsigned int step = i*depthSensor->ImageWidth();
      for (unsigned int j = 0; j < depthSensor->ImageWidth(); ++j)
      {
        float d = g_depthBuffer[step + j];
        EXPECT_NEAR(expectedDepth, d, DOUBLE_TOL);
      }
    }
  }
  // Verify Point Cloud XYZ
  {
    // all points should have the same X value
    for (unsigned int i = 0; i < depthSensor->ImageHeight(); ++i)
    {
      unsigned int step = i*depthSensor->ImageWidth()*3;
      for (unsigned int j = 0; j < depthSensor->ImageWidth(); ++j)
      {
        float x = g_pointsXYZBuffer[step + j*3];
        EXPECT_NEAR(expectedDepth, x, DOUBLE_TOL);
      }
    }

    // Verify Point Cloud RGB
    // all points should be the same
    for (unsigned int i = 0; i < depthSensor->ImageHeight(); ++i)
    {
      unsigned int step = i*depthSensor->ImageWidth()*3;
      for (unsigned int j = 0; j < depthSensor->ImageWidth(); ++j)
      {
        unsigned int r =
            static_cast<unsigned int>(g_pointsRGBBuffer[step + j*3]);
        unsigned int g =
            static_cast<unsigned int>(g_pointsRGBBuffer[step + j*3 + 1]);
        unsigned int b =
            static_cast<unsigned int>(g_pointsRGBBuffer[step + j*3 + 2]);
        EXPECT_EQ(g, r);
        EXPECT_EQ(b, g);
      }
    }
  }

  g_infoMutex.unlock();
  g_mutex.unlock();
  g_pcMutex.unlock();

  // clean up rendering ptrs
  blue.reset();
  box.reset();

  // Clean up
  mgr.Remove(depthSensor->Id());
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(DepthCameraSensorTest, ImagesWithBuiltinSDF)
{
  ImagesWithBuiltinSDF(GetParam());
  gz::common::Console::SetVerbosity(4);
}

//////////////////////////////////////////////////
void DepthCameraSensorTest::DepthCameraIntrinsics(
    const std::string &_renderEngine)
{
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "depth_camera_intrinsics.sdf");
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
    gzdbg << "Engine '" << _renderEngine
          << "' is not supported" << std::endl;
    return;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");
  gz::rendering::VisualPtr root = scene->RootVisual();
  ASSERT_NE(nullptr, root);

  // create box visual
  gz::rendering::VisualPtr box = scene->CreateVisual("box");
  ASSERT_NE(nullptr, box);
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(gz::math::Vector3d(4.0, 1, 0.5));
  box->SetLocalRotation(0, 0, 0);
  root->AddChild(box);

  // Do the test
  gz::sensors::Manager mgr;

  // there are 3 cameras:
  //     - camera1: no intrinsics params
  //     - camera2: has intrinsics params that are the same as default values
  //     - camera3: has intrinsics params that are different from default values
  // camera1 and camera2 should produce very similar images and camera3 should
  // produce different images from 1 and 2.
  auto *sensor1 = mgr.CreateSensor<gz::sensors::DepthCameraSensor>(
      cameraWithoutIntrinsicsTag);
  auto *sensor2 = mgr.CreateSensor<gz::sensors::DepthCameraSensor>(
      cameraWithIntrinsicsTag);
  auto *sensor3 = mgr.CreateSensor<gz::sensors::DepthCameraSensor>(
      cameraWithDiffIntrinsicsTag);

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
  // Subscribe to the camera topic
  gz::transport::Node node;
  node.Subscribe(infoTopic1, camera1InfoCallback);
  node.Subscribe(infoTopic2, camera2InfoCallback);
  node.Subscribe(infoTopic3, camera3InfoCallback);

  // Wait for a few camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // Run to get image and check image format in callback
  bool done = false;
  int sleep = 0;
  int maxSleep = 10;
  while (!done && sleep++ < maxSleep)
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    done = (camera1Counter > 0u && camera2Counter > 0u && camera3Counter > 0u);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

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

  // Clean up rendering ptrs
  box.reset();

  // Clean up
  mgr.Remove(sensor1->Id());
  mgr.Remove(sensor2->Id());
  mgr.Remove(sensor3->Id());
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(DepthCameraSensorTest, CameraIntrinsics)
{
  gz::common::Console::SetVerbosity(2);
  DepthCameraIntrinsics(GetParam());
}

//////////////////////////////////////////////////
void DepthCameraSensorTest::DepthCameraProjection(
    const std::string &_renderEngine)
{
  std::string path = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "depth_camera_projection.sdf");
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
    gzdbg << "Engine '" << _renderEngine
          << "' is not supported" << std::endl;
    return;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");
  gz::rendering::VisualPtr root = scene->RootVisual();
  ASSERT_NE(nullptr, root);

  // create box visual
  gz::rendering::VisualPtr box = scene->CreateVisual("box");
  ASSERT_NE(nullptr, box);
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(gz::math::Vector3d(4.0, 1, 0.5));
  box->SetLocalRotation(0, 0, 0);
  root->AddChild(box);

  // Do the test
  gz::sensors::Manager mgr;

  auto *sensor1 = mgr.CreateSensor<gz::sensors::DepthCameraSensor>(
      cameraWithoutProjectionsTag);
  auto *sensor2 = mgr.CreateSensor<gz::sensors::DepthCameraSensor>(
      cameraWithProjectionsTag);
  auto *sensor3 = mgr.CreateSensor<gz::sensors::DepthCameraSensor>(
      cameraWithDiffProjectionsTag);

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

  // Subscribe to the camera topic
  gz::transport::Node node;
  node.Subscribe(infoTopic1, camera1InfoCallback);
  node.Subscribe(infoTopic2, camera2InfoCallback);
  node.Subscribe(infoTopic3, camera3InfoCallback);

  // Wait for a few camera frames
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // Run to get image and check image format in callback
  bool done = false;
  int sleep = 0;
  int maxSleep = 10;
  while (!done && sleep++ < maxSleep)
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    done = (camera1Counter > 0u && camera2Counter > 0u && camera3Counter > 0u);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Camera sensor without projection tag
  // account for error converting gl projection values back to
  // cv projection values
  double error = 1.0;
  EXPECT_EQ(camera1Info.width(), width);
  EXPECT_EQ(camera1Info.height(), height);
  EXPECT_NEAR(camera1Info.projection().p(0), 866.23, error);
  EXPECT_NEAR(camera1Info.projection().p(5), 866.23, error);
  EXPECT_DOUBLE_EQ(camera1Info.projection().p(2), 500.0);
  EXPECT_DOUBLE_EQ(camera1Info.projection().p(6), 500.0);
  EXPECT_DOUBLE_EQ(camera1Info.projection().p(3), 0.0);
  EXPECT_DOUBLE_EQ(camera1Info.projection().p(7), 0.0);

  // Camera sensor with projection tag
  EXPECT_EQ(camera2Info.width(), width);
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

  // Clean up rendering ptrs
  box.reset();

  // Clean up
  mgr.Remove(sensor1->Id());
  mgr.Remove(sensor2->Id());
  mgr.Remove(sensor3->Id());
  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(DepthCameraSensorTest, CameraProjection)
{
  gz::common::Console::SetVerbosity(2);
  DepthCameraProjection(GetParam());
}

INSTANTIATE_TEST_SUITE_P(DepthCameraSensor, DepthCameraSensorTest,
    RENDER_ENGINE_VALUES, gz::rendering::PrintToStringParam());
