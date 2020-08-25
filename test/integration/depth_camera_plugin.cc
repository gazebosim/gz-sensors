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
#include <ignition/msgs/camera_info.pb.h>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Event.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/sensors/DepthCameraSensor.hh>
#include <ignition/rendering.hh>
#include <ignition/msgs.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "TransportTestTools.hh"

#include "PointCloudUtil.hh"

#define DEPTH_TOL 1e-4
#define DOUBLE_TOL 1e-6

std::mutex g_mutex;
unsigned int g_depthCounter = 0;
float *g_depthBuffer = nullptr;

std::mutex g_infoMutex;
unsigned int g_infoCounter = 0;
ignition::msgs::CameraInfo g_infoMsg;

std::mutex g_pcMutex;
unsigned int g_pcCounter = 0;
float *g_pointsXYZBuffer = nullptr;
unsigned char *g_pointsRGBBuffer = nullptr;

void UnpackPointCloudMsg(const ignition::msgs::PointCloudPacked &_msg,
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

void OnCameraInfo(const ignition::msgs::CameraInfo & _msg)
{
  g_infoMutex.lock();
  g_infoCounter++;
  g_infoMsg.CopyFrom(_msg);
  g_infoMutex.unlock();
}

void OnImage(const ignition::msgs::Image &_msg)
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

void OnPointCloud(const ignition::msgs::PointCloudPacked &_msg)
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
  if ((_renderEngine.compare("ogre") != 0) &&
      (_renderEngine.compare("ogre2") != 0))
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
  mgr.AddPluginPaths(ignition::common::joinPaths(PROJECT_BUILD_PATH, "lib"));

  ignition::sensors::DepthCameraSensor *depthSensor =
      mgr.CreateSensor<ignition::sensors::DepthCameraSensor>(sensorPtr);
  ASSERT_NE(depthSensor, nullptr);
  depthSensor->SetScene(scene);

  EXPECT_EQ(depthSensor->ImageWidth(), static_cast<unsigned int>(imgWidth));
  EXPECT_EQ(depthSensor->ImageHeight(), static_cast<unsigned int>(imgHeight));
  EXPECT_NEAR(depthSensor->FarClip(), far_, DOUBLE_TOL);
  EXPECT_NEAR(depthSensor->NearClip(), near_, DOUBLE_TOL);

  std::string topic =
    "/test/integration/DepthCameraPlugin_imagesWithBuiltinSDF/image";
  WaitForMessageTestHelper<ignition::msgs::Image> helper(topic);

  std::string pointsTopic =
    "/test/integration/DepthCameraPlugin_imagesWithBuiltinSDF/image/points";
  WaitForMessageTestHelper<ignition::msgs::PointCloudPacked>
    pointsHelper(pointsTopic);

  std::string infoTopic =
    "/test/integration/DepthCameraPlugin_imagesWithBuiltinSDF/camera_info";
  WaitForMessageTestHelper<ignition::msgs::CameraInfo> infoHelper(infoTopic);

  // Update once to create image
  mgr.RunOnce(std::chrono::system_clock::from_time_t(0));

  EXPECT_TRUE(helper.WaitForMessage()) << helper;
  EXPECT_TRUE(pointsHelper.WaitForMessage()) << pointsHelper;
  EXPECT_TRUE(infoHelper.WaitForMessage()) << infoHelper;

  // subscribe to the depth camera topic
  ignition::transport::Node node;
  node.Subscribe(topic, &OnImage);

  // subscribe to the depth camera points topic
  node.Subscribe(pointsTopic, &OnPointCloud);

  // subscribe to the depth camera topic
  node.Subscribe(infoTopic, &OnCameraInfo);

  // wait for a few depth camera frames
  mgr.RunOnce(std::chrono::system_clock::from_time_t(0), true);

  int midWidth = depthSensor->ImageWidth() * 0.5;
  int midHeight = depthSensor->ImageHeight() * 0.5;
  int mid = midHeight * depthSensor->ImageWidth() + midWidth -1;
  double expectedRangeAtMidPoint = boxPosition.X() - unitBoxSize * 0.5;

  auto waitTime = std::chrono::duration_cast< std::chrono::milliseconds >(
      std::chrono::duration< double >(0.001));
  int counter = 0;
  int infoCounter = 0;
  int pcCounter = 0;
  ignition::msgs::CameraInfo infoMsg;
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
  EXPECT_DOUBLE_EQ(g_depthBuffer[left], ignition::math::INF_D);
  int right = (midHeight+1) * depthSensor->ImageWidth() - 1;
  EXPECT_DOUBLE_EQ(g_depthBuffer[right], ignition::math::INF_D);
  g_infoMutex.unlock();
  g_mutex.unlock();

  // Check camera info
  EXPECT_TRUE(infoMsg.has_header());
  ASSERT_EQ(1, infoMsg.header().data().size());
  EXPECT_EQ("frame_id", infoMsg.header().data(0).key());
  ASSERT_EQ(1, infoMsg.header().data(0).value().size());
  EXPECT_EQ("camera1", infoMsg.header().data(0).value(0));
  EXPECT_TRUE(infoMsg.has_distortion());
  EXPECT_EQ(ignition::msgs::CameraInfo::Distortion::PLUMB_BOB,
      infoMsg.distortion().model());
  EXPECT_EQ(5, infoMsg.distortion().k().size());
  EXPECT_TRUE(infoMsg.has_intrinsics());
  EXPECT_EQ(9, infoMsg.intrinsics().k().size());
  EXPECT_TRUE(infoMsg.has_projection());
  EXPECT_EQ(12, infoMsg.projection().p().size());
  EXPECT_EQ(9, infoMsg.rectification_matrix().size());

  // Check that for a box really close it returns -inf
  root->RemoveChild(box);
  ignition::math::Vector3d boxPositionNear(
      unitBoxSize * 0.5 + near_ * 0.5, 0.0, 0.0);
  box->SetLocalPosition(boxPositionNear);
  root->AddChild(box);

  mgr.RunOnce(std::chrono::system_clock::from_time_t(0), true);
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

  EXPECT_DOUBLE_EQ(g_depthBuffer[mid], -ignition::math::INF_D);
  g_infoMutex.unlock();
  g_mutex.unlock();

  // Check that for a box really far it returns inf
  root->RemoveChild(box);
  ignition::math::Vector3d boxPositionFar(
      unitBoxSize * 0.5 + far_ * 1.5, 0.0, 0.0);
  box->SetLocalPosition(boxPositionFar);
  root->AddChild(box);

  mgr.RunOnce(std::chrono::system_clock::from_time_t(0), true);
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

  EXPECT_DOUBLE_EQ(g_depthBuffer[mid], ignition::math::INF_D);
  g_infoMutex.unlock();
  g_mutex.unlock();


  // Check that the depth values for a box do not warp.
  root->RemoveChild(box);
  ignition::math::Vector3d boxPositionFillFrame(
      unitBoxSize * 0.5 + 0.2, 0.0, 0.0);
  box->SetLocalPosition(boxPositionFillFrame);
  root->AddChild(box);

  mgr.RunOnce(std::chrono::system_clock::from_time_t(0), true);
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
  EXPECT_EQ(counter, pcCounter);
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

  // Clean up
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
