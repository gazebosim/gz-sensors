/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <gtest/gtest.h>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/PointCloudPackedUtils.hh>

#include <gz/msgs/Utility.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Event.hh>
#include <gz/sensors/Manager.hh>
#include <gz/sensors/RgbdCameraSensor.hh>
#include <gz/transport/Node.hh>

// TODO(louise) Remove these pragmas once gz-rendering is disabling the
// warnings
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <gz/rendering/DepthCamera.hh>
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

using namespace gz;

std::mutex g_mutex;
unsigned int g_depthCounter = 0;
float *g_depthBuffer = nullptr;

std::mutex g_imgMutex;
unsigned int g_imgCounter = 0;
unsigned char *g_imgBuffer = nullptr;

std::mutex g_pcMutex;
unsigned int g_pcCounter = 0;
float *g_pointsXYZBuffer = nullptr;
unsigned char *g_pointsRGBBuffer = nullptr;


std::mutex g_infoMutex;
unsigned int g_infoCounter = 0;
msgs::CameraInfo g_infoMsg;

void UnpackPointCloudMsg(const msgs::PointCloudPacked &_msg,
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


void OnCameraInfo(const msgs::CameraInfo & _msg)
{
  g_infoMutex.lock();
  g_infoCounter++;
  g_infoMsg.CopyFrom(_msg);
  g_infoMutex.unlock();
}

void OnDepthImage(const msgs::Image &_msg)
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

void OnImage(const msgs::Image &_msg)
{
  g_imgMutex.lock();
  unsigned int imgSize = _msg.width() * _msg.height() * 3;
  if (!g_imgBuffer)
    g_imgBuffer = new unsigned char[imgSize];
  memcpy(g_imgBuffer, _msg.data().c_str(), imgSize);
  g_imgCounter++;
  g_imgMutex.unlock();
}

void OnPointCloud(const msgs::PointCloudPacked &_msg)
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

class RgbdCameraSensorTest: public testing::Test,
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

  // Create a rgbd camera sensor from a SDF and gets a image message
  public: void ImagesWithBuiltinSDF(const std::string &_renderEngine);

  // Create a rgbd camera sensor and verify a custom camera info topic
  public: void CustomCameraInfoTopic(const std::string &_renderEngine);
};

void RgbdCameraSensorTest::ImagesWithBuiltinSDF(
    const std::string &_renderEngine)
{
  // get the darn test data
  std::string path = common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "rgbd_camera_sensor_builtin.sdf");
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
  math::Vector3d boxPosition(3.0, 0.0, 0.0);

  // If ogre is not the engine, don't run the test
  if ((_renderEngine.compare("ogre") != 0) &&
      (_renderEngine.compare("ogre2") != 0))
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
              << "' doesn't support rgbd cameras" << std::endl;
  }

  // Setup gz-rendering with an empty scene
  auto *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    GTEST_SKIP() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
  }

  rendering::ScenePtr scene = engine->CreateScene("scene");

  // Create an scene with a box in it
  scene->SetAmbientLight(0.0, 0.0, 1.0);
  rendering::VisualPtr root = scene->RootVisual();

  // red background
  scene->SetBackgroundColor(1.0, 0.0, 0.0);

  // create blue material
  rendering::MaterialPtr blue = scene->CreateMaterial();
  blue->SetAmbient(0.0, 0.0, 1.0);
  blue->SetDiffuse(0.0, 0.0, 1.0);
  blue->SetSpecular(0.0, 0.0, 1.0);

  // create box visual
  rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(boxPosition);
  box->SetLocalRotation(0, 0, 0);
  box->SetLocalScale(unitBoxSize, unitBoxSize, unitBoxSize);
  box->SetMaterial(blue);
  scene->DestroyMaterial(blue);
  root->AddChild(box);

  // do the test
  sensors::Manager mgr;

  sensors::RgbdCameraSensor *rgbdSensor =
      mgr.CreateSensor<sensors::RgbdCameraSensor>(sensorPtr);
  ASSERT_NE(rgbdSensor, nullptr);
  EXPECT_FALSE(rgbdSensor->HasConnections());
  rgbdSensor->SetScene(scene);

  EXPECT_EQ(rgbdSensor->ImageWidth(), static_cast<unsigned int>(imgWidth));
  EXPECT_EQ(rgbdSensor->ImageHeight(), static_cast<unsigned int>(imgHeight));

  std::string depthTopic =
    "/test/integration/RgbdCameraPlugin_imagesWithBuiltinSDF/depth_image";
  WaitForMessageTestHelper<msgs::Image> depthHelper(depthTopic);
  EXPECT_TRUE(rgbdSensor->HasConnections());

  std::string imageTopic =
    "/test/integration/RgbdCameraPlugin_imagesWithBuiltinSDF/image";
  WaitForMessageTestHelper<msgs::Image> imageHelper(imageTopic);

  std::string pointsTopic =
    "/test/integration/RgbdCameraPlugin_imagesWithBuiltinSDF/points";
  WaitForMessageTestHelper<msgs::PointCloudPacked>
      pointsHelper(pointsTopic);

  std::string infoTopic =
    "/test/integration/RgbdCameraPlugin_imagesWithBuiltinSDF/camera_info";
  WaitForMessageTestHelper<msgs::CameraInfo> infoHelper(infoTopic);

  // Update once to create image
  mgr.RunOnce(std::chrono::steady_clock::duration::zero());

  EXPECT_TRUE(depthHelper.WaitForMessage()) << depthHelper;
  EXPECT_TRUE(imageHelper.WaitForMessage()) << imageHelper;
  EXPECT_TRUE(pointsHelper.WaitForMessage()) << pointsHelper;

  // subscribe to the depth camera topic
  transport::Node node;
  node.Subscribe(depthTopic, &OnDepthImage);

  // subscribe to the image topic
  node.Subscribe(imageTopic, &OnImage);

  // subscribe to the point cloud topic
  node.Subscribe(pointsTopic, &OnPointCloud);

  // subscribe to the camera info topic
  node.Subscribe(infoTopic, &OnCameraInfo);

  // Update once more
  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);

  // wait for an image
  auto waitTime = std::chrono::duration_cast< std::chrono::milliseconds >(
      std::chrono::duration< double >(0.001));
  int counter = 0;
  int infoCounter = 0;
  int imgCounter = 0;
  int pcCounter = 0;
  msgs::CameraInfo infoMsg;
  for (int sleep = 0; sleep < 300 &&
      (counter == 0 || infoCounter == 0 || imgCounter == 0 || pcCounter == 0);
      ++sleep)
  {
    g_mutex.lock();
    g_infoMutex.lock();
    g_imgMutex.lock();
    g_pcMutex.lock();
    counter = g_depthCounter;
    infoCounter = g_infoCounter;
    imgCounter = g_imgCounter;
    pcCounter = g_pcCounter;
    infoMsg = g_infoMsg;
    g_mutex.unlock();
    g_infoMutex.unlock();
    g_imgMutex.unlock();
    g_pcMutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }

  g_mutex.lock();
  g_infoMutex.lock();
  g_pcMutex.lock();
  g_imgMutex.lock();
  g_depthCounter = 0;
  g_infoCounter = 0;
  g_imgCounter = 0;
  g_pcCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_EQ(counter, infoCounter);
  EXPECT_EQ(counter, imgCounter);
  EXPECT_EQ(counter, pcCounter);
  counter = 0;
  infoCounter = 0;
  imgCounter = 0;
  pcCounter = 0;

  // depth image indices
  int midWidth = static_cast<int>(rgbdSensor->ImageWidth() * 0.5);
  int midHeight = static_cast<int>(rgbdSensor->ImageHeight() * 0.5);
  int mid = midHeight * rgbdSensor->ImageWidth() + midWidth -1;
  double expectedRangeAtMidPoint = boxPosition.X() - unitBoxSize * 0.5;
  int left = midHeight * rgbdSensor->ImageWidth();
  int right = (midHeight+1) * rgbdSensor->ImageWidth() - 1;

  // xyz and rgb image indices
  int imgChannelCount = 3;
  int imgMid = midHeight * rgbdSensor->ImageWidth() * imgChannelCount
      + (midWidth-1) * imgChannelCount;
  int imgLeft = midHeight * rgbdSensor->ImageWidth() * imgChannelCount;
  int imgRight = (midHeight+1)
      * (rgbdSensor->ImageWidth() * imgChannelCount)
      - imgChannelCount;

  // check depth
  {
    // Depth sensor should see box in the middle of the image
    EXPECT_NEAR(g_depthBuffer[mid], expectedRangeAtMidPoint, DEPTH_TOL);

    // The left and right side of the depth frame should be inf
    EXPECT_DOUBLE_EQ(g_depthBuffer[left], math::INF_D);
    EXPECT_DOUBLE_EQ(g_depthBuffer[right], math::INF_D);
  }

  // check color
  {
    unsigned int mr = g_imgBuffer[imgMid];
    unsigned int mg = g_imgBuffer[imgMid + 1];
    unsigned int mb = g_imgBuffer[imgMid + 2];
    EXPECT_EQ(0u, mr);
    EXPECT_EQ(0u, mg);
#ifndef __APPLE__
    // See https://github.com/gazebosim/gz-sensors/issues/66
    EXPECT_GT(mb, 0u);
#endif

    unsigned int lr = g_imgBuffer[imgLeft];
    unsigned int lg = g_imgBuffer[imgLeft + 1];
    unsigned int lb = g_imgBuffer[imgLeft + 2];
    EXPECT_GT(lr, 0u);
    EXPECT_EQ(0u, lg);
    EXPECT_EQ(0u, lb);

    unsigned int rr = g_imgBuffer[imgRight];
    unsigned int rg = g_imgBuffer[imgRight + 1];
    unsigned int rb = g_imgBuffer[imgRight + 2];
    EXPECT_GT(rr, 0u);
    EXPECT_EQ(0u, rg);
    EXPECT_EQ(0u, rb);
  }

  // check point cloud
  {
    //  check mid point
    float mx = g_pointsXYZBuffer[imgMid];
    float my = g_pointsXYZBuffer[imgMid + 1];
    float mz = g_pointsXYZBuffer[imgMid + 2];
    EXPECT_FLOAT_EQ(g_depthBuffer[mid], mx);

    // check left and right points
    float lx = g_pointsXYZBuffer[imgLeft];
    float ly = g_pointsXYZBuffer[imgLeft + 1];
    float lz = g_pointsXYZBuffer[imgLeft + 2];
    EXPECT_FLOAT_EQ(math::INF_D, lx);
    EXPECT_FLOAT_EQ(math::INF_D, ly);
    EXPECT_FLOAT_EQ(math::INF_D, lz);

    float rx = g_pointsXYZBuffer[imgRight];
    float ry = g_pointsXYZBuffer[imgRight + 1];
    float rz = g_pointsXYZBuffer[imgRight + 2];
    EXPECT_FLOAT_EQ(math::INF_D, rx);
    EXPECT_FLOAT_EQ(math::INF_D, ry);
    EXPECT_FLOAT_EQ(math::INF_D, rz);

    // point to the left of mid point should have larger y value than mid
    // point, which in turn should have large y value than point to the
    // right of mid point
    float midLeftY = g_pointsXYZBuffer[imgMid + 1 - imgChannelCount];
    float midRightY = g_pointsXYZBuffer[imgMid + 1 + imgChannelCount];
    EXPECT_GT(midLeftY, my);
    EXPECT_GT(my, midRightY);
    EXPECT_GT(midLeftY, 0.0);
    EXPECT_LT(midRightY, 0.0);

    // all points on the box should have the same z position
    float midLeftZ = g_pointsXYZBuffer[imgMid + 2 - imgChannelCount];
    float midRightZ = g_pointsXYZBuffer[imgMid + 2 + imgChannelCount];
    EXPECT_NEAR(mz, midLeftZ, DEPTH_TOL);
    EXPECT_NEAR(mz, midRightZ, DEPTH_TOL);

    // Verify Point Cloud RGB values
    // The mid point should be blue
    unsigned int mr = g_pointsRGBBuffer[imgMid];
    unsigned int mg = g_pointsRGBBuffer[imgMid + 1];
    unsigned int mb = g_pointsRGBBuffer[imgMid + 2];
    EXPECT_EQ(0u, mr);
    EXPECT_EQ(0u, mg);
#ifndef __APPLE__
    // See https://github.com/gazebosim/gz-sensors/issues/66
    EXPECT_GT(mb, 0u);
#endif

    // Far left and right points should be red (background color)
    unsigned int lr = g_pointsRGBBuffer[imgLeft];
    unsigned int lg = g_pointsRGBBuffer[imgLeft + 1];
    unsigned int lb = g_pointsRGBBuffer[imgLeft + 2];
    EXPECT_EQ(255u, lr);
    EXPECT_EQ(0u, lg);
    EXPECT_EQ(0u, lb);

    unsigned int rr = g_pointsRGBBuffer[imgRight];
    unsigned int rg = g_pointsRGBBuffer[imgRight + 1];
    unsigned int rb = g_pointsRGBBuffer[imgRight + 2];
    EXPECT_EQ(255u, rr);
    EXPECT_EQ(0u, rg);
    EXPECT_EQ(0u, rb);
  }

  // Use PointCloudUtil to convert depth data into point clouds and compare
  // result against actual point cloud data
  {
    // init the point cloud msg to be filled
    msgs::PointCloudPacked pointsMsg;
    msgs::InitPointCloudPacked(pointsMsg, "depth2Image", false,
        {{"xyz", msgs::PointCloudPacked::Field::FLOAT32},
         {"rgb", msgs::PointCloudPacked::Field::FLOAT32}});
    pointsMsg.set_width(rgbdSensor->ImageWidth());
    pointsMsg.set_height(rgbdSensor->ImageHeight());
    pointsMsg.set_row_step(pointsMsg.point_step() * rgbdSensor->ImageWidth());

    // fill msg does the conversion from depth to points
    sensors::PointCloudUtil pointsUtil;
    pointsUtil.FillMsg(pointsMsg, 1.05, g_pointsRGBBuffer, g_depthBuffer);

    // Unpack the point cloud msg into separate rgb and xyz buffers
    unsigned char *rgbBuffer =
        new unsigned char[pointsMsg.width() * pointsMsg.height() * 3];
    float *xyzBuffer = new float[pointsMsg.width() * pointsMsg.height() * 3];
    UnpackPointCloudMsg(pointsMsg, xyzBuffer, rgbBuffer);

    for (unsigned int i = 0; i < pointsMsg.height(); ++i)
    {
      for (unsigned int j = 0; j < pointsMsg.width(); ++j)
      {
        int index = i*pointsMsg.width()*3 + j*3;

        float x = g_pointsXYZBuffer[index];
        float y = g_pointsXYZBuffer[index + 1];
        float z = g_pointsXYZBuffer[index + 2];
        float x2 = xyzBuffer[index];
        float y2 = xyzBuffer[index + 1];
        float z2 = xyzBuffer[index + 2];

        // Add special check for inf case. Conversion from depth image to
        // point cloud in the FillMsg call does not keep the sign of inf
        if (std::isinf(x))
          EXPECT_TRUE(std::isinf(x2));
        else
          EXPECT_NEAR(x2, x, DEPTH_TOL);
        if (std::isinf(y))
          EXPECT_TRUE(std::isinf(y2));
        else
          EXPECT_NEAR(y2, y, DEPTH_TOL);
        if (std::isinf(z))
          EXPECT_TRUE(std::isinf(z2));
        else
          EXPECT_NEAR(z2, z, DEPTH_TOL);
      }
    }
  }

  // Check camera info
  EXPECT_TRUE(infoMsg.has_header());
  ASSERT_EQ(1, infoMsg.header().data().size());
  EXPECT_EQ("frame_id", infoMsg.header().data(0).key());
  ASSERT_EQ(1, infoMsg.header().data(0).value().size());
  EXPECT_EQ("camera1", infoMsg.header().data(0).value(0));

  g_infoMutex.unlock();
  g_mutex.unlock();
  g_pcMutex.unlock();
  g_imgMutex.unlock();

  // Check that for a box really close it returns -inf
  math::Vector3d boxPositionNear(
      unitBoxSize * 0.5 + near_ * 0.5, 0.0, 0.0);
  box->SetLocalPosition(boxPositionNear);

  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  for (int sleep = 0; sleep < 300 &&
      (counter == 0 || infoCounter == 0 || imgCounter == 0 || pcCounter == 0);
      ++sleep)
  {
    g_mutex.lock();
    g_infoMutex.lock();
    g_imgMutex.lock();
    g_pcMutex.lock();
    counter = g_depthCounter;
    infoCounter = g_infoCounter;
    imgCounter = g_imgCounter;
    pcCounter = g_pcCounter;
    infoMsg = g_infoMsg;
    g_mutex.unlock();
    g_infoMutex.unlock();
    g_imgMutex.unlock();
    g_pcMutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }

  g_mutex.lock();
  g_infoMutex.lock();
  g_pcMutex.lock();
  g_imgMutex.lock();
  g_depthCounter = 0;
  g_infoCounter = 0;
  g_imgCounter = 0;
  g_pcCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_EQ(counter, infoCounter);
  EXPECT_EQ(counter, imgCounter);
  EXPECT_EQ(counter, pcCounter);
  counter = 0;
  infoCounter = 0;
  imgCounter = 0;
  pcCounter = 0;

  // check depth
  {
    for (unsigned int i = 0; i < rgbdSensor->ImageHeight(); ++i)
    {
      unsigned int step = i * rgbdSensor->ImageWidth();
      for (unsigned int j = 0; j < rgbdSensor->ImageWidth(); ++j)
      {
        unsigned int index = step + j;
        EXPECT_FLOAT_EQ(-math::INF_D, g_depthBuffer[index]);
      }
    }
  }

  // check color
  {
    for (unsigned int i = 0; i < rgbdSensor->ImageHeight(); ++i)
    {
      unsigned int step = i * rgbdSensor->ImageWidth() * 3;
      for (unsigned int j = 0; j < rgbdSensor->ImageWidth(); ++j)
      {
        unsigned int index = step + j*3;
        unsigned int r = g_imgBuffer[index];
        unsigned int g = g_imgBuffer[index + 1];
        unsigned int b = g_imgBuffer[index + 2];
        EXPECT_GT(r, 0u);
        EXPECT_EQ(0u, g);
        EXPECT_EQ(0u, b);
      }
    }
  }

  // check point cloud xyz and rgb values
  {
    for (unsigned int i = 0; i < rgbdSensor->ImageHeight(); ++i)
    {
      unsigned int step = i * rgbdSensor->ImageWidth() * 3;
      for (unsigned int j = 0; j < rgbdSensor->ImageWidth(); ++j)
      {
        unsigned int index = step + j*3;
        float x = g_pointsXYZBuffer[index];
        float y = g_pointsXYZBuffer[index + 1];
        float z = g_pointsXYZBuffer[index + 2];
        EXPECT_FLOAT_EQ(-math::INF_D, x);
        EXPECT_FLOAT_EQ(-math::INF_D, y);
        EXPECT_FLOAT_EQ(-math::INF_D, z);

        unsigned int r = g_pointsRGBBuffer[index];
        unsigned int g = g_pointsRGBBuffer[index + 1];
        unsigned int b = g_pointsRGBBuffer[index + 2];
        EXPECT_GT(r, 0u);
        EXPECT_EQ(0u, g);
        EXPECT_EQ(0u, b);
      }
    }
  }

  g_infoMutex.unlock();
  g_mutex.unlock();
  g_imgMutex.unlock();
  g_pcMutex.unlock();

  // Check that for a box really far it returns inf
  math::Vector3d boxPositionFar(
      unitBoxSize * 0.5 + far_ * 1.5, 0.0, 0.0);
  box->SetLocalPosition(boxPositionFar);

  mgr.RunOnce(std::chrono::steady_clock::duration::zero(), true);
  for (int sleep = 0; sleep < 300 &&
      (counter == 0 || infoCounter == 0 || imgCounter == 0 || pcCounter == 0);
      ++sleep)
  {
    g_mutex.lock();
    g_infoMutex.lock();
    g_imgMutex.lock();
    g_pcMutex.lock();
    counter = g_depthCounter;
    infoCounter = g_infoCounter;
    imgCounter = g_imgCounter;
    pcCounter = g_pcCounter;
    infoMsg = g_infoMsg;
    g_mutex.unlock();
    g_infoMutex.unlock();
    g_imgMutex.unlock();
    g_pcMutex.unlock();
    std::this_thread::sleep_for(waitTime);
  }

  g_mutex.lock();
  g_infoMutex.lock();
  g_pcMutex.lock();
  g_imgMutex.lock();
  g_depthCounter = 0;
  g_infoCounter = 0;
  g_imgCounter = 0;
  g_pcCounter = 0;
  EXPECT_GT(counter, 0);
  EXPECT_EQ(counter, infoCounter);
  EXPECT_EQ(counter, imgCounter);
  EXPECT_EQ(counter, pcCounter);
  counter = 0;
  infoCounter = 0;
  imgCounter = 0;
  pcCounter = 0;

  // check depth
  {
    for (unsigned int i = 0; i < rgbdSensor->ImageHeight(); ++i)
    {
      unsigned int step = i * rgbdSensor->ImageWidth();
      for (unsigned int j = 0; j < rgbdSensor->ImageWidth(); ++j)
      {
        unsigned int index = step + j;
        EXPECT_FLOAT_EQ(math::INF_D, g_depthBuffer[index]);
      }
    }
  }

  // check color
  {
    for (unsigned int i = 0; i < rgbdSensor->ImageHeight(); ++i)
    {
      unsigned int step = i * rgbdSensor->ImageWidth() * 3;
      for (unsigned int j = 0; j < rgbdSensor->ImageWidth(); ++j)
      {
        unsigned int index = step + j*3;
        unsigned int r = g_imgBuffer[index];
        unsigned int g = g_imgBuffer[index + 1];
        unsigned int b = g_imgBuffer[index + 2];
        EXPECT_GT(r, 0u);
        EXPECT_EQ(0u, g);
        EXPECT_EQ(0u, b);
      }
    }
  }

  // check point cloud xyz and rgb values
  {
    for (unsigned int i = 0; i < rgbdSensor->ImageHeight(); ++i)
    {
      unsigned int step = i * rgbdSensor->ImageWidth() * 3;
      for (unsigned int j = 0; j < rgbdSensor->ImageWidth(); ++j)
      {
        unsigned int index = step + j*3;
        float x = g_pointsXYZBuffer[index];
        float y = g_pointsXYZBuffer[index + 1];
        float z = g_pointsXYZBuffer[index + 2];
        EXPECT_FLOAT_EQ(math::INF_D, x);
        EXPECT_FLOAT_EQ(math::INF_D, y);
        EXPECT_FLOAT_EQ(math::INF_D, z);

        unsigned int r = g_pointsRGBBuffer[index];
        unsigned int g = g_pointsRGBBuffer[index + 1];
        unsigned int b = g_pointsRGBBuffer[index + 2];
        EXPECT_GT(r, 0u);
        EXPECT_EQ(0u, g);
        EXPECT_EQ(0u, b);
      }
    }
  }

  g_infoMutex.unlock();
  g_mutex.unlock();
  g_imgMutex.unlock();
  g_pcMutex.unlock();

  // Clean up rendering ptrs
  box.reset();
  blue.reset();

  // Clean up
  mgr.Remove(rgbdSensor->Id());
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(RgbdCameraSensorTest, ImagesWithBuiltinSDF)
{
  common::Console::SetVerbosity(4);
  ImagesWithBuiltinSDF(GetParam());
}

//////////////////////////////////////////////////
void RgbdCameraSensorTest::CustomCameraInfoTopic(
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
    if(linkPtr->Get<std::string>("name") == "rgbd_camera_link")
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
      mgr.CreateSensor<gz::sensors::RgbdCameraSensor>(sensorPtr);

  ASSERT_NE(nullptr, sensor);
  sensor->SetScene(scene);

  const std::string customInfoTopic = "/test_rgbd_camera_info";
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
TEST_P(RgbdCameraSensorTest, CustomCameraInfoTopic)
{
  gz::common::Console::SetVerbosity(4);
  CustomCameraInfoTopic(GetParam());
}

//////////////////////////////////////////////////
/// \brief The bytes of the depth image, point cloud and image are those the
/// sensor published before it wrote them in place: messages built here as
/// RgbdCameraSensor::Update built them, from the frames the depth camera
/// delivered, then serialized. Frames with and without a dense cloud (the
/// serialized is_dense follows the data), with and without depth clip.
TEST_P(RgbdCameraSensorTest, PublishedBytesMatchCopy)
{
  const std::string engineName = GetParam();
  if (engineName != "ogre2")
    GTEST_SKIP() << "ogre2 only";
  auto *engine = rendering::engine(engineName);
  if (!engine)
    GTEST_SKIP() << "Engine '" << engineName << "' is not supported";

  rendering::ScenePtr scene = engine->CreateScene("scene");
  scene->SetAmbientLight(0.3, 0.3, 0.3);
  scene->SetBackgroundColor(0.2, 0.3, 0.4);
  rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetLocalPosition(3.0, 0.4, 0.0);
  rendering::MaterialPtr red = scene->CreateMaterial();
  red->SetAmbient(1.0, 0.1, 0.0);
  red->SetDiffuse(1.0, 0.1, 0.0);
  box->SetMaterial(red);
  scene->DestroyMaterial(red);
  scene->RootVisual()->AddChild(box);

  // (one manager: the sensors stay until the end of the test)
  sensors::Manager mgr;
  for (const bool clip : {false, true})
  {
    SCOPED_TRACE(clip ? "depth clip" : "no depth clip");
    const std::string topic = std::string("/test/rgbd_bytes_") +
        (clip ? "clip" : "noclip");
    // near clip 2.6: the box's front face (2.5 m) becomes -inf in the
    // depth image and, through it, in the point cloud
    const std::string sdfString = std::string(
        "<?xml version='1.0'?><sdf version='1.6'><model name='m'>"
        "<link name='l'><sensor name='rgbd_") + (clip ? "c" : "n") +
        "' type='rgbd_camera'><update_rate>10</update_rate><topic>" + topic +
        "</topic><camera><horizontal_fov>1.05</horizontal_fov>"
        "<image><width>61</width><height>47</height></image>"
        "<clip><near>0.1</near><far>10.0</far></clip>" +
        (clip ? "<depth_camera><clip><near>2.6</near><far>9.0</far></clip>"
                "</depth_camera>" : "") +
        "</camera></sensor></link></model></sdf>";
    sdf::SDFPtr doc(new sdf::SDF());
    sdf::init(doc);
    ASSERT_TRUE(sdf::readString(sdfString, doc));
    auto sensorPtr = doc->Root()->GetElement("model")->GetElement("link")
        ->GetElement("sensor");

    auto *sensor = mgr.CreateSensor<sensors::RgbdCameraSensor>(sensorPtr);
    ASSERT_NE(nullptr, sensor);
    sensor->SetScene(scene);
    const unsigned int width = sensor->ImageWidth();
    const unsigned int height = sensor->ImageHeight();
    const unsigned int n = width * height;

    // The frames, as the camera delivers them to the sensor too
    auto camera = std::dynamic_pointer_cast<rendering::DepthCamera>(
        scene->SensorByName(sensor->Name()));
    ASSERT_NE(nullptr, camera);
    std::vector<std::vector<float>> depthFrames, cloudFrames;
    auto depthConn = camera->ConnectNewDepthFrame(
        [&](const float *_d, unsigned int, unsigned int, unsigned int,
            const std::string &) { depthFrames.emplace_back(_d, _d + n); });
    auto cloudConn = camera->ConnectNewRgbPointCloud(
        [&](const float *_c, unsigned int, unsigned int, unsigned int _ch,
            const std::string &)
        {
          ASSERT_EQ(4u, _ch);
          cloudFrames.emplace_back(_c, _c + n * 4);
        });

    // The serialized messages, as they leave the publishers
    // (shared: a callback still running after a failed assertion keeps it)
    struct Received
    {
      std::mutex mutex;
      std::map<std::string, std::vector<std::string>> raw;
    };
    auto received = std::make_shared<Received>();
    std::mutex &mutex = received->mutex;
    auto &raw = received->raw;
    transport::Node node;
    const std::vector<std::pair<std::string, std::string>> topics = {
        {topic + "/depth_image", "gz.msgs.Image"},
        {topic + "/points", "gz.msgs.PointCloudPacked"},
        {topic + "/image", "gz.msgs.Image"}};
    for (const auto &[t, type] : topics)
    {
      ASSERT_TRUE(node.SubscribeRaw(t,
          [received, t = t](const char *_data, std::size_t _size,
              const transport::MessageInfo &)
          {
            std::lock_guard<std::mutex> lock(received->mutex);
            received->raw[t].emplace_back(_data, _size);
          }, type));
    }

    // dense: a wall filling the view; not dense: a box and the sky
    const std::vector<math::Vector3d> scales = {
        {1, 1, 1}, {1, 20, 20}, {1, 20, 20}, {1, 1, 1}};
    const auto stamp0 = std::chrono::seconds(2);
    for (std::size_t k = 0; k < scales.size(); ++k)
    {
      box->SetLocalScale(scales[k]);
      EXPECT_TRUE(sensor->Update(stamp0 + k * std::chrono::milliseconds(100)));
    }
    for (int i = 0; i < 300; ++i)
    {
      {
        std::lock_guard<std::mutex> lock(mutex);
        bool all = true;
        for (const auto &tt : topics)
          all = all && raw[tt.first].size() >= scales.size();
        if (all)
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // one message per frame and topic, no more
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
      std::lock_guard<std::mutex> lock(mutex);
      for (const auto &tt : topics)
        EXPECT_EQ(scales.size(), raw[tt.first].size()) << tt.first;
    }
    ASSERT_EQ(scales.size(), depthFrames.size());
    ASSERT_EQ(scales.size(), cloudFrames.size());

    // The messages RgbdCameraSensor::Update built
    msgs::PointCloudPacked pointMsg;
    msgs::InitPointCloudPacked(pointMsg, sensor->OpticalFrameId(), false,
        {{"xyz", msgs::PointCloudPacked::Field::FLOAT32},
         {"rgb", msgs::PointCloudPacked::Field::FLOAT32}});
    pointMsg.set_width(width);
    pointMsg.set_height(height);
    pointMsg.set_row_step(pointMsg.point_step() * width);
    sensors::PointCloudUtil util;
    int denseFrames = 0;
    for (std::size_t k = 0; k < scales.size(); ++k)
    {
      SCOPED_TRACE("frame " + std::to_string(k));
      const auto stamp = msgs::Convert(
          stamp0 + k * std::chrono::milliseconds(100));
      auto header = [&](msgs::Header *_h, const std::string &_frame)
      {
        *_h->mutable_stamp() = stamp;
        auto *f = _h->add_data();
        f->set_key("frame_id");
        f->add_value(_frame);
        auto *seq = _h->add_data();
        seq->set_key("seq");
        seq->add_value(std::to_string(k));
      };
      std::vector<float> depth = depthFrames[k];
      std::vector<float> cloud = cloudFrames[k];
      if (clip)
      {
        for (float &d : depth)
        {
          if (d > 9.0)
            d = math::INF_D;
          if (d < 2.6)
            d = -math::INF_D;
        }
        for (unsigned int i = 0; i < n; ++i)
        {
          if (std::isinf(depth[i]))
            cloud[i * 4] = cloud[i * 4 + 1] = cloud[i * 4 + 2] = depth[i];
        }
      }

      msgs::Image depthMsg;
      depthMsg.set_width(width);
      depthMsg.set_height(height);
      depthMsg.set_step(width * 4);
      depthMsg.set_pixel_format_type(msgs::PixelFormatType::R_FLOAT32);
      header(depthMsg.mutable_header(), sensor->FrameId());
      depthMsg.set_data(depth.data(), n * sizeof(float));

      std::vector<unsigned char> rgb(n * 3);
      pointMsg.clear_header();
      header(pointMsg.mutable_header(), sensor->OpticalFrameId());
      util.FillMsg(pointMsg, cloud.data(), true, rgb.data());
      denseFrames += pointMsg.is_dense();

      msgs::Image imageMsg;
      imageMsg.set_width(width);
      imageMsg.set_height(height);
      imageMsg.set_step(width * 3);
      imageMsg.set_pixel_format_type(msgs::PixelFormatType::RGB_INT8);
      header(imageMsg.mutable_header(), sensor->FrameId());
      imageMsg.set_data(rgb.data(), rgb.size());

      const std::string expected[] = {depthMsg.SerializeAsString(),
          pointMsg.SerializeAsString(), imageMsg.SerializeAsString()};
      for (int t = 0; t < 3; ++t)
      {
        std::lock_guard<std::mutex> lock(mutex);
        const auto &got = raw[topics[t].first];
        ASSERT_GT(got.size(), k) << topics[t].first;
        std::size_t diff = 0;
        while (diff < got[k].size() && diff < expected[t].size() &&
            got[k][diff] == expected[t][diff])
          ++diff;
        EXPECT_TRUE(got[k] == expected[t]) << topics[t].first
            << ": " << got[k].size() << " bytes, expected "
            << expected[t].size() << ", first diff at byte " << diff;
      }
    }
    // without the clip, both kinds of frames (with it, the wall is -inf)
    EXPECT_EQ(clip ? 0 : 2, denseFrames);

    depthConn.reset();
    cloudConn.reset();
  }
  box.reset();
  red.reset();
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

INSTANTIATE_TEST_SUITE_P(RgbdCameraSensor, RgbdCameraSensorTest,
    RENDER_ENGINE_VALUES, rendering::PrintToStringParam());
