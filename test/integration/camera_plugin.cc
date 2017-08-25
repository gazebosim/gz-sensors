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
#include <ignition/msgs.hh>
#include "test_config.h"
#include "TransportTestTools.hh"


TEST(CameraPlugin, imagesWithBuiltinSDF)
{
  // get the darn test data
  std::string path = ignition::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "integration", "camera_sensor_builtin.sdf");
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

  // do the test
  ignition::sensors::Manager mgr;
  mgr.AddPluginPaths(ignition::common::joinPaths(
        PROJECT_BUILD_DIR, "src", "camera"));
  auto ids = mgr.LoadSensor(sensorPtr);
  ASSERT_EQ(1, ids.size());
  ASSERT_NE(ignition::sensors::NO_SENSOR, ids[0]);

  std::string topic = "/test/integration/CameraPlugin_imagesWithBuiltinSDF";
  WaitForMessageTestHelper<ignition::msgs::ImageStamped> helper(topic);

  // Update once to create image
  mgr.RunOnce(ignition::common::Time::Zero);

  EXPECT_TRUE(helper.WaitForMessage()) << helper;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
