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
#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <sdf/Root.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/World.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "ignition/sensors/Util.hh"

/// \brief Test Util functions
class Util_TEST : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);
  }
};

using namespace gz;
using namespace sensors;

//////////////////////////////////////////////////
TEST_F(Util_TEST, customType)
{
  auto sdfFile = gz::common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "sdf", "custom_sensors.sdf");

  sdf::Root root;
  auto errors = root.Load(sdfFile);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(1u, root.WorldCount());
  auto world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  EXPECT_EQ(1u, world->ModelCount());
  auto model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  EXPECT_EQ(1u, model->LinkCount());
  auto link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);

  EXPECT_EQ(3u, link->SensorCount());

  {
    auto sensor = link->SensorByIndex(0);
    ASSERT_NE(nullptr, sensor);
    EXPECT_EQ("complete", sensor->Name());
    EXPECT_EQ("sensor_type", customType(*sensor));
    EXPECT_EQ("sensor_type", customType(sensor->Element()));
  }

  {
    auto sensor = link->SensorByIndex(1);
    ASSERT_NE(nullptr, sensor);
    EXPECT_EQ("wrong_type", sensor->Name());
    EXPECT_TRUE(customType(*sensor).empty());
    EXPECT_TRUE(customType(sensor->Element()).empty());
  }

  {
    auto sensor = link->SensorByIndex(2);
    ASSERT_NE(nullptr, sensor);
    EXPECT_EQ("missing_ignition_type", sensor->Name());
    EXPECT_TRUE(customType(*sensor).empty());
    EXPECT_TRUE(customType(sensor->Element()).empty());
  }

  sdf::ElementPtr ptr{nullptr};
  EXPECT_TRUE(customType(ptr).empty());
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
