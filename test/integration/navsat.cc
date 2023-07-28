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

#include <sdf/sdf.hh>

#include <gz/math/Helpers.hh>

#include <gz/msgs/navsat.pb.h>

#include <gz/sensors/NavSatSensor.hh>
#include <gz/sensors/SensorFactory.hh>

#include "test_config.hh"  // NOLINT(build/include)
#include "TransportTestTools.hh"

using namespace gz;
using namespace sensors;
using namespace std::chrono_literals;

/// \brief Helper function to create a navsat sdf element
sdf::ElementPtr NavSatToSdf(const std::string &_name,
    const math::Pose3d &_pose, const double _updateRate,
    const std::string &_topic, const bool _alwaysOn,
    const bool _visualize)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='" << _name << "' type='navsat'>"
    << "      <pose>" << _pose << "</pose>"
    << "      <topic>" << _topic << "</topic>"
    << "      <update_rate>"<< _updateRate <<"</update_rate>"
    << "      <alwaysOn>" << _alwaysOn <<"</alwaysOn>"
    << "      <visualize>" << _visualize << "</visualize>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  if (!sdf::readString(stream.str(), sdfParsed))
    return sdf::ElementPtr();

  return sdfParsed->Root()->GetElement("model")->GetElement("link")
    ->GetElement("sensor");
}

/// \brief Helper function to create a navsat sdf element with noise
sdf::ElementPtr NavSatToSdfWithNoise(const std::string &_name,
    const math::Pose3d &_pose, const double _updateRate,
    const std::string &_topic, const bool _alwaysOn,
    const bool _visualize, double _mean, double _stddev, double _bias)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='" << _name << "' type='navsat'>"
    << "      <pose>" << _pose << "</pose>"
    << "      <topic>" << _topic << "</topic>"
    << "      <update_rate>"<< _updateRate <<"</update_rate>"
    << "      <alwaysOn>" << _alwaysOn <<"</alwaysOn>"
    << "      <visualize>" << _visualize << "</visualize>"
    << "      <navsat>"
    << "        <position_sensing>"
    << "          <vertical>"
    << "            <noise type='gaussian'>"
    << "              <mean>" << _mean << "</mean>"
    << "              <stddev>" << _stddev << "</stddev>"
    << "              <bias_mean>" << _bias << "</bias_mean>"
    << "            </noise>"
    << "          </vertical>"
    << "          <horizontal>"
    << "            <noise type='gaussian'>"
    << "              <mean>" << _mean << "</mean>"
    << "              <stddev>" << _stddev << "</stddev>"
    << "              <bias_mean>" << _bias << "</bias_mean>"
    << "            </noise>"
    << "          </horizontal>"
    << "        </position_sensing>"
    << "        <velocity_sensing>"
    << "          <vertical>"
    << "            <noise type='gaussian'>"
    << "              <mean>" << _mean << "</mean>"
    << "              <stddev>" << _stddev << "</stddev>"
    << "              <bias_mean>" << _bias << "</bias_mean>"
    << "            </noise>"
    << "          </vertical>"
    << "          <horizontal>"
    << "            <noise type='gaussian'>"
    << "              <mean>" << _mean << "</mean>"
    << "              <stddev>" << _stddev << "</stddev>"
    << "              <bias_mean>" << _bias << "</bias_mean>"
    << "            </noise>"
    << "          </horizontal>"
    << "        </velocity_sensing>"
    << "      </navsat>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  if (!sdf::readString(stream.str(), sdfParsed))
    return sdf::ElementPtr();

  return sdfParsed->Root()->GetElement("model")->GetElement("link")
    ->GetElement("sensor");
}

/// \brief Test navsat sensor
class NavSatSensorTest: public testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
  }
};

/////////////////////////////////////////////////
TEST_F(NavSatSensorTest, CreateNavSat)
{
  // Create SDF describing a navsat sensor
  const std::string name = "TestNavSat";
  const std::string topic = "/gz/sensors/test/navsat";
  const std::string topicNoise = "/gz/sensors/test/navsat_noise";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Create sensor SDF
  math::Pose3d sensorPose(math::Vector3d(0.25, 0.0, 0.5),
      math::Quaterniond::Identity);
  auto navsatSdf = NavSatToSdf(name, sensorPose, updateRate, topic, alwaysOn,
      visualize);

  auto navsatSdfNoise = NavSatToSdfWithNoise(name, sensorPose, updateRate,
      topicNoise, alwaysOn, visualize, 1.0, 0.2, 10.0);

  // create the sensor using sensor factory
  SensorFactory sf;
  auto sensor = sf.CreateSensor<NavSatSensor>(navsatSdf);
  ASSERT_NE(nullptr, sensor);

  EXPECT_EQ(name, sensor->Name());
  EXPECT_EQ(topic, sensor->Topic());
  EXPECT_DOUBLE_EQ(updateRate, sensor->UpdateRate());

  auto sensorNoise = sf.CreateSensor<NavSatSensor>(navsatSdfNoise);
  ASSERT_NE(nullptr, sensorNoise);

  EXPECT_EQ(name, sensorNoise->Name());
  EXPECT_EQ(topicNoise, sensorNoise->Topic());
  EXPECT_DOUBLE_EQ(updateRate, sensorNoise->UpdateRate());
}

/////////////////////////////////////////////////
TEST_F(NavSatSensorTest, SensorReadings)
{
  // Create SDF describing a navsat sensor
  const std::string name{"TestNavSat"};
  const std::string topic{"/gz/sensors/test/navsat"};
  const std::string topicNoise{"/gz/sensors/test/navsat_noise"};
  const double updateRate{30.0};
  const bool alwaysOn{true};
  const bool visualize{true};

  // Create sensor SDF
  math::Pose3d sensorPose(math::Vector3d(0.25, 0.0, 0.5),
      math::Quaterniond::Identity);
  auto navsatSdf = NavSatToSdf(name, sensorPose, updateRate, topic,
      alwaysOn, visualize);

  auto navsatSdfNoise = NavSatToSdfWithNoise(name, sensorPose, updateRate,
      topicNoise, alwaysOn, visualize, 1.0, 0.2, 10.0);

  // create the sensor using sensor factory
  SensorFactory sf;
  auto sensor = sf.CreateSensor<NavSatSensor>(navsatSdf);
  ASSERT_NE(nullptr, sensor);
  EXPECT_FALSE(sensor->HasConnections());

  auto sensorNoise = sf.CreateSensor<NavSatSensor>(navsatSdfNoise);
  ASSERT_NE(nullptr, sensorNoise);

  // verify initial readings
  EXPECT_DOUBLE_EQ(0.0, sensor->Latitude().Degree());
  EXPECT_DOUBLE_EQ(0.0, sensor->Longitude().Degree());
  EXPECT_DOUBLE_EQ(0.0, sensor->Altitude());
  EXPECT_DOUBLE_EQ(0.0, sensor->Velocity().X());
  EXPECT_DOUBLE_EQ(0.0, sensor->Velocity().Y());
  EXPECT_DOUBLE_EQ(0.0, sensor->Velocity().Z());

  EXPECT_DOUBLE_EQ(0.0, sensorNoise->Latitude().Degree());
  EXPECT_DOUBLE_EQ(0.0, sensorNoise->Longitude().Degree());
  EXPECT_DOUBLE_EQ(0.0, sensorNoise->Altitude());
  EXPECT_DOUBLE_EQ(0.0, sensorNoise->Velocity().X());
  EXPECT_DOUBLE_EQ(0.0, sensorNoise->Velocity().Y());
  EXPECT_DOUBLE_EQ(0.0, sensorNoise->Velocity().Z());

  // set state and verify readings
  math::Angle lat{GZ_DTOR(20)};
  sensor->SetLatitude(lat);
  sensorNoise->SetLatitude(lat);
  EXPECT_EQ(lat, sensor->Latitude());
  EXPECT_EQ(lat, sensorNoise->Latitude());

  math::Angle lon{GZ_DTOR(-20)};
  sensor->SetLongitude(lon);
  sensorNoise->SetLongitude(lon);
  EXPECT_EQ(lon, sensor->Longitude());
  EXPECT_EQ(lon, sensorNoise->Longitude());

  double altitude{20.0};
  sensor->SetAltitude(altitude);
  sensorNoise->SetAltitude(altitude);
  EXPECT_DOUBLE_EQ(altitude, sensor->Altitude());
  EXPECT_DOUBLE_EQ(altitude, sensorNoise->Altitude());

  lat += GZ_DTOR(20);
  lon += GZ_DTOR(20);
  altitude += 100;
  sensor->SetPosition(lat, lon, altitude);
  sensorNoise->SetPosition(lat, lon, altitude);
  EXPECT_EQ(lat, sensor->Latitude());
  EXPECT_EQ(lat, sensorNoise->Latitude());
  EXPECT_EQ(lon, sensor->Longitude());
  EXPECT_EQ(lon, sensorNoise->Longitude());
  EXPECT_DOUBLE_EQ(altitude, sensor->Altitude());
  EXPECT_DOUBLE_EQ(altitude, sensorNoise->Altitude());

  math::Vector3d velocity{1.0, 2.0, 3.0};
  sensor->SetVelocity(velocity);
  sensorNoise->SetVelocity(velocity);
  EXPECT_EQ(velocity, sensor->Velocity());
  EXPECT_EQ(velocity, sensorNoise->Velocity());

  // verify msg received on the topic
  WaitForMessageTestHelper<msgs::NavSat> msgHelper(topic);
  EXPECT_TRUE(sensor->HasConnections());
  sensor->Update(std::chrono::steady_clock::duration(1s));
  EXPECT_TRUE(msgHelper.WaitForMessage()) << msgHelper;
  auto msg = msgHelper.Message();
  EXPECT_EQ(1, msg.header().stamp().sec());
  EXPECT_EQ(0, msg.header().stamp().nsec());
  EXPECT_DOUBLE_EQ(lat.Degree(), msg.latitude_deg());
  EXPECT_DOUBLE_EQ(lon.Degree(), msg.longitude_deg());
  EXPECT_DOUBLE_EQ(altitude, msg.altitude());
  EXPECT_DOUBLE_EQ(velocity.X(), msg.velocity_east());
  EXPECT_DOUBLE_EQ(velocity.Y(), msg.velocity_north());
  EXPECT_DOUBLE_EQ(velocity.Z(), msg.velocity_up());

  // verify msg with noise received on the topic
  WaitForMessageTestHelper<msgs::NavSat> msgHelperNoise(topicNoise);
  sensorNoise->Update(std::chrono::steady_clock::duration(1s));
  EXPECT_TRUE(msgHelperNoise.WaitForMessage()) << msgHelperNoise;
  auto msgNoise = msgHelperNoise.Message();
  EXPECT_EQ(1, msgNoise.header().stamp().sec());
  EXPECT_EQ(0, msgNoise.header().stamp().nsec());
  EXPECT_FALSE(math::equal(lat.Degree(), msgNoise.latitude_deg()));
  EXPECT_FALSE(math::equal(lon.Degree(), msgNoise.longitude_deg()));
  EXPECT_FALSE(math::equal(altitude, msgNoise.altitude()));
  EXPECT_FALSE(math::equal(velocity.X(), msgNoise.velocity_east()));
  EXPECT_FALSE(math::equal(velocity.Y(), msgNoise.velocity_north()));
  EXPECT_FALSE(math::equal(velocity.Z(), msgNoise.velocity_up()));
}

/////////////////////////////////////////////////
TEST_F(NavSatSensorTest, Topic)
{
  const std::string name{"TestNavSat"};
  const double updateRate{30.0};
  const bool alwaysOn{true};
  const bool visualize{true};
  auto sensorPose = math::Pose3d();

  // Factory
  SensorFactory factory;

  // Default topic
  {
    const std::string topic;
    auto navsatSdf = NavSatToSdf(name, sensorPose, updateRate, topic,
        alwaysOn, visualize);

    auto navsat = factory.CreateSensor<NavSatSensor>(navsatSdf);
    ASSERT_NE(nullptr, navsat);

    EXPECT_EQ("/navsat", navsat->Topic());
  }

  // Convert to valid topic
  {
    const std::string topic = "/topic with spaces/@~characters//";
    auto navsatSdf = NavSatToSdf(name, sensorPose, updateRate, topic,
        alwaysOn, visualize);

    auto navsat = factory.CreateSensor<NavSatSensor>(navsatSdf);
    ASSERT_NE(nullptr, navsat);

    EXPECT_EQ("/topic_with_spaces/characters", navsat->Topic());
  }

  // Invalid topic
  {
    const std::string topic = "@@@";
    auto navsatSdf = NavSatToSdf(name, sensorPose, updateRate, topic,
        alwaysOn, visualize);

    auto sensor = factory.CreateSensor<
        NavSatSensor>(navsatSdf);
    ASSERT_EQ(nullptr, sensor);
  }
}
