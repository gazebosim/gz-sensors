/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <sstream>
#include <string>

#include <sdf/sdf.hh>

#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>

#include <gz/sensors/MarineRadarSensor.hh>
#include <gz/sensors/SensorFactory.hh>

/// \brief Build a <sensor> element for a custom sensor.
/// \param[in] _type Value of the gz:type attribute
/// \param[in] _custom Contents of the <gz:marine_radar> element, or an empty
/// string to leave the element out altogether
sdf::ElementPtr RadarToSdf(const std::string &_name,
    const std::string &_topic, const std::string &_type,
    const std::string &_custom)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.9'>"
    << " <model name='usv'>"
    << "  <link name='radar_link'>"
    << "    <sensor name='" << _name << "' type='custom' gz:type='"
    << _type << "'>"
    << "      <topic>" << _topic << "</topic>"
    << "      <update_rate>60</update_rate>"
    << "      <always_on>1</always_on>";
  if (!_custom.empty())
    stream << "      <gz:marine_radar>" << _custom << "</gz:marine_radar>";
  stream
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

/////////////////////////////////////////////////
TEST(MarineRadarSensor, CreateMarineRadar)
{
  auto sdfSensor = RadarToSdf("radar", "/test_radar", "marine_radar",
      "<range_min>15</range_min>"
      "<range_max>500</range_max>"
      "<range_resolution>2.5</range_resolution>"
      "<horizontal_fov_deg>360</horizontal_fov_deg>"
      "<samples>2048</samples>"
      "<rotation_rate_rpm>30</rotation_rate_rpm>"
      "<image_size>256</image_size>"
      "<range_noise><noise type='gaussian'><stddev>0.75</stddev></noise>"
      "</range_noise>");
  ASSERT_NE(nullptr, sdfSensor);

  gz::sensors::SensorFactory factory;
  auto sensor =
      factory.CreateSensor<gz::sensors::MarineRadarSensor>(sdfSensor);
  ASSERT_NE(nullptr, sensor);

  EXPECT_EQ("radar", sensor->Name());
  EXPECT_EQ("/test_radar", sensor->Topic());
  EXPECT_EQ("/test_radar/image", sensor->ImageTopic());
  EXPECT_EQ("/test_radar/targets", sensor->TargetsTopic());
  EXPECT_EQ("/test_radar/returns", sensor->ReturnsTopic());

  EXPECT_DOUBLE_EQ(15.0, sensor->RangeMin());
  EXPECT_DOUBLE_EQ(500.0, sensor->RangeMax());
  EXPECT_DOUBLE_EQ(2.5, sensor->RangeResolution());
  EXPECT_DOUBLE_EQ(30.0, sensor->RotationRateRpm());
  EXPECT_NEAR(2 * GZ_PI, sensor->HorizontalFOV(), 1e-9);
  EXPECT_EQ(2048u, sensor->SpokesPerScan());
  EXPECT_NEAR(2 * GZ_PI / 2048.0, sensor->AngularResolution(), 1e-12);
  EXPECT_EQ(194u, sensor->RangeBins());
  EXPECT_EQ(256u, sensor->ImageSize());
  EXPECT_EQ(0u, sensor->ScanCount());
  EXPECT_EQ(0u, sensor->TargetCount());
}

/////////////////////////////////////////////////
TEST(MarineRadarSensor, DefaultConfiguration)
{
  // No <gz:marine_radar> element at all must give a fully defaulted radar.
  auto sdfSensor = RadarToSdf("radar", "", "marine_radar", "");
  ASSERT_NE(nullptr, sdfSensor);

  gz::sensors::SensorFactory factory;
  auto sensor =
      factory.CreateSensor<gz::sensors::MarineRadarSensor>(sdfSensor);
  ASSERT_NE(nullptr, sensor);

  EXPECT_EQ("/marine_radar", sensor->Topic());
  EXPECT_DOUBLE_EQ(10.0, sensor->RangeMin());
  EXPECT_DOUBLE_EQ(1000.0, sensor->RangeMax());
  EXPECT_DOUBLE_EQ(24.0, sensor->RotationRateRpm());
  EXPECT_EQ(4096u, sensor->SpokesPerScan());
  EXPECT_EQ(512u, sensor->ImageSize());
}

/////////////////////////////////////////////////
TEST(MarineRadarSensor, WrongCustomType)
{
  // A custom sensor of a different type must be refused.
  auto sdfSensor = RadarToSdf("radar", "/test_radar", "odometer", "");
  ASSERT_NE(nullptr, sdfSensor);

  gz::sensors::SensorFactory factory;
  auto sensor =
      factory.CreateSensor<gz::sensors::MarineRadarSensor>(sdfSensor);
  EXPECT_EQ(nullptr, sensor);
}

/////////////////////////////////////////////////
TEST(MarineRadarSensor, WrongSensorType)
{
  // A built-in sensor type must be refused: the marine radar is only
  // loadable as <sensor type="custom" gz:type="marine_radar">.
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.9'>"
    << " <model name='usv'>"
    << "  <link name='radar_link'>"
    << "    <sensor name='radar' type='air_pressure'>"
    << "      <topic>/test_radar</topic>"
    << "    </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";
  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));
  auto sdfSensor = sdfParsed->Root()->GetElement("model")
      ->GetElement("link")->GetElement("sensor");

  gz::sensors::SensorFactory factory;
  auto sensor =
      factory.CreateSensor<gz::sensors::MarineRadarSensor>(sdfSensor);
  EXPECT_EQ(nullptr, sensor);
}

/////////////////////////////////////////////////
TEST(MarineRadarSensor, InvalidConfigurationIsRefused)
{
  // range_max below range_min is a fatal configuration error, so the sensor
  // must not load rather than run with a wrong configuration.
  auto sdfSensor = RadarToSdf("radar", "/test_radar", "marine_radar",
      "<range_min>100</range_min><range_max>50</range_max>");
  ASSERT_NE(nullptr, sdfSensor);

  gz::sensors::SensorFactory factory;
  auto sensor =
      factory.CreateSensor<gz::sensors::MarineRadarSensor>(sdfSensor);
  EXPECT_EQ(nullptr, sensor);
}

/////////////////////////////////////////////////
TEST(MarineRadarSensor, UnsupportedNoiseModelIsRefused)
{
  auto sdfSensor = RadarToSdf("radar", "/test_radar", "marine_radar",
      "<range_noise><noise type='gaussian_quantized'><stddev>1</stddev>"
      "</noise></range_noise>");
  ASSERT_NE(nullptr, sdfSensor);

  gz::sensors::SensorFactory factory;
  auto sensor =
      factory.CreateSensor<gz::sensors::MarineRadarSensor>(sdfSensor);
  EXPECT_EQ(nullptr, sensor);
}

/////////////////////////////////////////////////
TEST(MarineRadarSensor, SetRangeMax)
{
  auto sdfSensor = RadarToSdf("radar", "/test_radar", "marine_radar",
      "<range_max>500</range_max>");
  ASSERT_NE(nullptr, sdfSensor);

  gz::sensors::SensorFactory factory;
  auto sensor =
      factory.CreateSensor<gz::sensors::MarineRadarSensor>(sdfSensor);
  ASSERT_NE(nullptr, sensor);

  EXPECT_TRUE(sensor->SetRangeMax(1500.0));
  EXPECT_DOUBLE_EQ(1500.0, sensor->RangeMax());

  // Below range_min, and therefore invalid: the old range is kept.
  EXPECT_FALSE(sensor->SetRangeMax(1.0));
  EXPECT_DOUBLE_EQ(1500.0, sensor->RangeMax());
}

/////////////////////////////////////////////////
TEST(MarineRadarSensor, UpdateWithoutSceneFails)
{
  auto sdfSensor = RadarToSdf("radar", "/test_radar", "marine_radar", "");
  ASSERT_NE(nullptr, sdfSensor);

  gz::sensors::SensorFactory factory;
  auto sensor =
      factory.CreateSensor<gz::sensors::MarineRadarSensor>(sdfSensor);
  ASSERT_NE(nullptr, sensor);

  // No rendering scene has been set, so there is nothing to cast rays at.
  EXPECT_EQ(nullptr, sensor->Scene());
  EXPECT_FALSE(sensor->Update(std::chrono::steady_clock::duration::zero()));
  EXPECT_EQ(0u, sensor->RayCount());
}
