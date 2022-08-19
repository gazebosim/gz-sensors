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
#include <gtest/gtest.h>
#include <sdf/sdf.hh>

#include <gz/sensors/Export.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/sensors/Manager.hh>

sdf::ElementPtr cameraToBadSdf()
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='cam_name' type='not_camera'>"
    << "      <not_camera>"
    << "      </not_camera>"
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

sdf::ElementPtr CameraToSdf(const std::string &_type,
    const std::string &_name, double _updateRate,
    const std::string &_topic, bool _alwaysOn, bool _visualize)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "    <sensor name='" << _name << "' type='" << _type << "'>"
    << "      <topic>" << _topic << "</topic>"
    << "      <update_rate>"<< _updateRate <<"</update_rate>"
    << "      <always_on>"<< _alwaysOn <<"</always_on>"
    << "      <visualize>" << _visualize << "</visualize>"
    << "      <camera>"
    << "        <horizontal_fov>.75</horizontal_fov>"
    << "        <image>"
    << "          <width>640</width>"
    << "          <height>480</height>"
    << "          <format>R8G8B8</format>"
    << "        </image>"
    << "        <clip>"
    << "          <near>0.2</near>"
    << "          <far>12.3</far>"
    << "        </clip>"
    << "        <save enabled='true'>"
    << "          <path>/tmp/cam</path>"
    << "        </save>"
    << "        <noise>"
    << "          <type>gaussian</type>"
    << "          <mean>0.5</mean>"
    << "          <stddev>0.1</stddev>"
    << "        </noise>"
    << "        <distortion>"
    << "          <k1>0.1</k1>"
    << "          <k2>0.2</k2>"
    << "          <k3>0.3</k3>"
    << "          <p1>0.4</p1>"
    << "          <p2>0.5</p2>"
    << "          <center>0.2 0.4</center>"
    << "        </distortion>"
    << "        <lens>"
    << "          <type>custom</type>"
    << "          <scale_to_hfov>false</scale_to_hfov>"
    << "          <custom_function>"
    << "            <c1>1.1</c1>"
    << "            <c2>2.2</c2>"
    << "            <c3>3.3</c3>"
    << "            <f>1.2</f>"
    << "            <fun>sin</fun>"
    << "          </custom_function>"
    << "          <cutoff_angle>0.7505</cutoff_angle>"
    << "          <env_texture_size>128</env_texture_size>"
    << "          <intrinsics>"
    << "            <fx>280</fx>"
    << "            <fy>281</fy>"
    << "            <cx>162</cx>"
    << "            <cy>124</cy>"
    << "            <s>1.2</s>"
    << "          </intrinsics>"
    << "        </lens>"
    << "      </camera>"
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

/// \brief Test camera sensor
class Camera_TEST : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);
  }
};

//////////////////////////////////////////////////
TEST(Camera_TEST, CreateCamera)
{
  gz::sensors::Manager mgr;

  sdf::ElementPtr camSdf = CameraToSdf("camera", "my_camera", 60.0, "/cam",
      true, true);

  // Create a CameraSensor
  gz::sensors::CameraSensor *cam =
    mgr.CreateSensor<gz::sensors::CameraSensor>(camSdf);

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, cam);

  // Check topics
  EXPECT_EQ("/cam", cam->Topic());
  EXPECT_EQ("/camera_info", cam->InfoTopic());

  // however camera is not loaded because a rendering scene is missing so
  // updates will not be successful and image size will be 0
  EXPECT_FALSE(cam->Update(gz::common::Time()));
  EXPECT_EQ(0u, cam->ImageWidth());
  EXPECT_EQ(0u, cam->ImageHeight());

  // Bad camera
  sdf::ElementPtr camBadSdf = cameraToBadSdf();

  // Create a CameraSensor
  gz::sensors::CameraSensor *badCam =
    mgr.CreateSensor<gz::sensors::CameraSensor>(camBadSdf);
  EXPECT_TRUE(badCam == nullptr);
}

/////////////////////////////////////////////////
TEST(Camera_TEST, Topic)
{
  const std::string type = "camera";
  const std::string name = "TestCamera";
  const double updateRate = 30;
  const bool alwaysOn = 1;
  const bool visualize = 1;

  // Factory
  gz::sensors::Manager mgr;

  // Default topic
  {
    const std::string topic;
    auto cameraSdf = CameraToSdf(type, name, updateRate, topic, alwaysOn,
        visualize);

    auto sensorId = mgr.CreateSensor(cameraSdf);
    EXPECT_NE(gz::sensors::NO_SENSOR, sensorId);

    auto sensor = mgr.Sensor(sensorId);
    EXPECT_NE(nullptr, sensor);

    auto camera = dynamic_cast<gz::sensors::CameraSensor *>(sensor);
    ASSERT_NE(nullptr, camera);

    EXPECT_EQ("/camera", camera->Topic());
  }

  // Convert to valid topic
  {
    const std::string topic = "/topic with spaces/@~characters//";
    auto cameraSdf = CameraToSdf(type, name, updateRate, topic, alwaysOn,
        visualize);

    auto sensorId = mgr.CreateSensor(cameraSdf);
    EXPECT_NE(gz::sensors::NO_SENSOR, sensorId);

    auto sensor = mgr.Sensor(sensorId);
    EXPECT_NE(nullptr, sensor);

    auto camera = dynamic_cast<gz::sensors::CameraSensor *>(sensor);
    ASSERT_NE(nullptr, camera);

    EXPECT_EQ("/topic_with_spaces/characters", camera->Topic());
  }

  // Invalid topic
  {
    const std::string topic = "@@@";
    auto cameraSdf = CameraToSdf(type, name, updateRate, topic, alwaysOn,
        visualize);

    auto sensorId = mgr.CreateSensor(cameraSdf);
    EXPECT_EQ(gz::sensors::NO_SENSOR, sensorId);
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
