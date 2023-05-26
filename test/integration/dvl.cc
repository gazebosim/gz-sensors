/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <array>
#include <string>

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

#include <gz/sensors/DopplerVelocityLog.hh>
#include <gz/sensors/Manager.hh>

#include <sdf/sdf.hh>

#include "test_config.hh"  // NOLINT(build/include)
#include "TransportTestTools.hh"

using namespace gz;
using DopplerVelocityLog = sensors::DopplerVelocityLog;

struct DVLConfig
{
  std::string name = "dvl";
  std::string topic = "/gz/sensors/test/dvl";
  double updateRate = 30;  // Hz

  double apertureAngle = 2.;  // degrees
  double tiltAngle = 30.;  // degrees
  std::array<double, 4> rotationAngles{45, 135, -135, -45};  // degrees

  std::string bottomTrackingMode = "never";

  std::string waterMassTrackingMode = "never";
  double waterMassNearBoundary = 20.;
  double waterMassFarBoundary = 60.;

  double trackingNoise = 0.001;

  std::string waterVelocityVariable = "underwater_current_velocity";

  bool alwaysOn = true;
};

/// \brief Helper function to create an imu sdf element
sdf::ElementPtr MakeDVLSdf(const DVLConfig &_config)
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='model'>"
    << "  <link name='link'>"
    << "   <sensor name='" << _config.name << "' type='custom' gz:type='dvl'>"
    << "    <always_on>" << _config.alwaysOn << "</always_on>"
    << "    <update_rate>" << _config.updateRate << "</update_rate>"
    << "    <topic>" << _config.topic << "</topic>"
    << "    <visualize>1</visualize>"
    << "    <gz:dvl>"
    << "     <type>phased_array</type>"
    << "     <arrangement degrees='true'>"
    << "      <beam id='1'>"
    << "       <aperture>" << _config.apertureAngle << "</aperture>"
    << "       <rotation>" << _config.rotationAngles[0] << "</rotation>"
    << "       <tilt>" << _config.tiltAngle << "</tilt>"
    << "      </beam>"
    << "      <beam>"
    << "       <aperture>" << _config.apertureAngle << "</aperture>"
    << "       <rotation>" << _config.rotationAngles[1] << "</rotation>"
    << "       <tilt>" << _config.tiltAngle << "</tilt>"
    << "      </beam>"
    << "      <beam>"
    << "       <aperture>" << _config.apertureAngle << "</aperture>"
    << "       <rotation>" << _config.rotationAngles[2] << "</rotation>"
    << "       <tilt>" << _config.tiltAngle << "</tilt>"
    << "      </beam>"
    << "      <beam>"
    << "       <aperture>" << _config.apertureAngle << "</aperture>"
    << "       <rotation>" << _config.rotationAngles[3] << "</rotation>"
    << "       <tilt>" << _config.tiltAngle << "</tilt>"
    << "      </beam>"
    << "     </arrangement>"
    << "     <tracking>"
    << "      <bottom_mode>"
    << "       <when>" << _config.bottomTrackingMode << "</when>"
    << "       <noise type='gaussian'>"
    << "        <stddev>" << _config.trackingNoise << "</stddev>"
    << "       </noise>"
    << "       <visualize>1</visualize>"
    << "      </bottom_mode>"
    << "      <water_mass_mode>"
    << "       <when>" << _config.waterMassTrackingMode << "</when>"
    << "       <noise type='gaussian'>"
    << "        <stddev>" << _config.trackingNoise << "</stddev>"
    << "       </noise>"
    << "       <water_velocity>"
    << "        <x>" << _config.waterVelocityVariable << "_x</x>"
    << "        <y>" << _config.waterVelocityVariable << "_y</y>"
    << "       </water_velocity>"
    << "       <boundaries>"
    << "        <near>" << _config.waterMassNearBoundary << "</near>"
    << "        <far>" << _config.waterMassFarBoundary << "</far>"
    << "       </boundaries>"
    << "       <bins>10</bins>"
    << "       <visualize>1</visualize>"
    << "      </water_mass_mode>"
    << "     </tracking>"
    << "     <resolution>0.01</resolution>"
    << "     <maximum_range>100.</maximum_range>"
    << "     <minimum_range>0.1</minimum_range>"
    << "    </gz:dvl>"
    << "   </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";
  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  if (!sdf::readString(stream.str(), sdfParsed))
    return sdf::ElementPtr();
  return sdfParsed->Root()->GetElement("model")
    ->GetElement("link")->GetElement("sensor");
}

/// \brief Test DVL sensor
class DopplerVelocityLogTest : public testing::Test,
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

    common::Console::SetVerbosity(4);
    this->engine = rendering::engine(GetParam());
    if (!this->engine)
    {
      GTEST_SKIP() << "Engine '" << GetParam()
                   << "' is not supported";
    }
    this->scene = this->engine->CreateScene("scene");
    this->scene->SetAmbientLight(1.0, 1.0, 1.0);
    rendering::VisualPtr root = this->scene->RootVisual();

    rendering::MaterialPtr sand = this->scene->CreateMaterial();
    sand->SetAmbient(0.5, 0.5, 0.5);
    sand->SetDiffuse(0.5, 0.5, 0.5);

    rendering::VisualPtr seabed = this->scene->CreateVisual();
    seabed->AddGeometry(this->scene->CreatePlane());
    const math::Pose3d seabedPose(
        math::Vector3d(0.0, 0.0, -seabedDepth),
        math::Quaterniond::Identity);
    seabed->SetLocalPose(seabedPose);
    seabed->SetLocalScale(math::Vector3d(1e3, 1e3, 0.0));
    seabed->SetMaterial(sand);
    root->AddChild(seabed);

    constexpr uint64_t seabedEntity = 100u;
    seabed->SetUserData("gazebo-entity", seabedEntity);
    sensors::EntityKinematicState & seabedState =
        this->worldState.kinematics[seabedEntity];
    seabedState.pose = seabedPose;

    sensors::EnvironmentalData::FrameT dataframe;
    math::InMemoryTimeVaryingVolumetricGridFactory<double, double> xGridFactory;  // NOLINT
    xGridFactory.AddPoint(0., math::Vector3d(1e3, 1e3, 0.), 1.);
    xGridFactory.AddPoint(0., math::Vector3d(-1e3, 1e3, 0.), 1.);
    xGridFactory.AddPoint(0., math::Vector3d(-1e3, -1e3, 0.), 1.);
    xGridFactory.AddPoint(0., math::Vector3d(1e3, -1e3, 0.), 1.);
    xGridFactory.AddPoint(0., math::Vector3d(1e3, 1e3, -100.), 1.);
    xGridFactory.AddPoint(0., math::Vector3d(-1e3, 1e3, -100.), 1.);
    xGridFactory.AddPoint(0., math::Vector3d(-1e3, -1e3, -100.), 1.);
    xGridFactory.AddPoint(0., math::Vector3d(1e3, -1e3, -100.), 1.);
    xGridFactory.AddPoint(1e5, math::Vector3d(1e3, 1e3, 0.), 1.);
    xGridFactory.AddPoint(1e5, math::Vector3d(-1e3, 1e3, 0.), 1.);
    xGridFactory.AddPoint(1e5, math::Vector3d(-1e3, -1e3, 0.), 1.);
    xGridFactory.AddPoint(1e5, math::Vector3d(1e3, -1e3, 0.), 1.);
    xGridFactory.AddPoint(1e5, math::Vector3d(1e3, 1e3, -100.), 1.);
    xGridFactory.AddPoint(1e5, math::Vector3d(-1e3, 1e3, -100.), 1.);
    xGridFactory.AddPoint(1e5, math::Vector3d(-1e3, -1e3, -100.), 1.);
    xGridFactory.AddPoint(1e5, math::Vector3d(1e3, -1e3, -100.), 1.);
    dataframe["underwater_current_velocity_x"] = xGridFactory.Build();
    math::InMemoryTimeVaryingVolumetricGridFactory<double, double>  yGridFactory;  // NOLINT
    yGridFactory.AddPoint(0., math::Vector3d(1e3, 1e3, 0.), 0.5);
    yGridFactory.AddPoint(0., math::Vector3d(-1e3, 1e3, 0.), 0.5);
    yGridFactory.AddPoint(0., math::Vector3d(-1e3, -1e3, 0.), 0.5);
    yGridFactory.AddPoint(0., math::Vector3d(1e3, -1e3, 0.), 0.5);
    yGridFactory.AddPoint(0., math::Vector3d(1e3, 1e3, -100.), 0.5);
    yGridFactory.AddPoint(0., math::Vector3d(-1e3, 1e3, -100.), 0.5);
    yGridFactory.AddPoint(0., math::Vector3d(-1e3, -1e3, -100.), 0.5);
    yGridFactory.AddPoint(0., math::Vector3d(1e3, -1e3, -100.), 0.5);
    yGridFactory.AddPoint(1e5, math::Vector3d(1e3, 1e3, 0.), 0.5);
    yGridFactory.AddPoint(1e5, math::Vector3d(-1e3, 1e3, 0.), 0.5);
    yGridFactory.AddPoint(1e5, math::Vector3d(-1e3, -1e3, 0.), 0.5);
    yGridFactory.AddPoint(1e5, math::Vector3d(1e3, -1e3, 0.), 0.5);
    yGridFactory.AddPoint(1e5, math::Vector3d(1e3, 1e3, -100.), 0.5);
    yGridFactory.AddPoint(1e5, math::Vector3d(-1e3, 1e3, -100.), 0.5);
    yGridFactory.AddPoint(1e5, math::Vector3d(-1e3, -1e3, -100.), 0.5);
    yGridFactory.AddPoint(1e5, math::Vector3d(1e3, -1e3, -100.), 0.5);
    dataframe["underwater_current_velocity_y"] = yGridFactory.Build();

    constexpr bool useTime = true;
    this->environment = sensors::EnvironmentalData::MakeShared(
      std::move(dataframe), math::SphericalCoordinates::GLOBAL,
      sensors::EnvironmentalData::ReferenceUnits::RADIANS, useTime);
  }
  // Documentation inherited
  protected: void TearDown() override
  {
    engine->DestroyScene(scene);
    rendering::unloadEngine(engine->Name());
  }

  rendering::RenderEngine *engine{nullptr};
  rendering::ScenePtr scene;
  sensors::Manager manager;

  static constexpr double seabedDepth = 80.;
  sensors::WorldState worldState;
  std::shared_ptr<sensors::EnvironmentalData> environment;
};

/////////////////////////////////////////////////
TEST_P(DopplerVelocityLogTest, CreateSensor)
{
  const DVLConfig config{};
  auto *sensor = this->manager.
      CreateSensor<DopplerVelocityLog>(MakeDVLSdf(config));

  EXPECT_EQ(config.name, sensor->Name());
  EXPECT_EQ(config.topic, sensor->Topic());
  EXPECT_DOUBLE_EQ(config.updateRate, sensor->UpdateRate());
}

/////////////////////////////////////////////////
TEST_P(DopplerVelocityLogTest, BottomTrackingWhileStatic)
{
  // Add DVL sensor
  DVLConfig config;
  config.bottomTrackingMode = "always";
  auto *sensor = this->manager.
      CreateSensor<DopplerVelocityLog>(MakeDVLSdf(config));

  constexpr uint64_t deviceEntity = 200u;
  sensor->SetEntity(deviceEntity);
  sensor->SetScene(this->scene);
  sensor->SetManualSceneUpdate(true);

  rendering::VisualPtr root = this->scene->RootVisual();
  rendering::VisualPtr device = this->scene->CreateVisual();
  const math::Pose3d devicePose(
      math::Vector3d::Zero,
      math::Quaterniond::Identity);
  device->SetLocalPose(devicePose);
  device->SetUserData("gazebo-entity", deviceEntity);
  for (auto renderingSensor : sensor->RenderingSensors())
  {
      device->AddChild(renderingSensor);
  }
  root->AddChild(device);

  // Subscribe to DVL readings
  WaitForMessageTestHelper<
    msgs::DVLVelocityTracking
  > msgHelper(sensor->Topic());
  EXPECT_TRUE(sensor->HasConnections());

  // Update DVL readings
  sensors::EntityKinematicState & deviceState =
      this->worldState.kinematics[deviceEntity];
  deviceState.pose = devicePose;
  sensor->SetWorldState(this->worldState);

  const auto now = (
    std::chrono::seconds(100) +
    std::chrono::nanoseconds(100));
  this->scene->PreRender();
  sensor->Update(now);
  this->scene->PostRender();
  sensor->PostUpdate(now);

  EXPECT_TRUE(msgHelper.WaitForMessage(std::chrono::seconds(10)));

  // Verify DVL readings
  const msgs::DVLVelocityTracking & message = msgHelper.Message();
  EXPECT_EQ(now, msgs::Convert(message.header().stamp()));
  using DVLVelocityTracking = msgs::DVLVelocityTracking;
  EXPECT_EQ(DVLVelocityTracking::DVL_TYPE_PHASED_ARRAY, message.type());
  using DVLTrackingTarget = msgs::DVLTrackingTarget;
  EXPECT_EQ(DVLTrackingTarget::DVL_TARGET_BOTTOM, message.target().type());
  using DVLKinematicEstimate = msgs::DVLKinematicEstimate;
  constexpr auto velocityReference = DVLKinematicEstimate::DVL_REFERENCE_SHIP;
  EXPECT_EQ(velocityReference, message.velocity().reference());
  EXPECT_TRUE(math::Vector3d::Zero.Equal(
    msgs::Convert(message.velocity().mean()),
    4 * config.trackingNoise));
  EXPECT_EQ(4, message.beams_size());
  for (int i = 0; i < message.beams_size(); ++i)
  {
    EXPECT_TRUE(message.beams(i).locked());

    EXPECT_LE(message.target().range().mean(),
              message.beams(i).range().mean());
    // check range measurements are consistent
    // with nominal beam tilt and aperture angles
    const double estimatedTiltAngle = GZ_RTOD(std::acos(
      seabedDepth / message.target().range().mean()));
    EXPECT_NEAR(estimatedTiltAngle, config.tiltAngle,
                config.apertureAngle / 2);

    EXPECT_EQ(velocityReference, message.beams(i).velocity().reference());
    EXPECT_TRUE(math::Vector3d::Zero.Equal(
      msgs::Convert(message.beams(i).velocity().mean()),
      4 * config.trackingNoise));
  }
  EXPECT_EQ(0, message.status());

  this->manager.Remove(sensor->Id());
}

/////////////////////////////////////////////////
TEST_P(DopplerVelocityLogTest, WaterMassTrackingWhileStatic)
{
  // Add DVL sensor
  DVLConfig config;
  config.waterMassTrackingMode = "always";
  auto *sensor = this->manager.
      CreateSensor<DopplerVelocityLog>(MakeDVLSdf(config));

  constexpr uint64_t deviceEntity = 200u;
  sensor->SetEntity(deviceEntity);
  sensor->SetScene(this->scene);
  sensor->SetManualSceneUpdate(true);

  rendering::VisualPtr root = this->scene->RootVisual();
  rendering::VisualPtr device = this->scene->CreateVisual();
  const math::Pose3d devicePose(
      math::Vector3d(0., 0., -10),
      math::Quaterniond::Identity);
  device->SetLocalPose(devicePose);
  device->SetUserData("gazebo-entity", deviceEntity);
  for (auto renderingSensor : sensor->RenderingSensors())
  {
      device->AddChild(renderingSensor);
  }
  root->AddChild(device);

  // Subscribe to DVL readings
  WaitForMessageTestHelper<
    msgs::DVLVelocityTracking
  > msgHelper(sensor->Topic());
  EXPECT_TRUE(sensor->HasConnections());

  // Update DVL readings
  sensors::EntityKinematicState & deviceState =
      this->worldState.kinematics[deviceEntity];
  deviceState.pose = devicePose;
  sensor->SetWorldState(this->worldState);
  sensor->SetEnvironmentalData(*this->environment);

  const auto now = (
    std::chrono::seconds(100) +
    std::chrono::nanoseconds(100));
  this->scene->PreRender();
  sensor->Update(now);
  this->scene->PostRender();
  sensor->PostUpdate(now);

  EXPECT_TRUE(msgHelper.WaitForMessage(std::chrono::seconds(10)));

  // Verify DVL readings
  const msgs::DVLVelocityTracking & message = msgHelper.Message();
  EXPECT_EQ(now, msgs::Convert(message.header().stamp()));
  using DVLVelocityTracking = msgs::DVLVelocityTracking;
  EXPECT_EQ(DVLVelocityTracking::DVL_TYPE_PHASED_ARRAY, message.type());
  using DVLTrackingTarget = msgs::DVLTrackingTarget;
  EXPECT_EQ(DVLTrackingTarget::DVL_TARGET_WATER_MASS, message.target().type());
  using DVLKinematicEstimate = msgs::DVLKinematicEstimate;
  constexpr auto velocityReference = DVLKinematicEstimate::DVL_REFERENCE_SHIP;
  EXPECT_EQ(velocityReference, message.velocity().reference());
  const math::Vector3d waterVelocity(1.0, 0.5, 0.0);
  EXPECT_TRUE((-waterVelocity).Equal(
    msgs::Convert(message.velocity().mean()),
    4 * config.trackingNoise));
  EXPECT_EQ(4, message.beams_size());
  for (int i = 0; i < message.beams_size(); ++i)
  {
    EXPECT_TRUE(message.beams(i).locked());
    const math::Quaterniond beamRotation(
      0., GZ_DTOR(config.tiltAngle),
      -GZ_DTOR(config.rotationAngles[i]));
    const auto beamAxis = beamRotation * -math::Vector3d::UnitZ;

    const double meanBeamRange = (
        config.waterMassNearBoundary + config.waterMassFarBoundary
    ) / (-2. * math::Vector3d::UnitZ.Dot(beamAxis));
    EXPECT_DOUBLE_EQ(meanBeamRange, message.beams(i).range().mean());
    EXPECT_LE(message.target().range().mean(),
              message.beams(i).range().mean());

    const auto beamVelocity = beamAxis * beamAxis.Dot(-waterVelocity);
    EXPECT_TRUE(beamVelocity.Equal(
      msgs::Convert(message.beams(i).velocity().mean()),
      4 * config.trackingNoise));
    EXPECT_EQ(velocityReference, message.beams(i).velocity().reference());
  }
  EXPECT_EQ(0, message.status());

  this->manager.Remove(sensor->Id());
}

/////////////////////////////////////////////////
TEST_P(DopplerVelocityLogTest, BottomTrackingWhileInMotion)
{
  // Add DVL sensor
  DVLConfig config;
  config.bottomTrackingMode = "always";
  auto *sensor = this->manager.
      CreateSensor<DopplerVelocityLog>(MakeDVLSdf(config));

  constexpr uint64_t deviceEntity = 200u;
  sensor->SetEntity(deviceEntity);
  sensor->SetScene(this->scene);
  sensor->SetManualSceneUpdate(true);

  rendering::VisualPtr root = this->scene->RootVisual();
  rendering::VisualPtr device = this->scene->CreateVisual();
  const math::Pose3d devicePose(
      math::Vector3d::Zero,
      math::Quaterniond::Identity);
  device->SetLocalPose(devicePose);
  device->SetUserData("gazebo-entity", deviceEntity);
  for (auto renderingSensor : sensor->RenderingSensors())
  {
      device->AddChild(renderingSensor);
  }
  root->AddChild(device);

  // Subscribe to DVL readings
  WaitForMessageTestHelper<
    msgs::DVLVelocityTracking
  > msgHelper(sensor->Topic());
  EXPECT_TRUE(sensor->HasConnections());

  // Update DVL readings
  sensors::EntityKinematicState & deviceState =
      this->worldState.kinematics[deviceEntity];
  deviceState.pose = devicePose;
  deviceState.linearVelocity = math::Vector3d::UnitX;
  sensor->SetWorldState(this->worldState);

  const auto now = (
    std::chrono::seconds(100) +
    std::chrono::nanoseconds(100));
  this->scene->PreRender();
  sensor->Update(now);
  this->scene->PostRender();
  sensor->PostUpdate(now);

  EXPECT_TRUE(msgHelper.WaitForMessage(std::chrono::seconds(10)));

  // Verify DVL readings
  const msgs::DVLVelocityTracking & message = msgHelper.Message();
  EXPECT_EQ(now, msgs::Convert(message.header().stamp()));
  using DVLVelocityTracking = msgs::DVLVelocityTracking;
  EXPECT_EQ(DVLVelocityTracking::DVL_TYPE_PHASED_ARRAY, message.type());
  using DVLTrackingTarget = msgs::DVLTrackingTarget;
  EXPECT_EQ(DVLTrackingTarget::DVL_TARGET_BOTTOM, message.target().type());
  using DVLKinematicEstimate = msgs::DVLKinematicEstimate;
  constexpr auto velocityReference = DVLKinematicEstimate::DVL_REFERENCE_SHIP;
  EXPECT_EQ(velocityReference, message.velocity().reference());
  EXPECT_TRUE(math::Vector3d::UnitX.Equal(
    msgs::Convert(message.velocity().mean()),
    4 * config.trackingNoise));
  EXPECT_EQ(4, message.beams_size());
  for (int i = 0; i < message.beams_size(); ++i)
  {
    EXPECT_TRUE(message.beams(i).locked());
    const math::Quaterniond beamRotation(
      0., GZ_DTOR(config.tiltAngle),
      -GZ_DTOR(config.rotationAngles[i]));
    const auto beamAxis = beamRotation * -math::Vector3d::UnitZ;
    const auto beamVelocity =
      beamAxis * beamAxis.Dot(deviceState.linearVelocity);
    EXPECT_TRUE(beamVelocity.Equal(
      msgs::Convert(message.beams(i).velocity().mean()),
      4 * config.trackingNoise));
    EXPECT_EQ(velocityReference, message.beams(i).velocity().reference());
  }
  EXPECT_EQ(0, message.status());

  this->manager.Remove(sensor->Id());
}

INSTANTIATE_TEST_SUITE_P(DopplerVelocityLogTests, DopplerVelocityLogTest,
    RENDER_ENGINE_VALUES, rendering::PrintToStringParam());
