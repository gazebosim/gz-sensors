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

#include <iostream>

#include <sdf/Camera.hh>
#include <sdf/Noise.hh>
#include <sdf/Sensor.hh>

#include <ignition/common/Image.hh>
#include <ignition/common/Console.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/rendering.hh>
#include <ignition/sensors.hh>

void OnImageFrame(const ignition::msgs::Image &_image)
{
  const unsigned char *data = reinterpret_cast<const unsigned char*>(
      _image.data().c_str());
  auto format = static_cast<ignition::common::Image::PixelFormatType>(
      _image.pixel_format_type());
  ignition::common::Image image;
  image.SetFromData(data, _image.width(), _image.height(), format);
  std::cout << "Saving image.png\n";
  image.SavePNG("image.png");
}

void BuildScene(ignition::rendering::ScenePtr _scene);

int main()
{
  // Setup ign-rendering with a scene
#ifdef WITH_OGRE
  auto engine = ignition::rendering::engine("ogre");
#else
#ifdef WITH_OGRE2
  auto engine = ignition::rendering::engine("ogre2");
#endif
#endif

  if (!engine)
  {
    std::cerr << "Failed to load ogre\n";
    return 1;
  }
  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");

  // Add stuff to take a picture of
  BuildScene(scene);

  // Create a sensor manager
  ignition::sensors::Manager mgr;

  // Create SDF describing a camera sensor
  const std::string name = "ExampleCamera";
  const std::string topic = "/ignition/sensors/examples/save_image";
  const double hz = 30;
  const std::size_t width = 480;
  const std::size_t height = 320;
  const std::size_t hfov = IGN_DTOR(60);
  const double near = 0.1;
  const double far = 100;
  const auto format = sdf::PixelFormatType::RGB_INT8;
  sdf::Camera cameraSdf;
  cameraSdf.SetImageWidth(width);
  cameraSdf.SetImageHeight(height);
  cameraSdf.SetHorizontalFov(hfov);
  cameraSdf.SetNearClip(near);
  cameraSdf.SetFarClip(far);
  cameraSdf.SetPixelFormat(format);

  // Add noise
  sdf::Noise noise;
  noise.SetMean(0.1);
  noise.SetStdDev(0.05);
  noise.SetType(sdf::NoiseType::GAUSSIAN);
  cameraSdf.SetImageNoise(noise);

  sdf::Sensor sensorSdf;
  sensorSdf.SetType(sdf::SensorType::CAMERA);
  sensorSdf.SetName(name);
  sensorSdf.SetTopic(topic);
  sensorSdf.SetUpdateRate(hz);
  sensorSdf.SetCameraSensor(cameraSdf);

  // Create a CameraSensor
  auto cameraSensor = mgr.CreateSensor<ignition::sensors::CameraSensor>(
      sensorSdf);

  if (!cameraSensor)
  {
    ignerr << "Unable to load camera sensor\n";
    return 1;
  }
  cameraSensor->SetScene(scene);

  // Set a callback on the camera sensor to get a camera frame
  ignition::common::ConnectionPtr connection =
    cameraSensor->ConnectImageCallback(&OnImageFrame);

  // Force the camera to generate an image
  mgr.RunOnce(std::chrono::system_clock::from_time_t(0), true);

  return 0;
}

// Copy/paste from an ignition-rendering example
void BuildScene(ignition::rendering::ScenePtr _scene)
{
  // initialize _scene
  _scene->SetAmbientLight(0.3, 0.3, 0.3);
  ignition::rendering::VisualPtr root = _scene->RootVisual();

  // create directional light
  ignition::rendering::DirectionalLightPtr light0 =
    _scene->CreateDirectionalLight();
  light0->SetDirection(-0.5, 0.5, -1);
  light0->SetDiffuseColor(0.5, 0.5, 0.5);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  // create point light
  ignition::rendering::PointLightPtr light2 = _scene->CreatePointLight();
  light2->SetDiffuseColor(0.5, 0.5, 0.5);
  light2->SetSpecularColor(0.5, 0.5, 0.5);
  light2->SetLocalPosition(3, 5, 5);
  root->AddChild(light2);

  // create green material
  ignition::rendering::MaterialPtr green = _scene->CreateMaterial();
  green->SetAmbient(0.0, 0.5, 0.0);
  green->SetDiffuse(0.0, 0.7, 0.0);
  green->SetSpecular(0.5, 0.5, 0.5);
  green->SetShininess(50);
  green->SetReflectivity(0);

  // create center visual
  ignition::rendering::VisualPtr center = _scene->CreateVisual();
  center->AddGeometry(_scene->CreateSphere());
  center->SetLocalPosition(3, 0, 0);
  center->SetLocalScale(0.1, 0.1, 0.1);
  center->SetMaterial(green);
  root->AddChild(center);

  // create red material
  ignition::rendering::MaterialPtr red = _scene->CreateMaterial();
  red->SetAmbient(0.5, 0.0, 0.0);
  red->SetDiffuse(1.0, 0.0, 0.0);
  red->SetSpecular(0.5, 0.5, 0.5);
  red->SetShininess(50);
  red->SetReflectivity(0);

  // create sphere visual
  ignition::rendering::VisualPtr sphere = _scene->CreateVisual();
  sphere->AddGeometry(_scene->CreateSphere());
  sphere->SetOrigin(0.0, -0.5, 0.0);
  sphere->SetLocalPosition(3, 0, 0);
  sphere->SetLocalRotation(0, 0, 0);
  sphere->SetLocalScale(1, 2.5, 1);
  sphere->SetMaterial(red);
  root->AddChild(sphere);

  // create blue material
  ignition::rendering::MaterialPtr blue = _scene->CreateMaterial();
  blue->SetAmbient(0.0, 0.0, 0.3);
  blue->SetDiffuse(0.0, 0.0, 0.8);
  blue->SetSpecular(0.5, 0.5, 0.5);
  blue->SetShininess(50);
  blue->SetReflectivity(0);

  // create box visual
  ignition::rendering::VisualPtr box = _scene->CreateVisual();
  box->AddGeometry(_scene->CreateBox());
  box->SetOrigin(0.0, 0.5, 0.0);
  box->SetLocalPosition(3, 0, 0);
  box->SetLocalRotation(IGN_PI / 4, 0, IGN_PI / 3);
  box->SetLocalScale(1, 2.5, 1);
  box->SetMaterial(blue);
  root->AddChild(box);

  // create white material
  ignition::rendering::MaterialPtr white = _scene->CreateMaterial();
  white->SetAmbient(0.5, 0.5, 0.5);
  white->SetDiffuse(0.8, 0.8, 0.8);
  white->SetReceiveShadows(true);
  white->SetReflectivity(0);

  // create sphere visual
  ignition::rendering::VisualPtr plane = _scene->CreateVisual();
  plane->AddGeometry(_scene->CreatePlane());
  plane->SetLocalScale(5, 8, 1);
  plane->SetLocalPosition(3, 0, -0.5);
  plane->SetMaterial(white);
  root->AddChild(plane);
}
