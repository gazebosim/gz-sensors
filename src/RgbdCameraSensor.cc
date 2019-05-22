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

#include <ignition/msgs/image.pb.h>
#include <ignition/msgs/pointcloud2.pb.h>

#include <ignition/common/Image.hh>
#include <ignition/math/Helpers.hh>

#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/DepthCamera.hh>

#include <sdf/Sensor.hh>

#include "ignition/sensors/RgbdCameraSensor.hh"
#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/GaussianNoiseModel.hh"

/// \brief Private data for RgbdCameraSensor
class ignition::sensors::RgbdCameraSensorPrivate
{
  /// \brief Create the cameras in a scene.
  /// \return True on success.
  public: bool CreateCameras(const std::string &_name,
              ignition::rendering::ScenePtr _scene);


  /// \brief Remove a camera from a scene
  public: void RemoveCamera(ignition::rendering::ScenePtr _scene);

  public: void CreateAndPublishPointCloud();

  /// \brief Depth data callback used to get the data from the sensor
  /// \param[in] _scan pointer to the data from the sensor
  /// \param[in] _width width of the depth image
  /// \param[in] _height height of the depth image
  /// \param[in] _channel bytes used for the depth data
  /// \param[in] _format string with the format
  public: void OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &_format);

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher imagePub;

  /// \brief publisher to publish depth images
  public: transport::Node::Publisher depthPub;

  /// \brief publisher to publish points clouds
  public: transport::Node::Publisher pointPub;

  /// \brief Camera info publisher to publish images
  public: transport::Node::Publisher infoPub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Rendering camera
  public: ignition::rendering::CameraPtr camera;

  /// \brief Rendering camera
  public: ignition::rendering::DepthCameraPtr depthCamera;

  /// \brief Depth data buffer.
  public: float *depthBuffer = nullptr;

  /// \brief Pointer to an image to be published
  public: ignition::rendering::Image image;

  /// \brief Pointer to a depth image to be published
  public: ignition::rendering::Image depthImage;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: ignition::common::EventT<
          void(const ignition::msgs::Image &)> imageEvent;

  /// \brief Connection from depth camera with a new image
  public: ignition::common::ConnectionPtr connection;

  /// \brief Connection to the Manager's scene change event.
  public: ignition::common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief True to save images
  public: bool saveImage = false;

  /// \brief path directory to where images are saved
  public: std::string saveImagePath = "./";

  /// \prefix of an image name
  public: std::string saveImagePrefix = "./";

  /// \brief counter used to set the image filename
  public: std::uint64_t saveImageCounter = 0;

  /// \brief SDF Sensor DOM object.
  public: sdf::Sensor sdfSensor;

  /// \brief Camera information message.
  public: msgs::CameraInfo infoMsg;

  /// \brief Depth camera near clip.
  public: double depthNear = 0.1;

  /// \brief Depth camera far clip.
  public: double depthFar = 6.0;
};

using namespace ignition;
using namespace sensors;

//////////////////////////////////////////////////
void RgbdCameraSensorPrivate::RemoveCamera(
    rendering::ScenePtr _scene)
{
  if (_scene)
  {
    // \todo(nkoenig) Remove camera from scene!
  }
  // this->depthCamera = nullptr;
}


//////////////////////////////////////////////////
RgbdCameraSensor::RgbdCameraSensor()
  : dataPtr(new RgbdCameraSensorPrivate())
{
}

//////////////////////////////////////////////////
RgbdCameraSensor::~RgbdCameraSensor()
{
  this->dataPtr->connection.reset();
  if (this->dataPtr->depthBuffer)
    delete [] this->dataPtr->depthBuffer;
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::Load(const sdf::Sensor &_sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!Sensor::Load(_sdf))
  {
    return false;
  }

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::RGBD_CAMERA)
  {
    ignerr << "Attempting to a load a RGBD Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    ignerr << "Attempting to a load an RGBD Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  // Create the 2d image publisher
  this->dataPtr->imagePub =
      this->dataPtr->node.Advertise<ignition::msgs::Image>(
          this->Topic() + "/image");
  if (!this->dataPtr->imagePub)
    return false;

  // Create the depth image publisher
  this->dataPtr->depthPub =
      this->dataPtr->node.Advertise<ignition::msgs::Image>(
          this->Topic() + "/depth_image");
  if (!this->dataPtr->depthPub)
    return false;

  // Create the point cloud publisher
  this->dataPtr->pointPub =
      this->dataPtr->node.Advertise<ignition::msgs::PointCloud2>(
          this->Topic() + "/points");
  if (!this->dataPtr->pointPub)
    return false;

  this->dataPtr->infoPub =
      this->dataPtr->node.Advertise<ignition::msgs::CameraInfo>(
          this->Topic() + "/camera_info");
  if (!this->dataPtr->infoPub)
    return false;

  if (this->Scene())
    this->dataPtr->CreateCameras(this->Name(), this->Scene());

  this->dataPtr->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool RgbdCameraSensorPrivate::CreateCameras(const std::string &_name,
              ignition::rendering::ScenePtr _scene)
{
  const sdf::Camera *cameraSdf = this->sdfSensor.CameraSensor();

  if (!cameraSdf)
  {
    ignerr << "Unable to access camera SDF element\n";
    return false;
  }

  int width = cameraSdf->ImageWidth();
  int height = cameraSdf->ImageHeight();

  // Set some values of the camera info message.
  {
    msgs::CameraInfo::Distortion *distortion =
      this->infoMsg.mutable_distortion();

    distortion->set_distortion_model(msgs::CameraInfo::Distortion::PLUMB_BOB);
    distortion->set_k1(cameraSdf->DistortionK1());
    distortion->set_k2(cameraSdf->DistortionK2());
    distortion->set_k3(cameraSdf->DistortionK3());
    distortion->set_t1(cameraSdf->DistortionP1());
    distortion->set_t2(cameraSdf->DistortionP2());

    msgs::CameraInfo::Intrinsics *intrinsics =
      this->infoMsg.mutable_intrinsics();
    intrinsics->set_fx(cameraSdf->LensIntrinsicsFx());
    intrinsics->set_fy(cameraSdf->LensIntrinsicsFy());
    intrinsics->set_cx(cameraSdf->LensIntrinsicsCx());
    intrinsics->set_cy(cameraSdf->LensIntrinsicsCy());

    msgs::CameraInfo::Projection *proj =
      this->infoMsg.mutable_projection();
    proj->set_fx(cameraSdf->LensIntrinsicsFx());
    proj->set_fy(cameraSdf->LensIntrinsicsFy());
    proj->set_cx(cameraSdf->LensIntrinsicsCx());
    proj->set_cy(cameraSdf->LensIntrinsicsCy());

    // Set the rectifcation matrix to identity
    this->infoMsg.add_rectification_matrix(1.0);
    this->infoMsg.add_rectification_matrix(0.0);
    this->infoMsg.add_rectification_matrix(0.0);

    this->infoMsg.add_rectification_matrix(0.0);
    this->infoMsg.add_rectification_matrix(1.0);
    this->infoMsg.add_rectification_matrix(0.0);

    this->infoMsg.add_rectification_matrix(0.0);
    this->infoMsg.add_rectification_matrix(0.0);
    this->infoMsg.add_rectification_matrix(1.0);

    auto infoFrame = this->infoMsg.mutable_header()->add_data();
    infoFrame->set_key("frame_id");
    infoFrame->add_value(_name);

    this->infoMsg.set_width(width);
    this->infoMsg.set_height(height);
  }

  this->depthCamera = _scene->CreateDepthCamera(_name + "_depth");
  this->depthCamera->SetImageWidth(width);
  this->depthCamera->SetImageHeight(height);
  // \todo(nkoenig) Fix this to be a parameter.
  this->depthCamera->SetFarClipPlane(this->depthFar);

  this->camera = _scene->CreateCamera(_name);
  this->camera->SetImageWidth(width);
  this->camera->SetImageHeight(height);
  this->camera->SetNearClipPlane(cameraSdf->NearClip());
  this->camera->SetFarClipPlane(cameraSdf->FarClip());

  /*const std::map<SensorNoiseType, sdf::Noise> noises = {
    {CAMERA_NOISE, cameraSdf->ImageNoise()},
  };

  for (const auto & [noiseType, noiseSdf] : noises)
  {
    // Add gaussian noise to camera sensor
    if (noiseSdf.Type() == sdf::NoiseType::GAUSSIAN)
    {
      this->noises[noiseType] =
        NoiseFactory::NewNoiseModel(noiseSdf, "depth");

      std::dynamic_pointer_cast<ImageGaussianNoiseModel>(
           this->noises[noiseType])->SetCamera(
             this->depthCamera);
    }
    else if (noiseSdf.Type() != sdf::NoiseType::NONE)
    {
      ignwarn << "The depth camera sensor only supports Gaussian noise. "
       << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
       << "] is not supported." << std::endl;
    }
  }
  */

  // \todo(nkoeng) these parameters via sdf
  this->depthCamera->SetAntiAliasing(2);
  this->camera->SetAntiAliasing(2);

  math::Angle angle = cameraSdf->HorizontalFov();
  if (angle < 0.01 || angle > IGN_PI*2)
  {
    ignerr << "Invalid horizontal field of view [" << angle << "]\n";

    return false;
  }

  this->depthCamera->SetAspectRatio(static_cast<double>(width)/height);
  this->depthCamera->SetHFOV(angle);
  this->camera->SetAspectRatio(static_cast<double>(width)/height);
  this->camera->SetHFOV(angle);

  // Create depth texture when the camera is reconfigured from default values
  this->depthCamera->CreateDepthTexture();

  // \todo(nkoenig) Port Distortion class
  // This->distortion.reset(new Distortion());
  // This->distortion->Load(this->sdf->GetElement("distortion"));

  this->depthCamera->SetImageFormat(ignition::rendering::PF_FLOAT32_R);
  this->depthImage = this->depthCamera->CreateImage();

  this->camera->SetImageFormat(ignition::rendering::PF_R8G8B8);
  this->image = this->depthCamera->CreateImage();

  _scene->RootVisual()->AddChild(this->depthCamera);
  _scene->RootVisual()->AddChild(this->camera);

  this->connection = this->depthCamera->ConnectNewDepthFrame(
      std::bind(&RgbdCameraSensorPrivate::OnNewDepthFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  return true;
}

/////////////////////////////////////////////////
void RgbdCameraSensor::SetScene(ignition::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    this->dataPtr->RemoveCamera(this->Scene());
    RenderingSensor::SetScene(_scene);

    if (this->dataPtr->initialized)
      this->dataPtr->CreateCameras(this->Name(), _scene);
  }
}

/////////////////////////////////////////////////
void RgbdCameraSensorPrivate::OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &/*_format*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  unsigned int depthSamples = _width * _height;
  unsigned int depthBufferSize = depthSamples * sizeof(float);

  if (!this->depthBuffer)
    this->depthBuffer = new float[depthSamples];

  memcpy(this->depthBuffer, _scan, depthBufferSize);

  for (unsigned int i = 0; i < depthSamples; ++i)
  {
    // Mask ranges outside of min/max to +/- inf, as per REP 117
    if (this->depthBuffer[i] >= this->depthFar)
    {
      this->depthBuffer[i] = ignition::math::INF_D;
    }
    else if (this->depthBuffer[i] <= this->depthNear)
    {
      this->depthBuffer[i] = -ignition::math::INF_D;
    }
  }
}

//////////////////////////////////////////////////
bool RgbdCameraSensor::Update(const ignition::common::Time &_now)
{
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->depthCamera || !this->dataPtr->camera)
  {
    ignerr << "Depth or image cameras do exist.\n";
    return false;
  }

  // move the depth camera to the current pose
  // \todo(nkoenig) Ignition Gazebo attaches the rendering camera with
  // this->Name() to the appropriate model link. When physics updates the
  // link's pose, the visual is also moved. This in turn moves the rendering
  // rendering camera. The problem is that the RGBD sensor has two cameras,
  // one with a name that equals 'this->Name()' and one with a name that
  // equal 'this->Name() + "_depth"'. In order to make the depth camera
  // move, we have to manually set the world pose of the depth camera to
  // match this->dataPtr->camera.
  //
  // It would be nice if the whole sensor + rendering + gazebo pipeline
  // could handle this use-case more elegantly.
  this->dataPtr->depthCamera->SetWorldPose(this->dataPtr->camera->WorldPose());

  unsigned int width = this->dataPtr->camera->ImageWidth();
  unsigned int height = this->dataPtr->camera->ImageHeight();

  // create and publish the 2d image message
  {
    this->dataPtr->camera->Capture(this->dataPtr->image);
    unsigned char *data = this->dataPtr->image.Data<unsigned char>();

    ignition::msgs::Image msg;
    msg.set_width(width);
    msg.set_height(height);
    msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
          this->dataPtr->camera->ImageFormat()));
    msg.set_pixel_format_type(msgs::PixelFormatType::RGB_INT8);
    msg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
    msg.mutable_header()->mutable_stamp()->set_nsec(_now.nsec);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->Name());
    msg.set_data(data, this->dataPtr->camera->ImageMemorySize());

    // publish the image message
    this->dataPtr->imagePub.Publish(msg);
  }

  // create and publish the depthmessage
  {
    // generate sensor data
    this->dataPtr->depthCamera->Update();

    ignition::msgs::Image msg;
    msg.set_width(width);
    msg.set_height(height);
    msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
          this->dataPtr->depthCamera->ImageFormat()));
    msg.set_pixel_format_type(msgs::PixelFormatType::R_FLOAT32);
    msg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
    msg.mutable_header()->mutable_stamp()->set_nsec(_now.nsec);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->Name());

    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    msg.set_data(this->dataPtr->depthBuffer,
        this->dataPtr->depthCamera->ImageMemorySize());

    // publish
    this->dataPtr->depthPub.Publish(msg);
  }

  // Create and publish point cloud information
  this->dataPtr->CreateAndPublishPointCloud();

  // publish the camera info message
  this->dataPtr->infoMsg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
  this->dataPtr->infoMsg.mutable_header()->mutable_stamp()->set_nsec(
      _now.nsec);
  this->dataPtr->infoPub.Publish(this->dataPtr->infoMsg);

  return true;
}

//////////////////////////////////////////////////
unsigned int RgbdCameraSensor::ImageWidth() const
{
  return this->dataPtr->depthCamera->ImageWidth();
}

//////////////////////////////////////////////////
unsigned int RgbdCameraSensor::ImageHeight() const
{
  return this->dataPtr->depthCamera->ImageHeight();
}

//////////////////////////////////////////////////
double RgbdCameraSensor::FarClip() const
{
  return this->dataPtr->depthCamera->FarClipPlane();
}

//////////////////////////////////////////////////
double RgbdCameraSensor::NearClip() const
{
  return this->dataPtr->sdfSensor.CameraSensor()->NearClip();
}

//////////////////////////////////////////////////
void RgbdCameraSensorPrivate::CreateAndPublishPointCloud()
{
  // This code was inspired by
  // https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_openni_kinect.cpp
  msgs::PointCloud2 msg;

  double hfov = this->depthCamera->HFOV().Radian();
  unsigned int width = this->depthCamera->ImageWidth();
  unsigned int height = this->depthCamera->ImageHeight();

  double fl = width / (2.0 * tan(hfov/2.0));

  int index = 0;

  unsigned char *imageData = this->image.Data<unsigned char>();

  // convert depth to point cloud
  for (unsigned int y = 0; y < height; ++y)
  {
    double pAngle = 0.0;
    if (height > 1)
    {
      pAngle = atan2(static_cast<double>(y) -
                     0.5 * static_cast<double>(height - 1), fl);
    }

    for (unsigned int x = 0; x < width; ++x)
    {
      double yAngle = 0.0;
      if (width > 1)
      {
        yAngle = atan2(static_cast<double>(x) -
                       0.5*static_cast<double>(width-1), fl);
      }

      double depth = this->depthBuffer[index++];
      msgs::Vector3d *pt = msg.add_points();
      pt->set_x(depth * tan(yAngle));
      pt->set_y(depth * tan(pAngle));
      pt->set_z(depth);

      // put image color data for each point.

      // \todo(nkoenig) we are assuming RGB_INT8 format, which is also
      // hardcoded in this sensor.
      msgs::Color *clr = msg.add_color();
      if (imageData)
      {
        clr->set_r(imageData[x*3 + y*height*3 + 0]);
        clr->set_g(imageData[x*3 + y*height*3 + 1]);
        clr->set_b(imageData[x*3 + y*height*3 + 2]);
      }
      else
      {
        // No color data
        clr->set_r(0);
        clr->set_g(0);
        clr->set_b(0);
      }
    }
  }

  this->pointPub.Publish(msg);
}

IGN_SENSORS_REGISTER_SENSOR(RgbdCameraSensor)
