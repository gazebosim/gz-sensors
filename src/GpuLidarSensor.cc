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
#include <ignition/msgs/pointcloud_packed.pb.h>

#include <ignition/common/Console.hh>
#include "ignition/sensors/GpuLidarSensor.hh"
#include "ignition/sensors/SensorFactory.hh"

#include "PointCloud.hh"

using namespace ignition::sensors;

/// \brief Private data for the GpuLidar class
class ignition::sensors::GpuLidarSensorPrivate
{
  public: void FillMsg();

  /// \brief Rendering camera
  public: ignition::rendering::GpuRaysPtr gpuRays;

  /// \brief Connection to the Manager's scene change event.
  public: ignition::common::ConnectionPtr sceneChangeConnection;

  /// \brief The point cloud message.
  public: msgs::PointCloudPacked pointMsg;

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish point cloud
  public: transport::Node::Publisher pointPub;

  public: PointCloud pointCloud;
};

//////////////////////////////////////////////////
GpuLidarSensor::GpuLidarSensor()
  : dataPtr(new GpuLidarSensorPrivate())
{
}

//////////////////////////////////////////////////
GpuLidarSensor::~GpuLidarSensor()
{
  this->RemoveGpuRays(this->Scene());

  this->dataPtr->sceneChangeConnection.reset();

  if (this->laserBuffer)
  {
    delete [] this->laserBuffer;
    this->laserBuffer = nullptr;
  }
}

/////////////////////////////////////////////////
void GpuLidarSensor::SetScene(ignition::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->lidarMutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    this->RemoveGpuRays(this->Scene());
    RenderingSensor::SetScene(_scene);

    if (this->initialized)
      this->CreateLidar();
  }
}

//////////////////////////////////////////////////
void GpuLidarSensor::RemoveGpuRays(
    ignition::rendering::ScenePtr _scene)
{
  if (_scene)
  {
    _scene->DestroySensor(this->dataPtr->gpuRays);
  }
  this->dataPtr->gpuRays.reset();
  this->dataPtr->gpuRays = nullptr;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Load(const sdf::Sensor &_sdf)
{
  // Check if this is being loaded via "builtin" or via a plugin
  if (!Lidar::Load(_sdf))
  {
    return false;
  }

  if (this->Scene())
    this->CreateLidar();

  this->dataPtr->sceneChangeConnection =
    RenderingEvents::ConnectSceneChangeCallback(
        std::bind(&GpuLidarSensor::SetScene, this, std::placeholders::_1));

  // Create the depth image publisher
  this->dataPtr->pointPub =
      this->dataPtr->node.Advertise<ignition::msgs::PointCloudPacked>(
          this->Topic() + "/points");
  if (!this->dataPtr->pointPub)
    return false;


  this->initialized = true;

  // Setup the point cloud message.
  uint32_t offset = 0;
  msgs::PointCloudPacked::Field *field = this->dataPtr->pointMsg.add_field();
  field->set_name("x");
  field->set_count(1);
  field->set_datatype(msgs::PointCloudPacked::Field::FLOAT32);
  field->set_offset(offset);
  offset += 4;

  field = this->dataPtr->pointMsg.add_field();
  field->set_name("y");
  field->set_count(1);
  field->set_datatype(msgs::PointCloudPacked::Field::FLOAT32);
  field->set_offset(offset);
  offset += 4;

  field = this->dataPtr->pointMsg.add_field();
  field->set_name("z");
  field->set_count(1);
  field->set_datatype(msgs::PointCloudPacked::Field::FLOAT32);
  field->set_offset(offset);
  offset += 4;

  this->dataPtr->pointMsg.set_point_step(offset);

  // Set the frame
  msgs::Header::Map *frame =
    this->dataPtr->pointMsg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  return true;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool GpuLidarSensor::CreateLidar()
{
  this->dataPtr->gpuRays = this->Scene()->CreateGpuRays(
      this->Name());

  if (!this->dataPtr->gpuRays)
  {
    ignerr << "Unable to create gpu laser sensor\n";
    return false;
  }

  this->dataPtr->gpuRays->SetWorldPosition(this->Pose().Pos());
  this->dataPtr->gpuRays->SetWorldRotation(this->Pose().Rot());

  this->dataPtr->gpuRays->SetNearClipPlane(this->RangeMin());
  this->dataPtr->gpuRays->SetFarClipPlane(this->RangeMax());

  // Mask ranges outside of min/max to +/- inf, as per REP 117
  this->dataPtr->gpuRays->SetClamp(false);

  this->dataPtr->gpuRays->SetAngleMin(this->AngleMin().Radian());
  this->dataPtr->gpuRays->SetAngleMax(this->AngleMax().Radian());

  this->dataPtr->gpuRays->SetVerticalAngleMin(
      this->VerticalAngleMin().Radian());
  this->dataPtr->gpuRays->SetVerticalAngleMax(
      this->VerticalAngleMax().Radian());

  this->dataPtr->gpuRays->SetRayCount(this->RayCount());
  this->dataPtr->gpuRays->SetVerticalRayCount(
      this->VerticalRayCount());

  this->Scene()->RootVisual()->AddChild(
      this->dataPtr->gpuRays);

  return true;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Update(const ignition::common::Time &_now)
{
  if (!this->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->gpuRays)
  {
    ignerr << "GpuRays doesn't exist.\n";
    return false;
  }

  int len = this->dataPtr->gpuRays->RayCount() *
    this->dataPtr->gpuRays->VerticalRayCount() * 3;

  if (this->laserBuffer == nullptr)
  {
    this->laserBuffer = new float[len];
  }

  this->dataPtr->gpuRays->Update();
  this->dataPtr->gpuRays->Copy(this->laserBuffer);

  this->PublishLidarScan(_now);

  {
    // Set the time stamp
    this->dataPtr->pointMsg.mutable_header()->mutable_stamp()->set_sec(
        _now.sec);
    this->dataPtr->pointMsg.mutable_header()->mutable_stamp()->set_nsec(
        _now.nsec);
    this->dataPtr->pointMsg.set_width(this->dataPtr->gpuRays->ImageWidth());
    this->dataPtr->pointMsg.set_height(this->dataPtr->gpuRays->ImageHeight());
    this->dataPtr->pointMsg.set_row_step(
        this->dataPtr->pointMsg.point_step() *
        this->dataPtr->gpuRays->ImageWidth() *
        this->dataPtr->gpuRays->ImageHeight());

    this->dataPtr->pointMsg.set_is_dense(true);

    this->dataPtr->FillMsg();

    this->dataPtr->pointPub.Publish(this->dataPtr->pointMsg);
  }
  return true;
}

/////////////////////////////////////////////////
ignition::common::ConnectionPtr GpuLidarSensor::ConnectNewLidarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _heighti, unsigned int _channels,
                  const std::string &/*_format*/)> _subscriber)
{
  return this->dataPtr->gpuRays->ConnectNewGpuRaysFrame(_subscriber);
}

/////////////////////////////////////////////////
ignition::rendering::GpuRaysPtr GpuLidarSensor::GpuRays() const
{
  return this->dataPtr->gpuRays;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::IsHorizontal() const
{
  return this->dataPtr->gpuRays->IsHorizontal();
}

//////////////////////////////////////////////////
ignition::math::Angle GpuLidarSensor::HFOV() const
{
  return this->dataPtr->gpuRays->HFOV();
}

//////////////////////////////////////////////////
ignition::math::Angle GpuLidarSensor::VFOV() const
{
  return this->dataPtr->gpuRays->VFOV();
}

//////////////////////////////////////////////////
void GpuLidarSensorPrivate::FillMsg()
{
  uint32_t width = this->pointMsg.width();
  uint32_t height = this->pointMsg.height();

  double angleStep =
    (this->gpuRays->AngleMax() - this->gpuRays->AngleMin()).Radian() /
    (this->gpuRays->RangeCount()-1);

  double verticleAngleStep = (this->gpuRays->VerticalAngleMax() -
      this->gpuRays->VerticalAngleMin()).Radian() /
    (this->gpuRays->VerticalRangeCount()-1);

  // Angles of ray currently processing, azimuth is horizontal, inclination
  // is vertical
  double inclination = this->gpuRays->VerticalAngleMin().Radian();
  int channels = 3;

  std::string *msgBuffer = this->pointMsg.mutable_data();
  msgBuffer->resize(this->pointMsg.row_step());
  char *msgBufferIndex = msgBuffer->data();

  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < height; ++j)
  {
    double azimuth = this->gpuRays->AngleMin().Radian();

    for (uint32_t i = 0; i < width; ++i)
    {
      // Index of current point
      auto index = j * width * channels + i * channels;
      double depth = this->gpuRays->Data()[index];

      // Convert spherical coordinates to Cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      *reinterpret_cast<float*>(msgBufferIndex) =
        depth * std::cos(inclination) * std::cos(azimuth);
      msgBufferIndex += 4;
      *reinterpret_cast<float*>(msgBufferIndex) =
        depth * std::cos(inclination) * std::sin(azimuth);
      msgBufferIndex += 4;
      *reinterpret_cast<float*>(msgBufferIndex) = depth * std::sin(inclination);
      msgBufferIndex += 4;

      azimuth += angleStep;
    }
    inclination += verticleAngleStep;
  }
}

IGN_SENSORS_REGISTER_SENSOR(GpuLidarSensor)
