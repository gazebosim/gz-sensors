/*
 * copyright (c) 2017 open source robotics foundation
 *
 * licensed under the apache license, version 2.0 (the "license");
 * you may not use this file except in compliance with the license.
 * you may obtain a copy of the license at
 *
 *     http://www.apache.org/licenses/license-2.0
 *
 * unless required by applicable law or agreed to in writing, software
 * distributed under the license is distributed on an "as is" basis,
 * without warranties or conditions of any kind, either express or implied.
 * see the license for the specific language governing permissions and
 * limitations under the license.
 *
*/

#include <ignition/sensors/GpuLidarSensor.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>
#include <ignition/math/Angle.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/sensors/Events.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/transport.hh>

// Generated header that doesn't get installed
#include "gpu_lidar/shaders.hh"

using namespace ignition::sensors;


class ignition::sensors::GpuLidarSensorPrivate
{
  /// \brief constructor
  public: GpuLidarSensorPrivate();

  /// \brief destructor
  public: ~GpuLidarSensorPrivate();

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: ignition::common::EventT<
          void (const ignition::msgs::LaserScan &)> dataEvent;

  /// \brief A scene the sensor is generating data from
  public: ignition::rendering::ScenePtr scene;

  /// \brief Camera used to create data
  public: ignition::rendering::CameraPtr camera;

  /// \brief Pointer to an image that contains range data
  public: ignition::rendering::Image image;

  /// \brief Connection to the Manager's scene change event.
  public: ignition::common::ConnectionPtr sceneChangeConnection;

  /// \brief callback that sets the scene pointer
  public: void SetScene(ignition::rendering::ScenePtr _scene);

  /// \brief create a camera
  public: bool CreateCamera();

  /// \brief remove campera
  public: void RemoveCamera(ignition::rendering::ScenePtr _scene);
};

//////////////////////////////////////////////////
GpuLidarSensorPrivate::GpuLidarSensorPrivate()
{
  this->sceneChangeConnection = Events::ConnectSceneChangeCallback(
      std::bind(&GpuLidarSensorPrivate::SetScene,this, std::placeholders::_1));
}

//////////////////////////////////////////////////
GpuLidarSensorPrivate::~GpuLidarSensorPrivate()
{
  Events::sceneEvent.Disconnect(this->sceneChangeConnection->Id());
}

/////////////////////////////////////////////////
void GpuLidarSensorPrivate::SetScene(ignition::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->RemoveCamera(this->scene);
  this->scene = _scene;
  if (this->initialized)
    this->CreateCamera();
}

//////////////////////////////////////////////////
bool GpuLidarSensorPrivate::CreateCamera()
{
  this->camera = this->scene->CreateCamera("todo_some_unique_name");
  // TODO(Sloretz) populate from SDF
  // temp params: 90 FOV, 1 pixel per degree
  this->camera->SetImageWidth(90);
  this->camera->SetImageHeight(1);
  this->camera->SetAspectRatio(90.0 / 1.0);
  this->camera->SetHFOV(1.570796);
  this->camera->SetImageFormat(ignition::rendering::PF_R8G8B8);
  this->image = this->camera->CreateImage();
  this->scene->RootVisual()->AddChild(this->camera);
}

void GpuLidarSensorPrivate::RemoveCamera(ignition::rendering::ScenePtr _scene)
{
  if (_scene)
  {
    // TODO Remove camera from scene!
  }
  this->camera = nullptr;
}

//////////////////////////////////////////////////
GpuLidarSensor::GpuLidarSensor()
  : dataPtr(new GpuLidarSensorPrivate())
{
}

//////////////////////////////////////////////////
GpuLidarSensor::~GpuLidarSensor()
{
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Load(sdf::ElementPtr _sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (!this->Sensor::Load(_sdf))
    return false;

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::LaserScan>(this->Topic());
  if (!this->dataPtr->pub)
    return false;

  this->dataPtr->initialized = true;
  return true;
}

/////////////////////////////////////////////////
ignition::common::ConnectionPtr GpuLidarSensor::ConnectCallback(
    std::function<void(const ignition::msgs::LaserScan &)> _callback)
{
  return this->dataPtr->dataEvent.Connect(_callback);
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Update(const common::Time &_now)
{
  if (!this->dataPtr->initialized)
    return false;

  // TODO(sloretz) Generate data

  // create message
  ignition::msgs::LaserScan msg;
  msg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
  msg.mutable_header()->mutable_stamp()->set_nsec(_now.nsec);

  // publish
  this->dataPtr->pub.Publish(msg);

  // Trigger callbacks.
  try
  {
    this->dataPtr->dataEvent(msg);
  }
  catch(...)
  {
    ignerr << "Exception thrown in an image callback.\n";
  }
  return true;
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(
    ignition::sensors::GpuLidarSensor,
    ignition::sensors::Sensor)
