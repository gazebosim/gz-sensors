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
//#include "gpu_lidar/shaders.hh"

using namespace ignition::sensors;


class ignition::sensors::GpuLidarSensorPrivate : LidarSensorPrivate
{
  /// \brief constructor
  public: GpuLidarSensorPrivate();

  /// \brief destructor
  public: ~GpuLidarSensorPrivate();

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

  /// \brief True if the sensor was rendered.
  public: bool rendered;

  /// \brief GPU laser rendering.
  //public: rendering::GpuLaserPtr laserCam;
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


  return true;
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
  this->Fini();
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Init()
{
/*
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "Unable to create GpuRaySensor. Rendering is disabled.\n";
    return;
  }

  std::string worldName = this->world->Name();

  if (!worldName.empty())
  {
    this->scene = rendering::get_scene(worldName);

    if (!this->scene)
      this->scene = rendering::create_scene(worldName, false, true);

    this->dataPtr->laserCam = this->scene->CreateGpuLaser(
        this->sdf->Get<std::string>("name"), false);

    if (!this->dataPtr->laserCam)
    {
      gzerr << "Unable to create gpu laser sensor\n";
      return;
    }
    this->dataPtr->laserCam->SetCaptureData(true);

    // initialize GpuLaser from sdf
    if (this->dataPtr->vertRayCount == 1)
    {
      this->dataPtr->vertRangeCount = 1;
      this->dataPtr->laserCam->SetIsHorizontal(true);
    }
    else
      this->dataPtr->laserCam->SetIsHorizontal(false);

    this->dataPtr->rangeCountRatio =
      this->dataPtr->horzRangeCount / this->dataPtr->vertRangeCount;

    this->dataPtr->laserCam->SetNearClip(this->RangeMin());
    this->dataPtr->laserCam->SetFarClip(this->RangeMax());

    this->dataPtr->laserCam->SetHorzFOV(
        (this->AngleMax() - this->AngleMin()).Radian());
    this->dataPtr->laserCam->SetVertFOV(
        (this->VerticalAngleMax() - this->VerticalAngleMin()).Radian());

    this->dataPtr->laserCam->SetHorzHalfAngle(
      (this->AngleMax() + this->AngleMin()).Radian() / 2.0);

    this->dataPtr->laserCam->SetVertHalfAngle((this->VerticalAngleMax()
            + this->VerticalAngleMin()).Radian() / 2.0);

    if (this->HorzFOV() > 2 * M_PI)
      this->dataPtr->laserCam->SetHorzFOV(2*M_PI);

    this->dataPtr->laserCam->SetCameraCount(1);

    if (this->HorzFOV() > 2.8)
    {
      if (this->HorzFOV() > 5.6)
        this->dataPtr->laserCam->SetCameraCount(3);
      else
        this->dataPtr->laserCam->SetCameraCount(2);
    }

    this->dataPtr->laserCam->SetHorzFOV(this->HorzFOV() / this->CameraCount());
    this->dataPtr->horzRayCount /= this->CameraCount();

    if (this->VertFOV() > M_PI / 2)
    {
      gzwarn << "Vertical FOV for block GPU laser is capped at 90 degrees.\n";
      this->dataPtr->laserCam->SetVertFOV(M_PI / 2);
      this->SetVerticalAngleMin(this->dataPtr->laserCam->VertHalfAngle() -
                                (this->VertFOV() / 2));
      this->SetVerticalAngleMax(this->dataPtr->laserCam->VertHalfAngle() +
                                (this->VertFOV() / 2));
    }

    if ((this->dataPtr->horzRayCount * this->dataPtr->vertRayCount) <
        (this->dataPtr->horzRangeCount * this->dataPtr->vertRangeCount))
    {
      this->dataPtr->horzRayCount =
        std::max(this->dataPtr->horzRayCount, this->dataPtr->horzRangeCount);
      this->dataPtr->vertRayCount =
        std::max(this->dataPtr->vertRayCount, this->dataPtr->vertRangeCount);
    }

    if (this->dataPtr->laserCam->IsHorizontal())
    {
      if (this->dataPtr->vertRayCount > 1)
      {
        this->dataPtr->laserCam->SetCosHorzFOV(
          2 * atan(tan(this->HorzFOV()/2) / cos(this->VertFOV()/2)));
        this->dataPtr->laserCam->SetCosVertFOV(this->VertFOV());
        this->dataPtr->laserCam->SetRayCountRatio(
          tan(this->CosHorzFOV()/2.0) / tan(this->VertFOV()/2.0));

        if ((this->dataPtr->horzRayCount / this->RayCountRatio()) >
            this->dataPtr->vertRayCount)
        {
          this->dataPtr->vertRayCount =
            this->dataPtr->horzRayCount / this->RayCountRatio();
        }
        else
        {
          this->dataPtr->horzRayCount =
            this->dataPtr->vertRayCount * this->RayCountRatio();
        }
      }
      else
      {
        this->dataPtr->laserCam->SetCosHorzFOV(this->HorzFOV());
        this->dataPtr->laserCam->SetCosVertFOV(this->VertFOV());
      }
    }
    else
    {
      if (this->dataPtr->horzRayCount > 1)
      {
        this->dataPtr->laserCam->SetCosHorzFOV(this->HorzFOV());
        this->dataPtr->laserCam->SetCosVertFOV(
          2 * atan(tan(this->VertFOV()/2) / cos(this->HorzFOV()/2)));
        this->dataPtr->laserCam->SetRayCountRatio(
          tan(this->HorzFOV()/2.0) / tan(this->CosVertFOV()/2.0));

        if ((this->dataPtr->horzRayCount / this->RayCountRatio()) >
            this->dataPtr->vertRayCount)
        {
          this->dataPtr->vertRayCount =
            this->dataPtr->horzRayCount / this->RayCountRatio();
        }
        else
        {
          this->dataPtr->horzRayCount = this->dataPtr->vertRayCount *
            this->RayCountRatio();
        }
      }
      else
      {
        this->dataPtr->laserCam->SetCosHorzFOV(this->HorzFOV());
        this->dataPtr->laserCam->SetCosVertFOV(this->VertFOV());
      }
    }

    // Initialize camera sdf for GpuLaser
    this->dataPtr->cameraElem.reset(new sdf::Element);
    sdf::initFile("camera.sdf", this->dataPtr->cameraElem);

    this->dataPtr->cameraElem->GetElement("horizontal_fov")->Set(
        this->CosHorzFOV());

    sdf::ElementPtr ptr = this->dataPtr->cameraElem->GetElement("image");
    ptr->GetElement("width")->Set(this->dataPtr->horzRayCount);
    ptr->GetElement("height")->Set(this->dataPtr->vertRayCount);
    ptr->GetElement("format")->Set("R8G8B8");

    ptr = this->dataPtr->cameraElem->GetElement("clip");
    ptr->GetElement("near")->Set(this->dataPtr->laserCam->NearClip());
    ptr->GetElement("far")->Set(this->dataPtr->laserCam->FarClip());

    // Load camera sdf for GpuLaser
    this->dataPtr->laserCam->Load(this->dataPtr->cameraElem);


    // initialize GpuLaser
    this->dataPtr->laserCam->Init();
    this->dataPtr->laserCam->SetRangeCount(
        this->dataPtr->horzRangeCount, this->dataPtr->vertRangeCount);
    this->dataPtr->laserCam->SetClipDist(this->RangeMin(), this->RangeMax());
    this->dataPtr->laserCam->CreateLaserTexture(
        this->ScopedName() + "_RttTex_Laser");
    this->dataPtr->laserCam->CreateRenderTexture(
        this->ScopedName() + "_RttTex_Image");
    this->dataPtr->laserCam->SetWorldPose(this->pose);
    this->dataPtr->laserCam->AttachToVisual(this->ParentId(), true, 0, 0);

    this->dataPtr->laserMsg.mutable_scan()->set_frame(this->ParentName());
  }
  else
    gzerr << "No world name\n";

  // Disable clouds and moon on server side until fixed and also to improve
  // performance
  this->scene->SetSkyXMode(rendering::Scene::GZ_SKYX_ALL &
      ~rendering::Scene::GZ_SKYX_CLOUDS &
      ~rendering::Scene::GZ_SKYX_MOON);
*/
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
void GpuLidarSensor::Fini()
{
  /*
  if (this->scene)
    this->scene->RemoveCamera(this->dataPtr->laserCam->Name());
  this->scene.reset();

  this->dataPtr->laserCam.reset();
  */
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Load(sdf::ElementPtr _sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Validate that SDF has a sensor ray on it
  if (_sdf->GetName() == "sensor")
  {
    if (!_sdf->GetElement("ray"))
    {
      ignerr << "<sensor><camera> SDF element not found while attempting to "
        << "load a ignition::sensors::CameraSensor\n";
      return false;
    }
  }

  // Load sensor element
  if (!this->Sensor::Load(_sdf))
  {
    return false;
  }

  // Register publisher
  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::LaserScan>(
        this->Topic());
  if (!this->dataPtr->pub)
  {
    return false;
  }

  // Load ray atributes
  sdf::ElementPtr rayElem = this->SDF()->GetElement("ray");
  this->dataPtr->scanElem = rayElem->GetElement("scan");
  this->dataPtr->horzElem = this->dataPtr->scanElem->GetElement("horizontal");
  this->dataPtr->rangeElem = rayElem->GetElement("range");

  if (this->dataPtr->scanElem->HasElement("vertical"))
    this->dataPtr->vertElem = this->dataPtr->scanElem->GetElement("vertical");

  this->dataPtr->horzRayCount = this->RayCount();
  this->dataPtr->vertRayCount = this->VerticalRayCount();

  if (this->dataPtr->horzRayCount == 0 || this->dataPtr->vertRayCount == 0)
  {
    ignerr << "GpuLidarSensor: Image has 0 size!\n" ;
  }

  this->dataPtr->horzRangeCount = this->RangeCount();
  this->dataPtr->vertRangeCount = this->VerticalRangeCount();

  // Handle noise model settings.
  //if (rayElem->HasElement("noise"))
  //{
  //  this->noises[GPU_RAY_NOISE] =
  //      NoiseFactory::NewNoiseModel(rayElem->GetElement("noise"),
  //      this->Type());
  //}

//  this->dataPtr->parentEntity =
//    this->world->EntityByName(this->ParentName());
//  GZ_ASSERT(this->dataPtr->parentEntity != nullptr,
//      "Unable to get the parent entity.");

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::Update(const common::Time &_now)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (!this->dataPtr->initialized)
    return false;

//  this->dataPtr->laserCam->PostRender();

  // create message
  ignition::msgs::LaserScan msg;
  msg.mutable_header()->mutable_stamp()->set_sec(_now.sec);
  msg.mutable_header()->mutable_stamp()->set_nsec(_now.nsec);

  // Store the latest laser scans into laserMsg
  //msgs::Set(msg.mutable_world_pose(),
  //    this->pose + this->dataPtr->parentEntity->GetWorldPose().Ign());
  msg.set_angle_min(this->AngleMin().Radian());
  msg.set_angle_max(this->AngleMax().Radian());
  msg.set_angle_step(this->AngleResolution());
  msg.set_count(this->RangeCount());

  msg.set_vertical_angle_min(this->VerticalAngleMin().Radian());
  msg.set_vertical_angle_max(this->VerticalAngleMax().Radian());
  msg.set_vertical_angle_step(this->VerticalAngleResolution());
  msg.set_vertical_count(this->dataPtr->vertRangeCount);

  msg.set_range_min(this->dataPtr->rangeMin);
  msg.set_range_max(this->dataPtr->rangeMax);

  bool add = msg.ranges_size() == 0;
  const float *laserBuffer; // = this->dataPtr->laserCam->LaserData();

  for (unsigned int j = 0; j < this->dataPtr->vertRangeCount; ++j)
  {
    for (unsigned int i = 0; i < this->dataPtr->horzRangeCount; ++i)
    {
      int index = j * this->dataPtr->horzRangeCount + i;
      double range = 0; // laserBuffer[index*3];

      // Mask ranges outside of min/max to +/- inf, as per REP 117
      if (range >= this->dataPtr->rangeMax)
      {
        range = IGN_DBL_INF;
      }
      else if (range <= this->dataPtr->rangeMin)
      {
        range = -IGN_DBL_INF;
      }
      // TODO (jchoclin): add GPU_RAY_NOSE
      //else if (this->noises.find(GPU_RAY_NOISE) !=
      //         this->noises.end())
      //{
      //  range = this->noises[GPU_RAY_NOISE]->Apply(range);
      //  range = ignition::math::clamp(range,
      //      this->dataPtr->rangeMin, this->dataPtr->rangeMax);
      //}

      range = ignition::math::isnan(range) ? this->dataPtr->rangeMax : range;

      if (add)
      {
        msg.add_ranges(range);
//        msg.add_intensities(
//            this->dataPtr->laserCam->LaserData()[index * 3 + 1]);
      }
      else
      {
        msg.set_ranges(index, range);
//        msg.set_intensities(index,
//            this->dataPtr->laserCam->LaserData()[index * 3 + 1]);
      }
    }
  }

  // publish
  // Should I publish only if there are connections?
  //if (this->dataPtr->pub && this->dataPtr->pub->HasConnections())
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

  this->dataPtr->rendered = false;
  return true;
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
unsigned int GpuLidarSensor::CameraCount() const
{
//  return this->dataPtr->laserCam->CameraCount();
  return 0;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::IsHorizontal() const
{
//  return this->dataPtr->laserCam->IsHorizontal();
  return 0;
}

//////////////////////////////////////////////////
double GpuLidarSensor::GetHorzHalfAngle() const
{
//  return this->dataPtr->laserCam->HorzHalfAngle();
  return 0;
}

//////////////////////////////////////////////////
double GpuLidarSensor::GetVertHalfAngle() const
{
//  return this->dataPtr->laserCam->VertHalfAngle();
  return 0;
}

//////////////////////////////////////////////////
double GpuLidarSensor::HorzFOV() const
{
//  return this->dataPtr->laserCam->HorzFOV();
  return 0;
}

//////////////////////////////////////////////////
double GpuLidarSensor::CosHorzFOV() const
{
//  return this->dataPtr->laserCam->CosHorzFOV();
  return 0;
}

//////////////////////////////////////////////////
double GpuLidarSensor::VertFOV() const
{
//  return this->dataPtr->laserCam->VertFOV();
  return 0;
}

//////////////////////////////////////////////////
double GpuLidarSensor::CosVertFOV() const
{
//  return this->dataPtr->laserCam->CosVertFOV();
  return 0;
}

//////////////////////////////////////////////////
double GpuLidarSensor::RayCountRatio() const
{
//  return this->dataPtr->laserCam->RayCountRatio();
  return 0;
}

//////////////////////////////////////////////////
void GpuLidarSensor::Render()
{
//  if (!this->dataPtr->laserCam || !this->IsActive() || !this->NeedsUpdate())
    return;

// TODO: get running clock (?)
//  this->lastMeasurementTime = this->scene->SimTime();

//  this->dataPtr->laserCam->Render();
  this->dataPtr->rendered = true;
}

//////////////////////////////////////////////////
bool GpuLidarSensor::IsActive() const
{
//  return Sensor::IsActive() ||
//    (this->dataPtr->pub && this->dataPtr->pub->HasConnections());
  return true;
}

//////////////////////////////////////////////////
//rendering::GpuLaserPtr GpuLidarSensor::LaserCamera() const
//{
//  return this->dataPtr->laserCam;
//}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(
    ignition::sensors::GpuLidarSensor,
    ignition::sensors::Sensor)
