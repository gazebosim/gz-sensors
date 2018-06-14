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



#include "ignition/rendering/RenderTypes.hh"


namespace Ogre
{
  class Camera;
  class Material;
  class MovableObject;
  class RenderTarget;
  class SceneNode;
  class Texture;
  class Viewport;
}

namespace ignition
{
  namespace common
  {
    class Mesh;
  }

  namespace rendering
  {
    /// \internal
    /// \brief Private data for the GpuLaser class
    class GpuLaserPrivate
    {
      /// \brief Event triggered when new laser range data are available.
      /// \param[in] _frame New frame containing raw laser data.
      /// \param[in] _width Width of frame.
      /// \param[in] _height Height of frame.
      /// \param[in] _depth Depth of frame.
      /// \param[in] _format Format of frame.
      public: event::EventT<void(const float *_frame, unsigned int _width,
                   unsigned int _height, unsigned int _depth,
                   const std::string &_format)> newLaserFrame;

      /// \brief Raw buffer of laser data.
      public: float *laserBuffer;

      /// \brief Outgoing laser data, used by newLaserFrame event.
      public: float *laserScan;

      /// \brief Pointer to Ogre material for the first rendering pass.
      public: Ogre::Material *matFirstPass;

      /// \brief Pointer to Ogre material for the sencod rendering pass.
      public: Ogre::Material *matSecondPass;

      /// \brief An array of first pass textures.
      public: Ogre::Texture *firstPassTextures[3];

      /// \brief Second pass texture.
      public: Ogre::Texture *secondPassTexture;

      /// \brief First pass render targets.
      public: Ogre::RenderTarget *firstPassTargets[3];

      /// \brief Second pass render target.
      public: Ogre::RenderTarget *secondPassTarget;

      /// \brief First pass viewports.
      public: Ogre::Viewport *firstPassViewports[3];

      /// \brief Second pass viewport
      public: Ogre::Viewport *secondPassViewport;

      /// \brief Number of first pass textures.
      public: unsigned int textureCount;

      /// \brief A list of camera angles for first pass rendering.
      public: double cameraYaws[4];

      /// \brief Temporary pointer to the current render target.
      public: Ogre::RenderTarget *currentTarget;

      /// \brief Temporary pointer to the current material.
      public: Ogre::Material *currentMat;

      /// \brief Ogre orthorgraphic camera used in the second pass for
      /// undistortion.
      public: Ogre::Camera *orthoCam;

      /// \brief Ogre scenenode where the orthorgraphic camera is attached to.
      public: Ogre::SceneNode *pitchNodeOrtho;

      /// \brief Ogre mesh used to create a canvas for undistorting range values
      /// in the second rendering pass.
      public: common::Mesh *undistMesh;

      /// \brief Ogre movable object created from the canvas mesh.
      public: Ogre::MovableObject *object;

      /// \brief Pointer to visual that holds the canvas.
      public: VisualPtr visual;

      /// \brief Image width.
      public: unsigned int w2nd;

      /// \brief Image height.
      public: unsigned int h2nd;

      /// \brief Time taken to complete the two rendering passes.
      public: double lastRenderDuration;

      /// \brief List of texture unit indices used during the second
      /// rendering pass.
      public: std::vector<int> texIdx;

      /// Number of second pass texture units created.
      public: static int texCount;
    };
  }
}

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
    this->Init();
    this->SetRangeCount(
        this->dataPtr->horzRangeCount, this->dataPtr->vertRangeCount);
    this->SetClipDist(this->RangeMin(), this->RangeMax());
    this->CreateLaserTexture(this->ScopedName() + "_RttTex_Laser");
    this->CreateRenderTexture(this->ScopedName() + "_RttTex_Image");
    this->SetWorldPose(this->pose);
    this->AttachToVisual(this->ParentId(), true, 0, 0);

    this->dataPtr->laserMsg.mutable_scan()->set_frame(this->ParentName());
  }
  else
    gzerr << "No world name\n";

  // Disable clouds and moon on server side until fixed and also to improve
  // performance
  this->scene->SetSkyXMode(rendering::Scene::GZ_SKYX_ALL &
      ~rendering::Scene::GZ_SKYX_CLOUDS &
      ~rendering::Scene::GZ_SKYX_MOON);

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
bool GpuLidarSensor::Update(const common::Time &_now)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (!this->dataPtr->initialized)
    return false;

// PostRender();
  this->PublishLaserScan(_now);

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
void GpuLaser::CreateLaserTexture(const std::string &_textureName)
{
  this->camera->yaw(Ogre::Radian(this->horzHalfAngle));
  this->camera->pitch(Ogre::Radian(this->vertHalfAngle));

  this->CreateOrthoCam();

  this->dataPtr->textureCount = this->cameraCount;

  if (this->dataPtr->textureCount == 2)
  {
    this->dataPtr->cameraYaws[0] = -this->hfov/2;
    this->dataPtr->cameraYaws[1] = +this->hfov;
    this->dataPtr->cameraYaws[2] = 0;
    this->dataPtr->cameraYaws[3] = -this->hfov/2;
  }
  else
  {
    this->dataPtr->cameraYaws[0] = -this->hfov;
    this->dataPtr->cameraYaws[1] = +this->hfov;
    this->dataPtr->cameraYaws[2] = +this->hfov;
    this->dataPtr->cameraYaws[3] = -this->hfov;
  }

  for (unsigned int i = 0; i < this->dataPtr->textureCount; ++i)
  {
    std::stringstream texName;
    texName << _textureName << "first_pass_" << i;
    this->dataPtr->firstPassTextures[i] =
      Ogre::TextureManager::getSingleton().createManual(
      texName.str(), "General", Ogre::TEX_TYPE_2D,
      this->ImageWidth(), this->ImageHeight(), 0,
      Ogre::PF_FLOAT32_RGB, Ogre::TU_RENDERTARGET).getPointer();

    this->Set1stPassTarget(
        this->dataPtr->firstPassTextures[i]->getBuffer()->getRenderTarget(), i);

    this->dataPtr->firstPassTargets[i]->setAutoUpdated(false);
  }

  this->dataPtr->matFirstPass = (Ogre::Material*)(
  Ogre::MaterialManager::getSingleton().getByName("Gazebo/LaserScan1st").get());

  this->dataPtr->matFirstPass->load();

  this->dataPtr->secondPassTexture =
      Ogre::TextureManager::getSingleton().createManual(
      _textureName + "second_pass",
      "General",
      Ogre::TEX_TYPE_2D,
      this->ImageWidth(), this->ImageHeight(), 0,
      Ogre::PF_FLOAT32_RGB,
      Ogre::TU_RENDERTARGET).getPointer();

  this->Set2ndPassTarget(
      this->dataPtr->secondPassTexture->getBuffer()->getRenderTarget());

  this->dataPtr->secondPassTarget->setAutoUpdated(false);

  this->dataPtr->matSecondPass = (Ogre::Material*)(
  Ogre::MaterialManager::getSingleton().getByName("Gazebo/LaserScan2nd").get());

  this->dataPtr->matSecondPass->load();

  Ogre::TextureUnitState *texUnit;
  for (unsigned int i = 0; i < this->dataPtr->textureCount; ++i)
  {
    unsigned int texIndex = this->dataPtr->texCount++;
    Ogre::Technique *technique = this->dataPtr->matSecondPass->getTechnique(0);
    GZ_ASSERT(technique, "GpuLaser material script error: technique not found");

    Ogre::Pass *pass = technique->getPass(0);
    GZ_ASSERT(pass, "GpuLaser material script error: pass not found");

    if (!pass->getTextureUnitState(
        this->dataPtr->firstPassTextures[i]->getName()))
    {
      texUnit = pass->createTextureUnitState(
            this->dataPtr->firstPassTextures[i]->getName(), texIndex);

      this->dataPtr->texIdx.push_back(texIndex);

      texUnit->setTextureFiltering(Ogre::TFO_NONE);
      texUnit->setTextureAddressingMode(Ogre::TextureUnitState::TAM_MIRROR);
    }
  }

  this->CreateCanvas();
}

//////////////////////////////////////////////////
void GpuLaser::PostRender()
{
//  common::Timer postRenderT, blitT;
//  postRenderT.Start();
//  double blitDur = 0.0;
//  double postRenderDur = 0.0;

  for (unsigned int i = 0; i < this->dataPtr->textureCount; ++i)
  {
    this->dataPtr->firstPassTargets[i]->swapBuffers();
  }

  this->dataPtr->secondPassTarget->swapBuffers();

  if (this->newData && this->captureData)
  {
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer;

    unsigned int width = this->dataPtr->secondPassViewport->getActualWidth();
    unsigned int height = this->dataPtr->secondPassViewport->getActualHeight();

    // Get access to the buffer and make an image and write it to file
    pixelBuffer = this->dataPtr->secondPassTexture->getBuffer();

    size_t size = Ogre::PixelUtil::getMemorySize(
                    width, height, 1, Ogre::PF_FLOAT32_RGB);

    // Blit the depth buffer if needed
    if (!this->dataPtr->laserBuffer)
      this->dataPtr->laserBuffer = new float[size];

    memset(this->dataPtr->laserBuffer, 255, size);

    Ogre::PixelBox dstBox(width, height,
        1, Ogre::PF_FLOAT32_RGB, this->dataPtr->laserBuffer);

//    blitT.Start();
    pixelBuffer->blitToMemory(dstBox);
//    blitDur = blitT.GetElapsed().Double();

    if (!this->dataPtr->laserScan)
    {
      int len = this->dataPtr->w2nd * this->dataPtr->h2nd * 3;
      this->dataPtr->laserScan = new float[len];
    }

    memcpy(this->dataPtr->laserScan, this->dataPtr->laserBuffer,
           this->dataPtr->w2nd * this->dataPtr->h2nd * 3 *
           sizeof(this->dataPtr->laserScan[0]));

    this->dataPtr->newLaserFrame(this->dataPtr->laserScan, this->dataPtr->w2nd,
        this->dataPtr->h2nd, 3, "BLABLA");
  }

  this->newData = false;
//  postRenderDur = postRenderT.GetElapsed().Double();

/*  std::cerr << " Render: " << this->dataPtr->lastRenderDuration * 1000
              << " BLIT: " << blitDur * 1000
              << " postRender: " << postRenderDur * 1000
              << " TOTAL: "
              << (this->dataPtr->lastRenderDuration + postRenderDur) * 1000
              << " Total - BLIT: "
              << (this->dataPtr->lastRenderDuration + postRenderDur - blitDur)
                  * 1000 << "\n";   */
}

/////////////////////////////////////////////////
void GpuLaser::UpdateRenderTarget(Ogre::RenderTarget *_target,
                   Ogre::Material *_material, Ogre::Camera *_cam,
                   const bool _updateTex)
{
  Ogre::RenderSystem *renderSys;
  Ogre::Viewport *vp = NULL;
  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();
  Ogre::Pass *pass;

  renderSys = this->scene->OgreSceneManager()->getDestinationRenderSystem();
  // Get pointer to the material pass
  pass = _material->getBestTechnique()->getPass(0);

  // Render the depth texture
  // OgreSceneManager::_render function automatically sets farClip to 0.
  // Which normally equates to infinite distance. We don't want this. So
  // we have to set the distance every time.
  _cam->setFarClipDistance(this->FarClip());

  Ogre::AutoParamDataSource autoParamDataSource;

  vp = _target->getViewport(0);

  // Need this line to render the ground plane. No idea why it's necessary.
  renderSys->_setViewport(vp);
  sceneMgr->_setPass(pass, true, false);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(_target);
  autoParamDataSource.setCurrentSceneManager(sceneMgr);
  autoParamDataSource.setCurrentCamera(_cam, true);

  renderSys->setLightingEnabled(false);
  renderSys->_setFog(Ogre::FOG_NONE);

  pass->_updateAutoParams(&autoParamDataSource, 1);

  if (_updateTex)
  {
    pass->getFragmentProgramParameters()->setNamedConstant("tex1",
      this->dataPtr->texIdx[0]);
    if (this->dataPtr->texIdx.size() > 1)
    {
      pass->getFragmentProgramParameters()->setNamedConstant("tex2",
        this->dataPtr->texIdx[1]);
      if (this->dataPtr->texIdx.size() > 2)
        pass->getFragmentProgramParameters()->setNamedConstant("tex3",
          this->dataPtr->texIdx[2]);
    }
  }

  // NOTE: We MUST bind parameters AFTER updating the autos
  if (pass->hasVertexProgram())
  {
    renderSys->bindGpuProgram(
        pass->getVertexProgram()->_getBindingDelegate());

    renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
      pass->getVertexProgramParameters(), 1);
  }

  if (pass->hasFragmentProgram())
  {
    renderSys->bindGpuProgram(
    pass->getFragmentProgram()->_getBindingDelegate());

      renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
      pass->getFragmentProgramParameters(), 1);
  }
}

/////////////////////////////////////////////////
void GpuLaser::notifyRenderSingleObject(Ogre::Renderable *_rend,
      const Ogre::Pass* /*pass*/, const Ogre::AutoParamDataSource* /*source*/,
      const Ogre::LightList* /*lights*/, bool /*supp*/)
{
  Ogre::Vector4 retro = Ogre::Vector4(0, 0, 0, 0);
  try
  {
    retro = _rend->getCustomParameter(1);
  }
  catch(Ogre::ItemIdentityException& e)
  {
    _rend->setCustomParameter(1, Ogre::Vector4(0, 0, 0, 0));
  }

  Ogre::Pass *pass = this->dataPtr->currentMat->getBestTechnique()->getPass(0);
  Ogre::RenderSystem *renderSys =
                  this->scene->OgreSceneManager()->getDestinationRenderSystem();

  Ogre::AutoParamDataSource autoParamDataSource;

  Ogre::Viewport *vp = this->dataPtr->currentTarget->getViewport(0);

  renderSys->_setViewport(vp);
  autoParamDataSource.setCurrentRenderable(_rend);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(this->dataPtr->currentTarget);
  autoParamDataSource.setCurrentSceneManager(this->scene->OgreSceneManager());
  autoParamDataSource.setCurrentCamera(this->camera, true);

  pass->_updateAutoParams(&autoParamDataSource,
      Ogre::GPV_GLOBAL || Ogre::GPV_PER_OBJECT);
  pass->getFragmentProgramParameters()->setNamedConstant("retro", retro[0]);
  renderSys->bindGpuProgram(
      pass->getVertexProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
      pass->getVertexProgramParameters(),
      Ogre::GPV_GLOBAL || Ogre::GPV_PER_OBJECT);

  renderSys->bindGpuProgram(
      pass->getFragmentProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
      pass->getFragmentProgramParameters(),
      Ogre::GPV_GLOBAL || Ogre::GPV_PER_OBJECT);
}

//////////////////////////////////////////////////
void GpuLaser::RenderImpl()
{
  common::Timer firstPassTimer, secondPassTimer;

  firstPassTimer.Start();

  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();

  sceneMgr->_suppressRenderStateChanges(true);
  sceneMgr->addRenderObjectListener(this);

  for (unsigned int i = 0; i < this->dataPtr->textureCount; ++i)
  {
    if (this->dataPtr->textureCount > 1)
    {
      // Cannot call Camera::RotateYaw because it rotates in world frame,
      // but we need rotation in camera local frame
      this->sceneNode->roll(Ogre::Radian(this->dataPtr->cameraYaws[i]));
    }

    this->dataPtr->currentMat = this->dataPtr->matFirstPass;
    this->dataPtr->currentTarget = this->dataPtr->firstPassTargets[i];

    this->UpdateRenderTarget(this->dataPtr->firstPassTargets[i],
                  this->dataPtr->matFirstPass, this->camera);
    this->dataPtr->firstPassTargets[i]->update(false);
  }

  if (this->dataPtr->textureCount > 1)
      this->sceneNode->roll(Ogre::Radian(this->dataPtr->cameraYaws[3]));

  sceneMgr->removeRenderObjectListener(this);

  double firstPassDur = firstPassTimer.GetElapsed().Double();
  secondPassTimer.Start();

  this->dataPtr->visual->SetVisible(true);

  this->UpdateRenderTarget(this->dataPtr->secondPassTarget,
                this->dataPtr->matSecondPass, this->dataPtr->orthoCam, true);
  this->dataPtr->secondPassTarget->update(false);

  this->dataPtr->visual->SetVisible(false);

  sceneMgr->_suppressRenderStateChanges(false);

  double secondPassDur = secondPassTimer.GetElapsed().Double();
  this->dataPtr->lastRenderDuration = firstPassDur + secondPassDur;
}

//////////////////////////////////////////////////
const float* GpuLaser::LaserData() const
{
  return this->dataPtr->laserBuffer;
}

//////////////////////////////////////////////////
GpuLaser::DataIter GpuLaser::LaserDataBegin() const
{
  const unsigned int index = 0;
  // Data stuffed into three floats (RGB)
  const unsigned int skip = 3;
  // range data in R channel
  const unsigned int rangeOffset = 0;
  // intensity data in G channel
  const unsigned int intenOffset = 1;
  return DataIter(index, this->dataPtr->laserBuffer, skip, rangeOffset,
      intenOffset, this->ImageWidth());
}

//////////////////////////////////////////////////
GpuLaser::DataIter GpuLaser::LaserDataEnd() const
{
  const unsigned int index = this->ImageHeight() * this->ImageWidth();
  // Data stuffed into three floats (RGB)
  const unsigned int skip = 3;
  // range data in R channel
  const unsigned int rangeOffset = 0;
  // intensity data in G channel
  const unsigned int intenOffset = 1;
  return DataIter(index, this->dataPtr->laserBuffer, skip, rangeOffset,
      intenOffset, this->ImageWidth());
}

/////////////////////////////////////////////////
void GpuLaser::CreateOrthoCam()
{
  this->dataPtr->pitchNodeOrtho =
    this->GetScene()->WorldVisual()->GetSceneNode()->createChildSceneNode();

  this->dataPtr->orthoCam = this->scene->OgreSceneManager()->createCamera(
        this->dataPtr->pitchNodeOrtho->getName() + "_ortho_cam");

  // Use X/Y as horizon, Z up
  this->dataPtr->orthoCam->pitch(Ogre::Degree(90));

  // Don't yaw along variable axis, causes leaning
  this->dataPtr->orthoCam->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);

  this->dataPtr->orthoCam->setDirection(1, 0, 0);

  this->dataPtr->pitchNodeOrtho->attachObject(this->dataPtr->orthoCam);
  this->dataPtr->orthoCam->setAutoAspectRatio(true);

  if (this->dataPtr->orthoCam)
  {
    this->dataPtr->orthoCam->setNearClipDistance(0.01);
    this->dataPtr->orthoCam->setFarClipDistance(0.02);
    this->dataPtr->orthoCam->setRenderingDistance(0.02);

    this->dataPtr->orthoCam->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  }
}

/////////////////////////////////////////////////
Ogre::Matrix4 GpuLaser::BuildScaledOrthoMatrix(const float _left,
    const float _right, const float _bottom, const float _top,
    const float _near, const float _far)
{
  float invw = 1 / (_right - _left);
  float invh = 1 / (_top - _bottom);
  float invd = 1 / (_far - _near);

  Ogre::Matrix4 proj = Ogre::Matrix4::ZERO;
  proj[0][0] = 2 * invw;
  proj[0][3] = -(_right + _left) * invw;
  proj[1][1] = 2 * invh;
  proj[1][3] = -(_top + _bottom) * invh;
  proj[2][2] = -2 * invd;
  proj[2][3] = -(_far + _near) * invd;
  proj[3][3] = 1;

  return proj;
}

//////////////////////////////////////////////////
void GpuLaser::Set1stPassTarget(Ogre::RenderTarget *_target,
                                const unsigned int _index)
{
  this->dataPtr->firstPassTargets[_index] = _target;

  if (this->dataPtr->firstPassTargets[_index])
  {
    // Setup the viewport to use the texture
    this->dataPtr->firstPassViewports[_index] =
      this->dataPtr->firstPassTargets[_index]->addViewport(this->camera);
    this->dataPtr->firstPassViewports[_index]->setClearEveryFrame(true);
    this->dataPtr->firstPassViewports[_index]->setOverlaysEnabled(false);
    this->dataPtr->firstPassViewports[_index]->setShadowsEnabled(false);
    this->dataPtr->firstPassViewports[_index]->setSkiesEnabled(false);
    this->dataPtr->firstPassViewports[_index]->setBackgroundColour(
        Ogre::ColourValue(this->farClip, 0.0, 1.0));
    this->dataPtr->firstPassViewports[_index]->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
  }
  if (_index == 0)
  {
    this->camera->setAspectRatio(this->rayCountRatio);
    this->camera->setFOVy(Ogre::Radian(this->vfov));
  }
}

//////////////////////////////////////////////////
void GpuLaser::Set2ndPassTarget(Ogre::RenderTarget *_target)
{
  this->dataPtr->secondPassTarget = _target;

  if (this->dataPtr->secondPassTarget)
  {
    // Setup the viewport to use the texture
    this->dataPtr->secondPassViewport =
        this->dataPtr->secondPassTarget->addViewport(this->dataPtr->orthoCam);
    this->dataPtr->secondPassViewport->setClearEveryFrame(true);
    this->dataPtr->secondPassViewport->setOverlaysEnabled(false);
    this->dataPtr->secondPassViewport->setShadowsEnabled(false);
    this->dataPtr->secondPassViewport->setSkiesEnabled(false);
    this->dataPtr->secondPassViewport->setBackgroundColour(
        Ogre::ColourValue(0.0, 1.0, 0.0));
    this->dataPtr->secondPassViewport->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
  }
  Ogre::Matrix4 p = this->BuildScaledOrthoMatrix(
      0, static_cast<float>(this->ImageWidth() / 10.0),
      0, static_cast<float>(this->ImageHeight() / 10.0),
      0.01, 0.02);

  this->dataPtr->orthoCam->setCustomProjectionMatrix(true, p);
}

/////////////////////////////////////////////////
void GpuLaser::CreateMesh()
{
  std::string meshName = this->Name() + "_undistortion_mesh";

  common::Mesh *mesh = new common::Mesh();
  mesh->SetName(meshName);

  common::SubMesh *submesh = new common::SubMesh();

  double dx, dy;
  submesh->SetPrimitiveType(common::SubMesh::POINTS);

  double viewHeight = this->ImageHeight()/10.0;

  if (this->dataPtr->h2nd == 1)
    dy = 0;
  else
    dy = 0.1;

  dx = 0.1;

  double startX = dx;
  double startY = viewHeight;

  double phi = this->vfov / 2;

  double vAngMin = -phi;

  if (this->ImageHeight() == 1)
    phi = 0;

  unsigned int ptsOnLine = 0;
  for (unsigned int j = 0; j < this->dataPtr->h2nd; ++j)
  {
    double gamma = 0;
    if (this->dataPtr->h2nd != 1)
      gamma = ((2 * phi / (this->dataPtr->h2nd - 1)) * j) + vAngMin;
    for (unsigned int i = 0; i < this->dataPtr->w2nd; ++i)
    {
      double thfov = this->dataPtr->textureCount * this->hfov;
      double theta = this->hfov / 2;
      double delta = ((thfov / (this->dataPtr->w2nd - 1)) * i);

      unsigned int texture = delta / (theta*2);

      if (texture > this->dataPtr->textureCount-1)
      {
        texture -= 1;
        delta -= (thfov / (this->dataPtr->w2nd - 1));
      }

      delta = delta - (texture * (theta*2));

      delta = delta - theta;

      startX -= dx;
      if (ptsOnLine == this->ImageWidth())
      {
        ptsOnLine = 0;
        startX = 0;
        startY -= dy;
      }
      ptsOnLine++;
      submesh->AddVertex(texture/1000.0, startX, startY);

      double u, v;
      if (this->isHorizontal)
      {
        u = -(cos(phi) * tan(delta))/(2 * tan(theta) * cos(gamma)) + 0.5;
        v = ignition::math::equal(phi, 0.0) ?
            -tan(gamma)/(2 * tan(phi)) + 0.5 : 0.5;
      }
      else
      {
        v = -(cos(theta) * tan(gamma))/(2 * tan(phi) * cos(delta)) + 0.5;
        u = ignition::math::equal(theta, 0.0) ?
            -tan(delta)/(2 * tan(theta)) + 0.5 : 0.5;
      }
      submesh->AddTexCoord(u, v);
    }
  }

  for (unsigned int j = 0; j < (this->dataPtr->h2nd ); ++j)
    for (unsigned int i = 0; i < (this->dataPtr->w2nd ); ++i)
      submesh->AddIndex(this->dataPtr->w2nd * j + i);

  mesh->AddSubMesh(submesh);

  this->dataPtr->undistMesh = mesh;

  common::MeshManager::Instance()->AddMesh(this->dataPtr->undistMesh);
}

/////////////////////////////////////////////////
void GpuLaser::CreateCanvas()
{
  this->CreateMesh();

  Ogre::Node *parent = this->dataPtr->visual->GetSceneNode()->getParent();
  parent->removeChild(this->dataPtr->visual->GetSceneNode());

  this->dataPtr->pitchNodeOrtho->addChild(
      this->dataPtr->visual->GetSceneNode());

  this->dataPtr->visual->InsertMesh(this->dataPtr->undistMesh);

  std::ostringstream stream;
  std::string meshName = this->dataPtr->undistMesh->GetName();
  stream << this->dataPtr->visual->GetSceneNode()->getName()
      << "_ENTITY_" << meshName;

  this->dataPtr->object = (Ogre::MovableObject*)
      (this->dataPtr->visual->GetSceneNode()->getCreator()->createEntity(
      stream.str(), meshName));

  this->dataPtr->visual->AttachObject(this->dataPtr->object);
  this->dataPtr->object->setVisibilityFlags(GZ_VISIBILITY_ALL
      & ~GZ_VISIBILITY_SELECTABLE);

  ignition::math::Pose3d pose;
  pose.Pos().Set(0.01, 0, 0);
  pose.Rot().Euler(ignition::math::Vector3d(0, 0, 0));

  this->dataPtr->visual->SetPose(pose);

  this->dataPtr->visual->SetMaterial("Gazebo/Green");
  this->dataPtr->visual->SetAmbient(ignition::math::Color(0, 1, 0, 1));
  this->dataPtr->visual->SetVisible(true);
  this->scene->AddVisual(this->dataPtr->visual);
}

//////////////////////////////////////////////////
void GpuLaser::SetRangeCount(const unsigned int _w, const unsigned int _h)
{
  this->dataPtr->w2nd = _w;
  this->dataPtr->h2nd = _h;
}

/////////////////////////////////////////////////
void GpuLaser::SetHorzHalfAngle(const double _angle)
{
  this->horzHalfAngle = _angle;
}

//////////////////////////////////////////////////
void GpuLaser::SetVertHalfAngle(const double _angle)
{
  this->vertHalfAngle = _angle;
}

//////////////////////////////////////////////////
double GpuLaser::HorzHalfAngle() const
{
  return this->horzHalfAngle;
}

//////////////////////////////////////////////////
double GpuLaser::VertHalfAngle() const
{
  return this->vertHalfAngle;
}

//////////////////////////////////////////////////
void GpuLaser::SetIsHorizontal(const bool _horizontal)
{
  this->isHorizontal = _horizontal;
}

//////////////////////////////////////////////////
bool GpuLaser::IsHorizontal() const
{
  return this->isHorizontal;
}

//////////////////////////////////////////////////
double GpuLaser::HorzFOV() const
{
  return this->hfov;
}

//////////////////////////////////////////////////
double GpuLaser::VertFOV() const
{
  return this->vfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetHorzFOV(const double _hfov)
{
  this->hfov = _hfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetVertFOV(const double _vfov)
{
  this->vfov = _vfov;
}

//////////////////////////////////////////////////
double GpuLaser::CosHorzFOV() const
{
  return this->chfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetCosHorzFOV(const double _chfov)
{
  this->chfov = _chfov;
}

//////////////////////////////////////////////////
double GpuLaser::CosVertFOV() const
{
  return this->cvfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetCosVertFOV(const double _cvfov)
{
  this->cvfov = _cvfov;
}

//////////////////////////////////////////////////
double GpuLaser::NearClip() const
{
  return this->nearClip;
}

//////////////////////////////////////////////////
double GpuLaser::FarClip() const
{
  return this->farClip;
}

//////////////////////////////////////////////////
void GpuLaser::SetNearClip(const double _near)
{
  this->nearClip = _near;
}

//////////////////////////////////////////////////
void GpuLaser::SetFarClip(const double _far)
{
  this->farClip = _far;
}

//////////////////////////////////////////////////
unsigned int GpuLaser::CameraCount() const
{
  return this->cameraCount;
}

//////////////////////////////////////////////////
void GpuLaser::SetCameraCount(const unsigned int _cameraCount)
{
  this->cameraCount = _cameraCount;
}

//////////////////////////////////////////////////
double GpuLaser::RayCountRatio() const
{
  return this->rayCountRatio;
}

//////////////////////////////////////////////////
void GpuLaser::SetRayCountRatio(const double _rayCountRatio)
{
  this->rayCountRatio = _rayCountRatio;
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(
    ignition::sensors::GpuLidarSensor,
    ignition::sensors::Sensor)
