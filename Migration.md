# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.


## Ignition Sensors 2.X to 3.X

### Modifications

1. Depend on ignition-rendering3, ignition-msgs5 and ignition-transport8.

1. ImageGaussiaNoiseModel class is spearated out from GaussianNoiseModel source
and header files. Similarly, noise models for rendering sensors need be created
using the new ImageNoiseFactory class instead of NoiseFactory by including
ImageNoise.hh.

## Ignition Sensors 1.X to 2.X

### Additions

1. Rendering sensors moved to `rendering` component.

1. New sensors: altimeter, IMU, magnetometer, RGBD camera

### Modifications

1. Depend on ignition-msgs4, ignition-rendering2, ignition-transport7.

### Deprecations

1. **include/sensors/Noise.hh**
   + ***Deprecation*** virtual void Load(sdf::ElementPtr _sdf)
   + ***Replacement*** virtual void Load(const sdf::Noise &_sdf)

1. **include/sensors/Events.hh**
    + ***Deprecation:*** public: static ignition::common::ConnectionPtr ConnectSceneChangeCallback(std::function<void(const ignition::rendering::ScenePtr &)>)
    + ***Replacement:*** RenderingEvents::ConnectSceneChangeCallback

1. **include/sensors/Manager.hh**
    + ***Deprecation:*** public: bool Init(ignition::rendering::ScenePtr);
    + ***Replacement:***  RenderingSensor::SetScene
    + ***Deprecation:*** public: void SetRenderingScene(ignition::rendering::ScenePtr
    + ***Replacement:***  RenderingSensor::SetScene

    + ***Deprecation:*** public: ignition::rendering::ScenePtr RenderingScene() const
    + ***Replacement:*** RenderingSensor::Scene()

1. **include/sensors/Noise.hh**
    + ***Deprecation:*** public: virtual void SetCamera(rendering::CameraPtr)
    + ***Replacement:***  TODO (to be implemented in ImageGaussianNoiseModel)


