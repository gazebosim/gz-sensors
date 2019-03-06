# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.


## Ignition Sensors 1.X to 2.X

### Modifications

### Deprecations

1. **include/sensors/Events.hh**
    + ***Deprecation:** public: static ignition::common::ConnectionPtr ConnectSceneChangeCallback(std::function<void(const ignition::rendering::ScenePtr &)>)
    + ***Replacement:** RenderingEvents::ConnectSceneChangeCallback

1. **include/sensors/Manager.hh**
    + ***Deprecation:** public: bool Init(ignition::rendering::ScenePtr);
    + ***Replacement:**  RenderingSensor::SetScene
    + ***Deprecation:** public: void SetRenderingScene(ignition::rendering::ScenePtr
    + ***Replacement:**  RenderingSensor::SetScene

    + ***Deprecation:** public: ignition::rendering::ScenePtr RenderingScene() const
    + ***Replacement:** RenderingSensor::Scene()

1. **include/sensors/Noise.hh**
    + ***Deprecation:** public: virtual void SetCamera(rendering::CameraPtr)
    + ***Replacement:**  TODO (to be implemented in ImageGaussianNoiseModel)


