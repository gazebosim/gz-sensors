# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.

## Gazebo Sensors 6.X to 7.X

1. The `ignition` namespace is deprecated and will be removed in future versions.  Use `gz` instead.

1. Header files under `ignition/...` are deprecated and will be removed in future versions.
     Use `gz/...` instead.

1. The `ignition:type` SDF attribute is deprecated and will be removed.
     Please use `gz:type` instead.

1. The shared libraries have `gz` where there used to be `ignition`.
  1. Using the un-migrated version is still possible due to tick-tocks, but will be removed in future versions.

1. **CameraSensor**: the default anti-aliasing value has changed from `2` to `4`.

* The project name has been changed to use the `gz-` prefix, you **must** use the `gz` prefix!
  * This also means that any generated code that use the project name (e.g. CMake variables, in-source macros) would have to be migrated.
  * Some non-exhaustive examples of this include:
    * `GZ_<PROJECT>_<VISIBLE/HIDDEN>`
    * CMake `-config` files
    * Paths that depend on the project name

## Gazebo Sensors 6.0.1 to 6.1.0

### Modifications

1. Point Cloud Density flag (**is_dense** from GpuLidarSensor) is set to **false**
   if invalid points (NaN or +/-INF) are found, and **true** otherwise.


## Gazebo Sensors 5.X to 6.X

1. Sensors aren't loaded as plugins anymore. Instead, downstream libraries must
   link to the library of the sensor they're interested in, and instantiate
   new sensors knowing their type. For example:

    * `auto camera = std::make_unique<gz::sensors::CameraSensor>();`
    * `auto camera = sensorFactory.CreateSensor<gz::sensors::CameraSensor>(_sdf);`
    * `auto camera = manager.CreateSensor<gz::sensors::CameraSensor>(_sdf);`

1. **include/sensors/SensorFactory.hh**
   + ***Deprecation*** SensorPlugin
   + ***Replacement*** None; see above.
   + ***Deprecation*** SensorTypePlugin
   + ***Replacement*** None; see above.
   + ***Deprecation*** std::unique_ptr<Sensor> CreateSensor(sdf::ElementPtr);
   + ***Replacement*** template<typename SensorType> std::unique_ptr<SensorType> CreateSensor(sdf::ElementPtr);
   + ***Deprecation*** std::unique_ptr<Sensor> CreateSensor(const sdf::Sensor &);
   + ***Replacement*** template<typename SensorType> std::unique_ptr<SensorType> CreateSensor(const sdf::Sensor &);
   + ***Deprecation*** void AddPluginPaths(const std::string &)
   + ***Replacement*** None; see above.
   + ***Deprecation*** IGN_SENSORS_REGISTER_SENSOR
   + ***Replacement*** None; see above.

1. **include/sensors/Manager.hh**
   + ***Deprecation*** SensorId CreateSensor(sdf::ElementPtr);
   + ***Replacement*** template<typename SensorType, typename SdfType> SensorType> \*CreateSensor(SdfType);
   + ***Deprecation*** SensorId CreateSensor(const sdf::Sensor &);
   + ***Replacement*** template<typename SensorType, typename SdfType> SensorType \*CreateSensor(SdfType);
   + ***Deprecation*** void AddPluginPaths(const std::string &)
   + ***Replacement*** None; see above.

## Gazebo Sensors 3.X to 4.X

1. **include/sensors/Sensor.hh**
   + ***Deprecation*** virtual bool Update(const gz::common::Time &)
   + ***Replacement*** virtual bool Update(const std::chrono::steady_clock::duration &)
   + ***Deprecation*** virtual bool Update(const gz::common::Time &, const bool)
   + ***Replacement*** virtual bool Update(const std::chrono::steady_clock::duration &, const bool)
   + ***Deprecation*** gz::common::Time NextUpdateTime() const
   + ***Replacement*** std::chrono::steady_clock::duration NextDataUpdateTime() const

1. **include/sensors/Manager.hh**
   + ***Deprecation*** void RunOnce(const gz::common::Time &, bool);
   + ***Replacement*** void RunOnce(const std::chrono::steady_clock::duration &, bool)

1. **include/sensors/Lidar.hh**
   + ***Deprecation*** virtual bool PublishLidarScan(const gz::common::Time &)
   + ***Replacement*** virtual bool PublishLidarScan(const std::chrono::steady_clock::duration &)

## Gazebo Sensors 2.X to 3.X

### Additions

1. The core ignition-sensors library no longer depends on ign-rendering. All
rendering sensors and noise models are now part of the `rendering` component

1. New sensors: thermal camera

### Modifications

1. Depend on ignition-rendering3, ignition-msgs5 and ignition-transport8.

1. ImageGaussiaNoiseModel class is spearated out from GaussianNoiseModel source
and header files. Similarly, noise models for rendering sensors need be created
using the new ImageNoiseFactory class instead of NoiseFactory by including
ImageNoise.hh.

## Gazebo Sensors 1.X to 2.X

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
    + ***Deprecation:*** public: static gz::common::ConnectionPtr ConnectSceneChangeCallback(std::function<void(const gz::rendering::ScenePtr &)>)
    + ***Replacement:*** RenderingEvents::ConnectSceneChangeCallback

1. **include/sensors/Manager.hh**
    + ***Deprecation:*** public: bool Init(gz::rendering::ScenePtr);
    + ***Replacement:***  RenderingSensor::SetScene
    + ***Deprecation:*** public: void SetRenderingScene(gz::rendering::ScenePtr
    + ***Replacement:***  RenderingSensor::SetScene

    + ***Deprecation:*** public: gz::rendering::ScenePtr RenderingScene() const
    + ***Replacement:*** RenderingSensor::Scene()

1. **include/sensors/Noise.hh**
    + ***Deprecation:*** public: virtual void SetCamera(rendering::CameraPtr)
    + ***Replacement:***  TODO (to be implemented in ImageGaussianNoiseModel)
