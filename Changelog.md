## Ignition Sensors 4

### Ignition Sensors 4.X.X

### Ignition Sensors 4.1.0 (2021-02-10)

1. Added issue and PR templates.
    * [Pull request 91](https://github.com/ignitionrobotics/ign-sensors/pull/91)

1. Added `set_rate` service to all sensors.
    * [Pull request 95](https://github.com/ignitionrobotics/ign-sensors/pull/95)

1. Added support for 8 bit thermal camera image format.
    * [Pull request 92](https://github.com/ignitionrobotics/ign-sensors/pull/92)

1. All features up to version 3.2.0.

### Ignition Sensors 4.0.0 (2020-09-30)

1. Fix link in README.md
    * [Pull request 51](https://github.com/ignitionrobotics/ign-sensors/pull/51)

1. Move installation instructions from README.md to Installation tutorial
    * [Pull request 50](https://github.com/ignitionrobotics/ign-sensors/pull/50)

1. Bump ign-math to 6.6
    * [Pull request 48](https://github.com/ignitionrobotics/ign-sensors/pull/48)

1. Replaced common::Time with std::chrono
    * [Pull request 41](https://github.com/ignitionrobotics/ign-sensors/pull/41)

1. Depend on ign-msgs6, ign-transport9, sdf10
    * [Pull request 31](https://github.com/ignitionrobotics/ign-sensors/pull/31)

1. GitHub migration
    * [Pull request 12](https://github.com/ignitionrobotics/ign-sensors/pull/12)
    * [Pull request 16](https://github.com/ignitionrobotics/ign-sensors/pull/16)
    * [Pull request 22](https://github.com/ignitionrobotics/ign-sensors/pull/22)

1. Set camera sensor visibility mask
    * [BitBucket pull request 115](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/115)

1. Depend on ign-rendering4
    * [BitBucket pull request 111](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/111)

## Ignition Sensors 3

### Ignition Sensors 3.2.0 (2021-02-08)

1. Apply noise to lidar point cloud.
    * [Pull request 86](https://github.com/ignitionrobotics/ign-sensors/pull/86)

1. Add Windows Installation.
    * [Pull request 82](https://github.com/ignitionrobotics/ign-sensors/pull/82)

1. Added thermal camera tutorial.
    * [Pull request 61](https://github.com/ignitionrobotics/ign-sensors/pull/61)

1. Prevent segfaults on test failures, make tests verbose. 
    * [Pull request 56](https://github.com/ignitionrobotics/ign-sensors/pull/56)

1. Resolve updated codecheck issues.
    * [Pull request 57](https://github.com/ignitionrobotics/ign-sensors/pull/57)

1. Improve fork experience.
    * [Pull request 54](https://github.com/ignitionrobotics/ign-sensors/pull/54)

### Ignition Sensors 3.1.0 (2020-09-03)

1. Update camera sensor only when needed
    * [Pull request 37](https://github.com/ignitionrobotics/ign-sensors/pull/37)

1. Add noise to RGBD camera.
    * [Pull Request 35](https://github.com/ignitionrobotics/ign-sensors/pull/35)

1. Fix version numbers in config.hh
    * [Pull Request 42](https://github.com/ignitionrobotics/ign-sensors/pull/42)

1. Make sure all sensors have a default topic. When invalid topics are passed
   in, convert them to valid topics if possible. If not possible to convert
   into valid topic, fail gracefully.
    * [Pull Request 33](https://github.com/ignitionrobotics/ign-sensors/pull/33)

1. GitHub migration
    * [Pull request 11](https://github.com/ignitionrobotics/ign-sensors/pull/11)
    * [Pull request 21](https://github.com/ignitionrobotics/ign-sensors/pull/21)

### Ignition Sensors 3.0.0 (2019-12-10)

1. Add support for sdformat frame semantics
    * [BitBucket pull request 104](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/104)

1. Remove deprecations in ign-sensors3
    * [BitBucket pull request 103](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/103)

1. Break out image noise classes
    * [BitBucket pull request 102](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/102)

1. Depend on ign-transport8, ign-msgs5, sdformat9
    * [BitBucket pull request 101](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/101)
    * [BitBucket pull request 105](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/105)

1. Add Thermal Camera Sensor
    * [BitBucket pull request 100](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/100)

1. Updating exports and includes
    * [BitBucket pull request 98](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/98)

1. Removed deprecations from Manager.
    * [BitBucket pull request 99](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/99)

1. Depend on ign-rendering3
    * [BitBucket pull request 88](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/88)

## Ignition Sensors 2

### Ignition Sensors 2.9.0 (2020-08-07)

1. Add noise to RGBD camera.
    * [Pull Request 35](https://github.com/ignitionrobotics/ign-sensors/pull/35)

1. Make sure all sensors have a default topic.When invalid topics are passed
   in, convert them to valid topics if possible. If not possible to convert
   into valid topic, fail gracefully.
    * [Pull Request 33](https://github.com/ignitionrobotics/ign-sensors/pull/33)


### Ignition Sensors 2.8.0 (2020-03-04)

1. Added sequence numbers to sensor data messages.
    * [BitBucket pull request 112](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/112)

### Ignition Sensors 2.7.0 (2019-12-16)

1. Add clipping for depth camera on rgbd camera sensor (requires sdformat 8.7.0)
    * [BitBucket pull request 107](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/107)

### Ignition Sensors 2.6.1 (2019-09-13)

1. Fix IMU noise model dt
    * [BitBucket pull request 94](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/94)

### Ignition Sensors 2.6.0 (2019-08-27)

1. Update depth and rgbd camera sensor to output point cloud data generated by ign-rendering DepthCamera
    * [BitBucket pull request 91](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/91)

### Ignition Sensors 2.5.1 (2019-08-12)

1. Add intensity and ring fields to GpuLidarSensor point cloud msg
    * [BitBucket pull request 89](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/89)

### Ignition Sensors 2.5.0

1. Add `IGN_PROFILER_ENABLE` cmake option for enabling the ign-common profiler.
    * [BitBucket pull request 82](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/82)

1. Deduplicate `frame_ids` from sensor message headers
    * [BitBucket pull request 83](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/83)

1. Baseline for stereo cameras
    * [BitBucket pull request 84](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/84)

### Ignition Sensors 2.4.0 (2019-07-17)

1. Support manual scene updates for rendering sensors
    * [BitBucket pull request 81](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/81)

### Ignition Sensors 2.3.0 (2019-07-16)

1. The GpuLidar and Rgbd sensors publish point cloud data using
   `msgs::PointCloudPacked`.
    * [BitBucket pull request 78](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/78)

### Ignition Sensors 2.2.0 (2019-06-27)

1. Update the GPU Lidar to use the sensor's name as the `frame_id`.
    * [BitBucket pull request 74](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/74)

1. Fix camera_info topic to be on the same level as image and depth_image for RGBD Camera.
    * [BitBucket pull request 73](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/73)

### Ignition Sensors 2.1.0 (2019-06-18)

1. Adds an RGBD camera sensor that combines a CameraSensor and DepthCameraSensor, and also
   outputs a pointcloud.
    * [BitBucket pull request 70](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/70)

1. Create and publish on `camera_info` topics for the Camera and DepthCamera
   sensors.
    * [BitBucket pull request 67](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/67)

### Ignition Sensors 2.0.0 (2019-05-21)

1. Zero update rate, virtual SetParent and fix gpu_lidar
    * [BitBucket pull request 66](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/66)

1. Add `frame_id` to sensor messages
    * [BitBucket pull request 63](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/63)

1. Restore `pixel_format` in message and add deprecation comment.
    * [BitBucket pull request 62](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/62)
    * [BitBucket pull request 65](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/65)

1. Added noise to  camera and lidar sensors.
    * [BitBucket pull request 60](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/60)
    * [BitBucket pull request 61](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/61)

1. Add support for loading a Lidar sensor from an SDF Sensor DOM object.
    * [BitBucket pull request 59](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/59)

1. Add support for loading an IMU sensor from an SDF Sensor DOM object.
    * [BitBucket pull request 58](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/58)

1. Add support for loading a camera and depth camera sensor from an SDF Sensor DOM object.
    * [BitBucket pull request 57](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/57)

1. Add support for loading an air pressure sensor from an SDF Sensor DOM object.
    * [BitBucket pull request 56](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/56)

1. Add support for loading an altimeter sensor from an SDF Sensor DOM object.
    * [BitBucket pull request 55](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/55)

1. Noise factory uses `sdf::Noise` objects, Magnetometer sensor utilizes
   noise parameters.
    * [BitBucket pull request 54](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/54)

1. Add support for loading a magnetometer sensor from an SDF Sensor DOM object.
    * [BitBucket pull request 53](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/53)

1. Add magnetometer
    * [BitBucket pull request 47](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/47)

1. Add IMU
    * [BitBucket pull request 44](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/44)

1. Add altimeter
    * [BitBucket pull request 43](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/43)

1. Create component for rendering sensor classes
    * [BitBucket pull request 42](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/42)

1. Upgrade to ignition-rendering2
    * [BitBucket pull request 45](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/45)

1. Upgrade to ignition-msgs4 and ignition-transport7
    * [BitBucket pull request 51](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/51)

### Ignition Sensors 1.X.X (2019-XX-XX)

1. Fix windows linking
    * [BitBucket pull request 49](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/49)
    * [Issue 6](https://github.com/osrf/gazebo/issues/6)

### Ignition Sensors 1.0.0 (2019-03-01)
