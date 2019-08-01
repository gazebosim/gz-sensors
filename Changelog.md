## Ignition Sensors

### Ignition Sensors 2.X.X

### Ignition Sensors 2.5.0

1. Add `IGN_PROFILER_ENABLE` cmake option for enabling the ign-common profiler.
    * [Pull request 82](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/82)

1. Deduplicate `frame_ids` from sensor message headers
    * [Pull request 83](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/83)

1. Baseline for stereo cameras
    * [Pull request 84](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/84)

### Ignition Sensors 2.4.0 (2019-07-17)

1. Support manual scene updates for rendering sensors
    * [Pull request 81](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/81)

### Ignition Sensors 2.3.0 (2019-07-16)

1. The GpuLidar and Rgbd sensors publish point cloud data using
   `msgs::PointCloudPacked`.
    * [Pull request 78](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/78)

### Ignition Sensors 2.2.0 (2019-06-27)

1. Update the GPU Lidar to use the sensor's name as the `frame_id`.
    * [Pull request 74](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/74)

1. Fix camera_info topic to be on the same level as image and depth_image for RGBD Camera.
    * [Pull request 73](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/73)

### Ignition Sensors 2.1.0 (2019-06-18)

1. Adds an RGBD camera sensor that combines a CameraSensor and DepthCameraSensor, and also
   outputs a pointcloud.
    * [Pull request 70](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/70)

1. Create and publish on `camera_info` topics for the Camera and DepthCamera
   sensors.
    * [Pull request 67](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/67)

### Ignition Sensors 2.0.0 (2019-05-21)

1. Zero update rate, virtual SetParent and fix gpu_lidar
    * [Pull request 66](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/66)

1. Add `frame_id` to sensor messages
    * [Pull request 63](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/63)

1. Restore `pixel_format` in message and add deprecation comment.
    * [Pull request 62](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/62)
    * [Pull request 65](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/65)

1. Added noise to  camera and lidar sensors.
    * [Pull request 60](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/60)
    * [Pull request 61](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/61)

1. Add support for loading a Lidar sensor from an SDF Sensor DOM object.
    * [Pull request 59](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/59)

1. Add support for loading an IMU sensor from an SDF Sensor DOM object.
    * [Pull request 58](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/58)

1. Add support for loading a camera and depth camera sensor from an SDF Sensor DOM object.
    * [Pull request 57](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/57)

1. Add support for loading an air pressure sensor from an SDF Sensor DOM object.
    * [Pull request 56](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/56)

1. Add support for loading an altimeter sensor from an SDF Sensor DOM object.
    * [Pull request 55](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/55)

1. Noise factory uses `sdf::Noise` objects, Magnetometer sensor utilizes
   noise parameters.
    * [Pull request 54](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/54)

1. Add support for loading a magnetometer sensor from an SDF Sensor DOM object.
    * [Pull request 53](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/53)

1. Add magnetometer
    * [Pull request 47](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/47)

1. Add IMU
    * [Pull request 44](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/44)

1. Add altimeter
    * [Pull request 43](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-request/43)

1. Create component for rendering sensor classes
   * [Pull request 42](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/42)

1. Upgrade to ignition-rendering2
   * [Pull request 45](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/45)

1. Upgrade to ignition-msgs4 and ignition-transport7
   * [Pull request 51](https://bitbucket.org/ignitionrobotics/ign-sensors/pull-requests/51)

### Ignition Sensors 1.X.X (2019-XX-XX)

1. Fix windows linking
    * [Pull request 49](https://bitbucket.org/osrf/gazebo/pull-request/49)
    * [Issue 6](https://bitbucket.org/osrf/gazebo/issues/6)

### Ignition Sensors 1.0.0 (2019-03-01)
