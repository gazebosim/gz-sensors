# Gazebo Sensors : Sensor models for simulation

**Maintainer:** ichen AT openrobotics DOT org

[![GitHub open issues](https://img.shields.io/github/issues-raw/gazebosim/gz-sensors.svg)](https://github.com/gazebosim/gz-sensors/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/gazebosim/gz-sensors.svg)](https://github.com/gazebosim/gz-sensors/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/gazebosim/gz-sensors/tree/gz-sensors9/graph/badge.svg)](https://codecov.io/gh/gazebosim/gz-sensors/tree/gz-sensors9)
Ubuntu Noble | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=gz_sensors-ci-gz-sensors9-noble-amd64)](https://build.osrfoundation.org/job/gz_sensors-ci-gz-sensors9-noble-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=gz_sensors-ci-gz-sensors9-homebrew-amd64)](https://build.osrfoundation.org/job/gz_sensors-ci-gz-sensors9-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=gz_sensors-9-clowin)](https://build.osrfoundation.org/job/gz_sensors-9-clowin/)

Gazebo Sensors, a component of [Gazebo](https://gazebosim.org),
provides numerous sensor models
designed to generate realistic data from simulation environments. Gazebo Sensors is used in conjunction with [Gazebo Libraries](https://gazebosim.org/libs), and especially relies on the rendering capabilities from [Gazebo Rendering](https://gazebosim.org/libs/rendering) and physics simulation from [Gazebo Physics](https://gazebosim.org/libs/physics).

# Table of Contents

[Features](#features)

[Install](#install)

[Usage](#usage)

[Folder Structure](#folder-structure)

[Code of Conduct](#code-of-conduct)

[Contributing](#contributing)

[Versioning](#versioning)

[License](#license)

# Features

Gazebo Sensors provides a set of sensors models that can be
configured at run time to mimic specific real-world sensors. A noise model
is also provided that can be used to introduce Gaussian or custom noise
models into sensor streams.

## Supported Sensors

| **Sensor Name**           | **API Link**                                                                                                 | **SDF Spec**                                                                                      | **Example**                                                                                                           | **Notes**                                          |
|---------------------------|-------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------|----------------------------------------------------|
| **Air Pressure Sensor**   | [AirPressureSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1AirPressureSensor.html)        | [SDF Air Pressure](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_air_pressure)              | [air_pressure.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/air_pressure.cc)                | Measures atmospheric pressure                      |
| **Air Speed Sensor**      | [AirSpeedSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1AirSpeedSensor.html)              | [SDF Air Speed](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_air_speed)                    | [air_speed.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/air_speed.cc)                      | Measures the speed of air relative to the sensor   |
| **Altimeter**             | [AltimeterSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1AltimeterSensor.html)            | [SDF Altimeter](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_altimeter)                    | [altimeter.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/altimeter.cc)                      | Measures altitude above a reference point          |
| **Bounding Box Camera**   | [BoundingBoxCameraSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1BoundingBoxCameraSensor.html) | [SDF Bounding Box Camera](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_boundingbox_camera) | [boundingbox_camera.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/boundingbox_camera.cc)    | Captures images with bounding box annotations      |
| **Camera**                | [CameraSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1CameraSensor.html)                  | [SDF Camera](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_camera)                          | [camera.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/camera.cc)                            | Captures standard RGB images                       |
| **Depth Camera**          | [DepthCameraSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1DepthCameraSensor.html)        | [SDF Depth Camera](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_depth_camera)              | [depth_camera.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/depth_camera.cc)                | Captures depth information                         |
| **Doppler Velocity Log**  | [DopplerVelocityLog](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1DopplerVelocityLog.html)      | [SDF DVL](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_dvl)                                | [dvl.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/dvl.cc)                                  | Measures relative velocity of an underwater vehicle        |
| **Force-Torque**          | [ForceTorqueSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1ForceTorqueSensor.html)        | [SDF Force-Torque](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_force_torque)              | [force_torque.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/force_torque.cc)                | Measures forces and torques on a joint             |
| **GPU Lidar**             | [GpuLidarSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1GpuLidarSensor.html)              | [SDF GPU Lidar](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_gpu_lidar)                    | [gpu_lidar_sensor.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/gpu_lidar_sensor.cc)                      | Simulates a 3D laser scanner using GPU acceleration|
| **IMU**                   | [ImuSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1ImuSensor.html)                        | [SDF IMU](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_imu)                                | [imu.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/imu.cc)                                  | Measures acceleration and angular velocity         |
| **Logical Camera**        | [LogicalCameraSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1LogicalCameraSensor.html)    | [SDF Logical Camera](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_logical_camera)          | [logical_camera.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/logical_camera.cc)            | Detects models within a specified volume           |
| **Magnetometer**       | [MagnetometerSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1MagnetometerSensor.html)            | [SDF Magnetometer](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_magnetometer)                  | [magnetometer.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/magnetometer.cc)                | Measures magnetic field strength and direction |
| **NavSat (GPS)**       | [NavSatSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1NavSatSensor.html)                        | [SDF NavSat](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_navsat)                              | [navsat.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/navsat.cc)                            | Simulates GPS-like positioning sensor          |
| **RGBD Camera**        | [RGBDCameraSensor](https://gazebosim.org/api/sensors/7/classgz_1_1sensors_1_1RgbdCameraSensor.html)                | [SDF RGBD](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_rgbd_camera)                           | [rgbd_camera.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/rgbd_camera.cc)                  | Captures RGB + Depth streams                   |
| **Thermal Camera**     | [ThermalCameraSensor](https://gazebosim.org/api/sensors/7/classgz_1_1sensors_1_1ThermalCameraSensor.html)          | [SDF Thermal Camera](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_thermal_camera)              | [thermal_camera.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/thermal_camera.cc)            | Detects heat signature                         |
| **Wide Angle Camera**  | [WideAngleCameraSensor](https://gazebosim.org/api/sensors/8/classgz_1_1sensors_1_1WideAngleCameraSensor.html)      | [SDF Wide Angle](http://sdformat.org/spec?ver=1.9&elem=sensor#sensor_wideanglecamera)                 | [wide_angle_camera.cc](https://github.com/gazebosim/gz-sensors/blob/main/test/integration/wide_angle_camera.cc)      | Captures wide field of view                    |

# Install

See the [installation tutorial](https://gazebosim.org/api/sensors/9/installation.html).

# Usage

Please refer to the [examples directory](https://github.com/gazebosim/gz-sensors/tree/main/examples).

A list of sensors and SDF examples can be found in the [SDF specification](http://sdformat.org/spec?ver=1.12&elem=sensor)

# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
├── examples                  Example programs.
├── include/gz/sensors        Header files that will be installed.
├── src                       Source files and unit tests.
├── test
│    ├── integration          Integration tests.
│    ├── performance          Performance tests.
│    └── regression           Regression tests.
├── tutorials                 Tutorials, written in markdown.
├── Changelog.md              Changelog.
├── CMakeLists.txt            CMake build script.
└── README.md                 This readme.
```

# Contributing

Please see the [contribution guide](https://gazebosim.org/docs/all/contributing).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/gazebosim/gz-sim/blob/main/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Gazebo project](https://gazebosim.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Gazebo website](https://gazebosim.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/gazebosim/gz-sensors/blob/main/LICENSE) file.
