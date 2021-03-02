# Ignition Sensors : Sensor models for simulation

**Maintainer:** ichen AT openrobotics DOT org

[![GitHub open issues](https://img.shields.io/github/issues-raw/ignitionrobotics/ign-sensors.svg)](https://github.com/ignitionrobotics/ign-sensors/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/ignitionrobotics/ign-sensors.svg)](https://github.com/ignitionrobotics/ign-sensors/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/ignitionrobotics/ign-sensors/branch/master/graph/badge.svg)](https://codecov.io/gh/ignitionrobotics/ign-sensors)  
Ubuntu Bionic | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_sensors-ci-master-bionic-amd64)](https://build.osrfoundation.org/job/ignition_sensors-ci-master-bionic-amd64)  
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_sensors-ci-master-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_sensors-ci-master-homebrew-amd64)  
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_sensors-ci-master-windows7-amd64)](https://build.osrfoundation.org/job/ignition_sensors-ci-master-windows7-amd64)

Ignition Sensors, a component of [Ignition
Robotics](https://ignitionrobotics.org), provides numerous sensor models
designed to generate realistic data from simulation environments. Ignition Sensors is used in conjunction with [Ignition Libraries](https://ignitionrobotics/libs), and especially relies on the rendering capabilities from [Ignition Rendering](https://ignitionrobotics.org/libs/rendering) and physics simulation from [Ignition Physics](https://ignitionrobotics.org/libs/physics).

# Table of Contents

[Features](#features)

[Install](#install)

* [Binary Install](#binary-install)

* [Source Install](#source-install)

    * [Prerequisites](#prerequisites)
  
    * [Building from Source](#building-from-source)

[Usage](#usage)

[Supported Sensors](#supported-sensors)

[Folder Structure](#folder-structure)

[Code of Conduct](#code-of-conduct)

[Contributing](#code-of-contributing)

[Versioning](#versioning)

[License](#license)

# Features

Ignition Sensors provides a set of sensors models that can be
configured at run time to mimic specific real-world sensors. A noise model
is also provided that can be used to introduce Gaussian or custom noise
models into sensor streams.

# Install

See the [installation tutorial](https://ignitionrobotics.org/api/sensors/4.0/installation.html).

# Usage

Please refer to the [examples directory](https://github.com/ignitionrobotics/ign-sensors/raw/master/examples/).

# Supported Sensors

[Air Pressure Sensor](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1AirPressureSensor.html)

[Altimeter](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1AltimeterSensor.html)

[Camera Sensor](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1CameraSensor.html)

[Depth Camera Sensor](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1DepthCameraSensor.html)

[Gaussian Noise Model](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1GaussianNoiseModel.html)

[GPU Lidar Sensor](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1GpuLidarSensor.html)

[Image Gaussian Noise Model](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1ImageGaussianNoiseModel.html)

[Image Noise Factory](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1ImageNoiseFactory.html)

[IMU Sensor](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1ImuSensor.html)

[Lidar](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1Lidar.html)

[Logical Camera Sensor](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1LogicalCameraSensor.html)

[Magnetometer Sensor](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1MagnetometerSensor.html)

[Manager](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1Manager.html)

[Noise](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1Noise.html)

[Noise Factory](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1NoiseFactory.html)

[Rendering Events](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1RenderingEvents.html)

[Rendering Sensor](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1RenderingSensor.html)

[RGBD Camera Sensor](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1RgbdCameraSensor.html)

[Sensor](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1Sensor.html)

[Sensor Factory](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1SensorFactory.html)

[Sensor Plugin](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1SensorPlugin.html)

[Sensor Type Plugin](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1SensorTypePlugin.html)

[Thermal Camera Sensor](https://ignitionrobotics.org/api/sensors/4.1/classignition_1_1sensors_1_1ThermalCameraSensor.html)

# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
├── examples                  Example programs.
├── include/ignition/sensors  Header files that will be installed.
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

Please see
[CONTRIBUTING.md](https://github.com/ignitionrobotics/ign-gazebo/blob/master/CONTRIBUTING.md).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/ignitionrobotics/ign-gazebo/blob/master/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Ignition Robotics project](https://ignitionrobotics.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Ignition Robotics website](https://ignitionrobotics.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/ignitionrobotics/ign-sensors/blob/master/LICENSE) file.
