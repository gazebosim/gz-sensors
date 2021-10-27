# Ignition Sensors : Sensor models for simulation

**Maintainer:** ichen AT openrobotics DOT org

[![GitHub open issues](https://img.shields.io/github/issues-raw/ignitionrobotics/ign-sensors.svg)](https://github.com/ignitionrobotics/ign-sensors/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/ignitionrobotics/ign-sensors.svg)](https://github.com/ignitionrobotics/ign-sensors/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/ignitionrobotics/ign-sensors/branch/main/graph/badge.svg)](https://codecov.io/gh/ignitionrobotics/ign-sensors)
Ubuntu Bionic | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_sensors-ci-main-bionic-amd64)](https://build.osrfoundation.org/job/ignition_sensors-ci-main-bionic-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_sensors-ci-main-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_sensors-ci-main-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/job/ign_sensors-ci-win/badge/icon)](https://build.osrfoundation.org/job/ign_sensors-ci-win/)

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

See the [installation tutorial](https://ignitionrobotics.org/api/sensors/5.0/installation.html).

# Usage

Please refer to the [examples directory](https://github.com/ignitionrobotics/ign-sensors/raw/main/examples/).

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

Please see the [contribution guide](https://ignitionrobotics.org/docs/all/contributing).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/ignitionrobotics/ign-gazebo/blob/main/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Ignition Robotics project](https://ignitionrobotics.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Ignition Robotics website](https://ignitionrobotics.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/ignitionrobotics/ign-sensors/blob/main/LICENSE) file.
