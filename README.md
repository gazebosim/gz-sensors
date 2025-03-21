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
