# Gazebo Sensors : Sensor models for simulation

**Maintainer:** nate AT openrobotics DOT org

[![GitHub open issues](https://img.shields.io/github/issues-raw/gazebosim/gz-sensors.svg)](https://github.com/gazebosim/gz-sensors/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/gazebosim/gz-sensors.svg)](https://github.com/gazebosim/gz-sensors/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/gazebosim/gz-sensors/branch/ign-sensors3/graph/badge.svg)](https://codecov.io/gh/gazebosim/gz-sensors/branch/ign-sensors3)
Ubuntu Focal | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_sensors-ci-ign-sensors3-focal-amd64)](https://build.osrfoundation.org/job/ignition_sensors-ci-ign-sensors3-focal-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_sensors-ci-ign-sensors3-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_sensors-ci-ign-sensors3-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/job/ign_sensors-ign-3-win/badge/icon)](https://build.osrfoundation.org/job/ign_sensors-ign-3-win/)

Gazebo Sensors, a component of [Gazebo](https://gazebosim.org), provides numerous sensor models
designed to generate realistic data from simulation environments. Gazebo Sensors is used in conjunction with [Gazebo Libraries](https://gazebosim/libs), and especially relies on the rendering capabilities from [Gazebo Rendering](https://gazebosim.org/libs/rendering) and physics simulation from [Gazebo Physics](https://gazebosim.org/libs/physics).

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

Gazebo Sensors provides a set of sensors models that can be
configured at run time to mimic specific real-world sensors. A noise model
is also provided that can be used to introduce Gaussian or custom noise
models into sensor streams.

# Install

We recommend following the [Binary Install](#binary-install) instructions to get up and running as quickly and painlessly as possible.

The [Source Install](#source-install) instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

## Binary Install

On Ubuntu systems, `apt-get` can be used to install `ignition-sensors`:

**Ubuntu Bionic**

1. Configure package repositories.

    ```
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    ```

    ```
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-prerelease `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-prerelease.list'
    ```

    ```
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    ```

    ```
    sudo apt-get update
    ```

```
sudo apt install libignition-sensors-dev
```

At the time of this writing, there is only one released version of
Gazebo Sensors. It's possible that additional versions have been released,
  in which case you can use add a numeral (2, 3, etc) to install a different
  version. For example, to install version 2:

```
sudo apt-get install libignition-sensors2-dev
```

## Source Install

Source installation can be performed in UNIX systems by first installing the
necessary prerequisites followed by building from source.

### Prerequisites

Gazebo Sensors requires:

  * [Ubuntu Bionic](http://releases.ubuntu.com/18.04/)
  * [Gazebo CMake](https://gazebosim.org/libs/cmake)
  * [Gazebo Math](https://gazebosim.org/libs/math)
  * [Gazebo Common](https://gazebosim.org/libs/common)
  * [Gazebo Transport](https://gazebosim.org/libs/transport)
  * [Gazebo Rendering](https://gazebosim.org/libs/rendering)
  * [Gazebo Msgs](https://gazebosim.org/libs/msgs)
  * [SDFormat](https://github.com/osrf/sdformat)
  * [Protobuf3](https://developers.google.com/protocol-buffers/)

### Building from source

1. Make sure you are running [Ubuntu Bionic](http://releases.ubuntu.com/18.04/).

2. Install the [Prerequisites](#prerequisites).

3. Configure gcc8

    ```
    sudo apt-get install g++-8
    ```

    ```
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
    ```

4. Clone the repository

    ```
    git clone https://github.com/gazebosim/gz-sensors
    ```

5. Configure and build

    ```
    cd gz-sensors; mkdir build;cd build; cmake ..;  make
    ```

6. Optionally, install Gazebo Common

    ```
    sudo make install
    ```

# Usage

Please refer to the [examples directory](https://github.com/gazebosim/gz-sensors/raw/ign-sensors3/examples/).

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

Please see the [contribution guide](https://gazebosim.org/docs/all/contributing).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/gazebosim/gz-sim/blob/main/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Gazebo project](https://gazebosim.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Gazebo website](https://gazebosim.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/gazebosim/gz-sensors/blob/main/LICENSE) file.
