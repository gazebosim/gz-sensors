# Ignition Sensors : Sensor models for simulation

**Maintainer:** nate AT openrobotics DOT org 

[![GitHub open issues](https://img.shields.io/github/issues-raw/ignitionrobotics/ign-sensors.svg)](https://github.com/ignitionrobotics/ign-sensors/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/ignitionrobotics/ign-sensors.svg)](https://github.com/ignitionrobotics/ign-sensors/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/bb/ignitionrobotics/ign-sensors/branch/default/graph/badge.svg)](https://codecov.io/bb/ignitionrobotics/ign-sensors)  
Ubuntu Bionic | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_sensors-ci-default-bionic-amd64)](https://build.osrfoundation.org/job/ignition_sensors-ci-default-bionic-amd64)  
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_sensors-ci-default-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_sensors-ci-default-homebrew-amd64)  
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_sensors-ci-default-windows7-amd64)](https://build.osrfoundation.org/job/ignition_sensors-ci-default-windows7-amd64)

Ignition Sensors, a component of [Ignition
Robotics](https://ignitionrobotics.org), provides numerous sensor models
designed to generate realistic data from simulation environments. Ignition Sensors is used in conjunction with [Ignition Libraries](https://ignitionrobotics/libs), and especially relies on the rendering capabilities from [Ignition Rendering](https://ignitionrobotics.org/libs/rendering) and physics simulation from [Ignition Physics](https://ignitionrobotics.org/libs/physics).

# Table of Contents

[Features](#markdown-header-features)

[Install](#markdown-header-install)

* [Binary Install](#markdown-header-binary-install)

* [Source Install](#markdown-header-source-install)

    * [Prerequisites](#markdown-header-prerequisites)
  
    * [Building from Source](#markdown-header-building-from-source)

[Usage](#markdown-header-usage)

[Documentation](#markdown-header-documentation)

[Testing](#markdown-header-testing)

[Folder Structure](#markdown-header-folder-structure)

[Code of Conduct](#markdown-header-code-of-conduct)

[Contributing](#markdown-header-code-of-contributing)

[Versioning](#markdown-header-versioning)

[License](#markdown-header-license)

# Features

Ignition Sensors provides a set of sensors models that can be
configured at run time to mimic specific real-world sensors. A noise model
is also provided that can be used to introduce Gaussian or custom noise
models into sensor streams.

# Install

We recommend following the [Binary Install](#markdown-header-binary-install) instructions to get up and running as quickly and painlessly as possible.

The [Source Install](#markdown-header-source-install) instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

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
ignition-sensors. It's possible that additional versions have been released,
  in which case you can use add a numeral (2, 3, etc) to install a different
  version. For example, to install version 2:

```
sudo apt-get install libignition-sensors2-dev
```

## Source Install

Source installation can be performed in UNIX systems by first installing the
necessary prerequisites followed by building from source. 

### Prerequisites

Ignition Sensors requires:

  * [Ubuntu Bionic](http://releases.ubuntu.com/18.04/)
  * [Ignition CMake](https://ignitionrobotics.org/libs/cmake)
  * [Ignition Math](https://ignitionrobotics.org/libs/math)
  * [Ignition Common](https://ignitionrobotics.org/libs/common)
  * [Ignition Transport](https://ignitionrobotics.org/libs/transport)
  * [Ignition Rendering](https://ignitionrobotics.org/libs/rendering)
  * [Ignition Msgs](https://ignitionrobotics.org/libs/msgs)
  * [SDFormat](https://github.com/osrf/sdformat)
  * [Protobuf3](https://developers.google.com/protocol-buffers/)

### Building from source

1. Make sure you are running [Ubuntu Bionic](http://releases.ubuntu.com/18.04/).

2. Install the [Prerequisites](#markdown-header-prerequisites).

3. Configure gcc8

    ```
    sudo apt-get install g++-8
    ```

    ```
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
    ```

4. Clone the repository

    ```
    git clone https://github.com/ignitionrobotics/ign-sensors
    ```

5. Configure and build

    ```
    cd ign-sensors; mkdir build;cd build; cmake ..;  make
    ```

6. Optionally, install Ignition Common

    ```
    sudo make install
    ```

# Usage

Please refer to the [examples directory](https://github.com/ignitionrobotics/ign-sensors/raw/master/examples/).

# Documentation

API and tutorials can be found at [https://ignitionrobotics.org/libs/sensors](https://ignitionrobotics.org/libs/sensors).

You can also generate the documentation from a clone of this repository by following these steps.

1. You will need Doxygen. On Ubuntu Doxygen can be installed using

    ```
    sudo apt-get install doxygen
    ```

2. Clone the repository

    ```
    git clone https://github.com/ignitionrobotics/ign-sensors
    ```

3. Configure and build the documentation.

    ```
    cd ign-sensors; mkdir build; cd build; cmake ../; make doc
    ```

4. View the documentation by running the following command from the build directory.

    ```
    firefox doxygen/html/index.html
    ```

# Testing

Follow these steps to run tests and static code analysis in your clone of this repository.

1. Follow the [source install instruction](#markdown-header-source-install).

2. Run tests.

    ```
    make test
    ```

3. Static code checker.

    ```
    make codecheck
    ```

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
