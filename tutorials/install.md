\page installation Installation

Next Tutorial: \ref custom_sensors

We recommend following the binary install instructions to get up and running as quickly and painlessly as possible.

The source install instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

## Binary Install

### Ubuntu

1. Setup your computer to accept software from packages.osrfoundation.org:

```{.sh}
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
```

2. Install Gazebo Sensors
```{.sh}
# Change <#> to a version number, like 7 or 8
sudo apt install libgz-sensors<#>-dev
```

### macOS

1. On macOS, add OSRF packages:
  ```
  ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
  brew tap osrf/simulation
  ```

2. Install Gazebo Sensors:
  ```
  brew install gz-sensors<#>
  ```

Be sure to replace `<#>` with a number value, such as 7 or 8, depending on
which version you need.

### Windows

#### Install Prerequisites

First, follow the [source installation](https://gazebosim.org/docs/ionic/install_windows_src/) tutorial until step 5 included for installing Conda, Visual Studio, CMake, etc., prerequisites, and creating a Conda environment.

Navigate to `condabin` if necessary to use the `conda` command (i.e., if Conda is not in your `PATH` environment variable. You can find the location of `condabin` in Anaconda Prompt, `where conda`).

Create if necessary, and activate a Conda environment:

```
conda create -n gz-ws
conda activate gz-ws
```

#### Binary Installation

```
conda install libgz-sensors<#> --channel conda-forge
```

Be sure to replace `<#>` with a number value, such as 7 or 8, depending on
which version you need.

## Source Install

Source installation can be performed in UNIX systems by first installing the
necessary prerequisites followed by building from source.

### Prerequisites

Gazebo Sensors requires:

  * [Gazebo CMake](https://gazebosim.org/libs/cmake)
  * [Gazebo Math](https://gazebosim.org/libs/math)
  * [Gazebo Common](https://gazebosim.org/libs/common)
  * [Gazebo Transport](https://gazebosim.org/libs/transport)
  * [Gazebo Rendering](https://gazebosim.org/libs/rendering)
  * [Gazebo Msgs](https://gazebosim.org/libs/msgs)
  * [SDFormat](https://github.com/osrf/sdformat)
  * [Protobuf3](https://developers.google.com/protocol-buffers/)

### Ubuntu

1. Make sure you are running [Ubuntu Focal](http://releases.ubuntu.com/20.04/) or above.

2. Install the Prerequisites.

3. Clone the repository
```{.sh}
  git clone https://github.com/gazebosim/gz-sensors
  ```

4. Configure and build
  ```
  cd gz-sensors; mkdir build; cd build; cmake ..;  make
  ```

5. Optionally, install the library
  ```
  sudo make install
  ```

### macOS

1. Clone the repository
  ```
  git clone https://github.com/gazebosim/gz-sensors -b gz-sensors<#>
  ```
  Be sure to replace `<#>` with a number value, such as 5 or 6, depending on
  which version you need.

2. Install dependencies
  ```
  brew install --only-dependencies gz-sensors<#>
  ```
  Be sure to replace `<#>` with a number value, such as 5 or 6, depending on
  which version you need.

3. Configure and build
  ```
  cd gz-sensors
  mkdir build
  cd build
  cmake ..
  make
  ```

4. Optionally, install
  ```
  sudo make install
  ```

### Windows

This assumes you have created and activated a Conda environment while [installing the Prerequisites](#install-prerequisites).

1. Install Gazebo dependencies:

  You can view available versions and their dependencies:
  ```
  conda search libgz-sensors* --channel conda-forge --info
  ```

  Install dependencies, replacing `<#>` with the desired versions:

  ```
  conda install libgz-cmake<#> libgz-common<#> libgz-math<#> libgz-transport<#> libgz-msgs<#> libgz-rendering<#> --channel conda-forge
  ```

2. Navigate to where you would like to build the library, and clone the repository.
  ```
  # Optionally, append `-b gz-sensors#` (replace # with a number) to check out a specific version
  git clone https://github.com/gazebosim/gz-sensors.git
  ```

3. Configure and build
  ```
  cd gz-sensors
  mkdir build
  cd build
  cmake .. -DBUILD_TESTING=OFF  # Optionally, -DCMAKE_INSTALL_PREFIX=path\to\install
  cmake --build . --config Release
  ```

4. Optionally, install. You will likely need to run a terminal with admin privileges for this call to succeed.
  ```
  cmake --install . --config Release
  ```

# Documentation

API and tutorials can be found at [https://gazebosim.org/libs/sensors](https://gazebosim.org/libs/sensors).

You can also generate the documentation from a clone of this repository by following these steps.

1. You will need Doxygen. On Ubuntu Doxygen can be installed using
  ```
  sudo apt-get install doxygen
  ```

2. Clone the repository
  ```
  git clone https://github.com/gazebosim/gz-sensors
  ```

3. Configure and build the documentation.
  ```
  cd gz-sensors; mkdir build; cd build; cmake ../; make doc
  ```

4. View the documentation by running the following command from the build directory.
  ```
  firefox doxygen/html/index.html
  ```

# Testing

Follow these steps to run tests and static code analysis in your clone of this repository.

1. Run tests.
  ```
  make test
  ```

2. Static code checker.
  ```
  make codecheck
  ```
