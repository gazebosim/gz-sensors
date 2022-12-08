\page installation Installation

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
# Change <#> to a version number, like 3 or 4
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

Be sure to replace `<#>` with a number value, such as 5 or 6, depending on
which version you need.

### Windows

Binary install is pending `gz-rendering` and `gz-sensors` being added to conda-forge.

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
  git clone https://github.com/gazebosim/gz-sensors -b ign-sensors<#>
  ```
  Be sure to replace `<#>` with a number value, such as 5 or 6, depending on
  which version you need.

2. Install dependencies
  ```
  brew install --only-dependencies ignition-sensors<#>
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

#### Install Prerequisites

First, follow the [gz-cmake](https://github.com/gazebosim/gz-cmake) tutorial for installing Conda, Visual Studio, CMake, etc., prerequisites, and creating a Conda environment.

Navigate to `condabin` if necessary to use the `conda` command (i.e., if Conda is not in your `PATH` environment variable. You can find the location of `condabin` in Anaconda Prompt, `where conda`).

Create if necessary, and activate a Conda environment:

```
conda create -n gz-ws
conda activate gz-ws
```

Install Gazebo dependencies, replacing `<#>` with the desired versions:

```
conda install libgz-cmake<#> libgz-common<#> libgz-math<#> libgz-transport<#> libgz-msgs<#> --channel conda-forge
```

Before [gz-rendering](https://github.com/gazebosim/gz-rendering) becomes available on conda-forge, follow its tutorial to build it from source.

#### Build from source

1. Activate the Conda environment created in the prerequisites:
  ```
  conda activate gz-ws
  ```

2. Navigate to where you would like to build the library, and clone the repository.
  ```
  # Optionally, append `-b ign-sensors#` (replace # with a number) to check out a specific version
  git clone https://github.com/gazebosim/gz-sensors.git
  ```

3. Configure and build
  ```
  cd gz-sensors
  mkdir build
  cd build
  ```

4. Before `gz-rendering` becomes available on conda-forge, we need to build it from source and specify the path containing `gz-rendering-config.cmake` in `CMAKE_PREFIX_PATH`, for cmake to find `gz-rendering`. That path could be `gz-rendering-install-path\lib\cmake\gz-rendering4`, for example.
  ```
  cmake .. -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH=path\containing\ignition-rendering-config  # Optionally, -DCMAKE_INSTALL_PREFIX=path\to\install
  cmake --build . --config Release
  ```

5. Optionally, install. You will likely need to run a terminal with admin privileges for this call to succeed.
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
