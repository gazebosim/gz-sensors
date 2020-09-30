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

1. Install Ignition Sensors

```{.sh}
# This installs ign-sensors4. Change the number after libignition-sensors to the version you want
sudo apt install libignition-sensors4-dev
```

## Source Install

Source installation can be performed in UNIX systems by first installing the
necessary prerequisites followed by building from source.

### Prerequisites

Ignition Sensors requires:

  * [Ignition CMake](https://ignitionrobotics.org/libs/cmake)
  * [Ignition Math](https://ignitionrobotics.org/libs/math)
  * [Ignition Common](https://ignitionrobotics.org/libs/common)
  * [Ignition Transport](https://ignitionrobotics.org/libs/transport)
  * [Ignition Rendering](https://ignitionrobotics.org/libs/rendering)
  * [Ignition Msgs](https://ignitionrobotics.org/libs/msgs)
  * [SDFormat](https://github.com/osrf/sdformat)
  * [Protobuf3](https://developers.google.com/protocol-buffers/)

### Building from source

1. Make sure you are running [Ubuntu Bionic](http://releases.ubuntu.com/18.04/) or above.

2. Install the [Prerequisites](#prerequisites).

3. Configure to use gcc8 if that is not the default compiler

```{.sh}
sudo apt-get install g++-8
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
```

4. Clone the repository

```{.sh}
git clone https://github.com/ignitionrobotics/ign-sensors
```

5. Configure and build

```{.sh}
cd ign-sensors; mkdir build; cd build; cmake ..;  make
```

6. Optionally, install the library

```{.sh}
sudo make install
```


