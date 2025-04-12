\page thermalcameraigngazebo Thermal Camera in Gazebo

Next Tutorial: \ref segmentationcamera_igngazebo

In this tutorial, we will discuss how to use a thermal camera sensor in [Gazebo](https://gazebosim.org/libs/sim).

There are currently a few limitations with the thermal camera, which will be mentioned at the end of the tutorial.

## Requirements

Since this tutorial will show how to use a thermal camera sensor in Gazebo, you'll need to have Gazebo installed.
We recommend installing all Gazebo libraries, using version Citadel or newer (the thermal camera is not available in Gazebo versions prior to Citadel).

The following thermal camera capabilities are available starting from Gazebo Dome:
* Heat signature capabilities (supports objects with a variable surface temperature)
* 8-bit thermal camera image format (the default thermal camera image format is 16-bit)

If you need to install Gazebo, [pick the version you'd like to use](https://gazebosim.org/docs) and then follow the installation instructions.

## Setting up the thermal camera

Here's an example of how to attach a thermal camera sensor to a model in a [SDF](http://sdformat.org/) file:

```xml
<model name="thermal_camera_8bit">
  <pose>4.5 0 0.5 0.0 0.0 3.14</pose>
  <link name="link">
    <pose>0.05 0.05 0.05 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <box>
          <size>0.1 0.1 0.1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.1 0.1 0.1</size>
        </box>
      </geometry>
    </visual>
    <sensor name="thermal_camera_8bit" type="thermal">
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>320</width>
          <height>240</height>
          <format>L8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <topic>thermal_camera_8bit/image</topic>
      <plugin
        filename="gz-sim-thermal-sensor-system"
        name="gz::sim::systems::ThermalSensor">
        <min_temp>253.15</min_temp>
        <max_temp>673.15</max_temp>
        <resolution>3.0</resolution>
      </plugin>
    </sensor>
  </link>
  <static>true</static>
</model>
```

Let's take a closer look at the portion of the code above that focuses on the thermal camera sensor:

```xml
<sensor name="thermal_camera_8bit" type="thermal">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>320</width>
      <height>240</height>
      <format>L8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
  <topic>thermal_camera_8bit/image</topic>
  <plugin
    filename="gz-sim-thermal-sensor-system"
    name="gz::sim::systems::ThermalSensor">
    <min_temp>253.15</min_temp>
    <max_temp>673.15</max_temp>
    <resolution>3.0</resolution>
  </plugin>
</sensor>
```

As we can see, we define a sensor with the following SDF elements:
* `<camera>`: The camera, which has the following child elements:
    - `<horizontal_fov>`: The horizontal field of view, in radians.
    - `<image>`: The image size, in pixels.
      - `<format>`: The image format. Here, 8-bit (`L8`) is used, but 16-bit (`L16`) can also be used.
                    The default thermal camera image format is 16-bit unless otherwise specified.
    - `<clip>`: The near and far clip planes. Objects are only rendered if they're within these planes.
* `<always_on>`: Whether the sensor will always be updated (indicated by `1`) or not (indicated by `0`).
                 This is currently unused by Gazebo.
* `<update_rate>`: The sensor's update rate, in Hz.
* `<visualize>`: Whether the sensor should be visualized in the GUI (indicated by `true`) or not (indicated by `false`).
                 This is currently unused by Gazebo.
* `<topic>`: The name of the topic where sensor data is published. This is needed for visualization.

There's also an optional plugin used here that allows for further configuration of the thermal camera.
Here's a description of the elements in this plugin (if the plugin isn't used, the default values mentioned below are used):
* `<min_temp>`: The thermal camera's minimum temperature, in Kelvin.
                The default minimum temperature is `0`.
* `<max_temp>`: The thermal camera's maximum temperature, in Kelvin.
                The default maximum temperature is calculated as `(2^(bitDepth) - 1) * resolution`.
                For a 8-bit thermal camera with a linear resolution of `3.0`, the maximum temperature is `((2^8) - 1) * 3 = 765 Kelvin`.
* `<resolution>`: The thermal camera's linear resolution.
                  The default linear resolution is `0.01` for a 16-bit thermal camera, and `3.0` for a 8-bit thermal camera.

## Assigning a temperature to a model

Now that we have set up our thermal camera, we'll need to assign temperatures to models in the environment.
If a model doesn't have a temperature associated with it, then the model's temperature is set to the ambient temperature.
There will be some variation in the model's temperature based on the model's color and the world's temperature gradient.

The thermal camera can support objects that either have a uniform or varying surface temperature.
We will go over each approach in the following subsections.

### Setting atmospheric temperature properties

It is recommended to specify the ambient temperature and world's temperature gradient, since this dictates how objects with no specified temperature appear.
Here's an example that sets the ambient temperature to 300 kelvin, and the temperature gradient to 0.1 Kelvin/meter:

```xml
<atmosphere type="adiabatic">
  <temperature>300</temperature>
  <!--
    This is a more exaggerated temperature gradient, which produces a
    temperature range of ~11.5 kelvin for objects in the thermal camera
    view that don't have a user-specified temperature.
    Typical temperature gradient is -0.0065 K/m which produces a
    temperature range of 0.75 kelvin.
  -->
  <temperature_gradient>0.1</temperature_gradient>
</atmosphere>
```

The default values for ambient temperature and temperature gradient can be found in the [SDFormat Specification](http://sdformat.org/spec?ver=1.7&elem=world#world_atmosphere).

### Objects with a uniform temperature

Here's an example of a box model that has a uniform temperature assigned to it:

```xml
<model name="box">
  <pose>-1 1 0.5 0 0 0</pose>
  <link name="box_link">
    <inertial>
      <inertia>
        <ixx>1</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>1</iyy>
        <iyz>0</iyz>
        <izz>1</izz>
      </inertia>
      <mass>1.0</mass>
    </inertial>
    <collision name="box_collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="box_visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
        <specular>1 0 0 1</specular>
      </material>
      <plugin
        filename="gz-sim-thermal-system"
        name="gz::sim::systems::Thermal">
        <temperature>285.0</temperature>
      </plugin>
    </visual>
  </link>
</model>
```

Most of the code above is for the model - here's the key piece for temperature assignment:

```xml
<plugin
  filename="gz-sim-thermal-system"
  name="gz::sim::systems::Thermal">
  <temperature>285.0</temperature>
</plugin>
```

Once the thermal system plugin is specified, all we need to do is include the `<temperature>` element, which specifies the model's temperature in Kelvin.

### Objects with a varying surface temperature

Most of the time, objects have a varying surface temperature.
Here's an example of the [Rescue Randy](https://app.gazebosim.org/OpenRobotics/fuel/models/Rescue%20Randy) model, which has a "heat signature" applied to it.
The heat signature is a texture that can be applied to a model, which specifies the model's temperature across its surface.

If we take a look at Rescue Randy's `model.sdf`, we can see that it incorporates the thermal system plugin, just like the uniform temperature box example above:

```xml
<plugin
  filename="gz-sim-thermal-system"
  name="gz::sim::systems::Thermal">
  <heat_signature>materials/textures/RescueRandy_Thermal.png</heat_signature>
  <max_temp>310</max_temp>
</plugin>
```

There are a few differences to note for objects with a varying surface temperature:
* `<heat_signature>`: The path to the heat signature texture.
                      In this example, the path is given relative to where the Rescue Randy fuel model is located.
                      This can either be an [Gazebo Fuel](https://gazebosim.org/libs/fuel_tools) URI, or a path to a file that is on your machine locally.
* `<min_temp>`: The minimum temperature (in Kelvin) that should be represented by the heat signature.
                This defaults to the world's ambient temperature if not specified.
* `<max_temp>`: The maximum temperature (in Kelvin) that should be represented by the heat signature.
                This defaults to the world's ambient temperature if not specified.

## Running an example:

Now that we've discussed how a thermal camera and models with temperature can be specified, let's start an example world that uses the thermal camera.

Run the following command:

```
gz sim -r thermal_camera.sdf
```

You should see something similar to this:

@image html files/thermal_camera/thermal_camera_demo.png

Taking a look at the [SDF file](https://github.com/gazebosim/gz-sim/blob/e647570f25f962d63af75cf669ff72731d57bd5e/examples/worlds/thermal_camera.sdf) for this example shows that the box was assigned a temperature of 285 Kelvin.

If we take a look at the [Rescue Randy](https://app.gazebosim.org/OpenRobotics/fuel/models/Rescue%20Randy) and [Samsung J8](https://app.gazebosim.org/OpenRobotics/fuel/models/Samsung%20J8%20Black) Fuel models, we see that they have the following temperature range (the SDF file we are using with these models has an [ambient temperature of 300 Kelvin](https://github.com/gazebosim/gz-sim/blob/e647570f25f962d63af75cf669ff72731d57bd5e/examples/worlds/thermal_camera.sdf#L135-L144)):
* Rescue Randy: 300 Kelvin to 310 Kelvin
* Samsung J8: 298.75 Kelvin to 300 Kelvin

The temperature of objects in the view of the camera are normalized between 0 (black) and 255 (white) values.
The object with the highest temperature in the camera's view will always have white color.
If you move the object with the highest temperature out of the camera's view, the object with second highest temperature now becomes white.

You can move the position of the objects and/or camera around in the world to see the effect it has on the camera's output (the different "camera outputs" are how they are visualized in the GUI).
An easy way to move objects in the world is by using the `Transform Control` tool:

@image html files/thermal_camera/thermal_camera_demo_2.png

Another thing that you can do is modify the temperature ranges for objects with a heat signature.
For example, if you go to the fuel cache on your machine (located at `~/.gz/fuel/` by default) and then modify Rescue Randy's `model.sdf` to have `min_temp` be `200`, and `max_temp` be `500`, you should see output similar to this (be sure to re-start the simulator by closing the current simulator and then re-running `gz sim -r thermal_camera.sdf`):

@image html files/thermal_camera/thermal_camera_demo_3.png

Since Rescue Randy's maximum temperature (500 Kelvin) is significantly larger (~200 Kelvin or more) than the temperature of other objects in the scene, Rescue Randy is now a lot brighter compared to the rest of each camera's output.

## Processing the thermal camera's output

In the example above, the thermal cameras publish an [image message](https://github.com/gazebosim/gz-msgs/blob/main/proto/gz/msgs/image.proto) to the following topics whenever the camera has a new image:
* 8-bit thermal camera: `/thermal_camera_8bit/image`
* 16-bit thermal camera: `/thermal_camera`

We can observe the contents of a single message from the 16-bit camera by running the following command:

```
gz topic -e -n 1 -t /thermal_camera
```

Here are some important things to observe from the message output:
* `width`: 320
* `height`: 240
* `pixel_format_type`: L_INT16 (16-bit integer)

This means that the image is stored in the message as a byte array of size 320x240 pixels.
In order to do anything useful with the image stored in the message, we must convert the data in the array back to kelvin.
Since the `pixel_format_type` is `L_INT16`, that means we need to cast the byte array to an array of unsigned 16-bit integers.

Here's an example 16-bit thermal camera subscriber that performs this conversion:

```c
#include <cstdint>

#include <gz/msgs/image.pb.h>
#include <gz/transport.hh>

// used to set the proper resolution of the camera's output (10mK)
double linearResolution = 0.01;

// a callback function that is triggered whenever the thermal camera
// topic receives a new image message
void OnImage(const gz::msgs::Image &_msg)
{
  // convert the serialized image data to 16-bit temperature values
  unsigned int thermalSamples = _msg.width() * _msg.height();
  unsigned int thermalBufferSize = thermalSamples * sizeof(uint16_t);
  auto *thermalBuffer = new uint16_t[thermalSamples];
  memcpy(thermalBuffer, _msg.data().c_str(), thermalBufferSize);

  for (auto r = 0; r < _msg.height(); ++r)
  {
    // need to figure out the row offset in order to mimic 2D array access
    // with 1D array indexing
    auto rowOffset = r * _msg.width();

    for (auto c = 0; c < _msg.width(); ++c)
    {
      // convert the 16-bit value to Kelvin via the camera's linearResolution
      auto temp = thermalBuffer[rowOffset + c] * linearResolution;

      // do something useful with the temperature (in Kelvin) here
    }
  }

  delete[] thermalBuffer;
}

int main(int argc, char **argv)
{
  gz::transport::Node node;
  if (!node.Subscribe("/thermal_camera", &OnImage))
  {
    std::cerr << "Error subscribing to the thermal camera topic" << std::endl;
    return -1;
  }

  gz::transport::waitForShutdown();
}
```

Save the above code to a file named `main.cpp` and use this `CMakeLists.txt` to build it.

```cmake
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(image-listener)

# Find the Gazebo Libraries used directly by the example
find_package(gz-msgs11 REQUIRED)
find_package(gz-transport REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC gz-msgs11 gz-transport)
target_include_directories(${PROJECT_NAME} PUBLIC ${gz_msgs11_INCLUDE_DIRS})
```

Although most of the code above is described in the comments, let's go over the key points again:
* The image published by the camera is represented as serialized bytes.
  In order to read the actual temperature data that's stored in the image, we need to convert the data in the image to unsigned 16-bit integers.
* Once the image is stored in an array of unsigned 16-bit integers, we'll need to multiply the data in this array by the camera's linear resolution in order to ensure we are working with units of Kelvin.
Since this thermal camera has a linear resolution of .01 (10mK), and the maximum value that can be stored in an unsigned 16-bit integer is 65535, this means that the maximum temperature that can be recorded by this camera is (65535 * .01) = 655.35 Kelvin / 680.72 F / 360.4 C.

## Thermal Camera Limitations

The thermal camera has a few limitations:
* If one object with a given temperature blocks another object with a given temperature (with respect to the thermal camera's point of view), the blocked object will not be displayed at all, regardless of each object's thickness and temperature difference between them (see the image below for an example).
  A more realistic implementation would have the camera display temperature fluctuations in the closer object if the temperature difference between the two objects is large enough, and if the closer object isn't too thick.
    - More information about thermal camera behavior can be found [here](https://www.flir.com/discover/cores-components/can-thermal-imaging-see-through-walls/).
* There's a precision loss when the thermal camera converts temperature readings to gray scale output.
To help quantify the magnitude of this precision loss, running the conversion code above on [this SDF file](https://github.com/gazebosim/gz-sim/blob/990e4f240bbb3246a0e1d0c89b74e0ef8f109b4b/examples/worlds/thermal_camera.sdf) results in the following processed temperatures for each object in the camera's image:
    - sphere: listed in the SDF file as 600 Kelvin, processed from the thermal camera image topic as 598.81 Kelvin
    - box: listed in the SDF file as 200 Kelvin, processed from the thermal camera image topic as 200.46 Kelvin
    - cylinder: listed in the SDF file as 400 Kelvin, processed from the thermal camera image topic as 400.92 Kelvin

@image html files/thermal_camera/hot_object_behind_cool_object.png
