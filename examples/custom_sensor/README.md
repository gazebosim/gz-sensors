# Custom sensor

This example creates a simple custom sensor called `DoubleSensor`, which
produces a single number as data and publishes it to a topic.

## Build

Compile the sensor as follows:

```
cd examples/custom_sensor
mkdir build
cd build
cmake ..
make
```

This will generate a shared library with the sensor called `double_sensor`.

## Use

This sensor with Ignition Gazebo, or with any downstream application that uses
the Ignition Sensors API. Listed here are two ways of testing this sensor, one
with Gazebo and one with a custom program.

### With a custom program

The [loop_sensor](../loop_sensor) example can be used to load an SDF file with
configuration for this sensor and run it in a loop. See that example's
instructions.

### With Ignition Gazebo

The
[custom_sensor_system](https://github.com/ignitionrobotics/ign-gazebo/tree/main/examples/plugin/custom_sensor_system)
example can be used to load this sensor into Gazebo and update it during the
simulation.

