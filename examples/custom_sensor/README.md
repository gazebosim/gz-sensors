# Custom sensor

This example creates a simple custom sensor called `Odometer`, which
publishes the total distance travelled by a robot.

## Build

Compile the sensor as follows:

```
cd examples/custom_sensor
mkdir build
cd build
cmake ..
make
```

This will generate a shared library with the sensor called `libodometer`.

## Use

This sensor can be used with Gazebo, or with any downstream
application that uses the Gazebo Sensors API. Listed here are two ways of
testing this sensor, one with Gazebo and one with a custom program.

### With a custom program

The [loop_sensor](../loop_sensor) example can be used to load an SDF file with
configuration for this sensor and run it in a loop. See that example's
instructions.

### With Gazebo

The
[custom_sensor_system](https://github.com/gazebosim/gz-sim/tree/main/examples/plugin/custom_sensor_system)
example can be used to load this sensor into Gazebo and update it during the
simulation.

