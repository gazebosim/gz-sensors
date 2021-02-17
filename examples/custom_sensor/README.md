# Custom sensor

This example creates a simple custom sensor that produces a single number as
data and publishes it to a topic.

## Build

Compile the sensor as follows:

```
cd examples/custom_sensor
mkdir build
cd build
cmake ..
make
```

This will generate a shared library with the sensor called `ignition-sensors5-custom_sensor`.

## Test

The "loop_sensor" example can be used to load an SDF file with configuration for
this sensor and run it in a loop. See that example's instructions.
