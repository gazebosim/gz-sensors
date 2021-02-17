# Loop sensor

Example executable that loads a sensor from an SDF file and updates it in a
loop until the user closes the program.

This example can't load rendering sensors.

## Build

Compile as follows:

```
cd examples/loop_sensor
mkdir build
cd build
cmake ..
make
```

This will generate an executable called `loop_sensor`.

## Load an official sensor

Try for example loading the accompanying `altimeter_sensor.sdf` file to run
an altimeter sensor, which is installed with Ignition Sensors:

```
cd examples/loop_sensor/build
./loop_sensor ../altimeter_example.sdf
```

On another terminal, check that the altimeter is generating data on a topic:

```
ign topic -l
```

You should see:

``
/altimeter
```

Then listen to the data:

```
ign topic -e -t /altimeter
```

You'll see data like:

```
...
header {
  stamp {
    sec: 12
  }
  data {
    key: "frame_id"
    value: "altimeter"
  }
  data {
    key: "seq"
    value: "12"
  }
}
vertical_position: 0.9903850972191578
vertical_velocity: 2.9159486154028573

header {
  stamp {
    sec: 13
  }
  data {
    key: "frame_id"
    value: "altimeter"
  }
  data {
    key: "seq"
    value: "13"
  }
}
vertical_position: 1.139457534900868
vertical_velocity: 3.1484160275030266
...
```

## Load a custom sensor

Let's try loading the sensor from the `custom_sensor` example.

First we need to tell Ignition Sensors where to find the custom sensor
by setting the `IGN_SENSORS_PATH` environment variable to the path where
`ignition-sensors5-custom_sensor` is located:

```
cd examples/loop_sensor/build
export IGN_SENSORS_PATH=../..//custom_sensor/build
```

Then we can run the `custom_sensor.sdf` file provided with that sensor:

```
cd examples/loop_sensor/build
./loop_sensor ../../custom_sensor/custom_sensor.sdf
```

On another terminal, check that the sensor is generating data on a topic:

```
ign topic -l
```

You should see:

``
/custom_data
```

Then listen to the data:

```
ign topic -e -t /custom_data
```

You'll see data like:

```
...
header {
  stamp {
    sec: 12
  }
  data {
    key: "frame_id"
    value: "my_sensor"
  }
  data {
    key: "seq"
    value: "12"
  }
}
data: 12.799322041404857

header {
  stamp {
    sec: 13
  }
  data {
    key: "frame_id"
    value: "my_sensor"
  }
  data {
    key: "seq"
    value: "13"
  }
}
data: 13.783310442250663
...
```



