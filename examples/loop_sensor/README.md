# Loop sensor

Example executable that loads sensors from an SDF world file and updates them in
a loop until the user closes the program.

The example works with 2 types of sensors:

* A built-in type: altimeter
* A custom type: "double sensor"

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

It will also compile the `custom_sensor` example, creating a library called
`libdouble_sensor`.

## Run

Try loading the accompanying `sensors.sdf`:

```
cd examples/loop_sensor/build
./loop_sensor ../sensors.sdf
```

On another terminal, check that the altimeter and double sensor are generating
data on a topic:

```
ign topic -l
```

You should see:

```
/altimeter
/double
```

Then listen to the altimeter data:

```
ign topic -e -t /altimeter
```

You'll see data like:

```
...
header {
  stamp {
    sec: 35
  }
  data {
    key: "frame_id"
    value: "altimeter"
  }
  data {
    key: "seq"
    value: "35"
  }
}
vertical_position: 0.91297697595395233
vertical_velocity: 10.596832320931542
...
```

Then listen to the custom sensor:

```
ign topic -e -t /double
```

You'll see data like:

```
...
header {
  stamp {
    sec: 14
  }
  data {
    key: "frame_id"
    value: "custom_double"
  }
  data {
    key: "seq"
    value: "14"
  }
}
data: 3.1325939574820185
...
```

