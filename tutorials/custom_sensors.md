\page custom_sensors Custom sensors

Ignition Sensors comes with various built-in sensor types ready to be used.
Users aren't limited to those sensor types though. This tutorial will go over
the process of implementing a custom sensor that leverages Ignition Sensors
and can be used with downstream applications such as Ignition Gazebo.

## Why custom sensors

The set of built-in sensors provided by Ignition Sensors is tied to the
SDFormat specification. SDFormat is meant to be as general-purpose as possible,
so adding lots of exotic sensors to the specification may become a maintenance
burden and counter-productive. Therefore, sensors that are very specific to
certain applications are better implemented closer to the applications that
need them, than added to the general-purpose specification.

It may also be interesting for downstream projects to implement very specific
versions of sensors which are not generally useful for other projects.

Finally, another reason to develop custom sensors is for projects that need to
implement proprietary sensors that can't be shared publicly.

## Do I need to use Ignition Sensors?

A simulated sensor consists of code that can be used to extract specific data
from a running simulation. A camera extracts pixels, an IMU extracts
accelerations, etc. This doesn't necessarily need to be done though Ignition
Sensors. On Ignition Gazebo, for example, it's implement functionality similar
to a sensor just using a system plugin without any connection to Ignition
Sensors.

With that in mind, it's worth going over some of the features that Ignition
Sensors provides that make it easier to implement sensors, instead of writing
everything from scratch:

* Standard handling of common parameters, like topic and update rate;
* Common APIs for updating the sensors in a loop;
* A standard way for loading custom sensors from an SDF `<sensor>` tag;
* Efficient updates for various rendering sensors using the same scene.

## Let's get started!

Let's go over the creation of a custom
[odometer](https://en.wikipedia.org/wiki/Odometer) sensor which will measure
the total distance travelled by a robot.

This example is located at
[ign-sensors/examples/custom_sensor](https://github.com/ignitionrobotics/ign-sensors/tree/main/examples/custom_sensor),
and the integration with Ignition Gazebo is at
[ign-gazebo/examples/plugins/custom_sensor_system](https://github.com/ignitionrobotics/ign-gazebo/tree/main/examples/plugin/custom_sensor_system).

Wherever "odometer" is used, that can be substituted for any other custom sensor
type. Seismometer? Breathalyzer? Just adapt the logic to your needs.

### SDF

Custom sensors follow these rules to be loaded from SDFormat:

* Use `type="custom"` inside the `<sensor>` tag
* Add an extra `ignition:type="odometer"` attribute, where `odometer`
  is a string that uniquely identifies the sensor type.
* Optionally, add an `<ignition:odometer/>` element with configuration
  specific to that sensor.

With that in mind, here's what the sensor tag would look like:

```xml
<sensor name="the_odometer" type="custom" ignition:type="odometer">
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <topic>odom</topic>
  <ignition:odometer>
    <noise type="gaussian">
      <mean>0.2</mean>
      <stddev>0.1</stddev>
    </noise>
  </ignition:odometer>
</sensor>
```

### Sensor implementation

The sensor consists of a class, `Odometer`, which inherits from
`ignition::sensors::Sensor`.

Take a look at
[ign-sensors/examples/custom_sensor](https://github.com/ignitionrobotics/ign-sensors/tree/main/examples/custom_sensor)
for the full code. Here are some important pointers:

* Be sure to link your sensor to Ignition Sensors
* Implement all pure abstraction functions, like `Update`
* Be sure to implement at least one of the `Load` functions so the sensor can be initialized
* Don't forget to call `Sensor::Load` to load common sensor properties
* It's recommended that the sensor publishes its data using Ignition Transport

You may choose to compile your sensor as a shared library, or to embed it
directly into other application code.

### Loading into Ignition Gazebo

In order to use the sensor on Ignition Gazebo, one needs to write a system
plugin that instantiates the sensor and updates it periodically with data from
simulation.

Take a look at
[ign-gazebo/examples/plugins/custom_sensor_system](https://github.com/ignitionrobotics/ign-gazebo/tree/main/examples/plugin/custom_sensor_system).
for the full code. Here are some important pointers:

* Check for new entities that have `ignition::gazebo::components::CustomSensor`
  during the `PreUpdate` callback and instantiate new sensors as they appear
  in simulation.
* Don't assume all `CustomSensors` are of the type you need, be sure to check
  the type using the `ignition::sensors::customType` function.
* During `PostUpdate`, update all sensors with new data coming from simulation.
* Also during `PostUpdate`, delete any sensors that have been removed from
  simulation.

