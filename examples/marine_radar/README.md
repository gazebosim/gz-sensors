# Marine radar

`gz::sensors::MarineRadarSensor` is built into Gazebo Sensors as the
`marine_radar` component, so unlike the [custom_sensor](../custom_sensor)
example there is no sensor library to compile here. This directory only holds
SDF: three ready-made presets and one demonstration world.

| File | Contents |
|---|---|
| `radar_short_range_100m.sdf` | 100 m preset: harbour and docking, fine range cells, 48 rpm. |
| `radar_medium_range_500m.sdf` | 500 m preset: coastal navigation and collision avoidance. |
| `radar_long_range_1000m.sdf` | 1000 m preset with the realistic imperfections (noise, missed echoes, false echoes) turned on. |
| `marine_radar_demo.sdf` | A world with a vessel carrying the radar and obstacles at known ranges and bearings. |

The three presets are `<sensor>` blocks meant to be pasted into a `<link>` of
your own model. They are also checked by the `UNIT_RadarConfig_TEST` unit
test, which parses each of them and asserts that the radar configuration
parser accepts it without errors or warnings.

## Use

### With Gazebo

SDFormat has no native `radar` sensor type, so the radar is declared as a
custom sensor:

```xml
<sensor name="marine_radar" type="custom" gz:type="marine_radar">
  <topic>/marine_radar</topic>
  <gz:marine_radar>
    <range_max>1000</range_max>
  </gz:marine_radar>
</sensor>
```

Gazebo Sim does not know about custom sensor types, so a small System plugin
has to create the sensor and keep its pose up to date, exactly as the
[custom_sensor_system](https://github.com/gazebosim/gz-sim/tree/main/examples/plugin/custom_sensor_system)
example does for the `Odometer`. The only differences are the type string it
matches and the class it instantiates:

```cpp
#include <gz/sensors/MarineRadarSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Util.hh>

// In PreUpdate, for each entity with a components::CustomSensor:
_ecm.EachNew<gz::sim::components::CustomSensor,
             gz::sim::components::ParentEntity>(
  [&](const gz::sim::Entity &_entity,
      const gz::sim::components::CustomSensor *_custom,
      const gz::sim::components::ParentEntity *_parent) -> bool
  {
    sdf::Sensor data = _custom->Data();

    // Only handle this sensor's own custom type.
    if (gz::sensors::customType(data) != "marine_radar")
      return true;

    // Give the sensor a scoped topic if the SDF did not set one.
    auto scopedName = gz::sim::scopedName(_entity, _ecm, "::", false);
    data.SetName(scopedName);
    if (data.Topic().empty())
      data.SetTopic(scopedName + "/marine_radar");

    gz::sensors::SensorFactory sensorFactory;
    auto sensor =
        sensorFactory.CreateSensor<gz::sensors::MarineRadarSensor>(data);
    if (nullptr == sensor)
      return true;

    sensor->SetParent(
        _ecm.Component<gz::sim::components::Name>(_parent->Data())->Data());
    this->entitySensorMap.insert(
        std::make_pair(_entity, std::move(sensor)));
    return true;
  });
```

Two things are specific to this sensor:

* It is a `gz::sensors::RenderingSensor`, so the System plugin has to hand it
  the rendering scene (`sensor->SetScene(scene)`, or let
  `gz::sensors::RenderingEvents::ConnectSceneChangeCallback` do it, which the
  sensor already subscribes to in `Load()`) and run on the rendering thread,
  the way the `Sensors` system does for cameras and GPU lidars. In a world
  where the `Sensors` system is already loaded, the scene-change event is
  enough.
* Its pose must be refreshed every step, because the rays are cast from the
  antenna in world coordinates:
  `sensor->SetPose(gz::sim::worldPose(_entity, _ecm))` before
  `sensor->Update(_info.simTime)`.

### With a custom program

Any program that links `gz-sensors::marine_radar` can create the sensor
directly from an `sdf::Sensor`, drive it with
`gz::sensors::Manager::RunOnce()` and feed it a scene, following the
[loop_sensor](../loop_sensor) example. The radar additionally needs a
rendering scene with the obstacles in it, because its rays are cast with
`gz::rendering::RayQuery`.

## Checking the output

```
gz topic -l
```

shows

```
/marine_radar/image
/marine_radar/returns
/marine_radar/targets
```

The PPI image can be viewed with Gazebo's Image Display GUI plugin, or
dumped with

```
gz topic -e -t /marine_radar/targets
```

The maximum range can be changed while the simulation runs:

```
gz topic -t /marine_radar/cmd/range_max -m gz.msgs.Double -p 'data: 500'
```
