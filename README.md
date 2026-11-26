# Gazebo Sensors : Sensor models for simulation

**Maintainer:** iche AT intrinsic DOT ai

[![GitHub open issues](https://img.shields.io/github/issues-raw/gazebosim/gz-sensors.svg)](https://github.com/gazebosim/gz-sensors/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/gazebosim/gz-sensors.svg)](https://github.com/gazebosim/gz-sensors/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/gazebosim/gz-sensors/tree/main/graph/badge.svg)](https://codecov.io/gh/gazebosim/gz-sensors/tree/main)
GitHub: Ubuntu Noble  | [![Ubuntu CI](https://github.com/gazebosim/gz-sensors/actions/workflows/ci.yml/badge.svg)](https://github.com/gazebosim/gz-sensors/actions/workflows/ci.yml?query=branch%3Amain)
Jenkins: Ubuntu Noble | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=gz_sensors-ci-main-noble-amd64)](https://build.osrfoundation.org/job/gz_sensors-ci-main-noble-amd64)
Jenkins: Homebrew     | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=gz_sensors-ci-main-homebrew-amd64)](https://build.osrfoundation.org/job/gz_sensors-ci-main-homebrew-amd64)
Jenkins: Windows      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=gz_sensors-main-cnlwin)](https://build.osrfoundation.org/job/gz_sensors-main-cnlwin/)

Gazebo Sensors, a component of [Gazebo](https://gazebosim.org),
provides numerous sensor models
designed to generate realistic data from simulation environments. Gazebo Sensors is used in conjunction with [Gazebo Libraries](https://gazebosim.org/libs), and especially relies on the rendering capabilities from [Gazebo Rendering](https://gazebosim.org/libs/rendering) and physics simulation from [Gazebo Physics](https://gazebosim.org/libs/physics).

# Table of Contents

[Features](#features)

[Install](#install)

[Usage](#usage)

[Folder Structure](#folder-structure)

[Code of Conduct](#code-of-conduct)

[Contributing](#contributing)

[Versioning](#versioning)

[License](#license)

# Features

Gazebo Sensors provides a set of sensors models that can be
configured at run time to mimic specific real-world sensors. A noise model
is also provided that can be used to introduce Gaussian or custom noise
models into sensor streams.

## Supported Sensors

| **Sensor Name**           | **API Link**                                                                                                 | **SDF Element**                                                                                      | **Notes**                                          |
|---------------------------|-------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------|----------------------------------------------------|
| **Air Pressure Sensor**   | [AirPressureSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1AirPressureSensor.html)        | [`<air_pressure>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_air_pressure)                | Measures atmospheric pressure                      |
| **Air Speed Sensor**      | [AirSpeedSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1AirSpeedSensor.html)              | [`<air_speed>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_air_speed)                                                                                  | Measures the speed of air relative to the sensor    |
| **Altimeter**             | [AltimeterSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1AltimeterSensor.html)            | [`<altimeter>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_altimeter)                      | Measures altitude above a reference point          |
| **Bounding Box Camera**   | [BoundingBoxCameraSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1BoundingBoxCameraSensor.html) | [`<camera>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_camera)                         | Captures images with bounding box annotations      |
| **Camera**                | [CameraSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1CameraSensor.html)                  | [`<camera>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_camera)                           | Captures standard RGB images                       |
| **Depth Camera**          | [DepthCameraSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1DepthCameraSensor.html)        | [`<depth_camera>`](http://sdformat.org/spec?ver=1.12&elem=sensor#camera_depth_camera)              | Captures depth information                         |
| **Doppler Velocity Log**  | [DopplerVelocityLog](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1DopplerVelocityLog.html)      | N/A                                | Measures relative velocity of an underwater vehicle |
| **Force-Torque**          | [ForceTorqueSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1ForceTorqueSensor.html)        | [`<force_torque>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_force_torque)              | Measures forces and torques on a joint             |
| **GPU Lidar**             | [GpuLidarSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1GpuLidarSensor.html)              | [`<lidar>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_lidar)                            | Simulates a 3D laser scanner using GPU acceleration|
| **IMU**                   | [ImuSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1ImuSensor.html)                        | [`<imu>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_imu)                                | Measures linear acceleration and angular velocity  |
| **Logical Camera**        | [LogicalCameraSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1LogicalCameraSensor.html)    | [`<logical_camera>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_logical_camera)          | Detects models within a specified volume           |
| **Magnetometer**          | [MagnetometerSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1MagnetometerSensor.html)      | [`<magnetometer>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_magnetometer)              | Measures magnetic field strength and direction     |
| **Marine Radar**          | [MarineRadarSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1MarineRadarSensor.html)  | [`<sensor type="custom" gz:type="marine_radar">`](#marine-radar-sensor)  | Rotating 2-D marine navigation radar: PPI image and target tracks |
| **NavSat (GPS)**          | [NavSatSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1NavSatSensor.html)                  | [`<navsat>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_navsat)                          | Simulates GPS-like positioning sensor              |
| **RGBD Camera**           | [RGBDCameraSensor](https://gazebosim.org/api/sensors/7/classgz_1_1sensors_1_1RgbdCameraSensor.html)          | [`<camera>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_camera) | Captures RGB + Depth streams                       |
| **Segmentation Camera**   | [SegmentationCameraSensor](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1SegmentationCameraSensor.html) | [`<camera>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_camera)              | Captures labeled segmentation images               |
| **Thermal Camera**        | [ThermalCameraSensor](https://gazebosim.org/api/sensors/7/classgz_1_1sensors_1_1ThermalCameraSensor.html)    | [`<camera>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_camera)                         | Detects heat signature                             |
| **Wide Angle Camera**     | [WideAngleCameraSensor](https://gazebosim.org/api/sensors/8/classgz_1_1sensors_1_1WideAngleCameraSensor.html)| [`<camera>`](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_camera)                         | Captures wide field of view                        |


## Marine Radar Sensor

`gz::sensors::MarineRadarSensor` (the `marine_radar` component) simulates a
rotating 2-D marine X/S band navigation radar. The antenna rotates clockwise
seen from above and transmits one *spoke* per angular step; every spoke is
sampled with a fan of gravity-levelled rays, one per entry of
`<slice_heights>`, cast with `gz::rendering::RayQuery` against the rendering
scene. The geometric hits are turned into radar video by a simplified radar
equation (SNR proportional to |cos incidence| / R<sup>4</sup> with a Gaussian
two-way beam pattern), a detection threshold, range quantisation and the
configured noise, missed-echo and false-echo statistics. Echo cells are
clustered into targets with persistent IDs, and scan-converted into a Plan
Position Indicator (PPI) image. The image and the detections come from the
same echo video, so they are always consistent with each other.

SDFormat has no native `radar` sensor type, so the radar is declared as a
[custom sensor](https://gazebosim.org/api/sensors/9/custom_sensors.html):

```xml
<sensor name="marine_radar" type="custom" gz:type="marine_radar">
  <always_on>1</always_on>
  <update_rate>0</update_rate>
  <topic>/marine_radar</topic>
  <frame_id>radar_link</frame_id>
  <gz:marine_radar>
    <range_min>10</range_min>
    <range_max>1000</range_max>
    <range_resolution>2</range_resolution>
    <horizontal_fov_deg>360</horizontal_fov_deg>
    <samples>4096</samples>
    <rotation_rate_rpm>24</rotation_rate_rpm>
    <beamwidth_horizontal_deg>1.2</beamwidth_horizontal_deg>
    <beamwidth_vertical_deg>20</beamwidth_vertical_deg>
    <slice_heights>0.5 1.5 3 6 12 24</slice_heights>
    <range_noise>
      <noise type="gaussian">
        <stddev>0.5</stddev>
      </noise>
    </range_noise>
    <image_size>512</image_size>
    <image_rate>5</image_rate>
  </gz:marine_radar>
</sensor>
```

`<update_rate>0</update_rate>` updates the radar every simulation step. The
antenna schedule lives inside the sensor, so the update rate only has to be
fast enough to keep up with the spoke rate
(`samples` x `rotation_rate_rpm` / 60 spokes per second).

Ready-made presets for 100 m, 500 m and 1000 m, a demonstration world, and
the System plugin glue a consuming application needs are in
[examples/marine_radar](examples/marine_radar).

### Topics

| Topic | Type | Rate | Content |
|---|---|---|---|
| `<topic>/image` | `gz.msgs.Image` | `image_rate` | PPI display, `RGB_INT8` |
| `<topic>/targets` | `gz.msgs.PointCloudPacked` | once per revolution | one point per extracted target |
| `<topic>/returns` | `gz.msgs.PointCloudPacked` | once per revolution | one point per echo cell |
| `<topic>/cmd/range_max` | `gz.msgs.Double` | input | change `range_max` at run time |

The target cloud carries, per point, `world_x`/`world_y`/`world_z`
(`FLOAT64`), `x`/`y`/`z`/`range`/`bearing`/`elevation`/`intensity`
(`FLOAT32`), `id`/`age`/`num_cells` (`UINT32`) and
`min_range`/`range_extent`/`bearing_extent`/`time_offset` (`FLOAT32`). The
returns cloud carries the same world and sensor-frame positions plus
`intensity`, `range`, `bearing`, `elevation` and `time_offset`. Bearing is
the REP-103 azimuth in the sensor frame (counter-clockwise from +X, in
(-pi, pi]); range is the slant range from the antenna. Header stamps are the
simulation time at which the revolution completed, and each point's
`time_offset` is its own observation time relative to that stamp.

### Parameters

Angles are given in degrees (keys ending in `_deg`), distances in metres and
times in seconds. Every value is validated when the sensor loads: malformed
numbers, out-of-range values and inconsistent combinations are logged one per
line and the sensor is **not** created, rather than running with a wrong
configuration. Unknown elements produce a warning and are ignored, and an
angular resolution that does not divide the field of view is adjusted to the
nearest exact spoke spacing with a warning.

Range:

| Key | Default | Valid | Description |
|---|---|---|---|
| `range_min` | 10 | >= 0 | Blind range [m]. Rays start here, so own-ship structure inside it is never seen. |
| `range_max` | 1000 | > `range_min`, <= 200000 | Instrumented range [m]. Can be changed at run time. |
| `range_resolution` | 2 | > 0 | Range cell size [m] (pulse length). Cells = (max - min) / resolution, at most 16384. |

Antenna and scan:

| Key | Default | Valid | Description |
|---|---|---|---|
| `horizontal_fov_deg` | 360 | (0, 360] | Transmitting sector centred on the bow (+X). Less than 360 means sector blanking. |
| `samples` | 4096 | 1-16384 | Spokes per sweep of the field of view. |
| `angular_resolution_deg` | 360/4096 | > 0 | Spoke spacing. Alternative to `samples`; giving both requires them to agree. |
| `rotation_rate_rpm` | 24 | (0, 600] | Antenna speed. One detection message per revolution, i.e. rpm / 60 Hz. |
| `beamwidth_horizontal_deg` | 1.2 | (0, 45] | Two-way -3 dB horizontal beam width (azimuth smearing). |
| `beamwidth_vertical_deg` | 20 | (0, 90] | Two-way -3 dB vertical beam width. |
| `slice_heights` | `0.5 1.5 3 6 12 24` | 1-64 values in [0, 1000] | Heights above `sea_level` [m] of the gravity-levelled rays cast along each spoke, one ray per slice per spoke. |

Echo and detection model:

| Key | Default | Valid | Description |
|---|---|---|---|
| `snr_reference_db` | 30 | finite | SNR of a normal-incidence hit at `reference_range` on the beam axis. |
| `reference_range` | 1000 | > 0 | [m] |
| `detection_threshold_db` | 8 | finite | Cells below this SNR are dropped. With the R<sup>-4</sup> law this sets a physical detection range independent of `range_max`. |
| `intensity_noise_stddev_db` | 1.5 | >= 0 | Per-hit SNR fluctuation (detection noise). |
| `range_noise_stddev` | 0 | >= 0 | Per-echo Gaussian range noise [m]. |
| `bearing_noise_stddev_deg` | 0 | >= 0 | Gaussian noise added to each reported target bearing. |
| `echo_detection_probability` | 1.0 | [0, 1] | Probability each echo segment of a spoke is received (missed detections). |
| `false_alarm_rate` | 0 | [0, 10000] | Expected false echoes per scan (Poisson, uniform in the polar grid). |
| `sea_level_filter` | true | bool | Ignore hits below `sea_level`. |
| `sea_level` | 0 | finite | World z of the sea surface [m]; also the reference of `slice_heights`. |

The three noise standard deviations can also be given as standard SDFormat
`<noise>` elements nested in `<range_noise>`, `<bearing_noise>` and
`<intensity_noise>`, the way `//altimeter/vertical_position` does. Only
`gaussian` is supported, because these values parameterise the radar echo
model rather than being applied to a single published sample, so
`gz::sensors::NoiseFactory` does not apply to them.

Target extraction and identification:

| Key | Default | Valid | Description |
|---|---|---|---|
| `min_target_cells` | 1 | >= 1 | Clusters with fewer cells are discarded. |
| `cluster_spoke_gap` | 1 | 0-16 | Empty spokes bridged when grouping cells into one target. |
| `cluster_bin_gap` | 2 | 0-64 | Empty range cells bridged when grouping. |
| `association_gate` | 25 | >= 0 | [m] Max world-frame distance for a target to keep its ID from the previous scan. |
| `track_coast_scans` | 2 | 0-100 | Scans an ID survives without a match. |

PPI image:

| Key | Default | Valid | Description |
|---|---|---|---|
| `image_size` | 512 | 64-4096 | Width = height [px]. |
| `image_rate` | 5 | [0, 100] | Image publish rate [Hz] in simulation time; 0 disables the image. |
| `display_range` | 0 | >= 0 | Radius shown [m]; 0 follows `range_max`. |
| `range_rings` | 4 | 0-50 | Equally spaced rings out to the display range. |
| `bearing_line_spacing_deg` | 30 | [0, 360] | 0 disables the bearing lines. |
| `display_mode` | `head_up` | `head_up` / `north_up` | Head-up: bow up. North-up: world +Y up (ENU), stabilised. |
| `afterglow` | 0.6 | [0, 1] | Brightness lost over one revolution (phosphor-like trail). |
| `echo_expansion` | true | bool | Grow echoes by one pixel, like the EXP key on marine radars. |
| `display_dynamic_range_db` | 50 | [1, 150] | SNR span above the threshold mapped onto the colour scale. |
| `draw_target_markers` | true | bool | Boxes on the targets of the last completed scan. |

Sensor-level parameters, handled by `MarineRadarSensor` itself:

| Key | Default | Valid | Description |
|---|---|---|---|
| `publish_image` | true | bool | Publish the PPI image. |
| `publish_returns` | true | bool | Publish the per-cell returns cloud. |
| `normal_estimation` | `probe` | `probe` / `none` | `probe` casts two extra rays per hit to recover the surface normal, and therefore the incidence angle of the echo, because the render engine's ray query does not report normals. `none` assumes normal incidence and uses a third of the rays. |
| `normal_probe_angle_deg` | 0.05 | (0, 5] | Angular offset of those probe rays. |
| `seed` | 42 | [0, 2<sup>32</sup>-1] | Random seed. Runs with the same scene, configuration and seed are reproducible. |

### Limitations

* **Ray sampling, not wave physics.** Objects narrower than the ray spacing
  (`range` x 360 / `samples`; 1.5 m at 1000 m with 4096 samples) can fall
  between rays and be missed or flicker.
* **Height slices.** Objects that exist only between two slice heights are
  missed. Occlusion is evaluated along the levelled slices, not the slanted
  line of sight from the antenna, so a low obstacle shadows taller objects
  behind it slightly more than a real radar would. The slices are
  gravity-levelled, but vessel roll and pitch still change the vertical gain
  through each hit's true elevation.
* **Simplified radar equation.** There is no radar cross-section per
  material, multipath, ducting, rain or sea clutter, STC/FTC, side lobes,
  pulse compression or Doppler. Noise, missed echoes and false echoes are
  statistical add-ons, not physical.
* **First surface only, visual geometry only.** A ray reports the nearest
  intersected *visual*, so visual-only models (such as a water surface) are
  seen and collision-only models are not. There are no
  multiple-reflection ghosts.
* **Approximate incidence angles.** `gz::rendering::RayQuery` reports a hit
  point but no surface normal, so the incidence angle is estimated from two
  probe rays per hit. On an edge, or where the probes land on a different
  object, the echo falls back to normal incidence.
* **Target extraction** is single-scan connected-component clustering with
  nearest-neighbour ID association. There is no Kalman tracking, velocity
  estimation (ARPA/CPA) or classification. Objects closer together than about
  one beam width or range cell merge into one target, as in reality.
* **Cost.** Rays per second = `samples` x `len(slice_heights)` x `rpm` / 60
  (9830 by default, tripled by `normal_estimation` `probe`), each a CPU ray
  query against the scene. Reduce it with fewer slices, fewer `samples`, a
  lower `rotation_rate_rpm` or `normal_estimation` `none`.
* `elevation` and `z` are simulation conveniences; a real 2-D marine radar
  does not measure elevation. `world_*` positions use the simulator's true
  radar pose.

# Install

See the [installation tutorial](https://gazebosim.org/api/sensors/9/installation.html).

# Usage

Please refer to the [examples directory](https://github.com/gazebosim/gz-sensors/tree/main/examples).

A list of sensors and SDF examples can be found in the [SDF specification](http://sdformat.org/spec?ver=1.12&elem=sensor)

# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
├── examples                  Example programs.
├── include/gz/sensors        Header files that will be installed.
├── src                       Source files and unit tests.
├── test
│    ├── integration          Integration tests.
│    ├── performance          Performance tests.
│    └── regression           Regression tests.
├── tutorials                 Tutorials, written in markdown.
├── Changelog.md              Changelog.
├── CMakeLists.txt            CMake build script.
└── README.md                 This readme.
```

# Contributing

Please see the [contribution guide](https://gazebosim.org/docs/all/contributing).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/gazebosim/gz-sim/blob/main/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Gazebo project](https://gazebosim.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Gazebo website](https://gazebosim.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/gazebosim/gz-sensors/blob/main/LICENSE) file.
