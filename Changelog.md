## Gazebo Sensors 7

### Gazebo Sensors 7.X.X

### Gazebo Sensors 7.3.0 (2023-09-26)

1. Infrastructure
    * [Pull request #372](https://github.com/gazebosim/gz-sensors/pull/372)
    * [Pull request #371](https://github.com/gazebosim/gz-sensors/pull/371)

1. Expose optical frame in CameraSensor so it can be set in DepthCameraSensor
    * [Pull request #362](https://github.com/gazebosim/gz-sensors/pull/362)

1. Fix CameraSensor to check if element is null before access
    * [Pull request #361](https://github.com/gazebosim/gz-sensors/pull/361)

1. Support protobuf >= 22
    * [Pull request #351](https://github.com/gazebosim/gz-sensors/pull/351)

1. Minor cleanup - lint, typos
    * [Pull request #352](https://github.com/gazebosim/gz-sensors/pull/352)
    * [Pull request #344](https://github.com/gazebosim/gz-sensors/pull/344)

1. Fix frame_id for depth camera point cloud
    * [Pull request #350](https://github.com/gazebosim/gz-sensors/pull/350)

1. Add support for bayer images to camera sensor
    * [Pull request #336](https://github.com/gazebosim/gz-sensors/pull/336)

1. Fix flaky trigger camera test
    * [Pull request #346](https://github.com/gazebosim/gz-sensors/pull/346)

1. Generate default trigger topic name if empty
    * [Pull request #343](https://github.com/gazebosim/gz-sensors/pull/343)


### Gazebo Sensors 7.2.0 (2023-04-13)

1. Cleanup resources in CameraSensor destructor
    * [Pull request #340](https://github.com/gazebosim/gz-sensors/pull/340)

1. CI workflow: use checkout v3
    * [Pull request #335](https://github.com/gazebosim/gz-sensors/pull/335)

1. Rename COPYING to LICENSE
    * [Pull request #334](https://github.com/gazebosim/gz-sensors/pull/334)

1. Fix links in Changelog
    * [Pull request #330](https://github.com/gazebosim/gz-sensors/pull/330)

1. Fix flaky triggered bounding box camera test
    * [Pull request #329](https://github.com/gazebosim/gz-sensors/pull/329)

1. Fix Camera info test
    * [Pull request #326](https://github.com/gazebosim/gz-sensors/pull/326)

1. Added trigger to BoundingBoxCamera
    * [Pull request #322](https://github.com/gazebosim/gz-sensors/pull/322)

1. clean up rendering resources
    * [Pull request #324](https://github.com/gazebosim/gz-sensors/pull/324)

1. Added Camera Info topic support for cameras
    * [Pull request #285](https://github.com/gazebosim/gz-sensors/pull/285)

1. ign -> gz Migrate Ignition Headers : gz-sensors
    * [Pull request #260](https://github.com/gazebosim/gz-sensors/pull/260)

### Gazebo Sensors 7.1.0 (2023-02-09)

1. Added airspeed sensor
    * [Pull request #305](https://github.com/gazebosim/gz-sensors/pull/305)

1. Add HasInfoConnections() method to expose infoPub.HasConnections() to child of CameraSensor class
    * [Pull request #310](https://github.com/gazebosim/gz-sensors/pull/310)

1. CameraInfo is now published when there's a CameraSensor subscriber
    * [Pull request #308](https://github.com/gazebosim/gz-sensors/pull/308)

1. Disable ogre tests on windows
    * [Pull request #303](https://github.com/gazebosim/gz-sensors/pull/303)

1. Add and Revert DopplerVelocityLog sensor
    * [Pull request #302](https://github.com/gazebosim/gz-sensors/pull/302)
    * [Pull request #290](https://github.com/gazebosim/gz-sensors/pull/290)

1. Set custom projection matrix based on intrinsics params from SDF
    * [Pull request #293](https://github.com/gazebosim/gz-sensors/pull/293)

1. Renamed `ignition_frame_id` to `gz_frame_id` and fix warning
    * [Pull request #296](https://github.com/gazebosim/gz-sensors/pull/296)

1. Fix navsat frame id
    * [Pull request #295](https://github.com/gazebosim/gz-sensors/pull/295)

1. Fix compile time warning
    * [Pull request #297](https://github.com/gazebosim/gz-sensors/pull/297)

1. Update Camera Intrinsics in camera_info topic
    * [Pull request #281](https://github.com/gazebosim/gz-sensors/pull/281)

1. RgbdCameraSensor.cc: fix include
    * [Pull request #280](https://github.com/gazebosim/gz-sensors/pull/280)

1. Camera: configure projection matrix from SDFormat
    * [Pull request #249](https://github.com/gazebosim/gz-sensors/pull/249)

1. Improved noise coverage
    * [Pull request #278](https://github.com/gazebosim/gz-sensors/pull/278)

1. Improved coverage Lidar
    * [Pull request #277](https://github.com/gazebosim/gz-sensors/pull/277)

1. Add support for 16 bit image format
    * [Pull request #276](https://github.com/gazebosim/gz-sensors/pull/276)

1. Add optional optical frame id to camera sensors
    * [Pull request #259](https://github.com/gazebosim/gz-sensors/pull/259)

1. Add missing DEPENDS_ON_COMPONENTS parameters
    * [Pull request #262](https://github.com/gazebosim/gz-sensors/pull/262)

1. Remove redundant namespace references
    * [Pull request #258](https://github.com/gazebosim/gz-sensors/pull/258)


### Gazebo Sensors 7.0.0 (2022-09-27)

1. Clear scene before exiting wide_angle_camera_test
    * [Pull request #266](https://github.com/gazebosim/ign-sensors/pull/266)

1. Fixed INTEGRATION triggered_camera test
    * [Pull request #272](https://github.com/gazebosim/ign-sensors/pull/272)

1. Update README.md
    * [Pull request #267](https://github.com/gazebosim/ign-sensors/pull/267)

1. Tutorial and documentation updates
    * [Pull request #270](https://github.com/gazebosim/ign-sensors/pull/270)
    * [Pull request #269](https://github.com/gazebosim/ign-sensors/pull/269)
    * [Pull request #268](https://github.com/gazebosim/ign-sensors/pull/268)
    * [Pull request #271](https://github.com/gazebosim/ign-sensors/pull/271)

1. Fix msgs.hh uses
    * [Pull request #263](https://github.com/gazebosim/ign-sensors/pull/263)

1. Fix forward-port: use gz_add_component
    * [Pull request #253](https://github.com/gazebosim/ign-sensors/pull/253)

1. Add missing msgs include statements
    * [Pull request #242](https://github.com/gazebosim/ign-sensors/pull/242)

1. ign -> gz
    * [Pull request #245](https://github.com/gazebosim/ign-sensors/pull/245)
    * [Pull request #244](https://github.com/gazebosim/ign-sensors/pull/244)
    * [Pull request #235](https://github.com/gazebosim/ign-sensors/pull/235)
    * [Pull request #240](https://github.com/gazebosim/ign-sensors/pull/240)
    * [Pull request #232](https://github.com/gazebosim/ign-sensors/pull/232)
    * [Pull request #237](https://github.com/gazebosim/ign-sensors/pull/237)
    * [Pull request #226](https://github.com/gazebosim/ign-sensors/pull/226)
    * [Pull request #221](https://github.com/gazebosim/ign-sensors/pull/221)
    * [Pull request #233](https://github.com/gazebosim/ign-sensors/pull/233)

1. Make sensors' HasConnections function virtual
    * [Pull request #234](https://github.com/gazebosim/ign-sensors/pull/234)

1. Disable thermal camera test on MacOS
    * [Pull request #243](https://github.com/gazebosim/ign-sensors/pull/243)

1. Update GoogleTest to latest version
    * [Pull request #241](https://github.com/gazebosim/ign-sensors/pull/241)

1. [garden] Use anti-aliasing value from <anti_aliasing> SDF element
    * [Pull request #210](https://github.com/gazebosim/ign-sensors/pull/210)

1. Use pose multiplication instead of subtraction
    * [Pull request #231](https://github.com/gazebosim/ign-sensors/pull/231)

1. Add support for single channel 16 bit grayscale image format
    * [Pull request #229](https://github.com/gazebosim/ign-sensors/pull/229)

1. Remove deprecated common/PluginMacros.hh include
    * [Pull request #230](https://github.com/gazebosim/ign-sensors/pull/230)

1. Bumps in garden : ign-sensors7
    * [Pull request #216](https://github.com/gazebosim/ign-sensors/pull/216)
    * [Pull request #168](https://github.com/gazebosim/ign-sensors/pull/168)
    * [Pull request #183](https://github.com/gazebosim/ign-sensors/pull/183)
    * [Pull request #173](https://github.com/gazebosim/ign-sensors/pull/173)

1. Remove Bionic from future releases (Garden+)
    * [Pull request #205](https://github.com/gazebosim/ign-sensors/pull/205)

1. Add API to set next sensor update time
    * [Pull request #196](https://github.com/gazebosim/ign-sensors/pull/196)

1. Change to use SuppressWarning from ign-utils
    * [Pull request #193](https://github.com/gazebosim/ign-sensors/pull/193)

1. Let derived sensors call Sensor::Update
    * [Pull request #187](https://github.com/gazebosim/ign-sensors/pull/187)

1. Add wide angle camera sensor
    * [Pull request #171](https://github.com/gazebosim/ign-sensors/pull/171)

## Gazebo Sensors 6

### Gazebo Sensors 6.8.0 (2024-01-12)

1. Allow specifying gz_frame_id as an alternative to ignition_frame_id
    * [Pull request #409](https://github.com/gazebosim/gz-sensors/pull/409)

1. [backport fortress] camera info topic published with the right data
    * [Pull request #383](https://github.com/gazebosim/gz-sensors/pull/383)

1. Infrastructure
    * [Pull request #401](https://github.com/gazebosim/gz-sensors/pull/401)

### Gazebo Sensors 6.7.1 (2023-09-01)

1. Support protobuf >= 22
    * [Pull request #351](https://github.com/gazebosim/gz-sensors/pull/351)

1. Infrastructure
    * [Pull request #335](https://github.com/gazebosim/gz-sensors/pull/335)

1. Rename COPYING to LICENSE
    * [Pull request #334](https://github.com/gazebosim/gz-sensors/pull/334)

1. Fix links in Changelog
    * [Pull request #330](https://github.com/gazebosim/gz-sensors/pull/330)

1. Fix Camera info test
    * [Pull request #326](https://github.com/gazebosim/gz-sensors/pull/326)

1. clean up rendering resources
    * [Pull request #324](https://github.com/gazebosim/gz-sensors/pull/324)

1. Added Camera Info topic support for cameras
    * [Pull request #285](https://github.com/gazebosim/gz-sensors/pull/285)

### Gazebo Sensors 6.7.0 (2023-02-13)

1. Disable thermal camera test on MacOS.
    * [Pull request #243](https://github.com/gazebosim/gz-sensors/pull/243)

1. Add optional optical frame id to camera sensors.
    * [Pull request #259](https://github.com/gazebosim/gz-sensors/pull/259)

1. Add support for 16 bit image format.
    * [Pull request #276](https://github.com/gazebosim/gz-sensors/pull/276)

1. Fix navsat frame id.
    * [Pull request #298](https://github.com/gazebosim/gz-sensors/pull/298)

1. CameraInfo is now published when there's a CameraSensor subscriber.
    * [Pull request #308](https://github.com/gazebosim/gz-sensors/pull/308)

1. Add HasInfoConnections() method to expose infoPub.HasConnections() to
   child of CameraSensor class.
    * [Pull request #310](https://github.com/gazebosim/gz-sensors/pull/310)

1. Forward port 3.5.0.

### Gazebo Sensors 6.6.0 (2022-06-17)

1. Add BoundingBox Sensor
    * [Pull request #136](https://github.com/gazebosim/gz-sensors/pull/136)

### Gazebo Sensors 6.5.0 (2022-05-24)

1. Add HasConnections function
    * [Pull request #222](https://github.com/gazebosim/gz-sensors/pull/222)

### Gazebo Sensors 6.4.0 (2022-05-13)

1. Set lidar visibility mask
    * [Pull request #224](https://github.com/gazebosim/gz-sensors/pull/224)

1. Fix triggered camera test
    * [Pull request #215](https://github.com/gazebosim/gz-sensors/pull/215)

1. Add support for l8 format
    * [Pull request #220](https://github.com/gazebosim/gz-sensors/pull/220)

1. Fix `<ignition_frame_id>` not working for GpuLidarSensor
    * [Pull request #218](https://github.com/gazebosim/gz-sensors/pull/218)

### Gazebo Sensors 6.3.0 (2022-04-04)

1. IMU custom_rpy parent_frame should be set to 'world'
    * [Pull request #212](https://github.com/gazebosim/gz-sensors/pull/212)

1. Triggered Camera
    * [Pull request #194](https://github.com/gazebosim/gz-sensors/pull/194)

1. Check if noise or distortion render pass is null
    * [Pull request #211](https://github.com/gazebosim/gz-sensors/pull/211)

### Gazebo Sensors 6.2.0 (2022-03-29)

1. Distortion Camera Sensor
    * [Pull request #192](https://github.com/gazebosim/gz-sensors/pull/192)

1. Add Ubuntu Jammy CI
    * [Pull request #206](https://github.com/gazebosim/gz-sensors/pull/206)

1. Add function for enabling / disabling a sensor
    * [Pull request #204](https://github.com/gazebosim/gz-sensors/pull/204)

1. IMU sensor API to get world ref frame and heading offset
    * [Pull request #186](https://github.com/gazebosim/gz-sensors/pull/186)

1. Use pose multiplication instead of addition
    * [Pull request #199](https://github.com/gazebosim/gz-sensors/pull/199)

1. Enable cpplint check in github actions CI
    * [Pull request #198](https://github.com/gazebosim/gz-sensors/pull/198)

1. Conform to ros format for header field frame_id of sensor msgs
    * [Pull request #195](https://github.com/gazebosim/gz-sensors/pull/195)

1. Fix compiler warnings (CMP0072 and copy elision)
    * [Pull request #188](https://github.com/gazebosim/gz-sensors/pull/188)

### Gazebo Sensors 6.1.0 (2022-01-04)

1. Add NavSat (GPS) sensor
    * [Pull request #177](https://github.com/gazebosim/gz-sensors/pull/177)

1. Added Logic to flag pointcloud as not dense if invalid point is detected
    * [Pull request #180](https://github.com/gazebosim/gz-sensors/pull/180)

1. IMU ``custom_rpy``  tag parsing added
    * [Pull request #178](https://github.com/gazebosim/gz-sensors/pull/178)

### Gazebo Sensors 6.0.1 (2021-11-12)

1. Disable GPU lidar tests on macOS
    * [Pull request #163](https://github.com/gazebosim/gz-sensors/pull/163)

1. Added macOS install instructions.
    * [Pull request #162](https://github.com/gazebosim/gz-sensors/pull/162)

1. Destroy rendering sensors when sensor is removed.
    * [Pull request #169](https://github.com/gazebosim/gz-sensors/pull/169)

### Gazebo Sensors 6.0.0 (2021-09-30)

1. Trivial tutorial typo correction in Custom Sensors tutorial
    * [Pull request #160](https://github.com/gazebosim/gz-sensors/pull/160)

1. Bumps in fortress: ign-sensors6
    * [Pull request #120](https://github.com/gazebosim/gz-sensors/pull/120)

1. Port codecov to new configuration
    * [Pull request #129](https://github.com/gazebosim/gz-sensors/pull/129)

1. Remove deprecations: tock
    * [Pull request #141](https://github.com/gazebosim/gz-sensors/pull/141)

1. Make Sensors aware of CameraPassCountPerGpuFlush & Scene::PostFrame
    * [Pull request #145](https://github.com/gazebosim/gz-sensors/pull/145)

1. Remove plugin interface and support custom sensors
    * [Pull request #90](https://github.com/gazebosim/gz-sensors/pull/90)

1. Run ogre 1.x tests on macos
    * [Pull request #156](https://github.com/gazebosim/gz-sensors/pull/156)

1. Segmentation sensor
    * [Pull request #133](https://github.com/gazebosim/gz-sensors/pull/133)

1. Joint Force-Torque Sensor
    * [Pull request #144](https://github.com/gazebosim/gz-sensors/pull/144)

## Gazebo Sensors 5

### Gazebo Sensors 5.X.X

### Gazebo Sensors 5.1.0 (2021-10-15)

1. Depend on ign-msgs 7.2 and libSDFormat 11.3
    * [Pull request #154](https://github.com/gazebosim/gz-sensors/pull/154)

1. 👩‍🌾 Print debug messages when sensors advertise topics
    * [Pull request #151](https://github.com/gazebosim/gz-sensors/pull/151)

1. Infrastructure
    * [Pull request #150](https://github.com/gazebosim/gz-sensors/pull/150)
    * [Pull request #130](https://github.com/gazebosim/gz-sensors/pull/130)
    * [Pull request #126](https://github.com/gazebosim/gz-sensors/pull/126)
    * [Pull request #124](https://github.com/gazebosim/gz-sensors/pull/124)

1. Publish performance sensor metrics.
    * [Pull request #146](https://github.com/gazebosim/gz-sensors/pull/146)

1. Add API for enabling / disabling IMU orientation
    * [Pull request #142](https://github.com/gazebosim/gz-sensors/pull/142)

1. Init will now set the nextUpdateTime to zero
    * [Pull request #137](https://github.com/gazebosim/gz-sensors/pull/137)

1. Remove clamping from lidar noise
    * [Pull request #132](https://github.com/gazebosim/gz-sensors/pull/132)

1. 👩‍🌾 Disable tests that consistently fail on macOS
    * [Pull request #121](https://github.com/gazebosim/gz-sensors/pull/121)

### Gazebo Sensors 5.0.0 (2021-03-30)

1. Bump in edifice: ign-common4
    * [Pull request #85](https://github.com/gazebosim/gz-sensors/pull/85)

1. Bump in edifice: sdformat11
    * [Pull request #78](https://github.com/gazebosim/gz-sensors/pull/78)

1. Bump in edifice: ign-msgs7
    * [Pull request #75](https://github.com/gazebosim/gz-sensors/pull/75)

1. Bump in edifice: ign-rendering5
    * [Pull request #55](https://github.com/gazebosim/gz-sensors/pull/55)

1. Documentation updates
    * [Pull request #116](https://github.com/gazebosim/gz-sensors/pull/116)

## Gazebo Sensors 4

### Gazebo Sensors 4.X.X

### Gazebo Sensors 4.2.0 (2021-07-12)

1. Add API for enabling / disabling IMU orientation
    * [Pull request #142](https://github.com/gazebosim/gz-sensors/pull/142)

1. Init will now set the nextUpdateTime to zero
    * [Pull request #137](https://github.com/gazebosim/gz-sensors/pull/137)

1. Remove clamping from lidar noise
    * [Pull request #132](https://github.com/gazebosim/gz-sensors/pull/132)

1. Remove tools/code_check and update codecov
    * [Pull request #130](https://github.com/gazebosim/gz-sensors/pull/130)

1. Disable macOS workflow
    * [Pull request #124](https://github.com/gazebosim/gz-sensors/pull/124)

1. 👩‍🌾 Disable tests that consistently fail on macOS
    * [Pull request #121](https://github.com/gazebosim/gz-sensors/pull/121)

1. Master branch updates
    * [Pull request #106](https://github.com/gazebosim/gz-sensors/pull/106)

1. 👩‍🌾 Clear Windows warnings (backport #58)
    * [Pull request #102](https://github.com/gazebosim/gz-sensors/pull/102)

1. Update thermal camera tutorial - include varying temp. objects
    * [Pull request #79](https://github.com/gazebosim/gz-sensors/pull/79)

1. Fix macOS/windows tests that failed to load library
    * [Pull request #60](https://github.com/gazebosim/gz-sensors/pull/60)

1. Removed issue & PR templates
    * [Pull request #99](https://github.com/gazebosim/gz-sensors/pull/99)

### Gazebo Sensors 4.1.0 (2021-02-10)

1. Added issue and PR templates.
    * [Pull request 91](https://github.com/gazebosim/gz-sensors/pull/91)

1. Added `set_rate` service to all sensors.
    * [Pull request 95](https://github.com/gazebosim/gz-sensors/pull/95)

1. Added support for 8 bit thermal camera image format.
    * [Pull request 92](https://github.com/gazebosim/gz-sensors/pull/92)

1. All features up to version 3.2.0.

### Gazebo Sensors 4.0.0 (2020-09-30)

1. Fix link in README.md
    * [Pull request 51](https://github.com/gazebosim/gz-sensors/pull/51)

1. Move installation instructions from README.md to Installation tutorial
    * [Pull request 50](https://github.com/gazebosim/gz-sensors/pull/50)

1. Bump ign-math to 6.6
    * [Pull request 48](https://github.com/gazebosim/gz-sensors/pull/48)

1. Replaced common::Time with std::chrono
    * [Pull request 41](https://github.com/gazebosim/gz-sensors/pull/41)

1. Depend on ign-msgs6, ign-transport9, sdf10
    * [Pull request 31](https://github.com/gazebosim/gz-sensors/pull/31)

1. GitHub migration
    * [Pull request 12](https://github.com/gazebosim/gz-sensors/pull/12)
    * [Pull request 16](https://github.com/gazebosim/gz-sensors/pull/16)
    * [Pull request 22](https://github.com/gazebosim/gz-sensors/pull/22)

1. Set camera sensor visibility mask
    * [BitBucket pull request 115](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/115)

1. Depend on ign-rendering4
    * [BitBucket pull request 111](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/111)

## Gazebo Sensors 3

### Gazebo Sensors 3.6.0 (2024-01-05)

1. Update github action workflows
    * [Pull request #401](https://github.com/gazebosim/gz-sensors/pull/401)
    * [Pull request #335](https://github.com/gazebosim/gz-sensors/pull/335)
    * [Pull request #334](https://github.com/gazebosim/gz-sensors/pull/334)

1. Clean up rendering resources
    * [Pull request #324](https://github.com/gazebosim/gz-sensors/pull/324)

1. Destroy rendering sensors when sensor is removed
    * [Pull request #169](https://github.com/gazebosim/gz-sensors/pull/169)

1. Support protobuf >= 22
    * [Pull request #351](https://github.com/gazebosim/gz-sensors/pull/351)

1. Fix links in Changelog
    * [Pull request #330](https://github.com/gazebosim/gz-sensors/pull/330)

1. Fix Camera info test
    * [Pull request #326](https://github.com/gazebosim/gz-sensors/pull/326)

1. Added Camera Info topic support for cameras
    * [Pull request #285](https://github.com/gazebosim/gz-sensors/pull/285)

### Gazebo Sensors 3.5.0 (2022-11-30)

1. Add missing DEPENDS_ON_COMPONENTS parameters.
    * [Pull request #262](https://github.com/gazebosim/gz-sensors/pull/262)

1. Improved coverage Lidar.
    * [Pull request #277](https://github.com/gazebosim/gz-sensors/pull/277)

1. Improved noise coverage.
    * [Pull request #278](https://github.com/gazebosim/gz-sensors/pull/278)

1. Camera: configure projection matrix from SDFormat.
    * [Pull request #249](https://github.com/gazebosim/gz-sensors/pull/249)

1. RgbdCameraSensor.cc: fix include.
    * [Pull request #280](https://github.com/gazebosim/gz-sensors/pull/280)

1. Ignition to Gazebo header migration.
    * [Pull request #260](https://github.com/gazebosim/gz-sensors/pull/260)

### Gazebo Sensors 3.4.0 (2022-08-16)

1. Remove redundant namespace references
    * [Pull request #258](https://github.com/gazebosim/gz-sensors/pull/258)

1. Ignition -> Gazebo
    * [Pull request #245](https://github.com/gazebosim/gz-sensors/pull/245)

1. Conform to ros format for header field `frame_id` of sensor msgs
    * [Pull request #195](https://github.com/gazebosim/gz-sensors/pull/195)

1. Fix compiler warnings (`CMP0072` and copy elision)
    * [Pull request #188](https://github.com/gazebosim/gz-sensors/pull/188)

1. Require ign-transport >= 8.2
    * [Pull request #167](https://github.com/gazebosim/gz-sensors/pull/167)

### Gazebo Sensors 3.3.0 (2021-08-26)

1. 👩‍🌾 Print debug messages when sensors advertise topics
    * [Pull request #151](https://github.com/gazebosim/gz-sensors/pull/151)

1. Publish performance sensor metrics.
    * [Pull request #146](https://github.com/gazebosim/gz-sensors/pull/146)

1. CI and infrastructure
    * [Pull request #130](https://github.com/gazebosim/gz-sensors/pull/130)
    * [Pull request #150](https://github.com/gazebosim/gz-sensors/pull/150)
    * [Pull request #106](https://github.com/gazebosim/gz-sensors/pull/106)

1. 👩‍🌾 Disable tests that consistently fail on macOS
    * [Pull request #121](https://github.com/gazebosim/gz-sensors/pull/121)

1. 👩‍🌾 Clear Windows warnings (backport #58)
    * [Pull request #58](https://github.com/gazebosim/gz-sensors/pull/58)

1. Fix macOS/windows tests that failed to load library (backport #60)
    * [Pull request #60](https://github.com/gazebosim/gz-sensors/pull/60)

### Gazebo Sensors 3.2.0 (2021-02-08)

1. Apply noise to lidar point cloud.
    * [Pull request 86](https://github.com/gazebosim/gz-sensors/pull/86)

1. Add Windows Installation.
    * [Pull request 82](https://github.com/gazebosim/gz-sensors/pull/82)

1. Added thermal camera tutorial.
    * [Pull request 61](https://github.com/gazebosim/gz-sensors/pull/61)

1. Prevent segfaults on test failures, make tests verbose.
    * [Pull request 56](https://github.com/gazebosim/gz-sensors/pull/56)

1. Resolve updated codecheck issues.
    * [Pull request 57](https://github.com/gazebosim/gz-sensors/pull/57)

1. Improve fork experience.
    * [Pull request 54](https://github.com/gazebosim/gz-sensors/pull/54)

### Gazebo Sensors 3.1.0 (2020-09-03)

1. Update camera sensor only when needed
    * [Pull request 37](https://github.com/gazebosim/gz-sensors/pull/37)

1. Add noise to RGBD camera.
    * [Pull Request 35](https://github.com/gazebosim/gz-sensors/pull/35)

1. Fix version numbers in config.hh
    * [Pull Request 42](https://github.com/gazebosim/gz-sensors/pull/42)

1. Make sure all sensors have a default topic. When invalid topics are passed
   in, convert them to valid topics if possible. If not possible to convert
   into valid topic, fail gracefully.
    * [Pull Request 33](https://github.com/gazebosim/gz-sensors/pull/33)

1. GitHub migration
    * [Pull request 11](https://github.com/gazebosim/gz-sensors/pull/11)
    * [Pull request 21](https://github.com/gazebosim/gz-sensors/pull/21)

### Gazebo Sensors 3.0.0 (2019-12-10)

1. Add support for sdformat frame semantics
    * [BitBucket pull request 104](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/104)

1. Remove deprecations in ign-sensors3
    * [BitBucket pull request 103](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/103)

1. Break out image noise classes
    * [BitBucket pull request 102](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/102)

1. Depend on ign-transport8, ign-msgs5, sdformat9
    * [BitBucket pull request 101](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/101)
    * [BitBucket pull request 105](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/105)

1. Add Thermal Camera Sensor
    * [BitBucket pull request 100](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/100)

1. Updating exports and includes
    * [BitBucket pull request 98](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/98)

1. Removed deprecations from Manager.
    * [BitBucket pull request 99](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/99)

1. Depend on ign-rendering3
    * [BitBucket pull request 88](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/88)

## Gazebo Sensors 2

### Gazebo Sensors 2.9.1 (2020-12-23)

1. Fix version numbers in config.hh
    * [Pull Request 42](https://github.com/gazebosim/gz-sensors/pull/42)

1. Resolve codecheck issues
    * [Pull Request 57](https://github.com/gazebosim/gz-sensors/pull/57)

### Gazebo Sensors 2.9.0 (2020-08-07)

1. Add noise to RGBD camera.
    * [Pull Request 35](https://github.com/gazebosim/gz-sensors/pull/35)

1. Make sure all sensors have a default topic.When invalid topics are passed
   in, convert them to valid topics if possible. If not possible to convert
   into valid topic, fail gracefully.
    * [Pull Request 33](https://github.com/gazebosim/gz-sensors/pull/33)


### Gazebo Sensors 2.8.0 (2020-03-04)

1. Added sequence numbers to sensor data messages.
    * [BitBucket pull request 112](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/112)

### Gazebo Sensors 2.7.0 (2019-12-16)

1. Add clipping for depth camera on rgbd camera sensor (requires sdformat 8.7.0)
    * [BitBucket pull request 107](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/107)

### Gazebo Sensors 2.6.1 (2019-09-13)

1. Fix IMU noise model dt
    * [BitBucket pull request 94](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/94)

### Gazebo Sensors 2.6.0 (2019-08-27)

1. Update depth and rgbd camera sensor to output point cloud data generated by ign-rendering DepthCamera
    * [BitBucket pull request 91](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/91)

### Gazebo Sensors 2.5.1 (2019-08-12)

1. Add intensity and ring fields to GpuLidarSensor point cloud msg
    * [BitBucket pull request 89](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/89)

### Gazebo Sensors 2.5.0

1. Add `IGN_PROFILER_ENABLE` cmake option for enabling the ign-common profiler.
    * [BitBucket pull request 82](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/82)

1. Deduplicate `frame_ids` from sensor message headers
    * [BitBucket pull request 83](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/83)

1. Baseline for stereo cameras
    * [BitBucket pull request 84](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/84)

### Gazebo Sensors 2.4.0 (2019-07-17)

1. Support manual scene updates for rendering sensors
    * [BitBucket pull request 81](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/81)

### Gazebo Sensors 2.3.0 (2019-07-16)

1. The GpuLidar and Rgbd sensors publish point cloud data using
   `msgs::PointCloudPacked`.
    * [BitBucket pull request 78](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/78)

### Gazebo Sensors 2.2.0 (2019-06-27)

1. Update the GPU Lidar to use the sensor's name as the `frame_id`.
    * [BitBucket pull request 74](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/74)

1. Fix camera_info topic to be on the same level as image and depth_image for RGBD Camera.
    * [BitBucket pull request 73](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/73)

### Gazebo Sensors 2.1.0 (2019-06-18)

1. Adds an RGBD camera sensor that combines a CameraSensor and DepthCameraSensor, and also
   outputs a pointcloud.
    * [BitBucket pull request 70](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/70)

1. Create and publish on `camera_info` topics for the Camera and DepthCamera
   sensors.
    * [BitBucket pull request 67](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/67)

### Gazebo Sensors 2.0.0 (2019-05-21)

1. Zero update rate, virtual SetParent and fix gpu_lidar
    * [BitBucket pull request 66](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/66)

1. Add `frame_id` to sensor messages
    * [BitBucket pull request 63](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/63)

1. Restore `pixel_format` in message and add deprecation comment.
    * [BitBucket pull request 62](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/62)
    * [BitBucket pull request 65](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/65)

1. Added noise to  camera and lidar sensors.
    * [BitBucket pull request 60](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/60)
    * [BitBucket pull request 61](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/61)

1. Add support for loading a Lidar sensor from an SDF Sensor DOM object.
    * [BitBucket pull request 59](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/59)

1. Add support for loading an IMU sensor from an SDF Sensor DOM object.
    * [BitBucket pull request 58](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/58)

1. Add support for loading a camera and depth camera sensor from an SDF Sensor DOM object.
    * [BitBucket pull request 57](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/57)

1. Add support for loading an air pressure sensor from an SDF Sensor DOM object.
    * [BitBucket pull request 56](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/56)

1. Add support for loading an altimeter sensor from an SDF Sensor DOM object.
    * [BitBucket pull request 55](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/55)

1. Noise factory uses `sdf::Noise` objects, Magnetometer sensor utilizes
   noise parameters.
    * [BitBucket pull request 54](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/54)

1. Add support for loading a magnetometer sensor from an SDF Sensor DOM object.
    * [BitBucket pull request 53](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/53)

1. Add magnetometer
    * [BitBucket pull request 47](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/47)

1. Add IMU
    * [BitBucket pull request 44](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/44)

1. Add altimeter
    * [BitBucket pull request 43](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/43)

1. Create component for rendering sensor classes
    * [BitBucket pull request 42](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/42)

1. Upgrade to gz-rendering2
    * [BitBucket pull request 45](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/45)

1. Upgrade to gz-msgs4 and gz-transport7
    * [BitBucket pull request 51](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-sensors/pull-requests/51)

### Gazebo Sensors 1.X.X (2019-XX-XX)

1. Fix windows linking
    * [BitBucket pull request 49](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/49)
    * [Issue 6](https://github.com/osrf/gazebo/issues/6)

### Gazebo Sensors 1.0.0 (2019-03-01)
