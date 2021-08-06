# Bounding Box Camera in Ignition Gazebo
In this tutorial, we will discuss how to use a bounding box camera sensor in Ignition Gazebo.

## Requirements

Since this tutorial will show how to use a bounding box camera sensor in Ignition Gazebo, you'll need to have Ignition Gazebo installed. We recommend installing all Ignition libraries, using version Fortress or newer (the bounding box camera is not available in Ignition versions prior to Fortress).
If you need to install Ignition, pick the version you'd like to use and then follow the installation instructions.

## Setting up the bounding box camera
Here's an example of how to attach a bounding box camera sensor to a model in a SDF file:

```xml
<model name="boundingbox_camera">
     <pose>4 0 1.0 0 0.0 3.14</pose>
     <link name="link">
       <pose>0.05 0.05 0.05 0 0 0</pose>
       <inertial>
         <mass>0.1</mass>
         <inertia>
           <ixx>0.000166667</ixx>
           <iyy>0.000166667</iyy>
           <izz>0.000166667</izz>
         </inertia>
       </inertial>
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

       <sensor name="boundingbox_camera" type="boundingbox_camera">
         <box_type>2d</box_type>
         <topic>boxes</topic>
         <camera>
           <horizontal_fov>1.047</horizontal_fov>
           <image>
             <width>800</width>
             <height>600</height>
           </image>
           <clip>
             <near>0.1</near>
             <far>10</far>
           </clip>
         </camera>
         <always_on>1</always_on>
         <update_rate>30</update_rate>
         <visualize>true</visualize>
       </sensor>
     </link>
   </model>
```
Let’s take a closer look at the portion of the code above that focuses on the bounding box camera sensor:

```xml
<sensor name="boundingbox_camera" type="boundingbox_camera">
         <box_type>2d</box_type>
         <topic>boxes</topic>
         <camera>
           <horizontal_fov>1.047</horizontal_fov>
           <image>
             <width>800</width>
             <height>600</height>
           </image>
           <clip>
             <near>0.1</near>
             <far>10</far>
           </clip>
         </camera>
         <always_on>1</always_on>
         <update_rate>30</update_rate>
         <visualize>true</visualize>
       </sensor>
```

As we can see, we define a sensor with the following SDF elements:
* `<camera>`: The camera, which has the following child elements:
	* `<horizontal_fov>`: The horizontal field of view, in radians.
	* `<image>`: The image size, in pixels.
	* `<clip>`: The near and far clip planes. Objects are only rendered if they're within these planes.
* `<always_on>`: Whether the sensor will always be updated (indicated by 1) or not (indicated by 0). This is currently unused by Ignition Gazebo.
* `<update_rate>`: The sensor's update rate, in Hz.
* `<visualize>`: Whether the sensor should be visualized in the GUI (indicated by true) or not (indicated by false). This is currently unused by Ignition Gazebo.
* `<topic>`: The name of the topic where sensor data is published.

<br>
There's also an optional plugin used here that allows for further configuration of the bounding box camera. Here's a description of the elements in this plugin (if the plugin isn't used, the default values mentioned below are used):

`<box_type>`: The type of bounding boxes, boxes can be 2d or 3d, and 2d boxes can be visible 2d boxes or full 2d boxes

Visible 2d box: 2D axis aligned box that shows the visible part of the occluded object
Full 2d box: 2D axis aligned box that shows the full box of occluded objects
3D box: Oriented 3D box defined by the center position / orientation / size
Default value for the `<box_type>` is “Visible 2d box”
The `<box_type>` values can be
- For visible 2d boxes: [“2d”, “visible_2d”, “visible_box_2d” ]
- For full 2d boxes: [“full_2d”, “visible_box_2d”]
- For 3d boxes: [“3d”]


## Assigning a label to a model
Only models with labels (annotated classes) will be visible by the bounding box camera sensor, and unlabeled models will be considered as a background

To assign a label to a model we use the label plugin in the SDF file

```xml
<model name="box">
      <pose>0 -1 0.5 0 0 0</pose>
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
            <ambient>0 0 0.5 1</ambient>
            <diffuse>0 0 1 1</diffuse>
            <specular>0 0 0.3 1</specular>
          </material>
          <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
            <label>10</label>
          </plugin>
        </visual>
      </link>
    </model>
```

Lets zoom in the label plugin

```xml
          <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
            <label>10</label>
          </plugin>
```

We assign the label of the model by adding that plugin to the `<visual>` tag of the model or the `<model>` tag(will be shown below), And we add the `<label>` tag to assign the label class to the model.

The code above is an example for adding the label plugin as a child for the `<visual>` tag.

Example for adding the label plugin as a child for the `<model>` tag

```xml
    <model name="sphere">
      <static>true</static>
      <pose>-1 -2 0.5 0 0 0</pose>
      <link name="sphere_link">
      ...
      </link>
      <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
        <label>20</label>
      </plugin>
    </model>
```

Or by including a model from ignition fuel, you can add the label plugin as a child for the <include> tag

```xml
   <include>
      <pose>-1 0 3 0.0 0.0 1.57</pose>
      <uri>
      https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Construction Cone
      </uri>
      <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
        <label>30</label>
      </plugin>
    </include>
```

## Running an example:
Now that we've discussed how a bounding box camera and models with labels can be specified, let's start with an example world that uses the bounding box camera.
Run the following command:
```
ign gazebo boundingbox_camera.sdf
```

You should see something similar to this:
@image html files/boundingbox_camera/boundingbox_camera_visible.png


Note that the mode was visible 2d, so that the occluded sphere has a bounding box on just the visible part of it, that is the way of the annotation in famous datasets such as COCO & PASCAL VOC datasets.


Change the `<box_type>` to `full_2d` to see the full bounding boxes mode which shows the full box of the occluded objects, and this mode is implemented in datasets such as `KITTI` and `nuscenes`, and it is benefit in the case that you need a the model to guess the hidden parts of the objects!.

You will see output like this:
@image html files/boundingbox_camera/boundingbox_camera_full.png




To visualize the output from different SDF file, you can include the model of the sensor in any SDF file then run it via ign gazebo and open the `Image Display` plugin and select the topic you specified in the `<topic>` tag.


Taking a look at the SDF file for this example.




## Processing the bounding box camera's output
In the example above, the bounding box cameras publish an axis aligned 2d bounding box message in case of 2d boxes mode (visible 2d or full 2d) or a oriented 3d bounding box message  to the topic provided in the `<topic>` tag in SDF
For example, in the next code we publish on the topic “boxes”

```xml
      <sensor name="boundingbox_camera" type="boundingbox_camera">
         <box_type>2d</box_type>
         <topic>boxes</topic>
	    ....
```
The camera also publishes an image message of an image with the bounding boxes drawn on it(not valid in case of 3d boxes), and it publish it on topic “TOPIC_TAG_NAME_image”, for example, in the above code, it will publish the image message on “boxes_image”.

We can see the bounding boxes by running the following command:
```ign topic -e -t TOPIC_TAG_NAME```
In the above example, it will be:
```ign topic -e -t boxes```


Here's an example for bounding box camera subscriber that gets the boxes:
```cpp
#include <cstdint>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

// a callback function that is triggered whenever the bounding box camera
// topic receives a new 2d boxes message
void OnNewBoxes2D(const ignition::msgs::AnnotatedAxisAligned2DBox_V &_msg)
{
 int size = _msg.annotated_box_size();
 std::cout << "Reveived [" << size << "] 2D bounding boxes: " << std::endl;
 for (int i = 0; i < size; i++)
 {
   ignition::msgs::AnnotatedAxisAligned2DBox annotatedBox = _msg.annotated_box(i);
   std::cout << annotatedBox.DebugString() << std::endl;
 }
 std::cout << std::endl;
}

// a callback function that is triggered whenever the bounding box camera
// topic receives a new 3d boxes message
void OnNewBoxes3D(const ignition::msgs::AnnotatedOriented3DBox_V &_msg)
{
 int size = _msg.annotated_box_size();
 std::cout << "Reveived [" << size << "] 3D bounding boxes: " << std::endl;
 for (int i = 0; i < size; i++)
 {
   ignition::msgs::AnnotatedOriented3DBox annotatedBox = _msg.annotated_box(i);
   std::cout << annotatedBox.DebugString() << std::endl;
 }
 std::cout << std::endl;
}

int main(int argc, char **argv)
{
 ignition::transport::Node node;
 bool subscribed = false;

 if (argc > 1)
 {
   std::string is3D = argv[1];
   if (is3D == "3d")
     subscribed = node.Subscribe("/boxes", &OnNewBoxes3D);
 }
 else
 {
   subscribed = node.Subscribe("/boxes", &OnNewBoxes2D);
 }
 if (!subscribed)
 {
   std::cerr << "Error subscribing to the boundingbox camera topic" << std::endl;
   return -1;
 }

 ignition::transport::waitForShutdown();
 return 0;
}
```
To run the sensor in 3d boxes mode, give it argument `3d`, for example: `./executable_name 3d`


And Its CMakeLists.txt file

```cmake
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(ignition-sensors-camera-demo)

# Find the Ignition Libraries used directly by the example
find_package(ignition-rendering6 REQUIRED OPTIONAL_COMPONENTS ogre ogre2)
find_package(ignition-sensors6 REQUIRED COMPONENTS rendering camera)

if (TARGET ignition-rendering6::ogre)
  add_definitions(-DWITH_OGRE)
endif()
if (TARGET ignition-rendering6::ogre2)
  add_definitions(-DWITH_OGRE2)
endif()

add_executable(boundingbox_camera main.cc)
target_link_libraries(boundingbox_camera PUBLIC
 ignition-sensors6::camera)

```
