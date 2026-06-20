#!/usr/bin/env python3
#
# Copyright (C) 2026 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Generate a world with N RGBD cameras ringed around a small cluster of
# textured objects, for benchmarking the off-thread publish tail.
#
#   gen_world.py N [W H [RATE]] > out.sdf
#
#   N      number of rgbd_camera sensors (topics rgbd_0 .. rgbd_{N-1})
#   W H    image resolution (default 640 480)
#   RATE   requested update_rate in Hz (default 15; use a high value, e.g. 60,
#          to run the cameras uncapped so the render thread is the bottleneck
#          regardless of machine load)
import math
import sys

N = int(sys.argv[1])
W = int(sys.argv[2]) if len(sys.argv) > 2 else 640
H = int(sys.argv[3]) if len(sys.argv) > 3 else 480
RATE = int(sys.argv[4]) if len(sys.argv) > 4 else 15
R, Z = 6.0, 1.2
cams = []
for n in range(N):
    a = 2 * math.pi * n / max(N, 1)
    x, y = R * math.cos(a), R * math.sin(a)
    yaw = math.atan2(-y, -x)
    cams.append(f"""    <model name="rgbd_{n}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {Z:.3f} 0 0.15 {yaw:.4f}</pose>
      <link name="link">
        <sensor name="rgbd" type="rgbd_camera">
          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image><width>{W}</width><height>{H}</height></image>
            <clip><near>0.1</near><far>100</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>{RATE}</update_rate>
          <topic>rgbd_{n}</topic>
        </sensor>
      </link>
    </model>""")
palette = [(0.8,0.2,0.2),(0.2,0.7,0.3),(0.2,0.4,0.85),(0.85,0.75,0.2),(0.7,0.3,0.75),(0.3,0.75,0.8)]
positions = [(0,0,0.5),(1.2,0.4,0.7),(-1.0,0.8,0.4),(0.5,-1.2,0.6),(-1.1,-0.9,0.9),(1.3,-0.6,0.3)]
objs = []
for i,(px,py,pz) in enumerate(positions):
    r,g,b = palette[i % len(palette)]
    shape = (f"<box><size>{0.5+0.3*(i%3):.2f} {0.5+0.2*(i%2):.2f} {2*pz:.2f}</size></box>"
             if i % 2 == 0 else f"<sphere><radius>{pz:.2f}</radius></sphere>")
    objs.append(f"""    <model name="obj_{i}">
      <static>true</static>
      <pose>{px} {py} {pz} 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry>{shape}</geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular></material>
        </visual>
      </link>
    </model>""")
print(f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="rgbd{N}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane"><static>true</static><link name="link">
      <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
    </link></model>

{chr(10).join(objs)}

{chr(10).join(cams)}
  </world>
</sdf>""")
