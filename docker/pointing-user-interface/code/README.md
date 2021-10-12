# Oneswarm-hri
This repository contains various code and tools used during the development of Task 4.3 of 1-SWARM.
- `oneswarm_hri_scenes`: CoppeliaSim scenes
- `led_strip_msgs`: ROS2 interface to a LED strip (used as of now just in simulation)
- `led_strip_utils`: A ROS2 package with utilities to test the LED strip interfaces.
- `conveyor_msgs`: ROS2 interface for packages travelling on conveyor belts
- `conveyor_utils`: A ROS2 package with utilities to load maps, transform between strips and belts coordinate systems, and test the system (dummy package generator, simple package rendering on LED strips, ...).
- `conveyor_gui`: An rqt plugin that displays the map and let one selects packages (travelling on conveyor belts)


## Map format

The coppeliaSim scenes and the ROS packages uses the following map format to represent the main objects.

If there are no belts but strips, it is implicitly assumed that belts are emulated on the strips, i.e., for every strip there is a belt with the same geometry, named `strip_<uid>`.

```yaml
belts:
  <name>:
    centerline: [[<x>, <y>, <z>], ..., [<x>, <y>, <z>]] # [floats, in m]
    width: <width>  # [float, in m]
    length: <length>  # optional, [float, in m]
leds:
  <name>:  # [string]
    position: [<x>, <y>, <z>] # floats in m
strips:
  <uid>: # equal to the channel id [int]
    pixels: <pixels> # number of LEDs [int]
    direction: <direction> # 1 (first pixel in line[0]) or -1 (last pixel in line[0])
    line: [[<x>, <y>, <z>], ..., [<x>, <y>, <z>]]  #  [floats, in m]
    name: <name>  # [string]
```
