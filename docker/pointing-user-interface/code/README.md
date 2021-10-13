# Pointing User Interface (PUI)
This repository contains various code and tools used to develop a generic pointing-based interaction. Also, it provides specific code for pointing-based selection with LED based feedback, deployed in an industrial scenario where users have to select defected packages moving on a conveyor belt.
- `conveyor_gui`: An rqt plugin that displays the map and let one selects packages (travelling on conveyor belts)
- `conveyor_msgs`: ROS2 interface for packages travelling on conveyor belts
- `conveyor_pui`: 
- `conveyor_utils`: A ROS2 package with utilities to load maps, transform between strips and belts coordinate systems, and test the system (dummy package generator, simple package rendering on LED strips, ...).
- `led_strip_msgs`: ROS2 interface to a LED strip.
- `led_strip_utils`: A ROS2 package with utilities to test the LED strip interfaces.
- `coppelia_scenes`: CoppeliaSim scenes
- `relloc`: code to execute the relative localization. It also contains the launch files and the code to run the scenarious and demonstrate the system. 

## Using the PUI
The PUI works as it follows:
- each user has a set of dedicated node, under a specified namespace. Those can be launched all together using [user.launch](#relloc/launch/user.launch). [Here](TODO LINK) the available parameters are documented in details.
- each user, once localized, can subscribe to the globally available `pui_node`. This node is launched through [scenario_1_2_3_4.launch](relloc/launch/scenario_1_2_3_4.launch). Available parameters are described [here](TODO LINK).
- the `pui_node` waits for users to subscribe. Once it receives a subscription request, it starts receiving all the needed inputs (user's location and pointing rays) and, thanks to the environment map, can provide pointing cursors positions.
- other nodes are used to implement our scenarios (e.g. simulating packages, feeders, checking for package selection, etc.). Those can be seen from [lab.launch](code/conveyor_utils/launch/lab.launch), for scenarios simulating packages on LED strips, and from [belt.launch](code/conveyor_utils/launch/belt.launch) for the scenario with the actual conveyor belt.

Note that for convenience, [docker_hri.launch](relloc/launch/docker_hri.launch) can be used to run both the global nodes and a set of nodes for one user. This was done to easily deploy demos from the docker containers.

### scenario_1_2_3_4.launch
This launch file will launch the nodes needed for the PUI and for the interaction with the systems.
For the PUI it will launch, based on `single_LED` parameters, one between:
- `pui_node`: standard PUI node.
- `single_pui_node`: PUI node which compute the intersection between pointing rays and single LED lights.

For the LED/conveyor system, it will call based on `sim_packages_on_LED` parameters, one between:
- `lab.launch`: available [here](TODO LINK). This will launch the nodes to draw packages and pointing cursor on LED strips. It will also instantiate the nodes which simulate creating and feeding packages.
- `belt.launch`: available [here](TODO LINK). This will launch just the nodes to draw packages position and cursor on LED strip, since it is used the actual conveyor belt.

These are the parameter available for this launch file:
| parameter             | type   | valid values | default                          | description                                                                                                                                  |
|-----------------------|--------|--------------|----------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| `map_path`            | string |              | "conveyor_utils/launch/lab.yaml" | path to the yaml file containing the system map                                                                                              |
| `single_LED`          | bool   | True, False  | False                            | True for scenario_1: it runs a different PUI node which  checks the intersection of pointing rays with single LEDs instead of LED strips     |
| `time_for_selection`  | float  | >0.0         | 0.5                              | seconds of overlay needed to select a package                                                                                                |
| `pointing_tol`        | float  | >0.0         | 0.7                              | If the distance between the pointing ray and LED strip is below this value,  the pointing cursor is drawn on the closest point on the strip. |
| `sim_packages_on_LED` | bool   | True, False  | True                             | If True, packages are simulated on the LED strips. Must be set to False,  when using the actual conveyor belt.                               |
| `joy_dev`             | int    |              | 0                                | number of the joy interface to which the joypad is connected. The joypad can be used  to control the conveyor system.                        |
| `feed`                | bool   | True, False  | False                            | Whether the package feeder (for packages simulated on LEDs) is active from the start or not                                                  |
