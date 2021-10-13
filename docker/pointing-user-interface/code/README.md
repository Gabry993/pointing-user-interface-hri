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
- each user has a set of dedicated node, under a specified namespace. Those can be launched all together using [user.launch](relloc/launch/user.launch). [Here](TODO LINK) the available parameters are documented in details.
- each user, once localized, can subscribe to the globally available `pui_node`. This node is launched through [scenario_1_2_3_4.launch](relloc/launch/scenario_1_2_3_4.launch). Available parameters are described [here](TODO LINK).
- the `pui_node` waits for users to subscribe. Once it receives a subscription request, it starts receiving all the needed inputs (user's location and pointing rays) and, thanks to the environment map, can provide pointing cursors positions.
- other nodes are used to implement our scenarios (e.g. simulating packages, feeders, checking for package selection, etc.). Those can be seen from [lab.launch](code/conveyor_utils/launch/lab.launch), for scenarios simulating packages on LED strips, and from [belt.launch](code/conveyor_utils/launch/belt.launch) for the scenario with the actual conveyor belt.

Note that for convenience, [docker_hri.launch](relloc/launch/docker_hri.launch) can be used to run both the global nodes and a set of nodes for one user. This was done to easily deploy demos from the docker containers.

### scenario_1_2_3_4.launch
This [launch file](TODO LINK) will launch the nodes needed for the PUI and for the interaction with the systems.
For the PUI it will launch, based on `single_LED` parameters, one between:
- `pui_node`: standard PUI node.
- `single_pui_node`: PUI node which compute the intersection between pointing rays and single LED lights.

For the LED/conveyor system, it will call based on `sim_packages_on_LED` parameters, one between:
- `lab.launch`: available [here](TODO LINK). This will launch the nodes to draw packages and pointing cursor on LED strips. It will also instantiate the nodes which simulate creating and feeding packages.
- `belt.launch`: available [here](TODO LINK). This will launch just the nodes to draw packages position and cursor on LED strip, since it is used the actual conveyor belt.

These are the parameters available for this launch file:
| parameter             | type   | valid values | default                          | description                                                                                                                                  |
|-----------------------|--------|--------------|----------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| `map_path`            | string |              | `conveyor_utils/launch/lab.yaml` | path to the yaml file containing the system map                                                                                              |
| `single_LED`          | bool   | `True`, `False`  | `False`                            | True for scenario_1: it runs a different PUI node which  checks the intersection of pointing rays with single LEDs instead of LED strips     |
| `time_for_selection`  | float  | `> 0.0`         | `0.5`                              | seconds of overlay needed to select a package                                                                                                |
| `pointing_tol`        | float  | `> 0.0`         | `0.7`                              | If the distance between the pointing ray and LED strip is below this value,  the pointing cursor is drawn on the closest point on the strip. |
| `sim_packages_on_LED` | bool   | `True`, `False`  | `True`                             | If True, packages are simulated on the LED strips. Must be set to False,  when using the actual conveyor belt.                               |
| `joy_dev`             | int    |              | `0`                                | number of the joy interface to which the joypad is connected. The joypad can be used  to control the conveyor system.                        |
| `feed`                | bool   | `True`, `False`  | `False`                            | Whether the package feeder (for packages simulated on LEDs) is active from the start or not                                                  |

### user.launch
This [launch file](TODO LINK HERE) will starts all the node needed to add a user interacting with the system. All those nodes will be running under the provided `user_name` namespace. This will launch:
- `human_kinematics.launch`: this starts the nodes deploying the pointing model and publishing the pointing rays for the user.
- `pointing_demo`: this is the node coordinating the state of our scenarios interactions between nodes
- `relloc_exec`: node that actually performs the relative localization. It runs only if `do_relloc` parameter is set to `True`.
- `relloc_node`: node in charge of localizing the user, either with tracking, fixed transform or relative localization
- `static_transform_publisher`: a couple of publishers which are helpful to publish fixed transform for the user, based on the desired setup

These are the parameter available for this launch file:
| parameter         | type                  | valid values                         | default                          | description                                                                                                                                           |
|-------------------|-----------------------|--------------------------------------|----------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| `map_path`        | string                |                                      | `conveyor_utils/launch/lab.yaml` | Path to the yaml file containing the system map                                                                                                       |
| `sim_imu`         | bool                  | `True`, `False`                          | `False`                            | True when we are simulating the IMU in coppelia                                                                                                       |
| `update_rate`     | float                 | `> 0.0`                                 | `10.`                              | Hz rate of desired LED refresh rate                                                                                                                   |
| `do_relloc`       | bool                  | `True`, `False`                          | `True`                             | Whether user has to perform relative localization or not                                                                                              |
| `use_state_LED`   | bool                  | `True`, `False`                          | `False`                            | Whether one of the single LEDs should be used as system status LED                                                                                    |
| `led_on_period`   | float                 | `> 0.0`                                 | `3.`                               | How long in seconds the single LEDs must stay ON during relloc                                                                                        |
| `relloc_leds`     | [string]              | `[led_n, led_m]`                     | `['led_1, led_2]`                | List with the two names of the LEDs used for the relloc                                                                                               |
| `gt_x`            | float                 |                                      | `0.0`                              | x position of the user, when manually set                                                                                                             |
| `gt_y`            | float                 |                                      | `0.0`                              | y position of the user, when manually set                                                                                                             |
| `gt_yaw`          | float                 |                                      | `0.0`                              | Radians of user heading, when manually set                                                                                                            |
| `rotation_topic`  | string                | `metawear_ros/rotation` or `imu` | `metawear_ros/rotation`          | Name of the topic publishing the IMU values used to track user arm/shoulder orientation                                                               |
| `user_kinematics` | string                | `human` or `bill`                    | `human`                          | Name for the user kinematics file describing user's biometry                                                                                          |
| `user_name`       | string                |                                      | `user1`                          | Unique user name to identify it in the system. This will also be used to name the namespace for nodes and to subscribe to the PUI.                    |
| `pointer_size`    | float                 | `> 0.0`                                 | `0.2`                              | Horizontal size of user's pointing cursor                                                                                                             |
| `pointer_color`   | [float, float, float] |                                      | `[1., 1., 1.]`                     | R, G, B triple with float in [0., 1.] for user's pointing cursor color                                                                                |
| `pointer_cmap`    | string                | `viridis`, `matplotlib cmap`           | `""`                               | If a valid matplotlib cmap name is provided, the cursor color will be mapped to it, based on the distance between the pointing ray and the LED strips |
