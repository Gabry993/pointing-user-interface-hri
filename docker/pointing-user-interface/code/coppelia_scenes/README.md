# CoppeliaSim scenes

## Setup

Most scenes expose a ROS2 interface which requires the ROS2 coppeliaSim plugin. Follow the instruction on https://github.com/jeguzzi/simExtROS2 (branch `oneswarm`) to install it. In particular, you need to add the messages definitions you find listed in `interfaces.txt`, which in turn requires to add the relative packages as a dependence.

## Objects

### Customizable LED pole

The scene `customizable_led_pole.ttt` contains a customizable RGB LED on a pole. You can change height and name.

![Customizable LED pole screenshot](https://github.com/idsia-robotics/oneswarm-hri/raw/main/oneswarm_hri_scenes/customizable_led_pole.png "Customizable LED pole")

The LED subscribe to two ROS2 topics:
- a `std_msgs/msg/ColorRGBA` topic `<name>`, which you can use to set the LED color
- a `led_strip_msgs/msg/LedEffect` topic `<name>/effect` which sets the led effect (`off|on|flash|pulse`)

and provide one service
- a `led_strip_msgs/srv/SetBrightness` service `<name>/set_brightness`.

### LED strip

The scene `led_strip.ttt` contains a customizable RGB LED strip.

![Customizable LED strip screenshot](https://github.com/idsia-robotics/oneswarm-hri/raw/main/oneswarm_hri_scenes/led_strip.png "Customizable LED strip")

The LED strip is emulated using a texture attached to all 6 sides of the [cuboid] shape, each LED is emulated by 1 pixel. There is no interpolation between pixels.

You can customize it by changing the parameters:
- `name`     the ROS namespace
- `number`   the number of LED
- `width`    the length of the strip (in meters)
- `channel`  the channel and uid (integer >= 0)
- `freezed`  freezes the led strip and avoid recreating the texture.

Give *different* names and channels to different strips!

Each time you change the number or width, a new texture is created.
You have then to manually adjust one parameter in the shape properties dialog:
`Scene Object Properties > Adjust texture > Apply Mode > "add"
`
else the strip will be [completely] black when the LEDs are off.

When enabled through the parameter `enable_individual_interfaces`,
each LED strip subscribes to the following ROS2 topics:
- `<name>`        a `led_strip_msgs/ColorArray` message with the array of (real-valued in [0, 1]) colors (one for each LED).  If there are more colors than LEDs, they are ignored.  If there are less colors than LEDs, they are filled-in with blacks.
- `<name>/image`  a `sensor_msgs/Image` with the same interpretation as above but as a flat char array [r, g, b, r, g, b, ...]
- `<name>/blob`   a `led_strip_msgs/Blob` message to display a blob of a given (relative) size and position along the strip.

#### Led Driver

The strips are also exposed through a single interface like when using a LED Driver. When enabled through the parameter `enable_serial_led_driver`, it subscribes to one ROS2 topics:
- a `led_strip_msgs/msg/LedStrips` topic `/led_strips`

and provide one service
- a `led_strip_msgs/srv/SetBrightness` service `<name>/set_brightness`.

### Pointing Bill

The scene `pointing_bill.ttt` contains a customizable Bill character that can point to target object defined by the value of the integer signal `pointing_target`. Set the value to the handle of an object in the scene to make Bill track that object. Clear the signal to make Bill stop tracking objects.

![Pointing Bill screenshot](https://github.com/idsia-robotics/oneswarm-hri/raw/main/oneswarm_hri_scenes/pointing_bill.png "Pointing Bill")

Bill carries on his wrist an IMU that publishes a `geometry_msgs/msg/QuaternionStamped` topic at `imu`. The quaternion is defined with respect to the frame `IMU_ref`, that has a random orientation (per run) along the common z-axis, to mimic the behavior of a real imu sensor without absolute orientation.

### Bill's Ghost

To test localization, you can use the ghost of Bill contained in scene `ghost_of_bill.ttt`. One the TF transform between world and Bill is published, it locates the ghost correspondingly

### Synesis Demostrator

The scene reproduce the conveyor belt system at the Synesis demonstrator near Como. We have added LED strips on both sides of the belts that exposes the same ROS2 interface as LED Driver above. Moreover, there is a package tracker that publish a `conveyor_msgs/msg/PackageList` on the `packages` topic with the list of packages currently transported by the belts.

![Synesis Demostrator screenshot](https://github.com/idsia-robotics/oneswarm-hri/raw/main/oneswarm_hri_scenes/synesis_demostrator.png "Synesis Demostrator")

## Scenarios

At the start of the demo, the `Demo` objects save a map of all relevant objects in a file at a location defined the `LED_MAP_PATH` environment variable.

### Scenario 1

The scene `scenario_1.ttt` basic scenario to test localization and target identification with LED poles.

![Scenario 1 screenshot](https://github.com/idsia-robotics/oneswarm-hri/raw/main/oneswarm_hri_scenes/scenario_1.png "Scenario 1")

The demo follows this plot:
1. the human is waiting for the first LED to switch on
2. until the LED are not off by more than 1 second, the user keep pointing at the one active LED, privileging the one that is longer active
3. the user start pointing for random times between 5 and 10 seconds to any LED, ignoring their state

### Scenario 2

The scene `scenario_2.ttt` another basic scenario to test localization and target identification with LED poles and 2 LED strips.

![Scenario 2 screenshot](https://github.com/idsia-robotics/oneswarm-hri/raw/main/oneswarm_hri_scenes/scenario_2.png "Scenario 2")

The demo follows this plot:
1. the human is waiting for the first LED to switch on
2. until the LED are not off by more than 1 second, the user keep pointing at the one active LED, privileging the one that is longer active
3. the user start pointing for random times between 5 and 10 seconds to any LED, ignoring their state

The demo exposes one parameter:
- `manual_target_control`: set to true, to manually control the pointing target (`bool`, `false` by default)

Manual control works by moving the dummy `Demo > pointing_target` and enabling Camera layer 3 to display it.

#### TF

The scene also display a ghost of Bill (visible as grey-colored if you enable Layer 4) at the position where ROS localize Bill (i.e., with respect to the TF transform `world -> human`). The ghost is not moving its arm.

It also publishes a TF transform `world -> human_gt` with the ground truth human pose. Please note that this is the pose of the trunk, not of the shoulder or the arm, i.e., it won't change if Bill rotate the arm while keeping the trunk still.

### Scenario 3 (packages)

The scene `scenario_3.ttt` is an extension of scenario 2 where packages running on a conveyor belt are emulated by drawing them on the LED strip using an external [ROS2] controller.

![Scenario 3 screenshot](https://github.com/idsia-robotics/oneswarm-hri/raw/main/oneswarm_hri_scenes/scenario_3.png "Scenario 3")

The demo exposes an additional parameter:
- `interesting_package_type`: the type of packages to point to (`integer`, `0` by default)

The demo follows this plot:
1. the human is waiting for the first LED to switch on;
2. until any LED is not off by more than 1 second, the user keeps pointing at the active LED, privileging the one that has been active longer;
3. the user points at an interesting packages until they are all selected, starting with the first one.

The package list is read from a ROS topic `/packages`. Only packages with `type=<interesting_package_type>` are considered.

To get a [minimal] working demo:
1. `ros2 launch conveyor_utils lab.launch`
2. switch on/off one LED once to start the interaction (which mimic the localization procedure)
3. select the packages by clicking on the `conveyor_gui` map.

To get a complete working demo with localization and proper LED strip feedback:

> TODO(Gabriele)

### Synesis Demostrator

The scene `scenario_4.ttt` reproduce the same demo of Scenario 3 but packages travel on a conveyor belt instead of on LED strips.

The package list is now generate in the scene and published on the topic `/packages`.

![Scenario 4 screenshot](https://github.com/idsia-robotics/oneswarm-hri/raw/main/oneswarm_hri_scenes/scenario_4.png "Scenario 4")

To get a [minimal] working demo:
1. `ros2 launch conveyor_utils beltlaunch`
2. switch on/off one LED once to start the interaction (which mimic the localization procedure)
3. select the packages by clicking on the `conveyor_gui` map.

To get a complete working demo with localization and proper LED strip feedback:

> TODO(Gabriele)
