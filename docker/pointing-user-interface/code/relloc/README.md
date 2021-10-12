
### To Do
- [x] Oculus VR setup
- [x] VR scene launch for 1 2 3
- [x] Single launch file for 1, 2, 3 sim, real and VR (to do in lab)
- [x] docker hub for metawear on demo
- [x] check docker hub for uros on demo (and if it works)
- [ ] Scenario 4 Real
- [x] Scenario 4 Sim
- [ ] Scenario 4 VR
- [x] launch coppelia on omen batch file
- [x] launch frame rebrodcaster with batch from desktop on omen 
- [ ] links
- [ ] readme for iros task (90%)
- [x] add relloc functionalities to iros task
- [ ] fix led strips driver
- [x] multicast for ros_optitrack ?
 
# Demos  

Here we explain how to launch our demos.  

## Setup  
All packages available in [this](https://github.com/idsia-robotics/oneswarm-hri) repository must be installed in a ROS2 workspace.  
Moreover, you will need those packages as well:  
- `volaly_kinematics` to reconstruct the pointing model. The ros2 version is available [here](https://github.com/Gabry993/volaly_kinematics/tree/ros2)  
- `volaly_msgs` (maybe this can be removed). The ros2 version is available [here](https://github.com/Gabry993/volaly_msgs/tree/ros2)  

## Sim, Real and VR
All the demos can be run in simulation (running the corresponding CoppeliaSim scenes available [here](https://github.com/idsia-robotics/oneswarm-hri/tree/main/oneswarm_hri_scenes#scenarios)) and in real world. 
So, each section will cover how to run the scene in both cases.  
Of course, to run the real demos, the LEDs map must correspond to the actual physical positions in the real world. 
For our lab, the map `scenario_1_2_3_lab_map.yaml` does this. Single LEDs must be placed in the correct position:  
- tape marked `led_0` on the ground -> LED board with black cable  
- tape marked `led_1` on the ground -> LED board with white cable

To run demos in VR, you will need to run both simulation (i. e. CoppeliaSim) and real world components (i.e. optitrack 
and metawear).

### CoppeliaSim Config
Each coppeliaSim scene exposes two config GUIs. Those can be accessed by inspecting the parameters for `Demo` and `Bill` 
objects in the scene. Here are the available parameters:

#### Demo config GUI
- `ignore Bill` flag to enable/disable Bill. If disabled, it won't be rendered in VR, his transformations and IMU won't 
  be published. If enabled, those parameters are available:
	- `Bill should perform localization procedure` flag to set Bill behavior. If set, Bill will perform the relative 
	  localization procedure before doing anything else.
	- `Bill behavior after localization` teardown menu to se Bill behavior (after the localization if it applies). 
	  `manual pointing` allows the user to move the pointing target (visible by enabling `layer 3` in coppelia). Bill will point at it; 
	  `Random pointing on the strips` will make Bill point at random positions on the LEDs strips; 
	  `Random pointing LED` will make Bill point randomly at the single LEDs; 
	  `Selecting a package` will make Bill point at interesting packages (either simulated on the LEDs strips or actual packages).
	- `Pick random duration between [s-s]` Bill will change pointed target after a random duration in this range
	- `Select package of class` set class number for interesting packages (red=1, blue=0)
- `Save map` flag to store the LEDs/belts yaml map at the given location.
- `Package length (tracker)` package length rendered by the LED tracker (i.e. will change the LED render, not the actual package size)

#### Bill config GUI
- `Random colors` flag to set Bill appearance to random colors
- `Control` all parameters here control Bill's movement
- `IMU` enable and disable Bill's IMU. Also, set the IMU yaw bias to a value or randomly

## IMU sources  
All the (real world) demos can be run with different IMU sources. Here we explain how to run them using both the 
optitrack or the metawear. Note that the metawear will allow you to press its button to proceed during the demos. 
With the optitrack, this can be done by pressing the `n` key on the PC where the demo is running. You can also keep the 
metawear on and use the button while tracking the arm orientation with the optitrack.  

###  Optitrack Setup  
To use the optitrack, first start *Motive* on *Omen* windows PC and make sure the rigid body `hand` is being streamed. 
Then, from the linux *Demo* machine, launch the `optitrack-ros2-multi` container from docker compose UI.
Then, from Omen desktop launch `RebroadcastMotiveData.bat`, then press `m`, then `u`, then again `m` (there's a bug).
Now you should be able to see a topic `/optitrack/hand` published (if the hand is in the tracked area).  

### Metawear Setup  
To use the metawear, launch the `metawear_gabri` container from docker compose UI from the linux *Demo* machine. 

Note that there's a bug with the Bluetooth, so if the metawear vibrates, blinks red and disconnects immediately, 
you just have to launch the container again.  
To change the metawear to use, you need to change the MAC address in this file: `/home/Demos/launch/metawear_gabri/_main.launch` . 
The MAC address can be found attached to each metawear (label on the back).  

## LEDs strips and single LEDs  
To run the real world demos, you will need to launch also the driver for the LEDs strips and for the single LEDs.  

### LEDs strips driver  
To run the driver, first connect the LED board to the power (you will find it in the lab corner). Make sure the odroid 
board is connected via ethernet to the *drone-wifi* router. Then, from *drone-wifi*, connect to [192.168.201.32:9000](http://192.168.201.32:9000) 
and start `docker_led_strip_driver_1` container.

Note that due to a bug, you may need to restart it every time you start a demo.  

### Single LEDs driver  
To run the driver,  launch the `uros` container from docker compose UI from the linux *Demo* machine.

# Scenarios 1-2-3
Scenarios 1, 2 and 3 are all based on the same CoppeliaSim scene [place link here](TODO). Also, they correspond to the same
launch file [place link here](TODO). Here are described the single scenes and all the parameters available at launch time. 

Setting up a scenario is a matter of configuring CoppeliaSim scene GUIs as explained above and setting the launch parameters from ROS.
## Scenario 1 
In scenario 1 users can run the localization and then turn single LEDs on by pointing at them.
Once the demo starts, they can press the metawear button or `n` key on keyboard to start the interaction. Press again to localize again and so on.  

## Scenario 2  
In scenario 2, users can run the localization and then point at the LEDs strips to move a cursor over them.
Once the demo starts, they can press the metawear button or `n` key on keyboard to start the interaction. Press again to localize again and so on.  

## Scenario 3  
In scenario 3, users can localize and then select packages simulated on the LEDs strips.
From an interaction point of view, scenario 3 works exactly like scenario 2. To run it, just run the same launch files with the same parameters.

Then, you can add packages on the LEDs strips by   
calling the available service:
```
ros2 service call /add_pack conveyor_msgs/srv/AddPack "{'uid':1, 'kind':1}"
```
  
This will add a red package. You can also set the "belt" speed by calling:
```
ros2 service call /set_belt_speed conveyor_msgs/srv/SetBeltSpeed "{'speed': 1.}"
```
  
`speed` is in m/s. A value of 0 will stop the belt.  

## Scenario 4
In scenario 4 we are simulating Synesis conveyor in our lab. The interaction has the same setup as for scenarios 1-2-3, but
of course does not make much sense in real world (since we don't actually have the conveyor).

It uses a different map, which can be found [here](TODO).
### IROS task (Pointing User Interface - PUI)  
To run the task we developed for the IROS submission, you can launch the relevant launch file from `iros` package. 
This will work exactly like scenario 3, but will also start the task as soon as the user is localized. 
Parameters are the same as for scenario 2/3 (concerning the IMU and the localization), however, additional parameters are 
available to set up the task. You can find the instructions [here](../iros/README.md#configuration---gui--pui).

## Launch
The relevant launch file for scenarios 1, 2, 3 and 4 is [scenario_1_2_3_4_lab.launch](launch/scenario_1_2_3_4_lab.launch). This exposes the following parameters:
- `map_path` path to the `.yaml` map file. Default for our lab is [scenario_1_2_3_lab_map.yaml](../conveyor_utils/launch/scenario_1_2_3_lab_map.yaml). The one for scenario 4 is [this](TODO).
- `single_LED` `True` or `False`. If `True`, the pointing cursor will interact only with single LEDs (i.e. scenario 1).
  If `False`, the pointing cursor will interact with LEDs strips instead.
- `sim_imu` `True` or `False`. This must be `True` if we want to run just in simulation with Bill's IMU. `False` otherwise.
- `update_rate` rate for supervisor node update function. Can be ignored.
- `do_relloc` `True` or `False`. If `True`, pressing `n` key will trigger the relative localization before starting the interaction.
- `led_on_period` duration in seconds for which each LED must be On when doing the relloc.
- `publish_rate` publish rate of the metawear. Leave it to 50Hz.
- `gt_x`, `gt_y`, `gt_yaw` when the relloc is not done, those can be set to fix the human frame of reference in the world/optitrack frame.
Default values are 0, 0, 0 and correspond to optitrack origin, facing the back wall in the lab (the one in front of the door)
- `rotation_topic` IMU source for pointing reconstruction. Possible values are:
	- `/metawear_ros/rotation` for metawear bracelet
	- `/optitrack/imu` for optitrack hand marker
	- `/imu` for Bill's IMU from Coppelia. This must be used for simulation
- `user_kinematics` name of the file containing the user kinematics. It must be stored [there](../volaly_kinematics/config), 
  take [this](../volaly_kinematics/config/human_kinematics.yaml) as an example.
Available values are:
	- `human` right now is Gabri's kinematics
	- `bill` Bill's kinematics (to use in simulation)
- `time_for_selection` duration in seconds required to select/deselect a package when pointing at it
- `pointer_size` size of the pointer over the LEDs strips (m)
- `pointing_tol` If the pointing ray has a distance from the closest LEDs strip higher than `pointing_tol`, the pointer is not drawn
- `pointing_cmap` If a valid matplotlib colormap name is passed, the pointer color is set as a function of the ray-target distance and the colormap
- `sim_packages_on_LED` `True` or `False`. If `True`, packages will be simulated on LED strips by a ROS node (i.e. this should be `True` for scenarios 1,2 and 3).
If `False`, packages must be published by another source (e.g. Coppelia). This must be `False` in scenario 4, where Coppelia generates the packages.
## Go on within the demo
In all demos, users can "trigger" the next step by pressing the metawear button or `n` key on the keyboard. In general, the flow
will be as follows:

1. Launch the demo on ROS (and on Coppelia)
2. if `do_relloc==True` pressing `n`/metawear will move to 3, else to 4
3. perform relative localization procedure and go to 5
4. set the fixed transformation automatically. When using the metawear, this will also fix the yaw toward the direction being pointed at the moment.
   To fix it correctly, users must point in front of him while looking at the back wall in the lab (the one in front of the door).
5. start interaction, pressing `n`/metawear will go back to 2 
## Examples
### Real World
To run a demo in **real world**, reconstructing the pointing using **metawear imu**, doing the relative localization, launch:

```
ros2 launch relloc scenario_1_2_3_4_lab.launch do_relloc:=True user_kinematics:="human" sim_imu:=False rotation_topic:="/metawear_ros/rotation"
```

After the localization, users  will be able to move the cursor on the LEDs strips (scenario 2) and select packages (scenario 3).
To run scenario 1, add those parameters:
```
single_LED:=True pointing_cmap:="matplotlib valid colormap name"
# full example command:
ros2 launch relloc scenario_1_2_3_4_lab.launch do_relloc:=True user_kinematics:="human" sim_imu:=False rotation_topic:="/metawear_ros/rotation" single_LED:=True pointing_cmap:="viridis"
```

### Simulation
To run any scenario in simulation, this is the minimal required command:
```
ros2 launch relloc scenario_1_2_3_4_lab.launch user_kinematics:="bill" sim_imu:=True rotation_topic:="/imu"
```
Then, other parameters (such as `do_relloc`, `single_LED`, ...) must be set to match the desired behaviour as defined in
`Bill` and `Demo` GUIs within CoppeliaSim scene.

### Scenario 4
To run scenario 4 in simulation with Bill pointing, run:
```
ros2 launch relloc scenario_1_2_3_4_lab.launch user_kinematics:="bill" sim_imu:=True rotation_topic:="/imu" map_path:=TODO sim_packages_on_LED:=True 
```
Then, other parameters (such as `do_relloc`, `single_LED`, ...) must be set to match the desired behaviour as defined in
`Bill` and `Demo` GUIs within CoppeliaSim [scene](TODO)



### VR Setup
Both simulated (i.e. Bill doing the hard work) and real (i.e. actual user) interaction can be run in VR. Setting up the interaction type
it's just a matter of setting the parameters in Coppelia and at launch for ROS. CoppeliaSim scenes and ROS launch files are always the same,
so refer to the previous section for how to configure the scenario.

The only difference is that now CoppeliaSim must run also for interactions with a real world user, as it's needed as bridge between ROS and Unity.

Here are the steps to set up an example use case where an actual user tries scenario 2 with relative localization:
1. Turn on Oculus visor (long press power button on the right of the headset).
2. Mount the `Oculus` marker on the visor, with the arrow pointing away from the user wearing it.
3. On *Omen*: run Oculus app and turn on AirLink (Settings -> Beta -> AirLink).
4. From Oculus: wear the visor and connect to AirLink (from menu bar -> Settings (gear icon on the right) -> Quick Actions -> AirLink -> Launch and connect to Omen). 
   If AirLink is not visible, it must be enabled on the device itself: Settings -> Experimental Features -> AirLink.
5. On *Omen*: open Unity Project (Unity Hub (desktop icon) -> OptiUniNew).
6. On *Omen*: launch and config motive to stream `hand` and `Oculus` rigid bodies, using multicast.
7. Wear the hand marker on your right hand (as in the picture TODO).
8. On *Omen*: run `RebroadcastMotiveData.bat` from the desktop. Then press `m`, then `u`, then again `m` (there's a bug).
9. On *Omen*: run `run_coppelia.bat` from the desktop and load the relevant scene in Coppelia. For this example, [scenario_1_2_3_lab](TODO LINK HERE).
10. In Coppelia: check the `ignore Bill` flag in `Demo` config GUI (for this example).
11. Start Coppelia simulation
12. On any PC connected to drone-wifi, launch ROS nodes. For this example
```
ros2 launch relloc scenario_1_2_3_4_lab.launch do_relloc:=True user_kinematics:="human" sim_imu:=False rotation_topic:="/metawear_ros/rotation"
```
13. Press play on Unity
14. Wear Oculus and enjoy the VR world :)


Note that after the first setup, steps 1-9 are not needed anymore. After the initial setup, the scenario can be configured
from Coppelia and ROS launch files.

Again, all the other setups explained above apply: to track user pointing ray we still need to use either the metawear or the optitrack;
to turn on the actual LEDs, if one wants to, drivers must be launched, etc. etc.

### Tracking in VR
We are tracking user's head and right hand with the optitrack. This is needed to overlap the real world with the simulated environment
and to better track user's hand (Oculus hand tracking fails in some cases, like when we are pointing).

For the hand, this works like this: if the optitrack tracked hand is not visible, we take the hand tracking from Oculus (rendered as
a black wire-framed hand). If the optitrack tracking is available, we render the right hand as a gray hand with a "frozen position" (hand closed, index finger extend).

We track user's head so that we can render the real environment in simulation. The room is not rendered until 100 frames of tracking data for the `Oculus` motive
rigid body are available. After this, the transformation is set. To reset this transformation (in case it looks wrong), users need to perform a pinch gesture with their left hand
(which is always tracked by oculus), like this: when white circle is full a sound should play and the room transformation should be reset.

![Alt Text](pinch.gif)
