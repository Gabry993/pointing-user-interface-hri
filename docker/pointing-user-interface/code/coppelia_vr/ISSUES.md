List of issues for CoppeliaSim-VR-Toolbox
=========================================

We want to use the CoppeliaSim-VR-Toolbox (CoppeliaVRT) to connect CoppeliaSim to the VR-Headset (Oculus quest 2).

## Basics

- CoppeliaSim provides internal (LUA and C++) and external (C++, Python, ...) API.
- [CoppeliaVRT is open source](https://github.com/BorisBogaerts/CoppeliaSim-VR-Toolbox) and target windows as a Visual Studio project
- CoppeliaVRT uses CoppeliaSim remote C++ API.
- CoppeliaVRT uses VTK [8.2 but works with 9 too] for rendering. It creates a twin agent for each  coppeliaSim object.
- VTK can use CUDA for rendering.
- CoppeliaVRT uses the provided VTK interface to OpenVR to interface with SteamVR.
- CoppeliaVRT has also renderers which could be used for testing without VR.


## Missing features

CoppeliaVRT is working as expected on its demonstration scenes but it misses several features we need for one-swarm scenes.

### Dynamically created objects are not visualized

By default, CoppeliaVRT only updates objects created before the simulation starts.

Already available solution: by setting the integer-valued signal `dynamic_load_request`, from coppeliaSim we can tell CoppeliaVRT to reload all objects.

This should already be done by the [LUA script](https://github.com/BorisBogaerts/CoppeliaSim-VR-Toolbox/blob/80b59214612705dd3d8593aadb662cbd9e47208e/V-REP%20data/Scripts/HTC_VIVE.lua#L224)
```LUA
handles = simGetObjectsInTree(sim_handle_scene, sim.object_shape_type, 0)
if(numberOfObjects<#handles) then
sim.setIntegerSignal('dynamic_load_request',1)
end
```
but it is better to set it explicitly each time objects are created or destroyed.

### Color changes are not propagated from Coppelia to VR

CoppeliaVRT does not sync the color of objects, only their pose. Therefore we can not change objects' color as a way to simulate LEDs.

### Texture changes are not propagated from Coppelia to VR

CoppeliaVRT does not sync the texture of objects either. Therefore we can not change objects' textures as a way to simulate LEDs strips.

Adding support for color synchronization would allow us to implement LED strips as collection of color-changing shapes, *but* probably at an high run-time cost, as we would need to set/get thousands of colors shapes at each pass, both in LUA, through the remote API, and in CoppeliaVRT.

## Low performance

CoppeliaVRT is implemented as a stand-alone application that uses CoppeliaSim remote API (in C++) to continuously gather updates about the objects state (mostly poses) and pass them to twin VTK objects. VTK is then used to interface/render with OpenVR.

CoppeliaSim supports also plugins (C++), which run in the same coppeliaSim process and use an internal much more efficient API to communicate with CoppeliaSim.

The remote API could in principle allow CoppeliaVRT to run on a different machine, e.g. CoppeliaSim in Linux, CoppeliaVRT in Windows (to interface with the VR system), which would simplify our workflow but not by much, as we already installed ROS2 on windows and have all the [other] parts working.

The [plugin is hardcoded to use the local interface network](https://github.com/BorisBogaerts/CoppeliaSim-VR-Toolbox/blob/80b59214612705dd3d8593aadb662cbd9e47208e/V-REP%20VR%20Interface/VREP_VR_Interface.cxx#L51)
```cpp
clientID = simxStart((simxChar*)"127.0.0.1", portNb, true, true, -200000, 5);
```
so it does not take advantage of this possibility.


### Why does CoppeliaVRT uses the remote API?

### Could it be [re]written to use the internal API?
