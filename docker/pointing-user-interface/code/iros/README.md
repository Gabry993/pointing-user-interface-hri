# GUI vs PUI
This package contains the code to run our experiments for the IROS submission.
In particular, we have one task that can be configured and then solved using
two different interfaces: a Graphical User Interface (GUI) and a Pointing User Interface (PUI).

## Task
The task is to select all defected packages over a conveyor belt. In the paper context,
we simulate the conveyor and the packages using LEDs strips. So, the task becomes to select
all the "red" packages over the LEDs strips.
### Selection - GUI
Using the GUI, users have to click on the package representation on the screen
to select it.
### Selection - PUI
Using the PUI, users have to move the pointing cursor (drawn on the LEDs strips)
over a package for a defined amount of time to select and de-select it.
### Configuration - GUI & PUI
Task parameters can be configured at launch time:
- `map_path` location of the yaml file containing the LEDs map
- `belt_speed` speed at which the belt (and packages) will run (m/s)
- `package_length` size of the packages side (m)
- `n_packs` number of total packages that will be spawned for each task run
- `n_reds` number of red packages that will be spawned for each task run (can't be more than `n_packs`)
- `n_runs` number of run before considering the task completed
- `log` If True, the task will be logged in a file
- `log_rate` if the task is logged, log rate
- `exp_name` experiment name, useful to store different logs
- `log_path` where the log must be stored
- `interface` can be `PUI` or `GUI`. Only for log purposes.
### Configuration - PUI only
- `time_for_selection` duration in seconds required to select/deselect a package when pointing at it
- `pointer_size` size of the pointer over the LEDs strips (m)
- `pointing_tol` If the pointing ray has a distance from the closest LEDs strip higher than `pointing_tol`, the pointer is not drawn
- `pointing_cmap` If a valid matplotlib colormap name is passed, the pointer color is set as a function of the ray-target distance and the colormap
### Configuration - Others
Other parameters are needed to set other aspects of the PUI interaction (e.g. perform relative localization, run in simulation/real/vr, etc.).
All of them are explained [here](../relloc/README.md).
There is it also explained how the setup other requirements (IMU, LEDs drivers, etc.) 

## Launch GUI:
The GUI task can be launched like this:
```ros2 launch iros task_gui.launch```
This will open the GUI which will be empty. Press `n` key on the keyboard to start the interaction.
After a few seconds the packages will appear both on the LEDs and on the GUI.
Press `n` again to terminate a run and move to the next. After the last of `n_runs` is over, pressing `n` won't do anything.

## Launch PUI:
The PUI task can be launched like this:
```ros2 launch iros task_pui.launch```
Refer to scenario 3 [here](../relloc/README.md#scenario-3) to see how to setup the PUI and go on with the interaction.