List of TODOs to make CoppeliaSim-VR-Toolbox usable
=========================================

- [x] Make a testing scene (Jerome)
  - dynamically created and destroyed objects
  - objects with changing colors
  - objects with changing textures

- [x] Describe the issue to CoppeliaVRT author Boris Bogaerts and seek advice and answers to the open questions (Jerome)

- [x] Compile the tool box as-it-is (Jerome)
  - [x] install vtk with vcpkg
    - `vpkg.exe install vtk[mpi,openvr,opengl,cuda,vtkm]:x64-windows`
    - `vtk[all]` is not installing because it needs `ospray` which is not available yet in vcpkg.
  - [x] clone and correct the toolbox code:
    - [x] import in VS2019
    - [x] added include paths (vtk, coppeliaSim, ...)
    - [x] added missing defines
    - [x] one VTK api is deprecated, added define to ignore the warning.
    - [x] removed missing .rc (only for Release)
    - [x] removed missing `vrep_plot_container.{cpp,h}` and their usage in the rest of the code
    - [x] added [missing] `extApiInternal.h` from one of the coppeliaSim repos.
  - it works more or less the same as the installed version :-), fps about 75 on the test scene. Without CUDA (VTK), it's down to about 35 fps.
  - we now have a binary which renders the scene on a window and which we can use to test without VR.


- [ ] Add explicit `dynamic_load_request` on our side.

- [ ] Maybe, make the `dynamic_load_request` more efficient by telling CoppeliaVRT which objects are new/deleted.

- [ ] Tell CoppeliaVRT which (and how) objects changed colors (wither via its LUA module or via one or more signals).
  - [ ] pass the color change to VKT

- [ ] Tell CoppeliaVRT which (and how) objects changed texture (wither via its LUA module or via one or more signals). This could mimic the way Vision Sensor are currenly exposed/rendered in CoppeliaVRT.
  - [ ] pass the texture change to VKT
