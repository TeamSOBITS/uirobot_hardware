^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uirobot_hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-04-14)
------------------
* add CAN port support for Uirobot hardware communication
* implement position-control flow with `set_pos`, `set_vel`, and `move` commands
* add target position clamping, velocity limiting, and minimum velocity handling
* improve hardware reply validation and joint CPR / limit logging
* handle transient zero-position reads from the gateway more safely
* add utility scripts for brake check, brake release, and origin setting
* install the utility scripts via CMake
* TODO: send the brake-off signal directly inside `uirobot_hardware.cpp`
* TODO: expand support for a larger velocity range
* TODO: add dedicated speed-control and position-control scripts

0.0.0 (2024-04-24)
------------------
