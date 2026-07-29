<a name="readme-top"></a>

[JA](README_ja.md) | [EN](README.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Uirobot Hardware

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#overview">Overview</a>
    </li>
    <li>
      <a href="#setup">Setup</a>
      <ul>
        <li><a href="#environment-requirements">Environment Requirements</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#configuration">Configuration</a></li>
    <li><a href="#milestones">Milestones</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#references">References</a></li>
  </ol>
</details>



<!-- OVERVIEW -->
## Overview

This repository provides a [`ros2_control`](https://github.com/ros-controls/ros2_control) [`SystemInterface`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp) implementation to operate [UIROBOT](https://www.uirobot.com/) actuators.
Currently, this package is designed to operate the lifter mechanism mounted on the dual-arm mobile manipulator [SOBIT HOME](https://github.com/TeamSOBITS/sobit_home/tree/jazzy-devel).

Documents such as datasheets are available [here](https://www.uirobot.com/serve/show-74.html).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- SETUP -->
## Setup

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Environment Requirements

Please make sure your environment meets the following conditions before proceeding to the installation step.

| System  | Version |
| --- | --- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco |
| Python | 3.12 |

> [!NOTE]
> For instructions on installing `Ubuntu` and `ROS`, please refer to the [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1. Go to your ROS `src` directory.
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. Clone this repository.
    ```sh
    $ git clone https://github.com/TeamSOBITS/uirobot_hardware
    ```

3. Navigate into the repository directory.
    ```sh
    $ cd uirobot_hardware/
    ```

4. Install dependent packages.
    ```sh
    $ bash install.sh
    ```

5. Build the package.
    ```sh
    $ cd ~/colcon_ws/
    $ colcon build --symlink-install
    $ source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONFIGURATION -->
## Configuration

1. Set up the actuator controller parameters in your robot's URDF.

| Parameter | Type | Example | Description |
| --- | --- | --- | --- |
| port_name | string | /dev/ttyUSB0 | USB port name |
| baud_rate | int | 115200 | Communication speed with Uirobot Motor |
| use_dummy | bool | true | Whether to use a dummy actuator |
| id | int | 1 | Uirobot Motor ID |
| gear_ratio | float | 1.0 | Gear ratio or linear actuator conversion ratio |
| position_kp | float | 0.7 | Gain adjustment parameter |
| stop_threshold | float | 0.00005 | Threshold value to stop when reaching target position |
| min_velocity | float | 0.01 | Minimum velocity limit (m/s) |
| max_velocity | float | 0.03 | Maximum velocity limit (m/s) |
| mode | string | JOG or PTP | Switch operating mode on the motor chip |


Here is an example configuration.
An actual usage example can be viewed [here](https://github.com/TeamSOBITS/sobit_home/blob/jazzy-devel/sobit_home_description/urdf/controllers.urdf.xacro).
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<robot name="robot_name" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="controllers">

    <ros2_control name="uirobot_control" type="system">
      <hardware>
        <plugin>uirobot_hardware/UirobotHardware</plugin>
        <param name="port_name">/dev/ttyUSB0</param>
        <param name="baud_rate">115200</param>
        <!-- <param name="use_dummy">true</param> -->
      </hardware>

      <joint name="joint_1">
        <param name="id">1</param>
        <!-- <param name="gear_ratio">2.0</param> -->
        <command_interface name="position">
          <param name="min">0.0</param>
          <param name="max">0.6</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
    </ros2_control>
  </xacro:macro>
</robot>
```

2. Next is the setting for `controller manager`. Modify the settings according to the number of controllers and actuators used.
An actual usage example can be viewed [here](https://github.com/TeamSOBITS/sobit_home/blob/jazzy-devel/sobit_home_control/config/controllers.yaml).
```yaml
/**/controller_manager:
  ros__parameters:
    update_rate: 10  # Hz

    joint_trajectory_controller: # JOG mode
      type: joint_trajectory_controller/JointTrajectoryController
    
    # joint_trajectory_controller: # PTP mode
    #   type: position_controllers/JointGroupPositionController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

/**/joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    allow_partial_joints_goal: true
```

3. Finally, launch the controllers along with the configured parameters when running the robot.
An actual usage example can be viewed [here](https://github.com/TeamSOBITS/sobit_home/blob/jazzy-devel/sobit_home_bringup/launch/robot.launch.py).
```py
robot_description = os.path.join(get_package_share_directory(
    'robot_description'), 
    'robots.urdf.xacro'
)
robot_description_config = xacro.process_file(
    robot_description
)

controller_config = os.path.join(get_package_share_directory(
    'robot_control'),
    'config',
    'real_controllers.yaml'
)

controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    namespace=robot_name,
    parameters=[controller_config],
    remappings=[
        ("controller_manager/robot_description", "robot_description"),
    ],
    output="screen",
)

joint_state_broadcaster = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller',
        '--set-state', 'active',
        '--controller-manager', robot_name+'/controller_manager',
        'joint_state_broadcaster'
    ],
    output='screen'
)

joint_trajectory_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller',
        '--set-state', 'active',
        '--controller-manager', robot_name+'/controller_manager',
        'joint_trajectory_controller'
    ],
    output='screen'
)

robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    namespace=robot_name,
    parameters=[
        {"frame_prefix": robot_name + '/'},
        {"robot_description": robot_description_config.toxml()},
    ],
    output="screen",
)
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MILESTONES -->
## Milestones

- [] ---

See the [Issues page][issues-url] to check current bugs or request new features.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- REFERENCES -->
## References

* [ROS Jazzy](https://docs.ros.org/en/jazzy/index.html)
* [ROS2 Control](https://control.ros.org/jazzy/index.html)
* [UIROBOT](https://www.uirobot.com/)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/uirobot_hardware.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/uirobot_hardware/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/uirobot_hardware.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/uirobot_hardware/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/uirobot_hardware.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/uirobot_hardware/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/uirobot_hardware.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/uirobot_hardware/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/uirobot_hardware.svg?style=for-the-badge
[license-url]: LICENSE