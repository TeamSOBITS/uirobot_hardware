<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Dynamixel Hardware

<!-- Table of Contents -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#introduction">Introduction</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#setup">Setup</a></li>
    <li><a href="#milestone">Milestone</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#references">References</a></li>
  </ol>
</details>



## Introduction

This repository provides a [`ros2_control`](https://github.com/ros-controls/ros2_control) implementation with the [`SystemInterface`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp) for operating [ROBOTIS Dynamixel](https://emanual.robotis.com/docs/en/dxl/) actuators.

With the architecture of `ros2_control`, it is expected to support all Dynamixel actuators.

> [!IMPORTANT]
> Unlike the original package, this fork supports multiple control methods for different actuators. It also supports gear ratio and offset settings. However, switching control methods at runtime is not supported.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Getting Started

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Prerequisites

First, please set up the following environment before proceeding to the next installation stage.

| System  | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill |
| Python | 3.10 |
| Docker | latest |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1. Go to the `src` folder of ROS.
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. Clone this repository.
    ```sh
    $ git clone https://github.com/TeamSOBITS/dynamixel_hardware
    ```

3. Navigate into the repository.
    ```sh
    $ cd dynamixel_hardware/
    ```

4. Install the dependent packages.
    ```sh
    $ bash install.sh
    ```

5. Compile the package.
    ```sh
    $ cd ~/colcon_ws/
    $ colcon build --symlink-install
    $ source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Setup

1. Set up the actuator controllers in your robot's URDF.

| Parameter    | Type   | Example      | Description                        |
| ------------ | ------ | ----------- | ---------------------------------- |
| port_name    | string | /dev/ttyUSB0 | USB port name                      |
| baud_rate    | int    | 1000000      | Dynamixel communication speed      |
| use_dummy    | bool   | true         | Whether to use dummy actuators     |
| id           | int    | 1            | Dynamixel ID                       |
| control_mode | int    | 0            | Control mode                       |
| interface    | string | rs           | Interface type                     |

Control modes are as follows:
| `control_mode`                | Value |
| ----------------------------- | ----- |
| Position Control              | 0     |
| Velocity Control              | 1     |
| Torque Control                | 2     |
| Current Control               | 3     |
| Extended Position Control     | 4     |
| Multi Turn Control            | 5     |
| Current Based Position Control| 6     |
| PWM Control                   | 7     |

Interfaces are as follows:
| `interface` | Value |
| ----------- | ----- |
| RS485       | rs    |
| TTL         | ttl   |

You can also set parameters provided by Dynamixel Workbench, such as `Goal_Current`. For more details, see [Dynamixel Workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench/blob/humble/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_item.cpp#L27-L119).

Here is an example configuration:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<robot name="robot_name" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="controllers">

    <ros2_control name="dynamixel_control" type="system">
      <hardware>
        <plugin>dynamixel_hardware/DynamixelHardware</plugin>
        <param name="port_name">/dev/ttyUSB0</param>
        <param name="baud_rate">1000000</param>
        <!-- <param name="use_dummy">true</param> -->
      </hardware>

      <!-- Position Control -->
      <joint name="joint_1">
        <param name="id">1</param>
        <param name="control_mode">0</param>
        <!-- <param name="gear_ratio">2.0</param> -->
        <param name="interface">rs</param>
        <command_interface name="position">
          <param name="min">-3.141592654</param>
          <param name="max">3.141592654</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>

      <!-- Velocity Control -->
      <joint name="joint_2">
        <param name="id">2</param>
        <param name="control_mode">0</param>
        <param name="interface">rs</param>
        <command_interface name="velocity"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>

      <!-- Current Based Position Control -->
      <joint name="joint_3">
        <param name="id">3</param>
        <param name="control_mode">6</param>
        <param name="Goal_Current">50</param>
        <param name="interface">rs</param>
        <param name="gear_ratio">-59.504383354</param>
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      ...

    </ros2_control>
  </xacro:macro>
</robot>
```

2. Next, configure the `controller manager`. Adjust the settings according to the number of controllers and actuators you use.
```yaml
/**/controller_manager:
  ros__parameters:
    update_rate: 10  # Hz

    velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

/**/velocity_controller:
  ros__parameters:
    joints:
      - joint_2

/**/joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_3

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    allow_partial_joints_goal: true
```

3. Finally, launch the robot with the configured parameters.
```python
robot_description = os.path.join(get_package_share_directory(
    'robots_description'), 
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

velocity_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller',
        '--set-state', 'configured',
        '--controller-manager', robot_name+'/controller_manager',
        'velocity_controller'
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


## Milestone

- [x] Support for configuring control methods for different actuators
- [x] Support for setting gear ratios
- [] Switching between multiple control methods

See the [open issues][issues-url] for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## References

* [Dynamixel Hardware](https://github.com/dynamixel-community/dynamixel_hardware)
* [Dynamixel Workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench/)
* [ROS Humble](https://docs.ros.org/en/humble/index.html)
* [ROS2 Control](https://control.ros.org/humble/index.html)
* [ROS2 Control Gazebo](https://github.com/ros-controls/gz_ros2_control)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/dynamixel_hardware.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/dynamixel_hardware/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/dynamixel_hardware.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/dynamixel_hardware/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/dynamixel_hardware.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/dynamixel_hardware/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/dynamixel_hardware.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/dynamixel_hardware/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/dynamixel_hardware.svg?style=for-the-badge
[license-url]: LICENSE
