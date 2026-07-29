<a name="readme-top"></a>

[JA](README_ja.md) | [EN](README.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Uirobot Hardware

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#環境構築">環境構築</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#設定方法">設定方法</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>



<!-- 概要 -->
## 概要

本リポジトリは[UIROBOT](https://www.uirobot.com/)アクチュエータを動作させるための[`ros2_control`](https://github.com/ros-controls/ros2_control)の[`SystemInterface`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp)を提供します.
現レポジトリは，双腕型モバイルマニピュレータ[SOBIT HOME](https://github.com/TeamSOBITS/sobit_home/tree/jazzy-devel)に搭載されている昇降機構を動かすためのパッケージとなっています．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- セットアップ -->
## セットアップ

ここで，本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| --- | --- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco |
| Python | 3.12 |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)に参照してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法

1. ROSの`src`フォルダに移動します．
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. 本レポジトリをcloneします．
    ```sh
    $ git clone https://github.com/TeamSOBITS/uirobot_hardware
    ```

3. レポジトリの中へ移動します．
    ```sh
    $ cd uirobot_hardware/
    ```

4. 依存パッケージをインストールします．
    ```sh
    $ bash install.sh
    ```

5. パッケージをコンパイルします．
    ```sh
    $ cd ~/colcon_ws/
    $ colcon build --symlink-install
    $ source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 設定方法 -->
## 設定方法

1. ロボットのURDFにアクチュエータのコントローラを設定します．

| パラメータ | 型 | 一例 | 説明 |
| --- | --- | --- | --- |
| port_name | string | /dev/ttyUSB0 | USBポート名 |
| baud_rate | int | 115200 | Uirobot Motorとの通信速度 |
| use_dummy | bool | true | ダミーアクチュエータを使用するかどうか |
| id | int | 1 | Uirobot MotorのID |
| gear_ratio | float | 1.0 | ギア比やリニアアクチュエータ変換比 |
| position_kp | float | 0.7 | ゲインの調整 |
| stop_threshold | float | 0.00005 | 目標位置に対しストップするときのしきい値 |
| min_velocity | float | 0.01 | 速度の下限値(m/s) |
| max_velocity | float | 0.03 | 速度の上限値(m/s) |
| mode | string | JOG or PTP | モータのチップにあるモードの切り替え |


設定の一例はこちらとなります．
実際の使用例は，[こちら](https://github.com/TeamSOBITS/sobit_home/blob/jazzy-devel/sobit_home_description/urdf/controllers.urdf.xacro)から閲覧できます．
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

2. 次は`controller manager`の設定となります．使用されるコントローラ・アクチュエータの数に応じて設定を変更してください．
実際の使用例は，[こちら](https://github.com/TeamSOBITS/sobit_home/blob/jazzy-devel/sobit_home_control/config/controllers.yaml)から閲覧できます．
```yaml
/**/controller_manager:
  ros__parameters:
    update_rate: 10  # Hz

    joint_trajectory_controller: #JOGモード
      type: joint_trajectory_controller/JointTrajectoryController
    
    # joint_trajectory_controller: #PTPモード
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

3. 最後に，設定したパラメータ等をロボットを実行する際に，コントローラを立ち上げます．
実際の使用例は，[こちら](https://github.com/TeamSOBITS/sobit_home/blob/jazzy-devel/sobit_home_bringup/launch/robot.launch.py)から閲覧できます．
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

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- マイルストーン -->
## マイルストーン

- [] ---

現時点のバッグや新規機能の依頼を確認するために[Issueページ][issues-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 参考文献 -->
## 参考文献

* [ROS Jazzy](https://docs.ros.org/en/jazzy/index.html)
* [ROS2 Control](https://control.ros.org/jazzy/index.html)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



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
