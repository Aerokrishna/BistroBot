# BistroBot

An autonomous food-delivery robot for restaurants and food courts, built on **ROS 2 Humble** and **Nav2**. Staff load up to three trays, assign tables through a web UI or by voice, and the robot drives a multi-stop delivery route on its own. It has been tested in a real food court.

## Capabilities

- **Autonomous navigation**: Nav2 (NavFn planner, DWB controller) on a pre-built map, with AMCL localization and recovery behaviors.
- **Multi-stop delivery**: a custom action server takes a sequence of waypoints and visits them in order, with live feedback and cancellation.
- **Mapping**: SLAM Toolbox builds new maps while the robot is driven by joystick.
- **Sensor fusion**: an EKF (`robot_localization`) fuses wheel odometry with BNO055 IMU yaw.
- **Speech commands**: say *"go to table one then table five"*. The audio is transcribed and the locations are mapped to goal poses.
- **Web UI**: a tablet or phone UI for assigning delivery locations to each tray.
- **Manual override**: PS4/joystick teleoperation.

## Architecture

```
 ┌────────────── Web UI (Flask :3000) ─────────────┐     ┌───── Speech ─────┐
 │ Tray 1..3 → select locations → Start Delivery   │     │ Mic → ASR API    │
 └───────────────────────┬─────────────────────────┘     │ → spaCy matcher  │
                         │ DeliveryPlan.srv              │ (or GPT-4 text)  │
                         ▼                               └────────┬─────────┘
                ┌──────────────────────────┐   PynavGoal.action   │
                │  Navigation Manager      │◄─────────────────────┘
                │  (kpbot/navigation_goal) │
                └────────────┬─────────────┘
                             ▼
        ┌──────────── Nav2 (planner, controller, costmaps, AMCL) ───────────┐
        │   map ◄── RPLidar /scan      EKF ◄── /odom + /imu                  │
        └────────────┬──────────────────────────────────▲───────────────────┘
                     │ /cmd_vel                         │
              wheel_control.py (PID)              odometry.py
                     │ /wheel_pwm                       │ /sensor
 ════════════════════╪══════ micro-ROS agent (serial /dev/ttyACM0) ═══════════
                     ▼                                  │
              ┌──────────── Raspberry Pi Pico (micro-ROS) ─────────────┐
              │  Motor drivers (PWM + DIR)   Encoders (IRQ)   BNO055   │
              └────────────────────────────────────────────────────────┘
```

**Hardware:** a Raspberry Pi runs ROS 2, Nav2 and the RPLidar driver. A Raspberry Pi Pico handles real-time I/O over micro-ROS. The base is a differential drive (wheel radius 0.035 m, track width 0.33 m, 600 PPR encoders).

## Packages

| Package | Purpose |
|---|---|
| `kpbot_description` | URDF/xacro model, RViz config, robot state publisher |
| `kpbot_nav2` | Launch files, Nav2/EKF/SLAM params, maps, odometry, wheel PID and joystick nodes |
| `kpbot_lingonav` | Navigation manager action server, speech and NLP clients |
| `kpbot_interface` | `PynavGoal.action`, `DeliveryPlan.srv`, `WheelPWM` / `Sensor` msgs |
| `bistrobot_ui` | Flask web UI that bridges to ROS 2 |
| `pio_microros` | PlatformIO firmware for the Pico (micro-ROS, motors, encoders, IMU) |

## MCU / micro-ROS

The Pico firmware ([pio_microros/uros_base](pio_microros/uros_base)) runs a micro-ROS node over USB serial:

- **Publishes** `/sensor` (`kpbot_interface/Sensor`) at 10 Hz: encoder ticks and IMU yaw.
- **Subscribes** to `/wheel_pwm` (`kpbot_interface/WheelPWM`) and drives the motors.

On the host, `odometry.py` turns `/sensor` into `/odom` and `/imu`, and `wheel_control.py` turns `/cmd_vel` into closed-loop wheel PWM.

```bash
cd pio_microros/uros_base && pio run -t upload
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

## Speech integration

`kpbot_nav_client.py` records from the microphone, sends the audio to the IIT Madras ASR API and extracts known locations with a spaCy `PhraseMatcher`. It then sends the resulting waypoint sequence to the navigation manager. `lingonav_gpt.py` is an alternative that parses free-form text commands with GPT-4.

## Usage

```bash
colcon build && source install/setup.bash

ros2 launch kpbot_nav2 kpbot_mapping.launch.py      # build a map (joystick driving)
ros2 launch kpbot_nav2 kpbot_navigation.launch.py   # localize + Nav2 on food_court map
ros2 launch kpbot_nav2 kpbot_joy_control.launch.py  # teleop only

ros2 run kpbot_lingonav kpbot_nav_manager.py        # delivery action server
ros2 run kpbot_lingonav kpbot_nav_client.py         # voice command client
ros2 run bistrobot_ui user_client.py                # web UI → http://<robot-ip>:3000
```

The RPLidar driver (`rplidar_ros`) must be running for mapping and navigation.

## Supported

- ROS 2 Humble, Ubuntu 22.04
- Raspberry Pi with RPLidar, and a Raspberry Pi Pico (RP2040) as the MCU
- Differential-drive base with quadrature encoders and a BNO055 IMU
- English voice commands and any browser for the UI

## Future work

- A `start_delivery` service server that links the UI's tray plan to the navigation manager
- Tray-aware stops: wait at each table until the order is picked up, then continue
- On-device (offline) ASR, and a configurable location database in place of hard-coded coordinates
- Fleet coordination and order-system/POS integration
- Obstacle-aware speed limits in crowds, plus audio/visual cues for customers
- Docking and auto-charging, and a battery status topic
- Gazebo simulation for testing without hardware
