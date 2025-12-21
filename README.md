# DEXI MAVSDK Examples

Simple MAVSDK examples for controlling DEXI drone via MAVLink.

## Setup

```bash
pip3 install --break-system-packages mavsdk
```

## Directory Structure

```
dexi-mavsdk/
├── missions/      # Autonomous flight scripts
├── control/       # Manual/interactive control
└── config/        # Parameter configuration
```

## Missions

Autonomous flight scripts that takeoff, execute a task, and land.

```bash
# Basic takeoff to 2.5m and land
python3 missions/takeoff_and_land.py

# Velocity-based takeoff/land (bypasses position hold issues)
python3 missions/velocity_takeoff_land.py

# Takeoff, fly forward 2m, and land
python3 missions/takeoff_fly_forward_land.py

# Fly a 1m box pattern
python3 missions/box_mission.py
```

## Control

Interactive and manual control scripts.

```bash
# Keyboard control (WASD-style)
python3 control/keyboard_control.py

# Simple movement test
python3 control/move_left.py
```

## Config

Parameter configuration scripts.

```bash
# Configure EKF2 for vision-based navigation
python3 config/configure_vision_ekf.py

# Disable GPS (vision-only mode)
python3 config/disable_gps.py

# Re-enable GPS
python3 config/enable_gps.py
```

## Network Configuration

- PX4 SITL runs in `px4-sitl` container at `172.20.0.4`
- MAVLink is sent to `code-server` container at `172.20.0.8:14540`
- ROS2 uses uXRCE-DDS on port 8888 (separate from MAVLink)

## Notes

- MAVSDK and ROS2 can run simultaneously - they use different protocols
- MAVLink: Direct drone control via MAVSDK (code-server container)
- uXRCE-DDS: PX4 ROS2 topics via micro-ros-agent (ros2-dev container)
