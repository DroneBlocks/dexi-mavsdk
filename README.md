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

Parameter configuration scripts for EKF2 sensor fusion modes.

### GPS Mode (Default)

Standard mode using GPS for position. Use this for outdoor flight or to reset after testing vision mode.

```bash
python3 config/enable_gps.py
```

Sets: `EKF2_GPS_CTRL=7`, `EKF2_EV_CTRL=0`, `EKF2_HGT_REF=1` (GPS height)

### Vision Mode (No GPS)

Disables GPS and enables visual odometry fusion. Use with `apriltag_odometry` for indoor flight or GPS-denied environments.

```bash
python3 config/disable_gps.py
```

Sets: `EKF2_GPS_CTRL=0`, `EKF2_EV_CTRL=1` (horizontal position only), `EKF2_HGT_REF=0` (barometer)

**Warning:** In vision mode, the drone will drift if it loses sight of the AprilTag.

### Debug

```bash
# Monitor visual odometry messages being received by PX4
python3 config/debug_vision_odom.py
```

## Network Configuration

- PX4 SITL runs in `px4-sitl` container at `172.20.0.4`
- MAVLink is sent to `code-server` container at `172.20.0.8:14540`
- ROS2 uses uXRCE-DDS on port 8888 (separate from MAVLink)

## Notes

- MAVSDK and ROS2 can run simultaneously - they use different protocols
- MAVLink: Direct drone control via MAVSDK (code-server container)
- uXRCE-DDS: PX4 ROS2 topics via micro-ros-agent (ros2-dev container)
