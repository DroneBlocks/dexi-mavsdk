# DEXI MAVSDK Examples

Simple MAVSDK examples for controlling DEXI drone via MAVLink.

## Setup

From the code-server container terminal:

```bash
pip3 install --break-system-packages mavsdk
```

## Examples

### takeoff_and_land.py

Basic takeoff to 2.5m and land:

```bash
cd ~/dexi-mavsdk
python3 takeoff_and_land.py
```

## Network Configuration

- PX4 SITL runs in `px4-sitl` container at `172.20.0.4`
- MAVLink is sent to `code-server` container at `172.20.0.8:14540`
- ROS2 uses uXRCE-DDS on port 8888 (separate from MAVLink)

## Notes

- MAVSDK and ROS2 can run simultaneously - they use different protocols
- MAVLink: Direct drone control via MAVSDK (code-server container)
- uXRCE-DDS: PX4 ROS2 topics via micro-ros-agent (ros2-dev container)
