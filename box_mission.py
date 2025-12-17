#!/usr/bin/env python3
"""
MAVSDK example: Takeoff and fly a 1m box pattern.

Works with PX4 SITL simulation and DEXI drone.

This script uses offboard mode to:
1. Wait for connection
2. Arm the drone
3. Takeoff to 2.5 meters
4. Fly a box pattern (forward, right, back, left) in 1m increments
5. Land

Usage:
    python3 box_mission.py

The script listens on UDP port 14540 for MAVLink.
"""

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


def body_to_ned(forward, right, yaw_deg):
    """Convert body-frame offsets to NED offsets based on yaw heading."""
    yaw_rad = math.radians(yaw_deg)
    north = forward * math.cos(yaw_rad) - right * math.sin(yaw_rad)
    east = forward * math.sin(yaw_rad) + right * math.cos(yaw_rad)
    return north, east


async def fly_to_position(drone, north, east, down, yaw, description):
    """Fly to a position and wait to arrive."""
    print(f"{description} (N={north:.1f}m, E={east:.1f}m)...")
    await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
    await asyncio.sleep(3)


async def run():
    drone = System()

    print("Connecting to drone (listening on port 14540)...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    # Wait for connection
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Wait for drone to be ready to arm
    print("Waiting for drone to be ready to arm...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("Drone is ready to arm!")
            break

    # Arm the drone
    print("Arming...")
    await drone.action.arm()
    print("Armed!")

    # Takeoff
    print("Taking off to 2.5m...")
    await drone.action.set_takeoff_altitude(2.5)
    await drone.action.takeoff()

    # Wait for takeoff to complete (check altitude)
    print("Waiting to reach altitude...")
    async for position in drone.telemetry.position():
        if position.relative_altitude_m > 2.0:
            print(f"Reached altitude: {position.relative_altitude_m:.1f}m")
            break

    # Get current position for reference
    print("Getting current position...")
    async for position in drone.telemetry.position_velocity_ned():
        start_north = position.position.north_m
        start_east = position.position.east_m
        start_down = position.position.down_m
        print(f"Start position: N={start_north:.1f}m, E={start_east:.1f}m, D={start_down:.1f}m")
        break

    # Get current yaw to maintain heading
    print("Getting current heading...")
    async for attitude in drone.telemetry.attitude_euler():
        start_yaw = attitude.yaw_deg
        print(f"Current yaw: {start_yaw:.1f}°")
        break

    # Set initial setpoint (current position) before starting offboard mode
    print("Setting initial offboard setpoint...")
    await drone.offboard.set_position_ned(PositionNedYaw(
        start_north, start_east, start_down, start_yaw
    ))

    # Start offboard mode
    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard start failed: {error._result.result}")
        print("Landing...")
        await drone.action.land()
        return

    # Fly box pattern (1m sides) in body frame
    # Forward/back and left/right are relative to drone's heading

    # 1. Fly forward 1m (body frame)
    d_north, d_east = body_to_ned(1.0, 0.0, start_yaw)
    pos1_north = start_north + d_north
    pos1_east = start_east + d_east
    await fly_to_position(drone,
        pos1_north, pos1_east, start_down, start_yaw,
        "Flying forward 1m")

    # 2. Fly right 1m (body frame)
    d_north, d_east = body_to_ned(0.0, 1.0, start_yaw)
    pos2_north = pos1_north + d_north
    pos2_east = pos1_east + d_east
    await fly_to_position(drone,
        pos2_north, pos2_east, start_down, start_yaw,
        "Flying right 1m")

    # 3. Fly back 1m (body frame)
    d_north, d_east = body_to_ned(-1.0, 0.0, start_yaw)
    pos3_north = pos2_north + d_north
    pos3_east = pos2_east + d_east
    await fly_to_position(drone,
        pos3_north, pos3_east, start_down, start_yaw,
        "Flying back 1m")

    # 4. Fly left 1m (body frame) - back to start
    d_north, d_east = body_to_ned(0.0, -1.0, start_yaw)
    pos4_north = pos3_north + d_north
    pos4_east = pos3_east + d_east
    await fly_to_position(drone,
        pos4_north, pos4_east, start_down, start_yaw,
        "Flying left 1m (back to start)")

    print("Box pattern complete!")

    # Stop offboard mode
    print("Stopping offboard mode...")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Offboard stop failed: {error._result.result}")

    # Land
    print("Landing...")
    await drone.action.land()

    # Wait for landing
    print("Waiting for landing...")
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Landed!")
            break

    # Disarm
    print("Disarming...")
    await drone.action.disarm()
    print("Done!")


if __name__ == "__main__":
    asyncio.run(run())
