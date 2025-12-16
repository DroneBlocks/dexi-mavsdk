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
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


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

    # Set initial setpoint (current position) before starting offboard mode
    print("Setting initial offboard setpoint...")
    await drone.offboard.set_position_ned(PositionNedYaw(
        start_north, start_east, start_down, 0.0
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

    # Fly box pattern (1m sides)
    # NED frame: North = forward, East = right, Down = down

    # 1. Fly forward 1m (positive North)
    await fly_to_position(drone,
        start_north + 1.0, start_east, start_down, 0.0,
        "Flying forward 1m")

    # 2. Fly right 1m (positive East)
    await fly_to_position(drone,
        start_north + 1.0, start_east + 1.0, start_down, 0.0,
        "Flying right 1m")

    # 3. Fly back 1m (back to start North)
    await fly_to_position(drone,
        start_north, start_east + 1.0, start_down, 0.0,
        "Flying back 1m")

    # 4. Fly left 1m (back to start East)
    await fly_to_position(drone,
        start_north, start_east, start_down, 0.0,
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
