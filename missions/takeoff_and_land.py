#!/usr/bin/env python3
"""
MAVSDK example: Takeoff and land.

Works with PX4 SITL simulation and DEXI drone.

This script performs:
1. Wait for connection
2. Arm the drone
3. Takeoff to 2.5 meters
4. Hover for 5 seconds
5. Land

Usage:
    python3 takeoff_and_land.py

The script listens on UDP port 14540 for MAVLink.
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed


async def run():
    # Connect to PX4 SITL (px4-sitl container IP)
    drone = System()

    # Listen on UDP port 14540 for MAVLink
    print("Connecting to drone (listening on port 14540)...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    # Wait for connection
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # PX4 SITL requires offboard setpoints BEFORE arming (no RC controller)
    # Send initial setpoint to establish offboard control signal
    print("Sending initial offboard setpoint (required for SITL arming)...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))

    # Start offboard mode to send heartbeat - this allows arming in SITL
    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard start failed (expected on ground): {error._result.result}")

    # Give PX4 time to recognize the offboard signal
    await asyncio.sleep(1.5)

    # Now arm the drone (offboard signal allows arming without RC)
    print("Arming...")
    await drone.action.arm()
    print("Armed!")

    # Stop offboard so we can use action.takeoff()
    try:
        await drone.offboard.stop()
    except OffboardError:
        pass

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

    # Hover for 5 seconds
    print("Hovering for 5 seconds...")
    await asyncio.sleep(5)

    # Land
    print("Landing...")
    await drone.action.land()

    # Wait for landing
    print("Waiting for landing...")
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Landed!")
            break

    # Disarm (optional, PX4 auto-disarms after landing)
    print("Disarming...")
    await drone.action.disarm()
    print("Done!")


if __name__ == "__main__":
    asyncio.run(run())
