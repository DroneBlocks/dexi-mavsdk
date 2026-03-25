#!/usr/bin/env python3
"""
MAVSDK example: Arm for 5 seconds, then disarm.

Works with PX4 SITL simulation and DEXI drone.

This script performs:
1. Wait for connection
2. Arm the drone
3. Wait 5 seconds
4. Disarm

Usage:
    python3 arm_and_disarm.py

The script listens on UDP port 14540 for MAVLink.
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed


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

    # Send initial offboard setpoint (required for arming)
    print("Sending initial offboard setpoint...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))

    # Start offboard mode
    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard start failed: {error._result.result}")

    await asyncio.sleep(1.5)

    # Arm
    print("Arming...")
    await drone.action.arm()
    print("Armed!")

    # Wait 5 seconds
    print("Waiting 5 seconds...")
    await asyncio.sleep(5)

    # Disarm
    print("Disarming...")
    await drone.action.disarm()
    print("Disarmed!")

    # Stop offboard
    try:
        await drone.offboard.stop()
    except OffboardError:
        pass

    print("Done!")


if __name__ == "__main__":
    asyncio.run(run())
