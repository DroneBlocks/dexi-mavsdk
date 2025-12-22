#!/usr/bin/env python3
"""
Hybrid takeoff using action.takeoff(), then offboard velocity for hover/control.

This uses PX4's internal takeoff (which works with current EKF settings),
then switches to offboard velocity control once airborne.

Usage:
    python3 velocity_takeoff_land.py
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed


# Flight parameters
TAKEOFF_ALT = 1.5         # meters
HOVER_TIME = 5.0          # seconds
SETPOINT_INTERVAL = 0.02  # 50 Hz


async def run():
    drone = System()

    print("Connecting to drone...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    # Wait for drone to be ready
    print("Waiting for drone to be ready...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("Drone is ready!")
            break

    # Arm
    print("Arming...")
    await drone.action.arm()
    print("Armed!")

    # Use action.takeoff() - this uses PX4's internal control
    print(f"Taking off to {TAKEOFF_ALT}m using action.takeoff()...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT)
    await drone.action.takeoff()

    # Wait for takeoff to complete
    print("Waiting to reach altitude...")
    await asyncio.sleep(5)  # Give it time to climb

    # Check altitude
    async for position in drone.telemetry.position():
        print(f"Current altitude: {position.relative_altitude_m:.2f}m")
        break

    # Now try offboard for hover control
    print("\nSwitching to offboard mode for hover...")

    # Prime with setpoints
    print("Priming setpoints...")
    for i in range(50):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(SETPOINT_INTERVAL)

    try:
        await drone.offboard.start()
        print("Offboard mode started!")

        # Hover with continuous setpoints
        print(f"Hovering for {HOVER_TIME} seconds...")
        end_time = asyncio.get_event_loop().time() + HOVER_TIME
        while asyncio.get_event_loop().time() < end_time:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
            await asyncio.sleep(SETPOINT_INTERVAL)

        print("Stopping offboard...")
        await drone.offboard.stop()

    except OffboardError as e:
        print(f"Offboard failed: {e}")

    # Land using action.land()
    print("Landing...")
    await drone.action.land()

    # Wait for landing
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Landed!")
            break

    print("Done!")


if __name__ == "__main__":
    asyncio.run(run())
