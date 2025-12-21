#!/usr/bin/env python3
"""
Velocity-based takeoff, hover, and land.

Uses offboard velocity commands instead of action.takeoff() to avoid
position hold issues with EKF2_HGT_REF settings.

1. Arm
2. Climb using upward velocity command
3. Hover for 5 seconds (zero velocity)
4. Descend using downward velocity command
5. Disarm

Usage:
    python3 velocity_takeoff_land.py
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed


# Flight parameters
CLIMB_VELOCITY = 0.5      # m/s upward
DESCEND_VELOCITY = 0.3    # m/s downward
TARGET_ALTITUDE = 2.5     # meters
HOVER_TIME = 5.0          # seconds


async def run():
    drone = System()

    print("Connecting to drone...")
    await drone.connect(system_address="udp://:14540")

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

    # Set initial setpoint before starting offboard mode
    print("Setting initial setpoint...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))

    # Start offboard mode
    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
        print("Offboard mode started!")
    except OffboardError as e:
        print(f"Failed to start offboard mode: {e}")
        await drone.action.disarm()
        return

    # Climb using velocity command (negative Z = up in NED)
    print(f"Climbing to ~{TARGET_ALTITUDE}m...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0, 0, -CLIMB_VELOCITY, 0)
    )

    # Monitor altitude and stop climbing when target reached
    async for position in drone.telemetry.position():
        alt = position.relative_altitude_m
        print(f"  Altitude: {alt:.2f}m", end="\r")
        if alt >= TARGET_ALTITUDE:
            print(f"\nReached target altitude: {alt:.2f}m")
            break

    # Hover (zero velocity)
    print(f"Hovering for {HOVER_TIME} seconds...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    await asyncio.sleep(HOVER_TIME)

    # Descend using velocity command (positive Z = down in NED)
    print("Descending...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0, 0, DESCEND_VELOCITY, 0)
    )

    # Monitor altitude until near ground
    async for position in drone.telemetry.position():
        alt = position.relative_altitude_m
        print(f"  Altitude: {alt:.2f}m", end="\r")
        if alt <= 0.3:
            print(f"\nNear ground: {alt:.2f}m")
            break

    # Stop offboard and land
    print("Stopping offboard mode...")
    try:
        await drone.offboard.stop()
    except OffboardError as e:
        print(f"Failed to stop offboard: {e}")

    # Use land action for final touchdown
    print("Landing...")
    await drone.action.land()

    # Wait for landing
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
