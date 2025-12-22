#!/usr/bin/env python3
"""
Move the drone down.

WARNING: Drone must be in the air (post-takeoff) before running this script!
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

VELOCITY = 0.5      # m/s
DURATION = 1.0      # seconds


async def run():
    drone = System()

    print("Connecting...")
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    print("Setting initial setpoint...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))

    print("Starting offboard mode...")
    await drone.offboard.start()

    print(f"Moving DOWN at {VELOCITY} m/s for {DURATION}s...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0, 0, VELOCITY, 0)  # positive Z = down in NED
    )
    await asyncio.sleep(DURATION)

    print("Stopping...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    await asyncio.sleep(0.5)

    print("Stopping offboard mode...")
    await drone.offboard.stop()
    print("Done!")


if __name__ == "__main__":
    asyncio.run(run())
