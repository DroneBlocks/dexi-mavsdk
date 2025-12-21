#!/usr/bin/env python3
"""Disable GPS and enable vision-only mode."""

import asyncio
from mavsdk import System


async def run():
    drone = System()

    print("Connecting...")
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    print("\nDisabling GPS, enabling vision-only mode...\n")

    params = [
        ("EKF2_GPS_CTRL", 0, "int"),      # Disable GPS
        ("EKF2_EV_CTRL", 9, "int"),        # Enable vision XY + yaw
        ("EKF2_EV_DELAY", 50.0, "float"),  # Vision delay
    ]

    for name, value, ptype in params:
        try:
            if ptype == "float":
                await drone.param.set_param_float(name, value)
            else:
                await drone.param.set_param_int(name, value)
            print(f"  {name} = {value}")
        except Exception as e:
            print(f"  {name} FAILED: {e}")

    print("\nGPS DISABLED!")
    print("Vision is now the only XY position source.")
    print("If you lose sight of the AprilTag, the drone WILL DRIFT!")


if __name__ == "__main__":
    asyncio.run(run())
