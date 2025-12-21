#!/usr/bin/env python3
"""Enable GPS (hybrid mode with vision)."""

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

    print("\nEnabling GPS (hybrid mode)...\n")

    params = [
        ("EKF2_GPS_CTRL", 7, "int"),      # Enable GPS
        ("EKF2_EV_CTRL", 9, "int"),        # Keep vision enabled
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

    print("\nGPS ENABLED!")
    print("Running in hybrid mode (GPS + Vision).")


if __name__ == "__main__":
    asyncio.run(run())
