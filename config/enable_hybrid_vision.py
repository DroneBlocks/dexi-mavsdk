#!/usr/bin/env python3
"""
Enable hybrid vision mode - GPS + vision fusion.
Run this first to let EKF2 start fusing visual odometry.
Then run disable_gps.py to switch to vision-only.
"""

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

    print("\nEnabling hybrid GPS + vision mode...\n")

    params = [
        ("EKF2_GPS_CTRL", 7, "int"),       # Keep GPS enabled
        ("EKF2_EV_CTRL", 4, "int"),        # Enable vision horizontal position
        ("EKF2_EV_DELAY", 0.0, "float"),   # Vision delay (ms) - 0 for sim
        ("EKF2_EVP_NOISE", 0.1, "float"),  # Vision position noise (low = trust more)
        ("EKF2_EVP_GATE", 10.0, "float"),  # Innovation gate (larger = more tolerant)
        ("EKF2_HGT_REF", 0, "int"),        # Use barometer for height
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

    print("\nHybrid mode enabled!")
    print("EKF2 is now fusing GPS + AprilTag vision.")
    print("Wait 10-20 seconds, then run disable_gps.py to switch to vision-only.")


if __name__ == "__main__":
    asyncio.run(run())
