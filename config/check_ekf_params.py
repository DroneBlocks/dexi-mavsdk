#!/usr/bin/env python3
"""Check current EKF2 vision-related parameters."""

import asyncio
from mavsdk import System


async def run():
    drone = System()

    print("Connecting...")
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!\n")
            break

    params_to_check = [
        "EKF2_GPS_CTRL",
        "EKF2_EV_CTRL",
        "EKF2_EV_DELAY",
        "EKF2_EVP_NOISE",
        "EKF2_EVP_GATE",
        "EKF2_HGT_REF",
        "EKF2_HGT_MODE",
    ]

    print("Current EKF2 parameters:")
    print("-" * 40)

    for param in params_to_check:
        try:
            # Try int first, then float
            try:
                val = await drone.param.get_param_int(param)
            except:
                val = await drone.param.get_param_float(param)
            print(f"  {param}: {val}")
        except Exception as e:
            print(f"  {param}: ERROR - {e}")

    print("-" * 40)
    print("\nExpected for vision-only mode:")
    print("  EKF2_GPS_CTRL: 0 (GPS disabled)")
    print("  EKF2_EV_CTRL: 4 (horizontal position)")
    print("  EKF2_HGT_REF: 0 (barometer)")


if __name__ == "__main__":
    asyncio.run(run())
