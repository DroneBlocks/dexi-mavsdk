#!/usr/bin/env python3
"""Enable GPS and reset EKF2 to default flight mode."""

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

    print("\nResetting EKF2 to GPS mode (default settings)...\n")

    params = [
        # GPS settings
        ("EKF2_GPS_CTRL", 7, "int"),       # Enable GPS (all axes)

        # Disable external vision (reset to defaults)
        ("EKF2_EV_CTRL", 0, "int"),        # Disable vision fusion
        ("EKF2_EV_DELAY", 0.0, "float"),   # Reset delay
        ("EKF2_EVP_NOISE", 0.1, "float"),  # Default position noise
        ("EKF2_EVP_GATE", 5.0, "float"),   # Default position gate
        ("EKF2_EVA_NOISE", 0.05, "float"), # Default yaw angle noise (rad)

        # Height reference
        ("EKF2_HGT_REF", 1, "int"),        # Use GPS for height reference

        # GPS noise (defaults)
        ("EKF2_GPS_P_NOISE", 0.5, "float"), # Default GPS position noise
        ("EKF2_GPS_V_NOISE", 0.3, "float"), # Default GPS velocity noise
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

    print("\nGPS MODE RESTORED!")
    print("EKF2 reset to default GPS-based navigation.")
    print("You should be able to arm and take off normally now.")


if __name__ == "__main__":
    asyncio.run(run())
