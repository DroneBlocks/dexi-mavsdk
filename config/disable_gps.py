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

    # EKF2_EV_CTRL bitmask:
    #   Bit 0 (1): Horizontal position
    #   Bit 1 (2): Vertical position
    #   Bit 2 (4): 3D velocity
    #   Bit 3 (8): Yaw
    # Note: Yaw fusion disabled - AprilTag orientation is ambiguous when viewed from above
    params = [
        ("EKF2_GPS_CTRL", 0, "int"),       # Disable GPS
        ("EKF2_EV_CTRL", 1, "int"),        # Enable vision horizontal position only
        ("EKF2_EV_DELAY", 0.0, "float"),   # Vision delay (ms) - 0 for sim
        ("EKF2_EVP_GATE", 50.0, "float"),  # Large gate to allow movement while using vision
        ("EKF2_EVP_NOISE", 0.5, "float"),  # Expected position noise from vision
        ("EKF2_HGT_REF", 0, "int"),        # Use barometer for height (not GPS)
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
    print("Vision position fusion enabled (EKF2_EV_CTRL=1).")
    print("Yaw not fused - AprilTag orientation is ambiguous from above.")
    print("If you lose sight of the AprilTag, the drone WILL DRIFT!")


if __name__ == "__main__":
    asyncio.run(run())
