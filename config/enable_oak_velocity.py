#!/usr/bin/env python3
"""
Enable OAK-D velocity fusion in EKF2.

This enables the EKF2 to fuse velocity data from the OAK-D optical flow
(/oak/flow/twist -> /fmu/in/vehicle_visual_odometry).

Run this while GPS is still enabled - the drone will use both GPS and
OAK-D velocity for improved position hold.

EKF2_EV_CTRL bitmask:
  Bit 0 (1): Horizontal position
  Bit 1 (2): Vertical position
  Bit 2 (4): 3D velocity        <- We use this
  Bit 3 (8): Yaw
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

    print("\n" + "=" * 50)
    print("Enabling OAK-D velocity fusion...")
    print("=" * 50 + "\n")

    # First, read current values
    print("Current values:")
    try:
        ev_ctrl = await drone.param.get_param_int("EKF2_EV_CTRL")
        gps_ctrl = await drone.param.get_param_int("EKF2_GPS_CTRL")
        print(f"  EKF2_EV_CTRL = {ev_ctrl}")
        print(f"  EKF2_GPS_CTRL = {gps_ctrl}")
    except Exception as e:
        print(f"  Could not read: {e}")

    print("\nSetting parameters...")

    params = [
        # Enable velocity fusion from external vision
        ("EKF2_EV_CTRL", 4, "int"),         # Bit 2 = velocity fusion

        # Keep GPS enabled (hybrid mode)
        ("EKF2_GPS_CTRL", 7, "int"),        # GPS still active

        # Vision timing and noise
        ("EKF2_EV_DELAY", 50.0, "float"),   # 50ms delay for network latency
        ("EKF2_EVV_NOISE", 0.5, "float"),   # Velocity noise (m/s)
        ("EKF2_EVV_GATE", 5.0, "float"),    # Velocity innovation gate

        # Height reference (barometer, not vision)
        ("EKF2_HGT_REF", 0, "int"),         # Use barometer
    ]

    for name, value, ptype in params:
        try:
            if ptype == "float":
                await drone.param.set_param_float(name, value)
            else:
                await drone.param.set_param_int(name, value)
            print(f"  ✓ {name} = {value}")
        except Exception as e:
            print(f"  ✗ {name} FAILED: {e}")

    # Verify
    print("\nVerifying EKF2_EV_CTRL...")
    try:
        result = await drone.param.get_param_int("EKF2_EV_CTRL")
        print(f"  EKF2_EV_CTRL = {result}")
        if result == 4:
            print("\n✅ OAK-D velocity fusion ENABLED!")
        elif result & 4:
            print(f"\n✅ Velocity fusion enabled (EKF2_EV_CTRL={result})")
        else:
            print(f"\n⚠️  Velocity fusion may not be enabled (EKF2_EV_CTRL={result})")
    except Exception as e:
        print(f"  Could not verify: {e}")

    print("\n" + "=" * 50)
    print("OAK-D velocity is now being fused with GPS.")
    print("The drone should have improved position hold.")
    print("=" * 50)
    print("\nTo check if velocity is being fused, look for:")
    print("  cs_ev_vel: true")
    print("in /fmu/out/estimator_status_flags")


if __name__ == "__main__":
    asyncio.run(run())
