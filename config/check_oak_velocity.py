#!/usr/bin/env python3
"""
Check if OAK-D velocity is being fused by EKF2.

Monitors the estimator status flags and visual odometry topic
to verify the OAK-D velocity pipeline is working.
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
    print("Checking EKF2 parameters for OAK-D velocity...")
    print("=" * 50 + "\n")

    params_to_check = [
        ("EKF2_EV_CTRL", "External vision control (4=velocity)"),
        ("EKF2_GPS_CTRL", "GPS control (7=all enabled)"),
        ("EKF2_EV_DELAY", "Vision delay (ms)"),
        ("EKF2_EVV_NOISE", "Velocity noise (m/s)"),
        ("EKF2_EVV_GATE", "Velocity innovation gate"),
        ("EKF2_HGT_REF", "Height reference (0=baro)"),
    ]

    print("Parameter values:")
    for name, desc in params_to_check:
        try:
            # Try int first, then float
            try:
                value = await drone.param.get_param_int(name)
            except:
                value = await drone.param.get_param_float(name)
            print(f"  {name} = {value}")
            print(f"    ({desc})")
        except Exception as e:
            print(f"  {name} = ??? ({e})")

    # Check EV_CTRL specifically
    print("\n" + "-" * 50)
    try:
        ev_ctrl = await drone.param.get_param_int("EKF2_EV_CTRL")
        print("EKF2_EV_CTRL breakdown:")
        print(f"  Bit 0 (pos horiz):  {'✓' if ev_ctrl & 1 else '✗'}")
        print(f"  Bit 1 (pos vert):   {'✓' if ev_ctrl & 2 else '✗'}")
        print(f"  Bit 2 (velocity):   {'✓' if ev_ctrl & 4 else '✗'}  <- OAK-D flow")
        print(f"  Bit 3 (yaw):        {'✓' if ev_ctrl & 8 else '✗'}")

        if ev_ctrl & 4:
            print("\n✅ Velocity fusion is ENABLED in parameters")
        else:
            print("\n❌ Velocity fusion is DISABLED")
            print("   Run enable_oak_velocity.py to enable it")
    except Exception as e:
        print(f"Could not read EKF2_EV_CTRL: {e}")

    print("\n" + "=" * 50)
    print("To see if EKF2 is actually fusing velocity data,")
    print("check /fmu/out/estimator_status_flags for:")
    print("  cs_ev_vel: true")
    print("=" * 50)


if __name__ == "__main__":
    asyncio.run(run())
