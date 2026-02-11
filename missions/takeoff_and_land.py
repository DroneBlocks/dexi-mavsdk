#!/usr/bin/env python3
"""
MAVSDK: Takeoff and land for optical flow drones (no GPS).
Arms in POSCTL, velocity climb in offboard, position hold at target, then lands.
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, PositionNedYaw

TARGET_ALT = 1.0          # meters (range sensor)
CLIMB_SPEED = 1.0         # m/s upward
HOVER_TIME = 10           # seconds
TAKEOFF_TIMEOUT = 15      # seconds


async def run():
    drone = System()

    print("Connecting to drone (listening on port 14540)...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Arm in POSCTL (RC mode switch must be in POSCTL)
    print("Arming in POSCTL...")
    try:
        await drone.action.arm()
    except Exception as e:
        print(f"Arm failed: {e}")
        return
    print("Armed!")

    # Start offboard with velocity climb
    print("Switching to offboard...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard start failed: {e}")
        return
    print("Offboard active!")

    # Climb at 1.0 m/s
    print(f"Taking off to {TARGET_ALT}m...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0, 0, -CLIMB_SPEED, 0)
    )

    takeoff_start = asyncio.get_event_loop().time()
    async for dist in drone.telemetry.distance_sensor():
        alt = dist.current_distance_m
        elapsed = asyncio.get_event_loop().time() - takeoff_start

        if int(elapsed * 5) % 5 == 0:
            print(f"  {alt:.2f}m ({elapsed:.1f}s)")

        if alt >= TARGET_ALT:
            print(f"Reached {alt:.2f}m")
            break
        if elapsed > TAKEOFF_TIMEOUT:
            print(f"Timeout at {alt:.2f}m")
            break

    # Capture current NED position for hold
    pos_ned = None
    async for pv in drone.telemetry.position_velocity_ned():
        pos_ned = pv.position
        break

    print(f"Holding position: N={pos_ned.north_m:.2f} E={pos_ned.east_m:.2f} D={pos_ned.down_m:.2f}")

    # Switch to position hold (NaN yaw = don't control yaw, avoids fighting EKF heading drift)
    await drone.offboard.set_position_ned(
        PositionNedYaw(pos_ned.north_m, pos_ned.east_m, pos_ned.down_m, float('nan'))
    )

    print(f"Position hold for {HOVER_TIME}s...")
    hover_start = asyncio.get_event_loop().time()
    async for dist in drone.telemetry.distance_sensor():
        alt = dist.current_distance_m
        elapsed = asyncio.get_event_loop().time() - hover_start

        if int(elapsed * 2) % 2 == 0:
            print(f"  {alt:.2f}m ({elapsed:.1f}s)")

        if elapsed >= HOVER_TIME:
            break

    # Land
    print("Landing...")
    try:
        await drone.offboard.stop()
    except OffboardError:
        pass
    await drone.action.land()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Landed!")
            break

    await asyncio.sleep(2)
    try:
        await drone.action.disarm()
    except Exception:
        pass
    print("Done!")


if __name__ == "__main__":
    asyncio.run(run())
