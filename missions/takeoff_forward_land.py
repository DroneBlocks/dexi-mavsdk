#!/usr/bin/env python3
"""
MAVSDK: Takeoff, fly forward 1m, and land for optical flow drones (no GPS).
Arms in POSCTL, velocity climb in offboard, holds, flies forward, holds, then lands.
"""

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, PositionNedYaw

TARGET_ALT = 1.0          # meters (range sensor)
CLIMB_SPEED = 1.0         # m/s upward
FORWARD_DISTANCE = 1.0    # meters
FORWARD_SPEED = 0.5       # m/s (body-frame forward)
HOVER_TIME = 5            # seconds
TAKEOFF_TIMEOUT = 15      # seconds
FORWARD_TIMEOUT = 10      # seconds


async def get_position(drone):
    """Capture current NED position."""
    async for pv in drone.telemetry.position_velocity_ned():
        return pv.position


async def hold_position(drone, seconds, label="Position hold"):
    """Hold current position for a given duration, printing altitude."""
    pos = await get_position(drone)
    print(f"{label}: N={pos.north_m:.2f} E={pos.east_m:.2f} D={pos.down_m:.2f}")

    await drone.offboard.set_position_ned(
        PositionNedYaw(pos.north_m, pos.east_m, pos.down_m, float('nan'))
    )

    start = asyncio.get_event_loop().time()
    async for dist in drone.telemetry.distance_sensor():
        alt = dist.current_distance_m
        elapsed = asyncio.get_event_loop().time() - start

        if int(elapsed * 2) % 2 == 0:
            print(f"  {alt:.2f}m ({elapsed:.1f}s)")

        if elapsed >= seconds:
            break


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

    # Hold position for 5 seconds
    print(f"\nHolding for {HOVER_TIME}s...")
    await hold_position(drone, HOVER_TIME, "Hold after takeoff")

    # Fly forward 1m using body-frame velocity, tracking NED displacement
    start_pos = await get_position(drone)
    print(f"\nFlying forward {FORWARD_DISTANCE}m at {FORWARD_SPEED} m/s...")

    forward_start = asyncio.get_event_loop().time()
    async for pv in drone.telemetry.position_velocity_ned():
        pos = pv.position
        elapsed = asyncio.get_event_loop().time() - forward_start

        dn = pos.north_m - start_pos.north_m
        de = pos.east_m - start_pos.east_m
        traveled = math.sqrt(dn**2 + de**2)

        if int(elapsed * 5) % 5 == 0:
            print(f"  traveled {traveled:.2f}m ({elapsed:.1f}s)")

        if traveled >= FORWARD_DISTANCE:
            print(f"Reached {traveled:.2f}m forward")
            break
        if elapsed > FORWARD_TIMEOUT:
            print(f"Timeout at {traveled:.2f}m forward")
            break

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(FORWARD_SPEED, 0, 0, 0)
        )

    # Hold position for 5 seconds
    print(f"\nHolding for {HOVER_TIME}s...")
    await hold_position(drone, HOVER_TIME, "Hold after forward")

    # Land
    print("\nLanding...")
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
