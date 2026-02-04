#!/usr/bin/env python3
"""
Full Offboard Velocity Takeoff and Land - Safe for Optical Flow
TIME-BASED control (telemetry unreliable via MAVSDK)
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed


# Flight parameters
TARGET_ALT = 1.0          # meters
CLIMB_SPEED = 0.5         # m/s
DESCEND_SPEED = 0.3       # m/s
HOVER_TIME = 10.0         # seconds
SETPOINT_INTERVAL = 0.05  # 50 Hz

# Calculate climb/descend times from physics
CLIMB_TIME = TARGET_ALT / CLIMB_SPEED      # 1.0m / 0.3m/s = 3.33s
DESCEND_TIME = TARGET_ALT / DESCEND_SPEED  # Same


async def run():
    drone = System()

    print("Connecting to drone...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    print("Waiting for drone to be ready...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("Drone is ready!")
            break

    # Prime offboard
    print("Priming offboard setpoints...")
    for i in range(100):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(0.02)

    # Start offboard
    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
        print("Offboard mode active!")
    except OffboardError as e:
        print(f"Offboard failed: {e}")
        return

    # Arm
    print("Arming...")
    await drone.action.arm()
    print("Armed!")

    # Spool up
    print("Spooling up...")
    for i in range(25):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(SETPOINT_INTERVAL)

    # === TAKEOFF (time-based) ===
    print(f"Climbing at {CLIMB_SPEED} m/s for {CLIMB_TIME:.1f}s to reach {TARGET_ALT}m...")
    climb_end = asyncio.get_event_loop().time() + CLIMB_TIME

    while asyncio.get_event_loop().time() < climb_end:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, -CLIMB_SPEED, 0)
        )
        elapsed = CLIMB_TIME - (climb_end - asyncio.get_event_loop().time())
        est_alt = elapsed * CLIMB_SPEED
        print(f"  Climbing... est alt: {est_alt:.2f}m")
        await asyncio.sleep(SETPOINT_INTERVAL)

    print(f"Climb complete! Estimated altitude: {TARGET_ALT}m")

    # === HOVER ===
    print(f"Hovering for {HOVER_TIME} seconds...")
    hover_end = asyncio.get_event_loop().time() + HOVER_TIME
    while asyncio.get_event_loop().time() < hover_end:
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        remaining = hover_end - asyncio.get_event_loop().time()
        print(f"  Hovering... {remaining:.1f}s remaining")
        await asyncio.sleep(SETPOINT_INTERVAL)
    print("Hover complete!")

    # === DESCEND (time-based) ===
    print(f"Descending at {DESCEND_SPEED} m/s for {DESCEND_TIME:.1f}s...")
    descend_end = asyncio.get_event_loop().time() + DESCEND_TIME

    while asyncio.get_event_loop().time() < descend_end:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, DESCEND_SPEED, 0)
        )
        elapsed = DESCEND_TIME - (descend_end - asyncio.get_event_loop().time())
        est_alt = TARGET_ALT - (elapsed * DESCEND_SPEED)
        print(f"  Descending... est alt: {est_alt:.2f}m")
        await asyncio.sleep(SETPOINT_INTERVAL)

    print("Descent complete!")

    # === LAND ===
    print("Stopping offboard...")
    try:
        await drone.offboard.stop()
    except OffboardError as e:
        print(f"Offboard stop error: {e}")

    print("Landing...")
    await drone.action.land()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Landed!")
            break

    print("Disarming...")
    await drone.action.disarm()
    print("Done!")


if __name__ == "__main__":
    asyncio.run(run())
