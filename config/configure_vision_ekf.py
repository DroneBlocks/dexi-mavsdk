#!/usr/bin/env python3
"""
Configure PX4 EKF2 to use vision odometry instead of GPS.

This script disables GPS fusion and enables AprilTag vision fusion
for position estimation. Run this once after simulation startup.

Usage:
    python3 configure_vision_ekf.py
"""

import asyncio
from mavsdk import System


async def wait_for_connection(drone, timeout_sec=5):
    """Wait for drone connection with timeout."""
    async for state in drone.core.connection_state():
        if state.is_connected:
            return True
    return False


async def run():
    drone = System()

    # Try multiple connection methods
    connection_options = [
        ("udp://:14540", "Listening on port 14540 (SITL sends to us)"),
        ("udp://172.20.0.4:14540", "Connecting to px4-sitl container"),
    ]

    connected = False
    for conn_string, description in connection_options:
        print(f"\nTrying: {description}")
        print(f"  Address: {conn_string}")

        try:
            drone = System()  # Fresh instance for each attempt
            await drone.connect(system_address=conn_string)

            # Wait up to 5 seconds for connection
            try:
                connected = await asyncio.wait_for(
                    wait_for_connection(drone),
                    timeout=5.0
                )
                if connected:
                    print("  SUCCESS - Drone connected!")
                    break
            except asyncio.TimeoutError:
                print("  TIMEOUT - No response after 5 seconds")

        except Exception as e:
            print(f"  FAILED: {e}")

    if not connected:
        print("\nCould not connect to drone!")
        print("Check that px4-sitl container is running: docker ps")
        return

    print("\nConfiguring EKF2 for vision-based navigation...\n")

    # Parameters to set for vision-based position hold
    # Two modes available: GPS_DISABLED or HYBRID
    mode = "HYBRID"  # Change to "GPS_DISABLED" to fully disable GPS

    if mode == "GPS_DISABLED":
        params = [
            # Fully disable GPS - requires vision or velocity control only
            ("EKF2_GPS_CTRL", 0, "Disable GPS fusion"),
            ("EKF2_EV_CTRL", 9, "Enable vision XY + yaw fusion"),
            ("EKF2_HGT_MODE", 2, "Use distance sensor for height"),
            ("EKF2_EV_DELAY", 50.0, "Vision processing delay"),
        ]
    else:
        # HYBRID mode: GPS as fallback, vision preferred when available
        params = [
            # Keep GPS enabled as fallback
            ("EKF2_GPS_CTRL", 7, "Enable GPS (fallback)"),

            # Enable vision fusion: XY position + yaw
            ("EKF2_EV_CTRL", 9, "Enable vision XY + yaw fusion"),

            # Height mode: 2 = distance sensor
            ("EKF2_HGT_MODE", 2, "Use distance sensor for height"),

            # Vision delay compensation (ms)
            ("EKF2_EV_DELAY", 50.0, "Vision processing delay"),

            # Lower vision noise = EKF trusts vision more than GPS
            ("EKF2_EVP_NOISE", 0.05, "Vision position noise (low = trust more)"),

            # Higher GPS noise = EKF trusts GPS less
            ("EKF2_GPS_P_NOISE", 1.0, "GPS position noise (high = trust less)"),
        ]

    for param_name, value, description in params:
        try:
            if isinstance(value, float):
                await drone.param.set_param_float(param_name, value)
            else:
                await drone.param.set_param_int(param_name, value)
            print(f"  {param_name} = {value}  ({description})")
        except Exception as e:
            print(f"  {param_name} FAILED: {e}")

    print("\nConfiguration complete!")
    print(f"\nMode: {mode}")
    print("Note: These parameters are saved to RAM only.")

    if mode == "GPS_DISABLED":
        print("\nGPS is DISABLED. The drone will:")
        print("  - Use vision odometry for XY position")
        print("  - Use distance sensor for height")
        print("  - DRIFT if no AprilTag is visible!")
        print("\nUse velocity control to navigate to tags.")
    else:
        print("\nHYBRID mode enabled. The drone will:")
        print("  - Prefer AprilTag vision when tags are visible")
        print("  - Fall back to GPS when no tags are visible")
        print("  - Use distance sensor for height")
        print("\nYou can fly normally with GPS, and vision will take")
        print("priority when AprilTags come into view.")


if __name__ == "__main__":
    asyncio.run(run())
