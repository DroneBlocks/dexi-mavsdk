#!/usr/bin/env python3
"""
Keyboard control for drone using MAVSDK offboard position commands.

Each keypress moves the drone a fixed distance in the given direction,
relative to the drone's current heading.

Controls (type letter and press Enter):
    i/k - Forward/Back
    j/l - Left/Right
    w/s - Up/Down
    a/d - Yaw left/right
    t   - Takeoff
    n   - Land
    q   - Quit

Usage:
    python3 keyboard_control.py
"""

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# Control settings
MOVE_DISTANCE = 1.0     # meters per keypress
ALTITUDE_STEP = 0.5     # meters per keypress
YAW_STEP = 30.0         # degrees per keypress
TAKEOFF_ALT = 2.5       # meters
SETTLE_TIME = 0.5       # seconds to wait after sending position


class DroneController:
    def __init__(self):
        self.drone = System()
        self.in_offboard = False
        self.running = True
        # Current position/heading tracking
        self.north = 0.0
        self.east = 0.0
        self.down = 0.0
        self.yaw = 0.0

    async def connect(self):
        print("Connecting to drone...")
        await self.drone.connect(system_address="udpin://0.0.0.0:14540")

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Connected!")
                return

    async def update_position(self):
        """Read current position and heading from telemetry."""
        async for pos in self.drone.telemetry.position_velocity_ned():
            self.north = pos.position.north_m
            self.east = pos.position.east_m
            self.down = pos.position.down_m
            break
        async for att in self.drone.telemetry.attitude_euler():
            self.yaw = att.yaw_deg
            break

    async def takeoff(self):
        # Send initial setpoint before arming (required for SITL)
        print("Sending initial offboard setpoint...")
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0, 0, 0, 0)
        )

        print("Starting offboard signal...")
        try:
            await self.drone.offboard.start()
        except OffboardError as e:
            print(f"Offboard start failed (expected on ground): {e}")

        await asyncio.sleep(1.5)

        print("Arming...")
        await self.drone.action.arm()
        print("Armed!")

        # Stop offboard to use action.takeoff()
        try:
            await self.drone.offboard.stop()
        except OffboardError:
            pass

        print(f"Taking off to {TAKEOFF_ALT}m...")
        await self.drone.action.set_takeoff_altitude(TAKEOFF_ALT)
        await self.drone.action.takeoff()

        # Wait for altitude
        print("Waiting to reach altitude...")
        async for position in self.drone.telemetry.position():
            if position.relative_altitude_m > TAKEOFF_ALT - 0.5:
                print(f"Reached {position.relative_altitude_m:.1f}m")
                break

        # Get current state and enter offboard
        await self.update_position()
        await self.start_offboard()

    async def start_offboard(self):
        # Set current position as initial setpoint
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(self.north, self.east, self.down, self.yaw)
        )

        try:
            await self.drone.offboard.start()
            self.in_offboard = True
            print("Offboard mode ACTIVE - ready for commands!")
        except OffboardError as e:
            print(f"Offboard failed: {e}")

    async def land(self):
        print("Landing...")
        if self.in_offboard:
            try:
                await self.drone.offboard.stop()
            except OffboardError:
                pass
        await self.drone.action.land()
        self.in_offboard = False

        print("Waiting for landing...")
        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                print("Landed!")
                break

        await self.drone.action.disarm()
        print("Disarmed.")

    async def move(self, forward=0, right=0, down=0, yaw=0):
        """Move a fixed distance relative to current heading."""
        if not self.in_offboard:
            print("Not in offboard mode! Press 't' to takeoff first.")
            return

        await self.update_position()

        # Apply yaw change
        target_yaw = self.yaw + yaw

        # Convert body-frame forward/right to NED using current heading
        yaw_rad = math.radians(self.yaw)
        target_north = self.north + forward * math.cos(yaw_rad) - right * math.sin(yaw_rad)
        target_east = self.east + forward * math.sin(yaw_rad) + right * math.cos(yaw_rad)
        target_down = self.down + down

        await self.drone.offboard.set_position_ned(
            PositionNedYaw(target_north, target_east, target_down, target_yaw)
        )

        # Wait for drone to move
        await asyncio.sleep(SETTLE_TIME)


async def input_loop(controller):
    """Handle user input using a dedicated reader thread."""
    loop = asyncio.get_event_loop()
    queue = asyncio.Queue()

    def reader():
        """Blocking stdin reader running in a background thread."""
        while controller.running:
            try:
                line = input("> ").strip().lower()
                loop.call_soon_threadsafe(queue.put_nowait, line)
            except EOFError:
                loop.call_soon_threadsafe(queue.put_nowait, "q")
                break

    # Start single reader thread
    loop.run_in_executor(None, reader)

    while controller.running:
        try:
            cmd = await asyncio.wait_for(queue.get(), timeout=2.0)
        except asyncio.TimeoutError:
            continue

        if not cmd:
            continue

        if cmd == 'i':
            print(f"Forward {MOVE_DISTANCE}m...")
            await controller.move(forward=MOVE_DISTANCE)
        elif cmd == 'k':
            print(f"Backward {MOVE_DISTANCE}m...")
            await controller.move(forward=-MOVE_DISTANCE)
        elif cmd == 'j':
            print(f"Left {MOVE_DISTANCE}m...")
            await controller.move(right=-MOVE_DISTANCE)
        elif cmd == 'l':
            print(f"Right {MOVE_DISTANCE}m...")
            await controller.move(right=MOVE_DISTANCE)
        elif cmd == 'w':
            print(f"Up {ALTITUDE_STEP}m...")
            await controller.move(down=-ALTITUDE_STEP)
        elif cmd == 's':
            print(f"Down {ALTITUDE_STEP}m...")
            await controller.move(down=ALTITUDE_STEP)
        elif cmd == 'a':
            print(f"Yaw left {YAW_STEP}°...")
            await controller.move(yaw=-YAW_STEP)
        elif cmd == 'd':
            print(f"Yaw right {YAW_STEP}°...")
            await controller.move(yaw=YAW_STEP)
        elif cmd == 't':
            await controller.takeoff()
        elif cmd == 'n':
            await controller.land()
        elif cmd == 'q':
            controller.running = False
            break
        elif cmd == 'h' or cmd == '?':
            print_help()
        else:
            print(f"Unknown command: {cmd}")


def print_help():
    print(f"""
Commands (press key + Enter):
  i/k  - Forward/Back ({MOVE_DISTANCE}m)
  j/l  - Left/Right ({MOVE_DISTANCE}m)
  w/s  - Up/Down ({ALTITUDE_STEP}m)
  a/d  - Yaw left/right ({YAW_STEP}°)
  t    - Takeoff
  n    - Land
  q    - Quit
  h    - Help
""")


async def main():
    print("=" * 40)
    print("DRONE KEYBOARD CONTROL")
    print("=" * 40)
    print_help()

    controller = DroneController()
    await controller.connect()

    print("\nReady! Press 't' + Enter to takeoff.")
    print("Type command and press Enter.\n")

    try:
        await input_loop(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if controller.in_offboard:
            await controller.land()
        print("Exiting...")


if __name__ == "__main__":
    asyncio.run(main())
