#!/usr/bin/env python3
"""
Simple keyboard control for drone using MAVSDK offboard velocity control.

Controls (type letter and press Enter):
    i/k - Forward/Back
    j/l - Left/Right
    w/s - Up/Down
    a/d - Yaw left/right
    x   - Stop
    t   - Takeoff
    n   - Land
    q   - Quit

Usage:
    python3 keyboard_control.py
"""

import asyncio
import sys
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed


# Control settings
VELOCITY_XY = 1.0       # m/s forward/backward/left/right
VELOCITY_Z = 0.5        # m/s up/down
YAWSPEED = 30.0         # deg/s yaw rate
TAKEOFF_ALT = 2.5       # meters
MOVE_DURATION = 0.5     # seconds to move per keypress


class DroneController:
    def __init__(self):
        self.drone = System()
        self.in_offboard = False
        self.running = True

    async def connect(self):
        print("Connecting to drone...")
        await self.drone.connect(system_address="udp://:14540")

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Connected!")
                return

    async def takeoff(self):
        print("Arming...")
        await self.drone.action.arm()

        print(f"Taking off to {TAKEOFF_ALT}m...")
        await self.drone.action.set_takeoff_altitude(TAKEOFF_ALT)
        await self.drone.action.takeoff()

        await asyncio.sleep(5)
        print("Starting offboard mode...")
        await self.start_offboard()

    async def start_offboard(self):
        # Send initial setpoint
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, 0, 0)
        )

        try:
            await self.drone.offboard.start()
            self.in_offboard = True
            print("Offboard mode ACTIVE!")
        except OffboardError as e:
            print(f"Offboard failed: {e}")

    async def land(self):
        print("Landing...")
        if self.in_offboard:
            try:
                await self.drone.offboard.stop()
            except:
                pass
        await self.drone.action.land()
        self.in_offboard = False
        print("Landed.")

    async def move(self, forward=0, right=0, down=0, yaw=0):
        """Move in a direction for MOVE_DURATION seconds."""
        if not self.in_offboard:
            print("Not in offboard mode! Press 't' to takeoff first.")
            return

        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(forward, right, down, yaw)
        )
        await asyncio.sleep(MOVE_DURATION)
        # Stop
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, 0, 0)
        )

    async def hover(self):
        """Send zero velocity to maintain position."""
        if self.in_offboard:
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0, 0, 0, 0)
            )


async def input_loop(controller):
    """Handle user input."""
    loop = asyncio.get_event_loop()

    while controller.running:
        # Read input in executor to not block
        try:
            cmd = await asyncio.wait_for(
                loop.run_in_executor(None, lambda: input("> ").strip().lower()),
                timeout=0.5
            )
        except asyncio.TimeoutError:
            # Send hover command periodically
            await controller.hover()
            continue
        except EOFError:
            break

        if not cmd:
            continue

        # Process command
        if cmd == 'i':
            print("Forward...")
            await controller.move(forward=VELOCITY_XY)
        elif cmd == 'k':
            print("Backward...")
            await controller.move(forward=-VELOCITY_XY)
        elif cmd == 'j':
            print("Left...")
            await controller.move(right=-VELOCITY_XY)
        elif cmd == 'l':
            print("Right...")
            await controller.move(right=VELOCITY_XY)
        elif cmd == 'w':
            print("Up...")
            await controller.move(down=-VELOCITY_Z)
        elif cmd == 's':
            print("Down...")
            await controller.move(down=VELOCITY_Z)
        elif cmd == 'a':
            print("Yaw left...")
            await controller.move(yaw=-YAWSPEED)
        elif cmd == 'd':
            print("Yaw right...")
            await controller.move(yaw=YAWSPEED)
        elif cmd == 'x':
            print("Stop.")
            await controller.hover()
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
    print("""
Commands (press key + Enter):
  i/k  - Forward/Back
  j/l  - Left/Right
  w/s  - Up/Down
  a/d  - Yaw left/right
  x    - Stop/Hover
  t    - Takeoff
  n    - Land
  q    - Quit
  h    - Help
""")


async def main():
    print("="*40)
    print("DRONE KEYBOARD CONTROL")
    print("="*40)
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
