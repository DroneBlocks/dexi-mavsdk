#!/usr/bin/env python3
"""
MAVSDK example: Takeoff and scan for AprilTags, land when ID 1 is found.

Works with PX4 SITL simulation and DEXI drone.

This script:
1. Connects to the drone via MAVSDK
2. Starts a ROS2 node to subscribe to AprilTag detections
3. Arms and takes off to 2.5 meters
4. Hovers while scanning for AprilTag ID 1
5. When ID 1 is detected, initiates landing immediately
6. Lands after detection or timeout

Usage:
    python3 apriltag_land_mission.py

Requires ROS2 environment to be sourced and rosbridge running.
"""

import asyncio
import threading
from mavsdk import System

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from apriltag_msgs.msg import AprilTagDetectionArray


class AprilTagLandMonitor(Node):
    """ROS2 node that monitors AprilTag detections for landing trigger."""

    def __init__(self):
        super().__init__('apriltag_land_mission')

        # Track if we've found tag ID 1
        self.tag_one_found = False

        # Subscribe to AprilTag detections
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.detection_callback,
            10
        )

        self.get_logger().info('AprilTag land monitor initialized')
        self.get_logger().info('Waiting for AprilTag ID 1 to trigger landing...')

    def detection_callback(self, msg):
        """Process AprilTag detections."""
        if self.tag_one_found:
            return  # Already found, no need to process more

        for detection in msg.detections:
            self.get_logger().info(f'Detected AprilTag ID: {detection.id}')

            if detection.id == 1:
                self.get_logger().info('*** TAG ID 1 FOUND - INITIATING LANDING! ***')
                self.tag_one_found = True
                break


class LandMissionController:
    """Controls the drone flight and coordinates with ROS2 AprilTag monitor."""

    def __init__(self):
        self.apriltag_monitor = None
        self.ros_thread = None
        self.executor = None

    def start_ros_node(self):
        """Start the ROS2 node in a separate thread."""
        rclpy.init()
        self.apriltag_monitor = AprilTagLandMonitor()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.apriltag_monitor)

        def spin():
            while rclpy.ok():
                self.executor.spin_once(timeout_sec=0.1)

        self.ros_thread = threading.Thread(target=spin, daemon=True)
        self.ros_thread.start()
        print("ROS2 AprilTag land monitor started")

    def stop_ros_node(self):
        """Stop the ROS2 node."""
        if self.executor:
            self.executor.shutdown()
        if self.apriltag_monitor:
            self.apriltag_monitor.destroy_node()
        rclpy.shutdown()
        print("ROS2 node stopped")

    async def run_mission(self):
        """Execute the drone mission."""
        drone = System()

        print("Connecting to drone (listening on port 14540)...")
        await drone.connect(system_address="udpin://0.0.0.0:14540")

        # Wait for connection
        print("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("Drone connected!")
                break

        # Wait for drone to be ready to arm
        print("Waiting for drone to be ready to arm...")
        async for health in drone.telemetry.health():
            if health.is_armable:
                print("Drone is ready to arm!")
                break

        # Arm the drone
        print("Arming...")
        await drone.action.arm()
        print("Armed!")

        # Takeoff
        print("Taking off to 2.5m...")
        await drone.action.set_takeoff_altitude(2.5)
        await drone.action.takeoff()

        # Wait for takeoff to complete
        print("Waiting to reach altitude...")
        async for position in drone.telemetry.position():
            if position.relative_altitude_m > 2.0:
                print(f"Reached altitude: {position.relative_altitude_m:.1f}m")
                break

        # Hover and scan for AprilTags
        print("\n" + "=" * 50)
        print("SCANNING FOR APRILTAG ID 1 (LAND TRIGGER)...")
        print("Drone will hover until tag is found or timeout (60s)")
        print("=" * 50 + "\n")

        scan_timeout = 60  # seconds
        scan_start = asyncio.get_event_loop().time()
        last_status_print = 0

        while True:
            # Check if tag was found
            if self.apriltag_monitor and self.apriltag_monitor.tag_one_found:
                print("\n*** APRILTAG ID 1 DETECTED - LANDING NOW! ***\n")
                break

            # Check for timeout
            elapsed = asyncio.get_event_loop().time() - scan_start
            if elapsed >= scan_timeout:
                print(f"\nScan timeout after {scan_timeout}s - tag ID 1 not found")
                break

            # Print status every 5 seconds
            current_5s = int(elapsed) // 5
            if current_5s > last_status_print:
                last_status_print = current_5s
                remaining = scan_timeout - int(elapsed)
                print(f"Still scanning... {remaining}s remaining")

            await asyncio.sleep(0.5)

        # Land
        print("Landing...")
        await drone.action.land()

        # Wait for landing
        print("Waiting for landing...")
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                print("Landed!")
                break

        # Disarm
        print("Disarming...")
        await drone.action.disarm()
        print("Done!")


async def main():
    controller = LandMissionController()

    try:
        # Start ROS2 node for AprilTag monitoring
        controller.start_ros_node()

        # Give ROS2 a moment to initialize
        await asyncio.sleep(1)

        # Run the mission
        await controller.run_mission()

    except KeyboardInterrupt:
        print("\nMission interrupted by user")
    finally:
        controller.stop_ros_node()


if __name__ == "__main__":
    asyncio.run(main())
