#!/usr/bin/env python3
"""
Debug script to monitor visual odometry pipeline in real-time.
Run this while testing GPS-denied flight to diagnose issues.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
import time
from collections import deque


class VisionOdomDebugger(Node):
    def __init__(self):
        super().__init__('vision_odom_debugger')

        # QoS for PX4 topics
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Track rates
        self.detection_times = deque(maxlen=30)
        self.odom_times = deque(maxlen=30)
        self.last_print_time = time.time()

        # Latest data
        self.last_detection = None
        self.last_odom = None
        self.last_local_pos = None
        self.last_camera_info = None

        # Subscribers
        self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.detection_callback,
            10
        )

        self.create_subscription(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            self.odom_callback,
            px4_qos
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_callback,
            px4_qos
        )

        self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10
        )

        # Print timer
        self.create_timer(0.5, self.print_status)

        self.get_logger().info('Vision Odometry Debugger Started')
        self.get_logger().info('=' * 60)

    def detection_callback(self, msg):
        self.detection_times.append(time.time())
        self.last_detection = msg

    def odom_callback(self, msg):
        self.odom_times.append(time.time())
        self.last_odom = msg

    def local_pos_callback(self, msg):
        self.last_local_pos = msg

    def camera_info_callback(self, msg):
        self.last_camera_info = msg

    def calc_rate(self, times):
        if len(times) < 2:
            return 0.0
        duration = times[-1] - times[0]
        if duration <= 0:
            return 0.0
        return (len(times) - 1) / duration

    def print_status(self):
        # Clear screen and move cursor to top
        print('\033[2J\033[H', end='')
        print('=' * 60)
        print('       VISUAL ODOMETRY DEBUG MONITOR')
        print('=' * 60)

        # Detection status
        det_rate = self.calc_rate(self.detection_times)
        print(f'\n[AprilTag Detections] /apriltag_detections')
        if self.last_detection:
            n_tags = len(self.last_detection.detections)
            print(f'  Rate: {det_rate:.1f} Hz | Tags detected: {n_tags}')
            if n_tags > 0:
                for det in self.last_detection.detections[:3]:
                    print(f'    Tag ID: {det.id} | Decision margin: {det.decision_margin:.1f}')
        else:
            print('  ❌ NO DETECTIONS RECEIVED')

        # Visual odometry status
        odom_rate = self.calc_rate(self.odom_times)
        print(f'\n[Visual Odometry] /fmu/in/vehicle_visual_odometry')
        if self.last_odom:
            pos = self.last_odom.position
            q = self.last_odom.q
            print(f'  Rate: {odom_rate:.1f} Hz | Quality: {self.last_odom.quality}')
            print(f'  Position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]')
            print(f'  Quaternion: [{q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}]')
            pv = self.last_odom.position_variance
            print(f'  Pos variance: [{pv[0]:.2f}, {pv[1]:.2f}, {pv[2]:.2f}]')
        else:
            print('  ❌ NO ODOMETRY RECEIVED')

        # PX4 local position status
        print(f'\n[PX4 Local Position] /fmu/out/vehicle_local_position')
        if self.last_local_pos:
            lp = self.last_local_pos
            print(f'  XY valid: {lp.xy_valid} | Z valid: {lp.z_valid}')
            print(f'  Position: [{lp.x:.2f}, {lp.y:.2f}, {lp.z:.2f}]')
            print(f'  Velocity: [{lp.vx:.2f}, {lp.vy:.2f}, {lp.vz:.2f}]')

            # Check for vision position usage
            if hasattr(lp, 'xy_global') and hasattr(lp, 'z_global'):
                print(f'  Global XY: {lp.xy_global} | Global Z: {lp.z_global}')
        else:
            print('  ❌ NO LOCAL POSITION RECEIVED')

        # Camera info
        print(f'\n[Camera Info] /camera_info')
        if self.last_camera_info:
            ci = self.last_camera_info
            print(f'  Resolution: {ci.width}x{ci.height}')
            k = ci.k
            print(f'  Focal length: fx={k[0]:.1f}, fy={k[4]:.1f}')
            print(f'  Principal pt: cx={k[2]:.1f}, cy={k[5]:.1f}')
            # Calculate FOV from focal length
            import math
            hfov = 2 * math.atan(ci.width / (2 * k[0])) * 180 / math.pi
            vfov = 2 * math.atan(ci.height / (2 * k[4])) * 180 / math.pi
            print(f'  Implied FOV: H={hfov:.1f}° V={vfov:.1f}°')
        else:
            print('  ❌ NO CAMERA INFO RECEIVED')

        # Diagnostic summary
        print('\n' + '=' * 60)
        print('DIAGNOSTICS:')
        issues = []

        if det_rate < 1.0:
            issues.append('⚠️  Low detection rate - check camera view of AprilTag')

        if odom_rate < 1.0 and det_rate > 1.0:
            issues.append('⚠️  Detections OK but no odometry - check TF or apriltag_odometry node')

        if self.last_local_pos and not self.last_local_pos.xy_valid:
            issues.append('❌ PX4 XY not valid - check EKF2_EV_CTRL parameter (should be 12)')

        if self.last_camera_info and self.last_odom:
            # Check if position makes sense relative to altitude
            z_odom = abs(self.last_odom.position[2])
            z_px4 = abs(self.last_local_pos.z) if self.last_local_pos else 0
            if z_px4 > 0.5 and z_odom > 0:
                ratio = z_odom / z_px4
                if ratio < 0.7 or ratio > 1.4:
                    issues.append(f'⚠️  Height mismatch: TF={z_odom:.1f}m vs PX4={z_px4:.1f}m (ratio={ratio:.2f})')
                    issues.append('    Check Unity camera FOV matches real camera')

        if not issues:
            print('✅ All systems nominal')
        else:
            for issue in issues:
                print(issue)

        print('=' * 60)
        print('Press Ctrl+C to exit')


def main():
    rclpy.init()
    node = VisionOdomDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
