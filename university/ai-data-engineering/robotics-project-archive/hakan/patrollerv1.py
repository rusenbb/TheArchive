#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.callback_groups import ReentrantCallbackGroup
import json
import math
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Create callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.box_cmd_publisher = self.create_publisher(String, 'box_command', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
            
        # Subscribers
        self.create_subscription(String, 'box_detection', self.box_detected_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Patrol points matching the warehouse layout
        self.patrol_points = [
            {'x': 9.0, 'y': 4.0},
            {'x': 5.875, 'y': 4.0},
            {'x': 2.75, 'y': 4.0},
            {'x': -0.375, 'y': 4.0},
            {'x': -3.5, 'y': 4.0},
            {'x': -3.375, 'y': 2.0},
            {'x': -3.25, 'y': 0.0},
            {'x': -3.125, 'y': -2.0},
            {'x': -3.0, 'y': -4.0},
            {'x': 0.1, 'y': -4.0},
            {'x': 3.2, 'y': -4.0},
            {'x': 6.3, 'y': -4.0},
            {'x': 9.4, 'y': -4.0},
            {'x': 9.3, 'y': -2.0},
            {'x': 9.2, 'y': 0.0},
            {'x': 9.1, 'y': 2.0}
        ]

        self.current_point = 0
        self.is_collecting_box = False
        self.warehouse_door = {'x': 0.0, 'y': -6.0}
        self.current_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.target_position = None
        self.position_tolerance = 0.3
        self.angular_tolerance = 0.1
        
        # Movement speed parameters
        self.max_linear_speed = 0.8  # Increased from default
        self.max_angular_speed = 1.5  # Increased from default
        self.linear_acceleration = 0.4  # Added for smoother acceleration
        self.current_linear_speed = 0.0
        
        # Start timers
        self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        self.create_timer(1.0, self.publish_markers)  # 1Hz visualization update
        
        # Debug counters
        self.control_loop_count = 0
        self.last_status_print = self.get_clock().now()
        
        self.get_logger().info('Robot Controller initialized with patrol points:')
        for i, point in enumerate(self.patrol_points):
            self.get_logger().info(f'Point {i}: ({point["x"]}, {point["y"]})')

    def log_status(self):
        """Log periodic status updates"""
        now = self.get_clock().now()
        if (now - self.last_status_print).nanoseconds / 1e9 >= 5.0:  # Log every 5 seconds
            self.last_status_print = now
            self.get_logger().info(
                f'\nStatus Update:\n'
                f'Position: ({self.current_position["x"]:.2f}, {self.current_position["y"]:.2f}, θ={math.degrees(self.current_position["theta"]):.1f}°)\n'
                f'Current Point: {self.current_point}/{len(self.patrol_points)-1}\n'
                f'Target: {self.target_position}\n'
                f'Collecting Box: {self.is_collecting_box}'
            )

    def publish_markers(self):
        """Publish visualization markers for debugging"""
        marker_array = MarkerArray()
        
        # Patrol points markers
        for i, point in enumerate(self.patrol_points):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "patrol_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point['x']
            marker.pose.position.y = point['y']
            marker.pose.position.z = 0.1
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            # Highlight current target
            if self.target_position and point['x'] == self.target_position['x'] and point['y'] == self.target_position['y']:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3
            
            marker_array.markers.append(marker)
        
        # Path line marker
        path_marker = Marker()
        path_marker.header.frame_id = "odom"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "patrol_path"
        path_marker.id = len(self.patrol_points)
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05  # Line width
        path_marker.color.a = 0.5
        path_marker.color.b = 1.0
        
        # Create path points including return to start
        points = []
        for point in self.patrol_points:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = 0.1
            points.append(p)
        # Add first point to close the loop
        p = Point()
        p.x = self.patrol_points[0]['x']
        p.y = self.patrol_points[0]['y']
        p.z = 0.1
        points.append(p)
        
        path_marker.points = points
        marker_array.markers.append(path_marker)
        
        # Robot position marker
        robot_marker = Marker()
        robot_marker.header.frame_id = "odom"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "robot"
        robot_marker.id = len(self.patrol_points) + 1
        robot_marker.type = Marker.ARROW
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = self.current_position['x']
        robot_marker.pose.position.y = self.current_position['y']
        robot_marker.pose.position.z = 0.1
        # Convert theta to quaternion (arrow orientation)
        robot_marker.pose.orientation.z = math.sin(self.current_position['theta'] / 2.0)
        robot_marker.pose.orientation.w = math.cos(self.current_position['theta'] / 2.0)
        robot_marker.scale.x = 0.4  # Arrow length
        robot_marker.scale.y = 0.1  # Arrow width
        robot_marker.scale.z = 0.1  # Arrow height
        robot_marker.color.a = 1.0
        robot_marker.color.r = 1.0
        
        marker_array.markers.append(robot_marker)
        
        self.marker_publisher.publish(marker_array)

    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y
        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_position['theta'] = math.atan2(2.0 * (qw * qz + qx * qy), 
                                                  1.0 - 2.0 * (qy * qy + qz * qz))

    def control_loop(self):
        """Main control loop for robot movement"""
        self.control_loop_count += 1
        self.log_status()  # Periodic status logging
        
        if self.target_position is None:
            if not self.is_collecting_box:
                self.target_position = self.patrol_points[self.current_point]
                self.get_logger().info(
                    f'New target: Point {self.current_point} at '
                    f'({self.target_position["x"]}, {self.target_position["y"]})'
                )
                self.current_point = (self.current_point + 1) % len(self.patrol_points)
            return

        # Calculate movement vectors
        dx = self.target_position['x'] - self.current_position['x']
        dy = self.target_position['y'] - self.current_position['y']
        
        distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_position['theta']
        
        # Normalize angle
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi

        # Create velocity command
        cmd_vel = Twist()
        
        # Log detailed movement info every 50 loops
        if self.control_loop_count % 50 == 0:
            self.get_logger().debug(
                f'Movement details:\n'
                f'Distance to target: {distance:.2f}m\n'
                f'Angle difference: {math.degrees(angle_diff):.1f}°'
            )
        
        # Target reached check
        if distance < self.position_tolerance:
            self.target_position = None
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.get_logger().info('Target reached!')
            self.vel_publisher.publish(cmd_vel)
            return

        # Velocity control with improved speed
        if abs(angle_diff) > self.angular_tolerance:
            # Rotate faster when angle difference is larger
            cmd_vel.angular.z = self.max_angular_speed * (angle_diff / math.pi)
            # Slow down but don't stop completely during rotation
            cmd_vel.linear.x = min(0.1, self.current_linear_speed)
            if self.control_loop_count % 30 == 0:
                self.get_logger().debug(f'Rotating, angle diff: {math.degrees(angle_diff):.1f}°')
        else:
            # Accelerate smoothly to max speed
            target_speed = min(self.max_linear_speed, distance) * math.cos(angle_diff)
            if target_speed > self.current_linear_speed:
                self.current_linear_speed = min(target_speed, self.current_linear_speed + self.linear_acceleration)
            else:
                self.current_linear_speed = target_speed
                
            cmd_vel.linear.x = self.current_linear_speed
            cmd_vel.angular.z = self.max_angular_speed * angle_diff
            
            if self.control_loop_count % 30 == 0:
                self.get_logger().debug(f'Moving, speed: {cmd_vel.linear.x:.2f} m/s')

        self.vel_publisher.publish(cmd_vel)

    def box_detected_callback(self, msg):
        """Handle detected box information"""
        if self.is_collecting_box:
            return
            
        try:
            data = json.loads(msg.data)
            if 'box_name' in data and 'position' in data:
                self.get_logger().info(f'Box detected: {data["box_name"]} at position ({data["position"]["x"]}, {data["position"]["y"]})')
                
                # Calculate distance to box
                dx = data['position']['x'] - self.current_position['x']
                dy = data['position']['y'] - self.current_position['y']
                distance_to_box = math.sqrt(dx * dx + dy * dy)
                
                # Only collect if within reasonable range (2.0 meters)
                if distance_to_box <= 2.0:
                    self.get_logger().info(f'Box {data["box_name"]} within collection range ({distance_to_box:.2f}m). Starting collection.')
                    self.is_collecting_box = True
                    self.collect_box(data['box_name'], data['position'])
                else:
                    self.get_logger().debug(f'Box {data["box_name"]} too far ({distance_to_box:.2f}m). Continuing patrol.')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid box detection format')

    def collect_box(self, box_name, position):
        """Handle box collection sequence"""
        # Store current patrol point to resume later
        saved_point = self.current_point
        
        # Calculate approach position (slightly before the box)
        dx = position['x'] - self.current_position['x']
        dy = position['y'] - self.current_position['y']
        distance = math.sqrt(dx * dx + dy * dy)
        
        if distance == 0:
            approach_position = position
        else:
            # Create approach point 0.5m before the box
            approach_position = {
                'x': position['x'] - (dx / distance) * 0.5,
                'y': position['y'] - (dy / distance) * 0.5
            }
        
        # First navigate to approach position
        self.get_logger().info(f'Moving to approach position for box: {box_name}')
        self.target_position = approach_position
        
        # Wait until we reach the approach position
        while self.target_position is not None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Now move to exact box position
        self.get_logger().info(f'Moving to collect box: {box_name}')
        self.target_position = position
        
        # Wait until we reach the box
        while self.target_position is not None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Simulate picking up box
        self.get_logger().info(f'Picking up box: {box_name}')
        self.move_box_to_door(box_name)
        
        # Navigate to door
        self.get_logger().info('Moving to door')
        self.target_position = self.warehouse_door
        
        # Wait until we reach the door
        while self.target_position is not None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Reset collection flag and resume patrol
        self.is_collecting_box = False
        self.current_point = saved_point
        self.get_logger().info(f'Box {box_name} delivered. Resuming patrol from point {saved_point}')

    def move_box_to_door(self, box_name):
        """Send command to move box to warehouse door"""
        cmd = {
            'box_name': box_name,
            'position': self.warehouse_door,
            'action': 'move_to_door'  # Added action field for clarity
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.box_cmd_publisher.publish(msg)
        self.get_logger().info(f'Sent command to move box {box_name} to door')

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
