#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.callback_groups import ReentrantCallbackGroup
import json
import math
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import time

class YoloInferenceNode(Node):
    def __init__(self, frame_skip=5):
        super().__init__('yolo_inference_node')

        # Subscriber to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Publisher for predictions as JSON strings
        self.prediction_publisher = self.create_publisher(
            String,
            '/yolo/predictions',
            10
        )

        self.bridge = CvBridge()

        # Initialize YOLO model (adjust model path as needed)
        self.yolo_model = YOLO('/root/best.pt')  # Ensure the path is correct

        # Frame skipping parameters
        self.frame_skip = frame_skip  # Process every 'frame_skip' frames
        self.frame_counter = 0       # Initialize frame counter

        self.get_logger().info(f"YOLO Inference Node has been started with frame skipping: process every {self.frame_skip} frames.")

    def image_callback(self, msg: Image):
        """
        Receives the image from /camera/image_raw, runs YOLO inference on a subset of frames,
        and publishes the predictions as a JSON-formatted string.
        """
        self.frame_counter += 1

        if self.frame_counter % self.frame_skip != 0:
            # Skip processing this frame
            self.get_logger().debug(f"Frame {self.frame_counter} skipped.")
            return
        else:
            self.get_logger().debug(f"Processing frame {self.frame_counter}.")

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Get image dimensions
        height, width = cv_image.shape[:2]
        image_center_x = width / 2
        image_center_y = height / 2

        start_time = time.time()

        # Run YOLO inference
        results = self.yolo_model.predict(source=cv_image, verbose=False)

        inference_time = time.time() - start_time
        self.get_logger().info(f"YOLO inference took {inference_time:.2f} seconds.")

        # Prepare the list to hold all predictions
        predictions_list = []

        for pred in results:
            for box in pred.boxes:
                # Extract bounding box coordinates
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])

                # Get class name
                class_name = self.yolo_model.names[cls_id] if self.yolo_model.names else str(cls_id)

                # Calculate area
                area = (x2 - x1) * (y2 - y1)

                # Calculate center
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                # Calculate offset from image center
                offset_x = center_x - image_center_x
                offset_y = center_y - image_center_y

                # Define corners
                corners = [
                    {"x": float(x1), "y": float(y1), "z": 0.0},
                    {"x": float(x2), "y": float(y1), "z": 0.0},
                    {"x": float(x2), "y": float(y2), "z": 0.0},
                    {"x": float(x1), "y": float(y2), "z": 0.0}
                ]

                # Create a dictionary for the prediction
                prediction = {
                    "object_name": class_name,
                    "area": area,
                    "corners": corners,
                    "center": {"x": center_x, "y": center_y, "z": 0.0},
                    "center_offset": {"x": offset_x, "y": offset_y},
                    "confidence": conf
                }

                # Append the prediction to the list
                predictions_list.append(prediction)

        if predictions_list:
            # Serialize the predictions list to a JSON string
            combined_predictions = json.dumps(predictions_list)

            # Create and publish the String message
            prediction_msg = String()
            prediction_msg.data = combined_predictions
            self.prediction_publisher.publish(prediction_msg)

            self.get_logger().info(f'Published {len(predictions_list)} predictions as JSON.')
            # Log the predictions
            for i, pred in enumerate(predictions_list):
                self.get_logger().info(f'Prediction {i}: {pred}')
        else:
            self.get_logger().info('No predictions to publish for this frame.')


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Create callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
            
        # Subscribers
        self.create_subscription(String, '/yolo/predictions', self.box_detected_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, qos_profile=qos_profile_sensor_data)
        
        # -------------------------------------------------------------
        # PATROL POINTS
        # -------------------------------------------------------------
        # You can adjust these as needed; the robot cycles through them
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
        self.current_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.target_position = None
        self.position_tolerance = 0.3
        self.angular_tolerance = 0.1
        
        # Movement speed parameters
        self.max_linear_speed = 0.8  
        self.max_angular_speed = 1.5
        self.linear_acceleration = 0.4
        self.current_linear_speed = 0.0
        
        # EKF Variables
        self.state = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.P = np.eye(3) * 0.1  # Initial covariance
        self.Q = np.diag([0.05, 0.05, 0.02])  # Process noise covariance
        self.R = np.diag([0.2, 0.2, 0.1])  # Measurement noise covariance
        self.last_time = None
        self.last_cmd_vel = Twist()  # To store the last commanded velocities
        
        # Debug counters
        self.control_loop_count = 0
        
        # Timers
        self.create_timer(0.1, self.control_loop)         # 10Hz control loop
        self.create_timer(1.0, self.publish_markers)      # 1Hz visualization update
        self.create_timer(5.0, self.log_status)           # 5s status logging
        
        # ----------------------------------------------------------------
        # NEW STATE MACHINE FOR "person-box" BEHAVIOR
        # ----------------------------------------------------------------
        # States:
        # - "PATROLLING": normal path following
        # - "TURNING_CCW": turning 90 deg left
        # - "MOVING_FORWARD": moving 0.2 m forward
        # - "TURNING_CW": turning 90 deg right
        # - "WAITING_NO_PERSON": wait for 3 consecutive frames with no "person-box"
        # 
        self.action_state = "PATROLLING"
        
        # We will use these auxiliary variables:
        self.turn_target_angle = None
        self.move_start_position = None
        self.no_person_box_count = 0  # how many consecutive frames with NO "person-box" > 0.5 conf
        self.person_box_detected_this_frame = False
        
        self.get_logger().info('Robot Controller initialized with patrol points:')
        for i, point in enumerate(self.patrol_points):
            self.get_logger().info(f'Point {i}: ({point["x"]}, {point["y"]})')

    # ---------------
    # Logging
    # ---------------
    def log_status(self):
        """Log periodic status updates"""
        self.get_logger().info(
            f'\nStatus Update:\n'
            f'Current State: {self.action_state}\n'
            f'Position: ({self.current_position["x"]:.2f}, '
            f'{self.current_position["y"]:.2f}, '
            f'θ={math.degrees(self.current_position["theta"]):.1f}°)\n'
            f'Current Patrol Point Index: {self.current_point}/{len(self.patrol_points)-1}\n'
            f'Target: {self.target_position}\n'
        )

    # ---------------
    # Markers
    # ---------------
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
            if (self.target_position and 
                point['x'] == self.target_position['x'] and 
                point['y'] == self.target_position['y']):
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
        # Optionally add the first point again to "close" the loop visually
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

    # ---------------
    # Odometry (EKF)
    # ---------------
    def odom_callback(self, msg):
        """Handle odometry updates and perform EKF estimation"""
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return
        
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Extract control inputs (assumed to be the last commanded velocities)
        v = self.last_cmd_vel.linear.x
        omega = self.last_cmd_vel.angular.z
        
        # EKF Prediction Step
        theta = self.state[2]
        if abs(omega) > 1e-5:
            # Robot is turning
            dx = (v / omega) * (math.sin(theta + omega * dt) - math.sin(theta))
            dy = (v / omega) * (-math.cos(theta + omega * dt) + math.cos(theta))
            dtheta = omega * dt
        else:
            # Robot is moving straight
            dx = v * math.cos(theta) * dt
            dy = v * math.sin(theta) * dt
            dtheta = 0.0
        
        # State prediction
        self.state[0] += dx
        self.state[1] += dy
        self.state[2] += dtheta
        self.state[2] = self.normalize_angle(self.state[2])
        
        # Jacobian of the motion model
        if abs(omega) > 1e-5:
            J = np.array([
                [1, 0, (v / omega) * (math.cos(theta + omega * dt) - math.cos(theta))],
                [0, 1, (v / omega) * (math.sin(theta + omega * dt) - math.sin(theta))],
                [0, 0, 1]
            ])
        else:
            J = np.array([
                [1, 0, -v * math.sin(theta) * dt],
                [0, 1, v * math.cos(theta) * dt],
                [0, 0, 1]
            ])
        
        # Update covariance
        self.P = J @ self.P @ J.T + self.Q
        
        # EKF Update Step with Odometry Measurement
        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        ])
        
        # Measurement prediction
        z_pred = self.state.copy()
        
        # Innovation
        y = z - z_pred
        y[2] = self.normalize_angle(y[2])
        
        # Measurement matrix
        H = np.eye(3)
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.state += K @ y
        self.state[2] = self.normalize_angle(self.state[2])
        
        # Covariance update
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P
        
        # Update current_position with EKF estimate
        self.current_position['x'] = self.state[0]
        self.current_position['y'] = self.state[1]
        self.current_position['theta'] = self.state[2]

    def get_yaw_from_quaternion(self, orientation):
        """Extract yaw angle from quaternion"""
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
        return math.atan2(2.0 * (qw * qz + qx * qy), 
                         1.0 - 2.0 * (qy * qy + qz * qz))
    
    def normalize_angle(self, angle):
        """Normalize angle to be within [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ---------------
    # Main Control Loop
    # ---------------
    def control_loop(self):
        """Main control loop for robot movement and the special 'person-box' maneuver."""
        self.control_loop_count += 1
        
        # We'll handle a simple state machine for the "person-box" special behavior.
        cmd_vel = Twist()
        
        # ----------------------------------------------------------------
        # STATE MACHINE
        # ----------------------------------------------------------------
        if self.action_state == "PATROLLING":
            # Normal patrol. If "person-box" is detected with confidence > 0.5,
            # switch to TURNING_CCW.
            if self.person_box_detected_this_frame:
                self.get_logger().info("Detected 'person-box' > 0.5, switching to TURNING_CCW.")
                self.action_state = "TURNING_CCW"
                # We'll turn 90 degrees left from current heading
                self.turn_target_angle = self.normalize_angle(
                    self.current_position["theta"] + math.pi / 2
                )
                # Reset counters
                self.no_person_box_count = 0
                # We'll skip normal patrol movement this cycle
            else:
                # Continue normal patrol
                self.do_patrol_movement(cmd_vel)
        
        elif self.action_state == "TURNING_CCW":
            # Turn until we reach the target angle (within angular tolerance).
            angle_diff = self.turn_target_angle - self.current_position["theta"]
            angle_diff = self.normalize_angle(angle_diff)
            if abs(angle_diff) > self.angular_tolerance:
                # Continue turning in place
                # cmd_vel.angular.z = self.max_angular_speed * (angle_diff / math.pi)
                cmd_vel.angular.z = 1.5
                cmd_vel.linear.x = 0.0
            else:
                # Done turning
                self.get_logger().info("Finished 90° CCW turn, now moving forward 0.2m.")
                self.action_state = "MOVING_FORWARD"
                # Record the start position so we can measure 0.2m
                self.move_start_position = (self.current_position["x"], self.current_position["y"])
        
        elif self.action_state == "MOVING_FORWARD":
            # Move straight 0.6m from move_start_position in the current heading
            dist_moved = self.distance_2d(
                self.current_position["x"], self.current_position["y"],
                self.move_start_position[0], self.move_start_position[1]
            )
            if dist_moved < 0.8: 
                # Continue moving forward
                cmd_vel.linear.x = 0.2  # You can adjust this speed as necessary
                cmd_vel.angular.z = 0.0
            else:
                # Done moving 0.6m
                self.get_logger().info("Done moving 0.6m, now turning 90° CW.")
                self.action_state = "TURNING_CW"
                self.turn_target_angle = self.normalize_angle(
                    self.current_position["theta"] - math.pi / 2
                )

        
        elif self.action_state == "TURNING_CW":
            # Turn 90 degrees clockwise
            angle_diff = self.turn_target_angle - self.current_position["theta"]
            angle_diff = self.normalize_angle(angle_diff)
            if abs(angle_diff) > self.angular_tolerance:
                # cmd_vel.angular.z = self.max_angular_speed * (angle_diff / math.pi)
                cmd_vel.angular.z = -1.5
                cmd_vel.linear.x = 0.0
            else:
                self.get_logger().info("Finished 90° CW turn, waiting for no person-box in 3 consecutive frames.")
                self.action_state = "WAITING_NO_PERSON"
                self.no_person_box_count = 0
        
        elif self.action_state == "WAITING_NO_PERSON":
            # Robot stops and waits until no person-box is detected for 3 consecutive predictions
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            # We update no_person_box_count in box_detected_callback, so here we just check
            if self.no_person_box_count >= 3:
                self.get_logger().info("No 'person-box' > 0.5 for 3 consecutive frames. Resuming patrol.")
                self.action_state = "PATROLLING"
        
        # ----------------------------------------------------------------
        # Publish the velocity command
        # ----------------------------------------------------------------
        self.last_cmd_vel = cmd_vel
        self.vel_publisher.publish(cmd_vel)
        
        # Reset the detection flag each cycle 
        # (it will be set again by box_detected_callback if a new detection appears)
        self.person_box_detected_this_frame = False

    def distance_2d(self, x1, y1, x2, y2):
        """Helper function to compute Euclidean distance in 2D."""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    # ---------------
    # Normal Patrol Movement
    # ---------------
    def do_patrol_movement(self, cmd_vel):
        """Handles normal patrol logic to move between patrol points."""
        
        if self.target_position is None:
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
        angle_diff = self.normalize_angle(angle_diff)
        
        # Target reached check
        if distance < self.position_tolerance:
            self.target_position = None
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.current_linear_speed = 0.0  # Reset linear speed
            self.get_logger().info('Target reached!')
            return

        # Velocity control
        if abs(angle_diff) > self.angular_tolerance:
            cmd_vel.angular.z = self.max_angular_speed * (angle_diff / math.pi) * 3.0
            # Slow down but not to zero
            cmd_vel.linear.x = min(0.1, self.current_linear_speed)
        else:
            # Accelerate smoothly to max speed
            target_speed = min(self.max_linear_speed, distance) * math.cos(angle_diff)
            if target_speed > self.current_linear_speed:
                self.current_linear_speed = min(target_speed, self.current_linear_speed + self.linear_acceleration)
            else:
                self.current_linear_speed = target_speed
                
            cmd_vel.linear.x = self.current_linear_speed
            cmd_vel.angular.z = self.max_angular_speed * angle_diff * 3.0
    
    # ---------------
    # YOLO Callback
    # ---------------
    def box_detected_callback(self, msg):
        """
        Receives YOLO predictions as a JSON string.
        Checks if there's a "person-box" with confidence > 0.5.
        
        If we are in "WAITING_NO_PERSON" state, we count how many frames 
        in a row have no "person-box".
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid box detection format')
            return
        
        # Flag to see if we found a "person-box" > 0.5 in this frame
        found_person_box = False
        
        for box in data:
            if 'object_name' in box and 'confidence' in box:
                if box['object_name'] == 'person-box' and box['confidence'] > 0.5:
                    found_person_box = True
                    break
        
        if found_person_box:
            # If in PATROLLING, we handle detection in control_loop
            self.person_box_detected_this_frame = True
            
            # If we are in WAITING_NO_PERSON state, we reset the counter
            if self.action_state == "WAITING_NO_PERSON":
                self.no_person_box_count = 0
        else:
            # No person-box in this frame
            # If we are in WAITING_NO_PERSON, increase the counter
            if self.action_state == "WAITING_NO_PERSON":
                self.no_person_box_count += 1


def main(args=None):
    rclpy.init(args=args)
    
    # Create instances of both nodes
    yolo_node = YoloInferenceNode(frame_skip=5)  # Adjust frame_skip as needed
    controller_node = RobotController()
    
    # Create a MultiThreadedExecutor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(yolo_node)
    executor.add_node(controller_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        yolo_node.destroy_node()
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
