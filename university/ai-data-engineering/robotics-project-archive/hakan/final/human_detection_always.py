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
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose


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

        self.get_logger().info(
            f"YOLO Inference Node has been started with frame skipping: "
            f"process every {self.frame_skip} frames."
        )

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
                class_name = (self.yolo_model.names[cls_id]
                              if self.yolo_model.names else str(cls_id))

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
        self.angular_tolerance = 0.3
        
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
        # NEW STATE MACHINE & DETECTION LOGIC
        # ----------------------------------------------------------------
        # States (non-human-avoidance):
        # - "PATROLLING": normal path following
        # - "TOWARDS_BLUE_BOX": moving towards detected blue box
        # - "CARRY_BLUE_BOX": carrying blue box to door
        #
        # Human avoidance states:
        # - "STOPPING_FOR_CROSSWALK"
        # - "TURNING_CCW"
        # - "MOVING_FORWARD" (0.8m)
        # - "TURNING_CW"
        # - "WAITING_NO_PERSON"
        #
        self.action_state = "PATROLLING"
        
        # Dictionary for box detection
        self.blue_box_confidence_threshold = 0.4
        self.consecutive_box_detections = 0
        self.last_detected_box_position = None
        
        # Known box positions
        self.box_positions = {
            'workcell_bin': {'x': 0.09, 'y': 0.66, 'radius': 2.0},
            'workcell_bin_clone': {'x': 0.188, 'y': -6.4, 'radius': 2.0},
            'workcell_bin_clone_0': {'x': 6.65, 'y': -1.10, 'radius': 2.0},
            'workcell_bin_clone_1': {'x': 6.0, 'y': 6.0, 'radius': 2.0},
            'workcell_bin_clone_2': {'x': 9.64, 'y': -5.57, 'radius': 2.0},
            'workcell_bin_clone_3': {'x': -4.74, 'y': 4.48, 'radius': 2.0}
        }
        
        # Blue box handling parameters
        self.blue_box_target = None
        self.door_position = {'x': -8.0, 'y': -1.3}  # Door location
        self.carrying_box = False
        self.box_pickup_distance = 0.8  # Distance for "picking up" the box
        self.target_box_name = None
        self.box_names = [
            'workcell_bin',
            'workcell_bin_clone',
            'workcell_bin_clone_0',
            'workcell_bin_clone_1',
            'workcell_bin_clone_2',
            'workcell_bin_clone_3'
        ]
        
        # Scale factor for converting pixel offsets to approximate world meters
        self.pixel_to_meter_scale = 150.0
        
        # Service client for moving boxes in Gazebo
        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')
        
        # Variables for avoidance
        self.turn_target_angle = None
        self.move_start_position = None
        self.consecutive_no_person_frames = 0
        self.person_box_detected_this_frame = False
        self.last_in_range_area = 0.0
        self.consecutive_area_increases = 0
        self.crosswalk_stop_start_time = None
        self.crosswalk_time_limit = 15.0
        self.largest_person_area = 0.0
        
        # We store the previous state before entering avoidance
        self.previous_state = "PATROLLING"
        
        # Timer and box name for delayed hiding
        self.hide_box_timer = None
        self.hide_box_name = None
        
        # Define a specific waypoint for box delivery
        self.box_delivery_waypoint = {'x': -3.25, 'y': 0.0}
        self.reached_delivery_patrol_point = False
        self.carrying_box_waypoint_reached = False
        
        # Find index of that waypoint in the patrol list
        self.box_delivery_point_index = None
        for i, p in enumerate(self.patrol_points):
            if (abs(p['x'] - self.box_delivery_waypoint['x']) < 1e-3 and
                abs(p['y'] - self.box_delivery_waypoint['y']) < 1e-3):
                self.box_delivery_point_index = i
                break
        
        if self.box_delivery_point_index is None:
            self.get_logger().warn(
                "(-3.25, 0.0) is not present in patrol_points! "
                "Please add it to the patrol_points list."
            )
            self.box_delivery_point_index = 0  # fallback
        
        self.get_logger().info('Robot Controller initialized with patrol points:')
        for i, point in enumerate(self.patrol_points):
            self.get_logger().info(f'Point {i}: ({point["x"]}, {point["y"]})')

    # ----------------------------------------------------------------
    # LOGGING
    # ----------------------------------------------------------------
    def log_status(self):
        """Log periodic status updates."""
        self.get_logger().info(
            f'\nStatus Update:\n'
            f'Current State: {self.action_state}\n'
            f'Position: ({self.current_position["x"]:.2f}, '
            f'{self.current_position["y"]:.2f}, '
            f'θ={math.degrees(self.current_position["theta"]):.1f}°)\n'
            f'Current Patrol Point Index: {self.current_point}/{len(self.patrol_points)-1}\n'
            f'Target: {self.target_position}\n'
        )

    # ----------------------------------------------------------------
    # MARKERS
    # ----------------------------------------------------------------
    def publish_markers(self):
        """Publish visualization markers for debugging."""
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
                abs(point['x'] - self.target_position['x']) < 1e-3 and
                abs(point['y'] - self.target_position['y']) < 1e-3):
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

    # ----------------------------------------------------------------
    # ODOMETRY (EKF)
    # ----------------------------------------------------------------
    def odom_callback(self, msg):
        """Handle odometry updates and perform EKF estimation."""
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
        """Extract yaw angle from quaternion."""
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )
    
    def normalize_angle(self, angle):
        """Normalize angle to be within [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ----------------------------------------------------------------
    # MAIN CONTROL LOOP
    # ----------------------------------------------------------------
    def control_loop(self):
        """Control loop that handles both normal states and box carrying, 
           but also defers to human avoidance if needed."""
        self.control_loop_count += 1
        cmd_vel = Twist()

        # 1) Check / handle human avoidance states first.
        #    If we are in a human-avoidance state (or need to switch to one),
        #    human_avoidance() returns True to indicate it handled the cmd_vel
        #    and we should skip the rest of the logic this cycle.
        if self.human_avoidance(cmd_vel):
            self.last_cmd_vel = cmd_vel
            self.vel_publisher.publish(cmd_vel)
            return
        
        # ----------------------------------------------------------------
        # If not in or done with human_avoidance, proceed with normal states
        # ----------------------------------------------------------------
        if self.action_state == "PATROLLING":
            self.do_patrol_movement(cmd_vel)

        elif self.action_state == "TOWARDS_BLUE_BOX":
            if self.blue_box_target is None:
                self.action_state = "PATROLLING"
            else:
                dx = self.blue_box_target['x'] - self.current_position['x']
                dy = self.blue_box_target['y'] - self.current_position['y']
                distance = math.sqrt(dx * dx + dy * dy)
                
                if distance < self.box_pickup_distance:
                    # "Pick up" the box
                    self.carrying_box = True
                    self.action_state = "CARRY_BLUE_BOX"
                    if self.target_box_name:
                        self.get_logger().info(
                            f'Picking up box {self.target_box_name} at '
                            f'({self.current_position["x"]:.2f}, '
                            f'{self.current_position["y"]:.2f})'
                        )
                        self.move_box(self.target_box_name, 0, 0, visible=False)
                    self.get_logger().info('Picked up blue box, carrying to door.')
                else:
                    # Move towards the box
                    self.move_towards_target(cmd_vel, self.blue_box_target)

        elif self.action_state == "CARRY_BLUE_BOX":
            # We want to deliver the box to the door.
            # But first, we navigate the normal patrol route until we reach
            # the special waypoint, then head to the door.
            if not self.reached_delivery_patrol_point:
                # Travel along patrol until we reach the special waypoint index
                self.do_patrol_movement(cmd_vel)
                
                if self.current_point >= self.box_delivery_point_index:
                    # Check distance to that waypoint
                    dx = (self.box_delivery_waypoint['x'] - self.current_position['x'])
                    dy = (self.box_delivery_waypoint['y'] - self.current_position['y'])
                    dist = math.sqrt(dx*dx + dy*dy)
                    if dist < self.position_tolerance:
                        self.get_logger().info(
                            f"Reached box-delivery waypoint at {self.box_delivery_waypoint}, "
                            "now heading to the door."
                        )
                        self.reached_delivery_patrol_point = True
                        self.target_position = None  # Clear normal patrol target
            else:
                # Once we've reached that waypoint, head straight to the door
                dx = self.door_position['x'] - self.current_position['x']
                dy = self.door_position['y'] - self.current_position['y']
                distance = math.sqrt(dx * dx + dy * dy)
                if distance < 1.0:
                    # Drop the box
                    self.carrying_box = False
                    self.blue_box_target = None
                    if self.target_box_name:
                        # Place the box near the door
                        self.move_box(
                            self.target_box_name,
                            self.door_position['x'],
                            self.door_position['y'] + 0.3,
                            visible=True
                        )
                        self.get_logger().info(
                            f"Delivered box {self.target_box_name} at door "
                            f"({self.door_position['x']:.2f}, {self.door_position['y']:.2f})"
                        )
                        self.hide_box_name = self.target_box_name
                        self.hide_box_timer = self.create_timer(5.0, self.hide_box_callback)
                        self.target_box_name = None
                    
                    # Resume normal patrol from that waypoint
                    self.action_state = "PATROLLING"
                    self.current_point = self.box_delivery_point_index
                    self.reached_delivery_patrol_point = False
                    self.get_logger().info(
                        f"Box delivered. Resuming patrol at index {self.box_delivery_point_index}."
                    )
                else:
                    # Move towards the door
                    self.move_towards_target(cmd_vel, self.door_position)

        # Finally, publish the command velocity
        self.last_cmd_vel = cmd_vel
        self.vel_publisher.publish(cmd_vel)

        # Reset detection flags each cycle
        self.person_box_detected_this_frame = False

    # ----------------------------------------------------------------
    # HUMAN AVOIDANCE LOGIC (extracted)
    # ----------------------------------------------------------------
    def human_avoidance(self, cmd_vel):
        """
        Handles all human-avoidance states and transitions:
            - STOPPING_FOR_CROSSWALK
            - TURNING_CCW
            - MOVING_FORWARD (0.8m)
            - TURNING_CW
            - WAITING_NO_PERSON

        Also checks if we should transition INTO one of these states
        (e.g., crosswalk or a person that triggers the avoidance).

        Returns:
          True if we are currently handling an avoidance state or 
          just entered one and have set cmd_vel accordingly (i.e., skip 
          normal logic this cycle).
          False if no avoidance state is active and we should continue with
          normal logic.
        """
        # If we're already in one of these states, handle it directly:
        if self.action_state in [
            "STOPPING_FOR_CROSSWALK", "TURNING_CCW", 
            "MOVING_FORWARD", "TURNING_CW", "WAITING_NO_PERSON"
        ]:
            return self.handle_avoidance_states(cmd_vel)
        
        # Otherwise, check if we need to *enter* an avoidance state,
        # no matter if we are "PATROLLING", "TOWARDS_BLUE_BOX", or "CARRY_BLUE_BOX".
        
        # 1) CROSSWALK STOP scenario
        if (self.crosswalk_stop_start_time is not None
                and self.action_state not in [
                    "STOPPING_FOR_CROSSWALK", "TURNING_CCW", "MOVING_FORWARD",
                    "TURNING_CW", "WAITING_NO_PERSON"
                ]):
            # We store the previous state to return to once done
            self.previous_state = self.action_state
            self.action_state = "STOPPING_FOR_CROSSWALK"
            # Then handle it
            return self.handle_avoidance_states(cmd_vel)

        # 2) If a person in-range triggered the 90° sequence
        if self.person_box_detected_this_frame and self.action_state not in [
            "STOPPING_FOR_CROSSWALK", "TURNING_CCW", "MOVING_FORWARD",
            "TURNING_CW", "WAITING_NO_PERSON"
        ]:
            self.previous_state = self.action_state
            self.action_state = "TURNING_CCW"
            self.turn_target_angle = self.normalize_angle(
                self.current_position["theta"] + math.pi / 2
            )
            return self.handle_avoidance_states(cmd_vel)

        # If we get here, no avoidance is triggered
        return False

    def handle_avoidance_states(self, cmd_vel):
        """
        Actually handle the active human-avoidance state and set cmd_vel accordingly.
        Returns True to indicate we remain in an avoidance state and skip normal logic.
        Once we finish the sequence, we revert to self.previous_state (if relevant)
        and return False.
        """
        if self.action_state == "STOPPING_FOR_CROSSWALK":
            # Robot stops
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

            # Condition 1: 2 consecutive frames no person => done
            if self.consecutive_no_person_frames >= 0:
                self.get_logger().info("No person-box for 2 consecutive frames. Resuming previous action.")
                self.action_state = self.previous_state
                self.crosswalk_stop_start_time = None
                return False

            # Condition 2: Time limit
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if (now_sec - self.crosswalk_stop_start_time) > self.crosswalk_time_limit:
                self.get_logger().info("Crosswalk wait exceeded 15s. Resuming previous action.")
                self.action_state = self.previous_state
                self.crosswalk_stop_start_time = None
                return False

            return True  # Remain in STOPPING_FOR_CROSSWALK

        elif self.action_state == "TURNING_CCW":
            # Turn 90° left
            angle_diff = self.turn_target_angle - self.current_position["theta"]
            angle_diff = self.normalize_angle(angle_diff)
            if abs(angle_diff) > self.angular_tolerance:
                cmd_vel.angular.z = 1.5
                cmd_vel.linear.x = 0.0
                return True
            else:
                # Done turning
                self.get_logger().info("Finished 90° CCW turn, now moving forward 0.8m.")
                self.action_state = "MOVING_FORWARD"
                self.move_start_position = (self.current_position["x"], self.current_position["y"])
                return True

        elif self.action_state == "MOVING_FORWARD":
            # Move forward 0.8 m
            dist_moved = self.distance_2d(
                self.current_position["x"], self.current_position["y"],
                self.move_start_position[0], self.move_start_position[1]
            )
            if dist_moved < 0.8:
                cmd_vel.linear.x = 0.3
                cmd_vel.angular.z = 0.0
                return True
            else:
                # Done moving 0.8m
                self.action_state = "TURNING_CW"
                self.turn_target_angle = self.normalize_angle(
                    self.current_position["theta"] - math.pi / 2
                )
                return True

        elif self.action_state == "TURNING_CW":
            # Turn 90° right
            angle_diff = self.turn_target_angle - self.current_position["theta"]
            angle_diff = self.normalize_angle(angle_diff)
            if abs(angle_diff) > self.angular_tolerance:
                cmd_vel.angular.z = -1.5
                cmd_vel.linear.x = 0.0
                return True
            else:
                self.get_logger().info("Finished 90° CW turn, waiting for no person-box for at least 1 frame.")
                self.action_state = "WAITING_NO_PERSON"
                self.consecutive_no_person_frames = 0
                return True

        elif self.action_state == "WAITING_NO_PERSON":
            # Stop in place
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            # As soon as we get 0 or 1 frames with no person, we revert. 
            # The original code had 2 frames as well, but let's keep the logic 
            # consistent with the snippet above or a single threshold:
            # Here, we check if consecutive_no_person_frames >= 0 (which is always true).
            # That wouldn't make sense. Usually you'd want 2 frames as a threshold.
            # Let's use 1 or 2 frames? We interpret the original comment "wait for 2 consecutive frames with no 'person-box' > 0.5"
            # We'll do >= 2 to be consistent with the approach used in STOPPING_FOR_CROSSWALK:
            if self.consecutive_no_person_frames >= 2:
                self.action_state = self.previous_state
                self.get_logger().info("No person-box for 2 consecutive frames. Resuming previous action.")
                return False
            return True

        return False

    # ----------------------------------------------------------------
    # HELPER METHODS
    # ----------------------------------------------------------------
    def distance_2d(self, x1, y1, x2, y2):
        """Compute Euclidean distance in 2D."""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

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

        dx = self.target_position['x'] - self.current_position['x']
        dy = self.target_position['y'] - self.current_position['y']
        
        distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - self.current_position['theta'])
        
        # Check if we reached the current patrol point
        if distance < self.position_tolerance:
            self.target_position = None
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.current_linear_speed = 0.0
            self.get_logger().info('Target reached!')
            return

        # Rotate in place if needed
        if abs(angle_diff) > self.angular_tolerance:
            cmd_vel.angular.z = self.max_angular_speed * (angle_diff / math.pi) * 3.0
            cmd_vel.linear.x = min(0.1, self.current_linear_speed)
        else:
            # Move straight
            target_speed = min(self.max_linear_speed, distance) * math.cos(angle_diff)
            if target_speed > self.current_linear_speed:
                self.current_linear_speed = min(
                    target_speed,
                    self.current_linear_speed + self.linear_acceleration
                )
            else:
                self.current_linear_speed = target_speed
                
            cmd_vel.linear.x = self.current_linear_speed
            cmd_vel.angular.z = self.max_angular_speed * angle_diff * 1.0

    def move_towards_target(self, cmd_vel, target):
        """Move robot towards a specified (x,y) target."""
        dx = target['x'] - self.current_position['x']
        dy = target['y'] - self.current_position['y']
        
        distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - self.current_position['theta'])
        
        if abs(angle_diff) > self.angular_tolerance:
            # Rotate in place
            cmd_vel.angular.z = self.max_angular_speed * (angle_diff / math.pi) * 3.0
            cmd_vel.linear.x = min(0.1, self.current_linear_speed)
        else:
            # Move forward
            target_speed = min(self.max_linear_speed, distance)
            if target_speed > self.current_linear_speed:
                self.current_linear_speed = min(
                    target_speed, 
                    self.current_linear_speed + self.linear_acceleration
                )
            else:
                self.current_linear_speed = target_speed
                
            cmd_vel.linear.x = self.current_linear_speed
            cmd_vel.angular.z = self.max_angular_speed * angle_diff * 1.5

    # ----------------------------------------------------------------
    # DETECTIONS
    # ----------------------------------------------------------------
    def box_detected_callback(self, msg):
        """Handle YOLO detections."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid box detection format')
            return
        
        # First, handle person detection
        self.process_person_detection(data)
        
        # If we're already chasing a blue box or carrying it, skip detection for new boxes
        if self.action_state in ["TOWARDS_BLUE_BOX", "CARRY_BLUE_BOX"]:
            return

        # Otherwise, see if we detect a new blue box
        if self.action_state == "PATROLLING":
            closest_box, distance = self.find_closest_box(data)
            if closest_box:
                # If we have a previous detection, check if consistent
                if self.last_detected_box_position:
                    matched_box = self.match_detected_box_to_known_position(
                        self.last_detected_box_position
                    )
                    if matched_box:
                        self.consecutive_box_detections += 1
                        self.get_logger().info(
                            f'Consecutive detections: {self.consecutive_box_detections}'
                        )
                        
                        if self.consecutive_box_detections >= 1:
                            self.blue_box_target = {
                                'x': self.box_positions[matched_box]['x'],
                                'y': self.box_positions[matched_box]['y']
                            }
                            self.target_box_name = matched_box
                            self.action_state = "TOWARDS_BLUE_BOX"
                            self.get_logger().info(
                                f'Confirmed box {matched_box} after '
                                f'{self.consecutive_box_detections} consecutive detections '
                                f'at ({self.blue_box_target["x"]:.2f}, {self.blue_box_target["y"]:.2f})'
                            )
                    else:
                        # No position match
                        self.consecutive_box_detections = 0
                        self.get_logger().info('Reset consecutive detections - no position match')
            else:
                # No box found
                self.consecutive_box_detections = 0
                self.last_detected_box_position = None
                self.get_logger().debug('Reset consecutive detections - no box found')

    def process_person_detection(self, data):
        """Separate logic for detecting 'person-box' and updating relevant flags."""
        found_person_box_this_frame = False
        largest_in_range_area = 0.0
        crosswalk_trigger = False
        
        # Reset the largest_person_area for this new frame
        self.largest_person_area = 0.0
        
        for box in data:
            if 'object_name' in box and 'confidence' in box:
                if box['object_name'] == 'person-box' and box['confidence'] > 0.5:
                    found_person_box_this_frame = True
                    area = box['area']
                    offset_x = box['center_offset']['x']
                    
                    # Update global largest_person_area if needed
                    if area > self.largest_person_area:
                        self.largest_person_area = area
                    
                    # Condition for in-range
                    if abs(offset_x) <= 300 and area > 100000:
                        if area > largest_in_range_area:
                            largest_in_range_area = area
                    
                    # Condition for crosswalk
                    if abs(offset_x) > 300 and area > 100000:
                        crosswalk_trigger = True
        
        # Update consecutive_no_person_frames
        if found_person_box_this_frame:
            self.consecutive_no_person_frames = 0
        else:
            self.consecutive_no_person_frames += 1
        
        # Crosswalk scenario
        if crosswalk_trigger and self.crosswalk_stop_start_time is None:
            # Mark the time
            self.crosswalk_stop_start_time = self.get_clock().now().nanoseconds / 1e9

        # Check in-range scenario for 90° turn
        if largest_in_range_area > 0:
            if largest_in_range_area > self.last_in_range_area:
                self.consecutive_area_increases += 1
            else:
                self.consecutive_area_increases = 0
            
            self.last_in_range_area = largest_in_range_area
            if self.consecutive_area_increases >= 2:
                self.person_box_detected_this_frame = True
                self.consecutive_area_increases = 0
        else:
            self.consecutive_area_increases = 0
            self.last_in_range_area = 0.0

    def find_closest_box(self, boxes_data):
        """Find the closest visible blue box from YOLO detections."""
        closest_dist = float('inf')
        closest_box = None
        
        for box in boxes_data:
            if (box['object_name'] == 'blue-box' and 
                box['confidence'] > self.blue_box_confidence_threshold):
                
                self.get_logger().info(f'Found blue box with confidence: {box["confidence"]:.2f}')
                
                # Calculate approximate world position of the detected box
                detected_x = (self.current_position['x'] + 
                              box['center_offset']['x'] / self.pixel_to_meter_scale)
                detected_y = (self.current_position['y'] + 
                              box['center_offset']['y'] / self.pixel_to_meter_scale)
                
                # Calculate distance in pixel-space or approximate
                dx = box['center_offset']['x']
                dy = box['center_offset']['y']
                dist = math.sqrt(dx * dx + dy * dy)
                
                if dist < closest_dist:
                    closest_dist = dist
                    closest_box = box

                self.last_detected_box_position = {'x': detected_x, 'y': detected_y}
        
        if closest_box is None:
            self.consecutive_box_detections = 0
            self.last_detected_box_position = None
        
        return closest_box, closest_dist

    def match_detected_box_to_known_position(self, detected_position):
        """Match a detected box position to the closest known box position."""
        if detected_position is None:
            return None
            
        closest_box_name = None
        min_distance = float('inf')
        
        for name, pos in self.box_positions.items():
            dist = math.sqrt(
                (detected_position['x'] - pos['x'])**2 + 
                (detected_position['y'] - pos['y'])**2
            )
            if dist < pos.get('radius', 2.0) and dist < min_distance:
                min_distance = dist
                closest_box_name = name
        
        return closest_box_name

    # ----------------------------------------------------------------
    # GAZEBO BOX MANIPULATION
    # ----------------------------------------------------------------
    def move_box(self, box_name, x, y, visible=True):
        """Move a box to specified position or hide it."""
        entity_state = EntityState()
        entity_state.name = box_name
        entity_state.pose = Pose()
        
        if visible:
            self.get_logger().info(f'Making box {box_name} visible at ({x:.2f}, {y:.2f})')
            entity_state.pose.position.x = x
            entity_state.pose.position.y = y
            entity_state.pose.position.z = 0.0
        else:
            self.get_logger().info(f'Hiding box {box_name} (moving to far position)')
            # Update the dictionary so we don't find it again
            self.box_positions[box_name]['x'] = 99.0
            self.box_positions[box_name]['y'] = 99.0
            entity_state.pose.position.x = 99.0
            entity_state.pose.position.y = 99.0
            entity_state.pose.position.z = 0.0
            
        entity_state.pose.orientation.w = 1.0
        entity_state.reference_frame = 'world'

        request = SetEntityState.Request()
        request.state = entity_state

        future = self.set_entity_state_client.call_async(request)
        future.add_done_callback(self.move_box_callback)

    def move_box_callback(self, future):
        """Handle response from move_box service call."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Box move operation successful')
            else:
                self.get_logger().error('Failed to move box')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def hide_box_callback(self):
        """Hides the previously delivered box after a small delay."""
        if self.hide_box_name:
            self.move_box(self.hide_box_name, 99.0, 99.0, visible=False)
            self.get_logger().info(f"Box {self.hide_box_name} is now hidden from the map.")
            self.hide_box_name = None
        if self.hide_box_timer:
            self.hide_box_timer.cancel()
            self.hide_box_timer = None


def set_entities_positions(node, positions_dict):
    """
    Given a node and a dictionary of the form:
        {
          'entity_name': {'x': float, 'y': float},
          ...
        }
    Call the /gazebo/set_entity_state service to move each entity.
    """
    client = node.create_client(SetEntityState, '/gazebo/set_entity_state')
    
    # Wait up to 5 seconds for the service to be available
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service /gazebo/set_entity_state not available.')
        return
    
    for name, coords in positions_dict.items():
        request = SetEntityState.Request()
        entity_state = EntityState()
        entity_state.name = name
        entity_state.pose.position.x = coords['x']
        entity_state.pose.position.y = coords['y']
        entity_state.pose.position.z = 0.0
        entity_state.reference_frame = 'world'
        
        request.state = entity_state
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        if future.result() is not None:
            node.get_logger().info(
                f"Successfully set '{name}' to x={coords['x']}, y={coords['y']}"
            )
        else:
            node.get_logger().error(f"Failed to set entity '{name}'")


def main(args=None):
    rclpy.init(args=args)

    # Dictionary of entities to set in Gazebo:
    positions_dict = {
        'workcell_bin': {'x': 0.09, 'y': 0.66},
        'workcell_bin_clone': {'x': 0.188, 'y': -6.4},
        'workcell_bin_clone_0': {'x': 6.65, 'y': -1.10},
        'workcell_bin_clone_1': {'x': 6.0, 'y': 6.0},
        'workcell_bin_clone_2': {'x': 9.64, 'y': -5.57},
        'workcell_bin_clone_3': {'x': -4.9, 'y': 4.48}
    }
    
    # Create a temporary node just for setting entity positions
    temp_node = Node('set_entity_positions_node')
    set_entities_positions(temp_node, positions_dict)
    temp_node.destroy_node()
    
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
