#!/usr/bin/env python3
## EB
## motion.py
## 
## YZV406E Assignment 1 Skeleton
##
## Instructions: Implement the necessary functions to make the robot navigate around the world. 
## The robot should navigate around the world without colliding with obstacles.
## Your aim is to explore the world as most as the robot can.
## The robot should be able to enter the rooms and exit the rooms by sensing the environment.
## You may add helper functions to make code seem more clean. As long as you keep the main() function as it is, you can add as many functions as you want,
## and modify existing ones.
## 
## Notes to consier: Few helper functions and code snippets are already given to you. Examine the code carefully beforehand.
## You can also examine the explored area with mapping_callback function. You can utilize the received map data, it might be helpful for your solution.
##
## 
## STUDENT_ID:<150220755>
### DO NOT EDIT THESE LIBRARIES ###
import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import math
import numpy as np
### YOU CAN IMPORT STANDARD PACKAGES IF NEEDED BELOW HERE ###
### YOU CANNOT IMPORT PACKAGES THAT NEARLY IMPLEMENTS 
### THE DESIRED EXPLORATION ALGORITHM!!! ###



### ###

"""
HELPER FUNCTIONS
"""
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
        


class TurtleExplorer(Node):
    """
    Navigator node to make robot go from location A to B. 
    [IMPORTANT]
    IMPLEMENT YOUR CODES WITHIN THIS CLASS (You can define helper functions outside the class if you want)
    [IMPORTANT]
    """
    def __init__(self):
        super().__init__('turtle_explorer')
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.publish_twist = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.tf_buffer = Buffer() # for transformation
        self.tf_listener = TransformListener(self.tf_buffer, self) # for transformation
        # DO NOT CHANGE self.goal!
        self.robotX_tf = math.nan # initialize as null
        self.robotY_tf = math.nan # initialize as null
        self.robot_yaw = math.nan # initialize as null
        self.odom_counter = 0
        
        # Add new attributes for potential field navigation
        self.map_size = 1000  # Will be updated when first map message arrives
        self.potential_field = None
        self.cell_size = 0.01  # Default map resolution
        self.wall_repulsion = -3000.0  # Strong repulsion from walls
        self.decay_factor = 0.95  # How quickly repulsion/attraction falls off
        self.visited_penalty = -0.01  # Small penalty for visited areas
        self.unexplored_attraction = 120.0  # Attraction to unexplored areas
        
        # For tracking visited areas
        self.visited_positions = set()
        self.last_update = None

        # Add frontier memory
        self.frontier_memory = []  # List of (x, y, timestamp) tuples
        self.frontier_memory_size = 10  # Remember last 10 frontiers
        self.frontier_timeout = 30.0  # Frontiers older than 30 seconds are forgotten
        self.last_frontier_time = self.get_clock().now()

        # Add parameters for movement
        self.normal_speed = 1.5  # Increased from 0.2
        self.turning_speed = 0.9  # Increased from 0.5
        self.slow_speed = 0.3    # For precise movements
        self.rotation_threshold = 0.5  # When to start rotating

    def initialize_potential_field(self, width, height):
        """Initialize the potential field with zeros"""
        self.map_size = (width, height)
        self.potential_field = np.zeros((width, height))

    def update_potential_field(self, map_data, width, height):
        """Update potential field based on map data and visited positions"""
        if self.potential_field is None:
            self.initialize_potential_field(width, height)

        # Reset potential field
        self.potential_field.fill(0)

        # Add wall and obstacle repulsion
        for i in range(width):
            for j in range(height):
                idx = i + j * width
                if map_data[idx] > 65:  # Wall or obstacle
                    self.add_repulsion(i, j, self.wall_repulsion)
                elif map_data[idx] == -1:  # Unexplored
                    self.add_attraction(i, j, self.unexplored_attraction)

        # Add visited positions penalty
        for x, y in self.visited_positions:
            map_x, map_y = self.world_to_map(x, y)
            if 0 <= map_x < width and 0 <= map_y < height:
                self.potential_field[map_x, map_y] += self.visited_penalty

    def add_repulsion(self, x, y, strength):
        """Add repulsive potential around a point"""
        radius = 10  # Affect nearby cells
        for i in range(max(0, x-radius), min(self.map_size[0], x+radius)):
            for j in range(max(0, y-radius), min(self.map_size[1], y+radius)):
                distance = np.sqrt((x-i)**2 + (y-j)**2)
                if distance == 0:
                    continue
                self.potential_field[i, j] += strength * (self.decay_factor ** distance)

    def add_attraction(self, x, y, strength):
        """Add attractive potential around a point"""
        radius = 20  # Larger radius for attraction to unexplored areas
        for i in range(max(0, x-radius), min(self.map_size[0], x+radius)):
            for j in range(max(0, y-radius), min(self.map_size[1], y+radius)):
                distance = np.sqrt((x-i)**2 + (y-j)**2)
                if distance == 0:
                    continue
                self.potential_field[i, j] += strength * (self.decay_factor ** distance)

    def world_to_map(self, x, y):
        """Convert world coordinates to map coordinates"""
        return (
            int((x + self.map_size[0]*self.cell_size/2) / self.cell_size),
            int((y + self.map_size[1]*self.cell_size/2) / self.cell_size)
        )

    def scan_callback(self, msg):
        if math.isnan(self.robotX_tf) or math.isnan(self.robotY_tf):
            self.get_logger().warn('Position not initialized yet, skipping movement')
            return

        # Get new frontier points
        new_frontier_points = self.get_frontier_points(msg)
        
        # Update frontier memory and combine frontiers
        current_time = self.get_clock().now()
        if new_frontier_points:
            for x, y, _ in new_frontier_points:
                self.frontier_memory.append((x, y, current_time))
            self.last_frontier_time = current_time
        
        # Clean old frontiers
        self.frontier_memory = [
            (x, y, t) for x, y, t in self.frontier_memory 
            if (current_time - t).nanoseconds / 1e9 < self.frontier_timeout
        ]
        # Keep only the most recent ones
        self.frontier_memory = self.frontier_memory[-self.frontier_memory_size:]

        # Combine new and remembered frontiers
        all_frontier_points = new_frontier_points + [
            (x, y, 2.0) for x, y, _ in self.frontier_memory
        ]

        # Analyze potential field
        left_potentials = self.analyze_side_potentials(msg, -1)
        right_potentials = self.analyze_side_potentials(msg, 1)
        
        # Remove stuck detection code
        velocity_vec = self.calculate_movement(msg, current_time)
        self.publish_twist.publish(velocity_vec)

    def get_frontier_points(self, scan_msg):
        """Find points at the edge of empty space where scanner can't reach"""
        frontier_points = []
        angle = scan_msg.angle_min
        
        # Convert scan data to points
        for i, distance in enumerate(scan_msg.ranges):
            # Look for points where scanner shows infinity/nan (unreachable space)
            if math.isinf(distance) or math.isnan(distance):
                # Look at neighboring points to find valid distances
                prev_idx = (i - 1) % len(scan_msg.ranges)
                next_idx = (i + 1) % len(scan_msg.ranges)
                
                # If we have a valid measurement next to an infinite one,
                # that's our frontier edge
                if not math.isinf(scan_msg.ranges[prev_idx]) and not math.isnan(scan_msg.ranges[prev_idx]):
                    # Use the last valid measurement to place our frontier point slightly beyond it
                    valid_distance = scan_msg.ranges[prev_idx] + 0.5  # 0.5m beyond last valid point
                    x = self.robotX_tf + valid_distance * math.cos(angle + self.robot_yaw)
                    y = self.robotY_tf + valid_distance * math.sin(angle + self.robot_yaw)
                    frontier_points.append((x, y, valid_distance))
                
                elif not math.isinf(scan_msg.ranges[next_idx]) and not math.isnan(scan_msg.ranges[next_idx]):
                    # Same for the next valid measurement
                    valid_distance = scan_msg.ranges[next_idx] + 0.5  # 0.5m beyond last valid point
                    x = self.robotX_tf + valid_distance * math.cos(angle + self.robot_yaw)
                    y = self.robotY_tf + valid_distance * math.sin(angle + self.robot_yaw)
                    frontier_points.append((x, y, valid_distance))
                    
            angle += scan_msg.angle_increment
        
        return frontier_points

    def choose_best_frontier(self, frontier_points):
        """Choose the most promising frontier point based on distance, angle, and age"""
        if not frontier_points:
            return None
        
        # Score each frontier point
        scored_points = []
        current_time = self.get_clock().now()
        
        for x, y, distance in frontier_points:
            # Calculate angle to point
            dx = x - self.robotX_tf
            dy = y - self.robotY_tf
            angle = math.atan2(dy, dx)
            
            # Calculate angle difference from current robot orientation
            angle_diff = abs(angle - self.robot_yaw)
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            angle_diff = abs(angle_diff)
            
            # Calculate age penalty for remembered frontiers
            age_penalty = 0
            for mem_x, mem_y, timestamp in self.frontier_memory:
                if abs(x - mem_x) < 0.1 and abs(y - mem_y) < 0.1:
                    age = (current_time - timestamp).nanoseconds / 1e9
                    age_penalty = -0.5 * age / self.frontier_timeout
                    break
            
            # Score calculation
            distance_score = -((distance - 2.0) ** 2)  # Optimal distance around 2 meters
            angle_score = -2.0 * angle_diff  # Prefer points ahead
            score = distance_score + angle_score + age_penalty
            
            scored_points.append((score, (x, y)))
        
        # Return the point with highest score
        best_point = max(scored_points, key=lambda x: x[0])[1]
        self.get_logger().info(f'Selected frontier point at {best_point}')
        return best_point

    def map_callback(self, msg):
        # Basic map logging
        self.get_logger().info(f'Received map update. Width: {msg.info.width}, Height: {msg.info.height}')
        
        # Update cell size
        self.cell_size = msg.info.resolution
        
        # Count explored vs unexplored cells
        total_cells = len(msg.data)
        explored_cells = sum(1 for cell in msg.data if cell != -1)
        self.get_logger().info(f'Explored cells: {explored_cells}/{total_cells} ({explored_cells/total_cells*100:.2f}%)')

    def odom_callback(self, msg):

         
        robotX = msg.pose.pose.position.x
        robotY = msg.pose.pose.position.y
        
        to_frame_rel = "odom"
        from_frame_rel = "base_footprint"
        try:
            # grab the latest available transform from the odometry frame 
            # (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        _,_,robot_orient_z = euler_from_quaternion(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)
        
        robotX = t.transform.translation.x
        robotY = t.transform.translation.y
        if((self.odom_counter==0)):
            self.initial_robot_pose = {'x':robotX,'y':robotY,'yaw':robot_orient_z}
        else:
            pass
            
        self.robotX_tf = robotX
        self.robotY_tf = robotY
        self.robot_yaw = robot_orient_z # # only need the z axis, degree of orientation, between pi and -pi
        self.get_logger().info('X:'+str(self.robotX_tf),throttle_duration_sec=0.5) # once at a half of a second
        self.get_logger().info('Y:'+str(self.robotY_tf),throttle_duration_sec=0.5) # once at a half of a second
        self.get_logger().info('Yaw:'+str(self.robot_yaw),throttle_duration_sec=0.5) # once at a half of a second

        self.odom_counter+=1



        
    def analyze_side_potentials(self, scan_msg, side):
        """Analyze potential field on one side (side: -1 for left, 1 for right)"""
        if side == -1:
            ranges = scan_msg.ranges[75:105]  # Left side
        else:
            ranges = scan_msg.ranges[-105:-75]  # Right side
            
        valid_ranges = [r for r in ranges if not math.isinf(r) and not math.isnan(r)]
        if not valid_ranges:
            return 0.0
            
        # Calculate potential based on distances and frontiers
        potential = sum(1.0 / (r + 0.1) for r in valid_ranges) / len(valid_ranges)
        return potential

    def calculate_movement(self, msg, current_time):
        """Calculate movement considering obstacles and frontiers"""
        velocity_vec = Twist()
        
        # Get base movement direction from frontiers and obstacles
        base_angle = self.get_base_movement_angle(msg)
        final_angle = base_angle  # Simplified without temporary potentials

        if final_angle is not None:
            # Calculate angle difference
            angle_diff = final_angle - self.robot_yaw
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # Check for obstacles
            front_arc = [r for r in (msg.ranges[-15:] + msg.ranges[:15]) if not math.isinf(r) and not math.isnan(r)]
            min_front_distance = 0.5 if not front_arc else min(front_arc)

            if min_front_distance < 0.5:
                # Obstacle avoidance
                velocity_vec.linear.x = 0.0
                velocity_vec.angular.z = self.turning_speed * (1.0 if angle_diff > 0 else -1.0)
            else:
                # Normal movement
                if abs(angle_diff) > self.rotation_threshold:
                    velocity_vec.linear.x = self.slow_speed
                    velocity_vec.angular.z = self.turning_speed * (1.0 if angle_diff > 0 else -1.0)
                else:
                    velocity_vec.linear.x = self.normal_speed
                    velocity_vec.angular.z = angle_diff * 0.7
        else:
            # No direction found, rotate to search
            velocity_vec.linear.x = 0.0
            velocity_vec.angular.z = self.turning_speed

        return velocity_vec

    def get_base_movement_angle(self, msg):
        """Calculate base movement direction from frontiers and obstacles"""
        # Get frontier points
        frontier_points = self.get_frontier_points(msg)
        if frontier_points:
            best_point = self.choose_best_frontier(frontier_points)
            dx = best_point[0] - self.robotX_tf
            dy = best_point[1] - self.robotY_tf
            return math.atan2(dy, dx)
        return None

def main(args=None):
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    rclpy.init(args=args)

    navigator_node = TurtleExplorer()
    rclpy.spin(navigator_node) 

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
   
    
    navigator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()