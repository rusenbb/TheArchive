#!/usr/bin/env python3
from re import X
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import math
from time import time
import subprocess
class Tracker(Node):

    def __init__(self):
        super().__init__('tracker')

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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_counter = 0
        self.start = -1
        self.end = -1
        self.hasStarted = False
        self.robotX_tf = math.nan # initialize as null
        self.robotY_tf = math.nan # initialize as null
        # create a timer to publish elapsed time
        self.timer_counter = self.create_timer(1, self.timer_callback)
        self.map_save_timer = None  # Timer for map saving, created dynamically
        self.map_saved = False

    def map_callback(self, msg):
        # Retrieve map metadata
        map_width = msg.info.width
        map_height = msg.info.height
        resolution = msg.info.resolution

        # Calculate map size in meters
        map_width_meters = map_width * resolution
        map_height_meters = map_height * resolution

        # Log map size
        self.get_logger().info(f"Map size: {map_width_meters:.2f} x {map_height_meters:.2f} meters")

        # Analyze explored area
        data = msg.data
        explored_count = sum(1 for cell in data if cell != -1)
        total_cells = len(data)

        # Calculate explored percentage
        explored_percentage = (explored_count / total_cells) * 100
        self.get_logger().info(f"Explored area: {explored_percentage:.2f}%")
        


    def odom_callback(self, msg):
        robotX = msg.pose.pose.position.x
        robotY = msg.pose.pose.position.y
        to_frame_rel = "odom"
        from_frame_rel = "base_footprint"
    
        velocity_vec = Twist() # to fill the velocity message 

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        self.robotX_tf = t.transform.translation.x
        self.robotY_tf = t.transform.translation.y
        
        if ((round(msg.twist.twist.linear.x,2) > 0.0) | (round(msg.twist.twist.angular.z,2) > 0.0)):
            
            if (self.move_counter==0):
                self.get_logger().info('Clock has started, RUN TURTLEBOT')
                self.start = time()
                self.hasStarted = True
                self.start_map_save_timer()  # Start the map saving timer
            self.move_counter+=1
    def start_map_save_timer(self):
        if not self.map_save_timer:
            self.get_logger().info("Starting timer to save the map in 120 seconds...")
            self.map_save_timer = self.create_timer(120.0, self.save_map)
    def timer_callback(self):
        if (self.hasStarted):
            self.end = time()
            elapsed_time = self.end - self.start
            self.get_logger().info('Elapsed Time: '+ str(elapsed_time),throttle_duration_sec=1)
    def save_map(self):
        if not self.map_saved:
            self.get_logger().info("Saving map using CLI command...")
            try:
                # Execute the CLI command
                subprocess.run(
                    ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "my_map"],
                    check=True
                )
                self.get_logger().info("Map successfully saved as my_map.pgm and my_map.yaml")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"Failed to save map: {e}")
            self.map_saved = True  # Ensure the map is saved only once

def main(args=None):
    rclpy.init(args=args)

    ref = Tracker()

    rclpy.spin(ref)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
   
    
    ref.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()