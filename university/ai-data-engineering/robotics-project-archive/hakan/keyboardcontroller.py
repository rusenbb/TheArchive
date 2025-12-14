#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.executors import SingleThreadedExecutor
import threading
import sys
import termios
import tty
import select
import time

class MoveForwardAndMoveModel(Node):
    def __init__(self):
        super().__init__('keyboard_control_move_model')
        
        # Publisher to send velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Client to call the service to set entity state
        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # Wait for the service to be available
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')
        
        # Set up a one-time timer to move the model after 5 seconds
        self.create_timer(5.0, self.move_model_callback)
        self.model_moved = False  # Flag to ensure the model is moved only once
        
        # Subscribe to the Occupancy Grid
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Optionally subscribe to LaserScan if you need direct access
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Initialize storage for map data
        self.current_map = None
        self.current_scan = None

        # Keyboard control variables
        self.key_thread = threading.Thread(target=self.keyboard_listener)
        self.key_thread.daemon = True
        self.key_thread.start()

        # To handle graceful shutdown
        self.shutdown_flag = False

    def move_model_callback(self):
        if not self.model_moved:
            model_name = 'workcell_bin_clone'
            target_x = 99.0
            target_y = 99.0
            self.get_logger().info(f'Attempting to move {model_name} to x={target_x}, y={target_y}')
            self.move_model(model_name, target_x, target_y)
            self.model_moved = True  # Ensure this callback is executed only once

    def move_model(self, model_name, x, y):
        # Create the service request
        entity_state = EntityState()
        entity_state.name = model_name
        entity_state.pose = Pose()
        entity_state.pose.position.x = x
        entity_state.pose.position.y = y
        entity_state.pose.position.z = 0.0  # Assuming ground level
        entity_state.pose.orientation.w = 1.0  # No rotation
        entity_state.twist = Twist()  # Reset velocities
        entity_state.reference_frame = 'world'
        
        request = SetEntityState.Request()
        request.state = entity_state

        # Call the service
        future = self.set_entity_state_client.call_async(request)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Model moved successfully.')
            else:
                self.get_logger().error(f'Failed to move model: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def map_callback(self, msg: OccupancyGrid):
        self.current_map = msg
        self.get_logger().debug('Received updated map.')
        # Process the occupancy grid as needed
        # For example, print the map size
        width = msg.info.width
        height = msg.info.height
        self.get_logger().debug(f'Map size: {width} x {height}')
        # To print the entire map data (be cautious as it can be large)
        # self.get_logger().info(f'Occupancy Grid Data: {msg.data}')

    def scan_callback(self, msg: LaserScan):
        self.current_scan = msg
        # Process the laser scan data as needed
        # For example, print the number of ranges
        num_ranges = len(msg.ranges)
        self.get_logger().debug(f'Received LaserScan with {num_ranges} ranges.')

    def keyboard_listener(self):
        """
        Listens to keyboard inputs and publishes Twist messages accordingly.
        """
        # Save the terminal settings
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            self.get_logger().info("Keyboard control started. Use W/A/S/D to move, X to stop, Q to quit.")

            while not self.shutdown_flag:
                if self.is_data():
                    key = sys.stdin.read(1)
                    if key:
                        twist = Twist()
                        if key.lower() == 'w':
                            twist.linear.x = 0.5
                            twist.angular.z = 0.0
                            self.get_logger().info('Moving Forward')
                        elif key.lower() == 's':
                            twist.linear.x = -0.5
                            twist.angular.z = 0.0
                            self.get_logger().info('Moving Backward')
                        elif key.lower() == 'a':
                            twist.linear.x = 0.0
                            twist.angular.z = 0.5
                            self.get_logger().info('Turning Left')
                        elif key.lower() == 'd':
                            twist.linear.x = 0.0
                            twist.angular.z = -0.5
                            self.get_logger().info('Turning Right')
                        elif key.lower() == 'x':
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.get_logger().info('Stopping')
                        elif key.lower() == 'q':
                            self.get_logger().info('Quitting...')
                            self.shutdown_flag = True
                            break
                        else:
                            continue  # Ignore other keys
                        self.publisher.publish(twist)
                else:
                    time.sleep(0.05)  # Sleep briefly to reduce CPU usage
        finally:
            # Restore the terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.get_logger().info("Keyboard control stopped.")

    def is_data(self):
        """
        Checks if there is keyboard input available.
        """
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardAndMoveModel()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_flag = True
        node.key_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
