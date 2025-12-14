#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Twist as GeometryTwist
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from rclpy.executors import SingleThreadedExecutor

class MoveForwardAndMoveModel(Node):
    def __init__(self):
        super().__init__('move_forward_and_move_model')
        
        # Publisher to move the robot forward
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.move_timer = self.create_timer(0.1, self.move_forward)
        
        # Client to call the service to set entity state
        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # Wait for the service to be available
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')
        
        # Set up a one-time timer to move the model after 5 seconds
        self.create_timer(5.0, self.move_model_callback, callback_group=None)
        self.model_moved = False  # Flag to ensure the model is moved only once

    def move_forward(self):
        # Publish a constant forward velocity
        velocity = Twist()
        velocity.linear.x = 0.5  # Adjust the speed as needed
        velocity.angular.z = 0.0  # No rotation
        self.publisher.publish(velocity)

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
        entity_state.twist = GeometryTwist()
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

def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardAndMoveModel()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
