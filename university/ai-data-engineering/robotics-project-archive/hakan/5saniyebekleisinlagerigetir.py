#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, Pose, Twist as GeometryTwist
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

class MoveForwardAndMoveModel(Node):
    def __init__(self):
        super().__init__('move_forward_and_move_model')

        # 1. Publisher to move the robot forward
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.move_timer = self.create_timer(0.1, self.move_forward)

        # 2. Client to call the service to set entity state
        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # Wait for the set_entity_state service to be available
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')

        # Model name and desired coordinates
        self.model_name = 'workcell_bin_clone'
        self.first_x, self.first_y = 99.0, 99.0   # Where to move after 5 seconds
        self.second_x, self.second_y = 0.0, 0.0   # Where to move after 10 seconds (original or any other location)

        # 3. Create a one-time timer (5s) to move the model to (99,99)
        self.first_move_timer = self.create_timer(5.0, self.move_model_callback_first)
        self.first_move_done = False

        # 4. Second timer will be created dynamically after the first move to wait another 5 seconds
        self.second_move_timer = None
        self.second_move_done = False

    def move_forward(self):
        """
        Publishes a constant forward velocity to /cmd_vel.
        """
        velocity = Twist()
        velocity.linear.x = 0.5  # Forward speed
        velocity.angular.z = 0.0
        self.publisher.publish(velocity)

    def move_model_callback_first(self):
        """
        After 5 seconds, move the model to (99, 99) and then schedule the second move.
        """
        if not self.first_move_done:
            self.get_logger().info(f'[5s] Moving {self.model_name} to (x={self.first_x}, y={self.first_y}).')
            self.move_model(self.model_name, self.first_x, self.first_y)
            self.first_move_done = True

            # Schedule the second move after an additional 5 seconds
            self.second_move_timer = self.create_timer(5.0, self.move_model_callback_second)

    def move_model_callback_second(self):
        """
        After another 5 seconds (10s total), move the model back to (0, 0).
        """
        if not self.second_move_done:
            self.get_logger().info(f'[10s] Moving {self.model_name} back to (x={self.second_x}, y={self.second_y}).')
            self.move_model(self.model_name, self.second_x, self.second_y)
            self.second_move_done = True

    def move_model(self, model_name, x, y):
        """
        Calls /gazebo/set_entity_state to move the specified model to the desired (x, y).
        """
        entity_state = EntityState()
        entity_state.name = model_name
        entity_state.pose = Pose()
        entity_state.pose.position.x = x
        entity_state.pose.position.y = y
        entity_state.pose.position.z = 0.0
        entity_state.pose.orientation.w = 1.0  # No rotation
        entity_state.twist = GeometryTwist()
        entity_state.reference_frame = 'world'

        request = SetEntityState.Request()
        request.state = entity_state

        future = self.set_entity_state_client.call_async(request)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        """
        Logs the result of the set_entity_state service call.
        """
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
