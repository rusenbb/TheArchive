#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import json
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
        else:
            self.get_logger().info('No predictions to publish for this frame.')

def main(args=None):
    rclpy.init(args=args)
    
    # You can adjust the frame_skip parameter here
    frame_skip = 5  # Change to 10 for processing 1 out of every 10 frames
    node = YoloInferenceNode(frame_skip=frame_skip)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
