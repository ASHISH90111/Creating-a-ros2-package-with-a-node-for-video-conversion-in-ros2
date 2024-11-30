#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
from rclpy.service import Service
from example_interfaces.srv import SetBool

class ImageConversionNode(Node):
    def __init__(self):
        super().__init__('image_conversion_node')

        self.declare_parameter('input_topic', '/usb_cam/image_raw')
        self.declare_parameter('output_topic', '/image_conversion/output_image')
        
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.bridge = CvBridge()
        self.mode = 'color'  # Default mode is color

        self.image_subscriber = self.create_subscription(
            Image, 
            self.input_topic, 
            self.image_callback, 
            10
        )

        self.image_publisher = self.create_publisher(
            Image, 
            self.output_topic, 
            10
        )

        self.create_service(
            SetBool, 
            'set_mode', 
            self.set_mode_callback
        )

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            if self.mode == 'grayscale':
                # Convert to grayscale
                gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                ros_image = self.bridge.cv2_to_imgmsg(gray_image, 'mono8')
            else:
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')

            # Publish the processed image
            self.image_publisher.publish(ros_image)

        except Exception as e:
            self.get_logger().error(f"Error in image processing: {str(e)}")

    def set_mode_callback(self, request, response):
        # Switch the mode based on the service request
        if request.data:
            self.mode = 'grayscale'
            response.success = True
            response.message = 'Mode set to grayscale'
        else:
            self.mode = 'color'
            response.success = True
            response.message = 'Mode set to color'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ImageConversionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
