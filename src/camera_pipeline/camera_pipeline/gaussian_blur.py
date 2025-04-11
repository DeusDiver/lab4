import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class GaussianBlurNode(Node):
    """
    A node for blurring an image
    """
    def __init__(self):
        super().__init__('gaussian_blur')
        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            Image,
            'output_image',
            10)
        self.bridge = CvBridge()
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
        cv_blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        try:
            blur_msg = self.bridge.cv2_to_imgmsg(cv_blurred, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
        self.publisher.publish(blur_msg)

def ComposableNode(args=None):
    return GaussianBlurNode()

def main(args=None):
    rclpy.init(args=args)
    image_blur_node = GaussianBlurNode()
    rclpy.spin(image_blur_node)
    image_blur_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
