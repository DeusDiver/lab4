import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2

class GaussianBlurNode(Node):
    """
    A node for blurring an image
    """
    def __init__(self):
        """
        Constructs all the necessary attributes for the Gaussian blur node.
        """
        super().__init__('gaussian_blur')

        # Subscribe to a image topic
        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publish the filtered image
        self.publisher = self.create_publisher(
            Image,
            'output_image',
            10)
 
        # Initialize CVBridge
        self.bridge = CvBridge()
    
    def image_callback(self, msg):
        """
        Callback function for input image topic.
        Applies Gaussian blur to the received image and publishes the edges as an image.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
        
        # 
        # Legg koden din her
        # cv_image er bildet du skal bruke som input
        # cv_blurred (en variabel du lager selv) skal være sluttresultatet etter å bruke gaussisk uskarphet.
        
        # Convert back to ROS Image message
        try:
            blur_msg = self.bridge.cv2_to_imgmsg(cv_blurred, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
 
        # Publish the filtered image
        self.publisher.publish(blur_msg)

# Initialize the node
def main(args=None):
    rclpy.init(args=args)
    image_blur_node = GaussianBlurNode()
    rclpy.spin(image_blur_node)
    image_blur_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()