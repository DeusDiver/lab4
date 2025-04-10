#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CannyEdgeNode(Node):
    """
    En node for å anvende Canny Edge-detektering på et bilde.
    """
    def __init__(self):
        """
        Initialiserer noden og oppretter nødvendige publishers og subscribers.
        """
        # Endret nodenavnet til 'canny_edge'
        super().__init__('canny_edge')

        # Abonnerer på input-topic (skal remappes via launch-fil hvis ønskelig)
        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10)
        self.subscription  # For å unngå advarsel om ubrukt variabel

        # Publisher for det behandlede (kantede) bildet
        self.publisher = self.create_publisher(
            Image,
            'output_image',
            10)
  
        # Initialiserer CVBridge for konvertering mellom ROS Image og OpenCV-format
        self.bridge = CvBridge()
    

    def image_callback(self, msg):
        """
        Callback-funksjon for innkommende bildemelding.
        Tar inn bildet, anvender Canny Edge-detektering, og publiserer det prosesserte bildet.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
        
        # Bruker Canny Edge-detektering med terskelverdier 100 og 200
        cv_edge = cv2.Canny(cv_image, 100, 200)
        # Siden Canny-detektering gir et enkeltkanals gråtonebilde, konverterer vi det til BGR-format for kompatibilitet med "bgr8"
        cv_edge_color = cv2.cvtColor(cv_edge, cv2.COLOR_GRAY2BGR)

        # Konverterer det prosesserte bildet tilbake til en ROS Image-melding
        try:
            edge_msg = self.bridge.cv2_to_imgmsg(cv_edge_color, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
 
        # Publiserer det kantede bildet
        self.publisher.publish(edge_msg)

def main(args=None):
    rclpy.init(args=args)
    canny_edge_node = CannyEdgeNode()
    rclpy.spin(canny_edge_node)
    # Riktig variabel skal ødelegges
    canny_edge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
