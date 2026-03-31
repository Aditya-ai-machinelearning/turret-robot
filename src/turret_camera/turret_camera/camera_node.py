import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(0.033, self.run)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.get_logger().info('Camera node started!')

    def run(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('No frame')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()