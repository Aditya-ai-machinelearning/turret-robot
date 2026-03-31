import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Bool
import json

FRAME_W = 640
FRAME_H = 480

class TrackerNode(Node):

    def __init__(self):
        super().__init__('tracker_node')

        # default target
        self.target_class = 'bottle'

        # subscribers
        self.sub = self.create_subscription(
            String, '/detections', self.on_detection, 10)

        # listen for target changes from dashboard
        self.sub_target = self.create_subscription(
            String, '/set_target', self.on_set_target, 10)

        # publishers
        self.pub_error = self.create_publisher(
            Float32MultiArray, '/target_error', 10)
        self.pub_detected = self.create_publisher(
            Bool, '/target_detected', 10)

        self.get_logger().info('Tracker node started!')

    def on_set_target(self, msg):
        self.target_class = msg.data
        self.get_logger().info(f'Now tracking: {self.target_class}')

    def on_detection(self, msg):
        detections = json.loads(msg.data)

        # filter only our target
        targets = [d for d in detections
                   if d['class'] == self.target_class]

        detected_msg = Bool()

        if not targets:
            detected_msg.data = False
            self.pub_detected.publish(detected_msg)
            return

        detected_msg.data = True
        self.pub_detected.publish(detected_msg)

        # pick most confident detection
        best = max(targets, key=lambda d: d['conf'])

        # find center of bounding box
        cx = (best['x1'] + best['x2']) / 2
        cy = (best['y1'] + best['y2']) / 2

        # calculate error from frame center
        error_x = cx - (FRAME_W / 2)
        error_y = cy - (FRAME_H / 2)

        # publish error
        out = Float32MultiArray()
        out.data = [error_x, error_y]
        self.pub_error.publish(out)

        self.get_logger().info(
            f'Target: {self.target_class} | '
            f'error_x={error_x:.1f} error_y={error_y:.1f}')

def main():
    rclpy.init()
    node = TrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()