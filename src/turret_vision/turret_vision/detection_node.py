import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import json
import cv2

# YOLOv8 class numbers for common objects
CLASS_MAP = {
    'bottle': 39,
    'cup': 41,
    'person': 0,
    'cell phone': 67,
    'sports ball': 32,
    'chair': 56,
    'laptop': 63,
    'book': 73,
    'cat': 15,
    'dog': 16,
}

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        self.sub_image = self.create_subscription(
            Image, '/image_raw', self.on_image, 10)

        # listen for target changes from dashboard
        self.sub_target = self.create_subscription(
            String, '/set_target', self.on_set_target, 10)

        self.pub = self.create_publisher(String, '/detections', 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8s.pt')

        # default target
        self.target_class = 'bottle'
        self.target_id = [39]

        self.get_logger().info('Detection node started!')

    def on_set_target(self, msg):
        new_target = msg.data
        if new_target in CLASS_MAP:
            self.target_class = new_target
            self.target_id = [CLASS_MAP[new_target]]
            self.get_logger().info(
                f'Now detecting: {self.target_class} '
                f'(class {self.target_id[0]})')
        else:
            # unknown class — detect everything
            self.target_class = new_target
            self.target_id = None
            self.get_logger().info(
                f'Detecting all classes, filtering: {new_target}')

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.model(
            frame,
            verbose=False,
            classes=self.target_id,
            conf=0.25
        )

        detections = []
        for box in results[0].boxes:
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            cls = self.model.names[int(box.cls[0])]

            detections.append({
                'class': cls,
                'conf': round(conf, 2),
                'x1': x1, 'y1': y1,
                'x2': x2, 'y2': y2
            })

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{cls} {conf:.0%}',
                (x1, y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(frame, f'cx={cx} cy={cy}',
                (x1, y2 + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            self.get_logger().info(
                f'Detected: {cls} ({conf:.0%}) center=({cx},{cy})')

        # draw crosshair
        h, w = frame.shape[:2]
        cx_f, cy_f = w // 2, h // 2
        cv2.line(frame, (cx_f-20, cy_f),
                 (cx_f+20, cy_f), (0, 0, 255), 2)
        cv2.line(frame, (cx_f, cy_f-20),
                 (cx_f, cy_f+20), (0, 0, 255), 2)
        cv2.putText(frame, 'CENTER',
            (cx_f+5, cy_f-5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imshow('Turret Vision', frame)
        cv2.waitKey(1)

        out = String()
        out.data = json.dumps(detections)
        self.pub.publish(out)

def main():
    rclpy.init()
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()