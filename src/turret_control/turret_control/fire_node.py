import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import threading
import time

# timing
TRIGGER_TIME = 0.3       # seconds to push bullet
RESET_TIME = 0.3         # seconds to reset trigger
COOLDOWN_TIME = 2.0      # seconds before firing again

class FireNode(Node):

    def __init__(self):
        super().__init__('fire_node')

        # subscribe to locked signal
        self.sub_locked = self.create_subscription(
            Bool, '/locked', self.on_locked, 10)

        # subscribe to target detected signal
        self.sub_detected = self.create_subscription(
            Bool, '/target_detected', self.on_detected, 10)

        self.is_firing = False
        self.last_fire_time = 0.0
        self.bldc_spinning = False

        # ── WITH HARDWARE uncomment:
        # import RPi.GPIO as GPIO
        # self.BLDC_PIN_1 = 17
        # self.BLDC_PIN_2 = 18
        # self.TRIGGER_PIN = 27
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.BLDC_PIN_1, GPIO.OUT)
        # GPIO.setup(self.BLDC_PIN_2, GPIO.OUT)
        # GPIO.setup(self.TRIGGER_PIN, GPIO.OUT)
        # self.bldc1 = GPIO.PWM(self.BLDC_PIN_1, 50)
        # self.bldc2 = GPIO.PWM(self.BLDC_PIN_2, 50)
        # self.bldc1.start(0)
        # self.bldc2.start(0)

        self.get_logger().info('Fire node started!')

    def on_detected(self, msg):
        # as soon as target is detected spin up BLDCs
        if msg.data and not self.bldc_spinning:
            self.bldc_spinning = True
            self.get_logger().info(
                'Target detected — spinning up BLDCs...')
            # ── WITH HARDWARE:
            # self.bldc1.ChangeDutyCycle(100)
            # self.bldc2.ChangeDutyCycle(100)

        elif not msg.data and self.bldc_spinning:
            self.bldc_spinning = False
            self.get_logger().info(
                'Target lost — spinning down BLDCs...')
            # ── WITH HARDWARE:
            # self.bldc1.ChangeDutyCycle(0)
            # self.bldc2.ChangeDutyCycle(0)

    def on_locked(self, msg):
        if not msg.data:
            return
        if self.is_firing:
            return

        now = time.time()
        if now - self.last_fire_time < COOLDOWN_TIME:
            return

        self.is_firing = True
        self.last_fire_time = now
        thread = threading.Thread(target=self.fire_sequence)
        thread.daemon = True
        thread.start()

    def fire_sequence(self):
        self.get_logger().info('*** LOCKED — FIRING! ***')

        # push bullet into chamber
        # ── WITH HARDWARE:
        # GPIO.output(self.TRIGGER_PIN, GPIO.HIGH)
        time.sleep(TRIGGER_TIME)

        # reset trigger
        # ── WITH HARDWARE:
        # GPIO.output(self.TRIGGER_PIN, GPIO.LOW)
        time.sleep(RESET_TIME)

        self.get_logger().info('*** FIRED! ***')
        self.is_firing = False

def main():
    rclpy.init()
    node = FireNode()
    rclpy.spin(node)
    rclpy.shutdown()