import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String
import time

LOCK_THRESHOLD = 50.0
LOCK_HOLD_TIME = 0.3  # reduced — faster lock

class PIDNode(Node):

    def __init__(self):
        super().__init__('pid_node')

        # subscribers
        self.sub_error = self.create_subscription(
            Float32MultiArray, '/target_error',
            self.on_error, 10)

        # IMU compensation subscriber
        self.sub_imu = self.create_subscription(
            Float32MultiArray, '/imu_compensation',
            self.on_imu, 10)

        # publishers
        self.pub_servo = self.create_publisher(
            Float32MultiArray, '/servo_cmd', 10)
        self.pub_locked = self.create_publisher(
            Bool, '/locked', 10)
        self.pub_status = self.create_publisher(
            String, '/turret_status', 10)

        # PID gains
        self.kp = 0.003

        # servo positions
        self.pan_angle = 135.0
        self.tilt_angle = 135.0

        # IMU compensation values
        self.imu_pan = 0.0
        self.imu_tilt = 0.0
        self.prev_imu_pan = 0.0
        self.prev_imu_tilt = 0.0

        # lock state
        self.lock_start_time = None
        self.is_locked = False
        self.prev_time = time.time()

        self.get_logger().info('PID node started!')

    def on_imu(self, msg):
        # get IMU angle change since last reading
        new_pan = msg.data[0]
        new_tilt = msg.data[1]

        # calculate delta — how much car moved
        delta_pan = new_pan - self.prev_imu_pan
        delta_tilt = new_tilt - self.prev_imu_tilt

        # compensate servo for car movement
        # if car turns right, servo moves left to stay on target
        self.pan_angle -= delta_pan
        self.tilt_angle -= delta_tilt

        # clamp
        self.pan_angle = max(30.0, min(250.0, self.pan_angle))
        self.tilt_angle = max(60.0, min(200.0, self.tilt_angle))

        self.prev_imu_pan = new_pan
        self.prev_imu_tilt = new_tilt

    def on_error(self, msg):
        error_x = msg.data[0]
        error_y = msg.data[1]

        now = time.time()
        dt = now - self.prev_time
        if dt <= 0:
            return
        self.prev_time = now

        # proportional control
        pan_delta = self.kp * error_x
        tilt_delta = self.kp * error_y

        # limit step size
        pan_delta = max(-2.0, min(2.0, pan_delta))
        tilt_delta = max(-2.0, min(2.0, tilt_delta))

        # update angles
        self.pan_angle += pan_delta
        self.tilt_angle += tilt_delta

        # clamp
        self.pan_angle = max(30.0, min(250.0, self.pan_angle))
        self.tilt_angle = max(60.0, min(200.0, self.tilt_angle))

        # publish servo command
        servo_msg = Float32MultiArray()
        servo_msg.data = [self.pan_angle, self.tilt_angle]
        self.pub_servo.publish(servo_msg)

        # check lock
        currently_locked = (abs(error_x) < LOCK_THRESHOLD and
                            abs(error_y) < LOCK_THRESHOLD)

        if currently_locked:
            if self.lock_start_time is None:
                self.lock_start_time = now
                self.get_logger().info(
                    '*** TARGET ACQUIRED ***')

            lock_duration = now - self.lock_start_time
            if lock_duration >= LOCK_HOLD_TIME and not self.is_locked:
                self.is_locked = True
                self.get_logger().info(
                    '*** LOCKED — FIRING! ***')
        else:
            if self.lock_start_time is not None:
                self.lock_start_time = None
                self.is_locked = False

        # publish locked
        lock_msg = Bool()
        lock_msg.data = self.is_locked
        self.pub_locked.publish(lock_msg)

        # publish status
        status = String()
        if self.is_locked:
            status.data = 'LOCKED'
        elif self.lock_start_time is not None:
            status.data = 'ACQUIRING'
        else:
            status.data = f'TRACKING ex={error_x:.0f} ey={error_y:.0f}'
        self.pub_status.publish(status)

        self.get_logger().info(
            f'Pan={self.pan_angle:.1f}° '
            f'Tilt={self.tilt_angle:.1f}° '
            f'ex={error_x:.1f} ey={error_y:.1f}')

def main():
    rclpy.init()
    node = PIDNode()
    rclpy.spin(node)
    rclpy.shutdown()