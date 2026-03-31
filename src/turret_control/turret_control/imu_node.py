import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        # publishes IMU compensation values to PID node
        self.pub = self.create_publisher(
            Float32MultiArray, '/imu_compensation', 10)

        # timer reads IMU 50 times per second
        self.timer = self.create_timer(0.02, self.read_imu)

        # internal state
        self.prev_time = time.time()
        self.gyro_x = 0.0
        self.gyro_y = 0.0

        # ── WITH HARDWARE uncomment:
        # import smbus2
        # self.bus = smbus2.SMBus(1)
        # self.MPU = 0x68
        # self.bus.write_byte_data(self.MPU, 0x6B, 0)
        # self.get_logger().info('IMU MPU6050 initialized!')

        self.get_logger().info('IMU node started! (simulation mode)')

    def read_imu(self):
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        # ── WITHOUT HARDWARE: simulate zero movement
        gyro_x = 0.0
        gyro_y = 0.0

        # ── WITH HARDWARE replace above with:
        # raw = self.bus.read_i2c_block_data(
        #     self.MPU, 0x43, 4)
        # gyro_x = (raw[0] << 8 | raw[1]) / 131.0
        # gyro_y = (raw[2] << 8 | raw[3]) / 131.0
        # if gyro_x > 32768: gyro_x -= 65536
        # if gyro_y > 32768: gyro_y -= 65536

        # integrate to get angle change
        self.gyro_x += gyro_x * dt
        self.gyro_y += gyro_y * dt

        # publish compensation
        msg = Float32MultiArray()
        msg.data = [self.gyro_x, self.gyro_y]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = IMUNode()
    rclpy.spin(node)
    rclpy.shutdown()