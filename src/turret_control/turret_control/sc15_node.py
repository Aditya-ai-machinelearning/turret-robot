import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SC15Node(Node):

    def __init__(self):
        super().__init__('sc15_node')

        # subscribe to servo commands from pid_node
        self.sub = self.create_subscription(
            Float32MultiArray, '/servo_cmd', self.on_cmd, 10)

        # current servo positions
        self.pan_angle = 135.0
        self.tilt_angle = 135.0

        # ── SIMULATION MODE (no hardware yet)
        # When Pi + SC15 connected, uncomment these lines:
        # from scservo_sdk import *
        # self.port = PortHandler('/dev/ttyUSB0')
        # self.packet = PacketHandler(0)
        # self.port.openPort()
        # self.port.setBaudRate(1000000)

        self.get_logger().info('SC15 node started! (simulation mode)')

    def on_cmd(self, msg):
        self.pan_angle = msg.data[0]
        self.tilt_angle = msg.data[1]

        # ── WITHOUT HARDWARE: just print
        self.get_logger().info(
            f'[SIM] Pan servo → {self.pan_angle:.1f}°  '
            f'Tilt servo → {self.tilt_angle:.1f}°')

        # ── WITH HARDWARE: uncomment these lines
        # pan_pos = int(self.pan_angle / 300 * 4096)
        # tilt_pos = int(self.tilt_angle / 300 * 4096)
        # self.packet.write4ByteTxRx(self.port, 1, 116, pan_pos)
        # self.packet.write4ByteTxRx(self.port, 2, 116, tilt_pos)

def main():
    rclpy.init()
    node = SC15Node()
    rclpy.spin(node)
    rclpy.shutdown()