import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Motor speed limit 0-100
MAX_SPEED = 70

class DriveNode(Node):

    def __init__(self):
        super().__init__('drive_node')

        # subscribe to velocity commands
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.on_cmd_vel, 10)

        # ── SIMULATION MODE
        # When Pi connected uncomment:
        # import RPi.GPIO as GPIO
        # self.FL_IN1, self.FL_IN2, self.FL_PWM = 5, 6, 13
        # self.FR_IN1, self.FR_IN2, self.FR_PWM = 19, 26, 12
        # self.RL_IN1, self.RL_IN2, self.RL_PWM = 20, 21, 16
        # self.RR_IN1, self.RR_IN2, self.RR_PWM = 23, 24, 25
        # GPIO.setmode(GPIO.BCM)
        # self.setup_motor(self.FL_IN1, self.FL_IN2, self.FL_PWM)
        # self.setup_motor(self.FR_IN1, self.FR_IN2, self.FR_PWM)
        # self.setup_motor(self.RL_IN1, self.RL_IN2, self.RL_PWM)
        # self.setup_motor(self.RR_IN1, self.RR_IN2, self.RR_PWM)

        self.get_logger().info('Drive node started! (simulation mode)')

    def on_cmd_vel(self, msg):
        # read velocity commands
        x = msg.linear.x    # forward/backward
        y = msg.linear.y    # strafe left/right
        z = msg.angular.z   # rotation

        # omni wheel kinematics
        fl = x - y - z
        fr = x + y + z
        rl = x + y - z
        rr = x - y + z

        # scale to motor speed
        fl = self.scale(fl)
        fr = self.scale(fr)
        rl = self.scale(rl)
        rr = self.scale(rr)

        # ── WITHOUT HARDWARE: just print
        self.get_logger().info(
            f'[SIM] FL={fl}  FR={fr}  RL={rl}  RR={rr}')

        # ── WITH HARDWARE: uncomment
        # self.set_motor(self.FL_IN1, self.FL_IN2, self.FL_PWM, fl)
        # self.set_motor(self.FR_IN1, self.FR_IN2, self.FR_PWM, fr)
        # self.set_motor(self.RL_IN1, self.RL_IN2, self.RL_PWM, rl)
        # self.set_motor(self.RR_IN1, self.RR_IN2, self.RR_PWM, rr)

    def scale(self, value):
        return int(max(-MAX_SPEED, min(MAX_SPEED, value * MAX_SPEED)))

    # def setup_motor(self, in1, in2, pwm_pin):
    #     GPIO.setup(in1, GPIO.OUT)
    #     GPIO.setup(in2, GPIO.OUT)
    #     GPIO.setup(pwm_pin, GPIO.OUT)
    #     pwm = GPIO.PWM(pwm_pin, 100)
    #     pwm.start(0)

    # def set_motor(self, in1, in2, pwm_pin, speed):
    #     if speed > 0:
    #         GPIO.output(in1, GPIO.HIGH)
    #         GPIO.output(in2, GPIO.LOW)
    #     elif speed < 0:
    #         GPIO.output(in1, GPIO.LOW)
    #         GPIO.output(in2, GPIO.HIGH)
    #     else:
    #         GPIO.output(in1, GPIO.LOW)
    #         GPIO.output(in2, GPIO.LOW)

def main():
    rclpy.init()
    node = DriveNode()
    rclpy.spin(node)
    rclpy.shutdown()