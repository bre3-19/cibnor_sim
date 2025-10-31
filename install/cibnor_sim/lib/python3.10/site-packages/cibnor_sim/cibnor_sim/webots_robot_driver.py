import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.02
WHEEL_RADIUS = 0.172

class RobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__wheel_FL = self.__robot.getDevice('motor_FL')
        self.__wheel_FR = self.__robot.getDevice('motor_FR')
        self.__wheel_BL = self.__robot.getDevice('motor_BL')
        self.__wheel_BR = self.__robot.getDevice('motor_BR')

        self.__wheel_FL.setPosition(float('inf'))
        self.__wheel_FL.setVelocity(0)

        self.__wheel_FR.setPosition(float('inf'))
        self.__wheel_FR.setVelocity(0)

        self.__wheel_BL.setPosition(float('inf'))
        self.__wheel_BL.setVelocity(0)

        self.__wheel_BR.setPosition(float('inf'))
        self.__wheel_BR.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

def __cmd_vel_callback(self, twist):
    self.__target_twist = twist

def step(self):
    rclpy.spin_once(self.__node, timeout_sec=0)

    forward_speed = self.__target_twist.linear.x
    angular_speed = self.__target_twist.angular.z

    left_speed = -(forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
    right_speed = -(forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

    self.__wheel_FL.setVelocity(left_speed)
    self.__wheel_BL.setVelocity(left_speed)
    self.__wheel_FR .setVelocity(right_speed)
    self.__wheel_BR.setVelocity(right_speed)