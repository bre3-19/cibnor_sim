import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

HALF_DISTANCE_BETWEEN_WHEELS = 0.25
WHEEL_RADIUS = 0.175

class RobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        #Motors
        self.__wheel_FL = self.__robot.getDevice('motor_FL')
        self.__wheel_FR = self.__robot.getDevice('motor_FR')
        self.__wheel_BL = self.__robot.getDevice('motor_BL')
        self.__wheel_BR = self.__robot.getDevice('motor_BR')

        self.__wheel_FL.setPosition(float('inf'))
        self.__wheel_FL.setVelocity(0)
        self.__wheel_BL.setPosition(float('inf'))
        self.__wheel_BL.setVelocity(0)
        self.__wheel_FR.setPosition(float('inf'))
        self.__wheel_FR.setVelocity(0)
        self.__wheel_BR.setPosition(float('inf'))
        self.__wheel_BR.setVelocity(0)

        self.__target_twist = Twist()

        #Encoders
        self.__encoder_FL = self.__robot.getDevice('encoder_FL')
        self.__encoder_FR = self.__robot.getDevice('encoder_FR')
        self.__encoder_BL = self.__robot.getDevice('encoder_BL')
        self.__encoder_BR = self.__robot.getDevice('encoder_BR')

        self.__encoder_FL.enable(32)
        self.__encoder_FR.enable(32)
        self.__encoder_BL.enable(32)
        self.__encoder_BR.enable(32)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__joint_state_publisher = self.__node.create_publisher(JointState, 'wheels_encoders', 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Obtiene las velocidades lineal y angular del cmd_vel
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        # Calcula velocidades de cada lado del robot diferencial
        left_speed  = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        right_speed = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        # Aplica las velocidades a los motores
        # Invertimos solo el lado derecho si tus ruedas están orientadas al revés
        self.__wheel_FL.setVelocity(left_speed)
        self.__wheel_BL.setVelocity(left_speed)
        self.__wheel_FR.setVelocity(-right_speed)
        self.__wheel_BR.setVelocity(-right_speed)

        #Encoders
        joint_states_header = Header()
        joint_states_header.stamp = self.__node.get_clock().now().to_msg()
        joint_states_header.frame_id = ''

        joint_states = JointState()
        joint_states.header = joint_states_header
        joint_states.name = ['motor_FL', 'motor_BL', 'motor_FR', 'motor_BR']
        joint_states.position = [
            float(self.__encoder_FL.getValue()),
            float(self.__encoder_BL.getValue()),
            float(self.__encoder_FR.getValue()),
            float(self.__encoder_BR.getValue())
        ]

        self.__joint_state_publisher.publish(joint_states)