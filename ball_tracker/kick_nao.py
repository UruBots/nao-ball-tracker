import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed

class KickNAO(Node):
    def __init__(self):
        super().__init__('ball_tracker')

        self.declare_parameter('ball_color', 'red')  
        self.declare_parameter('image_topic', '/camera/bottom/image_raw')  

        self.ball_color = self.get_parameter('ball_color').value
        self.image_topic = self.get_parameter('image_topic').value

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointAnglesWithSpeed, '/joint_angles', 10)

        self.bridge = CvBridge()
        
        self.get_logger().info(f'Tracking {self.ball_color} ball on topic {self.image_topic}')

    def move_arm(self, angles, speed=0.2):
        """Mueve el brazo del NAO con los ángulos dados."""
        arm_cmd = JointAnglesWithSpeed()
        arm_cmd.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
        arm_cmd.joint_angles = angles
        arm_cmd.speed = speed
        arm_cmd.relative = False
        self.joint_pub.publish(arm_cmd)
        self.get_logger().info('Moviendo brazo')

    def control_hand(self, open_hand=True):
        """Abre o cierra la mano izquierda."""
        hand_cmd = JointAnglesWithSpeed()
        hand_cmd.joint_names = ['LHand']
        hand_cmd.joint_angles = [1.0 if open_hand else 0.0]  # 1.0 = abrir, 0.0 = cerrar
        hand_cmd.speed = 0.2
        hand_cmd.relative = False
        self.joint_pub.publish(hand_cmd)
        self.get_logger().info('Abriendo mano' if open_hand else 'Cerrando mano')

    def grab_and_throw(self):
        """Acción de agarrar y lanzar la pelota."""
        self.get_logger().info('Intentando agarrar la pelota...')
        
        # Extiende el brazo para agarrar
        self.move_arm([0.5, 0.2, -1.5, -0.5, 0.0])  
        rclpy.spin_once(self, timeout_sec=2.0)  

        # Cierra la mano para agarrar
        self.control_hand(open_hand=False)
        rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info('Pelota agarrada!')

        # Levanta el brazo para lanzar
        self.move_arm([-1.0, 0.0, 0.0, -1.0, 0.5])
        rclpy.spin_once(self, timeout_sec=2.0)

        # Abre la mano para soltar la pelota
        self.control_hand(open_hand=True)
        self.get_logger().info('Pelota lanzada!')

        # Baja el brazo a posición inicial
        self.move_arm([1.4, 0.0, 0.0, -0.5, 0.0])
        rclpy.spin_once(self, timeout_sec=2.0)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = frame.shape

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        color_ranges = {
            'red': ([0, 120, 70], [10, 255, 255]),
            'blue': ([100, 150, 0], [140, 255, 255]),
            'green': ([40, 70, 70], [80, 255, 255])
        }

        if self.ball_color in color_ranges:
            lower, upper = color_ranges[self.ball_color]
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        else:
            self.get_logger().warn(f"Color '{self.ball_color}' no reconocido.")
            return

        mask = cv2.medianBlur(mask, 5)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        move_cmd = Twist()

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)

            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

                center_x = width // 2

                if x < center_x - 50:
                    move_cmd.angular.z = 0.3
                elif x > center_x + 50:
                    move_cmd.angular.z = -0.3
                else:
                    move_cmd.angular.z = 0.0
                    move_cmd.linear.x = 0.2  

                if radius > 100:  
                    self.get_logger().info('Pelota alcanzada! Preparando para agarrar...')
                    move_cmd.linear.x = 0.0
                    self.cmd_pub.publish(move_cmd)
                    self.grab_and_throw()
                    return  

        self.get_logger().info(f"Movimiento: linear.x={move_cmd.linear.x}, angular.z={move_cmd.angular.z}")
        self.cmd_pub.publish(move_cmd)

        cv2.imshow("Ball Tracker", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = KickNAO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()