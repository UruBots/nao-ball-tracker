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

        self.declare_parameter('ball_color', 'red')  # Default color is red
        self.declare_parameter('image_topic', '/camera/bottom/image_raw')  # Default topic | /image_raw simulado

        # Get parameter values
        self.ball_color = self.get_parameter('ball_color').value
        self.image_topic = self.get_parameter('image_topic').value

        # Suscribirse a la cámara
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.subscription  # Evitar advertencias de variables sin usar

        # Publicador de comandos de movimiento
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointAnglesWithSpeed, '/joint_angles', 10)

        # Herramienta para convertir imágenes de ROS2 a OpenCV
        self.bridge = CvBridge()
        
        # Head default position
        self.default_head_pitch = 0.0  
        self.tracking_head_pitch = 0.3  
        
        self.get_logger().info(f'Tracking {self.ball_color} ball on topic {self.image_topic}')

    def move_head(self, pitch_angle):
        """Moves NAO's head to track the ball."""
        head_cmd = JointAnglesWithSpeed()
        head_cmd.joint_names = ['HeadPitch']
        head_cmd.joint_angles = [pitch_angle]  
        head_cmd.speed = 0.1
        head_cmd.relative = False
        self.joint_pub.publish(head_cmd)
        self.get_logger().info(f"Moving head to pitch: {pitch_angle}")

    def kick(self):
        """Executes a kick motion with the left leg."""
        self.get_logger().info("Preparing to kick...")

        # Step 1: Shift weight to the right leg
        shift_weight = JointAnglesWithSpeed()
        shift_weight.joint_names = ['LHipRoll', 'RHipRoll']
        shift_weight.joint_angles = [0.4, -0.4]  # Shift body to the right leg
        shift_weight.speed = 0.2
        shift_weight.relative = False
        self.joint_pub.publish(shift_weight)
        rclpy.spin_once(self, timeout_sec=1.0)

        # Step 2: Lift the left leg (pre-kick position)
        pre_kick_cmd = JointAnglesWithSpeed()
        pre_kick_cmd.joint_names = ['LHipPitch', 'LKneePitch', 'LAnklePitch']
        pre_kick_cmd.joint_angles = [0.2, 0.5, -0.3]  # Lift the leg slightly
        pre_kick_cmd.speed = 0.2
        pre_kick_cmd.relative = False
        self.joint_pub.publish(pre_kick_cmd)
        rclpy.spin_once(self, timeout_sec=1.0)

        # Step 3: Perform the kick (extend the knee)
        kick_cmd = JointAnglesWithSpeed()
        kick_cmd.joint_names = ['LKneePitch', 'LAnklePitch']
        kick_cmd.joint_angles = [1.0, 0.2]  # Extend knee to kick
        kick_cmd.speed = 0.3
        kick_cmd.relative = False
        self.joint_pub.publish(kick_cmd)
        self.get_logger().info("Executing kick motion")
        rclpy.spin_once(self, timeout_sec=1.5)

        # Step 4: Return leg to normal position
        reset_cmd = JointAnglesWithSpeed()
        reset_cmd.joint_names = ['LHipPitch', 'LKneePitch', 'LAnklePitch', 'LHipRoll', 'RHipRoll']
        reset_cmd.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0]  # Reset leg & weight
        reset_cmd.speed = 0.2
        reset_cmd.relative = False
        self.joint_pub.publish(reset_cmd)
        self.get_logger().info("Leg returned to normal position")
        rclpy.spin_once(self, timeout_sec=1.0)  


    def image_callback(self, msg):
        # Convertir imagen de ROS2 a OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = frame.shape

        # Convertir a espacio de color HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Definir rango de color para la pelota (ejemplo: pelota roja)
        color_ranges = {
            'red': ([0, 120, 70], [10, 255, 255]),  # Lower and upper HSV bounds for red
            'blue': ([100, 150, 0], [140, 255, 255]),  # Blue
            'green': ([40, 70, 70], [80, 255, 255]),  # Green
            'orange': ([10, 100, 20], [25, 255, 255])  # Orange range
        }

        if self.ball_color in color_ranges:
            lower, upper = color_ranges[self.ball_color]
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        else:
            self.get_logger().warn(f"Color '{self.ball_color}' is not recognized.")
            return

        # Crear una máscara
        mask = cv2.medianBlur(mask, 5)

        # Encontrar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        move_cmd = Twist()

        # Si hay al menos un contorno
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)

            if radius > 10:  # Filtrar detecciones pequeñas
                # Dibujar círculo en la imagen
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

                # Enviar comandos de movimiento al robot
                move_cmd = Twist()
                center_x = width // 2
                center_y = height // 2  # Middle of the camera view

                if x < center_x - 50:  # La pelota está a la izquierda
                    move_cmd.angular.z = 0.3
                elif x > center_x + 50:  # La pelota está a la derecha
                    move_cmd.angular.z = -0.3
                else:  # Centrado
                    move_cmd.angular.z = 0.0
                    move_cmd.linear.x = 0.2  # Avanzar hacia la pelota

                if radius < 50:
                    move_cmd.linear.x = 0.1  # Move forward
                elif radius > 100:
                    self.get_logger().info('Kicking the ball!')
                    move_cmd.linear.x = 0.0
                    self.cmd_pub.publish(move_cmd)
                    self.kick()
                    #rclpy.shutdown()

                        # Adjust head tracking based on ball position
                if y < center_y * 0.7:  # Ball is above the middle (look down)
                    self.move_head(self.tracking_head_pitch)
                else:  # Ball is at or below center (reset head)
                    self.move_head(self.default_head_pitch)

        else:
            self.move_head(self.default_head_pitch)  # No ball detected → Reset head

        # Publicar el comando de movimiento
        self.get_logger().info(f"Publishing move_cmd: linear.x={move_cmd.linear.x}, angular.z={move_cmd.angular.z}")
        self.cmd_pub.publish(move_cmd)

        # Mostrar imagen en una ventana
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