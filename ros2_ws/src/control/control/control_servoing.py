#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class PID:
    def __init__(self, kp, ki, kd, out_max, out_min):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_max, self.out_min = out_max, out_min
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, error, now):
        if self.last_time is None:
            dt = 0.0
        else:
            dt = now - self.last_time
        # Integral term
        self.integral += error * dt
        # Derivative term
        deriv = (error - self.prev_error) / dt if dt > 0 else 0.0
        # PID output
        out = self.kp * error + self.ki * self.integral + self.kd * deriv
        # Clamp output
        out = max(self.out_min, min(self.out_max, out))
        # Store for next iteration
        self.prev_error = error
        self.last_time = now
        return out

class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')
        self.bridge = CvBridge()
        self.width = 0
        self.height = 0
        self.center = (0,0)

        # Simulation time (will be set by callback)
        self.sim_time = None

        # Subscribers
        self.time_sub = self.create_subscription(
            Float32, '/simulation_time', self.time_cb, 10)
        self.image_sub = self.create_subscription(
            Image, '/PioneerP3DX/visionSensor', self.image_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID controllers (tuned for your robot)
        self.pid_ang = PID(kp=0.002, ki=0.0, kd=0.0005, out_max=1.0, out_min=-1.0)
        self.pid_lin = PID(kp=0.01,  ki=0.0, kd=0.005,  out_max=0.5, out_min=-0.5)

        self.get_logger().info('Esperando tiempo de simulación en /simulation_time...')

    def time_cb(self, msg: Float32):
        # Guardar tiempo de simulación (en segundos)
        if self.sim_time is None:
            self.get_logger().info('Tiempo de simulación recibido, iniciando visual servoing.')
        self.sim_time = msg.data

    def image_cb(self, msg: Image):
        # No procesar si aún no hemos recibido simulation_time
        if self.sim_time is None:
            return

        # Convertir a OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.width == 0:
            h, w = cv_img.shape[:2]
            self.width, self.height = w, h
            self.center = (w // 2, h // 2)

        # Detectar la esfera con HoughCircles
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, 1.0, minDist=100,
            param1=50, param2=30, minRadius=10, maxRadius=200
        )

        twist = Twist()
        if circles is not None:
            x, y, r = np.uint16(np.around(circles))[0][0]

            # Error angular: diferencia en X
            err_x = float(x - self.center[0])
            ang = self.pid_ang.compute(err_x, self.sim_time)

            # Error lineal: diferencia en tamaño del radio
            desired_r = 80.0
            err_r = desired_r - float(r)
            lin = self.pid_lin.compute(err_r, self.sim_time)

            twist.linear.x = lin
            twist.angular.z = -ang

        # Publicar comando de velocidad
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
