import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription = self.create_subscription(
            Image,
            '/PioneerP3DX/visionSensor',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Crear ventana OpenCV con nombre claro
        self.window_name = "/PioneerP3DX/visionSensor"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info(f'🚀 Subscrito a: {self.window_name}')

    def image_callback(self, msg):
        try:
            # Convertir imagen ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Mostrar la imagen en la ventana con nombre del tópico
            cv2.imshow(self.window_name, cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"❌ Error procesando imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
