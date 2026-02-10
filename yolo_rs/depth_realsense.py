import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DepthFromPixel(Node):
    def __init__(self):
        super().__init__('depth_from_pixel')

        self.bridge = CvBridge()

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        # 테스트할 픽셀 좌표
        self.pixel_x = 320
        self.pixel_y = 240

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

        depth_mm = depth_image[self.pixel_y, self.pixel_x]

        if depth_mm == 0:
            self.get_logger().warn('No depth data at this pixel')
            return

        depth_m = depth_mm / 1000.0
        self.get_logger().info(
            f'Pixel ({self.pixel_x}, {self.pixel_y}) → Z = {depth_m:.3f} m'
        )

def main():
    rclpy.init()
    node = DepthFromPixel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

