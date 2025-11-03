import rclpy
from rclpy.node import Node


heartbeat_period = 0.1

class VisionGeometry(Node):

    def __init__(self):
        super().__init__('vision_geometry')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    vision_geometry = VisionGeometry()
    vision_geometry.spin()
    vision_geometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
