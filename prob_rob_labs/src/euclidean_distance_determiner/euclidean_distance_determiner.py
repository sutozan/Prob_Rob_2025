import rclpy
from rclpy.node import Node


heartbeat_period = 0.1

class EuclideanDistanceDeterminer(Node):

    def __init__(self):
        super().__init__('euclidean_distance_determiner')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    euclidean_distance_determiner = EuclideanDistanceDeterminer()
    euclidean_distance_determiner.spin()
    euclidean_distance_determiner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
