import rclpy
from rclpy.node import Node


heartbeat_period = 0.1

class Odometry_Tracking(Node):

    def __init__(self):
        super().__init__('Odometry_Tracking')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    odometry_tracking = Odometry_Tracking()
    odometry_tracking.spin()
    odometry_tracking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
