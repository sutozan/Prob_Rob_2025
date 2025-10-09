import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time
import os
import csv

heartbeat_period = 0.1

class Lab3_DataCollection(Node):

    def __init__(self):
        super().__init__('Lab3_DataCollection')

        # Set up subscribers and publishers
        self.door_pub = self.create_publisher(Float64, '/hinged_glass_door/torque', 10)
        self.create_subscription(Float64,'/feature_mean', self.feature_callback, 10)

        # Set up logging (default to file)
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        # Setup file saving
        # Save inside the prob_rob_labs misc folder
        self.file_path = os.path.expanduser("~/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/misc/Lab3_BayesFilter/feature_data.csv")
        self.csv_file = open(self.file_path, 'a', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['feature_mean', 'door_state'])

        # Initialize data collection process
        self.feature_mean = 0.0
        self.state = 'open_door'
        self.counter = 0
        self.log.info("Start logging door features")

    def heartbeat(self):
        if self.state == 'open_door':
            self.door(5.0)
            self.counter += 1
            if self.counter == 60:
                self.door(0.0)
                self.state = 'record_open'
                self.counter = 0
                self.log.info('Door open - recording feature mean for open state')
        elif self.state == 'record_open':
            self.writer.writerow([self.feature_mean, 'open'])
            self.counter += 1
            if self.counter == 50:
                self.state = 'close_door'
                self.counter = 0
                self.log.info('Done recording open - now closing the door')
        elif self.state == 'close_door':
            self.door(-5.0)
            self.counter += 1
            if self.counter == 60:
                self.door(0.0)
                self.state = 'record_close'
                self.counter = 0
                self.log.info("Door closed - recording feature mean for closed state")
        elif self.state == 'record_close':
            self.writer.writerow([self.feature_mean, 'close'])
            self.counter += 1
            if self.counter == 50:
                self.log.info("Logging complete and Data is saved")
                self.file.close()
                rclpy.shutdown()

    def spin(self):
        rclpy.spin(self)

    def door(self, torque):
        msg = Float64()
        msg.data = torque
        self.door_pub.publish(msg)

    def feature_callback(self, msg):
        self.feature_mean = msg.data


def main():
    rclpy.init()
    data_collection = Lab3_DataCollection()
    data_collection.spin()
    data_collection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
