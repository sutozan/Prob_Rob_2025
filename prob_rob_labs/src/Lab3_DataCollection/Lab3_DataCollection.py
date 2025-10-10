import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
import os
import csv


class Lab3_DataCollection(Node):

    def __init__(self):
        super().__init__('Lab3_DataCollection')

        # Set up subscribers and publishers
        self.door_pub = self.create_publisher(Float64, '/hinged_glass_door/torque', 10)
        self.create_subscription(Float64, '/feature_mean', self.feature_callback, 10)

        # Setup CSV logging
        self.file_path = os.path.expanduser(
            "~/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/misc/Lab3_BayesFilter/feature_data.csv"
        )
        file_exists = os.path.exists(self.file_path)
        self.csv_file = open(self.file_path, 'a', newline='')
        self.writer = csv.writer(self.csv_file)
        if not file_exists:
            self.writer.writerow(['feature_mean', 'door_state'])

        self.door_state = 'idle'
        self.log = self.get_logger()
        self.log.info("Starting data collection...")

        # Immediately open the door
        self.open_door()

    def close_door(self):
        self.door(-5.0)
        self.log.info("Closing door...")
        time.sleep(4)
        self.door(0.0)
        self.door_state = 'close'
        self.log.info("Recording closed-door data...")
        self.create_timer(15.0, self.finish_run)

    def open_door(self):
        self.door(5.0)
        self.log.info("Opening door...")
        time.sleep(4)
        self.door(0.0)
        self.door_state = 'open'
        self.log.info("Recording open-door data...")
        self.create_timer(15.0, self.close_door)

    def finish_run(self):
        self.door(0.0)
        self.file.close()
        self.log.info("Data Collection complete. File saved.")
        rclpy.shutdown()


    def spin(self):
        rclpy.spin(self)

    def door(self, torque):
        msg = Float64()
        msg.data = torque
        self.door_pub.publish(msg)

    def feature_callback(self, msg):
        self.feature_mean = msg.data

        # Write feature mean and state to file
        if self.door_state in ('open', 'close'):
            self.writer.writerow([msg.data, self.door_state])
            self.csv_file.flush()


def main():
    rclpy.init()
    data_collection = Lab3_DataCollection()
    data_collection.spin()
    data_collection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
