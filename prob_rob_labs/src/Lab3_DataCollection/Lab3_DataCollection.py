import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import os
import csv


class Lab3_DataCollection(Node):

    def __init__(self):
        super().__init__('Lab3_DataCollection')

        # Publisher and subscriber
        self.door_pub = self.create_publisher(Float64, '/hinged_glass_door/torque', 10)
        self.create_subscription(Float64, '/feature_mean', self.feature_callback, 10)

        # CSV setup
        self.file_path = os.path.expanduser(
            "~/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/misc/Lab3_BayesFilter/feature_data.csv"
        )
        file_exists = os.path.exists(self.file_path)
        self.csv_file = open(self.file_path, 'a', newline='')
        self.writer = csv.writer(self.csv_file)
        if not file_exists:
            self.writer.writerow(['feature_mean', 'door_state'])

        self.log = self.get_logger()
        self.log.info("Starting data collection")

        # Simple state variables
        self.counter = 0
        self.door_state = 'opening'
        self.samples_per_phase = 1200  # number of samples per phase - selected this through trial and error 
        # This part was key as it determined the feature mean behavior. Small values would result in a highly noisy output, although 
        # 1200 is most likely overkill.

        # Start by opening the door
        self.door(5.0) # Apply the open torque
        self.log.info("Opening door")

    def feature_callback(self, msg):
        feature = msg.data
        self.counter += 1

        # Open first
        if self.door_state == 'opening':
            if self.counter > 50:  # Wait time for door open
                self.door_state = 'open'
                self.counter = 0
                self.door(0.0)  # stop applying torque - safety measure
                self.log.info("Door opened. Collecting open-door data")

        # Now we collect the data
        elif self.door_state == 'open':
            self.writer.writerow([feature, 'open']) # Write to the file
            self.csv_file.flush() # Make sure everthing went through
            if self.counter >= self.samples_per_phase:
                self.counter = 0
                self.door_state = 'closing' # Switching state so that we can collect closed data now
                self.door(-5.0)

        # Start the door closing
        elif self.door_state == 'closing':
            if self.counter > 50: # Wait to fully close.
                self.door_state = 'close'
                self.counter = 0
                self.door(0.0) # Stop movement - safety measure
                self.log.info("Door closed. Collecting closed-door data")

        # Finally collect door data.
        elif self.door_state == 'close':
            self.writer.writerow([feature, 'close'])
            self.csv_file.flush()
            if self.counter >= self.samples_per_phase:
                self.log.info("Data collection complete.") # Signal to download file.
                self.done() # Close down node and file 

    def door(self, torque):
        msg = Float64()
        msg.data = torque
        self.door_pub.publish(msg)

    # For future reference, when writing to file if we want to close node properties rather than call main create a new function
    # to have that done - allows interaction between our actions and what we want in the background.
    def done(self): 
        self.door(0.0)
        self.csv_file.close()
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = Lab3_DataCollection()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
