import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time
import os
import csv

heartbeat_period = 0.1

class DoorOpener(Node):

    def __init__(self):
        super().__init__('door_opener')
        self.door_ = self.create_publisher(Float64, '/hinged_glass_door/torque', 10)
        self.velocity_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.state = 'init'
        self.counter = 0
        self.create_subscription(Float64,'/feature_mean', self.feature_callback, 10)

        # Assignment 4: Adding a Node Parameter
        self.declare_parameter('forward_speed', 0.2)

        # Lab 3, Assignment 4: Calling in threshold
        self.declare_parameter('threshold', 280)
        self.threshold = self.get_parameter('threshold').value
        self.state = 'init'

        # adding log files
        log_path = '/run/host/workdir/local_stt2126/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/misc/feature_mean.csv'
        file_exists = os.path.exists(log_path)
        self.log_file = open(log_path, 'a', newline='')
        self.csv_writer = csv.writer(self.log_file)
        if not file_exists:
            self.csv_writer.writerow(['timesec', 'feature_mean'])
        self.start_time = time.time()
        self.log.info(f"Logging data to {log_path}")


    def heartbeat(self):
        if self.state == 'init':
            if self.feature_mean > self.threshold: # open the door(over the threshold)
                self.door(5.0)
                self.log.info('opening the door...')

            if self.feature_mean < self.threshold:
                self.state='move'
        elif self.state == 'move': 
            #self.move(2.0)
            forward_speed = self.get_parameter('forward_speed').value
            # self.move(forward_speed)
            self.counter += 1
            self.log.info(f'moving the robot at a speed of {forward_speed}...')
            if self.counter == 25:
                self.state ='close'
                self.counter = 0
        elif self.state == 'close': # stop the robot(under the threshold)
            self.move(0.0)
            self.door(-5.0)
            self.log.info('stopped moving the robot...')
            if self.feature_mean > self.threshold:
                # self.state = "finished"
                # self.log.info('door closed...')
                # self.log_file.flush()
                # self.log_file.close()
                self.counter += 1
                if self.counter == 15:
                    self.state = "finished"
                    self.log.info('door closed...')
                    self.log_file.flush()
                    self.log_file.close()
                    self.counter = 0

    def spin(self):
        rclpy.spin(self)

    def door(self, torque):
        msg = Float64()
        msg.data = torque
        self.door_.publish(msg)

    def move(self, speed):
        msg = Twist()
        msg.linear.x = speed
        self.velocity_.publish(msg)

    def feature_callback(self, msg):
        self.feature_mean = msg.data

        t = time.time() - self.start_time
        self.csv_writer.writerow([f"{t:.3f}", f"{msg.data:.3f}", self.state])
        self.log_file.flush()
        self.log.info(f"feature_mean={msg.data:.3f}")


def main():
    rclpy.init()
    door_opener = DoorOpener()
    door_opener.spin()
    door_opener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
