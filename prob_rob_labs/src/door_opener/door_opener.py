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

        # Creating a subscription to feature_mean
        self.create_subscription(Float64,'/feature_mean', self.feature_callback, 10)

        # Assignment 4: Adding a Node Parameter
        self.declare_parameter('forward_speed', 0.2)

        # Lab 3, Assignment 4: Calling in threshold
        self.declare_parameter('threshold', 280) # Default
        self.threshold = self.get_parameter('threshold').value
        self.state = 'init'
        self.counter = 0
        self.feature_mean = 0.0

        # Intialization for Bayesian Inference
        self.belief_open = 0.5
        self.belief_threshold = 0.999

        # Bring in conditional probabilities from A5
        self.P_open_open = 0.914 # P(z=open | x = open)
        self.P_close_open = 0.0856 # P(z=close | x = open)
        self.P_open_close = 0.0623 # P(z=open | x = close)
        self.P_close_close = 0.9377 # P(z=close | x = close)

        # Setup file saving to save the measurment and belief calculations
        # Save inside the prob_rob_labs misc folder
        self.file_path = os.path.expanduser("~/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/misc/Lab3_BayesFilter/A6-BeliefCalculations.csv")
        self.csv_file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['Measurment', 'Belief (open)'])


    def heartbeat(self):
        if self.state == 'init':
            # Now we add the Bayesian Inference here
            # Take a measurment
            if self.feature_mean < self.threshold:
                z = "open"
                P_z_open =  self.P_open_open
                P_z_close = self.P_open_close
            else:
                z = "close"
                P_z_open =  self.P_close_open
                P_z_close = self.P_close_close

            # Update the belief using formula from lecture
            self.belief_open = ((P_z_open * self.belief_open) / 
                                ((P_z_open * self.belief_open)+(P_z_close*(1-self.belief_open))))
            
            # Print the measurment and the updates belief
            self.log.info(f"Measurement: {z}, Belief (open) = {self.belief_open:0.5f}")
            self.writer.writerow([z, self.belief_open])
            
            # Here is were we start our control loop.
            self.door(5.0)
            self.log.info(f'Belief(door open) = {self.belief_open:.5f}')
            if self.belief_open > self.belief_threshold:
                self.state='open'
                self.log.info('Door confirmed to be open --> Moving robot forward')

        elif self.state == 'open': 
            forward_speed = self.get_parameter('forward_speed').value
            self.move(forward_speed)
            self.counter += 1
            self.log.info(f'Moving the robot at a speed of {forward_speed} --> Belief(door open) = {self.belief_open:.5f}')
            if self.counter >= 25:
                self.state ='close'
                self.counter = 0
                self.log.info('Reached target, closing door now')
        elif self.state == 'close': 
            self.move(0.0)
            self.log.info('Stopped moving the robot')
            self.door(-5.0)
            self.counter += 1
            if self.counter >= 10:
                self.state = "finished"
                self.log.info('Door closed.')
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


        



def main():
    rclpy.init()
    door_opener = DoorOpener()
    door_opener.spin()
    door_opener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
