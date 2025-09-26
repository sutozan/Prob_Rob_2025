import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time


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

    def heartbeat(self):
        self.log.info('heartbeat')
        if self.state =='init': # open the door
            self.door(5.0)
            self.counter += 1
            self.log.info('opening the door...')
            if self.counter == 20:
                self.state = "move"
                self.counter = 0
        elif self.state == "move": # move the robot
            self.move(2.0)
            self.counter += 1
            self.log.info('moving the robot...')
            if self.counter == 50:
                self.state = "close"
                self.counter = 0
        elif self.state == "close": # stop the robot
            self.move(0.0)
            self.door(-5.0)
            self.counter += 1
            self.log.info('moving the robot...')
            if self.counter == 10:
                self.state = "finished"
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



def main():
    rclpy.init()
    door_opener = DoorOpener()
    door_opener.spin()
    door_opener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
