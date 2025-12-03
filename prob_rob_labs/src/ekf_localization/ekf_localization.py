import rclpy
from rclpy.node import Node
import json
from prob_rob_msgs.msg import Point2DArrayStamped
import os



class EkfLocalization(Node):

    def __init__(self):
        super().__init__('ekf_localization')
        self.log = self.get_logger()

        # Load the map via parameterization
        self.declare_parameter('map_path','~/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/config/landmarks.json')
        
        # Get the parameter now
        self.map_path = self.get_parameter('map_path').get_parameter_value().string_value

        # Now we read the map - json file for ours.
        self.map_path = os.path.expanduser(self.map_path) # safety to make sure file path is found on another devices.
        with open(self.map_path, "r") as f:
            self.map = json.load(f)

        self.log.info(f'Landmark map loaded: {self.map}')

        # Now we create the subscriptions. Since we want it to manually detect the color based on the map, 
        for color in ['red', 'green', 'yellow', 'magenta', 'cyan']:
            topic_name = f'/vision_{color}/corners'
            # create subscription
            # Note - intial test found that just setting the callback to color and msg meant that it would look for the return of that
            # This is WRONG. So we had to use lambda which operates as a new function that remembers color - so it allows color to go through once
            # msg arrives.
            self.create_subscription(Point2DArrayStamped, topic_name, lambda msg, color=color: self.landmark_cb(msg, color), 10)
            self.log.info(f'Subscribing to topic: {topic_name}')

    def landmark_cb(self, msg, color):
        # Now get the correct landmark and do a sanity check of the values
        landmark = self.map[color]
        x = landmark["x"]
        y = landmark["y"]

        # Log in serial
        self.log.info(f'Landmark properties: {color}, x_pos: {x}, y_pos: {y}')


    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    ekf_localization = EkfLocalization()
    ekf_localization.spin()
    ekf_localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
