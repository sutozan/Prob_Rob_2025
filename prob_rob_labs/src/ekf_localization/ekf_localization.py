import rclpy
from rclpy.node import Node
import json
from prob_rob_msgs.msg import Point2DArrayStamped
import os
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32
import math as m


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

        # Now we create the subscriptions. Since we want it to manually detect the color based on the map, we create a list and iterate through it 
        # Get from topic -> to color
        #self.map.keys() = ['red', 'green', 'yellow', 'magenta', 'cyan']
        for color in self.map.keys():
            topic_name = f'/vision_{color}/corners'
            # create subscription
            # Note - intial test found that just setting the callback to color and msg meant that it would look for the return of that
            # This is WRONG. So we had to use lambda which operates as a new function that remembers color - so it allows color to go through once
            # msg arrives.
            # Confirmed usage with lab manual hint.
            self.create_subscription(Point2DArrayStamped, topic_name, lambda msg, color=color: self.landmark_cb(msg, color), 10)
            #self.log.info(f'Subscribing to topic: {topic_name}')

        # Assignment 2: Now we bring in the measurment values/calculations from the vision processing node

        #Define parameters
        self.declare_parameter('height', 0.5)
        self.height = self.get_parameter('height').get_parameter_value().double_value
        
        # create subscriptions for camera properties
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_callback, 10)
        
        # create publishers for distance and bearing - compared to last lab we need to consider the possible colors. so from topic to colors
        self.pub_distance = {}
        self.pub_bearing = {}
        for color in self.map.keys():
            topic_name_2 = f'/vision_{color}/distance'
            topic_name_3 = f'/vision_{color}/bearing'
            self.pub_distance[color] = self.create_publisher(Float32, topic_name_2, 10)
            self.pub_bearing[color] = self.create_publisher(Float32, topic_name_3 , 10)
            # Comment out to reduce clutter in terminal
            #self.log.info(f'Publishing distance on: {topic_name_2}')
            #self.log.info(f'Publishing bearing on: {topic_name_3}')

        # Trial and Error: Initalize variables
        self.fx = self.fy = self.cx = self.cy = None

        # Variance model constants from previous lab
        self.b_d = -0.0170
        self.b_b = 0.0034
        self.a_d = 0.0013331
        self.a_t = 0.035134

    
    def camera_callback(self, msg):
        # Call in projection model
        P = msg.p

        # From model in manual: [[fx, 0 , cx], [0, fy, cy], [0, 0, 1]]^T
        # However, when we echo topic we see that P is 3x4 --> has extra values Tx, Ty. Adjust accordingly
        self.fx = P[0]
        self.fy = P[5]
        self.cx = P[2]
        self.cy = P[6]
        

    def landmark_cb(self, msg, color):
        # Now get the correct landmark and do a sanity check of the values
        landmark = self.map[color]
        x = landmark["x"]
        y = landmark["y"]

        # Log in serial - commented out for A2 to reduce clutter
        #self.log.info(f'Landmark properties: {color}, x_pos: {x}, y_pos: {y}')

        if self.fx is None or self.cx is None:
            return
        
        # From here on is the same code as the vision_geometry code.
        points = msg.points

        # Have to check for points - or else crashes - also for A2 this is needed so the no points doesnt register an error
        if not points: return
        
        # define the x and y values
        x_values = [p.x for p in points]
        y_values = [p.y for p in points]
        
        # calculating height and veritical axis symmetry position
        dy = max(y_values) - min(y_values)

        # ## Extension for Dropping the Offending Measurment ##
        # realized we were getting 0 height sometime and this causes an issue within the reading.
        # need 4 points and positive height --> so we adjust for these.
        if len(points) < 2: return
        if dy <= 0: return

        vertical_x = (min(x_values)+max(x_values))/2
        #self.log.info(f"Height: {dy:.3f}, Position of Vertical Symmetry Axis: {vertical_x:.3f}")

        # Formulas frm lab manual.
        # calculate theta
        theta = m.atan((self.cx - vertical_x) / self.fx)
        # calculate distance
        d = self.height * (self.fy/(dy*m.cos(theta)))

        # publish results
        dist_msg = Float32()
        dist_msg.data = d
        self.pub_distance[color].publish(dist_msg)

        bearing_msg = Float32()
        bearing_msg.data = theta
        self.pub_bearing[color].publish(bearing_msg)

        # log info
        sigma_d2 = self.b_d + self.a_d * (d ** 2)
        sigma_th2 = self.b_b + self.a_t * (theta ** 2)


        # Main log we care about it!
        self.log.info(f"Color = {color}, Distance={d:.3f} m, Bearing={theta:.3f} rad, sigma_d²={sigma_d2:.5f}, sigma_b²={sigma_th2:.5f}")

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
