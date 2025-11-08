import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import math as m
import os


class CovarianceCalculations(Node):

    def __init__(self):
        super().__init__('covariance_calculations')
        self.log = self.get_logger()

        # First bring in parametized values 
        self.declare_parameter('color', 'cyan')
        self.declare_parameter('landmark_name', 'landmark_5::link')
        self.declare_parameter('dx_camera', 0.076) # We got these values from the camera topic!!
        self.declare_parameter('dy_camera', 0.0)

        # Now get the parameters - standard
        self.color = self.get_parameter('color').get_parameter_value().string_value
        self.landmark_name = self.get_parameter('landmark_name').get_parameter_value().string_value
        self.dx_camera = self.get_parameter('dx_camera').get_parameter_value().double_value
        self.dy_camera = self.get_parameter('dy_camera').get_parameter_value().double_value

        # Now we create out subscriptions. Need the link states for the landmark, pose for the ground truth (lab 4), and the the 
        # distance/bearing calculations from A2.
        self.create_subscription(LinkStates, '/gazebo/link_states', self.linkstate_callback, 10)
        self.create_subscription(PoseStamped, '/tb3/ground_truth/pose', self.groundtruth_callback, 10)

        # Need to define topic name first
        # Subscribe separately to distance and bearing topics
        topic_distance = f'/vision_{self.color}/distance'
        topic_bearing  = f'/vision_{self.color}/bearing'

        self.create_subscription(Float32, topic_distance, self.distance_callback, 10)
        self.create_subscription(Float32, topic_bearing, self.bearing_callback, 10)

        # create publishers for plotting
        self.error_distance = self.create_publisher(Float32, f'/vision_{self.color}/error_characterization/distance_error', 10)
        self.error_bearing = self.create_publisher(Float32, f'/vision_{self.color}/error_characterization/bearing_error', 10)

        self.get_logger().info(f"Node started for landmark: {self.landmark_name}, color: {self.color}")
        # initialize for first run
        self.landmark_position = None
        self.gt_pose = None
        self.d_measured = None
        self.theta_measured = None

    def linkstate_callback(self, msg):
        # get right link index
        footprintIndex = msg.name.index(self.landmark_name) 
        pose_landmark = msg.pose[footprintIndex].position # only get the position of landmark, X, Y, Z when echo
        self.landmark_position = (pose_landmark.x, pose_landmark.y)

    def groundtruth_callback(self, msg):
        position = msg.pose.position
        q = msg.pose.orientation

        # from last lab:  # To extract this information, we utlized the formula in this link: https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion
        yaw = m.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        self.gt_pose = (position.x, position.y, yaw)

    def wrap_to_pi(self, angle):
        return (angle + m.pi) % (2 * m.pi) - m.pi

    def compute_error(self):
        # Check initalization - foudn through trial and error or else it crashes during the inital run
        if self.landmark_position is None or self.gt_pose is None: return
        if self.d_measured is None or self.theta_measured is None: return

        # Now we calculate
        # First unpack measurment data and all the other data extracted from subscriptions
        d_measured = self.d_measured
        theta_measured = self.theta_measured
        x_gt, y_gt, theta_gt = self.gt_pose
        x_l, y_l = self.landmark_position

        # First lets transform the pose from the robot frame to the camera frame --> p' = R*p + t
        # showed derivation in lab report
        x_camera = (self.dx_camera * m.cos(theta_gt)) - (self.dy_camera * m.sin(theta_gt)) + x_gt
        y_camera = (self.dx_camera * m.sin(theta_gt)) + (self.dy_camera * m.cos(theta_gt)) + y_gt

        # now lets find the true distance and bearing - have to consider the transform from world to camera
        # true distance -- > sqrt(dx_gt^2 + dy_gt^2) where dx_gt and dy_gt are the distance from gt camera to landmark
        dx_gt = x_l - x_camera
        dy_gt = y_l - y_camera
        d_truth = m.sqrt(dx_gt**2 + dy_gt**2)

        # Now we calculate the bearing error.
        theta_world = m.atan2(dy_gt, dx_gt)
        theta_truth = theta_world - theta_gt

        # Now get errors and publish them - we made the absolute just as a preference
        d_error = abs(d_measured - d_truth)
        theta_error = abs(theta_measured - theta_truth)

        # publish results
        dist_msg = Float32()
        dist_msg.data = d_error
        self.error_distance.publish(dist_msg)

        bearing_msg = Float32()
        bearing_msg.data = theta_error
        self.error_bearing.publish(bearing_msg)

        # Also log - for checking
        self.get_logger().info(f"d_measured={d_measured:.2f}, d_error={d_error:.2f}, theta_measured={theta_measured:.2f}, theta_err={theta_error:.2f}")
        
        # This part is for Assignment 4! --> to collect data.
        # Save each measurement to a text log file for A4 data collect
        if not os.path.exists('errors.csv'): # creates file
            with open('errors.csv', "w") as f:
                f.write("d_measured,d_error,theta_measured,theta_error\n")

        with open('errors.csv', "a") as f: # if it does exist just appends to - MAKE SURE TO DELETE FILE IF WE WANT NEW START !
            f.write(f"{d_measured:.3f},{d_error:.3f},{theta_measured:.3f},{theta_error:.3f}\n")


    def distance_callback(self, msg):
        self.d_measured = msg.data
        self.compute_error()

    def bearing_callback(self, msg):
        self.theta_measured = msg.data
        self.compute_error()



    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    covariance_calculations = CovarianceCalculations()
    covariance_calculations.spin()
    covariance_calculations.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
