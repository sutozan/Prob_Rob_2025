import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import math as m


class CovarianceCalculations(Node):

    def __init__(self):
        super().__init__('covariance_calculations')
        self.log = self.get_logger()

        # First bring in parametized values
        self.declare_parameter('color', 'cyan')
        self.declare_parameter('landmark_name', 'landmark_5::link')
        self.declare_parameter('dx_camera', 0.076)
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
        topic_name = f'/vision_{self.color}/distance_bearing'
        self.create_subscription(Float32MultiArray, topic_name, self.measure_callback, 10)

        # create publisher for plotting
        self.error = self.create_publisher(Float32MultiArray, f'/vision_{self.color}/error_characterization', 10)

        self.get_logger().info(f"Node started for landmark: {self.landmark_name}, color: {self.color}")

        # initialize for first run
        self.landmark_position = None
        self.gt_pose = None

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

    def measure_callback(self, msg):
        # Check initalization - foudn through trial and error or else it crashes during the inital run
        if self.landmark_position is None or self.gt_pose is None: return

        # Now we calculate
        # First unpack measurment data and all the other data extracted from subscriptions
        d_measured, theta_measured = msg.data
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

        # now bearing. We need to also account for the angle of the robot
        camera_offset = 1.57
        theta_world = m.atan2(dy_gt, dx_gt) 
        theta_truth = theta_world - (theta_gt + camera_offset) # now transform from world to camera. Subtracting theta_gt means we
        # are removing the global postion of robot.

        # Now get errors and publish them
        d_error = d_measured - d_truth
        theta_error = theta_measured - theta_truth

        e_msg = Float32MultiArray()
        e_msg.data = [d_measured, d_error, theta_measured, theta_error]
        self.error.publish(e_msg)

        # Also log
        self.get_logger().info(f"d_measured={d_measured:.2f}, d_error={d_error:.2f}, theta_measured={theta_measured:.2f}, θ_err={theta_error:.2f}")


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
