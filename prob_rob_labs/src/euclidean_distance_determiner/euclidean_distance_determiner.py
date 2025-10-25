import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math



class EuclideanDistanceDeterminer(Node):

    def __init__(self):
        super().__init__('euclidean_distance_determiner')
        self.log = self.get_logger()

        # Define the subscriptions and positions
        self.create_subscription(PoseStamped, '/tb3/ground_truth/pose', self.groundtruth_callback, 10) # Take the ground truth
        self.create_subscription(Odometry, '/ekf_odom', self.ekf_callback, 10) # Also from odometry
        self.pub_pos = self.create_publisher(Float64, '/error_pos', 10) # Creating the publishers
        self.pub_ang = self.create_publisher(Float64, '/error_ang', 10)

        # Found that we needed this in intial runs when A1 node is not running to initalize - else code will fail.
        self.gt_pose = None

    def groundtruth_callback(self, msg):
        # Take in pose data from callback
        self.gt_pose = msg.pose

    def ekf_callback(self, msg):
        # Inital run showed that we need this when the A1 node is not running
        if self.gt_pose is None:
            return
    
        # Extract EKF estimated pose
        xe, ye = msg.pose.pose.position.x, msg.pose.pose.position.y
        qe = msg.pose.pose.orientation

        # To extract this information, we utlized the formula in this link: https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion
        yaw_e = math.atan2(2.0 * (qe.w * qe.z + qe.x * qe.y),
                           1.0 - 2.0 * (qe.y ** 2 + qe.z ** 2))

        # Extract Ground Truth pose the same way we did above.
        xg, yg = self.gt_pose.position.x, self.gt_pose.position.y
        qg = self.gt_pose.orientation
        yaw_g = math.atan2(2.0 * (qg.w * qg.z + qg.x * qg.y),
                           1.0 - 2.0 * (qg.y ** 2 + qg.z ** 2))

        # Compute Errors - formulas for this were taken from the lab manual.
        pos_err = math.sqrt((xg - xe) ** 2 + (yg - ye) ** 2)
        ang_err = yaw_g - yaw_e

        # wrap angle to [-pi, pi] --> 
        if ang_err > math.pi:
            ang_err -= 2 * math.pi
        elif ang_err < -math.pi:
            ang_err += 2 * math.pi

        # Publish
        self.pub_pos.publish(Float64(data=pos_err))
        self.pub_ang.publish(Float64(data=ang_err))

        # Log it out.
        self.log.info(f"pos_err={pos_err:.4f}  ang_err={math.degrees(ang_err):.2f}°")


    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    euclidean_distance_determiner = EuclideanDistanceDeterminer()
    euclidean_distance_determiner.spin()
    euclidean_distance_determiner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
