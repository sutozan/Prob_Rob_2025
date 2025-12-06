import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np
import math as m

class MapOdom(Node):

    def __init__(self):
        super().__init__('map_odom')
        self.log = self.get_logger()
        
        # First we create subscirbers
        self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.ekf_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Now we need publisher to tf topic
        # Aparently easier to do with broadcaster --> https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_timer(1.0/30.0, self.tf)

        # Like last node we also have to initalize here
        self.T_map_odom = np.eye(3)  
        # Intialize tracking here for covariance, ekf, and odom        
        self.last_cov = None                  
        self.last_ekf_pose = None            
        self.last_odom_pose = None 

    # A helper function to explain pose as a matrix so we can use transforms
    # Used equation here: https://en.wikipedia.org/wiki/Transformation_matrix
    # Rotation + translation --> recall
    # made it a helper since we use it alot below.
    def pose_to_matrix(self, x, y, yaw):
        return np.array([
            [m.cos(yaw), -m.sin(yaw), x],
            [m.sin(yaw),  m.cos(yaw), y],
            [0.0, 0.0, 1.0]
        ])

    def ekf_cb(self, msg):

        if self.last_odom_pose is None: # check
            return

        # Extract EKF estimated pose 
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Next yaw - # To extract this information, we utlized the formula in this link: https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion
        q = msg.pose.pose.orientation
        yaw = m.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.last_ekf_pose = (x, y, yaw) #redefine from the msg

        x_odom, y_odom, yaw_odom = self.last_odom_pose # bring in odom

        T_odom_base = self.pose_to_matrix(x_odom, y_odom, yaw_odom)
        T_map_base = self.pose_to_matrix(x, y, yaw)

        self.T_map_odom = T_map_base @ np.linalg.inv(T_odom_base) #T = T*T^-1 - transform formula - undo transfomation and return to frame
        # map to baselink multipled by the inverse from odom to base link will leave us with map to odom

    def odom_cb(self, msg): # repeat a similar process for odom so that we can get the odom frame

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = m.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.last_odom_pose = (x, y, yaw)

    # and now we publish calculation
    def tf(self):

        if self.last_ekf_pose is None or self.last_odom_pose is None: return
        self.log.info("Publishing map->odom") # check frequency

        t = TransformStamped() # define mesage
        t.header.stamp = self.get_clock().now().to_msg() # message time
        t.header.frame_id = 'map' # header
        t.child_frame_id = 'odom' # to child --> since we are transforming.

        # Extract translation
        tx = self.T_map_odom[0, 2]
        ty = self.T_map_odom[1, 2]

        # Extract yaw in same style as your EKF code 
        sin_yaw = self.T_map_odom[1, 0]
        cos_yaw = self.T_map_odom[0, 0]
        yaw = m.atan2(sin_yaw, cos_yaw) # get yaw

        # Convert yaw to quaternion so that it fits in our format - equations from link above
        qz = m.sin(yaw / 2.0)
        qw = m.cos(yaw / 2.0)

        # now fill in message using our calculated values above.
        # translation values
        t.transform.translation.x = tx # x values
        t.transform.translation.y = ty # y values
        t.transform.translation.z = 0.0 # dont conder z

        # rotation values.
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        # publish via broadcast

        self.tf_broadcaster.sendTransform(t)




    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    map_odom = MapOdom()
    map_odom.spin()
    map_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
