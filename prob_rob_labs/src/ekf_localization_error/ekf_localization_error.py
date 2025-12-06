import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32
import math as m


class EkfLocalizationError(Node):

    def __init__(self):
        super().__init__('ekf_localization_error')
        self.log = self.get_logger()

        # Via trial and error - forgot to intialize
        self.gt_pose = None
        self.ekf_pose = None
 
        # Create subscriptions first
        self.create_subscription(PoseStamped, '/tb3/ground_truth/pose', self.gt_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.ekf_cb, 10)

        # now we create the publisher as well = error for bearing and orientation
        self.position_err = self.create_publisher(Float32, '/ekf/error/position', 10)
        self.orientation_err = self.create_publisher(Float32, '/ekf/error/orientation', 10)

    
    # Helper functions - mainly extracted from codes in Lab 4 and 5 
    def wrap_to_pi(self, angle):
        return (angle + m.pi) % (2 * m.pi) - m.pi
    
    def gt_cb(self, msg):
        self.gt_pose = msg
        self.error()

    def ekf_cb(self, msg):
        self.ekf_pose = msg
        self.error()

    def error(self):

        # Initialize
        if self.gt_pose is None or self.ekf_pose is None: return  

        # First calculate position error
        x_gt = self.gt_pose.pose.position.x
        y_gt = self.gt_pose.pose.position.y

        x_ekf = self.ekf_pose.pose.pose.position.x
        y_ekf = self.ekf_pose.pose.pose.position.y

        pos_err = m.sqrt((x_gt - x_ekf)**2 + (y_gt - y_ekf)**2) # Utilize euclidean distance to characterize error

        # Next orientation error - # To extract this information, we utlized the formula in this link: https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion
        q_gt = self.gt_pose.pose.orientation
        yaw_gt = m.atan2(2.0 * (q_gt.w * q_gt.z + q_gt.x * q_gt.y), 1.0 - 2.0 * (q_gt.y ** 2 + q_gt.z ** 2))
        q_ekf = self.ekf_pose.pose.pose.orientation
        yaw_ekf = m.atan2(2.0 * (q_ekf.w * q_ekf.z + q_ekf.x * q_ekf.y), 1.0 - 2.0 * (q_ekf.y ** 2 + q_ekf.z ** 2))

        ang_err = self.wrap_to_pi(yaw_gt - yaw_ekf)

        #now we publish the errors to the topic
        msg_pos = Float32()
        msg_pos.data = pos_err
        self.position_err.publish(msg_pos)

        msg_ang = Float32()
        msg_ang.data = abs(ang_err)
        self.orientation_err.publish(msg_ang)

         # Finally log the errors.
        self.get_logger().info(f"pos_err={pos_err:.3f} m, ang_err={ang_err:.3f} rad")








    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    ekf_localization_error = EkfLocalizationError()
    ekf_localization_error.spin()
    ekf_localization_error.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
