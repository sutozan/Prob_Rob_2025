import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from message_filters import Subscriber, ApproximateTimeSynchronizer


# Everything works - commit.
# Also can run this for teleop -  ros2 run turtlebot3_teleop teleop_keyboard

class Odometry_Tracking(Node):

    def __init__(self):
        super().__init__('odometry_tracking')
        self.log = self.get_logger()
        
        # Considering hte latest command velocities
        self.u_v = 0.0
        self.u_w = 0.0
        self.last_time = None

        # Subscriber for /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Subscribers for measurment system
        imu_sub = Subscriber(self, Imu, '/imu')
        joint_sub = Subscriber(self, JointState, '/joint_states')

        # Time synchronizer
        sync = ApproximateTimeSynchronizer([imu_sub, joint_sub], 10, 0.03)
        sync.registerCallback(self.synced_callback)

        self.log.info("Step 0 - Testing Initialization")

    def cmd_callback(self, msg):
        self.u_v = msg.linear.x
        self.u_w = msg.angular.z

    def synced_callback(self, imu_msg, joint_msg):
        # Extracting w_g from gyroscope
        w_g = imu_msg.angular_velocity.z

        # Next we need wheel speeds from joint states
        w_r = w_l = 0.0
        for name, vel in zip(joint_msg.name, joint_msg.velocity):
            if 'wheel_right_joint' in name:
                w_r = vel
            elif 'wheel_left_joint' in name:
                w_l = vel
        
        # compute delta t # got 0.034s
        t = max(
            imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9,
            joint_msg.header.stamp.sec + joint_msg.header.stamp.nanosec * 1e-9,
        )

        if self.last_time is None:
            self.last_time = t
            return
        dt = t - self.last_time
        self.last_time = t

        # log
        self.log.info(
            f"Δt={dt:.3f}s  ωr={w_r:.2f}  ωl={w_l:.2f}  "
            f"ωg={w_g:.3f}  u_v={self.u_v:.2f}  u_ω={self.u_w:.2f}"
        )


    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    odometry_tracking = Odometry_Tracking()
    odometry_tracking.spin()
    odometry_tracking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
