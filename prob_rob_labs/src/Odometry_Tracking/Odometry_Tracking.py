import rclpy
from rclpy.node import Node
import math, numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from message_filters import Subscriber, ApproximateTimeSynchronizer


# Everything works - commit.
# Also can run this for teleop -  ros2 run turtlebot3_teleop teleop_keyboard

class Odometry_Tracking(Node):

    def __init__(self):
        super().__init__('odometry_tracking')
        self.log = self.get_logger()

        # Define model parameters for linear and angular velocity
        self.tau_lin = 0.55 #t90 from graph approx
        self.tau_ang = 0.35 #t90 from graph approx
        self.G_v = 1.0
        self.G_w = 1.0

        # Now we initilize the nonlinear state update
        self.state = np.zeros(5)

        # Initialize covariances for update
        self.state_covariance = np.eye(5) * 0.01
        self.input_covariance = np.diag([0.05**2, 0.05**2])

        # Initilize - Considering the latest command velocities
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

        self.log.info("Step 2 - Performing EKF Prediction")

    def forgetting(self, dt):
        a_v = math.pow(0.1, dt / self.tau_lin)
        a_w = math.pow(0.1, dt / self.tau_ang)
        return a_v, a_w
    
    def next_state(self, dt, a_v, a_w):
        theta, x, y, v, w = self.state
        u_v, u_w = self.u_v, self.u_w

        # first-order dynamic response from manual notes
        v_next = a_v * v + self.G_v * (1.0 - a_v) * u_v
        w_next = a_w * w + self.G_w * (1.0 - a_w) * u_w

        # Now get the next states
        theta_next = theta + w * dt
        x_next = x + v * math.cos(theta) * dt
        y_next = y + v * math.sin(theta) * dt

        # New states
        x_pred = np.array([theta_next, x_next, y_next, v_next, w_next])

        return  x_pred, theta_next, x_next, y_next, v_next, w_next
    
    def jacobians(self, dt, a_v, a_w, x_curr):
        theta, x, y, v, w = x_curr

        Gx = np.array([
            [1, 0, 0, 0, dt],
            [-v * math.sin(theta) * dt, 1, 0, math.cos(theta) * dt, 0],
            [ v * math.cos(theta) * dt, 0, 1, math.sin(theta) * dt, 0],
            [0, 0, 0, a_v, 0],
            [0, 0, 0, 0, a_w]
        ])

        Gu = np.array([
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [self.G_v * (1 - a_v), 0.0],
            [0.0, self.G_w * (1 - a_w)]
        ])

        return Gx, Gu


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

        # Log before the prediction model
        self.log.info(
            f"Δt={dt:.3f}s  wr={w_r:.2f}  wl={w_l:.2f}  "
            f"wg={w_g:.3f}  u_v={self.u_v:.2f}  u_w={self.u_w:.2f}"
        )

        # Now lets get our predicted results from the functions defined above
        a_v, a_w = self.forgetting(dt)
        Gx, Gu = self.jacobians(dt, a_v, a_w, self.state)
        self.state_covariance = Gx @ self.state_covariance @ Gx.T + Gu @ self.input_covariance @ Gu.T

        x_pred, theta_next, x_next, y_next, v_next, w_next = self.next_state(dt, a_v, a_w)
        self.state = x_pred

        # Log predicted results
        self.log.info(
            f"Predicted: x={x_next:.3f} y={y_next:.3f} θ={theta_next:.3f} "
            f"v={v_next:.3f} w={w_next:.3f} (a_v={a_v:.3f}, a_w={a_w:.3f})"
            f"Update Covariance = {self.state_covariance}"
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
