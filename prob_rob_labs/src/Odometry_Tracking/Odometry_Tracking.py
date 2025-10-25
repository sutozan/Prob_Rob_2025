import rclpy
from rclpy.node import Node
import math, numpy as np
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
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

        # The following were commented out but kept here for testing sake - we wanted to see how it impacted our results if we 
        # start at a different angle.
        #self.state = np.array([math.pi / 2, 0.0, 0.0, 0.0, 0.0]) #start at 90 degrees
        #self.state = np.array([math.radians(45), 0.0, 0.0, 0.0, 0.0]) #start at 45 degrees

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
        # Reason why this is differnet than above - we are using these to synchronize the time so its from the message filter class
        # otherwise we would have the two data come in at different rates 
        # reference - https://docs.ros.org/en/rolling/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Python.html
        imu_sub = Subscriber(self, Imu, '/imu')
        joint_sub = Subscriber(self, JointState, '/joint_states')

        # Time synchronizer - by choosing these two we are avoiding potential error of when t goes on but no cmd_vel is given.
        sync = ApproximateTimeSynchronizer([imu_sub, joint_sub], 10, 0.03)
        sync.registerCallback(self.synced_callback)

        # Publish EKF Odometry
        self.odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)

    def forgetting(self, dt):
        # Made this into its own function per lab manual.
        a_v = math.pow(0.1, dt / self.tau_lin)
        a_w = math.pow(0.1, dt / self.tau_ang)
        return a_v, a_w
    
    def next_state(self, dt, a_v, a_w):
        theta, x, y, v, w = self.state # Order defined based on what is wanted in lab manual.
        u_v, u_w = self.u_v, self.u_w # Initilaize values.

        # first-order dynamic response from manual notes - derivation can be found in document submission
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

        # calculations can be found in notes
        Gx = np.array([
            [1.0, 0.0, 0.0, 0.0, dt],
            [-v * math.sin(theta) * dt, 1.0, 0.0, math.cos(theta) * dt, 0.0],
            [ v * math.cos(theta) * dt, 0.0, 1.0, math.sin(theta) * dt, 0.0],
            [0.0, 0.0, 0.0, a_v, 0.0],
            [0.0, 0.0, 0.0, 0.0, a_w]
        ])

        # Note Gu = B matrix - made this way to remember

        Gu = np.array([
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [self.G_v * (1 - a_v), 0.0],
            [0.0, self.G_w * (1 - a_w)]
        ])

        return Gx, Gu
    
    def extract_measurements(self, imu_msg, joint_msg):
        # Wheel parameters - given in manual
        r_w = 0.033       # 33 mm = 0.033 m
        R_w = 0.07175     # 71.75 mm = 0.07175 m

        w_r = w_l = 0.0
        # after exploring topic, found we need to filter by name first.
        if joint_msg.velocity:
            for name, vel in zip(joint_msg.name, joint_msg.velocity):
                if 'wheel_right' in name:
                    w_r = vel
                elif 'wheel_left' in name:
                    w_l = vel
        w_g = imu_msg.angular_velocity.z
        z = np.array([w_r, w_l, w_g], dtype=float)

        # Measurement matrix (C)
        C = np.array([
            [0.0, 0.0, 0.0, 1.0 / r_w,  R_w / r_w],
            [0.0, 0.0, 0.0, 1.0 / r_w, -R_w / r_w],
            [0.0, 0.0, 0.0, 0.0, 1.0]
        ], dtype=float)

        # Measurement covariance (R) - reasoning for selection in manual
        R = np.diag([1.0, 1.0, 1.0])

        return z, C, R

    def cmd_callback(self, msg):
        # Update here with what was given as input.
        self.u_v = msg.linear.x
        self.u_w = msg.angular.z

    def synced_callback(self, imu_msg, joint_msg):
        
        # compute delta t # got 0.034s

        # We want to choose the one that takes the longest time - since intuitively the faster one would have already arrived
        # Found that we needed to actually make it into a float point since these are headers and are split into secs and nano.
        t = max(
            imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9,
            joint_msg.header.stamp.sec + joint_msg.header.stamp.nanosec * 1e-9,
        )
        
        # Trial and error - was failing when we didnt consider for the fact that there is no time in the first callback.
        if self.last_time is None:
            self.last_time = t
            return
        
        # calculate duration between last and current update - this becomes our dt - found it was pretty much consistent - 0.034s
        dt = t - self.last_time
        self.last_time = t

        # We moved this to the measurment extraction since it is needed there but will 
        # also keep it here to make sure we can see what the outputs are before the filter - so basically for sanity check.
        w_r = w_l = 0.0
        if joint_msg.velocity:
            for name, vel in zip(joint_msg.name, joint_msg.velocity):
                if 'wheel_right' in name:
                    w_r = vel
                elif 'wheel_left' in name:
                    w_l = vel
        w_g = imu_msg.angular_velocity.z

        # Log before the prediction model
        self.log.info(
            f"Δt={dt:.3f}s  wr={w_r:.2f}  wl={w_l:.2f}  "
            f"wg={w_g:.3f}  u_v={self.u_v:.2f}  u_w={self.u_w:.2f}"
        )

        # Prediction - Now lets get our predicted results from the functions defined above
        # Bring in all parameters from above functions.
        a_v, a_w = self.forgetting(dt)
        Gx, Gu = self.jacobians(dt, a_v, a_w, self.state)

        # calculate the hat_covariance
        self.state_covariance = Gx @ self.state_covariance @ Gx.T + Gu @ self.input_covariance @ Gu.T

        # Next, we get the n state.
        x_pred, theta_next, x_next, y_next, v_next, w_next = self.next_state(dt, a_v, a_w)
        self.state = x_pred

        # Measurement extraction
        z, C, R = self.extract_measurements(imu_msg, joint_msg)

        # EKF Innovation / Correction
        K = self.state_covariance @ C.T @ np.linalg.inv((C @ self.state_covariance @ C.T) + R)
        self.state = self.state + K @ (z - C @ self.state)
        self.state_covariance = (np.eye(5) - K @ C) @ self.state_covariance
        
        # Log predicted results
        self.log.info(
            f"Predicted: x={x_next:.3f} y={y_next:.3f} θ={theta_next:.3f} "
            f"v={v_next:.3f} w={w_next:.3f} (a_v={a_v:.3f}, a_w={a_w:.3f})"
            f"Update Covariance (Diag only) = {np.diag(self.state_covariance)}"
        )
        
        # Publish EKF Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = imu_msg.header.stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Now we publish the position (x,y,z)
        odom_msg.pose.pose.position.x = self.state[1]
        odom_msg.pose.pose.position.y = self.state[2]
        odom_msg.pose.pose.position.z = 0.0

        # Take consideration of the quaternions - publish
        # Reference for hte formula usage of getting yaw to quternion - https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        qz = math.sin(self.state[0]/2.0)
        qw = math.cos(self.state[0]/2.0)
        odom_msg.pose.pose.orientation = Quaternion(x =0.0, y=0.0, z=qz, w=qw)

        # Now we have to map 5x5 state to 6x6 expectation
        # First pose
        odom_msg.pose.covariance = [0.0] * 36 # create covariance - make everything 0 at first since we are only giving it 3 values.
        diag_covariances = np.diag(self.state_covariance)
        odom_msg.pose.covariance[0] = diag_covariances[1] # X covariance
        odom_msg.pose.covariance[7] = diag_covariances[2] # Y covariance
        odom_msg.pose.covariance[35] = diag_covariances[0] # theta covariance - just yaw

        # Next twist - same reasoning as above.
        odom_msg.twist.twist.linear.x = self.state[3]
        odom_msg.twist.twist.angular.z = self.state[4]
        odom_msg.twist.covariance = [0.0]*36
        odom_msg.twist.covariance[0] = diag_covariances[3]
        odom_msg.twist.covariance[35] = diag_covariances[4]

        # Publish everything
        self.odom_pub.publish(odom_msg)
    
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
