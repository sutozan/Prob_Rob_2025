import rclpy
from rclpy.node import Node
import json
from prob_rob_msgs.msg import Point2DArrayStamped
import os
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32
import math as m
import numpy as np
from geometry_msgs.msg import Twist, Quaternion,  PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class EkfLocalization(Node):

    def __init__(self):
        super().__init__('ekf_localization')
        self.log = self.get_logger()

        # Load the map via parameterization
        self.declare_parameter('map_path','~/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/config/landmarks.json')
        self.declare_parameter('dx_camera', 0.076) # We got these values from the camera topic!! --> needed for transformation
        self.declare_parameter('dy_camera', 0.0)
        self.declare_parameter('height', 0.5)
        
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
        # Assignment 3: Adding in the parameters that are needed for transformation

        #Define parameters
        self.height = self.get_parameter('height').get_parameter_value().double_value
        self.dx_camera = self.get_parameter('dx_camera').get_parameter_value().double_value
        self.dy_camera = self.get_parameter('dy_camera').get_parameter_value().double_value
        
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

        ## Now we bring in the intialization or the EKF - Simiar to Assignment 4

        # Now we initilize the nonlinear state update
        self.state = np.zeros(3)

        # Initialize covariances for update
        self.state_covariance = np.eye(3) * 1e-4
        self.input_covariance = np.diag([0.05**2, 0.05**2])

        # Initilize - Considering the latest command velocities
        self.u_v = 0.0
        self.u_w = 0.0
        self.w_epsilon = 1e-3  # Threshold for treating angular velocity as zero
        self.last_time = None
        self.initialized = False # this is for making sure measurment happens before odom

        # Subscriber for /cmd_vel and 
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher for EKF pose and covariance
        self.ekf_pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)

    # Helper functions - mainly extracted from codes in Lab 4 and 5
    
    def wrap_to_pi(self, angle):
        return (angle + m.pi) % (2 * m.pi) - m.pi
    
    def publish_ekf_pose(self, t): #made this into a function so we dont have to repeat code as we need it both in odom callback and hte landmark based one
     
        msg = PoseWithCovarianceStamped()
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int((t - int(t)) * 1e9) # fractional seconds.
        msg.header.frame_id = "odom" 

        # Position
        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        msg.pose.pose.position.z = 0.0

        # Orientation from yaw (theta)
        # Take consideration of the quaternions - publish
        # Reference for hte formula usage of getting yaw to quternion - https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        qz = m.sin(self.state[2]/2.0)
        qw = m.cos(self.state[2]/2.0)
        msg.pose.pose.orientation = Quaternion(x =0.0, y=0.0, z=qz, w=qw)


        # First pose
        msg.pose.covariance = [0.0] * 36 # create covariance - make everything 0 at first since we are only giving it 3 values.
        diag_covariances = np.diag(self.state_covariance)
        msg.pose.covariance[0] = diag_covariances[0] # X covariance
        msg.pose.covariance[7] = diag_covariances[1] # Y covariance
        msg.pose.covariance[35] = diag_covariances[2] # theta covariance - just yaw


        self.ekf_pub.publish(msg)
    
    def jacobians(self, v, w, dt, theta):
        
        if abs(w) < self.w_epsilon: # Change to linear model based on an epsilon - followed direction from manual - this changes the jacobians too
            Gx = np.array([
                [1.0, 0.0, -v*m.sin(theta)*dt],
                [0.0, 1.0,  v*m.cos(theta)*dt],
                [0.0, 0.0, 1.0]
            ])

            Gu = np.array([
                [m.cos(theta)*dt, 0.0],
                [m.sin(theta)*dt, 0.0],
                [0.0, dt]
            ])
        else: # All of these equations can be found in the textbook page 204 -- derived from there.
            Gx = np.array([
                [1.0, 0.0, (v/w)*(-m.cos(theta) + m.cos(theta + w*dt))],
                [0.0, 1.0, (v/w)*(-m.sin(theta) + m.sin(theta + w*dt))],
                [0.0, 0.0, 1.0]
            ])

            Gu = np.array([
                [(-m.sin(theta) + m.sin(theta + w*dt)) / w, v*( -m.cos(theta + w*dt)*dt ) / (w*w)],
                [( m.cos(theta) - m.cos(theta + w*dt)) / w, v*( -m.sin(theta + w*dt)*dt ) / (w*w)],
                [0.0, dt]
            ])
        
        return Gx, Gu
    
    def next_state(self, dt, v, w):
        x, y, theta = self.state # Order defined based on what is wanted in lab manual.

        if abs(w) < self.w_epsilon:
            # Straight line motion
            x_next = x + v * m.cos(theta) * dt
            y_next = y + v * m.sin(theta) * dt
            theta_next = theta
        else:
            # Circular motion - from textbook pages 205, equation 7.4
            theta_next = theta + w * dt
            x_next = x + (v/w) * (m.sin(theta + w * dt) - m.sin(theta))
            y_next = y - (v/w) * (m.cos(theta + w * dt) - m.cos(theta))

        # New states
        theta_next = self.wrap_to_pi(theta_next)
        self.x_pred = np.array([x_next, y_next, theta_next])
        self.state = self.x_pred


        # Now lets also have it calculate the covariance update
        Gx, Gu = self.jacobians(v, w, dt, theta)
        self.state_covariance = Gx @ self.state_covariance @ Gx.T + Gu @ self.input_covariance @ Gu.T


        return  self.state, self.state_covariance, theta_next, x_next, y_next

    def odom_callback(self, msg): #running for odometry sample
         # Extract velocity inputs from odometry topic
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        self.u_v = v
        self.u_w = w

        # Extract timestamp
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not self.initialized: return

        if self.last_time is None: 
            return

        dt = t - self.last_time # Now to check for the time - if negative elapsed time we drop it --> late message
        if dt <= 0:
            return  # late message
        else:
            # update everything
            self.next_state(dt, v, w)
            self.last_time = t
            self.publish_ekf_pose(t) # publish even when we dont see the landmark

            # Log results
            self.log.info(
                f"State: x={self.state[0]:.3f} y={self.state[1]:.3f} θ={self.state[2]:.3f} "
                f"Update Covariance (Diag only) = {np.diag(self.state_covariance)}"
            )


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

        # Time stamp of  measurement
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not self.initialized:
            self.last_time = t
            self.initialized = True
            return

        # Check filter intializtion - did the same in odom callback
        if self.last_time is None:
            self.last_time = t
            return

        dt = t - self.last_time

        # Check if measurment is late - ignore if it is
        if dt < 0.0:
            return
        else:
            self.next_state(dt, self.u_v, self.u_w)
            self.last_time = t

        # Now lets rotate from robot to camera frame
        x_r, y_r, theta_r = self.state
        x_camera = (self.dx_camera * m.cos(theta_r)) - (self.dy_camera * m.sin(theta_r)) + x_r
        y_camera = (self.dx_camera * m.sin(theta_r)) + (self.dy_camera * m.cos(theta_r)) + y_r


        x = landmark["x"]
        y = landmark["y"]

        # now we have all the values we need

        # Log in serial - commented out for A2 to reduce clutter
        #self.log.info(f'Landmark properties: {color}, x_pos: {x}, y_pos: {y}')

        if self.fx is None or self.fy is None or self.cx is None or self.cy is None: return
        
        # From here on is the same code as the vision_geometry code.
        points = msg.points

        # Have to check for points - or else crashes - also for A2 this is needed so the no points doesnt register an error
        if not points: return
        
        # define the x and y values
        x_values = [p.x for p in points]
        y_values = [p.y for p in points]
        
        # calculating height and veritical axis symmetry position
        dy = max(y_values) - min(y_values)

        # Extension for Dropping the Offending Measurment 
        # realized we were getting 0 height sometime and this causes an issue within the reading.
        # need 4 points and positive height --> so we adjust for these.
        if len(points) < 2: return
        if dy <= 0: return

        vertical_x = (min(x_values)+max(x_values))/2
        #self.log.info(f"Height: {dy:.3f}, Position of Vertical Symmetry Axis: {vertical_x:.3f}")

        # Formulas frm lab manual.
        # calculate theta
        theta_meas = m.atan((self.cx - vertical_x) / self.fx)
        # calculate distance
        d_meas = self.height * (self.fy/(dy*m.cos(theta_meas)))

        # publish results
        dist_msg = Float32()
        dist_msg.data = d_meas
        self.pub_distance[color].publish(dist_msg)

        bearing_msg = Float32()
        bearing_msg.data = theta_meas
        self.pub_bearing[color].publish(bearing_msg)

        # log info
        sigma_d2 = self.b_d + self.a_d * (d_meas ** 2)
        sigma_th2 = self.b_b + self.a_t * (theta_meas ** 2)

        # From trial and error - sometimes sigma_d2 and sigma_th2 become negative? so to make sure that doesnt throw off our covariance later we clamp them to positive values
        sigma_d2 = max(sigma_d2, 1e-6)
        sigma_th2 = max(sigma_th2, 1e-6)

        # Main log we care about it!
        self.log.info(f"Color = {color}, Distance={d_meas:.3f} m, Bearing={theta_meas:.3f} rad, sigma_d²={sigma_d2:.5f}, sigma_b²={sigma_th2:.5f}")

        ## Now lets perform the correction

        # Predicted measurement in camera frame - naming converntions from previous lab
        dx_gt = x - x_camera
        dy_gt = y - y_camera
        d_pred = m.sqrt(dx_gt**2 + dy_gt**2)

        # Now we calculate the bearing error.
        theta_world = m.atan2(dy_gt, dx_gt)
        theta_pred = self.wrap_to_pi(theta_world-theta_r)

        # Define measured z and one from vision - for update
        z_meas = np.array([d_meas, theta_meas])  
        z_hat  = np.array([d_pred, theta_pred]) 

        # Now for jacobian of the mesaurment model - naming convetion based off of textbook
        
        q = dx_gt**2 + dy_gt**2

        H = np.array([
            [-dx_gt/m.sqrt(q),      -dy_gt/m.sqrt(q),         0.0],
            [ dy_gt/q,           -dx_gt/q,             -1.0]
        ])

        Q = np.diag([sigma_d2, sigma_th2]) # measurment covariance

        # Calculate kalman gain
        S = H @ self.state_covariance @ H.T + Q
        K = self.state_covariance @ H.T @ np.linalg.inv(S)

        # now we update
        innovation = z_meas - z_hat
        innovation[1] = self.wrap_to_pi(innovation[1])  # need to make sure bearing is wrapped to pi

        self.state = self.state + K @ (innovation)
        self.state[2] = self.wrap_to_pi(self.state[2])  # need to make sure bearing is wrapped ot pi

        I = np.eye(3)
        self.state_covariance = (I - K @ H) @ self.state_covariance

        # Now we need to transform from camera to robot body per the lab manual - move around values from before
       
        x_r = self.state[0] - self.dx_camera*m.cos(self.state[2]) + self.dy_camera*m.sin(self.state[2])
        y_r = self.state[1] - self.dx_camera*m.sin(self.state[2]) - self.dy_camera*m.cos(self.state[2])
        theta_r = self.state[2]

        self.state = np.array([x_r, y_r, theta_r]) # redefine 

        # Log results
        #self.log.info(
            #f"State: color={color} x={self.state[0]:.3f} y={self.state[1]:.3f} θ={self.state[2]:.3f} "
            #f"Update Covariance (Diag only) = {np.diag(self.state_covariance)}"
        #)
        
        self.log.info(f"d_meas={d_meas} d_pred={d_pred} theta_meas = {theta_meas} theta_pred = {theta_pred}")
        self.publish_ekf_pose(t)

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
