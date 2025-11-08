import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32



class VarianceEstimation(Node):

    def __init__(self):
        super().__init__('variance_estimation')
        self.log = self.get_logger()

        # First lets set some parameters as well as our hardcoded constants from the calculations - see lab manual
        self.declare_parameter('color', 'cyan')
        self.b_d = -0.0170 #y-intercept for distance
        self.b_b = 0.0034 #y-intercept for bearing
        self.a_d = 0.0013331 #slope for distance
        self.a_t = 0.035134 # slope for theta

        # Define the parameters - take in these values from user input.
        self.color = self.get_parameter('color').get_parameter_value().string_value

        # Now subscribe to topics - measurments - from assignment 2
        self.create_subscription(Float32, f'/vision_{self.color}/distance', self.distance_cb, 10)
        self.create_subscription(Float32, f'/vision_{self.color}/bearing', self.bearing_cb, 10)

        # Define publishers for the variances.
        self.publish_sigma_d2 = self.create_publisher(Float32, f'/vision_{self.color}/variance_distance', 10)
        self.publish_sigma_b2 = self.create_publisher(Float32, f'/vision_{self.color}/variance_bearing', 10)

        # Initlialize measurments
        self.d = None
        self.theta = None

    def distance_cb(self, msg):
        self.distance = float(msg.data)
        self.variance_model()

    def bearing_cb(self, msg):
        self.theta = float(msg.data)
        self.variance_model()

    def variance_model(self):
        # First do the checks
        if self.distance is None or self.theta is None: return

        # Use our determined model to define variance calculation
        sigma_d2 = self.b_d + self.a_d * (self.distance ** 2)
        sigma_be2 = self.b_b + self.a_t * (self.theta ** 2)

        # Publish both variances
        msg_dis = Float32()
        msg_dis.data = sigma_d2
        self.publish_sigma_d2.publish(msg_dis)

        msg_bearing = Float32()
        msg_bearing.data = sigma_be2
        self.publish_sigma_b2.publish(msg_bearing)

        # Log for check
        self.log.info(f"Distance={self.distance:.3f} m, Bearing={self.theta:.3f} rad, sigma_d²={sigma_d2:.5f}, sigma_b²={sigma_be2:.5f}")



    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    variance_estimation = VarianceEstimation()
    variance_estimation.spin()
    variance_estimation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
