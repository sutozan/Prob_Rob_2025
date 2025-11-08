import rclpy
from rclpy.node import Node
from prob_rob_msgs.msg import Point2DArrayStamped
from sensor_msgs.msg import CameraInfo
import math as m
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

# Also can run this for teleop -  ros2 run turtlebot3_teleop teleop_keyboard

class MeasurmentCalculations(Node):

    def __init__(self):
        super().__init__('measurment_calculations')
        self.log = self.get_logger()
        
        # Define parameters first
        self.declare_parameter('color','cyan')
        self.declare_parameter('height', 0.5)

         # Define the parameters - take in these values from user input - for this lab it is the same, setup for next lab mostly.
        self.color = self.get_parameter('color').get_parameter_value().string_value
        self.height = self.get_parameter('height').get_parameter_value().double_value

        # define the topic names based on the parameter
        # Since the color of the landmark determines the topic we also parameterized this per the lab instructions.
        topic_name = f'/vision_{self.color}/corners'
        topic_name_2 = f'/vision_{self.color}/distance'
        topic_name_3 = f'/vision_{self.color}/bearing'
        
        # create subscriptions
        self.create_subscription(Point2DArrayStamped, topic_name, self.calculations, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_callback, 10)
        self.log.info(f'Subscribing to topic: {topic_name}')
        self.log.info(f'Subscribing to topic: {topic_name_2}')
        self.log.info(f'Subscribing to topic: {topic_name_3}')

        # create publishers for plotting
        self.pub_distance = self.create_publisher(Float32, topic_name_2, 10)
        self.pub_bearing = self.create_publisher(Float32, topic_name_3, 10)

        # Trial and Error: Initalize variables
        self.fx = self.fy = self.cx = self.cy = None

    def camera_callback(self, msg):
        
        # Call in projection model
        P = msg.p

        # From model in manual: [[fx, 0 , cx], [0, fy, cy], [0, 0, 1]]^T
        # However, when we echo topic we see that P is 3x4 --> has extra values Tx, Ty. Adjust accordingly
        self.fx = P[0]
        self.fy = P[5]
        self.cx = P[2]
        self.cy = P[6]

        # Check camera projection model values - commented out for now since it creates clutter in terminal.
        #self.log.info(f"Camera Projection Model: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}")


    def calculations(self, msg): # Remember a ROS 2 subscriber callback always 
        # takes exactly one argument which is the msg.

        # From trial and error found we need to include initlization here as well or else
        # it throws an error during the first run since it cant find the values.
        if self.fx is None or self.cx is None:
            return
        
        # From here on is the same code as the vision_geometry code.
        points = msg.points

        # Have to check for points - or else crashes
        if not points: return
        
        # define the x and y values
        x_values = [p.x for p in points]
        y_values = [p.y for p in points]
        
        # calculating height and veritical axis symmetry position
        dy = max(y_values) - min(y_values)

        # ## Extension for Dropping the Offending Measurment ##
        # realized we were getting 0 height sometime and this causes an issue within the reading.
        # need 4 points and positive height --> so we adjust for these.
        if len(points) < 2: return
        if dy <= 0: return

        vertical_x = (min(x_values)+max(x_values))/2
        self.log.info(f"Height: {dy:.3f}, Position of Vertical Symmetry Axis: {vertical_x:.3f}")

        # Formulas frm lab manual.
        # calculate theta
        theta = m.atan((self.cx - vertical_x) / self.fx)
        # calculate distance
        d = self.height * (self.fy/(dy*m.cos(theta)))

        # publish results
        dist_msg = Float32()
        dist_msg.data = d
        self.pub_distance.publish(dist_msg)

        bearing_msg = Float32()
        bearing_msg.data = theta
        self.pub_bearing.publish(bearing_msg)

        # log info
        self.log.info(f"Distance: {d:.3f} m, Bearing: {m.degrees(theta):.2f}°")


    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    measurment_calculations = MeasurmentCalculations()
    measurment_calculations.spin()
    measurment_calculations.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
