import rclpy
from rclpy.node import Node
from prob_rob_msgs.msg import Point2DArrayStamped


class VisionGeometry(Node):

    def __init__(self):
        super().__init__('vision_geometry')
        self.log = self.get_logger()
        
        # Define parameters first
        self.declare_parameter('color','cyan')
        self.declare_parameter('manual', False)

        # Define the parameters - take in these values from user input.
        color = self.get_parameter('color').get_parameter_value().string_value
        manual = self.get_parameter('manual').get_parameter_value().bool_value

        if manual: # As per the lab manual, we also wanted to test some manual points. this is not used in the rest of the lab.
            test_points = [
                (320, 263), (305, 263), (354, 136), (304, 137),
                (339, 264), (320, 264), (341, 250), (320, 132)
            ]
            self.compute_from_points(test_points)
        else:
            # define the topic name based on the parameter
            topic_name = f'/vision_{color}/corners'
            # create subscription
            self.create_subscription(Point2DArrayStamped, topic_name, self.height_position, 10)
            self.log.info(f'Subscribing to topic: {topic_name}')

    def height_position(self, msg): # Remember a ROS 2 subscriber callback always 
        # takes exactly one argument which is the msg --> Common error we keep making, noted here.
        points = msg.points # define points
        
        # define the x and y values --> get them from points - simple iteration.
        x_values = [p.x for p in points]
        y_values = [p.y for p in points]
        
        # calculating height and veritical axis symmetry position
        height = max(y_values) - min(y_values)
        vertical_x = (min(x_values)+max(x_values))/2
        self.log.info(f"Height: {height:.3f}, Position of Vertical Symmetry Axis: {vertical_x:.3f}")
    
    # This is only for feeding in some example points - since this is a list it is iterated through it differently here.
    def compute_from_points(self, test_points):
        x_values = [p[0] for p in test_points]
        y_values = [p[1] for p in test_points]

        # Same algorithm
        height = max(y_values) - min(y_values)
        vertical_x = (min(x_values)+max(x_values))/2
        self.log.info(f"Height: {height:.3f}, Position of Vertical Symmetry Axis: {vertical_x:.3f}")


    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    vision_geometry = VisionGeometry()
    vision_geometry.spin()
    vision_geometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
