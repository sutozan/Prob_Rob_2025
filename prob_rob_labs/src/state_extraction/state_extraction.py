import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


class StateExtraction(Node):

    def __init__(self):
        super().__init__('state_extraction')
        self.create_subscription(LinkStates,'/gazebo/link_states', self.feature_callback, 10)
        self.get_clock()
        self.log = self.get_logger()
        self.declare_parameter('frame_of_reference', 'odom')

        self.publisher_pose = self.create_publisher(PoseStamped, '/tb3/ground_truth/pose', 10)
        self.publisher_twist = self.create_publisher(TwistStamped, '/tb3/ground_truth/twist', 10)

    def spin(self):
        rclpy.spin(self)

    def feature_callback(self, msg):
        reference_frame = self.get_parameter('frame_od_parameter').value
        footprintIndex = msg.name.index('waffle_pi::base_footprint')
        self.log.info(f'{msg}')
        self.log.info(f'{footprintIndex}')
        self.log.info(f'Publishing: {msg.pose[footprintIndex]} and {msg.twist[footprintIndex]}') # base_footprint pose
        
        self.publisher_pose.publish(msg.pose[footprintIndex])
        self.publisher_twist.publish(msg.twist[footprintIndex])




def main():
    rclpy.init()
    state_extraction = StateExtraction()
    state_extraction.spin()
    state_extraction.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
