import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


class StateExtraction(Node):

    def __init__(self):
        super().__init__('StateExtraction')
        
        # Subscriber
        self.create_subscription(LinkStates,'/gazebo/link_states', self.feature_callback, 10)
        
        # Parameter for frame of reference
        self.declare_parameter('frame_of_reference', 'odom')

        # Publishers
        self.publisher_pose = self.create_publisher(PoseStamped, '/tb3/ground_truth/pose', 10)
        self.publisher_twist = self.create_publisher(TwistStamped, '/tb3/ground_truth/twist', 10)

        # Logger and Check Setup
        self.log = self.get_logger()
        self.log.info('State Extraction Node Started')


    def spin(self):
        rclpy.spin(self)

    def feature_callback(self, msg):
        reference_frame = self.get_parameter('frame_of_reference').get_parameter_value().string_value
        footprintIndex = msg.name.index('waffle_pi::base_footprint')
       
        # Check that everything is being recieved
        self.log.info('Received link_states message')
        
        # First, extract pose and then publish it.
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = reference_frame
        pose_msg.pose = msg.pose[footprintIndex]

        # Now we do the same for the twist
        twist_msg = TwistStamped()
        twist_msg.header.stamp = pose_msg.header.stamp
        twist_msg.header.frame_id = reference_frame
        twist_msg.twist = msg.twist[footprintIndex]

        # Publish the messages
        self.publisher_pose.publish(pose_msg)
        self.publisher_twist.publish(twist_msg)
        self.log.info('Published ground truth pose and twist.')






def main():
    rclpy.init()
    node = StateExtraction()       # ✅ instantiate the class
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
