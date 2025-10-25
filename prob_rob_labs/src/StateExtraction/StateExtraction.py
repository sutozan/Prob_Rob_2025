import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


class StateExtraction(Node):

    def __init__(self):
        super().__init__('StateExtraction')
        
        # Subscriber for Link States
        self.create_subscription(LinkStates,'/gazebo/link_states', self.feature_callback, 10)
        
        # Parameter for frame of reference - made default odom for the sake of this lab
        self.declare_parameter('frame_of_reference', 'odom')

        # Publishers - both for twist and Pose
        # For future reference: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
        self.publisher_pose = self.create_publisher(PoseStamped, '/tb3/ground_truth/pose', 10)
        self.publisher_twist = self.create_publisher(TwistStamped, '/tb3/ground_truth/twist', 10)

        # Logger and Check Setup
        self.log = self.get_logger()
        self.log.info('State Extraction Node Started')


    def spin(self):
        rclpy.spin(self)

    def feature_callback(self, msg):
        reference_frame = self.get_parameter('frame_of_reference').get_parameter_value().string_value
        footprintIndex = msg.name.index('waffle_pi::base_footprint') # To get the right linke of interest - robots body
       
        # Check that everything is being recieved
        self.log.info('Received link_states message')
        
        # First, extract pose and then publish it.
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg() # Get time
        pose_msg.header.frame_id = reference_frame # Get reference frame
        pose_msg.pose = msg.pose[footprintIndex] # and finally the pose of the link of interest - extraction here

        # Now we do the same for the twist
        twist_msg = TwistStamped()
        twist_msg.header.stamp = pose_msg.header.stamp
        twist_msg.header.frame_id = reference_frame
        twist_msg.twist = msg.twist[footprintIndex] # extraction of the twist - so velocity

        # Publish the full message that we have extracted.
        self.publisher_pose.publish(pose_msg)
        self.publisher_twist.publish(twist_msg)
        self.log.info('Published ground truth pose and twist.')






def main():
    rclpy.init()
    node = StateExtraction()      
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
