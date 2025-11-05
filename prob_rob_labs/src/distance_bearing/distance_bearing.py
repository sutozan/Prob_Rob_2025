import rclpy
from rclpy.node import Node
import math

# Message from Assignment 1
from prob_rob_msgs.msg import Point2DArrayStamped

# NEW: Messages for Assignment 2
from sensor_msgs.msg import CameraInfo  # To get camera properties
from std_msgs.msg import Float64        # We will publish this!


class DistanceBearing(Node):

    def __init__(self):
        super().__init__('vision_geometry')
        self.log = self.get_logger()
        
        # --- Parameters ---
        self.declare_parameter('color','cyan')
        self.declare_parameter('manual', False)

        color = self.get_parameter('color').get_parameter_value().string_value
        manual = self.get_parameter('manual').get_parameter_value().bool_value
        
        # --- Knowns ---
        self.h = 0.5 # Known landmark height in meters (from lab PDF)
        
        # --- Class variables to store camera intrinsics ---
        self.f_x = 0.0
        self.f_y = 0.0
        self.c_x = 0.0
        self.camera_info_ok = False # Flag to wait until we have info
        
        # --- Publishers ---
        # NEW: Create publishers for distance and bearing
        # We publish to topics that include the color name
        self.distance_pub = self.create_publisher(
            Float64,
            f'/landmark_{color}/distance', # e.g., /landmark_cyan/distance
            10)
        
        self.bearing_pub = self.create_publisher(
            Float64,
            f'/landmark_{color}/bearing', # e.g., /landmark_cyan/bearing
            10)

        # --- Subscribers ---
        # NEW: Subscriber for camera info.
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)

        # This is your main subscriber from Assignment 1
        if manual:
            test_points = [
                (320, 263), (305, 263), (354, 136), (304, 137),
                (339, 264), (320, 264), (341, 250), (320, 132)
            ]
            # Note: The manual test won't run correctly
            # because it doesn't get camera info.
            self.compute_from_points(test_points)
        else:
            topic_name = f'/vision_{color}/corners'
            self.create_subscription(
                Point2DArrayStamped, 
                topic_name, 
                self.height_position, # This is your main callback
                10)
            self.log.info(f'Subscribing to topic: {topic_name}')

    # NEW callback for Assignment 2
    def camera_info_callback(self, msg):
        """
        This callback runs to get the camera intrinsic parameters.
        """
        if self.camera_info_ok:
            return # We already have the info
            
        # The P matrix in the PDF is the K matrix in the message
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.f_x = msg.k[0] # P[0] in the lab doc
        self.c_x = msg.k[2] # P[2] in the lab doc
        self.f_y = msg.k[5] # P[5] in the lab doc
        
        # Check if we got valid data
        if self.f_x > 0 and self.f_y > 0:
            self.camera_info_ok = True # Set the flag
            
            self.log.info('--- Camera Info Received ---')
            self.log.info(f'  f_x: {self.f_x:.2f}')
            self.log.info(f'  f_y: {self.f_y:.2f}')
            self.log.info(f'  c_x: {self.c_x:.2f}')
            
            # We have what we need, so we can destroy this subscription
            self.destroy_subscription(self.camera_info_sub)

    # MODIFIED: This is your main logic from Assignment 1
    def height_position(self, msg): 
        
        # Wait until we have camera info
        if not self.camera_info_ok:
            self.log.warn('Waiting for camera info...', throttle_duration_sec=5.0)
            return

        points = msg.points
        if not points:
            return # No points, do nothing

        # --- Assignment 1 Logic ---
        x_values = [p.x for p in points]
        y_values = [p.y for p in points]
        
        y_min = min(y_values)
        y_max = max(y_values)
        x_min = min(x_values)
        x_max = max(x_values)

        delta_y_p = y_max - y_min # This is "height" from A1
        x_p = (x_min + x_max) / 2 # This is "vertical_x" from A1
        
        # --- Assignment 2 Logic ---
        
        if delta_y_p <= 0:
            return # Invalid data, skip this message

        bearing = math.atan((self.c_x - x_p) / self.f_x)
        cos_bearing = math.cos(bearing) 
        if cos_bearing <= 0.01: # Avoid division by zero or near-zero 		
            return 
            
        distance = (self.f_y * self.h) / (delta_y_p * cos_bearing)
        
        # Log the result
        self.log.info(f"Height: {delta_y_p:.1f}, Axis: {x_p:.1f} -> Dist: {distance:.3f} m, Bearing: {math.degrees(bearing):.3f} deg")

        # --- NEW: Publish the results ---
        dist_msg = Float64()
        dist_msg.data = distance
        self.distance_pub.publish(dist_msg)
        
        bear_msg = Float64()
        bear_msg.data = bearing
        self.bearing_pub.publish(bear_msg)
    
    # This is only for feeding in some example points
    def compute_from_points(self, test_points):
        x_values = [p[0] for p in test_points]
        y_values = [p[1] for p in test_points]

        height = max(y_values) - min(y_values)
        vertical_x = (min(x_values)+max(x_values))/2
        self.log.warn(f"Manual Mode: Height: {height:.3f}, Axis: {vertical_x:.3f}")
        self.log.warn("Manual Mode does not calculate distance or bearing.")

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    distance_bearing = DistanceBearing()
    try:
        distance_bearing.spin()
    except KeyboardInterrupt:
        pass
    finally:
        distance_bearing.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
