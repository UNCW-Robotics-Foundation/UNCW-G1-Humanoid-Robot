#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import cv2
from cv_bridge import CvBridge
import mediapipe as mp
import numpy as np

class PalmTrackerNode(Node):
    def __init__(self):
        super().__init__('palm_tracker_node')
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        # Camera Intrinsic placeholders
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.latest_depth_img = None

        # Subscribers
        # Ensure your RealSense launch aligns depth to the color frame!
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)

        # Publisher for the 3D Palm position
        self.palm_pub = self.create_publisher(PointStamped, '/palm_center_3d', 10)
        self.get_logger().info("Palm Tracker Node Initialized.")

    def info_callback(self, msg):
        # Extract intrinsic parameters from CameraInfo (K matrix)
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        # Cache the latest depth image (usually 16-bit unsigned integer in mm)
        self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_callback(self, msg):
        if self.latest_depth_img is None or self.fx is None:
            return

        # Convert ROS image to OpenCV BGR, then to RGB for MediaPipe
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        h, w, _ = cv_image.shape

        results = self.hands.process(rgb_image)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # 1. Select the palm perimeter landmarks
                # Landmark 0: Wrist, 5: Index MCP, 9: Middle MCP, 17: Pinky MCP
                palm_indices = [0, 5, 9, 17]
                
                u_list = []
                v_list = []
                for idx in palm_indices:
                    lm = hand_landmarks.landmark[idx]
                    u_list.append(int(lm.x * w))
                    v_list.append(int(lm.y * h))

                # 2. Compute pixel center of the palm
                u_center = int(np.mean(u_list))
                v_center = int(np.mean(v_list))

                # Prevent out-of-bounds array access
                if 0 <= u_center < w and 0 <= v_center < h:
                    # Get depth value (typically in millimeters for RealSense)
                    depth_z_mm = self.latest_depth_img[v_center, u_center]
                    
                    if depth_z_mm == 0:
                        # Fallback: if center pixel is invalid/blank, check immediate neighbors
                        roi = self.latest_depth_img[max(0, v_center-2):min(h, v_center+3), 
                                                    max(0, u_center-2):min(w, u_center+3)]
                        valid_depths = roi[roi > 0]
                        if len(valid_depths) > 0:
                            depth_z_mm = np.median(valid_depths)
                        else:
                            continue # Ignore frame if sensor depth is missing completely here

                    # Convert depth to meters
                    z_m = float(depth_z_mm) / 1000.0

                    # 3. Complete Pin-hole De-projection to get 3D X and Y coordinates
                    x_m = (u_center - self.cx) * z_m / self.fx
                    y_m = (v_center - self.cy) * z_m / self.fy

                    # 4. Publish PointStamped
                    point_msg = PointStamped()
                    point_msg.header = msg.header  # Keeps timestamp and 'camera_color_optical_frame'
                    point_msg.point.x = x_m
                    point_msg.point.y = y_m
                    point_msg.point.z = z_m
                    
                    self.palm_pub.publish(point_msg)

def main():
    rclpy.init()
    node = PalmTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
