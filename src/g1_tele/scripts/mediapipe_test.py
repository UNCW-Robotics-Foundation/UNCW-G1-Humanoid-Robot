#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class MediaPipeRosNode(Node):
    def __init__(self):
        super().__init__('mediapipe_ros_node')
        
        # Subscriber for raw input camera stream
        self.subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        
        # Publisher for the annotated tracking image
        self.image_pub = self.create_publisher(
            Image, '/camera/image_tracked', 10)
            
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Solutions & Drawing utilities
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

    def image_callback(self, msg):
        # 1. Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 2. MediaPipe expects RGB, OpenCV defaults to BGR
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_image)
        
        # 3. Draw the hand annotations on the original BGR image
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    cv_image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )
        
        # 4. Convert back to ROS message and publish
        tracked_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        # Keep timestamps synchronized with the input stream
        tracked_msg.header = msg.header 
        
        self.image_pub.publish(tracked_msg)

def main():
    rclpy.init()
    node = MediaPipeRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
