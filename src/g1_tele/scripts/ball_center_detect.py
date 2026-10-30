#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2

class HybridSphereDetector(Node):
    def __init__(self):
        super().__init__('hybrid_sphere_detector')
        
        self.cmd_subscription = self.create_subscription(
            Point, '/demo/estimate_cmd', self.cmd_callback, 10)

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points', 
            self.cloud_callback,
            10)
        
        self.ball_pub = self.create_publisher(PointStamped, '/demo/estimate_response', 10)
        self.get_logger().info("Hybrid Node Started.")

        # Fixed OpenCV HSV Teal mapping bounds
        self.lower_color = np.array([80, 100, 100])
        self.upper_color = np.array([110, 255, 255])

        self.estimate_flag = False

    def cmd_callback(self, msg: Point):
        if (msg.x == 1.0):
            self.estimate_flag = True
        if (msg.x == 0.0):
            self.estimate_flag = False

    def fit_sphere_ransac(self, pts, max_iters=150, thresh=0.008):
        num_pts = len(pts)
        if num_pts < 15:
            return None, None

        best_center = None
        best_radius = None
        max_inliers = -1

        for _ in range(max_iters):
            idx = np.random.choice(num_pts, 15, replace=False)
            sample = pts[idx]

            A = np.zeros((15, 4))
            A[:, 0] = sample[:, 0] * 2
            A[:, 1] = sample[:, 1] * 2
            A[:, 2] = sample[:, 2] * 2
            A[:, 3] = 1.0
            
            B = np.sum(sample**2, axis=1)

            try:
                sol, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
                center = sol[0:3]
                r_squared = np.sum(center**2) + sol[3]
                if r_squared <= 0: continue
                radius = np.sqrt(r_squared)
            except (np.linalg.LinAlgError, ValueError):
                continue

            distances = np.abs(np.linalg.norm(pts - center, axis=1) - radius)
            inliers_count = np.sum(distances < thresh)

            if inliers_count > max_inliers:
                max_inliers = inliers_count
                best_center = center
                best_radius = radius

        return best_center, best_radius

    def cloud_callback(self, msg):
        if (not self.estimate_flag):
            return
        # 1. Read unstructured point cloud
        flat_arr = pc2.read_points_numpy(msg, field_names=("x", "y", "z", "rgb"), skip_nans=False)
        
        try:
            cloud_3d = flat_arr.reshape((msg.height, msg.width, 4))
        except ValueError:
            return
        
        x_coords = cloud_3d[:, :, 0]
        y_coords = cloud_3d[:, :, 1]
        z_coords = cloud_3d[:, :, 2]
        rgb_raw  = cloud_3d[:, :, 3]

        # 2. Extract color layout
        rgb_packed = rgb_raw.view(np.uint32)
        r = ((rgb_packed >> 16) & 0xFF).astype(np.uint8)
        g = ((rgb_packed >> 8) & 0xFF).astype(np.uint8)
        b = (rgb_packed & 0xFF).astype(np.uint8)
        
        hsv_img = cv2.cvtColor(cv2.merge([r, g, b]), cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv_img, self.lower_color, self.upper_color)
        
        # FIX: Shave off 2 pixels from the border to eliminate background table noise bleeding
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        
        v_indices, u_indices = np.where(mask > 0)
        self.get_logger().info(f"Matching pixels (eroded): {len(v_indices)}", throttle_duration_sec=2.0)
        
        if len(v_indices) < 30: return

        # 3. Gather coordinates
        pts_x = x_coords[v_indices, u_indices]
        pts_y = y_coords[v_indices, u_indices]
        pts_z = z_coords[v_indices, u_indices]
        
        pts_3d = np.stack((pts_x, pts_y, pts_z), axis=-1)
        pts_3d = pts_3d[~np.isnan(pts_3d).any(axis=1)]
        
        if len(pts_3d) < 30: return

        # 4. Compute original RANSAC math
        center, radius = self.fit_sphere_ransac(pts_3d)
        
        if center is not None:
            if 0.025 <= radius <= 0.045:
                cx, cy, cz = map(float, center)
                point_msg = PointStamped(header=msg.header)
                point_msg.point.x = cx
                point_msg.point.y = cy
                point_msg.point.z = cz
                
                self.ball_pub.publish(point_msg)
                self.get_logger().info(f"🎯 Center: {cx:.3f}, {cy:.3f}, {cz:.3f} | R: {radius*100:.1f}cm")
            else:
                # UPDATED: See exactly what size your data is resolving to
                self.get_logger().warn(
                    f"⚠️ Shape rejected by bounds filter. Radius calculated: {radius*100:.1f}cm", 
                    throttle_duration_sec=2.0
                )

def main(args=None):
    rclpy.init(args=args)
    node = HybridSphereDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()