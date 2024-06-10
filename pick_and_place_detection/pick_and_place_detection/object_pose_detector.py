import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

class ObjectPoseDetector(Node):
    def __init__(self):
        super().__init__('object_pose_detector_node')
        self.image_subscription = Subscriber(
            self,
            Image,
            '/depth_camera/image_raw')
        self.pointcloud_sub = Subscriber(self, PointCloud2, '/depth_camera/points')
        
        self.ts = ApproximateTimeSynchronizer([self.image_subscription, self.pointcloud_sub], 10, 0.1)
        self.ts.registerCallback(self.listener_callback)
        
        self.bridge = CvBridge()

    def listener_callback(self, img_msg, pc_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

            centroids, processed_image = self.process_image(cv_image)

            z_values = self.get_z_values(pc_msg, centroids)

            cv2.imshow('Processed Image', processed_image)
            cv2.waitKey(1)

            for i, (x, y, z) in enumerate(z_values):
                self.get_logger().info(f'Centroid {i}: ({x}, {y}, {z})')

        except CvBridgeError as e:
            self.get_logger().info(f'CvBridgeError: {e}')

    def process_image(self, image):
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        lower_color = np.array([5, 250, 5])
        upper_color = np.array([5, 255, 5])
        mask = cv2.inRange(rgb, lower_color, upper_color)

        segmented_image = cv2.bitwise_and(image, image, mask=mask)

        gray = cv2.cvtColor(segmented_image, cv2.COLOR_BGR2GRAY)

        contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        centroids = []

        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX = 0
                cY = 0

            centroids.append((cX, cY))

            cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
            cv2.circle(image, (cX, cY), 5, (255, 0, 0), -1)
            cv2.putText(image, f"({cX}, {cY})", (cX - 20, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        return centroids, image

    def get_z_values(self, pc_msg, centroids):
        z_values = []

        # Read points from the point cloud message
        pc_data = list(point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True))

        if len(pc_data) == 0:
            self.get_logger().info('No points in the point cloud')
            return z_values

        # Convert list of tuples to 2D numpy array
        pc_array = np.array(pc_data)

        if pc_array.ndim == 1:
            self.get_logger().info('Point cloud array is 1-dimensional, converting to 2-dimensional')
            pc_array = np.array([list(point) for point in pc_data])

        self.get_logger().info(f'Point cloud array shape: {pc_array.shape}')

        for cX, cY in centroids:
            # Find the closest point in the point cloud to the centroid
            distances = np.linalg.norm(pc_array[:, :2] - np.array([cX, cY]), axis=1)
            closest_idx = np.argmin(distances)
            closest_point = pc_array[closest_idx]

            x, y, z = closest_point
            z_values.append((cX, cY, z))

        return z_values

def main(args=None):
    rclpy.init(args=args)
    object_pose_detector = ObjectPoseDetector()
    rclpy.spin(object_pose_detector)
    object_pose_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()