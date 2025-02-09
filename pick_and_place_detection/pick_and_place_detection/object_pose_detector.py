import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2

from geometry_msgs.msg import PoseArray, Pose

from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np

from message_filters import ApproximateTimeSynchronizer, Subscriber

from ultralytics import YOLO
import torch

from typing import List


class ObjectPoseDetector(Node):
    def __init__(self):
        super().__init__('object_pose_detector_node')
        self.__image_subscription = Subscriber(
            self,
            Image,
            '/depth_camera/image_raw')
        self.__pointcloud_sub = Subscriber(
            self, 
            PointCloud2, 
            '/depth_camera/points')
        self.__object_poses_publisher = self.create_publisher(
            PoseArray,
            '/detected_poses',
            10
        )

        self.__ts = ApproximateTimeSynchronizer(
            fs=[self.__image_subscription, self.__pointcloud_sub],
            queue_size=10,
            slop=0.1)
        self.__ts.registerCallback(self.__listener_callback)
        
        self.__bridge = CvBridge()

        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.__yolo = YOLO(
            "/home/santoshbalaji/dev_ws/src/pick_and_place/pick_and_place_detection/weights/best.pt").to(device)
        self.get_logger().info("started object pose detector node")


    def __listener_callback(self, img_msg : Image, pc_msg : PointCloud2):
        try:
            cv_image = self.__bridge.imgmsg_to_cv2(
                img_msg, 
                desired_encoding='bgr8')
            height, width, _ = cv_image.shape

            records = self.__predict_bounding_boxes(image=cv_image)
            object_poses = self.__get_depth_info(
                pc_msg=pc_msg,
                centroids =records,
                width=width,
                height=height,
            )

            pose_array = PoseArray()
            for object_pose in object_poses:
                self.get_logger().info("object pose: " + str(object_pose))
                pose = Pose()
                pose.position.x = object_pose[0]
                pose.position.y = object_pose[1]
                pose.position.z = object_pose[2]
                pose_array.poses.append(pose)
            
            self.__object_poses_publisher.publish(pose_array)

        except CvBridgeError as e:
            self.get_logger().info(f'CvBridgeError: {e}')


    def __predict_bounding_boxes(self, image : Image) -> List[List[float]]:
        results = self.__yolo(image, conf=0.5)
        height, width, _ = image.shape

        records = list()
        for result in results:
            for box in result.boxes:
                record = list()

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = box.conf[0].item()
                cls = int(box.cls[0].item())

                print(x1, y1)
                print(x2, y2)
                print(((x1 + x2) / 2))
                print(((y1 + y2) / 2))

                x_center = ((x1 + x2) / (2 * width))
                y_center = ((y1 + y2) / (2 * height))
                coordinate_width = ((x2 - x1) / width)
                coordinate_height = ((y2 - y1) / height)

                record.append(x_center)
                record.append(y_center)
                record.append(coordinate_width)
                record.append(coordinate_height)
                record.append(conf)
                record.append(cls)
                record.append(((x1 + x2) / (2)))
                record.append(((y1 + y2) / (2)))

                records.append(record)

                self.get_logger().info("x_center: " + str(record[0]))
                self.get_logger().info("y_center: " + str(record[1]))
                self.get_logger().info("coordinate width: " + str(record[2]))
                self.get_logger().info("coordinate height: " + str(record[3]))
                self.get_logger().info("confidence score: " + str(record[4]))
                self.get_logger().info("class id: " + str(record[5]))
        return records


    def __get_depth_info(self, pc_msg, centroids, width, height):
        final_values = list()

        for centroid in centroids:
            final_value = list()
            cX = centroid[0]
            cY = centroid[1]

            pc_data = point_cloud2.read_points(
                pc_msg,
                field_names=("x", "y", "z"), 
                skip_nans=False)
            print(pc_data.shape)
            for point in pc_data:
                if point[2] < 1.15 and point[2] > 1.00:
                    # print(point[0], point[1], point[2])
                    pass
            
            print(centroid[6])
            print(centroid[7])
            pc_data = point_cloud2.read_points(
                pc_msg,
                field_names=("x", "y", "z"), 
                skip_nans=False,
                uvs=
                    np.array([round(centroid[6]), round(centroid[7])])
                )
            print(pc_data)

            for point in pc_data:
                _, _, z = point
                final_value.append(round(cX, 4))
                final_value.append(round(cY, 4))
                final_value.append(round(float(z), 4))
                print(point)
                # break
            final_values.append(final_value)
        return final_values


def main(args=None):
    rclpy.init(args=args)
    object_pose_detector = ObjectPoseDetector()
    rclpy.spin(object_pose_detector)
    object_pose_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()