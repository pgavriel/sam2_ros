#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

# Replace with your actual service type
from sam2_msgs.srv import Segment
from sam2_msgs.msg import BoundingBox

MODE = {"full": 2,
         "boxes": 1,
         "points": 0}

DELAY=1000

class SegmentClient(Node):
    def __init__(self):
        super().__init__("segment_test_client")
        self.cli = self.create_client(Segment, "/segment")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")

        self.bridge = CvBridge()

    def send_request(self, mode, image_cv, boxes=None, points=None):
        req = Segment.Request()

        # Convert CV image to ROS Image
        req.image = self.bridge.cv2_to_imgmsg(image_cv, encoding="bgr8")

        req.mode = mode  # string or enum depending on your srv definition

        if boxes:
            req.boxes = boxes
        if points:
            req.points = points

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()

    node = SegmentClient()

    # Load test image
    current_directory = os.path.dirname(os.path.abspath(__file__))
    im_name = os.path.join(current_directory,"test_img.png")
    img = cv2.imread(im_name)
    if img is None:
        raise RuntimeError(f"Could not load {im_name} from current folder")

    h, w = img.shape[:2]

    ### 1. Whole image segmentation
    node.get_logger().info("Testing whole-image segmentation")
    res = node.send_request(mode=MODE["full"], image_cv=img)
    mask_cv = node.bridge.imgmsg_to_cv2(res.mask, desired_encoding="8UC3")
    cv2.imshow("Whole Image Mask", mask_cv)
    cv2.waitKey(DELAY)

    ### Bounding box segmentation
    node.get_logger().info("Testing single bounding-box segmentation")
    box = BoundingBox(x_min=w//4, y_min=h//4, x_max=3*w//4, y_max=3*h//4)
    res = node.send_request(mode=MODE["boxes"], image_cv=img, boxes=[box])
    mask_cv = node.bridge.imgmsg_to_cv2(res.mask, desired_encoding="8UC3")
    cv2.imshow("Single Box Mask", mask_cv)
    cv2.waitKey(DELAY)

    ### Multi Bounding box segmentation
    node.get_logger().info("Testing multi bounding-box segmentation")
    box1 = BoundingBox(x_min=337, y_min=156, x_max=365, y_max=179)
    box2= BoundingBox(x_min=286, y_min=230, x_max=307, y_max=261)
    res = node.send_request(mode=MODE["boxes"], image_cv=img, boxes=[box1,box2])
    mask_cv = node.bridge.imgmsg_to_cv2(res.mask, desired_encoding="8UC3")
    cv2.imshow("Multi Box Mask", mask_cv)
    cv2.waitKey(DELAY)

    ### Single Point-based segmentation
    node.get_logger().info("Testing single point-based segmentation")
    pt = Point(x=float(w//2), y=float(h//2), z=1.0)  # use z=1.0 as "positive" label
    res = node.send_request(mode=MODE["points"], image_cv=img, points=[pt])
    mask_cv = node.bridge.imgmsg_to_cv2(res.mask, desired_encoding="8UC3")
    cv2.imshow("Single Point Mask", mask_cv)
    cv2.waitKey(DELAY)

    ### Multi Point-based segmentation
    node.get_logger().info("Testing multi point-based segmentation")
    pt1 = Point(x=float(286), y=float(208), z=1.0)  # use z=1.0 as "positive" label
    pt2 = Point(x=float(375), y=float(250), z=1.0)  # use z=1.0 as "positive" label
    pt3 = Point(x=float(265), y=float(252), z=1.0)  # use z=1.0 as "positive" label
    res = node.send_request(mode=MODE["points"], image_cv=img, points=[pt1,pt2,pt3])
    mask_cv = node.bridge.imgmsg_to_cv2(res.mask, desired_encoding="8UC3")
    cv2.imshow("Multi Point Mask", mask_cv)
    cv2.waitKey(0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
