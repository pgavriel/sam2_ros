#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sam2_msgs.srv import Segment
from sam2_msgs.msg import BoundingBox
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import torch
import numpy as np
import os
import sys

SAM2_DIR = '/root/models/sam2' # Edit to SAM2 clone location
# SAM2_DIR = '/home/csrobot/Perception_Pipeline/SAM/sam2'

class SegmentService(Node):
    def __init__(self):
        super().__init__('segment_service')
        self.bridge = CvBridge()
        self.load_model()
        self.srv = self.create_service(Segment, 'segment', self.segment_callback)
        self.get_logger().info(f"SAM2 Service node created...")

    def load_model(self):
        self.get_logger().info(f"Loading SAM2 Model...")
        # Get the absolute path of the directory where the current script is located
        current_directory = os.path.dirname(os.path.abspath(__file__))
        # Add the current directory to the first position of sys.path
        if current_directory not in sys.path:
            sys.path.insert(0, current_directory)
        if SAM2_DIR not in sys.path:
            sys.path.insert(0, SAM2_DIR)

        # sys.path.append(os.path.dirname(os.path.abspath(__file__)))
        # ===== LOAD SAM MODEL =====
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        # from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
        # from sam_functions import *
        self.DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
        #TODO: Allow initialization with custom weights
        self.checkpoint = os.path.join(SAM2_DIR,"checkpoints/sam2.1_hiera_large.pt")
        self.model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"
        # Attempt to load model
        self.sam2 = build_sam2(self.model_cfg, self.checkpoint)
        self.predictor = SAM2ImagePredictor(self.sam2)
        # self.mask_generator = SAM2AutomaticMaskGenerator(self.sam2)

    def segment_callback(self, request, response):
        self.get_logger().info(f"= = = =\n\n")
        self.get_logger().info(f"Received request with mode={request.mode}")

        with torch.inference_mode(), torch.autocast(self.DEVICE, dtype=torch.bfloat16):
            # Convert ROS Image -> OpenCV (BGR by default)
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')
            self.get_logger().info(f"Image Shape = {cv_image.shape}")
            # Set inference image
            self.predictor.set_image(cv_image)

            # Placeholder masks
            masks = []

            if request.mode == 0:
                self.get_logger().info(f"Segmenting with {len(request.points)} points")
                # Run SAM2 with point prompts
                points = np.array(
                    [[point.x, point.y] for point in request.points],
                    dtype=np.float32
                )
                # NOTE: Assumes all points are positive prompts
                labels = np.ones(len(points), dtype=np.int32)
                # TO USE POINT Z AS LABELS:
                # labels = np.array([point.z for point in request.points], dtype=np.int32)
                masks, scores, _ = self.predictor.predict(
                    point_coords=points,
                    point_labels=labels,
                    multimask_output=False
                )
            elif request.mode == 1:
                self.get_logger().info(f"Segmenting with {len(request.boxes)} boxes")
                # Run SAM2 with box prompts (Expectes [x1, y1, x2, y2] format)
                # Convert to a numpy array of shape (N, 4), dtype float32
                boxes = np.array(
                    [[box.x_min, box.y_min, box.x_max, box.y_max] for box in request.boxes],
                    dtype=np.float32
                )
                self.get_logger().info(f"Boxes:\n{boxes}")
                # if len(boxes) > 1:
                for b in boxes:
                    mask, scores, _ = self.predictor.predict(
                        box=b[None,:],
                        multimask_output=False
                    )
                    # self.get_logger().info(f"Returned shape {mask.shape}")
                    for m in mask:
                        masks.append(m)
            elif request.mode == 2:
                self.get_logger().info("Segmenting entire image [NOT IMPLEMENTED]")
                # Run SAM2 on full image
                # h, w = cv_image.shape[:2]
                # box = np.array([[0, 0, w, h]], dtype=np.float32)
                # self.get_logger().info(f"Prompt:\n{box}")
                # masks, scores, _ = self.predictor.predict(
                #     box=box[None,:],
                #     multimask_output=False
                # # )
                # masks = self.mask_generator.generate(cv_image) 
            else:
                self.get_logger().warn("Invalid mode, returning empty mask")


        # Create Output Mask
        self.get_logger().info(f"Compiling {len(masks)} masks...")
        out_mask = np.zeros_like(cv_image)
        for m in masks:
            self.get_logger().info(f" > Shape: {m.shape}")
            out_mask[m.astype(bool)] = (255, 255, 255)  # White mask
        # Convert to ROS Image Message
        image_message = self.bridge.cv2_to_imgmsg(out_mask, encoding="passthrough")
        response.mask = image_message
        return response

def main():
    rclpy.init()
    node = SegmentService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
