import numpy as np
import torch
import cv2
from os.path import join

# ===== LOAD SAM MODEL =====
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
# from sam_functions import *
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


# ===== Main loop =====
if __name__ == "__main__":
    checkpoint = "/root/models/sam2/checkpoints/sam2.1_hiera_large.pt"
    model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"
    # Attempt to load model
    predictor = SAM2ImagePredictor(build_sam2(model_cfg, checkpoint))
    # Load image into model
    array = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8)
    predictor.set_image(array)
    print(f"Image loaded into model")
            
    # Run model inference
    try:
        boxes = np.array([[100,100,200,200]], dtype=np.float32)
        labels = np.ones(len(boxes), dtype=np.int32)
        masks, scores, _ = predictor.predict(
            box=boxes[None,:],
            multimask_output=False
        )
        print("Prediction returned.")
    except:
        print("Something went wrong\n")

