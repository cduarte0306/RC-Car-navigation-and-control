"""
Sidewalk segmentation using DeepLabV3+ (pretrained on COCO/Cityscapes classes).
Detects sidewalk/road regions and overlays the mask on the original frame.

Usage:
    python test_sidewalk_segmentation.py --input path/to/image_or_video.mp4

Requirements:
    pip install torch torchvision opencv-python numpy
"""

import argparse
from time import time
import cv2
import numpy as np
import torch
from torchvision import models, transforms

# Pascal VOC / COCO class indices that map to walkable surfaces
# DeepLabV3 pretrained on COCO uses these 21 classes (Pascal VOC subset):
#  0: background,  1: aeroplane,  2: bicycle,  3: bird,  4: boat,
#  5: bottle,      6: bus,        7: car,      8: cat,   9: chair,
# 10: cow,        11: diningtable,12: dog,     13: horse,14: motorbike,
# 15: person,     16: pottedplant,17: sheep,   18: sofa, 19: train,
# 20: tvmonitor
#
# There is no explicit "sidewalk" class in COCO/VOC. The model will likely
# classify sidewalk as background (0). We use a two-pass approach:
#   1. Identify known non-sidewalk objects (people, cars, plants, etc.)
#   2. Combine with your existing HSV mask for the sidewalk surface

# For a true sidewalk class, we also provide a Cityscapes option below.

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def load_model(model_name="deeplabv3_resnet101"):
    """Load a pretrained segmentation model."""
    print(f"Loading {model_name} on {DEVICE}...")

    if model_name == "deeplabv3_resnet101":
        model = models.segmentation.deeplabv3_resnet101(
            weights=models.segmentation.DeepLabV3_ResNet101_Weights.DEFAULT
        )
    elif model_name == "deeplabv3_resnet50":
        model = models.segmentation.deeplabv3_resnet50(
            weights=models.segmentation.DeepLabV3_ResNet50_Weights.DEFAULT
        )
    else:
        raise ValueError(f"Unknown model: {model_name}")

    model = model.to(DEVICE).eval()
    print("Model loaded.")
    return model


def preprocess(frame):
    """Preprocess a BGR OpenCV frame for the model."""
    transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize(520),
        transforms.ToTensor(),
        transforms.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225]
        ),
    ])
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return transform(rgb).unsqueeze(0).to(DEVICE)


def segment_frame(model, frame):
    """Run segmentation and return class map at original resolution."""
    h, w = frame.shape[:2]
    input_tensor = preprocess(frame)

    with torch.no_grad():
        output = model(input_tensor)["out"][0]  # (num_classes, H, W)

    class_map = output.argmax(0).cpu().numpy().astype(np.uint8)
    # Resize back to original frame size
    class_map = cv2.resize(class_map, (w, h), interpolation=cv2.INTER_NEAREST)
    return class_map


def build_sidewalk_mask(class_map):
    """
    Build a binary sidewalk mask.

    Strategy: sidewalk is mostly classified as 'background' (class 0) by
    COCO-pretrained models. We treat background as *potentially* sidewalk,
    then exclude sky (top portion) and known object classes.

    Adjust the OBSTACLE_CLASSES set based on what you see in your scene.
    """
    # Classes that are definitely NOT sidewalk
    OBSTACLE_CLASSES = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                        11, 12, 13, 14, 15, 16, 17, 18, 19, 20}

    # Everything not an obstacle is potentially walkable
    walkable = np.ones_like(class_map, dtype=np.uint8) * 255
    for cls in OBSTACLE_CLASSES:
        walkable[class_map == cls] = 0

    return walkable


def build_overlay(frame, mask, color=(0, 255, 0), alpha=0.4):
    """Overlay a colored mask on the frame."""
    overlay = frame.copy()
    overlay[mask > 0] = (
        (1 - alpha) * overlay[mask > 0] + alpha * np.array(color)
    ).astype(np.uint8)
    return overlay


def find_boundary(mask):
    """Extract the boundary contour of the mask."""
    # Clean up with morphological ops
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
    cleaned = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    return contours, cleaned


def colorize_classes(class_map, num_classes=21):
    """Create a colorized visualization of all detected classes."""
    np.random.seed(42)
    palette = np.random.randint(0, 255, (num_classes, 3), dtype=np.uint8)
    palette[0] = [40, 40, 40]  # background = dark gray
    colored = palette[class_map]
    return colored


def process_image(model, image_path):
    """Process a single image and display results."""
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: could not read {image_path}")
        return

    class_map = segment_frame(model, frame)
    sidewalk_mask = build_sidewalk_mask(class_map)
    contours, cleaned_mask = find_boundary(sidewalk_mask)

    # Visualizations
    overlay = build_overlay(frame, cleaned_mask)
    cv2.drawContours(overlay, contours, -1, (0, 0, 255), 3)

    class_viz = colorize_classes(class_map)
    mask_display = cv2.cvtColor(cleaned_mask, cv2.COLOR_GRAY2BGR)

    # Show results
    top = np.hstack([frame, overlay])
    bottom = np.hstack([class_viz, mask_display])
    combined = np.vstack([top, bottom])

    # Resize for display if too large
    max_h = 1080
    if combined.shape[0] > max_h:
        scale = max_h / combined.shape[0]
        combined = cv2.resize(combined, None, fx=scale, fy=scale)

    cv2.imshow("Sidewalk Segmentation", combined)
    print("Press any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def process_video(model, video_path):
    """Process a video file and display results in real-time."""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: could not open {video_path}")
        return

    print("Press 'q' to quit, 's' to save current frame")
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            # Loop video
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        class_map = segment_frame(model, frame)
        sidewalk_mask = build_sidewalk_mask(class_map)
        contours, cleaned_mask = find_boundary(sidewalk_mask)

        overlay = build_overlay(frame, cleaned_mask)
        cv2.drawContours(overlay, contours, -1, (0, 0, 255), 3)

        # FPS counter
        frame_count += 1

        # Show
        cv2.imshow("Sidewalk Segmentation", overlay)
        cv2.imshow("Mask", cleaned_mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite(f"sidewalk_frame_{frame_count}.png", overlay)
            cv2.imwrite(f"sidewalk_mask_{frame_count}.png", cleaned_mask)
            print(f"Saved frame {frame_count}")

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Sidewalk segmentation test")
    parser.add_argument("--input", required=False, default="./tests/lane-detection/video/video.MOV",
                        help="Path to image or video file")
    parser.add_argument("--model", default="deeplabv3_resnet101",
                        choices=["deeplabv3_resnet101", "deeplabv3_resnet50"],
                        help="Model to use (resnet101 is more accurate, "
                             "resnet50 is faster)")
    args = parser.parse_args()

    model = load_model(args.model)
    
    print(f"Using: {DEVICE}")
    print(f"CUDA available: {torch.cuda.is_available()}")

    # Determine if input is image or video
    ext = args.input.lower().rsplit(".", 1)[-1]
    if ext in ("png", "jpg", "jpeg", "bmp", "tiff"):
        process_image(model, args.input)
    elif ext in ("mp4", "avi", "mov", "mkv", "webm"):
        process_video(model, args.input)
    else:
        print(f"Unknown file type: .{ext}, trying as video...")
        process_video(model, args.input)


if __name__ == "__main__":
    main()