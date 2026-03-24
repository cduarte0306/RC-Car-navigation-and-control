import os
import json
import numpy as np
import cv2
import vpi
import time
import matplotlib.pyplot as plt


def detectLines(frames):
    org = frames.copy()

    # ROI points
    p1 = (300,  330)
    p2 = (1600, 330)
    p3 = (0,    680)
    p4 = (1920, 680)
    cv2.circle(org, p1, 10, (0, 0, 255), -1)
    cv2.circle(org, p2, 10, (0, 255, 0), -1)
    cv2.circle(org, p3, 10, (255, 0, 0), -1)
    cv2.circle(org, p4, 10, (255, 255, 0), -1)

    # Trackbar params
    canny_lo  = cv2.getTrackbarPos("Canny Low",  "Controls")
    canny_hi  = cv2.getTrackbarPos("Canny High", "Controls")
    threshold = cv2.getTrackbarPos("Threshold",  "Controls")
    min_len   = cv2.getTrackbarPos("Min Length", "Controls")
    max_gap   = cv2.getTrackbarPos("Max Gap",    "Controls")
    ksize     = cv2.getTrackbarPos("Blur Size",  "Controls")
    sigma     = cv2.getTrackbarPos("Blur Sigma", "Controls")
    contour_area = cv2.getTrackbarPos("Contour Area", "Controls")

    # HSV + blur
    hsv = cv2.cvtColor(frames, cv2.COLOR_BGR2HSV)
    k = max(1, ksize) * 2 + 1
    blur = cv2.GaussianBlur(hsv, (k, k), max(sigma, 1))

    # --- Color segmentation map ---
    # Sidewalk: low hue, low saturation, high value
    lower_sidewalk = np.array([5, 0, 150])
    upper_sidewalk = np.array([30, 50, 255])
    sidewalk_mask = cv2.inRange(blur, lower_sidewalk, upper_sidewalk)

    # Grass: green hue, moderate+ saturation
    grass_mask = ((blur[:,:,0] > 25) & (blur[:,:,0] < 85) & (blur[:,:,1] > 40)).astype(np.uint8) * 255

    # Build segmentation image
    segmented = np.zeros_like(frames)
    # segmented[sidewalk_mask > 0] = (0, 0, 255)   # red = sidewalk
    segmented[grass_mask > 0]    = (0, 255, 0)    # green = grass
    
    # Invert segmented
    segmented = cv2.bitwise_not(segmented)
    remaining = cv2.bitwise_not(cv2.bitwise_or(sidewalk_mask, grass_mask))

    # segmented[remaining > 0]     = (255, 0, 0)    # blue = unknown
    
    gray_segmented = cv2.cvtColor(segmented, cv2.COLOR_BGR2GRAY)
    # gray_segmented[sidewalk_mask > 0] = 255

    # --- Edge detection on grayscale, masked to sidewalk only ---
    gray = cv2.cvtColor(frames, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.GaussianBlur(gray, (k, k), max(sigma, 1))
    
    
    gray_masked = cv2.bitwise_and(gray_blur, gray_blur, mask=sidewalk_mask)
    gray_masked[gray_masked > 0] = 255
    
    # Fill the holes
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
    gray_masked = cv2.morphologyEx(gray_masked, cv2.MORPH_CLOSE, kernel)

    # Kill any remaining small black blobs
    contours, _ = cv2.findContours(cv2.bitwise_not(gray_masked),
                                cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) < contour_area:
            cv2.drawContours(gray_masked, [cnt], -1, 255, -1)
    
    # Overlay the gray_masked on top of the original image
    overlay_frame_rgb = np.zeros_like(frames)
    overlay_frame_rgb[gray_masked > 0] = (0, 255, 0)  # White where edges are detected
    # overlay = cv2.addWeighted(org, 0.7, overlay_frame_rgb, 0.3, 0)
    org = cv2.addWeighted(org, 0.7, overlay_frame_rgb, 0.3, 0)

    return org


def main():
    print("Starting lane detection test...")
    # Open video
    cv2.namedWindow("Controls", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Controls", 2400, 1200)
    cv2.createTrackbar("Canny Low",  "Controls", 30,  500, lambda x: None)
    cv2.createTrackbar("Canny High", "Controls", 90,  500, lambda x: None)
    cv2.createTrackbar("Threshold",  "Controls", 20,  200, lambda x: None)
    cv2.createTrackbar("Min Length", "Controls", 40,  300, lambda x: None)
    cv2.createTrackbar("Max Gap",    "Controls", 80,  300, lambda x: None)
    cv2.createTrackbar("Blur Size",  "Controls", 7,   20,  lambda x: None)
    cv2.createTrackbar("Blur Sigma", "Controls", 3,   20,  lambda x: None)
    cv2.createTrackbar("Contour Area", "Controls", 5000, 1000000, lambda x: None)

    cap = cv2.VideoCapture("./tests/lane-detection/video/video.MOV")
    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    # Live histogram setup — 8 horizontal segments
    N_SEGS = 8
    plt.ion()
    fig, axes = plt.subplots(N_SEGS, 1, figsize=(10, 12), sharex=True)
    fig.suptitle("HSV Projection per Segment (top → bottom)")
    width  = 640
    height = 360
    x = np.arange(width)
    seg_lines = []
    for i, ax in enumerate(axes):
        lh, = ax.plot(x, np.zeros(width), color='orange',  label='H', alpha=0.8)
        ls, = ax.plot(x, np.zeros(width), color='cyan',    label='S', alpha=0.8)
        lv, = ax.plot(x, np.zeros(width), color='magenta', label='V', alpha=0.8)
        ax.set_ylabel(f"Seg {i+1}", fontsize=7)
        ax.grid(True)
        ax.tick_params(labelsize=6)
        seg_lines.append((lh, ls, lv))
    axes[0].legend(['H', 'S', 'V (Intensity)'], loc='upper right', fontsize=7)
    axes[-1].set_xlabel("Column Index")
    fig.tight_layout()

    while True:
        # Read frame
        ret, frame = cap.read()
        if not ret:
            # Restart video if it ends
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
        frame = cv2.resize(frame, (640, 360))

        frame_org = frame.copy()

        frame = detectLines(frame)

        # Convert to HSV and update live histograms for each segment
        hsv_frame = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)
        seg_h = height // N_SEGS
        for i, (lh, ls, lv) in enumerate(seg_lines):
            seg = hsv_frame[i*seg_h:(i+1)*seg_h].astype(np.int64)
            proj = np.sum(seg, axis=0)          # shape: (width, 3)
            lh.set_ydata(proj[:, 0])
            ls.set_ydata(proj[:, 1])
            lv.set_ydata(proj[:, 2])
            axes[i].relim()
            axes[i].autoscale_view()
        fig.canvas.draw_idle()
        plt.pause(0.001)

        cv2.imshow("Frame", frame)
        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    # Release video and close windows
    cap.release()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()
