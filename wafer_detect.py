#!/usr/bin/env python3
"""
Standalone wafer detection tester.
Usage:
    python3 wafer_detect.py <image_path> [--slots N] [--threshold T]

Draw a bounding box over one tray with the mouse, then press ENTER.
The script warps the tray flat and runs brightness-based slot detection.
Press 'r' to redraw, 'q' to quit, +/- to adjust threshold live.
"""

import cv2
import numpy as np
import argparse
import sys

NUM_SLOTS = 25
THRESHOLD = 80          # starting brightness threshold
SAMPLE_RADIUS = 8       # px radius for each slot sample


def colorize_depth(depth):
    valid = depth[depth > 0]
    if len(valid) == 0:
        return np.zeros((*depth.shape, 3), dtype=np.uint8)
    d_min, d_max = np.percentile(valid, 1), np.percentile(valid, 99)
    norm = np.clip((depth.astype(float) - d_min) / max(d_max - d_min, 1) * 255, 0, 255).astype(np.uint8)
    norm[depth == 0] = 0
    colored = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
    colored[depth == 0] = 0
    return colored


def warp_roi(img, box):
    """Perspective-warp the region inside box (x1,y1,x2,y2) to a flat rectangle."""
    x1, y1, x2, y2 = box
    src = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)
    w, h = x2 - x1, y2 - y1
    dst = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (w, h))
    return warped, M


def detect_wafers(warped, num_slots, threshold):
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY) if warped.ndim == 3 else warped
    h, w = gray.shape
    r = SAMPLE_RADIUS

    brightnesses = []
    centers = []
    for i in range(1, num_slots + 1):
        t = i / (num_slots + 1)
        cx, cy = w // 2, int(t * h)
        centers.append((cx, cy))
        x1 = max(0, cx - r); x2 = min(w, cx + r)
        y1 = max(0, cy - r); y2 = min(h, cy + r)
        brightnesses.append(float(np.mean(gray[y1:y2, x1:x2])))

    # adaptive: occupancy = brighter than background * ratio AND below hard threshold
    background = np.percentile(brightnesses, 75)
    adaptive_thresh = background * 0.65
    effective_thresh = min(threshold, adaptive_thresh)

    occupied = [b < effective_thresh for b in brightnesses]
    return occupied, brightnesses, centers, effective_thresh, background


def draw_results(warped, occupied, brightnesses, centers, effective_thresh, background, num_slots):
    vis = warped.copy()
    r = SAMPLE_RADIUS
    for i, (occ, bright, (cx, cy)) in enumerate(zip(occupied, brightnesses, centers), 1):
        color = (0, 0, 255) if occ else (0, 200, 0)
        cv2.circle(vis, (cx, cy), r, color, 2)
        cv2.putText(vis, str(i), (cx - 8, cy - r - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
        cv2.putText(vis, f"{bright:.0f}", (cx - 10, cy + r + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)

    info = f"bg={background:.0f}  thresh={effective_thresh:.0f}  occupied={sum(occupied)}"
    cv2.putText(vis, info, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
    return vis


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", help="Path to image file")
    parser.add_argument("--slots", type=int, default=NUM_SLOTS)
    parser.add_argument("--threshold", type=int, default=THRESHOLD)
    args = parser.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        print(f"Could not load {args.image}")
        sys.exit(1)

    threshold = args.threshold
    num_slots = args.slots
    box = None
    warped = None

    print("Draw a box over the tray with the mouse, then press ENTER.")
    print("Keys: r=redraw  +/-=threshold  q=quit")

    while True:
        # Select ROI
        box = cv2.selectROI("Select tray", img, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("Select tray")
        if box[2] == 0 or box[3] == 0:
            print("No selection, quitting.")
            break

        x, y, bw, bh = box
        bbox = (x, y, x + bw, y + bh)
        warped, M = warp_roi(img, bbox)
        cv2.imwrite("debug_warped.jpg", warped)

        while True:
            occupied, brightnesses, centers, eff_thresh, bg = detect_wafers(warped, num_slots, threshold)
            vis = draw_results(warped, occupied, brightnesses, centers, eff_thresh, bg, num_slots)
            cv2.imwrite("debug_wafer_detect.jpg", vis)
            cv2.imshow("Wafer Detection", vis)
            key = cv2.waitKey(0) & 0xFF

            if key == ord('q'):
                cv2.destroyAllWindows()
                return
            elif key == ord('r'):
                cv2.destroyAllWindows()
                break
            elif key == ord('+') or key == ord('='):
                threshold += 5
                print(f"threshold -> {threshold}")
            elif key == ord('-'):
                threshold = max(5, threshold - 5)
                print(f"threshold -> {threshold}")
            elif key == 13:  # ENTER — print occupied slots
                slots = [i for i, o in enumerate(occupied, 1) if o]
                print(f"Occupied slots: {slots}")
                print(f"Brightness values: {[f'{b:.1f}' for b in brightnesses]}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
