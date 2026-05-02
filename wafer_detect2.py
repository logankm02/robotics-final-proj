#!/usr/bin/env python3
"""
Headless wafer detection - iterative version.
Uses known bounding box from Grounding DINO, warps tray, detects occupied slots
using a dual-rule approach:
  Rule 1: min_brightness < 50  → wafer group (very dark silicon)
  Rule 2: hue_at_x45 in (70,100) AND min_brightness < 90  → single wafer (color discriminator)

Ground truth for debug_contour.jpg: slots 8 (single) and 19,20,21,22 (group)
Bounding box: x1=1118, y1=324, x2=1485, y2=727
"""

import cv2
import numpy as np

IMAGE_PATH = "/home/nano/final_project_ws/debug_contour.jpg"
BBOX = (1118, 324, 1485, 727)   # x1, y1, x2, y2 from previous GSAM detection
NUM_SLOTS = 25

# Thresholds (tunable)
DARK_THRESH = 50        # min brightness → definite group wafer
SINGLE_THRESH = 90      # min brightness upper bound for single-wafer check
HUE_LOW = 70            # silicon blue-ish hue lower bound (OpenCV: 0-180)
HUE_HIGH = 100          # upper bound (excludes tray wall artifact)

SAMPLE_RADIUS = 12
# x positions (fraction of tray width) for brightness scan — covers center interior
XS_BRIGHT = [0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70]
X_HUE = 0.45            # single hue sample position


def warp_roi(img, box):
    x1, y1, x2, y2 = box
    src = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)
    w, h = x2 - x1, y2 - y1
    dst = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(img, M, (w, h)), M


def sample_patch(img, cy, cx, r):
    h, w = img.shape[:2]
    y1 = max(0, cy - r); y2 = min(h, cy + r)
    x1 = max(0, cx - r); x2 = min(w, cx + r)
    return img[y1:y2, x1:x2]


def detect_wafers(warped):
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    hsv  = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
    h, w = gray.shape
    r = SAMPLE_RADIUS

    occupied = []
    details  = []

    for slot in range(1, NUM_SLOTS + 1):
        t = slot / (NUM_SLOTS + 1)
        cy = int(t * h)

        # Minimum brightness across multiple x positions
        brights = []
        for xf in XS_BRIGHT:
            cx = int(xf * w)
            pg = sample_patch(gray, cy, cx, r)
            if pg.size > 0:
                brights.append(float(np.mean(pg)))
        min_b = min(brights) if brights else 255.0

        # Hue at fixed x=0.45 (discriminates silicon blue vs wood yellow)
        cx_hue = int(X_HUE * w)
        ph = sample_patch(hsv, cy, cx_hue, r)
        hue_val = float(np.mean(ph[:, :, 0])) if ph.size > 0 else 0.0

        # Rule 1: very dark → group wafer
        rule1 = min_b < DARK_THRESH
        # Rule 2: blue-ish hue + moderately dark → single wafer
        rule2 = (HUE_LOW < hue_val < HUE_HIGH) and (min_b < SINGLE_THRESH)

        occ = rule1 or rule2
        occupied.append(occ)
        details.append({'slot': slot, 'min_b': min_b, 'hue': hue_val,
                        'rule1': rule1, 'rule2': rule2})

    return occupied, details


def draw_results(warped, occupied, details):
    vis = warped.copy()
    h, w = vis.shape[:2]
    r = SAMPLE_RADIUS

    for occ, d in zip(occupied, details):
        slot = d['slot']
        t = slot / (NUM_SLOTS + 1)
        cy = int(t * h)
        cx_center = int(0.45 * w)

        if occ:
            color = (0, 0, 255)   # red = occupied
            rule = "G" if d['rule1'] else "S"
        else:
            color = (0, 200, 0)   # green = empty
            rule = ""

        cv2.circle(vis, (cx_center, cy), r, color, 2)
        cv2.putText(vis, f"{slot}{rule}", (cx_center + r + 2, cy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)
        cv2.putText(vis, f"b={d['min_b']:.0f} h={d['hue']:.0f}",
                    (cx_center + r + 2, cy + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, (200, 200, 200), 1)

    n_occ = sum(occupied)
    cv2.putText(vis, f"occupied={n_occ}", (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
    return vis


def main():
    img = cv2.imread(IMAGE_PATH)
    if img is None:
        print(f"Cannot load {IMAGE_PATH}")
        return

    warped, M = warp_roi(img, BBOX)
    cv2.imwrite("/home/nano/final_project_ws/iter2_warped.jpg", warped)

    occupied, details = detect_wafers(warped)
    vis = draw_results(warped, occupied, details)
    cv2.imwrite("/home/nano/final_project_ws/iter2_result.jpg", vis)

    detected = [d['slot'] for d, o in zip(details, occupied) if o]
    ground_truth = [8, 19, 20, 21, 22]

    print("=" * 60)
    print(f"Detected:     {detected}")
    print(f"Ground truth: {ground_truth}")
    tp = set(detected) & set(ground_truth)
    fp = set(detected) - set(ground_truth)
    fn = set(ground_truth) - set(detected)
    print(f"TP={sorted(tp)}  FP={sorted(fp)}  FN={sorted(fn)}")
    prec = len(tp) / (len(tp) + len(fp)) if (len(tp) + len(fp)) > 0 else 0
    rec  = len(tp) / (len(tp) + len(fn)) if (len(tp) + len(fn)) > 0 else 0
    f1   = 2 * prec * rec / (prec + rec) if (prec + rec) > 0 else 0
    print(f"Precision={prec:.3f}  Recall={rec:.3f}  F1={f1:.3f}")
    print("=" * 60)

    print("\nPer-slot detail:")
    print(f"{'Slot':>4} {'GT':>2} {'Det':>4} {'Rule':>5} {'minBr':>6} {'Hue':>5}")
    for d, o in zip(details, occupied):
        gt = "X" if d['slot'] in ground_truth else "."
        det = "X" if o else "."
        rule = ("G" if d['rule1'] else "S") if o else "-"
        print(f"{d['slot']:>4}  {gt}   {det}  {rule:>5}  {d['min_b']:>6.1f}  {d['hue']:>5.1f}")


if __name__ == "__main__":
    main()
