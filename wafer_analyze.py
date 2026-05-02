#!/usr/bin/env python3
"""
Deep analysis of slot pixel features to find discriminating features
for wafer vs empty slot detection.
"""

import cv2
import numpy as np

WARPED_PATH = "/home/nano/final_project_ws/iter_warped.jpg"
NUM_SLOTS = 25
SAMPLE_RADIUS = 8

def sample_slot(img_bgr, gray, slot_idx, num_slots, xs_frac):
    """Sample a slot at multiple x positions, return BGR and gray values."""
    h, w = gray.shape
    t = slot_idx / (num_slots + 1)
    cy = int(t * h)
    r = SAMPLE_RADIUS
    results = []
    for xf in xs_frac:
        cx = int(xf * w)
        x1 = max(0, cx - r); x2 = min(w, cx + r)
        y1 = max(0, cy - r); y2 = min(h, cy + r)
        patch_gray = gray[y1:y2, x1:x2]
        patch_bgr  = img_bgr[y1:y2, x1:x2]
        bright = float(np.mean(patch_gray))
        b = float(np.mean(patch_bgr[:,:,0]))
        g_val = float(np.mean(patch_bgr[:,:,1]))
        r_val = float(np.mean(patch_bgr[:,:,2]))
        br_ratio = b / max(r_val, 1.0)
        sat = float(np.mean(cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2HSV)[:,:,1]))
        results.append((bright, br_ratio, sat, b, g_val, r_val))
    return cy, results

def main():
    img = cv2.imread(WARPED_PATH)
    if img is None:
        print(f"Cannot load {WARPED_PATH}")
        return

    # Trim the red border that was drawn by bounding box visualization
    # The image from iter_warped should be clean warped ROI, but check anyway
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    print(f"Warped image size: {w}x{h}")

    # Sample at many x positions across the inner tray (avoid the tray teeth on sides)
    # Teeth are roughly 10-15% on each side
    xs = [0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85]

    # Ground truth: slot 8 (single wafer), slots 19-22 (group of 4)
    ground_truth = {8, 19, 20, 21, 22}

    print(f"\n{'Slot':>4} | {'GT':>2} | ", end="")
    print(" ".join([f"x={x:.2f}(br,B/R)" for x in xs]))
    print("-" * 140)

    all_min_bright = []
    all_max_br = []
    all_min_sat = []

    for slot in range(1, NUM_SLOTS + 1):
        cy, samples = sample_slot(img, gray, slot, NUM_SLOTS, xs)
        gt = "X" if slot in ground_truth else "."

        brights = [s[0] for s in samples]
        brs     = [s[1] for s in samples]
        sats    = [s[2] for s in samples]

        min_b = min(brights)
        max_br = max(brs)
        max_sat = max(sats)

        all_min_bright.append(min_b)
        all_max_br.append(max_br)
        all_min_sat.append(max_sat)

        print(f"{slot:>4} | {gt:>2} | ", end="")
        for bright, br, sat, b, g, r in samples:
            print(f"{bright:5.1f}/{br:.2f}  ", end="")
        print(f"  | minB={min_b:5.1f} maxBR={max_br:.2f} maxSat={max_sat:4.1f}")

    print("\n\n=== FEATURE SUMMARY ===")
    print(f"{'Slot':>4} | {'GT':>2} | minBright | maxBR | maxSat")
    print("-" * 50)
    for slot in range(1, NUM_SLOTS + 1):
        gt = "X" if slot in ground_truth else "."
        print(f"{slot:>4} | {gt:>2} | {all_min_bright[slot-1]:9.1f} | {all_max_br[slot-1]:5.2f} | {all_min_sat[slot-1]:6.1f}")

    # Now try different threshold combinations
    print("\n\n=== THRESHOLD SEARCH ===")
    best_f1 = 0
    best_params = None

    for bright_thresh in range(40, 150, 5):
        for br_thresh in [None, 1.3, 1.4, 1.5, 1.6, 1.7]:
            for sat_thresh in [None, 40, 60, 80, 100]:
                detected = set()
                for slot in range(1, NUM_SLOTS + 1):
                    mb = all_min_bright[slot-1]
                    mbr = all_max_br[slot-1]
                    msat = all_min_sat[slot-1]

                    cond = mb < bright_thresh
                    if br_thresh is not None:
                        cond = cond or (mbr > br_thresh and msat > (sat_thresh or 0))
                    if cond:
                        detected.add(slot)

                tp = len(detected & ground_truth)
                fp = len(detected - ground_truth)
                fn = len(ground_truth - detected)
                if tp + fp > 0 and tp + fn > 0:
                    prec = tp / (tp + fp)
                    rec  = tp / (tp + fn)
                    f1 = 2 * prec * rec / (prec + rec) if (prec + rec) > 0 else 0
                    if f1 > best_f1:
                        best_f1 = f1
                        best_params = (bright_thresh, br_thresh, sat_thresh, detected, tp, fp, fn)

    if best_params:
        bt, brt, st, det, tp, fp, fn = best_params
        print(f"Best F1={best_f1:.3f}: bright<{bt} OR (BR>{brt} AND sat>{st})")
        print(f"  Detected: {sorted(det)}")
        print(f"  TP={tp} FP={fp} FN={fn}")
        print(f"  Ground truth: {sorted(ground_truth)}")

    # Also try: minimum brightness alone (no color)
    print("\n=== BRIGHTNESS-ONLY THRESHOLDS ===")
    for t in range(30, 160, 5):
        det = {s for s in range(1, NUM_SLOTS+1) if all_min_bright[s-1] < t}
        tp = len(det & ground_truth)
        fp = len(det - ground_truth)
        fn = len(ground_truth - det)
        print(f"  thresh={t:3d}: detected={sorted(det)}  TP={tp} FP={fp} FN={fn}")

    # Visual output: draw all slot sample points on warped image
    vis = img.copy()
    for slot in range(1, NUM_SLOTS + 1):
        t = slot / (NUM_SLOTS + 1)
        cy = int(t * h)
        mb = all_min_bright[slot - 1]
        gt = slot in ground_truth

        # Color: red=GT wafer, green=detected by min brightness<80, yellow=FP, white=TN
        det = mb < 80
        if gt and det:
            color = (0, 255, 0)    # TP green
        elif gt and not det:
            color = (0, 0, 255)    # FN red
        elif not gt and det:
            color = (0, 165, 255)  # FP orange
        else:
            color = (200, 200, 200)  # TN gray

        cx = int(0.5 * w)
        cv2.circle(vis, (cx, cy), SAMPLE_RADIUS, color, 2)
        cv2.putText(vis, f"{slot}:{mb:.0f}", (cx + 12, cy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)

    cv2.imwrite("/home/nano/final_project_ws/iter_analysis.jpg", vis)
    print("\nSaved iter_analysis.jpg")

if __name__ == "__main__":
    main()
