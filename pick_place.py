#!/usr/bin/env python3
"""
pick_place.py - Compute pick/place positions for wafer robot arm.

Tray detection: Grounding DINO (IDEA-Research/grounding-dino-tiny) with text
  prompt "colored box." — mirrors gsam_slide_detect.py.

Pick  = first occupied wafer slot in source tray (from top)
Place = first empty slot in destination tray (from top)

Output: pick_place_result.jpg with crosshair markers + pixel coords printed.
For ROS migration: feed pixel coords into deproject_pixel_to_3d().

Run: python3 pick_place.py [image_path]
"""

import sys
import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

IMAGE_PATH = "debug_contour.jpg"
NUM_SLOTS   = 25
INTERIOR_X  = (0.12, 0.88)
BAND_RADIUS = 12
X_HUE       = 0.45
SLOT_Y_PAD       = 0.0    # detection sampling: no pad keeps hue samples on correct slots
SLOT_Y_DISP_PAD  = 0.08   # visual circle display: inset 5 % from bbox top/bottom (frame lip)

# Wafer detection thresholds (from wafer_detect3.py — F1=1.0)
DARK_ABS_THRESH = 42
SINGLE_THRESH   = 90
HUE_LOW_CV2     = 70
HUE_HIGH_CV2    = 100

GROUNDING_MODEL = "IDEA-Research/grounding-dino-tiny"
TEXT_PROMPT     = "plastic tray."
BOX_THRESHOLD   = 0.3


# ---------------------------------------------------------------------------
# Grounding DINO tray detection (mirrors gsam_slide_detect.py)
# ---------------------------------------------------------------------------

def detect_trays_gdino(image_path: str):
    """
    Run Grounding DINO on the image and return all detected bounding boxes.
    Returns list of (x1, y1, x2, y2) tuples sorted left→right.
    """
    import torch
    from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

    device = "cuda" if torch.cuda.is_available() else "cpu"

    processor = AutoProcessor.from_pretrained(GROUNDING_MODEL)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(GROUNDING_MODEL).to(device)

    pil_img = Image.open(image_path).convert("RGB")

    inputs = processor(images=pil_img, text=TEXT_PROMPT, return_tensors="pt").to(device)
    with torch.no_grad():
        outputs = model(**inputs)

    results = processor.post_process_grounded_object_detection(
        outputs,
        inputs.input_ids,
        text_threshold=BOX_THRESHOLD,
        target_sizes=[pil_img.size[::-1]],
    )

    boxes = results[0]["boxes"].cpu().numpy()
    scores = results[0]["scores"].cpu().numpy()

    print(f"Grounding DINO detected {len(boxes)} box(es):")
    for i, (box, score) in enumerate(zip(boxes, scores)):
        print(f"  [{i}] score={score:.3f}  bbox=({int(box[0])},{int(box[1])},{int(box[2])},{int(box[3])})")

    # Convert to int tuples, sort left→right
    bboxes = [(int(b[0]), int(b[1]), int(b[2]), int(b[3])) for b in boxes]
    return sorted(bboxes, key=lambda b: b[0])


# ---------------------------------------------------------------------------
# Wafer detection (wafer_detect3.py — Rule 1 + Rule 2)
# ---------------------------------------------------------------------------

def _hue_at_patch(roi_rgb, cy, cx, r, h, w):
    patch = roi_rgb[max(0, cy-r):min(h, cy+r), max(0, cx-r):min(w, cx+r)]
    if patch.size == 0:
        return 0.0
    rp = patch[:,:,0].astype(np.float32)/255; gp = patch[:,:,1].astype(np.float32)/255
    bp = patch[:,:,2].astype(np.float32)/255
    maxc = np.maximum(rp, np.maximum(gp, bp)); minc = np.minimum(rp, np.minimum(gp, bp))
    diff = maxc - minc; hue = np.zeros_like(rp)
    mr = (maxc == rp) & (diff > 0); mg = (maxc == gp) & (diff > 0); mb = (maxc == bp) & (diff > 0)
    hue[mr] = (60.0 * ((gp[mr]-bp[mr])/diff[mr])) % 360
    hue[mg] = 60.0 * ((bp[mg]-rp[mg])/diff[mg]) + 120
    hue[mb] = 60.0 * ((rp[mb]-gp[mb])/diff[mb]) + 240
    return float(np.mean(hue)) / 2.0


def detect_wafers(arr: np.ndarray, bbox: tuple) -> list:
    """Return per-slot occupancy list for tray at bbox."""
    x1, y1, x2, y2 = bbox
    roi_rgb  = arr[y1:y2, x1:x2]
    h, w = roi_rgb.shape[:2]
    roi_gray = np.mean(roi_rgb, axis=2).astype(np.float32)
    xi_s = int(INTERIOR_X[0] * w)
    xi_e = int(INTERIOR_X[1] * w)

    occupied = []
    for slot in range(1, NUM_SLOTS + 1):
        t = slot / (NUM_SLOTS + 1)
        cy = int(t * h)
        y1_b = max(0, cy - BAND_RADIUS); y2_b = min(h, cy + BAND_RADIUS)

        band_gray = roi_gray[y1_b:y2_b, xi_s:xi_e]
        if band_gray.size == 0:
            occupied.append(False); continue
        col_means = np.mean(band_gray, axis=0)
        min_b = float(np.min(col_means))

        cx_hue = int(X_HUE * w)
        hue = _hue_at_patch(roi_rgb, cy, cx_hue, BAND_RADIUS, h, w)

        rule1 = min_b < DARK_ABS_THRESH
        rule2 = (HUE_LOW_CV2 < hue < HUE_HIGH_CV2) and (min_b < SINGLE_THRESH)
        occupied.append(rule1 or rule2)

    return occupied


# ---------------------------------------------------------------------------
# Tray angle (mirrors gsam_slide_detect.py minAreaRect approach)
# ---------------------------------------------------------------------------

def tray_angle(arr: np.ndarray, bbox: tuple) -> float:
    """
    Estimate tray rotation angle in degrees from vertical.
    Positive = clockwise tilt. 0 = perfectly upright.
    In ROS: cv2.minAreaRect on SAM2 mask gives this directly.
    Standalone: Sobel edges + linear regression on tray ROI.
    """
    x1, y1, x2, y2 = bbox
    roi = arr[y1:y2, x1:x2]
    gray = np.mean(roi, axis=2).astype(np.float32)

    sx = ndimage.sobel(gray, axis=1).astype(np.float32)
    sy = ndimage.sobel(gray, axis=0).astype(np.float32)
    mag = np.hypot(sx, sy)

    thresh = np.percentile(mag, 92)
    ys, xs = np.where(mag > thresh)
    if len(xs) < 20:
        return 0.0

    # Fit x = a*y + b — tray edges are near-vertical so y is the independent var
    A = np.vstack([ys, np.ones(len(ys))]).T
    slope = float(np.linalg.lstsq(A, xs, rcond=None)[0][0])
    return float(np.degrees(np.arctan(slope)))


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def slot_pixel(bbox: tuple, slot: int):
    """Center pixel of a slot in original image coordinates."""
    x1, y1, x2, y2 = bbox
    h = y2 - y1
    pad = int(SLOT_Y_DISP_PAD * h)
    # Slot 1 at top inset, slot 25 at bottom inset, evenly spaced between
    t = (slot - 1) / (NUM_SLOTS - 1)
    cy = int(y1 + pad + t * (h - 2 * pad))
    cx = (x1 + x2) // 2
    return cx, cy


def draw_marker(draw: ImageDraw.Draw, cx: int, cy: int,
                color: tuple, label: str, size: int = 18):
    """Crosshair + circle marker."""
    draw.ellipse([cx-size, cy-size, cx+size, cy+size], outline=color, width=3)
    draw.ellipse([cx-4, cy-4, cx+4, cy+4], fill=color)
    draw.line([cx - size - 8, cy, cx + size + 8, cy], fill=color, width=2)
    draw.line([cx, cy - size - 8, cx, cy + size + 8], fill=color, width=2)
    draw.text((cx + size + 8, cy - 8), label, fill=color)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    image_path = sys.argv[1] if len(sys.argv) > 1 else IMAGE_PATH

    img = Image.open(image_path).convert("RGB")
    arr = np.array(img)
    print(f"Image: {image_path}  size: {arr.shape[1]}x{arr.shape[0]}")

    # ---- Detect trays with Grounding DINO ----
    bboxes = detect_trays_gdino(image_path)

    if len(bboxes) < 2:
        print(f"WARNING: expected 2 trays, got {len(bboxes)}. Check model weights / threshold.")
        if len(bboxes) == 0:
            print("Falling back to hard-coded boxes.")
            bboxes = [(510, 232, 840, 578), (1118, 324, 1486, 727)]

    # Identify source (has wafers) vs dest (empty) by checking wafer count
    occ_counts = [sum(detect_wafers(arr, b)) for b in bboxes]
    src_idx  = int(np.argmax(occ_counts))
    dest_idx = 1 - src_idx if len(bboxes) == 2 else int(np.argmin(occ_counts))

    src_bbox  = bboxes[src_idx]
    dest_bbox = bboxes[dest_idx]
    print(f"Source tray (idx {src_idx}, {occ_counts[src_idx]} wafers): {src_bbox}")
    print(f"Dest   tray (idx {dest_idx}, {occ_counts[dest_idx]} wafers): {dest_bbox}")

    src_occ  = detect_wafers(arr, src_bbox)
    dest_occ = detect_wafers(arr, dest_bbox)

    src_filled  = [s + 1 for s, o in enumerate(src_occ)  if o]
    dest_empty  = [s + 1 for s, o in enumerate(dest_occ) if not o]

    print(f"\nSource occupied slots: {src_filled}")
    print(f"Dest   empty   slots: {dest_empty}")

    if not src_filled:
        print("No wafers to pick."); return
    if not dest_empty:
        print("No empty slots in destination."); return

    pick_slot  = src_filled[0]
    place_slot = dest_empty[len(dest_empty) // 2]   # middle of empty run

    pick_px  = slot_pixel(src_bbox,  pick_slot)
    place_px = slot_pixel(dest_bbox, place_slot)

    src_angle  = tray_angle(arr, src_bbox)
    dest_angle = tray_angle(arr, dest_bbox)

    print()
    print("=" * 50)
    print(f"PICK  → slot {pick_slot:>2} in source  pixel ({pick_px[0]:4d}, {pick_px[1]:4d})  angle={src_angle:+.2f}°")
    print(f"PLACE → slot {place_slot:>2} in dest    pixel ({place_px[0]:4d}, {place_px[1]:4d})  angle={dest_angle:+.2f}°")
    print("(ROS: deproject_pixel_to_3d(pick_px)  → 3D XYZ)")
    print("(ROS: deproject_pixel_to_3d(place_px) → 3D XYZ)")
    print("=" * 50)

    # ---- Annotate image ----
    out_img = Image.fromarray(arr.copy())
    draw = ImageDraw.Draw(out_img)

    draw.rectangle(list(src_bbox),  outline=(255, 140, 0), width=3)
    draw.rectangle(list(dest_bbox), outline=(0,  200, 255), width=3)

    for slot in range(1, NUM_SLOTS + 1):
        cx, cy = slot_pixel(src_bbox, slot)
        col = (255, 60, 60) if src_occ[slot-1] else (60, 200, 60)
        draw.ellipse([cx-7, cy-7, cx+7, cy+7], outline=col, width=2)

    for slot in range(1, NUM_SLOTS + 1):
        cx, cy = slot_pixel(dest_bbox, slot)
        col = (255, 60, 60) if dest_occ[slot-1] else (60, 200, 60)
        draw.ellipse([cx-7, cy-7, cx+7, cy+7], outline=col, width=2)

    draw_marker(draw, pick_px[0],  pick_px[1],  (0, 255, 80),  f"PICK  s{pick_slot}")
    draw_marker(draw, place_px[0], place_px[1], (0, 180, 255), f"PLACE s{place_slot}")

    out_path = "pick_place_result.jpg"
    out_img.save(out_path)
    print(f"\nSaved: {out_path}")


if __name__ == "__main__":
    main()
