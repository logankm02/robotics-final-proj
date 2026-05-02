#!/usr/bin/env python3
"""
Deep feature analysis: texture, hue, saturation per slot.
Goal: distinguish single wafer (slot 8) from empty slots with wood grain (slots 9-18).
"""
import cv2
import numpy as np

WARPED_PATH = "/home/nano/final_project_ws/iter_warped.jpg"
NUM_SLOTS = 25
R = 12   # larger patch for texture stats

def patch(img, cy, cx, r, h, w):
    y1 = max(0, cy - r); y2 = min(h, cy + r)
    x1 = max(0, cx - r); x2 = min(w, cx + r)
    return img[y1:y2, x1:x2]

def analyze_slot(img_bgr, hsv, gray, slot, xs_frac):
    h, w = gray.shape
    t = slot / (NUM_SLOTS + 1)
    cy = int(t * h)
    stats = []
    for xf in xs_frac:
        cx = int(xf * w)
        pg = patch(gray, cy, cx, R, h, w)
        pb = patch(img_bgr, cy, cx, R, h, w)
        ph = patch(hsv, cy, cx, R, h, w)

        mean_gray = float(np.mean(pg))
        std_gray  = float(np.std(pg))          # texture: high=rough/wood, low=smooth/wafer
        hue       = float(np.mean(ph[:,:,0]))  # 0-180, wood~15-25, silicon~100-120?
        sat       = float(np.mean(ph[:,:,1]))
        val       = float(np.mean(ph[:,:,2]))

        b = float(np.mean(pb[:,:,0]))
        g = float(np.mean(pb[:,:,1]))
        r = float(np.mean(pb[:,:,2]))
        br = b / max(r, 1.0)

        stats.append({'mean': mean_gray, 'std': std_gray, 'hue': hue,
                      'sat': sat, 'val': val, 'br': br, 'b': b, 'g': g, 'r': r})
    return cy, stats

def main():
    img = cv2.imread(WARPED_PATH)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, w = gray.shape

    # Sample at positions that span the inner tray (avoid outer teeth)
    xs = [0.30, 0.45, 0.60]

    ground_truth = {8, 19, 20, 21, 22}

    # Per-slot feature aggregates
    rows = []
    for slot in range(1, NUM_SLOTS + 1):
        cy, stats = analyze_slot(img, hsv, gray, slot, xs)
        # aggregate across x positions
        min_mean = min(s['mean'] for s in stats)
        min_std  = min(s['std']  for s in stats)
        max_std  = max(s['std']  for s in stats)
        mean_hue = np.mean([s['hue'] for s in stats])
        max_sat  = max(s['sat']  for s in stats)
        max_br   = max(s['br']   for s in stats)
        # brightness at individual positions
        b30 = stats[0]['mean']
        b45 = stats[1]['mean']
        b60 = stats[2]['mean']
        h30 = stats[0]['hue']
        h45 = stats[1]['hue']
        h60 = stats[2]['hue']
        rows.append({
            'slot': slot, 'gt': slot in ground_truth,
            'min_mean': min_mean, 'min_std': min_std, 'max_std': max_std,
            'mean_hue': mean_hue, 'max_sat': max_sat, 'max_br': max_br,
            'b30': b30, 'b45': b45, 'b60': b60,
            'h30': h30, 'h45': h45, 'h60': h60, 'cy': cy,
        })

    print(f"{'Sl':>3} {'GT':>2}  {'minBr':>6} {'maxStd':>7} {'minStd':>7} {'meanH':>6} {'maxSat':>7} {'maxBR':>6}  {'b30':>5} {'b45':>5} {'b60':>5}  {'h30':>5} {'h45':>5} {'h60':>5}")
    print("-" * 110)
    for r in rows:
        gt = "X" if r['gt'] else "."
        print(f"{r['slot']:>3} {gt:>2}  {r['min_mean']:>6.1f} {r['max_std']:>7.1f} {r['min_std']:>7.1f} "
              f"{r['mean_hue']:>6.1f} {r['max_sat']:>7.1f} {r['max_br']:>6.2f}  "
              f"{r['b30']:>5.1f} {r['b45']:>5.1f} {r['b60']:>5.1f}  "
              f"{r['h30']:>5.1f} {r['h45']:>5.1f} {r['h60']:>5.1f}")

    print("\n\n=== ATTEMPT: dark + smooth (low texture) ===")
    print("Idea: wafers are smooth (low std) while wood grain is rough (high std)")
    print("      detect if min_brightness < T1 AND max_std < T2\n")

    best = (0, None, None, None)
    for t_bright in range(30, 120, 5):
        for t_std in range(10, 60, 5):
            det = {r['slot'] for r in rows if r['min_mean'] < t_bright and r['max_std'] < t_std}
            tp = len(det & ground_truth)
            fp = len(det - ground_truth)
            fn = len(ground_truth - det)
            f1 = (2*tp) / (2*tp + fp + fn) if (2*tp + fp + fn) > 0 else 0
            if f1 > best[0]:
                best = (f1, t_bright, t_std, det)
    f1, tb, ts, det = best
    print(f"  Best: bright<{tb} AND max_std<{ts} → F1={f1:.3f} det={sorted(det)}")
    print(f"  GT={sorted(ground_truth)}")

    print("\n=== ATTEMPT: dark + low hue (silicon blue vs wood orange) ===")
    print("Wood hue in HSV~15-25 (orange-yellow), silicon wafer hue~100-120 (blue)?\n")
    for r in rows:
        print(f"  slot {r['slot']:>2}: mean_hue={r['mean_hue']:5.1f} h30={r['h30']:5.1f} h45={r['h45']:5.1f} h60={r['h60']:5.1f}")

    print("\n=== ATTEMPT: darkness relative to local neighborhood ===")
    print("Compare each slot brightness to moving average of ±3 neighbor slots")
    mins = [r['min_mean'] for r in rows]
    for i, r in enumerate(rows):
        neighborhood = mins[max(0,i-4):i] + mins[i+1:min(NUM_SLOTS,i+5)]
        if len(neighborhood) == 0: continue
        nb_mean = np.mean(neighborhood)
        local_depression = nb_mean - r['min_mean']  # positive = darker than neighbors
        print(f"  slot {r['slot']:>2} {'X' if r['gt'] else '.'}:  min={r['min_mean']:5.1f}  nb_mean={nb_mean:5.1f}  depression={local_depression:+6.1f}")

    print("\n=== SPATIAL: full horizontal scan at each slot (to find dark disc) ===")
    print("Sample 20 points across the tray interior for each slot")
    xs_dense = np.linspace(0.12, 0.88, 20)
    disc_widths = []
    for r in rows:
        cy = r['cy']
        vals = []
        for xf in xs_dense:
            cx = int(xf * w)
            pg = patch(gray, cy, cx, 6, h, w)
            vals.append(float(np.mean(pg)))
        dark_thresh = 90
        dark_frac = sum(1 for v in vals if v < dark_thresh) / len(vals)
        min_v = min(vals)
        disc_widths.append((r['slot'], r['gt'], dark_frac, min_v, vals))
        print(f"  slot {r['slot']:>2} {'X' if r['gt'] else '.'}:  dark_frac={dark_frac:.2f}  min={min_v:5.1f}  profile=[{','.join(f'{int(v):3d}' for v in vals)}]")

    # Visualize on warped image
    vis = img.copy()
    for r in rows:
        cy = r['cy']
        # Color: TP=green, FN=red, FP=orange, TN=gray
        det_simple = r['min_mean'] < 80
        if r['gt'] and det_simple: col = (0, 255, 0)
        elif r['gt']:             col = (0, 0, 255)
        elif det_simple:          col = (0, 165, 255)
        else:                     col = (200, 200, 200)
        cx = int(0.45 * w)
        cv2.circle(vis, (cx, cy), 10, col, 2)
        cv2.putText(vis, f"{r['slot']}:{r['min_mean']:.0f}:std{r['max_std']:.0f}",
                    (cx + 13, cy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.28, col, 1)
    cv2.imwrite("/home/nano/final_project_ws/iter_analysis2.jpg", vis)
    print("\nSaved iter_analysis2.jpg")

if __name__ == "__main__":
    main()
