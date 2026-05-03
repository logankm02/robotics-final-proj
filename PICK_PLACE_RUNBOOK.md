# Pick & Place Detection — Runbook

## Prerequisites

- ROS 2 workspace built: `colcon build && source install/setup.bash`
- RealSense D435i plugged in via USB
- SAM2 model weights present at the path set in `gsam_pickplace.launch.py`

---

## Step 1 — Verify detection (perception only, no robot motion)

Start the camera and the perception node in separate terminals.

**Terminal 1 — Camera:**
```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=1920x1080x30 \
  align_depth.enable:=true
```

**Terminal 2 — Perception node:**
```bash
ros2 run realsense_cv gsam_slide_detect --ros-args \
  -p text_prompt:='plastic tray.' \
  -p sam2_checkpoint:='/home/nano/final_project_ws/src/perception/realsense_cv/models/sam2.1_hiera_small.pt' \
  -p sam2_model_config:='configs/sam2.1/sam2.1_hiera_s.yaml'
```

**Terminal 3 — Trigger detection:**
```bash
ros2 service call /detect_slides std_srvs/srv/Trigger {}
```

### Check the visual output

```bash
ros2 run rqt_image_view rqt_image_view
```

Select topic: `/camera/edges_overlay`

The image shows:
- **Blue rectangles** — detected tray bounding boxes (Grounding DINO)
- **Red circles** — occupied slots, **green circles** — empty slots (numbered)
- **Green crosshair** — PICK position (first occupied slot in source tray)
- **Cyan crosshair** — PLACE position (middle of empty slots in destination tray)

A copy is also saved to disk as `debug_slots.jpg` in the node's working directory.

**Only proceed to Step 2 if the crosshairs are on the correct slots.**

---

## Step 2 — Read the pick and place coordinates

Once detection looks correct, read the published TF frames directly.

**Pick position and orientation:**
```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame pick_pose
```

**Place position and orientation:**
```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame place_pose
```

Each frame gives XYZ translation (metres, camera frame) and XYZW quaternion rotation:
```
- Translation: [x, y, z]   (Z = 0.288 m hardcoded — camera height above table)
- Rotation:    [x, y, z, w] (Z-axis yaw = tray tilt angle from vertical)
```

To see both poses in 3D, open **RViz2**, set Fixed Frame to `camera_color_optical_frame`, and add a **TF** display.

---

## Step 3 — Full system (once detection is validated)

```bash
ros2 launch planning gsam_pickplace.launch.py
```

Then trigger:
```bash
ros2 service call /detect_slides std_srvs/srv/Trigger {}
```

> **Note:** The launch file sets `text_prompt: 'colored box.'` on line 85.
> Change this to `'plastic tray.'` to match the updated detection logic before running.

---

## Debug files (saved on each service call)

| File | Contents |
|---|---|
| `debug_slots.jpg` | Annotated image — slot circles, numbers, pick/place crosshairs |
| `debug_mask_tray0.png` | Raw SAM2 mask for tray 0 |
| `debug_mask_tray1.png` | Raw SAM2 mask for tray 1 |

---

## Node log output

```
[gsam_slide_detect]: Tray 0: 0/25 slots occupied
[gsam_slide_detect]: Tray 1: 5/25 slots occupied
[gsam_slide_detect]: PICK  slot  8 tray 1  px=(1302, 444)  xyz=(0.0412, -0.0231, 0.2880)  angle=+1.18°
[gsam_slide_detect]: PLACE slot 13 tray 0  px=(906,  540)  xyz=(-0.0821, 0.0105, 0.2880)  angle=-3.68°
```

Stream live:
```bash
ros2 topic echo /rosout
```

---

## How detection works (for reference)

1. **Grounding DINO** (`plastic tray.` prompt) detects both tray bounding boxes from the colour image.
2. **RGB wafer detection** runs on each tray crop — two rules per slot:
   - Rule 1: minimum column brightness `< 42` → group wafer (dark silicon)
   - Rule 2: hue at x=45% of tray width in range `(70, 100)` AND brightness `< 90` → single wafer
3. **Source tray** = tray with the most occupied slots. **Dest tray** = the other one.
4. **PICK** = first occupied slot from the top of the source tray.
5. **PLACE** = middle of the empty slots in the destination tray.
6. Pixel coords are deprojected to 3D and published as `pick_pose` / `place_pose` TF frames.
7. Tray tilt angle (from `cv2.minAreaRect` on the SAM2 mask) is encoded as a Z-axis yaw in each frame's rotation.
