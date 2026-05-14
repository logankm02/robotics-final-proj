# TM12 Wafer Pick-and-Place — Runbook

End-to-end guide: bring up the perception stack, the robot, and the planner,
then run and calibrate a wafer transfer. Written so someone who has never run
this system can get it working.

> Supersedes the old detection-only runbook. The pipeline is now service-based
> and uses live `slide_XX` TF frames (not the old `pick_pose`/`place_pose`).

---

## 1. What the system is

A Techman **TM12** arm with a 140 mm gripper moves wafers between two plastic
trays. Perception finds the trays and which slots are occupied; the planner
drives the arm slot-by-slot.

### The five components

| # | Component | Package / location | Provides |
|---|-----------|---------------------|----------|
| 1 | **TM12 driver + MoveIt** | `~/tmdriver_ws` | `/joint_states`, `/move_action`, `/set_positions`, robot TF |
| 2 | **Perception (CV)** | `start_cv.sh` → `realsense_cv` | camera images, `flange→camera` TF, `/detect_slides` service, `slide_XX` frames |
| 3 | **Gripper** | `actuation` | `/gripper/control` (open/close) |
| 4 | **Planner** | `planning` (`pick_and_place`) | `/wafer_pick_place`, `/go_to_viewing_pose` services |
| 5 | **Trigger** | `tools/run_wafer_pick_place_current_pose.py` | calls `/wafer_pick_place` |

### TF chain that must exist for a pick

```
base → link_1…link_6 → flange → camera_link → camera_color_optical_frame → slide_XX
└──── TM driver ─────┘ └ camera_tf ┘ └─ realsense ─┘              └─ GSAM ─┘
```
Break any link and the arm can't see, can't locate slots, or can't connect
them to its own coordinate frame.

### How `/wafer_pick_place` works (the re-scan loop)

```
CYCLE N:
  1. Move arm to the viewing/home joint pose
  2. Call /detect_slides  → GSAM runs, publishes slide_XX frames, returns occupied slots
  3. Pick the FIRST occupied wafer in the source tray, place it in the matching
     dest-tray slot
  → repeat
Stop when: source tray empty | max_wafers reached | a move fails
Then: return to the viewing/home pose
```
Detection re-runs **every cycle**, so the tray state is always fresh. Slots
that fail (TF/IK/execution) are remembered and skipped — no infinite retry.

---

## 2. Quick start — one command

Once the one-time prerequisites (§3) are met and the **pendant is prepared**
(§3 — Listen node + speed slider), the whole pipeline is a single script:

```bash
cd ~/final_project_ws
./run_pipeline.sh                       # source tray 1 -> dest tray 0, single pick
```

`run_pipeline.sh` brings up all four stages in order — TM driver → perception →
gripper → planner — **verifying each before the next**, runs the
`/go_to_viewing_pose` motion gate, then triggers the run in the foreground.
It's idempotent: anything already running is reused, so re-running it just
re-gates and re-triggers.

```bash
./run_pipeline.sh --source-tray 0 --dest-tray 1   # swap trays
./run_pipeline.sh --max-wafers 0                  # full loop (default 1 = single pick)
./run_pipeline.sh --no-run                        # bring everything up, don't trigger
./run_pipeline.sh --status                        # what's up / down
./run_pipeline.sh --stop                          # tear it all down
```
Env overrides: `ROBOT_IP`, `GRIPPER_PORT`, `FORCE_CPU`. Logs: `/tmp/pipe_*.log`
(TM/gripper/planner) and `/tmp/cv_*.log` (perception).

If a stage fails, the script stops with a specific error and points at the log
or the fix. The most common stop is the **motion gate** — that means the
pendant isn't ready (see §3).

> **§§3–9 below are the manual breakdown** of what `run_pipeline.sh` automates.
> Read them to understand the system, to calibrate, or to debug when the
> one-command path stops.

---

## 3. One-time prerequisites

- ROS 2 Humble. Both workspaces built:
  ```bash
  cd ~/final_project_ws && colcon build && source install/setup.bash
  cd ~/tmdriver_ws     && colcon build && source install/setup.bash
  ```
  (If `colcon build` fails with `option --uninstall not recognized`, delete the
  stale `build/<pkg>` and `install/<pkg>` dirs and rebuild that package.)
- RealSense D435i on USB.
- SAM2 weights at `/home/nano/CV/GSAM/checkpoints/sam2.1_hiera_small.pt`.
- Arduino gripper on `/dev/ttyCH341USB0`.
- TM12 powered, e-stop wired, on the network.

### Network — the robot is at `172.16.8.2`

**Not** the TM factory default `192.168.10.2`. It's on the Jetson's **wired**
interface `enP8p1s0` (`172.16.x.x`). Confirm before starting:
```bash
ping -c 3 172.16.8.2     # must get replies
```
If it fails: check the cable, and check the robot's IP on the pendant
(Settings → Network). Whatever it is, that's the `robot_ip:=` value below.

### Pendant — the Listen node (most common failure point)

The ROS driver's command channel (SCT) only connects when a TMflow project
**with a Listen node** is **playing and parked on that Listen node**. Before
starting ROS:

1. Load a project that contains a **Listen node**.
2. Set the robot to **AUTO mode** (not Manual).
3. Press **Play** — the project runs and stops at the Listen node.
4. Clear any fault; make sure it's not paused.
5. **Speed slider** — set it to a real value (start ~25–50 %, not 0 %).
   At 0 % the robot accepts commands but barely moves.

---

## 4. Startup — five terminals (manual)

Run in order. **Verify each stage before starting the next.** Use a sixth
"scratch" terminal for the verify commands:
```bash
source /opt/ros/humble/setup.bash && source ~/final_project_ws/install/setup.bash
```

### Terminal 1 — TM12 driver + MoveIt
```bash
cd ~/tmdriver_ws && source install/setup.bash
ros2 launch tm12_moveit_config tm12_run_move_group.launch.py robot_ip:=172.16.8.2
```
**Verify:**
```bash
timeout 3 ros2 topic hz /joint_states     # ~60 Hz, steady
ros2 topic echo /joint_states --once      # values match the pendant's J1–J6
```
- If Terminal 1 spams `TM_ROS: (Listen node): Reconnecting...` → the Listen node
  isn't active on the pendant (see §3). The driver auto-connects once it is.
- `robot_ip:=...` must be the only extra launch arg — the launch file's arg
  parsing is positional.

### Terminal 2 — Perception (camera + camera_tf + GSAM)
```bash
cd ~/final_project_ws && ./start_cv.sh
```
Runs **once** and exits; the three nodes keep running detached (logs in
`/tmp/cv_*.log`). Takes ~1–2 min — GSAM model load on CPU. Defaults baked in:
CPU mode (`FORCE_CPU=1`) and a relaxed tray-area filter (`GSAM_MAX_AREA=0.45`).

**Verify:** wait for `Perception stack is ready.`, then:
```bash
ros2 service list | grep detect_slides           # /detect_slides
ros2 node list | grep -E "gsam|camera"            # gsam_slide_detect_node + camera
jq '.detected_slots' ~/final_project_ws/debug_slot_geometry.json
xdg-open ~/final_project_ws/debug_slots.jpg       # blue quad on tray, red=occupied
```
`start_cv.sh` other modes: `--status`, `--stop`, `--restart`, `--restart-gsam`
(restart just GSAM, e.g. after editing detection code), `--test` (run one more
`/detect_slides`).

### Terminal 3 — Gripper
```bash
cd ~/final_project_ws && source install/setup.bash
ros2 run actuation gripper --ros-args \
  -p serial_port:=/dev/ttyCH341USB0 -p require_hardware:=true
```
**Verify:** node starts without a fatal serial error. Optional test:
```bash
ros2 service call /gripper/control std_srvs/srv/SetBool "{data: false}"   # open
ros2 service call /gripper/control std_srvs/srv/SetBool "{data: true}"    # close
```

### Terminal 4 — Planner (`pick_and_place`)
```bash
cd ~/final_project_ws && source install/setup.bash
ros2 run planning pick_and_place --ros-args \
  --params-file src/planning/config/viewing_pose.yaml \
  -p detection_mode:=gsam -p alignment_method:=direct \
  -p max_wafers:=1
```
**Verify** the startup log shows:
- `Connected to MoveGroup executor` + `Connected to TM set_positions service`
- `Connected to gripper controller`
- `Viewing/home joint pose: [...] (use_viewing_joint_pose=True)`
- `TM joint move: stall timeout 20s ...`
- `max_wafers=1: ... single-pick / limited mode`
- `Ready! Pick-and-Place node initialized`

`max_wafers:=1` = **single-pick mode**: one scan→pick→place→return, then stop.
Use this until calibration is dialed in. Drop the flag (or set `:=0`) for the
full loop.

### Pre-run check — move to the viewing pose
```bash
ros2 service call /go_to_viewing_pose std_srvs/srv/Trigger
```
First real motion. Stand by the e-stop. Should return `success=True` and the
arm should move to the saved viewing pose. If it fails, see Troubleshooting.

### Terminal 5 — Trigger a run
```bash
cd ~/final_project_ws && source install/setup.bash
python3 tools/run_wafer_pick_place_current_pose.py --source-tray 0 --dest-tray 1
```
- `--source-tray` / `--dest-tray` are GSAM tray indices, numbered **left→right**
  in the camera image. If only one tray is detected it's index `0`. Swap the
  args if it picks from the wrong tray.
- The helper supplies a required `scan_pose`, but with `use_viewing_joint_pose`
  (the default) the loop uses the saved joint pose instead — so you don't need
  to pre-position the arm.

---

## 5. Calibration

Do this in single-pick mode (`max_wafers:=1`), e-stop in hand.

### 5a. The viewing / home pose

The pose the arm returns to between picks and detects from. To re-capture:

1. Jog the arm on the pendant to a spot where the camera sees both trays.
2. ```bash
   python3 tools/save_viewing_pose.py
   ```
   Reads live `/joint_states`, overwrites `src/planning/config/viewing_pose.yaml`.
3. Restart Terminal 4 (it loads the file via `--params-file`).

Test any time with `ros2 service call /go_to_viewing_pose std_srvs/srv/Trigger`.

### 5b. Grasp offset — "too high / too far forward / off to the side"

If the arm consistently misses the wafer by a fixed amount, trim it with the
**base-frame** grasp offsets (metres). Applied to both source and destination,
so approach/grasp/lift/place all shift together.

| Symptom | Knob |
|---------|------|
| Stops too **high** | `grasp_offset_z` negative (lower) |
| Stops too **far forward** | `grasp_offset_x` negative (verify sign on your base frame) |
| Off to the **side** | `grasp_offset_y` |

Tune live between runs — no node restart needed:
```bash
ros2 param set /pick_and_place grasp_offset_z -0.010
ros2 param set /pick_and_place grasp_offset_x -0.010
# ...re-trigger Terminal 5, observe, repeat in ~5 mm steps
```
Or set at launch: `-p grasp_offset_z:=-0.015 -p grasp_offset_x:=-0.008`.

When dialed in, make it permanent — add under `ros__parameters:` in
`src/planning/config/viewing_pose.yaml`:
```yaml
    grasp_offset_x: -0.008
    grasp_offset_y: 0.0
    grasp_offset_z: -0.015
```

> If the error is **uniform** across the tray → grasp offset is the right fix.
> If it **grows toward the tray edges** → that's a camera-calibration rotation
> issue in `src/planning/planning/camera_static_transform.py` (the hardcoded
> `flange→camera` matrix), not a constant offset.

### 5c. Slow arm / move timeouts

A move is failed only if it **stalls** (no progress toward the target) for
`tm_joint_stall_timeout` s (default 20), with `tm_joint_timeout` s (default 600)
as an absolute backstop. A slow-but-progressing move will *not* time out.

- Genuinely long moves: raise the backstop, `-p tm_joint_timeout:=900`.
- Very slow arm: raise the stall window, `-p tm_joint_stall_timeout:=45`.
- **Better fix:** raise the pendant **speed slider** — moves finish in seconds.

If a move fails, the log now says which mode: `arm did NOT move` (pendant
state), `arm moved ... then STALLED` (obstacle/limit), or `/joint_states
feedback stalled` (driver connection).

### 5d. Perception tuning (occupancy detection)

`/detect_slides` decides a slot is occupied via depth **or** color/hue rules.
After a detection, inspect:
```bash
xdg-open ~/final_project_ws/debug_slots.jpg              # final occupancy overlay
xdg-open ~/final_project_ws/debug_color_warp_tray0.jpg   # warped tray, color sampling
jq '.color_detection, .trays[].slot_sample_centers[:3]' \
   ~/final_project_ws/debug_slot_geometry.json
```
Tune GSAM params, then `./start_cv.sh --restart-gsam`:

| Param | Effect |
|-------|--------|
| `dark_abs_thresh` (42) | Rule 1: lower = stricter dark-pixel detection |
| `hue_low_cv2` / `hue_high_cv2` (70 / 100) | Rule 2: wafer hue band (OpenCV 0–180) |
| `single_thresh` (90) | Rule 2: brightness gate |
| `enable_color_detection` (true) | turn off to use depth only |
| `use_oriented_rect` (true) | rotated tray quad vs axis-aligned bbox |
| `max_tray_area_fraction` (0.45) | raise if a close/large tray is filtered out |
| `depth_diff_threshold` (10 mm) | depth occupancy threshold |

Override per launch via `start_cv.sh` env vars: `GSAM_MAX_AREA`, `GSAM_PROMPT`,
`GSAM_MAX_TRAYS`, `GSAM_MIN_AREA`. Example:
`GSAM_MAX_AREA=0.5 ./start_cv.sh --restart-gsam`.

> Note: on CPU each detection takes ~70 s, so the re-scan loop is slow. That's
> expected, not a bug. (CUDA is faster but currently hits an NVML allocator
> crash — `start_cv.sh` forces CPU by default.)

---

## 6. Going to the full loop

Once a single pick is reliable, stop Terminal 4 and relaunch **without**
`max_wafers` (or `-p max_wafers:=0`):
```bash
ros2 run planning pick_and_place --ros-args \
  --params-file src/planning/config/viewing_pose.yaml \
  -p detection_mode:=gsam -p alignment_method:=direct
```
Re-trigger Terminal 5. Now it loops: viewing pose → detect → pick first wafer →
place → repeat, until the source tray is empty. Keep the e-stop in hand.

---

## 7. Shutdown

```bash
./start_cv.sh --stop          # camera + camera_tf + GSAM
# Ctrl-C Terminals 4, 3, 1
```
End of day: always `./start_cv.sh --stop` (GSAM holds GPU/CPU memory).

---

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `ping 172.16.8.2` fails | wrong IP / cable / subnet | check pendant Network page; use the wired `enP8p1s0` |
| Terminal 1 loops `(Listen node): Reconnecting...` | no active Listen node | load project w/ Listen node, AUTO mode, press Play (§3) |
| RViz robot ≠ real arm | wrong `robot_ip`, or `use_sim_time` | relaunch with correct `robot_ip:=`; if still wrong, set `use_sim_time:=False` on `move_group` |
| `/joint_states` silent | driver not connected | check `robot_ip`, ping, Terminal 1 errors |
| `go_to_viewing_pose` → `set_positions command failed` | robot rejecting commands | Listen node not active / Manual mode / paused |
| `TM joint target failed (stalled) — arm did NOT move` | pendant state | speed slider > 0 %, AUTO mode, not paused |
| `... arm moved ... then STALLED` | obstacle / joint limit / e-stop | clear obstruction; check reachability |
| `GSAM detection failed ... Is start_cv.sh running?` | `/detect_slides` missing | start / check Terminal 2 |
| `GSAM service ... not available after 120s` | GSAM not up when run triggered | bring up Terminal 2 first; it loads ~1–2 min |
| `Could not lookup source TF slide_XX` | `slide_XX` frame missing/stale | `./start_cv.sh --restart-gsam`; verify `ros2 run tf2_ros tf2_echo base slide_07` |
| `0 wafers / source tray is empty` but tray is full | GSAM failed (now logs an explicit error instead) | check Terminal 2 and `debug_slots.jpg` |
| Detection finds no trays | tray too large in frame, filtered out | `GSAM_MAX_AREA=0.5 ./start_cv.sh --restart-gsam` |
| Picks from the wrong tray | tray indices | swap `--source-tray` / `--dest-tray` |
| Grasp consistently off by a fixed amount | calibration | grasp offsets (§5b) |

### Quick health check (scratch terminal)
```bash
ros2 node list        # /camera/camera /constant_tf_publisher /gsam_slide_detect_node
                      # /pick_and_place /gripper + TM/MoveIt nodes
ros2 service list | grep -E "detect_slides|gripper/control|wafer_pick_place|set_positions"
ros2 run tf2_ros tf2_echo base link_6      # arm TF
ros2 run tf2_ros tf2_echo base slide_07    # perception→arm TF chain (after a detection)
```

---

## 9. Parameter reference

### `pick_and_place` (Terminal 4)
| Param | Default | Purpose |
|-------|---------|---------|
| `detection_mode` | `marker` | use `gsam` for the tray pipeline |
| `alignment_method` | `perpendicular` | use `direct` for GSAM |
| `max_wafers` | `0` | 0 = full loop; 1 = single-pick mode |
| `use_viewing_joint_pose` | `true` | return to saved joint pose between cycles |
| `viewing_joint_positions` | (in code / yaml) | the home/viewing joint config (rad) |
| `grasp_offset_x/y/z` | `0.0` | base-frame calibration nudge (m) |
| `tm_joint_stall_timeout` | `20.0` | s of no-progress before a move fails |
| `tm_joint_timeout` | `600.0` | absolute move backstop (s) |
| `z_velocity_scale` / `xy_velocity_scale` | `0.2` / `0.4` | TM speed scales |

### `gsam_slide_detect` (started by `start_cv.sh`)
| Param | Default | Purpose |
|-------|---------|---------|
| `text_prompt` | `colored box.` | Grounding DINO prompt (`start_cv.sh` can override) |
| `force_cpu` | `false` | `start_cv.sh` sets `true` by default |
| `num_slots` | `25` | slots per tray |
| `max_tray_area_fraction` | `0.45` | reject detections bigger than this fraction of the frame |
| `use_oriented_rect` | `true` | oriented (rotated) tray quad |
| `enable_color_detection` | `true` | hue + dark-pixel occupancy rules |
| `dark_abs_thresh` / `single_thresh` | `42` / `90` | color Rule 1 / Rule 2 brightness |
| `hue_low_cv2` / `hue_high_cv2` | `70` / `100` | color Rule 2 hue band |
| `depth_diff_threshold` | `10` | mm shallower than tray floor = occupied |

### Key files
| Path | What |
|------|------|
| `start_cv.sh` | brings up / manages the perception stack |
| `src/planning/config/viewing_pose.yaml` | viewing pose + planner params |
| `tools/save_viewing_pose.py` | re-capture the viewing pose from live joints |
| `tools/run_wafer_pick_place_current_pose.py` | trigger a `/wafer_pick_place` run |
| `~/final_project_ws/debug_slots.jpg` | latest detection overlay |
| `~/final_project_ws/debug_slot_geometry.json` | per-slot detection detail |
| `/tmp/cv_*.log` | camera / camera_tf / gsam logs |
