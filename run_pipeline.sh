#!/usr/bin/env bash
#
# run_pipeline.sh - One-command bringup of the full TM12 wafer pick-and-place
# pipeline, ending with a /wafer_pick_place run.
#
# Stages (each verified before the next is started):
#   1. TM12 driver + MoveIt       (~/tmdriver_ws)        -> /joint_states
#   2. Perception stack           (./start_cv.sh)        -> /detect_slides
#   3. Gripper                    (actuation)            -> /gripper/control
#   4. Planner                    (planning)             -> /wafer_pick_place
#   5. Motion gate                (/go_to_viewing_pose)  -> proves SCT/motion works
#   6. Trigger the run            (tools/run_wafer_pick_place_current_pose.py)
#
# Long-running nodes are launched detached (setsid+nohup); logs in
# /tmp/pipe_<name>.log, pids in /tmp/pipe_<name>.pid. Re-runs are idempotent:
# anything already up is reused, so a second invocation just re-gates and
# re-triggers.
#
# Usage:
#   ./run_pipeline.sh                          # bring up + run (source 1 -> dest 0)
#   ./run_pipeline.sh --source-tray 0 --dest-tray 1
#   ./run_pipeline.sh --max-wafers 0           # full loop (default 1 = single pick)
#   ./run_pipeline.sh --no-run                 # bring everything up, skip the trigger
#   ./run_pipeline.sh --status                 # what's up / down
#   ./run_pipeline.sh --stop                   # tear down everything this script started
#
# Env overrides:
#   ROBOT_IP=172.16.8.2        TM robot IP
#   GRIPPER_PORT=/dev/ttyCH341USB0
#   FORCE_CPU=1                passed through to start_cv.sh (default 1)

set -u

WS_DIR="/home/nano/final_project_ws"
TM_WS_DIR="/home/nano/tmdriver_ws"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROBOT_IP="${ROBOT_IP:-172.16.8.2}"
GRIPPER_PORT="${GRIPPER_PORT:-/dev/ttyCH341USB0}"

# Run defaults (overridable via flags)
SOURCE_TRAY=1
DEST_TRAY=0
MAX_WAFERS=1
DO_RUN=1
ACTION="start"

VIEWING_PARAMS="${WS_DIR}/src/planning/config/viewing_pose.yaml"

# -- pretty output ----------------------------------------------------------

C_GREEN='\033[0;32m'; C_RED='\033[0;31m'; C_YEL='\033[1;33m'
C_CYAN='\033[0;36m';  C_DIM='\033[2m';    C_RST='\033[0m'

log()   { printf "${C_CYAN}[pipeline]${C_RST} %s\n" "$*"; }
ok()    { printf "${C_GREEN}  [ok]${C_RST}   %s\n" "$*"; }
warn()  { printf "${C_YEL}  [warn]${C_RST} %s\n" "$*"; }
err()   { printf "${C_RED}  [err]${C_RST}  %s\n" "$*"; }
stage() { printf "\n${C_CYAN}=== %s ===${C_RST}\n" "$*"; }

# -- ROS env ----------------------------------------------------------------

source_ros() {
    # ROS setup.bash files reference unset vars, so drop -u while sourcing.
    set +u
    if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
        # shellcheck disable=SC1090
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
    else
        set -u; err "/opt/ros/${ROS_DISTRO}/setup.bash not found"; exit 1
    fi
    # tmdriver overlay first, final_project overlay last so its packages win.
    if [[ -f "${TM_WS_DIR}/install/setup.bash" ]]; then
        # shellcheck disable=SC1090
        source "${TM_WS_DIR}/install/setup.bash"
    else
        set -u; err "${TM_WS_DIR}/install/setup.bash not found - build tmdriver_ws"; exit 1
    fi
    if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
        # shellcheck disable=SC1090
        source "${WS_DIR}/install/setup.bash"
    else
        set -u; err "${WS_DIR}/install/setup.bash not found - run 'colcon build'"; exit 1
    fi
    set -u
}

# -- probes -----------------------------------------------------------------

node_alive()        { ros2 node list 2>/dev/null | grep -Fxq "$1"; }
service_alive()     { ros2 service list 2>/dev/null | grep -Fxq "$1"; }
topic_publishing()  { timeout "$2" ros2 topic echo --once "$1" >/dev/null 2>&1; }
pid_running()       { [[ -f "$1" ]] && kill -0 "$(cat "$1")" 2>/dev/null; }

wait_for() {        # wait_for <desc> <timeout_s> <test cmd...>
    local desc="$1" timeout="$2"; shift 2
    local start=$SECONDS
    while ! eval "$@" >/dev/null 2>&1; do
        if (( SECONDS - start > timeout )); then
            err "timeout waiting for ${desc} (${timeout}s)"
            return 1
        fi
        sleep 0.5
    done
    ok "${desc} ready ($((SECONDS - start))s)"
}

start_bg() {        # start_bg <name> <logf> <pidf> <cmd...>
    local name="$1" logf="$2" pidf="$3"; shift 3
    log "starting ${name} -> ${logf}"
    setsid nohup "$@" >"${logf}" 2>&1 &
    echo $! >"${pidf}"
    sleep 1
    if ! kill -0 "$(cat "${pidf}")" 2>/dev/null; then
        err "${name} died immediately - see ${logf}"
        return 1
    fi
}

# -- stages -----------------------------------------------------------------

preflight() {
    stage "PRE-FLIGHT"
    log "robot IP: ${ROBOT_IP}"
    if ping -c 2 -W 2 "${ROBOT_IP}" >/dev/null 2>&1; then
        ok "robot reachable at ${ROBOT_IP}"
    else
        err "cannot ping ${ROBOT_IP} - check the cable, the robot's IP on the"
        err "pendant (Settings -> Network), and that the Jetson is on the same"
        err "subnet. Override with ROBOT_IP=<addr> ./run_pipeline.sh"
        return 1
    fi
    warn "pendant checklist: project with a Listen node PLAYING, AUTO mode,"
    warn "not paused, speed slider > 0%. The motion gate (stage 5) verifies this."
}

start_tm() {
    stage "STAGE 1/5 - TM12 driver + MoveIt"
    if topic_publishing /joint_states 2; then
        ok "TM driver already up (/joint_states streaming)"
        return 0
    fi
    start_bg "tm_driver+moveit" /tmp/pipe_tm.log /tmp/pipe_tm.pid \
        ros2 launch tm12_moveit_config tm12_run_move_group.launch.py \
            robot_ip:="${ROBOT_IP}" || return 1
    # SVR channel (joint feedback) - this is up even without the Listen node.
    wait_for "/joint_states stream" 60 "topic_publishing /joint_states 5" || {
        err "no /joint_states - TM driver did not connect. See /tmp/pipe_tm.log"
        return 1
    }
    wait_for "/set_positions service" 30 "service_alive /set_positions" || return 1
}

start_cv() {
    stage "STAGE 2/5 - Perception (camera + camera_tf + GSAM)"
    if service_alive /detect_slides; then
        ok "perception already up (/detect_slides present)"
        return 0
    fi
    log "handing off to start_cv.sh (model load ~1-2 min on CPU)..."
    ( cd "${WS_DIR}" && ./start_cv.sh ) || {
        err "start_cv.sh failed - see its output and /tmp/cv_*.log"
        return 1
    }
    wait_for "/detect_slides service" 30 "service_alive /detect_slides" || return 1
}

start_gripper() {
    stage "STAGE 3/5 - Gripper"
    if service_alive /gripper/control; then
        ok "gripper already up (/gripper/control present)"
        return 0
    fi
    start_bg "gripper" /tmp/pipe_gripper.log /tmp/pipe_gripper.pid \
        ros2 run actuation gripper --ros-args \
            -p serial_port:="${GRIPPER_PORT}" \
            -p require_hardware:=true || return 1
    wait_for "/gripper/control service" 30 "service_alive /gripper/control" || {
        err "gripper service never appeared - serial port ${GRIPPER_PORT}?"
        err "see /tmp/pipe_gripper.log"
        return 1
    }
}

start_planner() {
    stage "STAGE 4/5 - Planner (pick_and_place)"
    if service_alive /wafer_pick_place; then
        ok "planner already up (/wafer_pick_place present)"
        return 0
    fi
    local params_arg=()
    [[ -f "${VIEWING_PARAMS}" ]] && params_arg=(--params-file "${VIEWING_PARAMS}")
    start_bg "pick_and_place" /tmp/pipe_planner.log /tmp/pipe_planner.pid \
        ros2 run planning pick_and_place --ros-args \
            "${params_arg[@]}" \
            -p detection_mode:=gsam \
            -p alignment_method:=direct \
            -p max_wafers:="${MAX_WAFERS}" || return 1
    # pick_and_place blocks in __init__ on MoveGroup/TM/gripper; the services
    # only appear once it finishes initializing, so this also gates on that.
    wait_for "/wafer_pick_place service"  90 "service_alive /wafer_pick_place"  || return 1
    wait_for "/go_to_viewing_pose service" 30 "service_alive /go_to_viewing_pose" || return 1
    log "planner mode: max_wafers=${MAX_WAFERS} "\
"($([[ ${MAX_WAFERS} -eq 0 ]] && echo 'full loop' || echo 'single-pick / limited'))"
}

motion_gate() {
    stage "STAGE 5/5 - Motion gate (/go_to_viewing_pose)"
    warn "first real motion - stand by the e-stop."
    local out
    out=$(ros2 service call /go_to_viewing_pose std_srvs/srv/Trigger 2>&1 || true)
    if grep -q "success=True" <<<"${out}"; then
        ok "arm reached the viewing/home pose - SCT channel + motion confirmed"
        return 0
    fi
    err "/go_to_viewing_pose failed - the arm is not executing motion."
    err "response: $(grep -oE "message=.*" <<<"${out}" | head -n1)"
    err "Almost always a pendant-side issue:"
    err "  - project with a Listen node must be PLAYING and parked on it"
    err "  - robot in AUTO mode (not Manual), not paused, no active fault"
    err "  - speed slider > 0%"
    err "Fix on the pendant, then re-run ./run_pipeline.sh (it will skip the"
    err "stages already up and retry the gate)."
    return 1
}

do_run() {
    stage "RUN - /wafer_pick_place  (source ${SOURCE_TRAY} -> dest ${DEST_TRAY})"
    log "trigger: tools/run_wafer_pick_place_current_pose.py"\
" --source-tray ${SOURCE_TRAY} --dest-tray ${DEST_TRAY}"
    warn "running in the foreground - watch the arm, e-stop in hand."
    echo
    cd "${WS_DIR}" && exec python3 tools/run_wafer_pick_place_current_pose.py \
        --source-tray "${SOURCE_TRAY}" --dest-tray "${DEST_TRAY}"
}

# -- status / stop ----------------------------------------------------------

status() {
    source_ros
    echo
    printf "  %-34s %s\n" "TM driver (/joint_states):" \
        "$(topic_publishing /joint_states 2 && echo STREAMING || echo DOWN)"
    printf "  %-34s %s\n" "  /set_positions:" \
        "$(service_alive /set_positions && echo UP || echo DOWN)"
    printf "  %-34s %s\n" "Perception (/detect_slides):" \
        "$(service_alive /detect_slides && echo UP || echo DOWN)"
    printf "  %-34s %s\n" "Gripper (/gripper/control):" \
        "$(service_alive /gripper/control && echo UP || echo DOWN)"
    printf "  %-34s %s\n" "Planner (/wafer_pick_place):" \
        "$(service_alive /wafer_pick_place && echo UP || echo DOWN)"
    printf "  %-34s %s\n" "  /go_to_viewing_pose:" \
        "$(service_alive /go_to_viewing_pose && echo UP || echo DOWN)"
    echo
    for n in tm gripper planner; do
        if [[ -f "/tmp/pipe_${n}.pid" ]]; then
            local p; p=$(cat "/tmp/pipe_${n}.pid")
            if kill -0 "$p" 2>/dev/null; then
                printf "  ${C_DIM}pid:${C_RST} pipe_%s (%s alive, log /tmp/pipe_%s.log)\n" "$n" "$p" "$n"
            else
                printf "  ${C_DIM}pid:${C_RST} pipe_%s (%s ${C_RED}dead${C_RST})\n" "$n" "$p"
            fi
        fi
    done
    echo
    log "perception stack status:"
    ( cd "${WS_DIR}" && ./start_cv.sh --status ) 2>/dev/null | sed 's/^/  /'
}

stop_all() {
    source_ros
    stage "STOP"
    # Reverse start order: planner -> gripper -> tm.
    for n in planner gripper tm; do
        local pidf="/tmp/pipe_${n}.pid"
        if [[ -f "${pidf}" ]]; then
            local p; p=$(cat "${pidf}")
            if kill -0 "$p" 2>/dev/null; then
                kill -TERM -- -"$p" 2>/dev/null || kill -TERM "$p" 2>/dev/null
                ok "stopped ${n} (pgid ${p})"
                for _ in $(seq 1 10); do kill -0 "$p" 2>/dev/null || break; sleep 0.5; done
                kill -0 "$p" 2>/dev/null && { warn "${n} SIGKILL"; \
                    kill -KILL -- -"$p" 2>/dev/null || kill -KILL "$p" 2>/dev/null; }
            else
                warn "${n} already gone"
            fi
            rm -f "${pidf}"
        fi
    done
    log "stopping perception stack..."
    ( cd "${WS_DIR}" && ./start_cv.sh --stop ) 2>/dev/null | sed 's/^/  /'
    ok "pipeline stopped"
}

# -- arg parsing ------------------------------------------------------------

while [[ $# -gt 0 ]]; do
    case "$1" in
        --source-tray) SOURCE_TRAY="$2"; shift 2 ;;
        --dest-tray)   DEST_TRAY="$2";   shift 2 ;;
        --max-wafers)  MAX_WAFERS="$2";  shift 2 ;;
        --no-run)      DO_RUN=0;         shift ;;
        --status)      ACTION="status";  shift ;;
        --stop)        ACTION="stop";    shift ;;
        -h|--help)     sed -n '2,40p' "$0"; exit 0 ;;
        *)             err "unknown arg: $1"; exit 2 ;;
    esac
done

case "$ACTION" in
    status) status; exit 0 ;;
    stop)   stop_all; exit 0 ;;
esac

# -- main bringup -----------------------------------------------------------

source_ros
log "workspace: ${WS_DIR}   (ROS ${ROS_DISTRO})"

preflight       || exit 1
start_tm        || { err "stage 1 (TM driver) failed";  exit 1; }
start_cv        || { err "stage 2 (perception) failed"; exit 1; }
start_gripper   || { err "stage 3 (gripper) failed";    exit 1; }
start_planner   || { err "stage 4 (planner) failed";    exit 1; }
motion_gate     || { err "stage 5 (motion gate) failed"; exit 1; }

echo
ok "All stages up and the arm moves. Pipeline is ready."

if (( DO_RUN )); then
    do_run        # exec's the trigger in the foreground
else
    echo
    log "--no-run set; skipping the trigger. Run it yourself with:"
    log "  cd ${WS_DIR} && python3 tools/run_wafer_pick_place_current_pose.py \\"
    log "    --source-tray ${SOURCE_TRAY} --dest-tray ${DEST_TRAY}"
    log "Tear down with: ./run_pipeline.sh --stop"
fi
