#!/usr/bin/env bash
#
# start_cv.sh - Bring up the perception-only slice of the pick-and-place stack:
#   1. realsense2_camera   (publishes /camera/camera/color + aligned depth)
#   2. camera_tf           (static flange<-camera transform)
#   3. gsam_slide_detect   (Grounding DINO + SAM2 + the new color/hue rules,
#                           exposes /detect_slides Trigger service)
#
# Each component is checked first; only missing pieces are launched. Logs go to
# /tmp/cv_<name>.log and PIDs to /tmp/cv_<name>.pid so re-runs stay idempotent.
#
# Usage:
#   ./start_cv.sh              # check + start anything missing, wait until the
#                              #   full pipeline has *finished* coming up
#                              #   (models loaded, frames flowing, service
#                              #    returns a real Trigger response)
#   ./start_cv.sh --status     # just print what is / isn't running
#   ./start_cv.sh --stop       # kill what this script launched and wait for ROS
#                              #   to drop the nodes
#   ./start_cv.sh --restart    # stop (with verification) then start
#   ./start_cv.sh --test       # run one /detect_slides call against a stack
#                              #   that is already up, and verify the new
#                              #   color/hue debug artifacts got refreshed
#   ./start_cv.sh --no-verify     # skip the end-to-end service call after launch
#   ./start_cv.sh --force-cpu     # pass force_cpu:=true to gsam_slide_detect
#   ./start_cv.sh --restart-gsam  # kill+restart only gsam (camera stays up).
#                                 # Combine with --force-cpu to recover from a
#                                 # bad CUDA state.
#
# Env-var overrides applied at GSAM launch (also work with --restart-gsam):
#   GSAM_MAX_AREA=0.45     ./start_cv.sh ...     # raise the "too_large" filter
#   GSAM_MIN_AREA=0.01     ./start_cv.sh ...     # lower the "too_small" filter
#   GSAM_PROMPT='colored box.' ./start_cv.sh ...  # change Grounding DINO prompt
#   GSAM_MAX_TRAYS=3       ./start_cv.sh ...     # keep more candidates

set -u

WS_DIR="/home/nano/final_project_ws"
ROS_DISTRO_DEFAULT="humble"
ROS_DISTRO="${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}"

# Required ROS topics / services + the node names ros2 lists when each
# component is alive.
CAMERA_NODE="/camera/camera"
CAMERA_TOPIC_COLOR="/camera/camera/color/image_raw"
CAMERA_TOPIC_DEPTH="/camera/camera/aligned_depth_to_color/image_raw"
CAMERA_TOPIC_INFO="/camera/camera/color/camera_info"

# Node names below are what each Python file passes to `super().__init__(...)`,
# NOT the colcon executable name. Mismatch -> "node not ready" timeouts even
# though the process is alive.
#   camera_static_transform.py -> ConstantTransformPublisher('constant_tf_publisher')
#   gsam_slide_detect.py       -> GSAMSlideDetectNode('gsam_slide_detect_node')
CAMERA_TF_NODE="/constant_tf_publisher"
GSAM_NODE="/gsam_slide_detect_node"
GSAM_SERVICE="/detect_slides"

# -- helpers ----------------------------------------------------------------

C_GREEN='\033[0;32m'; C_RED='\033[0;31m'; C_YEL='\033[1;33m'
C_CYAN='\033[0;36m'; C_DIM='\033[2m';    C_RST='\033[0m'

log()  { printf "${C_CYAN}[start_cv]${C_RST} %s\n" "$*"; }
ok()   { printf "${C_GREEN}  [ok]${C_RST}   %s\n" "$*"; }
warn() { printf "${C_YEL}  [warn]${C_RST} %s\n" "$*"; }
err()  { printf "${C_RED}  [err]${C_RST}  %s\n" "$*"; }

source_ros() {
    # ROS setup.bash files reference unset vars (AMENT_TRACE_SETUP_FILES,
    # COLCON_TRACE, ...) so they aren't safe under `set -u`. Drop -u just for
    # the sources, then re-enable for the rest of the script.
    set +u
    # Source overlay last so our gsam_slide_detect changes win.
    if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
        # shellcheck disable=SC1090
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
    else
        set -u
        err "/opt/ros/${ROS_DISTRO}/setup.bash not found - set ROS_DISTRO env var"
        exit 1
    fi
    if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
        # shellcheck disable=SC1090
        source "${WS_DIR}/install/setup.bash"
    else
        set -u
        err "${WS_DIR}/install/setup.bash not found - run 'colcon build' first"
        exit 1
    fi
    set -u
}

node_alive() {           # node_alive <node_name>
    ros2 node list 2>/dev/null | grep -Fxq "$1"
}

topic_alive() {          # topic_alive <topic_name>
    ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

topic_publishing() {     # topic_publishing <topic> <timeout_sec>
    # Returns 0 if at least one message arrives within timeout.
    timeout "$2" ros2 topic echo --once "$1" >/dev/null 2>&1
}

service_alive() {        # service_alive <service_name>
    ros2 service list 2>/dev/null | grep -Fxq "$1"
}

pid_running() {          # pid_running <pid_file>
    [[ -f "$1" ]] && kill -0 "$(cat "$1")" 2>/dev/null
}

start_bg() {             # start_bg <name> <log_file> <pid_file> <cmd...>
    local name="$1" logf="$2" pidf="$3"; shift 3
    log "starting ${name} -> ${logf}"
    # nohup so the process survives this shell; setsid so we can kill the group.
    setsid nohup "$@" >"${logf}" 2>&1 &
    echo $! >"${pidf}"
    sleep 1
    if ! kill -0 "$(cat "${pidf}")" 2>/dev/null; then
        err "${name} died immediately - see ${logf}"
        return 1
    fi
}

wait_for() {             # wait_for <description> <timeout> <bash_test_cmd>
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

# -- status / stop ----------------------------------------------------------

status_only() {
    source_ros
    echo
    log "ROS daemon: $(ros2 daemon status 2>/dev/null | head -n1 || echo unknown)"
    echo

    printf "  %-30s %s\n" "RealSense node ${CAMERA_NODE}:"   \
        "$(node_alive "${CAMERA_NODE}"   && echo UP || echo DOWN)"
    printf "  %-30s %s\n" "  color topic published:"          \
        "$(topic_publishing "${CAMERA_TOPIC_COLOR}" 2 && echo YES || echo NO)"
    printf "  %-30s %s\n" "  depth topic published:"          \
        "$(topic_publishing "${CAMERA_TOPIC_DEPTH}" 2 && echo YES || echo NO)"
    printf "  %-30s %s\n" "  camera_info published:"          \
        "$(topic_publishing "${CAMERA_TOPIC_INFO}"  2 && echo YES || echo NO)"
    printf "  %-30s %s\n" "camera_tf ${CAMERA_TF_NODE}:"      \
        "$(node_alive "${CAMERA_TF_NODE}" && echo UP || echo DOWN)"
    printf "  %-30s %s\n" "gsam node ${GSAM_NODE}:"           \
        "$(node_alive "${GSAM_NODE}"      && echo UP || echo DOWN)"
    printf "  %-30s %s\n" "  service ${GSAM_SERVICE}:"        \
        "$(service_alive "${GSAM_SERVICE}" && echo AVAILABLE || echo MISSING)"
    echo
    for name in camera camera_tf gsam; do
        if [[ -f "/tmp/cv_${name}.pid" ]]; then
            local pid; pid=$(cat "/tmp/cv_${name}.pid")
            if kill -0 "$pid" 2>/dev/null; then
                printf "  ${C_DIM}pid file:${C_RST}  /tmp/cv_%s.pid (pid %s alive, log /tmp/cv_%s.log)\n" "$name" "$pid" "$name"
            else
                printf "  ${C_DIM}pid file:${C_RST}  /tmp/cv_%s.pid (pid %s ${C_RED}dead${C_RST})\n" "$name" "$pid"
            fi
        fi
    done
    echo
}

stop_all() {
    log "stopping CV stack (only processes this script started)"
    for name in gsam camera_tf camera; do   # reverse start order
        local pidf="/tmp/cv_${name}.pid"
        if [[ -f "${pidf}" ]]; then
            local pid; pid=$(cat "${pidf}")
            if kill -0 "$pid" 2>/dev/null; then
                # Kill the whole process group (setsid set pgid=pid).
                if kill -TERM -- -"$pid" 2>/dev/null; then
                    ok "sent SIGTERM to ${name} pgid ${pid}"
                else
                    kill -TERM "$pid" 2>/dev/null && ok "sent SIGTERM to ${name} pid ${pid}"
                fi
                for _ in 1 2 3 4 5 6 7 8 9 10; do
                    kill -0 "$pid" 2>/dev/null || break
                    sleep 0.5
                done
                if kill -0 "$pid" 2>/dev/null; then
                    warn "${name} did not exit, sending SIGKILL"
                    kill -KILL -- -"$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
                fi
            else
                warn "${name} pid ${pid} already gone"
            fi
            rm -f "${pidf}"
        fi
    done

    # Process death != ROS graph cleanup. Wait for the discovery layer to
    # actually drop the nodes/topics/service so a follow-up start_all sees
    # a clean slate instead of stale advertisements.
    log "waiting for ROS graph to drop nodes/topics/service"
    local graph_ok=1
    for spec in \
        "node:${CAMERA_NODE}" \
        "node:${CAMERA_TF_NODE}" \
        "node:${GSAM_NODE}" \
        "service:${GSAM_SERVICE}" \
        "topic:${CAMERA_TOPIC_COLOR}"; do
        local kind="${spec%%:*}" name="${spec#*:}" start=$SECONDS
        while :; do
            case "${kind}" in
                node)    node_alive    "${name}" || break ;;
                service) service_alive "${name}" || break ;;
                topic)   topic_alive   "${name}" || break ;;
            esac
            if (( SECONDS - start > 15 )); then
                warn "${kind} ${name} still present in ROS graph after 15s"
                graph_ok=0
                break
            fi
            sleep 0.5
        done
    done
    if (( graph_ok )); then
        ok "ROS graph cleared - safe to relaunch"
    else
        warn "some ROS entities lingered - check stale ros2 daemon or manual nodes"
    fi
}

# -- start sequence ---------------------------------------------------------

start_camera() {
    if node_alive "${CAMERA_NODE}" && topic_publishing "${CAMERA_TOPIC_COLOR}" 2; then
        ok "RealSense already up"
        return 0
    fi
    if pid_running /tmp/cv_camera.pid; then
        warn "camera pid alive but topic silent - waiting"
    else
        start_bg "RealSense" /tmp/cv_camera.log /tmp/cv_camera.pid \
            ros2 launch realsense2_camera rs_launch.py \
                rgb_camera.color_profile:=1280x720x30 \
                align_depth.enable:=true || return 1
    fi
    wait_for "/camera/camera node"            45 "node_alive '${CAMERA_NODE}'"        || return 1
    wait_for "color image stream"             30 "topic_publishing '${CAMERA_TOPIC_COLOR}' 5" || return 1
    wait_for "aligned depth stream"           30 "topic_publishing '${CAMERA_TOPIC_DEPTH}' 5" || return 1
    wait_for "camera_info"                    20 "topic_publishing '${CAMERA_TOPIC_INFO}'  5" || return 1
}

start_camera_tf() {
    if node_alive "${CAMERA_TF_NODE}"; then
        ok "camera_tf already up"
        return 0
    fi
    start_bg "camera_tf" /tmp/cv_camera_tf.log /tmp/cv_camera_tf.pid \
        ros2 run planning camera_tf || return 1
    wait_for "camera_tf node" 20 "node_alive '${CAMERA_TF_NODE}'"
}

start_gsam() {
    # Build --ros-args list once so we only pass one `--ros-args` to ros2 run.
    local ros_args=()
    [[ "${FORCE_CPU:-0}" == "1" ]] && ros_args+=(-p force_cpu:=true)
    # Optional overrides via env vars - handy when the default tray-size filter
    # rejects your physical setup. e.g. GSAM_MAX_AREA=0.45 ./start_cv.sh ...
    [[ -n "${GSAM_MAX_AREA:-}"   ]] && ros_args+=(-p max_tray_area_fraction:="${GSAM_MAX_AREA}")
    [[ -n "${GSAM_MIN_AREA:-}"   ]] && ros_args+=(-p min_tray_area_fraction:="${GSAM_MIN_AREA}")
    [[ -n "${GSAM_PROMPT:-}"     ]] && ros_args+=(-p text_prompt:="${GSAM_PROMPT}")
    [[ -n "${GSAM_MAX_TRAYS:-}"  ]] && ros_args+=(-p max_trays:="${GSAM_MAX_TRAYS}")

    local extra_args=()
    (( ${#ros_args[@]} > 0 )) && extra_args=(--ros-args "${ros_args[@]}")

    if node_alive "${GSAM_NODE}" && service_alive "${GSAM_SERVICE}"; then
        ok "gsam_slide_detect already up"
        return 0
    fi
    start_bg "gsam_slide_detect" /tmp/cv_gsam.log /tmp/cv_gsam.pid \
        ros2 run realsense_cv gsam_slide_detect "${extra_args[@]}" || return 1
    # Model load is slow on Jetson - allow up to 5 minutes.
    wait_for "gsam_slide_detect node"      300 "node_alive '${GSAM_NODE}'"        || return 1
    wait_for "${GSAM_SERVICE} service"     300 "service_alive '${GSAM_SERVICE}'"  || return 1
}

stop_gsam_only() {
    # Used when the GSAM node is in a bad state (CUDA OOM, NVML assert) but
    # the RealSense + camera_tf side is healthy. Keeps the camera up so we
    # don't pay the USB enumeration + warm-up cost again.
    local pidf="/tmp/cv_gsam.pid"
    if [[ -f "${pidf}" ]]; then
        local pid; pid=$(cat "${pidf}")
        if kill -0 "$pid" 2>/dev/null; then
            log "stopping gsam_slide_detect (pgid ${pid})"
            kill -TERM -- -"$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
            for _ in 1 2 3 4 5 6 7 8 9 10; do
                kill -0 "$pid" 2>/dev/null || break
                sleep 0.5
            done
            kill -0 "$pid" 2>/dev/null && {
                warn "gsam did not exit, sending SIGKILL"
                kill -KILL -- -"$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
            }
        fi
        rm -f "${pidf}"
    fi
    # Also reap any same-named process that wasn't tracked by this script
    # (e.g. crashed gsam left behind a zombie service registration).
    pkill -f 'realsense_cv gsam_slide_detect' 2>/dev/null || true
    # Wait for ROS to drop the service so the next launch starts clean.
    local start=$SECONDS
    while service_alive "${GSAM_SERVICE}"; do
        if (( SECONDS - start > 15 )); then
            warn "${GSAM_SERVICE} still in ROS graph after 15s"
            break
        fi
        sleep 0.5
    done
    ok "gsam stopped"
}

verify_pipeline_ready() {
    # End-to-end check that GSAM has actually *finished* coming up:
    # models loaded, camera + depth frames arriving, service callable.
    # GSAM rejects the call until both last_image and last_depth are set, so
    # we treat "No image"/"No depth" responses as not-ready and retry.
    local timeout="${1:-180}" start=$SECONDS pre_ts post_ts
    local debug_json="${WS_DIR}/debug_slot_geometry.json"
    pre_ts=$( [[ -f "${debug_json}" ]] && stat -c %Y "${debug_json}" || echo 0 )

    log "verifying end-to-end readiness (calling ${GSAM_SERVICE}, up to ${timeout}s)"
    while :; do
        local out
        # `ros2 service call` exits after a single call by default; no --once flag.
        out=$(ros2 service call "${GSAM_SERVICE}" std_srvs/srv/Trigger 2>&1 || true)
        if grep -q "success=True"  <<<"${out}"; then
            ok "service call succeeded"
            echo "${out}" | sed -n 's/.*message=//p' | head -n1 | sed 's/^/      /'
            break
        elif grep -qE "No (image|depth) available" <<<"${out}"; then
            local msg
            msg=$(grep -oE "No (image|depth) available[^']*" <<<"${out}" | head -n1)
            printf "      waiting: %s\n" "${msg:-camera frames not flowing yet}"
        elif grep -q "success=False" <<<"${out}"; then
            err "${GSAM_SERVICE} returned failure:"
            echo "${out}" | sed -n 's/^/      /p'
            return 1
        else
            # Service not yet registered or call timed out at the rcl layer.
            :
        fi
        if (( SECONDS - start > timeout )); then
            err "pipeline did not finish initializing within ${timeout}s"
            err "last response:"
            echo "${out}" | sed -n 's/^/      /p'
            return 1
        fi
        sleep 2
    done

    post_ts=$( [[ -f "${debug_json}" ]] && stat -c %Y "${debug_json}" || echo 0 )
    if [[ "${post_ts}" -le "${pre_ts}" ]]; then
        warn "${debug_json} was not refreshed - check /tmp/cv_gsam.log"
    else
        ok "${debug_json} refreshed ($(date -d @"${post_ts}" '+%H:%M:%S'))"
    fi

    local n=0
    for f in "${WS_DIR}"/debug_color_warp_tray*.jpg; do
        [[ -f "$f" ]] || continue
        local mtime; mtime=$(stat -c %Y "$f")
        if (( mtime >= post_ts - 30 )); then
            n=$((n + 1))
        fi
    done
    if (( n > 0 )); then
        ok "color/hue debug images written for ${n} tray(s)"
    else
        warn "no fresh debug_color_warp_tray*.jpg - is enable_color_detection true?"
    fi
}

start_all() {
    source_ros
    log "workspace: ${WS_DIR}  (ROS ${ROS_DISTRO})"

    if ! ros2 daemon status >/dev/null 2>&1; then
        log "starting ros2 daemon"
        ros2 daemon start >/dev/null 2>&1 || true
    fi

    start_camera     || { err "camera bring-up failed";    return 1; }
    start_camera_tf  || { err "camera_tf bring-up failed"; return 1; }
    start_gsam       || { err "gsam bring-up failed";      return 1; }

    if (( ${VERIFY:-1} )); then
        verify_pipeline_ready 180 || {
            err "stack is partially up but pipeline never produced a result"
            err "logs: /tmp/cv_camera.log /tmp/cv_camera_tf.log /tmp/cv_gsam.log"
            return 1
        }
    else
        log "skipping end-to-end verification (--no-verify)"
    fi

    echo
    ok "Perception stack is ready."
    echo
    cat <<EOF
  Test it:
    ros2 service call ${GSAM_SERVICE} std_srvs/srv/Trigger

  Watch the new color/hue logs live:
    tail -f /tmp/cv_gsam.log

  Inspect color detection results after each service call:
    eog ${WS_DIR}/debug_color_warp_tray0.jpg ${WS_DIR}/debug_slots.jpg
    jq '.color_detection, .trays[].slot_sample_centers[0]' ${WS_DIR}/debug_slot_geometry.json

  Toggle / tune at runtime (no restart needed):
    ros2 param set ${GSAM_NODE} enable_color_detection false
    ros2 param set ${GSAM_NODE} hue_low_cv2 85
    ros2 param set ${GSAM_NODE} hue_high_cv2 115

  Stop everything this script started:
    $0 --stop
EOF
}

# -- arg parsing ------------------------------------------------------------

ACTION="start"
# Defaults baked in for our setup: force CPU (avoids the NVML allocator assert
# we hit) and raise the tray-size filter (our trays cover ~32% of the frame,
# above the original 20% cap). Override either at the env-var level if needed:
#   FORCE_CPU=0 ./start_cv.sh           # try CUDA again
#   GSAM_MAX_AREA=0.20 ./start_cv.sh    # restore the original filter
export FORCE_CPU="${FORCE_CPU:-1}"
export GSAM_MAX_AREA="${GSAM_MAX_AREA:-0.45}"
export VERIFY=1
for arg in "$@"; do
    case "$arg" in
        --status)       ACTION="status" ;;
        --stop)         ACTION="stop" ;;
        --restart)      ACTION="restart" ;;
        --restart-gsam) ACTION="restart-gsam" ;;
        --test)         ACTION="test" ;;
        --no-verify)    export VERIFY=0 ;;
        --force-cpu)    export FORCE_CPU=1 ;;
        -h|--help)
            sed -n '2,34p' "$0"; exit 0 ;;
        *)
            err "unknown arg: $arg"; exit 2 ;;
    esac
done

case "$ACTION" in
    status)  status_only ;;
    stop)    source_ros; stop_all ;;
    restart) source_ros; stop_all; start_all ;;
    restart-gsam)
        source_ros
        stop_gsam_only
        start_gsam || { err "gsam relaunch failed"; exit 1; }
        (( VERIFY )) && verify_pipeline_ready 180
        ;;
    start)   start_all ;;
    test)
        source_ros
        if ! service_alive "${GSAM_SERVICE}"; then
            err "${GSAM_SERVICE} not registered - run $0 first"
            exit 1
        fi
        verify_pipeline_ready 60
        ;;
esac
