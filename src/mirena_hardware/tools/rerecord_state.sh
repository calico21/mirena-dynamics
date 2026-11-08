#!/usr/bin/env bash
# Replay a sensor bag through mirena_state and record /state/car.
#
#   rerecord_state.sh <out_bag> [rate] [duration_s] [in_bag]
#
# Default input is the host-clock copy of the bag made with hostclock_bag.py:
# the raw /sensors/imu headers are GPS-stamped and 67 ms behind the rest of the
# bag at the start, which makes the filter reject the IMU predictions.
#
# The node runs with the calibrated parameters the mission launch files use
# (brake_test / trackdrive / skidpad / autocross). Playback publishes /clock
# from each message (--clock-topics-all) and the recorder stamps with sim time,
# so the new bag keeps the original timeline even when rate != 1. Playback
# starts paused and is resumed once the node and recorder are up, so the first
# samples are not lost.
set -o pipefail

source /opt/ros/jazzy/setup.bash
source /home/mirena/mirena_ws/install/setup.bash
set -u

OUT=${1:?usage: rerecord_state.sh <out_bag> [rate] [duration_s] [in_bag]}
RATE=${2:-1.0}
DUR=${3:-}
IN_BAG=${4:-/home/mirena/mirena_ws/log/bag_hostclock}

# --- input clock alignment -------------------------------------------------
# /sensors/imu is stamped from GPS hardware time while the rest of the bag is
# on the host clock. Replaying the raw bag makes the encoder time-sync predict
# run ahead of the IMU headers, so the filter rejects the IMU predictions and
# the lateral velocity drifts once the car moves. hostclock_bag.py shifts only
# the IMU headers onto the arrival clock; cache the result next to the bag.
ALIGN="$(cd "$(dirname "$0")" && pwd)/hostclock_bag.py"
case "${IN_BAG%/}" in
    *_hostclock) ;;  # already aligned
    *)
        if [ -f "$ALIGN" ] && \
           ros2 bag info "$IN_BAG" 2>/dev/null | grep -q '/sensors/imu '; then
            if [ -f "$IN_BAG" ]; then
                ALIGNED="${IN_BAG%.*}_hostclock"
            else
                ALIGNED="${IN_BAG%/}_hostclock"
            fi
            if [ ! -d "$ALIGNED" ] || [ "$IN_BAG" -nt "$ALIGNED" ]; then
                echo "[align] alineando reloj de $IN_BAG"
                rm -rf "$ALIGNED"
                python3 "$ALIGN" "$IN_BAG" "$ALIGNED" || exit 1
            else
                echo "[align] usando $ALIGNED"
            fi
            IN_BAG="$ALIGNED"
        fi
        ;;
esac

NODE_LOG="${OUT%/}.node.log"
REC_LOG="${OUT%/}.record.log"
PLAY_LOG="${OUT%/}.play.log"

rm -rf "$OUT"
mkdir -p "$(dirname "$OUT")"

NODE_BIN=/home/mirena/mirena_ws/install/mirena_state/lib/mirena_state/mirena_state

NODE=""
REC=""
PLAY=""

stop_all() {
    if [ -n "$PLAY" ]; then kill -INT "$PLAY" 2>/dev/null || kill -TERM "$PLAY" 2>/dev/null || true; wait "$PLAY" 2>/dev/null || true; PLAY=""; fi
    if [ -n "$NODE" ]; then kill -INT "$NODE" 2>/dev/null || kill -TERM "$NODE" 2>/dev/null || true; wait "$NODE" 2>/dev/null || true; NODE=""; fi
    if [ -n "$REC" ]; then kill -INT "$REC" 2>/dev/null || true; kill -TERM "$REC" 2>/dev/null || true; wait "$REC" 2>/dev/null || true; REC=""; fi
}
trap 'stop_all; exit 130' INT TERM

ros2 bag record -o "$OUT" --storage mcap --use-sim-time \
    /state/car /tf /health > >(tee "$REC_LOG") 2>&1 &
REC=$!
sleep 2

"$NODE_BIN" --ros-args \
    -r __node:=mirena_state \
    -p use_sim_time:=true \
    -p odom_timeout_sec:=0.5 \
    -p imu_timeout_sec:=0.2 \
    -p max_predict_dt_sec:=0.1 \
    -p process_noise_diag:='[0.01, 0.01, 0.005, 0.5, 0.5, 0.5]' \
    -p odom_meas_floor_diag:='[1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9]' \
    -p init_cov_floor_diag:='[0.01, 0.01, 0.001, 0.01, 0.01, 0.001]' \
    -p gyro_meas_var:=1.0e-5 \
    -p ins_reanchor_m:=12.0 \
    -p use_bicycle_model:=true \
    -p steering_degrees:=true \
    -p steering_sign:=1.0 \
    -p steering_ratio:=1.35 \
    -p steering_offset_rad:=0.030 \
    -p wheel_radius:=0.191 > >(tee "$NODE_LOG") 2>&1 &
NODE=$!
sleep 3

PLAY_ARGS=(
    "$IN_BAG"
    --rate "$RATE"
    --clock-topics-all
    --start-paused
    --disable-keyboard-controls
    --topics /sensors/imu /sensors/imu/odom /sensors/wheel_speeds /control/can
)
if [ -n "$DUR" ]; then
    PLAY_ARGS+=(--playback-duration "$DUR")
fi
ros2 bag play "${PLAY_ARGS[@]}" > >(tee "$PLAY_LOG") 2>&1 &
PLAY=$!
sleep 3

for _ in 1 2 3 4 5; do
    if ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume "{}" >/dev/null 2>&1; then
        break
    fi
    sleep 1
done
echo "playback resumed at $(date +%T)"

wait "$PLAY"
echo "play exit=$?"
PLAY=""

# let the last IMU samples flush through the filter, then close cleanly
sleep 3
stop_all
trap - INT TERM
echo "recorded: $OUT"
