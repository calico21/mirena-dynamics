#!/usr/bin/env bash
set +u
source /opt/ros/jazzy/setup.bash
source /home/mirena/mirena_ws/install/setup.bash
export ROS_DOMAIN_ID=42
cd /home/mirena/mirena_ws/imu_compare

DUR=470
rm -f sim_imu.csv sim_control.csv

python3 record_csv.py --out-prefix sim --imu-topic /sensors/imu \
  --control-topic /control --control-steer-units rad --duration "$DUR" \
  > /tmp/opencode/sim_rec4.log 2>&1 &
R=$!

python3 control_adapter.py --in /control_raw --out /control \
  --steer-units deg --duration "$DUR" \
  > /tmp/opencode/sim_adapter4.log 2>&1 &
A=$!

sleep 2
ros2 bag play /home/mirena/mirena_ws/rosbag2_2026_10_06-18_02_11 \
  --topics /control/can --remap /control/can:=/control_raw \
  --rate 1.0 --disable-keyboard-controls \
  > /tmp/opencode/sim_play4.log 2>&1
echo "bag_play_exit=$?"

wait "$R"
echo "rec_exit=$?"
wait "$A"
echo "adapter_exit=$?"
