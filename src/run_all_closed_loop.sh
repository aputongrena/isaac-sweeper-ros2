#!/usr/bin/env bash
# sweeper_ros2_v1: PhysX diff-drive sweeper + closed-loop ROS2 avoidance, 20 s @30fps
set +u
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROBE=${ROS2_PRELOAD_DIR:-$DIR/ros2_probe}
OUT=$DIR/out
PY=${ISAACLAB_PYTHON:-python3}
RESULT=$DIR/RESULT.txt

cd "$DIR" || exit 1
: > "$RESULT"
echo "[stage] start $(date '+%F %T')" | tee -a "$RESULT"

# ---- env: bundled ROS2 humble runtime ----
# shellcheck disable=SC1091
source "$PROBE/ros2_env.sh"
export ROS_DOMAIN_ID=77
export OMNI_KIT_ACCEPT_EULA=YES
export ISAACSIM_ASSET_REGION_PROFILE=china

# ---- pick a free RTX 5090 (used<4000 MiB, util<20%, lowest memory first) ----
GPU=$(nvidia-smi --query-gpu=index,memory.used,utilization.gpu --format=csv,noheader,nounits \
      | awk -F', *' '$2 < 4000 && $3 < 20 {print $2, $1}' | sort -n | head -1 | awk '{print $2}')
if [ -z "${GPU:-}" ]; then echo "[stage] NO FREE GPU" | tee -a "$RESULT"; exit 2; fi
export CUDA_VISIBLE_DEVICES=$GPU
echo "[stage] GPU=$GPU" | tee -a "$RESULT"

rm -rf "$OUT"; mkdir -p "$OUT/frames_main"

# ---- process A: simulation ----
setsid nohup env CUDA_VISIBLE_DEVICES=$GPU OMNI_KIT_ACCEPT_EULA=YES \
  ISAACSIM_ASSET_REGION_PROFILE=china ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  "$PY" "$DIR/ros_sweeper_sim.py" --out "$OUT" --frames 600 --fps 30 > "$DIR/sim.log" 2>&1 < /dev/null &
sleep 3
SIM_PID=$(pgrep -f "ros_sweeper_sim.py --out" | head -1)
echo "[stage] SIM_PID=${SIM_PID:-none}" | tee -a "$RESULT"
echo "${SIM_PID:-0}" > "$OUT/pids.txt"

# wait for the sim to start publishing (max 120 s)
for i in $(seq 1 120); do
  if grep -q "RUNNING pid" "$DIR/sim.log" 2>/dev/null; then break; fi
  sleep 1
done
grep -m1 "RUNNING pid" "$DIR/sim.log" | tee -a "$RESULT"

# ---- process B: external avoidance controller ----
setsid nohup env ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  "$PY" "$DIR/ros_avoid_controller.py" "$OUT" > "$DIR/ctrl.log" 2>&1 < /dev/null &
sleep 2
CTRL_PID=$(pgrep -f "ros_avoid_controller.py" | head -1)
echo "[stage] CTRL_PID=${CTRL_PID:-none}" | tee -a "$RESULT"
echo "${CTRL_PID:-0}" >> "$OUT/pids.txt"
grep -m1 "RUNNING pid" "$DIR/ctrl.log" | tee -a "$RESULT"

# ---- wait for sim to finish ----
for i in $(seq 1 900); do
  if ! kill -0 "$SIM_PID" 2>/dev/null; then break; fi
  sleep 1
done
echo "[stage] sim exited $(date '+%T')" | tee -a "$RESULT"
for i in $(seq 1 60); do kill -0 "$CTRL_PID" 2>/dev/null || break; sleep 1; done
kill "$CTRL_PID" 2>/dev/null || true

NFRAMES=$(ls "$OUT/frames_main" 2>/dev/null | wc -l)
echo "[stage] frames=$NFRAMES" | tee -a "$RESULT"

# ---- encode ----
MP4=$OUT/sweeper_ros2_v2_30fps.mp4
ffmpeg -y -v warning -framerate 30 -i "$OUT/frames_main/%06d.png" \
  -c:v libx264 -preset medium -pix_fmt yuv420p -b:v 6M -minrate 4M -maxrate 9M -bufsize 14M \
  -x264-params nal-hrd=cbr:force-cfr=1 "$MP4" >> "$DIR/encode.log" 2>&1
echo "[stage] mp4 rc=$? -> $MP4" | tee -a "$RESULT"

# ---- report ----
"$PY" "$DIR/report_gen.py" "$OUT" | tee -a "$RESULT"
echo "--- ffprobe ---" | tee -a "$RESULT"
ffprobe -v error -select_streams v:0 -show_entries stream=width,height,r_frame_rate,nb_frames \
  -show_entries format=bit_rate,duration -of default=noprint_wrappers=1 "$MP4" | tee -a "$RESULT"
echo "ALL_DONE" >> "$RESULT"
