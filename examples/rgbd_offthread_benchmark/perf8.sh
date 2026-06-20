#!/usr/bin/env bash
#
# Copyright (C) 2026 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# perf8.sh OUTDIR — RGBD-camera publish-throughput probe for the off-thread tail.
#
# Measures per-camera publish Hz (gz topic --frequency), the server process CPU
# (/proc jiffies, server-isolated) and GPU util/VRAM (nvidia-smi, if present)
# over a steady-state window. Honors GZ_SENSORS_RGBD_OFFTHREAD and
# GZ_SENSORS_OFFTHREAD_* from the caller's env, so the SAME script measures the
# OFF baseline and the ON path.
#
# `gz topic -f` subscribes without printing payloads: it both keeps the sensor
# active (gz-sim disables sub-less sensors) and reports "average rate:" per
# window. An image (color) subscriber cascades to the point connection, so the
# full RGBD tail (FillMsg + 3 publishes) runs server-side.
#
# Configuration (env):
#   GZ_SETUP   path to a Gazebo environment to source (e.g. a colcon
#              install/setup.bash that provides the off-thread-capable
#              gz-sensors). If unset, `gz` must already be on PATH.
#   WORLD      world file (default: ./worlds/rgbd16.sdf next to this script)
#   NCAM       number of cameras to probe (default: 16, matching rgbd16.sdf)
#   SUBTOPICS  per-camera sub-topics to probe (default: image)
#   WARMUP     warmup seconds before sampling (default: 15)
#   SAMPLE     steady-state sampling seconds (default: 25)
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Optionally source a Gazebo environment that provides gz + the patched
# gz-sensors; otherwise rely on whatever `gz` is already on PATH.
if [ -n "${GZ_SETUP:-}" ]; then set +u; . "$GZ_SETUP"; set -u; fi
command -v gz >/dev/null 2>&1 || {
  echo "error: 'gz' not on PATH. Install Gazebo, or set GZ_SETUP=/path/to/install/setup.bash" >&2
  exit 1; }

OUT="${1:?usage: perf8.sh OUTDIR}"; mkdir -p "$OUT"; rm -f "$OUT"/hz_* "$OUT"/gpu.csv
WORLD="${WORLD:-$HERE/worlds/rgbd16.sdf}"
NCAM="${NCAM:-16}"
SUBTOPICS="${SUBTOPICS:-image}"
WARMUP="${WARMUP:-15}"
SAMPLE="${SAMPLE:-25}"
export GZ_PARTITION="perf8_$$"

pkill -x gz-sim-main 2>/dev/null; pkill -f "gz topic -f" 2>/dev/null; sleep 1
echo "== perf8: world=$(basename "$WORLD") ncam=$NCAM subs='$SUBTOPICS' warmup=${WARMUP}s sample=${SAMPLE}s =="
echo "   GZ_SENSORS_RGBD_OFFTHREAD=${GZ_SENSORS_RGBD_OFFTHREAD:-<unset>} OFFTHREAD_WORKERS=${GZ_SENSORS_OFFTHREAD_WORKERS:-<default>} OFFTHREAD_QUEUE=${GZ_SENSORS_OFFTHREAD_QUEUE:-<default>}"

gz sim -v1 -s -r --render-engine ogre2 "$WORLD" > "$OUT/server.log" 2>&1 &
LAUNCH=$!
sleep 8

# subscribers: run for the full warmup+sample; payload-free frequency probes.
RUNFOR=$((WARMUP + SAMPLE + 6))
for n in $(seq 0 $((NCAM-1))); do
  for t in $SUBTOPICS; do
    timeout "$RUNFOR" gz topic -f -t "/rgbd_${n}/${t}" > "$OUT/hz_${n}_${t}.log" 2>&1 &
  done
done
sleep 2

SRV=$(pgrep -x gz-sim-main | head -1)
[ -z "$SRV" ] && SRV=$(pgrep -x gz-sim-server | head -1)
[ -z "$SRV" ] && SRV=$(pgrep -f "$(basename "$WORLD")" | grep -v "^$LAUNCH\$" | head -1)
[ -z "$SRV" ] && SRV=$LAUNCH
echo "   server pid=$SRV"

echo "   warmup ${WARMUP}s..."; sleep "$WARMUP"
echo "   sampling ${SAMPLE}s..."
nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader -lms 1000 > "$OUT/gpu.csv" 2>/dev/null &
NV=$!
CLK=$(getconf CLK_TCK)
J0=$(awk '{print $14+$15}' /proc/$SRV/stat 2>/dev/null)
sleep "$SAMPLE"
J1=$(awk '{print $14+$15}' /proc/$SRV/stat 2>/dev/null)
kill $NV 2>/dev/null

kill -INT $LAUNCH 2>/dev/null; sleep 2
pkill -x gz-sim-main 2>/dev/null; pkill -f "gz topic -f" 2>/dev/null; sleep 1

CPU=$(awk -v a="$J0" -v b="$J1" -v s="$SAMPLE" -v c="$CLK" 'BEGIN{if(a!=""&&b!="")printf "%.0f",100.0*(b-a)/c/s; else print "NA"}')
GPU=$(awk -F'[ ,%]+' '/[0-9]/{s+=$1;n++} END{if(n)printf "%.1f",s/n; else print "NA"}' "$OUT/gpu.csv")
VRAM=$(awk -F'[ ,%]+' '/[0-9]/{s+=$3;n++} END{if(n)printf "%.0f",s/n; else print "NA"}' "$OUT/gpu.csv")

# per-cam Hz: average the steady-state half of the "average rate:" lines (drop
# the first half ~= warmup transient). Measured on the first SUBTOPIC per cam.
PRIMARY=$(echo "$SUBTOPICS" | awk '{print $1}')
total=0; cnt=0
for n in $(seq 0 $((NCAM-1))); do
  f="$OUT/hz_${n}_${PRIMARY}.log"; [ -f "$f" ] || continue
  hz=$(awk '/average rate:/{v[++m]=$3} END{if(m){st=int(m/2)+1; for(i=st;i<=m;i++){s+=v[i];k++}; if(k)printf "%.2f",s/k; else print "NA"} else print "NA"}' "$f")
  printf "   cam %d (%s): %s Hz\n" "$n" "$PRIMARY" "$hz"
  if [ "$hz" != "NA" ]; then total=$(awk -v t="$total" -v h="$hz" 'BEGIN{print t+h}'); cnt=$((cnt+1)); fi
done
avg=$(awk -v t="$total" -v c="$cnt" 'BEGIN{if(c)printf "%.2f",t/c; else print "NA"}')
agg=$(awk -v t="$total" 'BEGIN{printf "%.1f",t}')
echo "-----"
echo "RESULT per-cam-avg=${avg}Hz aggregate=${agg}Hz cams=${cnt} serverCPU=${CPU}%ofcore GPU=${GPU}% VRAM=${VRAM}MiB"
