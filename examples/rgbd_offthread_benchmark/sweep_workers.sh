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
# sweep_workers.sh — verify the worker-count / queue-depth model.
#
# Runs perf8.sh across a (workers x queue) matrix plus the OFF baseline and
# tabulates per-cam Hz, aggregate Hz, server CPU and GPU into a TSV. Each cell
# is one perf8 run.
#
# Configuration (env, all optional):
#   GZ_SETUP   forwarded to perf8.sh (Gazebo environment to source)
#   WORLD      world file (default: ./worlds/rgbd16.sdf next to this script).
#              Use a render-bound world (many cameras / uncapped rate) so the
#              offload benefit and the worker knee are actually visible.
#   NCAM       number of cameras to probe (default: 16, matching rgbd16.sdf)
#   OUTDIR     where to write results (default: a fresh mktemp dir)
#   WARMUP     per-cell warmup seconds (default: 10)
#   SAMPLE     per-cell sample seconds  (default: 15)
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

OUT="${OUTDIR:-$(mktemp -d -t rgbd_offthread_sweep.XXXXXX)}"
mkdir -p "$OUT"
RES="$OUT/table.tsv"
printf "config\tworkers\tqueue\tper_cam_hz\taggregate_hz\tserver_cpu_pct\tgpu_pct\n" > "$RES"

export WORLD="${WORLD:-$HERE/worlds/rgbd16.sdf}" NCAM="${NCAM:-16}"
export WARMUP="${WARMUP:-10}" SAMPLE="${SAMPLE:-15}"
echo "sweep: world=$(basename "$WORLD") ncam=$NCAM results=$OUT"

run(){  # label workers queue on(1/0)
  local label="$1" w="$2" q="$3" on="$4"
  if [ "$on" = "1" ]; then
    export GZ_SENSORS_RGBD_OFFTHREAD=1 GZ_SENSORS_OFFTHREAD_WORKERS="$w" GZ_SENSORS_OFFTHREAD_QUEUE="$q"
  else
    unset GZ_SENSORS_RGBD_OFFTHREAD GZ_SENSORS_OFFTHREAD_WORKERS GZ_SENSORS_OFFTHREAD_QUEUE 2>/dev/null || true
  fi
  echo ">>> $label (workers=$w queue=$q on=$on)"
  local line; line=$(bash "$HERE/perf8.sh" "$OUT/$label" 2>/dev/null | grep '^RESULT' || echo "RESULT FAILED")
  local pc ag cpu gpu
  pc=$(echo "$line"  | grep -oE 'per-cam-avg=[0-9.]+' | cut -d= -f2); : "${pc:=NA}"
  ag=$(echo "$line"  | grep -oE 'aggregate=[0-9.]+'   | cut -d= -f2); : "${ag:=NA}"
  cpu=$(echo "$line" | grep -oE 'serverCPU=[0-9]+'    | cut -d= -f2); : "${cpu:=NA}"
  gpu=$(echo "$line" | grep -oE 'GPU=[0-9.]+'         | cut -d= -f2); : "${gpu:=NA}"
  printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\n" "$label" "$w" "$q" "$pc" "$ag" "$cpu" "$gpu" | tee -a "$RES"
}

# baseline (off-thread disabled)
run off - - 0
# worker sweep at queue=2 (find the keep-up knee; model predicts a small number)
for w in 1 2 3 4 6 8 12; do run "w${w}_q2" "$w" 2 1; done
# queue-depth effect: most likely to matter where workers are scarce
run "w1_q1" 1 1 1
run "w1_q4" 1 4 1
run "w2_q4" 2 4 1
run "w3_q4" 3 4 1

echo "=== SWEEP COMPLETE ($OUT/table.tsv) ==="
column -t -s $'\t' "$RES"
