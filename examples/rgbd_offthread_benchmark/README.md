# RGBD off-thread publish benchmark

Reproduction harness for the **off-thread RGBD publish** feature
(`GZ_SENSORS_RGBD_OFFTHREAD`) and its worker-pool tuning. It measures how many
concurrent RGBD cameras can hit their requested update rate, with the publish
tail running inline (off) versus on a worker pool (on), and sweeps the worker
count and queue depth to locate the optimum.

For the model and the analysis of the results these scripts produce, see the
tutorial: \ref rgbdoffthread "Off-thread RGBD publish: tuning the worker pool"
(`tutorials/rgbd_offthread_publish.md`).

## Requirements

- A Gazebo install whose `gz-sensors` includes the off-thread tail feature
  (the branch that adds `GZ_SENSORS_RGBD_OFFTHREAD`). If it is in a colcon
  workspace, point the scripts at it with `GZ_SETUP=/path/to/install/setup.bash`;
  otherwise just have `gz` on `PATH`.
- `python3` (only to regenerate worlds with `gen_world.py`).
- `nvidia-smi` is optional — GPU columns read `NA` without it.

## Files

| File | Purpose |
|------|---------|
| `gen_world.py` | Generate an N-camera world: `gen_world.py N [W H [RATE]] > out.sdf` |
| `worlds/rgbd16.sdf` | Pre-generated 16-camera, 640×480, 60 Hz world (render-bound) |
| `perf8.sh` | One throughput measurement (OFF or ON, per the env) |
| `sweep_workers.sh` | Drive `perf8.sh` across a workers × queue matrix |

## Quick start

Single OFF-vs-ON comparison on the bundled 16-camera world:

```bash
# baseline: tail runs inline on the render thread
./perf8.sh /tmp/off

# off-thread tail, default pool (3 workers, queue depth 2)
GZ_SENSORS_RGBD_OFFTHREAD=1 ./perf8.sh /tmp/on
```

Each run prints a `RESULT` line with per-camera Hz, aggregate Hz, server CPU and
GPU util. If your Gazebo lives in a workspace:

```bash
GZ_SETUP=~/ws/install/setup.bash GZ_SENSORS_RGBD_OFFTHREAD=1 ./perf8.sh /tmp/on
```

## Worker / queue sweep

```bash
./sweep_workers.sh                 # writes a TSV to a fresh mktemp dir
OUTDIR=/tmp/sweep ./sweep_workers.sh   # ...or a directory you choose
```

The sweep runs the OFF baseline, a worker sweep `{1,2,3,4,6,8,12}` at queue
depth 2, and a few queue-depth variations, then prints the table. Expect the
benefit to saturate at a small worker count and to *regress* past it — that is
the bandwidth / scheduling-contention ceiling described in the tutorial.

## Tuning the workload

The feature only helps when the render thread is the bottleneck. On an idle
machine a light world (few cameras, capped rate) already runs at its update-rate
ceiling, so OFF and ON look identical. Make the world render-bound:

```bash
# 24 cameras, 640×480, uncapped (60 Hz request) -> render thread saturates
python3 gen_world.py 24 640 480 60 > worlds/rgbd24.sdf
WORLD=worlds/rgbd24.sdf NCAM=24 ./sweep_workers.sh
```

## Environment variables

The feature itself (read by `gz-sensors`):

| Variable | Default | Meaning |
|----------|---------|---------|
| `GZ_SENSORS_RGBD_OFFTHREAD` | unset (off) | `1` moves the publish tail off the render thread |
| `GZ_SENSORS_OFFTHREAD_WORKERS` | `3` | worker threads in the shared pool |
| `GZ_SENSORS_OFFTHREAD_QUEUE` | `2` | scratch slots per camera (frames in flight) |

The harness (read by the scripts):

| Variable | Default | Meaning |
|----------|---------|---------|
| `GZ_SETUP` | unset | Gazebo environment to `source` before running |
| `WORLD` | `worlds/rgbd16.sdf` | world file |
| `NCAM` | `16` | cameras to probe (must match the world) |
| `SUBTOPICS` | `image` | per-camera topics to subscribe (`perf8.sh`) |
| `WARMUP` / `SAMPLE` | `15`/`25` (`perf8`), `10`/`15` (`sweep`) | timing windows |
| `OUTDIR` | mktemp | results directory (`sweep_workers.sh`) |
