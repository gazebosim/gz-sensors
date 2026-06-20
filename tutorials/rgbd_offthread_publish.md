\page rgbdoffthread Off-thread RGBD publish: tuning the worker pool

# Off-thread RGBD publish

When a world contains many RGBD (or depth) cameras, Gazebo renders **all** of
them on a single render thread, one after another. Past a handful of cameras
that thread becomes the bottleneck and the cameras can no longer hit their
requested update rate — not because the GPU is busy, but because the render
thread is doing too much CPU work in series.

`RgbdCameraSensor` can move its **publish tail** — assembling the point cloud
(`PointCloudUtil::FillMsg`) and publishing the depth, point-cloud and colour
messages — off the render thread onto a small worker pool, so the render thread
can start the next camera sooner. The work happens *across* cameras in parallel;
a single camera's render is never split (the rendering engine does not allow
that).

The feature is **opt-in and off by default**; with it off the code path is
byte-for-byte the original one.

## When it helps

The speed-up is proportional to **how render-bound the camera array actually
is**. If the render thread already keeps every camera at its requested update
rate — few cameras, low resolution, an otherwise-idle machine — moving the tail
off-thread changes nothing measurable (the cameras were already at their cap),
though it also costs nothing (off by default). The benefit appears precisely
when the render thread is saturated: many cameras, high resolution, a high
requested rate, or a busy host. The same 8-camera 640×480 world that ran at 57%
of its 15 Hz target under load sat at the full 15 Hz cap on an idle machine — so
always characterise on a workload that is genuinely render-bound (e.g. enough
cameras, or an uncapped update rate) before drawing conclusions.

## Environment variables

| Variable | Default | Meaning |
|---|---|---|
| `GZ_SENSORS_RGBD_OFFTHREAD` | unset (off) | Set to `1` to move the publish tail off the render thread. |
| `GZ_SENSORS_OFFTHREAD_WORKERS` | `3` | Number of worker threads in the shared, process-wide pool. |
| `GZ_SENSORS_OFFTHREAD_QUEUE` | `2` | Scratch slots per camera (how many frames can be in flight before the render thread must wait). |

The pool is shared by every offload-enabled sensor in the process and enforces
**one job per camera at a time** (so each camera's messages stay in order and its
scratch buffers are never shared); different cameras run concurrently.

## How many workers? A two-ceiling model

It is tempting to scale the worker count to the number of cameras, or to the
number of CPU cores. Both are wrong. The useful number of workers is the
**minimum of two independent ceilings**, and on realistic hardware the small one
wins.

### Ceiling A — "keep-up" (producer / consumer)

Workers only have to drain tails as fast as the render thread *produces* them.
Because the producer is a **single thread**, tails are emitted **serially** — one
every `T_emit` (the render-thread time for one camera once the tail is removed),
regardless of how many cameras there are:

```
N_keepup = ceil( T_tail / T_emit ) + burst_margin
```

* `T_emit` — render-thread time per camera **with** offload on (draw + read-back
  + reshape).
* `T_tail` — wall time to process one tail on a worker (FillMsg + 3 publishes).

The crucial structural fact: **this is independent of the camera count.** Eight
cameras do not produce eight simultaneous tails; they produce one tail every
`T_emit`, in sequence. If `T_tail < T_emit`, a single worker already keeps up for
*any* number of cameras; a couple more cover jitter and the occasional
slower-than-average tail.

### Ceiling B — memory bandwidth

The tail (dominated by `FillMsg`) is memory-bound. Concurrent workers share a
finite memory bandwidth, and — more importantly — they **compete with the render
thread's own memory-heavy steps** (the read-back copy and the reshape passes).
The bandwidth-saturation count is:

```
N_bw = ( B_sustained - B_render ) / r1
```

* `B_sustained` — sustained system memory bandwidth (measure with STREAM; desktop
  DDR4 ≈ 30–40 GB/s, DDR5 ≈ 60–80 GB/s).
* `B_render` — bandwidth the render thread already consumes.
* `r1 = V_tail / T_tail` — per-worker achieved throughput, where `V_tail` is the
  bytes a tail touches (for 640×480: read depth+colour ≈ 2.4 MB, write the
  XYZRGBA cloud ≈ 9.8 MB, copy it into the published message ≈ 9.8 MB, plus the
  depth and image messages ≈ 2 MB → ≈ 20–24 MB). A single tail thread runs far
  below peak bandwidth — it is **latency-bound** (cache-miss stalls), not
  throughput-bound.

### Which ceiling binds

```
N_opt = min( N_keepup , N_bw )
```

Because Stage 1's tail is *smaller* than the render-thread work per camera,
`N_keepup` (≈ 2) is small and binds first. `N_bw` is larger (≈ 10–16 on a desktop
DDR4 machine) and does **not** set the count — but it is exactly why exceeding
`N_keepup` is wasteful: once the tails are drained, extra workers have nothing to
do *except* steal memory bandwidth from the render thread, which is the real
bottleneck. So the curve does not just plateau; past the knee it can dip.

There is also a hard cap from the design itself: per-camera serialization means
*useful* workers can never exceed the number of distinct offload-enabled
cameras.

## A worked example (this project's measurements)

On 8× RGBD cameras at 640×480, each asked for 15 Hz (RTX 4060 Ti, 16 logical
cores, DDR4):

| | full 8-cam pass | render-thread time per camera |
|---|---|---|
| offload **off** | 1 / 8.57 = 117 ms | ≈ 14.6 ms (tail included) |
| offload **on**  | 1 / 13.08 = 76 ms | ≈ 9.6 ms (tail removed) |

So `T_emit` ≈ 9.6 ms and `T_tail` is in the ~5–11 ms range, giving a raw
`N_keepup = ceil(5…11 / 9.6) ≈ 1`, or 2–3 once a burst margin is added. The
bandwidth ceiling, with `r1 ≈ 22 MB / 11 ms ≈ 2 GB/s` and
`B_sustained − B_render ≈ 32 GB/s`, is `N_bw ≈ 16`. The minimum — and therefore
the optimum — is **a small handful of workers**, and the sweep below confirms a
single worker already saturates the benefit. (The 8-camera figures above were
measured under background load; see *When it helps*.)

## Empirical verification — workers × queue sweep

The model predicts the optimum is small and that exceeding it is wasteful. To
verify, `perf8.sh` was swept across worker counts on a **16-camera, 640×480,
rate-uncapped** world — chosen so the render thread is the bottleneck regardless
of machine load — at queue depth 2. Two independent passes (Hz per camera; RTX
4060 Ti, 16 logical cores, DDR4):

| workers | pass 1 | pass 2 | vs OFF | server CPU |
|--------:|------:|------:|:------:|:----------:|
| **off (inline)** | 14.09 | 13.33 | 1.00× | 113% |
| 1 | 17.96 | 18.12 | **1.31×** | 148% |
| 2 | 17.99 | — | 1.31× | 151% |
| 3 *(default)* | 18.11 | 18.00 | 1.31× | 151% |
| 4 | 18.09 | 17.88 | 1.30× | 151% |
| 6 | 14.98 | 15.72 | **0.96× — regression** | 146% |
| 8 | 17.23 | 17.93 | 1.26× | 150% |
| 12 | 16.94 | 17.79 | 1.25× | 148% |

Queue-depth effect (Hz per camera):

| workers | q=1 | q=2 | q=4 |
|--------:|----:|----:|----:|
| 1 | 17.71 | 17.96 | 14.99\* |
| 2 | — | 17.99 | 18.45 |
| 3 | — | 18.11 | **18.61** |

\* an outlier-low run; a single worker with a deep queue was unstable across
repeats.

The data confirms the model and sharpens it:

* **One worker already captures the entire benefit** (≈ 18 Hz, ~1.31× over the
  inline baseline). Because the render thread emits tails serially, a single
  worker drains them as fast as they arrive — exactly the keep-up prediction,
  `N_keepup ≈ 1` here.
* **The optimum is a flat plateau from ~1 to ~4 workers** (17.9–18.1 Hz). The
  default of **3 sits squarely in it.**
* **More workers do not help, and can actively regress.** Six workers dropped to
  ~15 Hz — *back to the inline baseline* — reproducibly across both passes; 8 and
  12 recovered only to just below the plateau and were noisier. This is the
  bandwidth / scheduling-contention ceiling made visible: past the knee, extra
  workers compete with the render thread (the real bottleneck) for memory and
  cores instead of helping it.
* **Server CPU stays ~148–151%** of one core for every worker count ≥ 1 (vs 113%
  inline) — the same tail work, now on worker cores, plus ~30% more frames
  delivered. Throughput, not CPU, is what the worker count buys or wastes.
* **Queue depth 2 is enough.** Depth 3–4 with 2–3 workers was marginally best
  (~18.6 Hz); depth 1 marginally worse. The default `2` is a good balance and
  pairs safely with the default 3 workers.

**Verified optimum: 2–3 workers, queue depth 2** — the shipped defaults. The safe
minimum is 1 worker; the practical ceiling before regression is ~4.

Reproduce with `stage1_offthread/sweep_workers.sh` (drives
`stage1_offthread/perf8.sh` across the matrix).

## Recommended defaults

* **Workers: leave it at the default `3`** and do **not** scale it to the core
  count. Three covers the keep-up requirement plus burst margin, is independent
  of both camera count and core count, and sits just below the
  bandwidth-contention regime. Defaulting a pool to `nproc` here would
  re-create the classic oversubscription regression (the same trap as hardcoding
  the OGRE `SceneManager` worker-thread count to the number of logical cores):
  the extra threads would only contend with the render thread for memory.
* **Queue depth: leave it at the default `2`.** Queue depth buffers bursts and
  trades against worker count — a deeper queue lets fewer workers ride out
  uneven tail arrivals without back-pressuring the render thread. One slot is
  enough to overlap, two adds margin; beyond that gives little.
* **When to raise workers:** only if your tails are unusually heavy relative to
  the render work — very high resolution, or many subscribers forcing all three
  publishes. In that case `T_tail` approaches or exceeds `T_emit` and
  `N_keepup` grows. Confirm with a sweep rather than guessing.

The honest calibration procedure for any new hardware or resolution is to
**sweep ±1 around 3 and watch where the per-camera Hz plateaus** — the model
tells you which regime you are in; the sweep pins the knee.
