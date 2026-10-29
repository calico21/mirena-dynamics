# mirena_perception

Colourless cone detection from the LiDAR alone.

The car does not read cone colour. What survives is *size*, because that is
geometry too: a blob tall enough to be a big orange cone comes out as
`cone_big_orange` and everything else as `unknown_cone`. That single distinction
is load bearing — the four big cones on the start line are what the unknown-track
planner roots the track on.

## How it works

Three ideas, and the detector is those three.

**The ground under a candidate is fitted around it, never through it.** A plane
is fitted to the lowest point per cell in a ring from 0.55 to 2.6 m, with the
candidate itself outside the ring. Every height is then measured against that
plane.

This is the part worth understanding, because the obvious alternative quietly
destroys the measurement. Fit a plane through the patch a cone stands in — which
is what any ground-segmentation algorithm does — and the cone's own base falls
inside the plane's thickness and is classified as ground. Measured on this
sensor, that costs a cone at 3 m two thirds of its points and leaves only the
tip, so a 0.325 m cone reports 0.21 m. At that point "height above ground" no
longer separates a cone from a bump in the asphalt, and no amount of tuning
downstream will recover it. Fitted around the cone instead, nothing the cone does
can move its own reference: the same cones measure 0.32–0.34 m.

**A cone is isolated.** What fools a size-and-height test is not usually a
cone-shaped object; it is a piece of something larger — rough ground, gravel,
vegetation, a bank — which any clustering with a fixed radius chops into
cone-sized pieces. The giveaway is the surroundings: count the points standing up
in a ring from 0.45 to 1.1 m, relative to the candidate's own. On all 34
hand-labelled cones that ratio is zero.

**The ground a cone stands on is smooth.** The standard deviation of the plane
fit's residual is 0.5–1.7 cm under a real cone and 3.3–8.9 cm under the rough
stuff, which separates the two populations almost on its own.

Candidates are local maxima on a 0.25 m grid, not connected components. At 5–8 m
the ground itself sits a few centimetres above a coarse minimum, so growing a
cluster by connectivity swallows the cone into that field and then rejects the
whole thing for being too wide. A peak cannot be swallowed.

There is no ground/non-ground classifier, no voxel grid, no k-d tree, no
Euclidean clustering and no return-count model.

## Interface

| Direction | Topic | Type |
| --- | --- | --- |
| in | `/sensors/lidar/points` | `sensor_msgs/PointCloud2` |
| out | `/perception/cones` | `mirena_common/EntityList` in `car/cog` |
| out | `health` | `mirena_common/Heartbeat` at 10 Hz |
| debug | `/perception/debug/clustered_points` | points that became cones, coloured per cone |
| debug | `/perception/debug/standing_points` | everything standing above the ground; grey means rejected |

Both debug topics are published only when something is subscribed and
`enable_debug` is true.

Detections carry a 2×2 position covariance, anisotropic and range-dependent:
along the beam a LiDAR measures a range directly, across it there is only the
pointing angle, so that error grows with distance. The numbers are measured
rather than assumed — repeat observations of the same cone land within 0.33 m of
each other, 0.56 m at the 90th percentile — and reporting them honestly is what
lets SLAM stop treating a cone at 25 m like one at 3 m.

Colour is never measured, so `confidence` is always `1.0`: a blob either passes
the tests or is dropped, and there is no graded score to invent.

## Ground truth

The thresholds are not guesses. Two LiDAR frames from
`rosbag2_2026_08_05-20_26_23`, at t = 20 s and t = 90 s, were hand-labelled cone
by cone from the raw points — 34 cones between them. Against those labels:

| | published | real cones | precision | recall |
| --- | --- | --- | --- | --- |
| this detector | 30 | 30 | 100% | 30/34 |

Every cone inside 20 m was found. The four misses are all beyond 21 m, where a
cone is a handful of returns.

Re-measure this when the sensor or its mounting changes, and re-label rather than
re-tune by eye: the failure mode of a geometric detector is that a threshold
stops discriminating without ever failing loudly.

## Parameters

The two to check first:

| Parameter | Default | |
| --- | --- | --- |
| `max_range` | `30.0` | Furthest a cone is claimed, metres. |
| `ego_half_length` / `ego_half_width` | `2.0` / `1.2` | Half-extents of the box that is the car looking at itself. On a hoop-mounted sensor that is a large permanent slab at cone height; nothing is lost by cutting it, because the sensor cannot see the track that close anyway. |

The two that do the discriminating, and the ones to reach for if the detector is
either flooding or going quiet:

| Parameter | Default | |
| --- | --- | --- |
| `max_roughness` | `0.045` | How rough the ground around a candidate may be, as the plane fit's residual in metres. Lower it if rough ground is being reported as cones. |
| `max_ring_ratio` | `1.2` | How much of the ring around a candidate may also be standing up, relative to the candidate itself. Same lever, from the other direction. |

Shape, all measured against the fitted local plane: `min_top` (`0.18`),
`max_top` (`0.58`), `core_radius` (`0.32`), `stand_height` (`0.10`),
`max_footprint` (`0.55`), `min_points` (`2`), `big_cone_min_height` (`0.40`).
The defaults bracket both cone sizes — small 228 × 228 × 325 mm, big orange
285 × 285 × 505 mm — with margin for how sparsely a LiDAR samples them.
`big_cone_min_height` sits nearer the small cone than halfway on purpose: a blob
is the part of a cone the sensor happened to catch, and its top under-reads the
real apex far more often than it over-reads it.

Candidate finding, which only decides what is worth a proper look: `coarse_cell`
(`0.40`), `coarse_ring` (`1.20`), `candidate_height` (`0.08`), `peak_cell`
(`0.25`), `peak_height` (`0.15`). The ground plane: `ring_inner` (`0.55`),
`ring_outer` (`2.60`), `ring_cell` (`0.40`).

How well a cone is located: `range_sigma` (`0.08`), `bearing_sigma` (`0.010`),
`cluster_spread` (`0.30`). Under-reporting these is not a cosmetic error — a
gate built from covariances four times too tight rejects its own landmarks and
spawns a duplicate beside every real cone.

Plus `enable_debug` (`true`).

The heartbeat carries `status_state`, `frames_processed`, `cones_last_frame` and
`seconds_since_frame`, and adds `standing_last_frame`, `candidates_last_frame`,
`total_ms`, the per-stage split (`level_ms`, `coarse_ms`, `peaks_ms`,
`accept_ms`) and `dropped_no_transform`. **Candidates against cones kept is the
number to watch**: around 230 candidates for 18 cones is healthy, and a ratio
collapsing towards 1 means the tests have stopped rejecting anything.

## Building

```sh
colcon build --packages-select mirena_perception
```

Nothing is vendored and nothing is fetched at configure time. PCL comes from apt
— `libpcl-dev` and `ros-jazzy-pcl-conversions`, both in `docker/Dockerfile.one` —
and only its `common` component is asked for, because PCL is used for the debug
clouds and nothing else. The detector itself is Eigen and a grid.

## Mounting

The car-from-LiDAR transform is looked up per cloud and applied whole, so a
tilted mount is handled by the extrinsic being right rather than by anything
here. What protects against the extrinsic being *slightly* wrong is that no
height is ever measured against a global plane: every one is measured against a
plane fitted within 2.6 m of the cone it belongs to, so an unmodelled degree of
pitch — 17 cm at 10 m against a global plane — becomes a few millimetres.

To check the extrinsic, fit a plane to the ground returns in the car frame over a
run in which the car changes heading: a mount error is a constant pitch with
near-zero roll, while a sloping site swings with heading and shows up in roll.

There is a near blind spot no software can fix: a sensor at height `h` tilted
down by `t` degrees first sees the track at about `h / tan(t + half the vertical
FOV)`, roughly 2.5 m as mounted.

## Performance

7.7 ms per cloud at the median, 11.5 ms at p90, 16.5 ms worst case, on ~66 k
points. A 9.4 Hz sensor gives 106 ms.

| stage | p50 | p90 |
| --- | --- | --- |
| `accept_ms` — one look per candidate | 5.5 | 8.5 |
| `peaks_ms` | 1.6 | 2.1 |
| `coarse_ms` | 0.6 | 0.7 |
| `level_ms` | 0.2 | 0.4 |
| **total** | **7.7** | **11.5** |

**The cost scales with candidates, not with point density.** That is deliberate.
Any design built on per-point neighbour searches pays for how close the points
are to each other, and a 0.2° sensor puts returns about a centimetre apart at
3 m — thousands inside a 30 cm radius. Here there is one linear pass to build the
grid and then a bounded number of candidates, each costing one bucketed ring
query, so the worst case is set by geometry rather than by how cluttered the
scene is. A bucket index over the working cloud is what keeps the ring query to a
few dozen buckets instead of a pass over the sweep.

On a Jetson the CPU governor is worth more than any of this: under `schedutil` a
bursty load leaves the cores well below their clock ceiling. `jetson_clocks`, or
the `performance` governor.

## Layout

- `src/cone_detector.{hpp,cpp}` — the detector. No ROS in it, so the heuristics
  can be read and tuned without reading the node.
- `src/mirena_perception.cpp` — the ROS shell: subscriptions, TF, the
  `EntityList`, the debug clouds, the heartbeat.
