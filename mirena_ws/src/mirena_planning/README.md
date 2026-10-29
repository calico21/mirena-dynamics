# mirena_planning

**mirena_planning** turns the SLAM cone map into the centreline the controller follows.
It subscribes to the map, orders it into a track, and publishes that track as a sequence of
gates in the frame the map arrived in.

## Interface

| | |
|---|---|
| Subscribes | `slam/map` (`mirena_common/EntityList`, in the `map` frame) |
| Publishes | `planning/track` (`mirena_common/Track`, latched), `health` (`mirena_common/Heartbeat`) |
| Debug | `planning/debug/delaunay`, `planning/debug/registration` (`visualization_msgs/MarkerArray`) |
| Lifecycle | Standard ROS 2 lifecycle node. Pass `-a` to configure and activate on startup. |

The mission is selected by the `mission` parameter, which the supervisor sets from
`mission_config/<MISSION>.yaml` before configuring the node. An unsupported mission fails
the configure transition rather than running with a planner that cannot work.

| `mission` | planner |
|---|---|
| `trackdrive`, `autocross` | unknown track: Delaunay triangulation |
| `skidpad`, `acceleration` | known track: template registration |

## How it works

### Unknown tracks

Delaunay-triangulate the blue and yellow cones, keep the edges that join the two colours
and are narrow enough to be a real gate, and take their midpoints as candidate centreline
samples. Ordering those midpoints is the part that matters: a plain nearest-neighbour walk
happily cuts across the track or doubles back, so each step takes the unvisited midpoint
minimising `distance * (1 + turn_penalty * (1 - cos turn))` and refuses anything turning
more than about 84 degrees off the current heading.

The walk starts at the start line, which is the centre of the big orange cones **near the
map origin**. That restriction is not cosmetic: SLAM anchors the map frame at the pose the
car started from, so the start line is at the origin, and on autocross — where the finish
line is somewhere else entirely — averaging every orange cone in the map puts the start
gate in the middle of the track.

Loop closure is declared when the path both leaves the start region and returns to it, and
only after `closure_hysteresis` consecutive agreeing cycles. The controller fits a periodic
spline when the flag is set and an open one when it is not, so a flag that chatters costs
it a re-fit and shifts the arc length its lap counter rides on.

### Known tracks

The layout is already surveyed in `resource/<mission>.json`, so the job is registration:
find the transform that puts the stored template onto the live map, then publish the stored
racing line through it.

That transform is very nearly known in advance. The JSON records `setup.car_start_pose`,
the pose the car starts from expressed in the template, and SLAM anchors the map frame at
that same pose — so the nominal transform is exact up to how accurately the car was placed
on the line. Registration starts from there and refines with colour-gated ICP; on later
cycles it warm-starts from the previous solution and converges in a couple of iterations.

Two-point RANSAC is the recovery path when ICP cannot lock on. It samples a pair of mapped
cones and looks up the template pairs of the same length and the same colours, which is
what makes hypothesis generation tractable — drawing three unlabelled correspondences at
random, as the previous implementation did, has a hit rate around `(1/N)³` per iteration and
essentially never converges. Only the winning hypothesis is polished with ICP.

A fit is accepted on three counts: enough inliers in absolute terms, a good enough inlier
*fraction* (over the cones that could match, not over every cone SLAM has mapped), and a
placement within `max_position_drift` of the nominal. The last one matters most on
acceleration, where a straight corridor of evenly spaced cones registers just as well
shifted along its own axis.

### Publishing

Planning runs on its own timer at `plan_period`, and the map subscription only parks the
latest message, so a fast map publisher cannot build a backlog of expensive callbacks. The
heartbeat runs on a separate callback group for the same reason: the supervisor reads
liveness from it.

The track is republished only when it has actually changed — a different gate count, a
different closure flag, or a sample that moved more than `republish_tolerance`. Re-sending a
centreline that jittered by a millimetre only makes the controller re-fit its spline. The
topic is latched, so a controller that subscribes late still gets the current track
immediately.

## Debug topics

Both are `MarkerArray`s in the map frame, and both are only built when something is
subscribed to them.

`planning/debug/delaunay` shows the full triangulation in grey, the edges accepted as track
gates in green, their midpoints in cyan, the traced centreline in magenta, and the start
gate as an orange sphere. Between them they explain nearly every bad plan: a missing gate
edge is a `max_gate_width` problem, a centreline that stops early is a `max_gate_gap` one,
and a start sphere in the wrong place means the big orange cones are not where the planner
expects.

`planning/debug/registration` shows the template cones and racing line under the current
fit, a green segment for every inlier correspondence, red spheres for mapped cones with no
match, and a text overlay with the score, the ICP iteration count, whether the fit came
from the seed or from RANSAC, and the fit itself.

## Parameters

| | default | |
|---|---|---|
| `mission` | `trackdrive` | selects the planner |
| `plan_period` | `0.1` | planning rate, decoupled from the map rate |
| `heartbeat_period` | `0.1` | |
| `republish_tolerance` | `0.25` | metres a gate must move before the track is resent |
| `min_cone_confidence` | `0.0` | cones below this are ignored |
| `publish_debug` | `true` | creates the debug publishers |
| `fallback_frame` | `map` | used only if the incoming map has no `frame_id` |

Unknown-track planner: `curvature_weight` (1.0), `lookahead` (5), `min_track_width` (2.0),
`max_track_width` (5.0), `cone_spacing` (5.0), `max_gate_gap` (8.5),
`start_gate_radius` (10.0), `closure_hysteresis` (5), `max_path_points` (400).

`min_track_width` and `max_track_width` bound how wide an edge may be and still be driven as
a gate. They are not symmetric in what they do.

`max_track_width` is the one that discriminates. It stops the trace cutting the inside of a
hairpin: the ground inside a doubling-back turn carries no cones, the triangulation spans it,
and those chords are wider than the track by half again — 4.9 m to 5.2 m where an honest gate
is 3.00 m. Raising it past about 5.5 lets them back in; lowering it starts refusing the
diagonals that bridge an unmapped cone.

`min_track_width` is a rail against degenerate edges — unmerged duplicate landmarks — and
fires on nothing else; from 0 to 2.5 it changes no result here. **Do not set it to the rules'
3 m.** It tests a measurement across two perceived cones, not the track, and that measurement
reads short: the narrowest gate really driven is 2.999 m on a perfect map and 2.469 m on a
degraded one. At 3.0 the perfect-map trace loses six of ten closures and ends up on the cones.

Known-track planner: `inlier_distance` (0.6), `min_score` (0.6), `min_inliers` (5),
`icp_iterations` (25), `ransac_iterations` (200), `max_position_drift` (5.0),
`max_heading_drift` (0.6).

## Heartbeat report

```json
{"start_point": {"x": 0.0, "y": 0.0, "z": 0.0}, "mission": "trackdrive",
 "gates": 31, "closed": true, "plan_ms": 0.4, "planned": true, "score": 0.0,
 "plan_age_s": 0.05}
```

`start_point` is null until the first plan, and keeps the shape the previous implementation
published. `score` is the registration inlier fraction and stays at zero on unknown tracks.

## Tests

`colcon test --packages-select mirena_planning`. The planning core carries no ROS
dependency, so the triangulation, the trace, the closure hysteresis and the registration
are all covered without spinning a node.
