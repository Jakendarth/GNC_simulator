# Collision-free Path Planning

Path planning selects where a vessel should travel before a LOS law decides its immediate heading. In an encounter, the planner combines route geometry, contact motion and traffic roles to produce offsets from the original path.

Read [Semantics](../topics/semantics.md) for rule-related vocabulary. This page follows the sequence route → encounter assessment → path offsets. [Line-of-Sight Laws](line-of-sight.md) explains the next stage.

## Route generation

Choose the planner whose geometry and inputs match your scenario:

- `traffic_planner.plan_route(start, stop)` samples a straight line at 20 points, including the endpoints.
- `intel_guidance.plan_route(start, stop, curve_degree, current_script_dir)` samples a straight or quadratic route and reads spacing from `input/SimParam_WC.json`.
- `global_planner(boundary, anchors, rho, distance_btw_wp)` builds a visibility graph between anchors and safety vertices, then selects a weighted shortest path. On no-path failure it returns `[]`, rather than a pair of coordinate lists. Its resampling call is commented out.

A visibility graph connects candidate points whose joining segment does not cross the supplied boundary segments. This separates geometric route selection from the per-step encounter-offset optimization below.

## Closest point of approach

The closest point of approach (CPA) estimates separation if both vessels continue with their current velocities. `compute_CPA` rotates body-frame velocities into the world frame before forming relative velocity.

Let \(p\) be own-vessel position minus contact position in metres, and \(v\) the corresponding relative velocity in metres per second. The estimated time to CPA, in seconds, is:

$$
t_{CPA}=-\frac{v^T p}{\|v\|^2}.
$$

For a future CPA, the distance is \(\|p+v t_{CPA}\|\). A negative time places CPA in the past, so the function reports current separation instead. The return tuple is `(t_cpa, d_cpa, current_distance)`.

The implementation does not guard division by zero when relative speed is zero. It also combines physical velocity with measured position/heading data. Validate these cases before using CPA as a decision threshold; the calculation does not predict future controller responses.

## Rule-aware path offsets

`Guidance_Reasoner` considers contact geometry within five own-vessel lengths, records regulation activation and calls the encounter state machine. If the role is Give-way, `generate_path_envelope2` adjusts route points using CVXPY optimization. Each problem minimizes squared displacement while applying contact, infrastructure and side-preference constraints.

The returned arrays \(D_x\) and \(D_y\) are offsets in the route's coordinate units. At waypoint \(k\), guidance uses:

$$
p_{reference,k}=p_{route,k}+\begin{bmatrix}D_{x,k}\\D_{y,k}\end{bmatrix}.
$$

LOS tracks the adjusted route. This path optimization precedes the actuator optimization performed by model predictive control (MPC).

If the envelope problem has no numeric displacement solution, it reuses the previous offsets supplied by its caller. The current reasoner supplies zero-initialized arrays, so this fallback does not retain the previous simulation step's optimized path.


## Functions and outputs

| Function | Purpose | Result and important state |
| --- | --- | --- |
| `intel_guidance.plan_route(start, stop, curve_degree, current_script_dir)` | Samples a straight or quadratic route | A two-row coordinate array; reads waypoint spacing from `input/SimParam_WC.json` |
| `traffic_planner.plan_route(start, stop)` | Simple straight-line alternative | 20 equally spaced points including endpoints |
| `global_planner(boundary, anchors, rho, distance_btw_wp)` | Visibility-graph route around boundary segments | Weighted shortest-path coordinates; returns `[]` if no path exists |
| `generate_safety_vertices`, `is_line_clear`, `intersect`, `ccw` | Geometry used to build feasible visibility edges | Candidate turning points and crossing checks |
| `safe_resample_route(...)` | Resamples while checking boundary visibility | Separate helper; resampling is commented out in `global_planner` |
| `compute_CPA(myvessel, instance)` | Estimates time/distance at closest approach | `(t_cpa, d_cpa, current_distance)` |
| `Finite_State_Machine(...)` | Classifies the encounter and decides a role | Mutates `scenario` and `role` |
| `Guidance_Reasoner(...)` | Combines rule activation, geometry and offset planning | Six results, including route offsets and contact list |
| `closest_segment(polyline, vessel, sim_scale)` | Finds the nearest map segment to measured position | Segment endpoints, or `None` for an empty/insufficient polyline |
| `generate_path_envelope2(...)` | Optimizes route-point deviations around contacts and infrastructure | Two offset arrays; reuses supplied previous offsets if no numeric solution is available |


Use explicit module names when choosing between the two `plan_route` functions; their signatures and configuration dependencies differ.

## Connect planning to guidance

`Guidance_Reasoner` returns route offsets, a safe-speed value, safe distance, scenario, contacts and nearest map segment. Pass its offsets to LOS and connect the heading reference to a controller. A returned safe-speed value does not automatically become a propeller command.

The [encounter pipeline example](../pipelines.md#example-3-connect-encounter-reasoning-to-los) shows these interfaces. It also records the shipped collision pipeline's older calls and map requirements. Watch the [encounter recordings](../demonstrations.md#encounter-recordings) for qualitative context.

## Related publications

- [Integrating rule awareness and semantic reasoning in collision-free vessel path planning](../publications.md#rule-aware-collision-free-planning) — research context for role assignment, path envelopes and LOS.
- [Virtual sensor-informed motion planning for safe autonomous waterborne navigation](../publications.md#virtual-sensor-informed-motion-planning) — related virtual-sensing work, listed with its supplied proceedings status.

Source: [`intel_guidance.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py), including [CPA](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py#L421-L435), [LOS](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py#L573-L638), and [global planning](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py#L841-L874).
