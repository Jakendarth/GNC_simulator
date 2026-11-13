# Routes, encounters and LOS guidance

Guidance answers two questions: which path should the vessel follow, and which heading should it track now? Route planners and encounter logic address the first; line-of-sight (LOS) guidance addresses the second.

The [semantic model](semantics.md) supplies vessel and regulation-related information. Guidance uses that information and measured motion to produce a reference for [actuator control](control.md). Read this page when choosing a guidance function or connecting encounter planning to route tracking.

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

## Line-of-sight tracking

Standard LOS steers toward the route while correcting lateral deviation. It projects measured position onto the active segment and advances past segments already crossed. Let \(\alpha\) be segment heading in radians, \(y_e\) signed cross-track error in metres, and \(L\) vessel length in metres. With lookahead distance \(\Delta\), the correction is:

$$
\psi_{raw}=\alpha-\operatorname{atan2}(y_e,\Delta),\qquad\Delta=5L.
$$

The function wraps the result to the angle range and limits each heading change to \((\pi/90)\,\texttt{vessel.timestep}\) radians. At the route end it holds the previous desired heading. `vessel.timestep` defaults to 1; changing a pipeline's integration interval does not automatically update this member. Standard LOS uses `5 * L` for lookahead, independently of the JSON `Lookahead distance` field.

The adaptive variant also integrates `eso_rhs`, an extended-state observer used to estimate motion and disturbance/fault-related quantities. It records estimated speed, heading, sideslip and cross-track histories, and reads lookahead from `input/SimParam.json`. Choose it only after checking the observer state and history requirements; sharing a heading return value does not make it a direct replacement for standard LOS.

## Function map

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
| `compute_LOS_ref(vessel, d_route, current_script_dir)` | Tracks an original or offset route | Heading in radians; updates waypoint index, cross-track error and heading history |
| `compute_LOS_ref_adaptive(...)` | Couples route following to an extended-state observer | Heading plus observer/history updates; reads `input/SimParam.json` |

Use explicit module names when choosing between the two `plan_route` functions. Their signatures, sampling and configuration dependencies differ.

## Connect guidance to a controller

Use this fragment inside a pipeline after initializing `barge`, its numeric route, ontology histories and timestep. The existing anti-grounding pipelines import the guidance and controller functions used here:

```python
# route_offsets=None follows the original route.
psi_d = compute_LOS_ref(barge, None, current_script_dir)
delta = PID_controller(barge, psi_d, t_step)
barge.u.append(delta)
```

This is a loop fragment, not a standalone program: route, ontology histories and vessel state must already be initialized. The [pipeline examples](../pipelines.md) show the surrounding sequence, and [recorded encounters](../demonstrations.md#encounter-recordings) provide visual context.

## Related publications

- [Integrating rule awareness and semantic reasoning in collision-free vessel path planning](../publications.md#rule-aware-collision-free-planning) — research context for role assignment, path envelopes and LOS.
- [Virtual sensor-informed motion planning for safe autonomous waterborne navigation](../publications.md#virtual-sensor-informed-motion-planning) — related virtual-sensing work, listed with its supplied proceedings status.

Source: [`intel_guidance.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py), including [CPA](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py#L421-L435), [LOS](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py#L573-L638), and [global planning](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py#L841-L874).
