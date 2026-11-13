# Routes, encounters and LOS guidance

Guidance turns a mission route and the vessel's current situation into a desired heading. It sits between the [semantic model](semantics.md) and [actuator control](control.md): it changes the reference the controller tracks, rather than directly applying forces.

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

## Closest point of approach

The implementation rotates each vessel's body-frame velocity into the world frame and computes relative motion. For relative position \(p\) and relative velocity \(v\), its constant-velocity estimate uses:

$$
t_{CPA}=-\frac{v^T p}{\|v\|^2}.
$$

For a future closest approach, distance is \(\|p+v t_{CPA}\|\). For a negative time, the function retains current separation as its CPA distance. It does not guard the zero-relative-speed denominator, so that case requires attention when composing or testing a pipeline.

CPA is a geometric prediction under the current velocities. It does not simulate both vessels' future closed-loop responses.

## Rule-aware path offsets

`Guidance_Reasoner` filters contacts within five vessel lengths, activates regulation-related histories and calls the encounter state machine. If Give-way behavior is selected, `generate_path_envelope2` solves per-waypoint CVXPY problems minimizing squared x/y displacement subject to contact, infrastructure and side-preference constraints.

Its output is a deviation from the existing route:

$$
p_{reference,k}=p_{route,k}+\begin{bmatrix}D_{x,k}\\D_{y,k}\end{bmatrix}.
$$

LOS consumes that reference. The offset optimizer is a separate stage from the MPC actuator optimizer. The current reasoner initializes the previous-offset arrays to zero before calling the envelope function, so its fallback does not preserve a previous simulation step's optimized path.

## Line-of-sight tracking

The standard LOS function projects position onto the active route segment and skips segments already passed. Let \(\alpha\) be segment heading and \(y_e\) signed cross-track error. The implemented correction is:

$$
\psi_{raw}=\alpha-\operatorname{atan2}(y_e,\Delta),\qquad\Delta=5L.
$$

The result is wrapped and limited to a heading change of \((\pi/90)\,\texttt{vessel.timestep}\) per call. At the route end it holds the previous desired heading. These details explain why a route's geometry, vessel length and timestep all affect the reference seen by the controller.

`compute_LOS_ref_adaptive` also integrates `eso_rhs` and records estimated speed, heading, fault/disturbance terms, sideslip and cross-track quantities. It expects the extended-state and history conventions of that path. It is not interchangeable with standard LOS solely on the basis of a matching return type.

## Connect guidance to a controller

Within an initialized pipeline:

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
