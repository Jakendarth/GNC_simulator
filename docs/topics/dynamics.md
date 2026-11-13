# Vessel dynamics, channel and sensing

The vessel model calculates motion from actuator commands. Sensors and history updates expose that motion to the next guidance step. Channel depth and distance to the banks influence both the physical response and clearance calculations.

Use this page when passing state between functions, choosing a depth representation, or replacing a map or measurement model. The [pipeline examples](../pipelines.md) show where these functions run.

## State and coordinate meanings

| Quantity | Meaning | Convention in the current loop |
| --- | --- | --- |
| State `u`, `v`, `r` | Forward velocity, lateral velocity, yaw rate | Body-frame velocities in m/s; yaw rate in rad/s |
| State `x`, `y`, `psi` | World position and heading | Positions in metres; heading in radians |
| `z` | Calculated sinkage in metres | Inserted at state index 5 after integration |
| `vessel.Np[-1]`, `vessel.u[-1]` | Latest propeller-speed and rudder commands | Rotations/s and radians respectively; `vessel.u` is a command history, distinct from state surge velocity |
| `yx`, `yy`, `ypsi` | Measurement histories | Latest samples are read by guidance |
| `x`, `y`, `psi` ontology properties | Physical trajectory histories | Distinct from measured histories |
| `UKC` | Under-keel clearance history | Depth minus draught minus sinkage at the logger's depth call |

The physical state is `[u, v, r, x, y, z, psi]`. The default integrator removes sinkage and advances `[u, v, r, x, y, psi]`, then restores the calculated `z`. The MPC adapter uses a third ordering: `[x, y, psi, u, v, r]`.

For the synthetic channel, screen x is scaled world x and screen y is negative scaled world y. Keep this conversion at the map boundary; screen pixels and world-state coordinates serve different functions.

## Physical-model functions

The active six-state model transforms body-frame velocities into world-frame position rates:

$$
\dot{x}=u\cos\psi-v\sin\psi,\qquad
\dot{y}=u\sin\psi+v\cos\psi,\qquad
\dot{\psi}=r.
$$

Here dots denote time derivatives and `psi` is heading. The remaining derivatives—surge acceleration, sway acceleration and yaw acceleration—depend on forces, added mass and coupling terms. Four `force_switch` entries scale hull, propeller, rudder and bank contributions, in that order. The default anti-grounding scenario enables all four.

The integration functions return sample times and six rows of motion samples, while updating the vessel's terminal state. `dynamic_ontology_update` consumes those samples to build histories. The alternative Radau model uses different parameter scaling; selecting it requires checking the physical parameter conventions as well as the function signature.

| Function | Responsibility | Connection |
| --- | --- | --- |
| `vessel.calculate_forces(v, ...)` | Numeric hull, propeller, rudder and bank contributions | Used by numeric dynamics; differs from the symbolic function with the same name |
| `vessel.rhs(s, v, ...)` | Numeric six-state motion derivatives and sinkage calculation | RK45 integration in the default loop |
| `vessel.differential_algebraic_model(tend)` | Updates dimensional mass/inertia, integrates, refreshes terminal state and sensors | Returns `(times, states)` for history logging |
| `vessel.three_dof_fossen(t, state)` | Alternative planar maneuvering model | Used by `run_models`, with different parameter scaling |
| `run_models(myvessel, time_step)` | Radau integration path | Separate from the default RK45 path |
| `vessel.calculate_hydro_params()` | Derivative and Nomoto-index calculations | Mutates ontology parameters; does not share exactly the default integrator's parameter convention |
| `vessel.update_derivatives_inland(envr)` | Alternative depth-dependent derivative update | Requires the attributes used by that implementation |
| `hydrostatics(file_name)` | Extracts selected PDF tables and writes an Excel file | Offline helper; not called by the default pipeline |


## Channel and clearance functions

| Function | Purpose | Important boundary |
| --- | --- | --- |
| `get_channel_depth(x, y, screen_width, screen_height, ...)` | Synthetic two-dimensional channel depth | Screen-coordinate query; piecewise depth; zero outside modeled water |
| `get_depths_from_file(filename)` | Loads x/depth samples from CSV | Returns a numeric table |
| `cal_depth(v, depths_data)` | One-dimensional profile lookup | Interpolates supplied samples, or uses its internal fallback profile |
| `cal_addedmass_nd(hdratio)` | Four depth/draught-dependent added-mass coefficients | Linear formulas with lower bounds |
| `calc_UKC_violations(t_vals, UKC_vals, UKC_limit)` | Finds below-threshold clearance intervals | Returns time intervals and corresponding index intervals |
| `retrieve_t_from_x(t_vals, x_vals, x_sample)` | Locates time for a requested longitudinal position | Assumes increasing x for interpolation |

Choose the depth function that matches your scenario's coordinates and data. `get_channel_depth` describes the synthetic channel in screen coordinates; `cal_depth` uses a longitudinal profile. Passing one representation into a caller designed for the other changes the model.

In the current synthetic function, `min_depth` and `max_depth` do not control the active formula. Controller prediction, numeric dynamics and UKC logging also use different scale constants. Review these call sites together before changing geometry; the [OpenWiki coupling notes](../openwiki/concepts/guidance-control.md#depth-geometry-and-coupling) identify them.

## Measurements and bank geometry

The simulator maintains measured quantities separately from the physical trajectory. `include_sensor_noise(noise)` sets relative velocity/speed noise, while position and heading have independent absolute noise settings. Setting relative noise to zero therefore does not remove all measurement perturbations.

`sensor_output()` updates the numeric measured state. `dynamic_ontology_update()` appends measured and physical trajectories at integrator sample times, repeating the current command and reference over those samples. Use both at their intended points in the pipeline; appending only the terminal position would change the histories consumed by guidance and plotting.

`include_sensor_faults(f, tf)` stores fault settings. The older fault-tolerant control (FTC) pipeline calls a different method name that the current vessel class does not define. Inspect the full observer and sensor path before using it as a fault-injection experiment.

| Geometry or rendering function | Role in a pipeline |
| --- | --- |
| `create_inland_depth_varying_map(...)` | Draws the synthetic channel and returns shoreline arrays |
| `calc_bank_distances_shore(...)` | Intersects lateral rays with shoreline segments; stores signed bank distances |
| `ray_segment_intersection(...)` | Geometric primitive for those measurements |
| `build_segment_tree`, `closest_point_on_segment` | Polygon segment and nearest-point helpers |
| `calc_bank_distances`, `calc_bank_distances_geopandas`, `calc_bank_distances_kml` | Alternative bank calculations for other map representations |
| `fetch_map`, `draw_map`, `to_world`, `to_screen` | Geographic data loading, rendering and coordinate conversion |
| `create_map(...)` | Effective later definition reads CSV boundaries; the earlier definition is shadowed |
| `draw_vessels_no_map`, `draw_route`, `draw_vessels` | Visual overlays with different geometry arguments |
| `check_vessel_bounds(...)` | Checks vessel positions against a supplied water path |

The shoreline ray calculation stores starboard distance as positive and port distance as negative. A missing intersection becomes positive or negative infinity respectively. Preserve these signs when supplying a new geometry implementation. In the default loop, bank distances refresh after dynamics and feed the following step.

Use the [function catalog](../reference/functions.md) for exact signatures of plotting utilities, CSV loaders and other helpers. `ARR`, `Division` and `Solution` are sensor-dependency and solution-related classes in `classes.py`; the default navigation pipeline does not assemble them into a resilient architecture. The [related paper](../publications.md#control-dynamics-and-resilience) provides the broader research context.

Source: [`classes.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/classes.py), [`channel.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/channel.py), [`mapper.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/mapper.py), and [`helper_utils.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/helper_utils.py).
