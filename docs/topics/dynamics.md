# Vessel dynamics, channel and sensing

The physical model closes the guidance/control loop: actuator commands produce vessel motion, measurements feed the next guidance step, and channel geometry affects forces and clearance.

## State and coordinate meanings

| Quantity | Meaning | Convention in the current loop |
| --- | --- | --- |
| `u`, `v`, `r` | Surge velocity, sway velocity, yaw rate | Body-frame motion quantities |
| `x`, `y`, `psi` | World position and heading | Heading used by sine/cosine functions is in radians |
| `z` | Calculated sinkage | Inserted at state index 5 after integration |
| `Np`, `u[-1]` | Propeller speed and rudder command | Propeller speed in rotations/s; rudder angle in radians |
| `yx`, `yy`, `ypsi` | Measurement histories | Latest samples are read by guidance |
| `x`, `y`, `psi` ontology properties | Physical trajectory histories | Distinct from measured histories |
| `UKC` | Under-keel clearance history | Depth minus draught minus sinkage at the logger's depth call |

The world-to-screen mapping used in the synthetic channel scales x and flips/scales y. Avoid passing screen coordinates directly into a world-state function.

## Physical-model functions

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

The planar kinematics in the active six-state system are:

$$
\dot{x}=u\cos\psi-v\sin\psi,\qquad
\dot{y}=u\sin\psi+v\cos\psi,\qquad
\dot{\psi}=r.
$$

The remaining derivatives depend on forces, added mass and coupling terms. Four `force_switch` entries independently scale hull, propeller, rudder and bank contributions. The default anti-grounding scenario sets all four to 1.

## Channel and clearance functions

| Function | Purpose | Important boundary |
| --- | --- | --- |
| `get_channel_depth(x, y, screen_width, screen_height, ...)` | Synthetic two-dimensional channel depth | Screen-coordinate query; piecewise depth; zero outside modeled water |
| `get_depths_from_file(filename)` | Loads x/depth samples from CSV | Returns a numeric table |
| `cal_depth(v, depths_data)` | One-dimensional profile lookup | Interpolates supplied samples, or uses its internal fallback profile |
| `cal_addedmass_nd(hdratio)` | Four depth/draught-dependent added-mass coefficients | Linear formulas with lower bounds |
| `calc_UKC_violations(t_vals, UKC_vals, UKC_limit)` | Finds below-threshold clearance intervals | Returns time intervals and corresponding index intervals |
| `retrieve_t_from_x(t_vals, x_vals, x_sample)` | Locates time for a requested longitudinal position | Assumes increasing x for interpolation |

These are alternative representations, not interchangeable depth services. The current synthetic function's `min_depth`/`max_depth` parameters do not control its active depth formula. Controller prediction, numeric dynamics and logging also use different scale constants; see the [OpenWiki coupling notes](../openwiki/concepts/guidance-control.md#depth-geometry-and-coupling).

## Measurements and bank geometry

`include_sensor_noise(noise)` sets relative velocity/speed measurement noise, while the vessel also initializes independent absolute positioning/heading noise. `sensor_output()` updates the numeric measured state. `dynamic_ontology_update()` records measured and true trajectories at the integrator's sampled times. Calling both is not equivalent to just appending one terminal point.

`include_sensor_faults(f, tf)` stores fault settings on the vessel; the older FTC pipeline calls a different, nonexistent method name. Review the observer and sensor path before treating that API as a complete fault-injection scenario.

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

Shore measurements use positive starboard and negative port distances; missing intersections become signed infinities. In the default loop, bank distances refresh after dynamics, making them inputs to the following step.

The complete [function catalog](../reference/functions.md) also lists plotting utilities, CSV loaders, sensor dependency classes and legacy helpers. `ARR`, `Division` and `Solution` exist in `classes.py`, but they are not assembled into the default navigation loop. Their presence does not establish that the complete resilient architecture from the [related paper](../publications.md#control-dynamics-and-resilience) is implemented here.

Source: [`classes.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/classes.py), [`channel.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/channel.py), [`mapper.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/mapper.py), and [`helper_utils.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/helper_utils.py).
