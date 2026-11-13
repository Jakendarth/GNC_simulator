# State and Sensors

Before choosing a navigation or guidance function, identify which representation of motion it reads. A physical state, a noisy measurement and an estimate can describe the same quantity but have different roles in the loop.

## Three representations of motion

| Representation | Where to find it | How it is used |
| --- | --- | --- |
| Physical state | `vessel.state` and ontology trajectory histories | Numeric dynamics advance the simulated vessel |
| Measurements | `vessel.y`, and ontology `yx`, `yy`, `ypsi`, `SOG` histories | Guidance and control read measured motion |
| Observer state and estimates | `vessel.extended_state`, `uhat`, `psihat`, `betahat`, `ehat` and related histories | Adaptive LOS uses the observer path |

The physical state order is `[u, v, r, x, y, z, psi]`: surge and sway velocity in m/s, yaw rate in rad/s, world position and sinkage in metres, and heading in radians. Numeric integration removes sinkage, while MPC reorders the six motion states. See [state and coordinate meanings](../topics/dynamics.md#state-and-coordinate-meanings) before passing arrays between functions.

## Measurement noise

`include_sensor_noise(noise)` sets a relative measurement-noise multiplier. Position and heading have separate absolute noise settings, initialized to 1 m, 1 m and 2 degrees respectively. The heading setting is stored in radians. Setting relative noise to zero therefore does not make every measurement noise-free.

`sensor_output()` refreshes the numeric measured state after integration. `dynamic_ontology_update()` appends physical and measured histories at integrator sample times, including speed over ground (`SOG`), heading reference and actuator commands. A simulation step can contribute multiple history samples.

| Function | Use it when | Result |
| --- | --- | --- |
| `update_ontology_start_values(player, x0)` | Initializing the existing pipeline's vessel | Synchronizes initial pose/velocity values and numeric state |
| `include_sensor_noise(noise)` | Setting relative measurement noise | Updates the vessel's noise setting |
| `sensor_output()` | Refreshing numeric measurements after physical integration | Updates `vessel.y` |
| `dynamic_ontology_update(...)` | Recording the integrator's sample arrays | Appends ontology histories for the next guidance step and later plotting |
| `include_sensor_faults(f, tf)` | Configuring fault-related settings for a compatible path | Stores settings; full behavior depends on the observer/sensor path |

## Position and bank measurements

World coordinates and screen coordinates differ: the synthetic map scales x and negates/scales y. `calc_bank_distances_shore` measures signed lateral ray intersections with shoreline segments in screen geometry, then converts distances by `sim_scale`.

Starboard distance is positive and port distance negative. Missing intersections are stored as positive and negative infinity respectively. These measurements feed the following step in the default loop because the redraw refreshes them after dynamics. The [geometry reference](../topics/dynamics.md#measurements-and-bank-geometry) lists alternatives for other map representations.

## Connect measurements to the loop

The [anti-grounding pipeline example](../pipelines.md#example-1-the-existing-anti-grounding-loop) integrates motion, appends measured/physical histories and refreshes bank measurements. Preserve that order before introducing a different sensor or observer.

Continue with [State Observers](observers.md) to understand how measurements can support estimated quantities. For the shared meaning of ontology properties, read [Semantics](../topics/semantics.md).

Source: [`classes.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/classes.py#L34-L192) and [`helper_utils.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/helper_utils.py#L543-L661).
