# PID Feedback Controllers

Feedback control uses the difference between desired and measured motion to calculate an actuator command. The available PID controller tracks heading and returns a rudder angle; the pipeline appends it to the command history.

Start here before studying [Model-based Controllers](model-based-controllers.md). You should understand how heading error becomes rudder action, which state is updated, and what remains outside this controller's responsibility.

## Model-free versus model-derived tuning

PID does not solve an online vessel-motion optimization, and it is often introduced as a model-free feedback structure. In this repository, however, its gains are calculated from hydrodynamic parameters and Nomoto-style steering indices. It is therefore not a strictly model-free implementation. The distinction concerns tuning as well as the feedback law.

## PID background and implementation

PID corrects the difference between desired and measured heading. The implemented law combines wrapped heading error \(e_\psi\), accumulated error, measured yaw rate \(r\) and desired yaw rate \(r_d\):

$$
\delta=K_p\left(e_\psi+\frac{1}{T_i}\int e_\psi\,dt+T_d(-r+r_d)\right).
$$

Here \(\delta\) is rudder angle in radians, \(K_p\) is the proportional gain, and \(T_i\) and \(T_d\) are integral and derivative time parameters. Headings are in radians and yaw rates in radians per second. The code clips the command to ±35 degrees, converted to radians.

Gains are calculated from vessel hydrodynamic parameters and Nomoto-style steering indices; this implementation does not read the JSON `PID Gains` object. Standard line-of-sight (LOS) guidance updates `psi_d_prev`, which PID also uses to compute `r_d`. Preserve call order when comparing controller behavior.

The function updates accumulated error and returns a scalar command. The pipeline appends that command to the rudder history. The existing PID scenario leaves the initialized propeller-speed history in place.


## Available functions

| Function or method | Responsibility | Contract |
| --- | --- | --- |
| `PID_controller(vessel, psi_d, dt)` | Heading feedback with accumulated error and yaw-rate term | Returns rudder angle in radians; mutates accumulated error |
| `vessel.PID_controller(psi_d, dt)` | Convenience delegation to PID | Returns the same scalar command |


## Connect heading feedback to the vessel

Inside an initialized pipeline, standard LOS supplies the heading reference and PID returns the rudder command:

```python
psi_d = compute_LOS_ref(barge, None, current_script_dir)
barge.u.append(PID_controller(barge, psi_d, t_step))
```

This fragment assumes the existing pipeline imports, vessel state, route and nonempty measurement histories. PID does not change the initialized propeller speed or impose an optimized clearance constraint. Follow the [PID pipeline example](../pipelines.md#example-2-substitute-the-existing-pid-controller) for the integration and history-update calls that follow.

See the [PID recording](../demonstrations.md#pid-navigation) for visual context. Exact gains and recording conditions are not documented there.

## Related publications

- [Virtual sensor-informed motion planning for safe autonomous waterborne navigation](../publications.md#virtual-sensor-informed-motion-planning) — related work on virtual sensing and motion planning; specific code-to-method correspondence is not asserted here.
- [An intelligent agent-based resilient framework for marine vessel mission adaptations](../publications.md#resilient-mission-adaptation) — broader resilient control/architecture context.

Source: [`traffic_planner.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/traffic_planner.py#L134-L513), [vessel wrappers](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/classes.py#L159-L175), and [PID pipeline](https://github.com/Jakendarth/GNC_simulator/blob/main/src/pipelines/sim_ag_PID.py#L87-L101).
