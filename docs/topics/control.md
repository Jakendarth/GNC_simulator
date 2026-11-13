# PID and predictive control

Controllers convert a desired heading into actuator commands. Proportional–integral–derivative (PID) control returns a rudder angle. Model predictive control (MPC) optimizes both rudder angle and propeller speed over a future horizon.

Use this page to choose a controller and understand its inputs, limits and state conventions. Before calling one, initialize the vessel's physical parameters, measurements and command histories as described in the [pipeline examples](../pipelines.md).

## PID background and implementation

PID corrects the difference between desired and measured heading. The implemented law combines wrapped heading error \(e_\psi\), accumulated error, measured yaw rate \(r\) and desired yaw rate \(r_d\):

$$
\delta=K_p\left(e_\psi+\frac{1}{T_i}\int e_\psi\,dt+T_d(-r+r_d)\right).
$$

Here \(\delta\) is rudder angle in radians, \(K_p\) is the proportional gain, and \(T_i\) and \(T_d\) are integral and derivative time parameters. Headings are in radians and yaw rates in radians per second. The code clips the command to ±35 degrees, converted to radians.

Gains are calculated from vessel hydrodynamic parameters and Nomoto-style steering indices; this implementation does not read the JSON `PID Gains` object. Standard line-of-sight (LOS) guidance updates `psi_d_prev`, which PID also uses to compute `r_d`. Preserve call order when comparing controller behavior.

The function updates accumulated error and returns a scalar command. The pipeline appends that command to the rudder history. The existing PID scenario leaves the initialized propeller-speed history in place.

## Predictive-control background and implementation

MPC predicts motion over a finite horizon, solves a constrained optimization and applies the first command. It repeats this process as the vessel moves. The builders receive `[x, y, psi, u, v, r]`, whereas the vessel stores `[u, v, r, x, y, z, psi]`. Use the step adapters to preserve this ordering. See [state and coordinate meanings](dynamics.md#state-and-coordinate-meanings) for units.

| Detail | Standard MPC | Anti-grounding MPC |
| --- | --- | --- |
| State | Six continuous states | Six continuous states plus algebraic sinkage |
| Heading objective | Code squares an already squared heading error, producing a fourth-power penalty | Weighted squared heading error |
| Speed treatment | Upper limit of 7 knots | Speed-reward term and upper limit of 5 knots |
| Grounding constraint | No corresponding soft UKC constraint in this builder | Soft UKC constraint with limited slack |
| Actuators | `Np`, then `delta` | `Np`, then `delta` |

In the default anti-grounding pipeline, a new controller is built on every step and initialized from current state and commands. Both builders bound propeller speed to 50/60–600/60 rotations per second and rudder angle to ±35 degrees in radians.

## Sinkage and clearance

Under-keel clearance (UKC) is water depth minus vessel draught and sinkage. Sinkage is the additional downward displacement calculated by the model. The anti-grounding builder represents sinkage \(z\) through an algebraic speed-dependent relation and samples depth along a straight-line prediction.

Let \(h\) be the minimum sampled depth and \(T_d\) the vessel draught, all in metres. The nominal clearance constraint is:

$$
z+T_d+0.05+0.15-h\leq 0.
$$

The constants are a 0.05 m minimum and a 0.15 m buffer. A soft constraint permits up to 0.15 m of violation with a weighted penalty, so a successful optimization does not by itself prove a particular clearance margin in the simulated trajectory. In this equation `Td` means draught; it is unrelated to the PID derivative time parameter above.

!!! note "Composition details"
    The vessel anti-grounding method replaces the supplied weight vector with defaults. Standard MPC initializes from `state[0:6]`, but its step adapter removes sinkage from a seven-element state; review that difference before composing a new pipeline. Numeric dynamics and symbolic MPC also have separate force implementations and depth conversions.

## Available controllers

| Function or method | Responsibility | Contract |
| --- | --- | --- |
| `PID_controller(vessel, psi_d, dt)` | Heading feedback with accumulated error and yaw-rate term | Returns rudder angle in radians; mutates accumulated error |
| `vessel.PID_controller(psi_d, dt)` | Convenience delegation to PID | Returns the same scalar command |
| `MPC_controller(vessel, ...)` | Constructs standard continuous-state MPC | Returns a do-mpc controller |
| `vessel.MPC_controller(...)` | Builds and stores standard MPC | Stores `vessel.MPC`; use `get_next_mpc_step()` for a command |
| `Anti_Grounding_MPC_controller(vessel, ...)` | Constructs MPC with algebraic sinkage and clearance constraint | Returns a do-mpc controller |
| `vessel.Anti_Grounding_MPC_controller(...)` | Builds and stores anti-grounding MPC | Stores `vessel.Anti_Grounding_MPC` |
| `get_next_mpc_step(vessel)` / `get_next_ag_step(vessel)` | Adapts physical state and calls `make_step` | Returns propeller command first, rudder command second |
| `traffic_planner.calculate_forces(...)` | Symbolic force model for optimization | Consumes symbolic state, actuator inputs, depth and sinkage |

Module-level MPC builders return controller objects. The corresponding vessel methods store those objects on the vessel; call a step method afterward to obtain commands. The default pipeline uses these vessel methods.

## Controller composition

This fragment belongs after guidance has produced `psi_d`. It assumes the default pipeline's initialized `barge`, `t_step` and `sim_scale`. The returned command array stores propeller speed first and rudder angle second:

```python
barge.Anti_Grounding_MPC_controller(
    psi_d=psi_d,
    MPC_prediction_horizon=5,
    MPC_time_step=t_step,
    min_depth=3.15,
    max_depth=3.3,
    sim_scale=sim_scale,
)
command = barge.get_next_ag_step()
barge.Np.append(float(command[0][0]))
barge.u.append(float(command[1][0]))
```

Next, integrate the vessel and append histories as shown in [Connecting the functions](../pipelines.md). For recorded examples, see the [PID and NMPC videos](../demonstrations.md#navigation-recordings).

## Related publications

- [Virtual sensor-informed motion planning for safe autonomous waterborne navigation](../publications.md#virtual-sensor-informed-motion-planning) — related work on virtual sensing and motion planning; specific code-to-method correspondence is not asserted here.
- [An intelligent agent-based resilient framework for marine vessel mission adaptations](../publications.md#resilient-mission-adaptation) — broader resilient control/architecture context.

Source: [`traffic_planner.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/traffic_planner.py#L134-L513), [vessel wrappers](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/classes.py#L159-L175), and [PID pipeline](https://github.com/Jakendarth/GNC_simulator/blob/main/src/pipelines/sim_ag_PID.py#L87-L101).
