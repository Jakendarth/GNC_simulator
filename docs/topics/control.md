# PID and predictive control

Controllers translate desired heading into rudder angle and, for MPC, propeller speed. They operate on a vessel that already has initialized physical parameters, measurements and command histories.

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

The Python wrappers and module-level builders have different return/mutation behavior. The default pipeline uses the vessel wrappers.

## PID background and implementation

The implemented law combines wrapped heading error \(e_\psi\), its accumulated integral, measured yaw rate \(r\) and a desired yaw-rate term \(r_d\):

$$
\delta=K_p\left(e_\psi+\frac{1}{T_i}\int e_\psi\,dt+T_d(-r+r_d)\right).
$$

The code clips \(\delta\) to ±35 degrees, expressed in radians. It computes gains using vessel hydrodynamic parameters and Nomoto-style indices. The `PID Gains` object in the bundled JSON is not read by this implementation. Standard LOS also updates `psi_d_prev`, which PID uses to calculate `r_d`; call order therefore affects that derivative term.

PID changes the rudder history. The existing PID scenario leaves the initialized propeller-speed history in place rather than appending an MPC speed command each step.

## Predictive-control background and implementation

MPC constructs a model, predicts motion over a finite horizon, solves a constrained optimization and applies the first actuator command. In these builders, the controller receives `[x, y, psi, u, v, r]`, while the vessel stores `[u, v, r, x, y, z, psi]`. The adapters are part of the interface, not optional rearrangement.

| Detail | Standard MPC | Anti-grounding MPC |
| --- | --- | --- |
| State | Six continuous states | Six continuous states plus algebraic sinkage |
| Heading objective | Code squares an already squared heading error, producing a fourth-power penalty | Weighted squared heading error |
| Speed treatment | Upper limit of 7 knots | Speed-reward term and upper limit of 5 knots |
| Grounding constraint | No corresponding soft UKC constraint in this builder | Soft UKC constraint with limited slack |
| Actuators | `Np`, then `delta` | `Np`, then `delta` |

The default anti-grounding pipeline rebuilds its controller at each step. It initializes guesses from current vessel state and commands rather than maintaining one controller instance throughout the run.

## Sinkage and clearance

The anti-grounding builder represents sinkage \(z\) with an algebraic speed-dependent relation and evaluates a minimum sampled depth \(h\) over a straight-line horizon estimate. Its clearance expression is:

$$
z+T_d+0.05+0.15-h\leq 0.
$$

A soft constraint allows up to 0.15 m of violation with a weighted penalty. This distinguishes the optimization's nominal buffer from an absolute guarantee of clearance in the simulated trajectory.

!!! note "Composition details"
    The vessel anti-grounding wrapper replaces its caller's weight vector with defaults. The standard MPC builder initializes from `state[0:6]`, while its step adapter explicitly removes sinkage when given a seven-element state. Review that initialization boundary before selecting standard MPC in a new pipeline. Numeric dynamics and symbolic optimization also use separate force implementations and depth conversions.

## Controller composition

Inside the initialized default pipeline:

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
