<a id="pid-and-predictive-control"></a>

# Control

Control answers: **which actuator commands should make the vessel follow its reference?** In these pipelines, the reference is a desired heading from Guidance. Controllers use the vessel's state and measurements described under [Navigation](../navigation/index.md).

## Learning path

| Read | Principle | Commands |
| --- | --- | --- |
| [PID Feedback Controllers](../control/feedback-controllers.md) | Correct heading error using feedback; inspect the model-derived gain tuning | Rudder angle |
| [Model-based Controllers](../control/model-based-controllers.md) | Predict vessel motion and solve a constrained optimization | Propeller speed and rudder angle |

Study PID first to understand the feedback loop and command history. Then study MPC to add prediction, actuator bounds and clearance constraints. The PID law has a feedback structure often described as model-free, but this implementation derives its gains from vessel parameters; it is not entirely model-free.

## PID background and implementation

See the [PID law and parameter meanings](../control/feedback-controllers.md#pid-background-and-implementation).

## Predictive-control background and implementation

See the [standard and anti-grounding MPC comparison](../control/model-based-controllers.md#predictive-control-background-and-implementation).

## Sinkage and clearance

See the [clearance equation and soft-constraint interpretation](../control/model-based-controllers.md#sinkage-and-clearance).

## Available controllers

The [PID lesson](../control/feedback-controllers.md#available-functions) and [MPC lesson](../control/model-based-controllers.md#available-functions) list their functions and mutation/return contracts. Both depend on consistent state ordering; see [state conventions](dynamics.md#state-and-coordinate-meanings).

## Controller composition

Follow the [MPC command fragment](../control/model-based-controllers.md#controller-composition) or [PID substitution](../pipelines.md#example-2-substitute-the-existing-pid-controller), then integrate the physical model and update histories.

## Related publications

- [Virtual sensor-informed motion planning for safe autonomous waterborne navigation](../publications.md#virtual-sensor-informed-motion-planning) — related work on virtual sensing and motion planning; specific code-to-method correspondence is not asserted here.
- [An intelligent agent-based resilient framework for marine vessel mission adaptations](../publications.md#resilient-mission-adaptation) — broader resilient control/architecture context.

Source: [`traffic_planner.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/traffic_planner.py#L134-L513), [vessel wrappers](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/classes.py#L159-L175), and [PID pipeline](https://github.com/Jakendarth/GNC_simulator/blob/main/src/pipelines/sim_ag_PID.py#L87-L101).
