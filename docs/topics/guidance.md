<a id="routes-encounters-and-los-guidance"></a>

# Guidance

Guidance answers: **where should the vessel go, and which heading should it follow next?** It takes a route, measured motion and, for encounter-aware planning, semantic and contact information. Its output becomes a reference for Control.

## Learning path

| Read | Question it answers | Main functions |
| --- | --- | --- |
| [Line-of-Sight Laws](../guidance/line-of-sight.md) | How does route error become a desired heading? | `compute_LOS_ref`, `compute_LOS_ref_adaptive` |
| [Collision-free Path Planning](../guidance/collision-free-planning.md) | How are routes and encounter-related offsets selected? | `plan_route`, `global_planner`, `compute_CPA`, `Guidance_Reasoner` |

First understand route tracking on a fixed path. Then add contact assessment and planning, while retaining the same heading interface to the controller. See [Navigation](../navigation/index.md) for the measurements these functions consume and [Control](control.md) for actuator commands.

## Closest point of approach

CPA predicts separation under current relative motion. Its equation and zero-relative-speed limitation are covered in [encounter assessment](../guidance/collision-free-planning.md#closest-point-of-approach).

## Rule-aware path offsets

The reasoner selects roles and supplies offsets to the original route. Follow [rule-aware path offsets](../guidance/collision-free-planning.md#rule-aware-path-offsets) to understand that optimization and its fallback.

## Line-of-sight tracking

Standard and adaptive LOS laws share a heading output but have different state and configuration requirements. Read [Line-of-Sight Laws](../guidance/line-of-sight.md).

## Function map

Use each lesson's function table for inputs and outputs, and the [catalog](../reference/functions.md) for exact signatures. The two modules' `plan_route` definitions have different contracts.

## Connect guidance to a controller

See the [LOS controller fragment](../guidance/line-of-sight.md#connect-guidance-to-a-controller), then the full [pipeline examples](../pipelines.md).

## Related publications

- [Integrating rule awareness and semantic reasoning in collision-free vessel path planning](../publications.md#rule-aware-collision-free-planning) — research context for role assignment, path envelopes and LOS.
- [Virtual sensor-informed motion planning for safe autonomous waterborne navigation](../publications.md#virtual-sensor-informed-motion-planning) — related virtual-sensing work, listed with its supplied proceedings status.

Source: [`intel_guidance.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py), including [CPA](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py#L421-L435), [LOS](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py#L573-L638), and [global planning](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py#L841-L874).
