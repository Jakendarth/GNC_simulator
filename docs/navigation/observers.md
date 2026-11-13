# State Observers

An observer combines measurements with a motion model to calculate estimated quantities. In this repository, the observer is coupled to `compute_LOS_ref_adaptive`: that guidance function integrates `vessel.eso_rhs` and uses estimated path-related quantities to choose a heading.

Read [State and Sensors](state-and-sensors.md) first. This page describes the available implementation and its state dependencies; it does not establish an independently validated sensor-fault diagnosis system.

## Where estimation runs

1. The vessel constructor initializes `extended_state` from six motion components and fifteen additional values.
2. Adaptive LOS identifies the active route segment and calculates its heading `alpha`.
3. It integrates `eso_rhs` using RK45 over `vessel.timestep` and stores the terminal extended state.
4. It appends selected components to estimate histories, then calculates a desired heading using estimated cross-track error and sideslip.

The default anti-grounding pipeline calls standard LOS instead. Selecting an MPC controller does not automatically activate the observer.

## Estimated quantities and histories

| History | Extended-state component | Use in this path |
| --- | --- | --- |
| `uhat` | 0 | Estimated surge-motion quantity |
| `psihat` | 5 | Estimated heading quantity |
| `fhatu`, `fhatx`, `fhatpsi` | 12, 15, 17 | Stored fault/disturbance-related estimate components |
| `betahat` | 18 | Estimated sideslip quantity used by adaptive LOS |
| `ehat` | 19 | Estimated cross-track quantity used by adaptive LOS |
| `timehat` | Integrated interval end plus vessel time | Time associated with the estimate histories |

`eso_rhs` uses numeric measurements, latest ontology histories, vessel parameters, force switches and fault-related settings. It also calculates sinkage on the vessel, so it has effects beyond returning a derivative vector.

## Before composing an observer-informed pipeline

- Initialize the extended state, route, measurement histories and bank distances expected by adaptive LOS.
- Check `vessel.timestep`; this controls the observer integration interval.
- Check the explicit lookahead read from `input/SimParam.json`, even if another input file selects the outer pipeline.
- Preserve history types. Standard LOS stores scalar `vessel.e`; adaptive LOS appends to `vessel.e` as a list. Switching between them on the same instance requires reviewing/resetting that state.
- Distinguish estimates from simulated truth when exporting or plotting. An estimate history's presence does not prove fault-detection accuracy.

## Connection to guidance and research

Follow [Adaptive LOS and virtual sensing](../guidance/line-of-sight.md#adaptive-los-and-virtual-sensing) for the heading-reference side of this coupling, and the [pipeline examples](../pipelines.md) for state ownership and call ordering.

[Virtual sensor-informed motion planning](../publications.md#virtual-sensor-informed-motion-planning) is related research. The documentation retains its supplied proceedings status and does not claim a verified function-by-function implementation of that paper.

Source: [adaptive LOS](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py#L453-L571) and [`eso_rhs`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/classes.py#L655-L734).
