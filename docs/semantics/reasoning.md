# Semantic Reasoning

Semantic reasoning uses the meaning of vessel, environment and traffic-rule information to support a navigation decision. In the current encounter path, Python functions combine annotated knowledge with geometry to select a traffic role and a reference-path deviation.

Read [Semantic Annotation](annotation.md) for the vocabulary and [Dynamic Updates](dynamic-updates.md) for current measurements and histories. This page follows **knowledge and geometry → role → path offsets**.

## The implemented decision sequence

1. `Guidance_Reasoner` computes contact geometry, including time and distance at closest point of approach.
2. It selects a regulation branch and records activation flags for the corresponding rule groups.
3. `Finite_State_Machine` evaluates relative angles, range, closest-approach distance and bank-distance conditions, then updates the vessel's `role` and `scenario`.
4. Stand-On behavior retains zero offsets; Give-way behavior invokes the path-envelope optimizer.
5. The pipeline passes offsets to LOS guidance, then uses a controller to follow the resulting heading reference.

## Regulation activation and geometric decisions

The code uses two regulation labels: COLREGS for international collision regulations, and BPR for the Dutch inland navigation regulations. `Guidance_Reasoner` selects the COLREGS branch when scaled x ≤ 600 and the BPR branch beyond it. This threshold is part of the scenario logic, not a general geographic lookup.

The function traverses regulation groups with `INDIRECT_equivalent_to` and appends `reg_active` flags. The XML connects rule individuals and situation groups using `owl:sameAs`. In OWL, that relation asserts identity; it is stronger than a membership relation. Review this modeling choice before changing the ontology or adding formal inference.

Distance, closest point of approach (CPA) and relative angles feed the encounter-role decision. Give-way behavior leads to optimized route offsets; Stand-On behavior keeps zero offsets. The default anti-grounding pipeline follows a different path: it calls line-of-sight (LOS) guidance directly and skips encounter reasoning.

:::{admonition} Scope of reasoning
:class: note

`Guidance_Reasoner` makes explicit Python decisions using ontology data. It does not invoke an OWL consistency or classification reasoner on each step. The XML also contains CCNR groups, named for the Central Commission for the Navigation of the Rhine, but this function selects COLREGS or BPR. Named rules and activation flags alone do not establish complete regulatory compliance.

:::


## Decision outputs and where they live

| Output | Stored or returned where | How it is used |
| --- | --- | --- |
| Rule activation | Ontology individuals' `reg_active` histories | Records the activation flags assigned by the current branch |
| `role`, `scenario` | Python vessel attributes | Determines the path-offset branch and describes the encounter |
| Route offsets | Returned arrays, also stored as `droute`, `Dx`, `Dy` | Added to the nominal route before LOS tracking |
| Safe-speed value and safe distance | Part of the six-value reasoner return | Available to a caller; the safe-speed value is not automatically applied as a propeller command |
| Contacts and nearest segment | Remaining reasoner results | Supports the geometric path-planning interface |

The code creates `active_regs` and `tobechecked_regs` lists while traversing rule groups. Those names do not establish a general rule-evaluation engine or a complete compliance checker. The finite-state logic and geometric optimizer provide the actual decisions described here.

## Connect reasoning to a pipeline

The [encounter composition example](../pipelines.md#example-3-connect-encounter-reasoning-to-los) shows the current six-result function interface and the map polyline it needs. The shipped `sim_colav.py` still uses older guidance/drawing calls, so review its integration gaps before expecting a complete run.

Continue with [Collision-free Path Planning](../guidance/collision-free-planning.md) for the closest-approach equation, route generation and the displacement optimization. The [encounter recordings](../demonstrations.md#encounter-recordings) provide qualitative context.

## Related publications

- [Integrating rule awareness and semantic reasoning in collision-free vessel path planning](../publications.md#rule-aware-collision-free-planning) — semantic traffic-rule information and encounter guidance.
- [An intelligent agent-based resilient framework for marine vessel mission adaptations](../publications.md#resilient-mission-adaptation) — the wider architecture and control perspective.

Source: [`classes.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/classes.py#L34-L148), [`helper_utils.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/helper_utils.py#L350-L361), and [`intel_guidance.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py#L83-L203).
