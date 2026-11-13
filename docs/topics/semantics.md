<a id="semantic-model-and-runtime-meaning"></a>

# Semantics

Semantics connects numeric simulation values with their meaning: which vessel or sensor they belong to, which operating conditions they describe, and how traffic knowledge enters a decision.

## Learning path

| Read | Main question | Outcome |
| --- | --- | --- |
| [Semantic Annotation](../semantics/annotation.md) | What does the simulator represent? | Understand vessels, traffic rules, environment, sensors, operators and design-space concepts |
| [Dynamic Updates](../semantics/dynamic-updates.md) | How does the knowledge model change during a run? | Trace initialization, measurements and sampled histories |
| [Semantic Reasoning](../semantics/reasoning.md) | How is annotated information used to make a decision? | Follow regulation activation, encounter roles and path offsets |

Read the pages in this order, then connect them to [Guidance](guidance.md), [Navigation](../navigation/index.md), [Control](control.md) and the [pipeline examples](../pipelines.md).

## What the ontology represents

The ontology covers the maritime system beyond the vessel alone. See [the six class categories](../semantics/annotation.md#a-vocabulary-spanning-the-maritime-system) for their purpose, actual individuals and runtime boundaries.

## Three meanings to keep separate

Annotation describes **what a value means**, updates describe **when it changes**, and reasoning describes **how it affects a decision**. The separate lessons follow these three responsibilities.

## Regulation activation and geometric decisions

Read [Semantic Reasoning](../semantics/reasoning.md#regulation-activation-and-geometric-decisions) for the implemented COLREGS/BPR selection and the limits of the ontology's traffic-rule representation.

## Functions that bridge knowledge and simulation

Initialization and history functions are covered under [Dynamic Updates](../semantics/dynamic-updates.md#initialization-and-runtime-updates). `Guidance_Reasoner` and `Finite_State_Machine` are explained under [Semantic Reasoning](../semantics/reasoning.md).

## Reading and inspecting the model

Use the [ontology-inspection examples](../semantics/annotation.md#reading-and-inspecting-the-model) to load the file and inspect vessel properties and classes without opening the graphical interface. Exact source signatures remain in the [function catalog](../reference/functions.md).

## Related publications

- [Integrating rule awareness and semantic reasoning in collision-free vessel path planning](../publications.md#rule-aware-collision-free-planning) — semantic traffic-rule information and encounter guidance.
- [An intelligent agent-based resilient framework for marine vessel mission adaptations](../publications.md#resilient-mission-adaptation) — the wider architecture and control perspective.

Source: [`classes.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/classes.py#L34-L148), [`helper_utils.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/helper_utils.py#L350-L361), and [`intel_guidance.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py#L83-L203).
