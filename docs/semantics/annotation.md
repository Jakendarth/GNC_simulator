# Semantic Annotation

Semantic annotation assigns meaning to the objects, values and relationships in the simulator. It describes not only a vessel's motion, but also the traffic rules, sensors, environment and human/system context associated with that motion.

Start here to understand **what the ontology represents**. Continue with [Dynamic Updates](dynamic-updates.md) to see how values change during a run, then [Semantic Reasoning](reasoning.md) to see how Python functions use them to make decisions.

## Classes, individuals and properties

The Web Ontology Language (OWL) describes categories called **classes**, named objects called **individuals**, and relationships or values called **properties**. Object properties connect individuals; data properties attach numeric or textual values. For the underlying terminology, see the [W3C OWL 2 Primer](https://www.w3.org/TR/owl2-primer/).

`main.py` loads [`assets/database.xml`](https://github.com/Jakendarth/GNC_simulator/blob/main/assets/database.xml), serialized in RDF/XML. The similarly named XML file at the repository root is not the startup ontology.

## A vocabulary spanning the maritime system

The current ontology declares six classes. Their variety matters: the semantic model can describe physical components, operating context, traffic knowledge and operator/design concepts in one vocabulary.

| Class | What it describes | Examples present in the ontology | What to understand before using it |
| --- | --- | --- | --- |
| `Vessel` | Named vessels and their physical/motion data | `Own`, `Other1`–`Other4`, `ASKO`, `Pusher_barge`, `Tito_Neri`, `JB114`, `KOMODO`, `NOORDPOOL`, `MAI_W` | Geometry, draught, force coefficients and runtime motion properties belong to vessel individuals. Not every named vessel is automatically selectable through the Python constructor. |
| `Regulations` | Traffic-rule knowledge and situation groups | `COLREGS/Rule_14`, `COLREGS/Rule_15`, `BPR/Article_604`, and COLREGS/BPR/CCNR situation groups | The file represents individual rules, descriptive comments, group relations and activation values. Runtime reasoning uses a subset of this knowledge. |
| `Environment` | The operating context | `Operational_environment`, with `environment_type` and information-flow links from `GNSS` and `Radar` | This semantic individual is distinct from the numerical depth profile or drawn shoreline used by a pipeline. |
| `Sensors` | Named sources of information | `GNSS`, `Radar`, `Gyro`, `Accelerometer`, `Anemometer`, `Camera` | Sensor individuals describe information sources. Their presence does not create hardware drivers or make every source active in the simulation. |
| `Operators` | Named operator profiles | `Profile_1`, `Profile_2`, `Profile_3` | Profiles are represented in the ontology; the default navigation loop does not implement a profile-selection interface. |
| `Design_space` | A declared category for design-related knowledge | The class is declared; the current XML has no explicitly typed named individual in it | A schema declaration is different from a populated design model or an implemented design optimizer. |

The class names above are exact code identifiers. In prose, “traffic rules” refers to individuals of `Regulations`, while “vessels” and “sensors” refer to individuals of `Vessel` and `Sensors`.

## What annotation adds to a value

A number becomes useful to other functions when its meaning, owner and convention are known:

| Annotation | Meaning in the simulator |
| --- | --- |
| `Own.L`, `Own.B`, `Own.Td` | Vessel length, breadth and draught used by numerical models |
| `Own.yx`, `Own.yy`, `Own.ypsi` | Measured position and heading histories, distinct from physical trajectory histories |
| `Own.psi_d`, `Own.rudder_angle` | A desired heading and an actuator command, not the same quantity |
| `COLREGS/Rule_14` | A named traffic-rule individual with a descriptive comment and activation data |
| `Operational_environment.inputFrom` | Declared information-source relationships, including GNSS and Radar |

The object property `inputFrom` is declared as the inverse of `outputTo`. These links describe relations in the knowledge model; they do not by themselves schedule Python functions or construct a complete executable pipeline.

## Binding annotations to a Python vessel

Creating `vessel("Own", ontology, x0)` binds a Python vessel object to `ontology.Own`. The constructor handles `Own`, `Other1`, `Other2` and `Other3` explicitly, and maps all other names to `Other4`. Adding a named individual to the XML alone does not extend that constructor mapping.

Physical parameters are commonly read through their first value, such as `L[0]`. Runtime measurements are commonly read through their latest value, such as `yx[-1]`. This distinction prepares you for [Dynamic Updates](dynamic-updates.md).

## Reading and inspecting the model

Activate the simulator environment and run the following Python code from the repository root. It loads the ontology and prints `Own` vessel parameters and properties without opening the graphical interface. Printed property order may vary.

```python
from pathlib import Path
from owlready2 import get_ontology

root = Path.cwd()  # repository root
ontology = get_ontology(str(root / "assets/database.xml")).load()
print(ontology.Own.L[0])
print(ontology.Own.Td[0])
for prop in ontology.Own.get_properties():
    print(prop.python_name, prop[ontology.Own])
```

Continue with [pipeline composition](../pipelines.md) and [the function catalog](../reference/functions.md).


To inspect the class vocabulary as well, run this fragment in the same Python session after loading `ontology`:

```python
for category in ontology.classes():
    print(category.name, [item.name for item in category.instances()])
```

## Annotation and runtime behavior

Annotation provides vocabulary and data. [Semantic Reasoning](reasoning.md) explains the specific traffic-role decisions made by current Python functions, including the limits of regulation activation and the `owl:sameAs` grouping choice. The wider ontology should not be read as a claim that every represented class or rule has an active runtime implementation.

## Related publications

- [Integrating rule awareness and semantic reasoning in collision-free vessel path planning](../publications.md#rule-aware-collision-free-planning) — semantic traffic-rule information and encounter guidance.
- [An intelligent agent-based resilient framework for marine vessel mission adaptations](../publications.md#resilient-mission-adaptation) — the wider architecture and control perspective.

Source: [`classes.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/classes.py#L34-L148), [`helper_utils.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/helper_utils.py#L350-L361), and [`intel_guidance.py`](https://github.com/Jakendarth/GNC_simulator/blob/main/src/modules/intel_guidance.py#L83-L203).
