# Research and publications

The papers below provide the research context for the wiki's topic tabs. Their contribution summaries come from the linked publisher/institutional records, or from author-supplied bibliographic information where noted. Topic-to-code mappings are reading guidance; they do not establish that every feature or experiment in a paper is implemented by this repository.

## Rule-aware collision-free planning

**Nikos Kougiatsos, Abhishek Dhyani and Vasso Reppa.** “Integrating rule awareness and semantic reasoning in collision-free vessel path planning.” *Control Engineering Practice* **175** (2026), 107113.

[Publisher / DOI](https://doi.org/10.1016/j.conengprac.2026.107113){ .md-button }
[TU Delft record](https://research.tudelft.nl/en/publications/integrating-rule-awareness-and-semantic-reasoning-in-collision-fr/){ .md-button }

The paper connects semantic traffic-rule information, encounter-role assignment, collision-free path envelopes and LOS tracking in multi-environment navigation. Its case study includes head-on and crossing encounters. This makes it the closest reading companion to the wiki's semantic and encounter-guidance topics. [Publication record](https://research.tudelft.nl/en/publications/integrating-rule-awareness-and-semantic-reasoning-in-collision-fr/).

**Read alongside:** [Semantics](topics/semantics.md), [Guidance](topics/guidance.md), [encounter composition](pipelines.md#example-3-connect-encounter-reasoning-to-los), and [encounter recordings](demonstrations.md#encounter-recordings).

```bibtex
@article{kougiatsos2026ruleawareness,
  author = {Kougiatsos, Nikos and Dhyani, Abhishek and Reppa, Vasso},
  title = {Integrating rule awareness and semantic reasoning in collision-free vessel path planning},
  journal = {Control Engineering Practice},
  volume = {175},
  pages = {107113},
  year = {2026},
  doi = {10.1016/j.conengprac.2026.107113}
}
```

## Resilient mission adaptation

**N. Kougiatsos, E. L. Scheffers, M. C. Van Benten, D. L. Schott, P. De Vos, R. R. Negenborn and V. Reppa.** “An intelligent agent-based resilient framework for marine vessel mission adaptations.” *IEEE Open Journal of Intelligent Transportation Systems* **6** (2025), 184–203.

[Publisher / DOI](https://doi.org/10.1109/OJITS.2025.3539419){ .md-button }
[TU Delft record](https://research.tudelft.nl/en/publications/an-intelligent-agent-based-resilient-framework-for-marine-vessel-/){ .md-button }

The paper describes semantic connections between vessel architecture and control design, with decision support for mission changes and resilient automation configurations. Its wider framework includes sensor and energy-management switching. These are research context for semantic composition and resilience; the current navigation pipelines should not be read as implementations of the paper's entire framework. [Publication record](https://research.tudelft.nl/en/publications/an-intelligent-agent-based-resilient-framework-for-marine-vessel-/).

**Read alongside:** [semantic vocabulary](topics/semantics.md), [state and sensing](topics/dynamics.md), and [pipeline composition](pipelines.md).

```bibtex
@article{kougiatsos2025resilience,
  author = {Kougiatsos, N. and Scheffers, E. L. and Van Benten, M. C.
            and Schott, D. L. and De Vos, P. and Negenborn, R. R. and Reppa, V.},
  title = {An intelligent agent-based resilient framework for marine vessel mission adaptations},
  journal = {IEEE Open Journal of Intelligent Transportation Systems},
  volume = {6},
  pages = {184--203},
  year = {2025},
  doi = {10.1109/OJITS.2025.3539419}
}
```

## Virtual sensor-informed motion planning

**N. Kougiatsos and V. Reppa.** “Virtual sensor-informed motion planning for safe autonomous waterborne navigation.” *Transport Research Arena 2026*, Budapest, Hungary (2026).

**Status:** to appear in the proceedings, according to the author-supplied citation. The official [TRA 2026 program booklet](https://traconference.eu/events/traconferece/pdfs/6eb61f9a-1779298793.pdf) lists this title and authors. A final proceedings DOI and page range have not been supplied or verified here.

**Read alongside:** [observer-informed LOS](topics/guidance.md#line-of-sight-tracking), [control](topics/control.md), and [sensing and histories](topics/dynamics.md#measurements-and-bank-geometry). This grouping follows the supplied title; it does not assign specific current code functions to the paper's method without a verified full-text mapping.

```bibtex
@inproceedings{kougiatsos2026virtualsensor,
  author = {Kougiatsos, N. and Reppa, V.},
  title = {Virtual sensor-informed motion planning for safe autonomous waterborne navigation},
  booktitle = {Transport Research Arena 2026},
  address = {Budapest, Hungary},
  year = {2026},
  note = {To appear in the proceedings; status from author-supplied citation}
}
```

## Semantics and rule-aware planning

Start with [rule-aware collision-free planning](#rule-aware-collision-free-planning) for regulation activation and encounter-role concepts, then [resilient mission adaptation](#resilient-mission-adaptation) for the broader semantic architecture perspective.

## Guidance and virtual sensing

Pair [rule-aware collision-free planning](#rule-aware-collision-free-planning) with [virtual sensor-informed motion planning](#virtual-sensor-informed-motion-planning) when reading the guidance tab. See the status and implementation-mapping notes above.

## Control, dynamics and resilience

The [resilient-framework paper](#resilient-mission-adaptation) and [virtual-sensor paper](#virtual-sensor-informed-motion-planning) are contextual reading. Controller equations and numerical state contracts in this wiki are documented from current source.

## Pipeline composition and mission adaptation

The [resilient-framework paper](#resilient-mission-adaptation) motivates the architectural reading, while [rule-aware planning](#rule-aware-collision-free-planning) accompanies the encounter-guidance composition.

## Cite the software

Use the repository's [`CITATION.cff`](https://github.com/Jakendarth/GNC_simulator/blob/main/CITATION.cff) for the software citation. Cite the relevant paper separately when discussing its research contribution. Adding a new paper or updating proceedings status requires only this page and the related topic links, followed by a strict documentation build.
