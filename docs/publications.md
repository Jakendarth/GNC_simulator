# Research and publications

Use this page to find the research behind the wiki's topics and copy citations. Each entry gives a bibliographic reference, a short account of its relevance, links to related documentation and BibTeX.

Publication summaries follow the linked publisher or institutional records. The conference entry retains the authors' supplied status. The reading suggestions connect topics; they do not establish that this repository implements every method or reproduces every experiment in a paper.

## Rule-aware collision-free planning

**Nikos Kougiatsos, Abhishek Dhyani and Vasso Reppa.** “Integrating rule awareness and semantic reasoning in collision-free vessel path planning.” *Control Engineering Practice* **175** (2026), 107113.

[Publisher / DOI](https://doi.org/10.1016/j.conengprac.2026.107113)
[TU Delft record](https://research.tudelft.nl/en/publications/integrating-rule-awareness-and-semantic-reasoning-in-collision-fr/)

Read this paper for the relationship between semantic traffic-rule information, encounter roles, collision-free path envelopes and line-of-sight (LOS) tracking. Its head-on and crossing case studies provide context for the wiki's encounter-planning material. [Publication record](https://research.tudelft.nl/en/publications/integrating-rule-awareness-and-semantic-reasoning-in-collision-fr/).

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

[Publisher / DOI](https://doi.org/10.1109/OJITS.2025.3539419)
[TU Delft record](https://research.tudelft.nl/en/publications/an-intelligent-agent-based-resilient-framework-for-marine-vessel-/)

Read this paper for a broader view of how semantic information connects vessel architecture, control design and mission adaptation. The framework includes decisions about sensor and energy-management switching. The current navigation pipelines cover a narrower scope than that complete framework. [Publication record](https://research.tudelft.nl/en/publications/an-intelligent-agent-based-resilient-framework-for-marine-vessel-/).

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

**Bibliographic status:** the author-supplied citation states “to appear in the proceedings.” The official [TRA 2026 program booklet](https://traconference.eu/events/traconferece/pdfs/6eb61f9a-1779298793.pdf) lists the title and authors. A final proceedings DOI and page range remain unverified in this documentation.

**Read alongside:** [observer-informed LOS](guidance/line-of-sight.md), [control](topics/control.md), and [sensing and histories](topics/dynamics.md#measurements-and-bank-geometry). These are topic suggestions based on the supplied title. A function-by-function mapping to the paper's method has not been verified.

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

For the Semantics tab, start with [rule-aware collision-free planning](#rule-aware-collision-free-planning). Continue with [resilient mission adaptation](#resilient-mission-adaptation) to place the ontology in a broader system-architecture context.

## Guidance and virtual sensing

For the Guidance tab, use [rule-aware collision-free planning](#rule-aware-collision-free-planning) to study encounter planning. The [virtual-sensor paper](#virtual-sensor-informed-motion-planning) is related reading for observer-informed motion planning; its bibliographic and code-mapping limits are noted above.

## Control, dynamics and resilience

For the Navigation and Control sections, use the [resilient-framework paper](#resilient-mission-adaptation) and [virtual-sensor paper](#virtual-sensor-informed-motion-planning) as context. The wiki's controller equations and state conventions describe current source and should be checked there when implementing changes.

## Pipeline composition and mission adaptation

For the Pipelines tab, read the [resilient-framework paper](#resilient-mission-adaptation) for architectural motivation and [rule-aware planning](#rule-aware-collision-free-planning) for the encounter-guidance sequence.

## Cite the software

Use [`CITATION.cff`](https://github.com/Jakendarth/GNC_simulator/blob/main/CITATION.cff) to cite the software, and cite the relevant paper when discussing a research contribution. To update a publication, revise this page and its topic links, then run the strict build described in the [documentation maintenance guide](developer/documentation.md).
