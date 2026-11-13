---
title: GNC Simulator
description: A practical guide to installing, running and understanding the maritime GNC simulator.
---

# GNC Simulator

## Maritime guidance, navigation and control

GNC Simulator combines vessel dynamics, guidance algorithms, PID and model predictive control with Pygame visualization. This manual explains how to run the current scenarios, inspect results and navigate the implementation.

[Install the simulator](installation.md){ .md-button .md-button--primary }
[Run your first scenario](openwiki/quickstart.md){ .md-button }

## Choose your next step

| Goal | Start here |
| --- | --- |
| Prepare Python and the project environment | [Installation](installation.md) |
| Choose inputs and inspect outputs | [User Guide](user-guide.md) |
| Compare the bundled scenario configurations | [Simulation Scenarios](scenarios.md) |
| Understand LOS guidance, PID and anti-grounding MPC | [GNC Algorithms](openwiki/concepts/guidance-control.md) |
| Extend the simulation or review state conventions | [Developer Guide](openwiki/architecture/simulation-state.md) |

!!! note "Current implementation status"
    The default configuration selects the synthetic anti-grounding MPC scenario. Older scenarios have documented interface gaps. No repository-owned automated simulation tests were found when the OpenWiki reference was initialized; see [validation and known gaps](openwiki/testing/validation.md).

## About this manual

The technical reference comes from the repository's existing OpenWiki documentation. The website presents those pages without modifying their generated Markdown or evidence records. Installation and workflow pages provide a user-facing route into that reference.

Source code, issues and citation information are available in the [GitHub repository](https://github.com/Jakendarth/GNC_simulator). See the repository's [citation metadata](https://github.com/Jakendarth/GNC_simulator/blob/main/CITATION.cff) and [Apache 2.0 license](https://github.com/Jakendarth/GNC_simulator/blob/main/LICENSE).
