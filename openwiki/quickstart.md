---
type: quickstart
title: Quickstart and task map
description: Start the maritime GNC simulator and find source-grounded documentation for state, control, configuration and validation.
tags: [quickstart, navigation, simulation]
verified:
  - by: openwiki/0.9.0
    at: 2026-11-13T18:28:11.946Z
sources:
  - id: openwiki-source-5a1fdf2bcd77bebcf6fee63f
    resource: repo://input/SimParam_anti_grounding.json
  - id: openwiki-source-833e692518af9eeaf8564cc6
    resource: repo://main.py
generated: { by: "codex", at: "2026-11-13T18:28:11.946Z" }
---

# Quickstart and task map

This repository is a Python maritime guidance, navigation and control simulator with Pygame visualization, ontology-backed vessel parameters and histories, numerical dynamics, and PID/MPC controllers. The current entry point selects the synthetic anti-grounding MPC scenario.

## Start a simulation

The project uses uv and declares Python >=3.12. From the repository root, the expected setup and launch commands are:

```sh
uv sync
mkdir -p output
uv run python main.py output
```

These commands are usage guidance derived from the manifest and entry point; initialization did not run a simulation. A graphical environment is needed for the normal Pygame window and interactive plots, and numerical/video dependencies must be usable in the local environment.

To choose a scenario, change `input_file` in `main.py`. The default is `input/SimParam_anti_grounding.json`, which imports `run_simulation` from `sim_ag_upg`. The positional CLI argument is the output directory. The default pipeline creates a synthetic route and vessel, executes LOS → MPC → dynamics → rendering, exports `grounding_cs_video.mp4`, and opens grounding plots. Some plots still use hard-coded `output/` inputs, including a historical comparison CSV.

Do not assume every bundled input is directly runnable with current interfaces. The WC and ASKO paths have source-level integration gaps described in the validation page.

## Find the right system

| Task | Read | Main source anchors |
| --- | --- | --- |
| Understand time stepping, state order or ontology histories | [Simulation lifecycle and vessel state](architecture/simulation-state.md) | `main.py`, `classes.py`, `sim_ag_upg.py` |
| Change waypoint tracking, encounter roles, actuator limits or clearance | [Guidance and anti-grounding control](concepts/guidance-control.md) | `intel_guidance.py`, `traffic_planner.py`, `channel.py` |
| Select inputs, replace traffic/map assets or consume results | [Scenario configuration and results](workflows/scenarios-results.md) | `helper_utils.py`, `mapper.py`, `plots.py`, `plot_results.py` |
| Check compatibility and plan meaningful verification | [Validation and known verification gaps](testing/validation.md) | Pipeline call sites, state adapters and exporters |

## Evidence and maintenance

The wiki reflects current source and available repository artifacts. No automated test suite was found; example CSV and videos do not prove current runtime correctness. Each factual page has repository-grounded Claims managed by OpenWiki. Verify consequential behavior against source, especially controller tuning, coordinate scales and older pipeline interfaces.

After source changes, request an OpenWiki update through the repository maintenance integration. OpenWiki owns generated indexes, evidence records and run metadata; page updates should use its resumable lifecycle.
