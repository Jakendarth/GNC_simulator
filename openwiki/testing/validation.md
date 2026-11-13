---
type: testing
title: Validation and known verification gaps
description: Source-grounded interface checks, absent automated coverage and suggested focused simulation validation.
tags: [testing, validation, integration]
verified:
  - by: openwiki/0.9.0
    at: 2026-11-13T18:28:11.946Z
sources:
  - id: openwiki-source-833e692518af9eeaf8564cc6
    resource: repo://main.py
  - id: openwiki-source-5044576d448aff1321f8847c
    resource: repo://src/modules/helper_utils.py
  - id: openwiki-source-fee98f614b96687de6862995
    resource: repo://src/modules/intel_guidance.py
  - id: openwiki-source-ba97ad01e03cf6f487b711ac
    resource: repo://src/pipelines/sim_colav.py
  - id: openwiki-source-fcbf79016d46d2ad66262a00
    resource: repo://src/pipelines/sim_ftc.py
generated: { by: "codex", at: "2026-11-13T18:28:11.946Z" }
---

# Validation and known verification gaps

No repository-owned automated test suite, pytest configuration or test command was found during initialization. `pyproject.toml` declares runtime dependencies without a test configuration. Existing CSV, videos and notebooks are examples or analysis artifacts; they do not establish that current source passes regression tests. This wiki was researched from source and available assets, without running the GUI simulation or numerical solvers.

## Source contracts to check before running alternate scenarios

| Boundary | Current source finding | Consequence |
| --- | --- | --- |
| Entry point → FTC | `main` passes six arguments; `sim_ftc.run_simulation` accepts four | ASKO configuration is not directly compatible with current startup |
| FTC → vessel | FTC calls `add_sensor_faults`; vessel defines `include_sensor_faults` | Fixing argument count alone does not establish FTC compatibility |
| Collision pipeline → guidance | Caller unpacks five values; reasoner returns six and processes a map argument | WC scenario requires interface reconciliation |
| Collision pipeline → drawing | Caller supplies six arguments; `draw_vessels` requires map bounds as well | Additional integration gap remains after guidance repair |
| Weights → controller | Vessel wrapper replaces supplied weights | Pipeline tuning vector is ineffective through this interface |
| Export → plot script | Export writes `<type>.csv`; script searches `vessel_*_comprehensive.csv` | Current exports are not discovered automatically |

These findings follow from function definitions and call sites; they are not claims that a runtime regression test has reproduced each failure. See [scenario workflow](../workflows/scenarios-results.md) for asset and output contracts.

## Focused checks for future changes

The following are suggested checks, not existing tests:

- For state adapters, use distinctive values in `[u,v,r,x,y,z,psi]` and verify integration removes only sinkage and MPC receives `[x,y,psi,u,v,r]`.
- For LOS, cover duplicate waypoints, projection beyond a segment, wrapped headings, terminal waypoint behavior and non-unit timestep.
- For channel depth, check centerline depth at x=0 and x=1200, side-slope boundaries, and zero outside the modeled channel. Compare geometry under each supported scale.
- For bank rays, verify signed port/starboard distances and absent-intersection infinities.
- For history/export, compare all column lengths and verify filenames against consumers. The estimation-length mismatch branch currently truncates `log_data1` rather than `log_data2`, so it does not repair unequal estimation columns.
- For controller changes, use a short simulation and inspect finite commands, solver status, heading error and clearance independently. The MPC's clearance constraint is soft, so solver completion alone is insufficient evidence of a clearance margin.

## Manual scenario validation

Run from the repository root with the environment installed and a writable output directory. Shorten simulation time in the selected input when evaluating a change, record the exact configuration and dependency environment, and inspect video and current-run histories. Random sensor perturbations are present without an explicit seed, so exact trajectory equality is not a suitable default oracle.

Grounding plots read `output/ag_data.csv` even though the default pipeline's CSV-writing block is commented out. Confirm plot inputs belong to the current run; a preexisting CSV can mix historical and current results. Plotting and video export are part of the pipeline completion path and may fail independently of integration.

See [quickstart](../quickstart.md), [state lifecycle](../architecture/simulation-state.md), and [guidance/control](../concepts/guidance-control.md).
