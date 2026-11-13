---
type: workflow
title: Scenario configuration and results
description: JSON dispatch, traffic and ontology assets, map boundaries, video and CSV result consumers.
tags: [configuration, scenarios, output, assets]
verified:
  - by: openwiki/0.9.0
    at: 2026-11-13T18:32:56.313Z
sources:
  - id: openwiki-source-83aec357e72fc620b7a2945d
    resource: repo://plot_results.py
  - id: openwiki-source-5044576d448aff1321f8847c
    resource: repo://src/modules/helper_utils.py
  - id: openwiki-source-e565ed0266e14cc3286d5eb2
    resource: repo://src/pipelines/sim_ag_PID.py
  - id: openwiki-source-8cc5a8d4446a67fdaa0ac2ef
    resource: repo://src/pipelines/sim_ag_upg.py
generated: { by: "codex", at: "2026-11-13T18:32:56.313Z" }
---

# Scenario configuration and results

## Select a pipeline

`main.py` selects `input/SimParam_anti_grounding.json` in its `input_file` assignment. Its positional command-line argument changes the output directory, not the input configuration. `load_sim` processes the JSON `imports` array with Python importlib, supporting module imports with optional aliases and named imports. Startup expects the returned dictionary to contain `run_simulation`. Treat these files as executable configuration: imported Python modules run normally, and import/lookup failures are not converted into friendly configuration errors.

| Input | Selected pipeline | Source status |
| --- | --- | --- |
| `SimParam_anti_grounding.json` | `sim_ag_upg` | Current default; synthetic channel and anti-grounding MPC |
| `SimParam_WC.json` | `sim_colav` | Traffic/encounter scenario with current interface gaps |
| `SimParam_ASKO.json` | `sim_ftc` | Older four-argument pipeline, incompatible with six-argument startup |
| `SimParam.json` | None | No `imports`; does not provide startup's required callable |

`sim_ag_PID.py` provides a PID variant: its active loop appends a PID rudder command, while anti-grounding MPC construction and command extraction are commented out. To select this variant, configure the JSON import to resolve its `run_simulation` callable.

The default pipeline reads simulation time, scale, time step and lookahead distance. It constructs its own route, initial conditions, force switches and controller settings. Some JSON keys are therefore scenario-specific or unused; there is no shared schema validation layer enforcing a universal configuration contract.

## Vessel and map assets

Startup loads `assets/database.xml`, not the similarly named root XML file. Vessel dynamics read ontology parameters from named individuals, while history arrays are modified during the run. Keep individual names aligned with the vessel mapping described in [state lifecycle](../architecture/simulation-state.md).

The collision pipeline reads `assets/vessel_traffic.csv` for initial conditions, route endpoints and draw/start times. The positional CSV reader builds `[u,v,r,x,y,0,heading]`, looks up the vessel by name and invokes route planning. The separate waypoint reader groups points by `mmsi` and matches a vessel to the nearest route start within 5 m, after coordinate offsets. It aligns heading to the first segment when available and falls back to a single origin point if no match exists. It does not match by vessel name or MMSI directly.

The effective `mapper.create_map` is the later definition in the module: it reads semicolon-separated coordinate arrays from the map CSV and draws two-point entries as rectangles. The earlier same-name function is shadowed. The default anti-grounding scenario instead uses the synthetic inland renderer and shoreline ray measurements.

`fetch_map` is a separate geospatial path that reads cached Natural Earth files where available and otherwise fetches remote GeoJSON. Those network/cache requirements are not part of the synthetic default route. Check callers before provisioning GIS data for a scenario.

## Results and consumers

The default pipeline records `grounding_cs_video.mp4` in the supplied output directory at 30 fps. Simulation time advances by the JSON timestep; the rendering clock limit is 300 ticks/s, so playback fps, wall-clock execution and simulated time are different quantities. The pipeline exports video after its loop and then invokes grounding plots.

Its `ag_data.csv` export block is commented out. `plots_grounding` nevertheless reads `output/ag_data.csv` at multiple locations and shows plots when output is not suppressed. A checked-in historical CSV is not automatically refreshed by the default run. Several plotting functions also hard-code `output/`, so the CLI output directory does not redirect every result consumer.

`export_simulation_data`, used by the collision pipeline, creates its target directory and writes `<vessel.type>.csv` and `<vessel.type>_estimation.csv`. The main table contains time, measured positions/heading, heading reference, speed, rudder and bank distances. Mismatched primary columns are truncated to the shortest length, with a warning. Estimation columns have a separate mismatch handling defect described in [validation](../testing/validation.md).

`plot_results.py` independently searches `output/vessel_*_comprehensive.csv`, then produces trajectory and heading PNG files. That glob does not match the exporter's current filenames; reconcile producer and consumer naming before using it as a postprocessing command.

See [quickstart](../quickstart.md), [guidance/control](../concepts/guidance-control.md), and [validation gaps](../testing/validation.md) before extending or comparing scenarios.
