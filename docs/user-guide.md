# Running and inspecting results

## Configure a run

1. Select a supported [scenario](scenarios.md).
2. Edit its JSON file under `input/` for the parameters actually consumed by that pipeline.
3. Set `input_file` in `main.py` to that file.
4. Run `uv run python main.py output` from the repository root.

The default JSON imports `run_simulation` from `sim_ag_upg`. Its pipeline reads simulation time, timestep, scale and lookahead distance, while constructing its own route and initial conditions. Keys in another scenario's JSON are not a universal configuration interface.

!!! tip "Keep a record of the run"
    Save the chosen configuration and note the source revision when comparing results. The simulator uses random sensor perturbations without an explicit seed in this path.

## Watch the simulation

The default scenario displays the vessel in a synthetic channel. Guidance calculates a heading reference, MPC supplies propeller and rudder commands, and the vessel model advances the state. It stops when the configured time is reached, the vessel reaches the route's final x position, or the window is closed.

For the order of control, integration and rendering, see [Simulation lifecycle and vessel state](openwiki/architecture/simulation-state.md).

## Inspect results

| Result | Current behavior |
| --- | --- |
| Default scenario video | `grounding_cs_video.mp4` in the selected output directory |
| Grounding plots | Opened after video export; include comparison data read from `output/ag_data.csv` |
| Collision-pipeline CSV export | `<vessel.type>.csv` and `<vessel.type>_estimation.csv`, when that pipeline reaches export |
| Standalone `plot_results.py` | Searches `output/vessel_*_comprehensive.csv`, which does not match current exporter filenames |

The output-directory argument does not redirect every plotting path. Confirm that a CSV belongs to the intended run before using it for analysis. Existing videos and CSV files in the repository are artifacts, not proof that the current scenario completed successfully.

See [Configuration, assets and results](openwiki/workflows/scenarios-results.md) for detailed producer/consumer contracts, and [Validation and known gaps](openwiki/testing/validation.md) when diagnosing a failure.
