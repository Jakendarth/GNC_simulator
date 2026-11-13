# Running and inspecting results

Use this guide after [installing the simulator](installation.md). It explains how to select inputs, run the application and identify which outputs belong to your run.

## Configure a run

1. Review the [scenario list](scenarios.md). Use the default anti-grounding scenario for an initial run.
2. Open its JSON file under `input/` and adjust the parameters read by that pipeline.
3. If you selected a different file, set `input_file` in `main.py` to its repository-relative path.
4. From the repository root, ensure the output directory exists with `mkdir -p output`, then run `uv run python main.py output`.

The JSON `imports` entry selects a Python `run_simulation` function. In the default file, that function comes from `sim_ag_upg`.

| Default parameter | Where it is used |
| --- | --- |
| `Simulation Time` | Upper limit for simulated time |
| `Time step` | Integration interval, time increment and MPC timestep |
| `scale` | Route/screen conversion and several geometry calls |
| `Lookahead distance` | Passed to the drawing function; standard LOS uses its own `5 * L` lookahead |

The pipeline constructs its route and initial state in Python. Other JSON keys are not automatically applied just because they appear in the file. Consult the [configuration reference](openwiki/workflows/scenarios-results.md) before transferring settings between scenarios.

:::{admonition} Keep a record of the run
:class: tip

Save the input configuration, source revision and output files together. The default path uses random sensor perturbations without an explicit seed, so repeated runs need not produce identical trajectories. Both anti-grounding variants use the same video filename; preserve a recording before running another comparison.

:::

## Watch the simulation

The default scenario displays a vessel in a synthetic channel. Line-of-sight (LOS) guidance calculates a heading reference; model predictive control (MPC) supplies propeller and rudder commands; numerical integration advances the vessel state.

The loop stops when simulated time reaches its limit, the vessel's x coordinate reaches the route's final x coordinate, or you close the window. Video export and plotting follow the loop, so closing the simulation window can still be followed by output processing.

For the order of control, integration and rendering, see [Simulation lifecycle and vessel state](openwiki/architecture/simulation-state.md).

## Inspect results

| Result | Current behavior |
| --- | --- |
| Default scenario video | `grounding_cs_video.mp4` in the selected output directory |
| Grounding plots | Opened after video export; include comparison data read from `output/ag_data.csv` |
| Collision-pipeline CSV export | `<vessel.type>.csv` and `<vessel.type>_estimation.csv`, when that pipeline reaches export |
| Standalone `plot_results.py` | Searches `output/vessel_*_comprehensive.csv`, which does not match current exporter filenames |

Use the video path to locate the default run's recording. Treat the grounding comparison CSV separately: its presence does not mean it was written by this run. Some plotting paths remain fixed under `output/`, even if you supplied another output directory.

If `plot_results.py` reports no input files after an export, check the filename pattern in the table above. The current producer and consumer names differ; running the script alone does not reconcile them.

See [Configuration, assets and results](openwiki/workflows/scenarios-results.md) for detailed producer/consumer contracts, and [Validation and known gaps](openwiki/testing/validation.md) when diagnosing a failure.
