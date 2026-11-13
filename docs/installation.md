# Installation

Use these steps to prepare the simulator environment and launch the default anti-grounding scenario. To build only the website, follow [Maintaining this manual](developer/documentation.md).

## Requirements

You need Git, **Python 3.12 or newer**, **uv**, and a graphical desktop environment. The normal simulation opens a Pygame window and interactive plots. `pyproject.toml` and `uv.lock` define the project environment, including numerical optimization, geospatial and video packages.

Install uv using its [official installation instructions](https://docs.astral.sh/uv/getting-started/installation/).

## Get the source and dependencies

Clone the repository, enter its root directory and install the project environment:

```sh
git clone https://github.com/Jakendarth/GNC_simulator.git
cd GNC_simulator
uv sync
```

`uv sync` installs dependencies into the project environment. Keep the repository root as your working directory for the commands below: several asset and plotting paths are relative to it.

## Launch the default scenario

Create the output directory and start the application:

```sh
mkdir -p output
uv run python main.py output
```

You should see a Pygame window showing the vessel and a synthetic channel. `main.py` currently selects `input/SimParam_anti_grounding.json`. The positional argument `output` names the output directory; omitting it uses the same default directory.

After the loop, the pipeline exports `grounding_cs_video.mp4` and invokes grounding plots. The plots also read the existing `output/ag_data.csv` as comparison data. The default pipeline does not regenerate that CSV, so distinguish its contents from the trajectory of your new run.

Continue with the [User Guide](user-guide.md) to choose inputs and inspect results. Read the [scenario compatibility notes](scenarios.md) before selecting another pipeline. These launch instructions follow the current entry point; the GUI simulation was not run as part of the documentation review.
