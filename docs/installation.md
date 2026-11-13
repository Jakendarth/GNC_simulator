# Installation

## Requirements

The project declares **Python 3.12 or newer** and uses **uv** for dependency management. The normal simulation opens a Pygame window and interactive plots, so use a graphical desktop environment. The project includes numerical optimization, geospatial and video dependencies; installing the documentation alone does not install the simulator.

Install uv using its [official installation instructions](https://docs.astral.sh/uv/getting-started/installation/).

## Get the source and dependencies

```sh
git clone https://github.com/Jakendarth/GNC_simulator.git
cd GNC_simulator
uv sync
```

Run commands from the repository root. The application uses repository-relative assets and some plotting paths still refer directly to `output/`.

## Launch the default scenario

```sh
mkdir -p output
uv run python main.py output
```

The current `main.py` selects `input/SimParam_anti_grounding.json`. The final argument specifies the output directory; it does not select a JSON file. With no argument, the output directory defaults to `output`.

The pipeline records a video and invokes grounding plots after the simulation loop. Those plots also read the preexisting `output/ag_data.csv` comparison data; the default pipeline does not regenerate that CSV.

Continue with the [User Guide](user-guide.md) and [scenario compatibility notes](scenarios.md). For website-only setup, see [Maintaining this manual](developer/documentation.md).
