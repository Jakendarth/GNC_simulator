[![Python 3.12](https://img.shields.io/badge/python-3.12-blue.svg)](https://www.python.org/downloads/release/python-3120/)
[![image](https://img.shields.io/pypi/v/uv.svg)](https://pypi.python.org/pypi/uv)
[![License: CC BY 4.0](https://img.shields.io/badge/License-CC_BY_4.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/)
[![uv](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/uv/main/assets/badge/v0.json)](https://github.com/astral-sh/uv)

The project uses UV to manage its dependencies, an extremely fast Python package and project manager, written in Rust. 

More information on working with UV: https://github.com/astral-sh/uv?tab=readme-ov-file#uv

The `input/<filename>.json` file determines the parameters used for the simulation (e.g., simulation time, simulation target (pipeline), initial conditions etc.)
You can make your own file and specify as many parameters you need. Remember to change the input file variable in `main.py` before you run the simulation.

Plot and video files can be stored in `output/` as .png images.


