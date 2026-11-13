# GNC Simulator

[![Python 3.12](https://img.shields.io/badge/python-3.12-blue.svg)](https://www.python.org/downloads/release/python-3120/)
[![uv](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/uv/main/assets/badge/v0.json)](https://github.com/astral-sh/uv)
[![image](https://img.shields.io/pypi/v/uv.svg)](https://pypi.python.org/pypi/uv)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![DOI](https://zenodo.org/badge/1271233098.svg)](https://doi.org/10.5281/zenodo.20731296)

> [Decription pending]

---

## Documentation

The technical wiki is configured for **[https://jakendarth.github.io/GNC_simulator/](https://jakendarth.github.io/GNC_simulator/)**. It becomes available after the documentation changes are reviewed, merged and deployed through GitHub Pages.

The wiki teaches Guidance, Navigation and Control through separate lessons on line-of-sight laws, collision-free planning, sensors and observers, PID feedback and model predictive control. Pipeline examples connect the functions. It also includes semantic vessel knowledge, four embedded recordings, related publications and an automatically generated function catalog. Existing `openwiki/` pages remain available with their generated sources and evidence records preserved.

For a local preview with Python 3.13:

```sh
python3.13 -m venv .venv-docs
source .venv-docs/bin/activate
python -m pip install -r requirements-docs.txt
python scripts/build_docs.py
python -m http.server 8000 --directory site
```

See [documentation maintenance](docs/developer/documentation.md) for OpenWiki regeneration, build instructions and GitHub Pages setup. Review changes through a pull request; the documentation workflow validates pull requests and deploys only from `main`.

---

## 🚀 Getting Started

### Dependency Management
This project uses **[uv](https://github.com/astral-sh/uv)** to manage its dependencies. It is an extremely fast Python package and project manager written in Rust. 

*For more information on working with uv, refer to their [official documentation](https://github.com/astral-sh/uv?tab=readme-ov-file#uv).*

### Configuration & Execution
Simulation parameters (such as simulation time, simulation target/pipeline, initial conditions, etc.) are driven by JSON configuration files.

1. Navigate to the `input/` directory.
2. Create or modify a `.json` file (e.g., `input/<filename>.json`) to specify the parameters you need.
3. Open `main.py` and update the input file variable to point to your target `.json` file.
4. Run the simulation.

### Outputs
Once the simulation completes, plot and video files will be automatically stored in the `output/` directory (e.g., such as `.png` images).

---

## 📚 How to Cite

If you use this software in your research, academic work, or projects, please cite it. All necessary metadata and formatting information can be found in the `CITATION.cff` file located in the root of this repository. 

*Tip: You can also easily generate citations in your preferred format by clicking the **"Cite this repository"** button in the "About" section on the right side of the GitHub repository page.*

---

## ⚖️ Acknowledgements & Disclaimer

*This software has been built during Nikos Kougiatsos's involvement in the Safe Autonomous Maritime Transport Research Group (Safenet) and Delft University of Technology.*

---

## 📝 License

This project is licensed under the **Apache License 2.0**. 
See the `LICENSE` and `NOTICE` files for full details.


