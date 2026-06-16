# GNC Simulator

[![Python 3.12](https://img.shields.io/badge/python-3.12-blue.svg)](https://www.python.org/downloads/release/python-3120/)
[![image](https://img.shields.io/pypi/v/uv.svg)](https://pypi.python.org/pypi/uv)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![uv](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/uv/main/assets/badge/v0.json)](https://github.com/astral-sh/uv)

> [Decription pending]

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

*This software has been built during my involvement in the Safe Autonomous Maritime Transport Research Group (Safenet) and Delft University of Technology.*

---

## 📝 License

This project is licensed under the **Apache License 2.0**. 
See the `LICENSE` and `NOTICE` files for full details.


