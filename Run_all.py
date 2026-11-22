"""
Batch-runner voor alle 16 runs (Run 0 t/m 15) van de gevoeligheidsanalyse.
Elke run draait in een geïsoleerd subproces zodat owlready2 en pygame per run schoon starten.

Gebruik:
    python run_all_simulations.py                         # Alle runs (0 t/m 15) met seed 42
    python run_all_simulations.py --runs 0 1 2 3          # Alleen specifieke runs
    python run_all_simulations.py --seed 100 --output output_seed100
    python run_all_simulations.py --headless              # Zonder zichtbaar Pygame-venster
"""
import argparse
import copy
import json
import os
import subprocess
import sys
import time

# Basisconfiguratie conform input/SimParam_WC.json
BASE_CONFIG = {
    "imports": [
        {
            "from": "sim_colav",
            "import": ["run_simulation"]
        }
    ],
    "Seed": 42,
    "Simulation Time": 2500,
    "Time step": 1,
    "Sensor noise": 0.0,
    "GPS noise": 0.0,
    "Heading noise": 0.0,
    "Wind speed": 0.0,
    "Wind direction": 0.0,
    "Wind speed noise": 0.0,
    "Wind direction noise": 0.0,
    "Current speed": 0.0,
    "Current direction": 0.0,
    "Current noise sigma": 0.0,
    "Wave height": 0.0,
    "Wave direction": 0.0,
    "Wave height noise sigma": 0.0,
    "Wave direction noise sigma": 0.0,
    "scale": 0.2,
    "Waypoint spacing": 250,
    "Lookahead distance": 200
}

# Alle 16 runs uit Tabel 5.1 van het verslag
RUN_MATRIX = {
    0:  {"label": "Baseline (Clean Run)",        "overrides": {}},

    # GPS Noise (sigma_GPS)
    1:  {"label": "GPS Noise (0.1 m)",           "overrides": {"GPS noise": 0.1}},
    2:  {"label": "GPS Noise (5.0 m)",           "overrides": {"GPS noise": 5.0}},
    3:  {"label": "GPS Noise (10.0 m)",          "overrides": {"GPS noise": 10.0}},

    # Heading Noise (sigma_psi)
    4:  {"label": "Heading Noise (0.1 deg)",     "overrides": {"Heading noise": 0.1}},
    5:  {"label": "Heading Noise (1.0 deg)",     "overrides": {"Heading noise": 1.0}},
    6:  {"label": "Heading Noise (3.0 deg)",     "overrides": {"Heading noise": 3.0}},

    # Wind at 90 deg
    7:  {"label": "Wind (5.0 m/s @ 90 deg)",     "overrides": {"Wind speed": 5.0,  "Wind direction": 90.0, "Wind speed noise": 1.0, "Wind direction noise": 3.0}},
    8:  {"label": "Wind (10.0 m/s @ 90 deg)",    "overrides": {"Wind speed": 10.0, "Wind direction": 90.0, "Wind speed noise": 1.0, "Wind direction noise": 5.0}},
    9:  {"label": "Wind (15.0 m/s @ 90 deg)",    "overrides": {"Wind speed": 15.0, "Wind direction": 90.0, "Wind speed noise": 1.0, "Wind direction noise": 5.0}},

    # Current at 90 deg (sea) + parabolic river profile (inland)
    10: {"label": "Current (0.5 m/s @ 90 deg)",  "overrides": {"Current speed": 0.5, "Current direction": 90.0, "Current noise sigma": 0.1}},
    11: {"label": "Current (1.0 m/s @ 90 deg)",  "overrides": {"Current speed": 1.0, "Current direction": 90.0, "Current noise sigma": 0.1}},
    12: {"label": "Current (1.5 m/s @ 90 deg)",  "overrides": {"Current speed": 1.5, "Current direction": 90.0, "Current noise sigma": 0.1}},

    # Waves at 90 deg (sea only; wave_factor = 0 in inland canal)
    13: {"label": "Waves (Hs=0.5 m @ 90 deg)",   "overrides": {"Wave height": 0.5, "Wave direction": 90.0, "Wave height noise sigma": 0.3, "Wave direction noise sigma": 0.0}},
    14: {"label": "Waves (Hs=1.0 m @ 90 deg)",   "overrides": {"Wave height": 1.0, "Wave direction": 90.0, "Wave height noise sigma": 0.5, "Wave direction noise sigma": 0.0}},
    15: {"label": "Waves (Hs=2.0 m @ 90 deg)",   "overrides": {"Wave height": 2.0, "Wave direction": 90.0, "Wave height noise sigma": 0.5, "Wave direction noise sigma": 0.0}},
}


def run_single_worker(run_id, seed, output_dir, headless=False):
    """Voert precies één simulatierun uit in dit geïsoleerde proces."""
    current_script_dir = os.path.dirname(os.path.abspath(__file__))
    path_to_src = os.path.join(current_script_dir, "src")

    if path_to_src not in sys.path:
        sys.path.insert(0, path_to_src)
        sys.path.insert(0, os.path.join(path_to_src, "modules"))
        sys.path.insert(0, os.path.join(path_to_src, "pipelines"))

    if headless:
        os.environ["SDL_VIDEODRIVER"] = "dummy"

    import pygame
    from owlready2 import get_ontology
    from src.modules.helper_utils import load_sim

    run_info = RUN_MATRIX[run_id]
    cfg = copy.deepcopy(BASE_CONFIG)
    cfg["Seed"] = seed
    cfg.update(run_info["overrides"])

    run_output_dir = os.path.join(output_dir, f"run_{run_id}")
    os.makedirs(run_output_dir, exist_ok=True)

    # Sla de exacte configuratie voor deze run op en gebruik deze direct als input
    run_config_path = os.path.join(run_output_dir, "SimParam_used.json")
    with open(run_config_path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=4)

    libs = load_sim(run_config_path)
    run_simulation = libs["run_simulation"]

    pygame.init()
    screen = pygame.display.set_mode((1200, 800))
    try:
        icon = pygame.image.load(os.path.join(current_script_dir, "assets/safenet.png"))
        pygame.display.set_icon(icon)
    except (FileNotFoundError, pygame.error):
        pass

    font = pygame.font.SysFont("Consolas", 25)
    ontology = get_ontology(os.path.join(current_script_dir, "assets/database.xml")).load()

    print(f"\n=== Start Run {run_id}: {run_info['label']} (Seed={seed}) ===")
    run_simulation(screen, ontology, run_config_path, font, current_script_dir, run_output_dir)
    pygame.quit()


def run_batch(run_ids, seed, output_dir, headless=False):
    """Start alle gekozen runs achter elkaar, elk in een schoon subproces."""
    results = []
    t_total_start = time.time()

    print(f"Batch-simulatie gestart | Runs: {run_ids} | Seed: {seed} | Output: {output_dir}/")

    for run_id in run_ids:
        if run_id not in RUN_MATRIX:
            print(f"[WAARSCHUWING] Run {run_id} bestaat niet in RUN_MATRIX, wordt overgeslagen.")
            continue

        label = RUN_MATRIX[run_id]["label"]
        t_start = time.time()

        cmd = [
            sys.executable,
            os.path.abspath(__file__),
            "--worker", str(run_id),
            "--seed", str(seed),
            "--output", output_dir,
        ]
        if headless:
            cmd.append("--headless")

        try:
            proc = subprocess.run(cmd)
            elapsed = time.time() - t_start
            status = "OK" if proc.returncode == 0 else f"FOUT (code {proc.returncode})"
            results.append((run_id, label, status, elapsed))
            print(f"--- Run {run_id:2d} klaar [{status}] in {elapsed:.1f} s ---")
        except KeyboardInterrupt:
            print("\nBatch onderbroken door gebruiker.")
            break

    total_elapsed = time.time() - t_total_start
    print("\n================ BATCH OVERZICHT ================")
    for run_id, label, status, elapsed in results:
        print(f"Run {run_id:2d} | {label:28s} | {status:12s} | {elapsed:6.1f} s")
    print(f"Totale tijd: {total_elapsed / 60:.1f} min")
    print("=================================================")


def main():
    parser = argparse.ArgumentParser(description="Automatische batch-runner voor alle 16 simulatieruns.")
    parser.add_argument("--runs", nargs="+", type=int, default=list(range(16)),
                        help="Lijst met run-IDs om te draaien (standaard: 0 t/m 15).")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed voor alle runs (standaard: 42).")
    parser.add_argument("--output", type=str, default="output",
                        help="Hoofdmap voor output (standaard: 'output').")
    parser.add_argument("--headless", action="store_true",
                        help="Draai zonder zichtbaar Pygame-venster (voorkomt pop-ups).")
    parser.add_argument("--worker", type=int, default=None,
                        help=argparse.SUPPRESS)  # Alleen voor interne subproces-aanroep
    args = parser.parse_args()

    if args.worker is not None:
        run_single_worker(args.worker, args.seed, args.output, headless=args.headless)
    else:
        run_batch(args.runs, args.seed, args.output, headless=args.headless)


if __name__ == "__main__":
    main()