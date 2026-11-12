import matplotlib.pyplot as plt
import numpy as np

from visualization.csv_io import load_csv
from visualization.plotter import _finish, _row_flywheel_mass


def pareto_mask(masses, flip_times):
    """Minimize mass and time. Inputs must contain finite, valid results."""
    masses = np.asarray(masses)
    flip_times = np.asarray(flip_times)

    # Sort by mass first, then time for equal masses.
    # np.lexsort uses the LAST key as the primary key.
    order = np.lexsort((flip_times, masses))

    keep = np.zeros(len(masses), dtype=bool)
    best_time = np.inf
    best_mass = None

    for i in order:
        mass = masses[i]
        time = flip_times[i]

        improves_time = time < best_time
        identical_best = time == best_time and mass == best_mass

        if improves_time or identical_best:
            keep[i] = True
            best_time = time
            best_mass = mass

    return keep


def plot_pareto_from_csv(path, show=True):
    rows = load_csv(path)
    if not rows:
        print("CSV is empty.")
        return

    masses = np.array([_row_flywheel_mass(r) for r in rows]) * 1000
    flip_times = np.array([r.flip_time_s for r in rows])
    keep = pareto_mask(masses, flip_times)

    front_order = np.argsort(masses[keep])
    front_masses = masses[keep][front_order]
    front_times = flip_times[keep][front_order]

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.scatter(masses[~keep], flip_times[~keep], s=15, color="lightgray", label="Dominated runs")
    ax.step(front_masses, front_times, where="post", color="r", alpha=0.6)
    ax.scatter(front_masses, front_times, color="r", zorder=3, label="Pareto frontier")
    ax.set_title("Pareto frontier: flywheel mass vs flip time")
    _finish(ax, "Flywheel mass (g)", "Flip time (s)")

    fig.tight_layout()
    if show:
        plt.show()


