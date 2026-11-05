import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize


def plot_csv_dashboard(path, selected_row=None):
    with Path(path).open(newline="") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise ValueError("The CSV contains no results.")

    if selected_row is not None and not 0 <= selected_row < len(rows):
        raise ValueError(f"selected_row must be between 0 and {len(rows) - 1}.")

    def column(name):
        return np.array([
            float(row[name]) if row.get(name) else np.nan
            for row in rows
        ])

    rpm = column("target_rpm")
    finite_rpm = rpm[np.isfinite(rpm)]
    if not finite_rpm.size:
        raise ValueError("The CSV needs a numeric target_rpm column.")

    norm = Normalize(vmin=finite_rpm.min(), vmax=finite_rpm.max())
    cmap = "viridis"

    # CSV column names, axis labels, and panel titles.
    plots = [
        (
            "cube_total_mass", "flip_time_s",
            "Total cube mass (kg)", "Flip time (s)",
            "Cube mass vs flip time",
        ),
        (
            "target_rpm", "flip_time_s",
            "Target flywheel speed (RPM)", "Flip time (s)",
            "Flywheel speed vs flip time",
        ),
        (
            "flip_time_s", "final_angular_velocity_deg_s",
            "Flip time (s)", "Landing angular speed (°/s)",
            "Flip time vs landing speed",
        ),
        (
            "flip_time_s", "positive_work_j",
            "Flip time (s)", "Positive gimbal work (J)",
            "Flip time vs gimbal work",
        ),
        (
            "gimbal_travel_deg", "flip_time_s",
            "Gimbal travel (°)", "Flip time (s)",
            "Gimbal travel vs flip time",
        ),
        (
            "peak_gimbal_speed_deg_s", "flip_time_s",
            "Peak gimbal speed (°/s)", "Flip time (s)",
            "Gimbal speed vs flip time",
        ),
        (
            "initial_gamma_deg", "flip_time_s",
            "Initial gimbal angle (°)", "Flip time (s)",
            "Starting angle vs flip time",
        ),
        (
            "target_gamma_deg", "flip_time_s",
            "Target gimbal angle (°)", "Flip time (s)",
            "Target angle vs flip time",
        ),
    ]

    fig, grid = plt.subplots(
        3, 3,
        figsize=(17, 12),
        layout="constrained",
    )
    axes = grid.ravel()

    for ax, (x_name, y_name, x_label, y_label, title) in zip(
        axes, plots
    ):
        x, y = column(x_name), column(y_name)
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(rpm)

        if valid.any():
            ax.scatter(
                x[valid], y[valid],
                c=rpm[valid],
                cmap=cmap,
                norm=norm,
                s=12,
                alpha=0.35,
                edgecolors="none",
                rasterized=True,
            )
        else:
            ax.text(
                0.5, 0.5, "No data available",
                transform=ax.transAxes,
                ha="center", va="center",
            )

        if selected_row is not None:
            i = selected_row
            if np.isfinite(x[i]) and np.isfinite(y[i]):
                ax.scatter(
                    x[i], y[i],
                    marker="*",
                    s=260,
                    color="#ef4444",
                    edgecolors="black",
                    linewidths=0.8,
                    zorder=10,
                )
                ax.annotate(
                    "Selected run",
                    (x[i], y[i]),
                    xytext=(9, 9),
                    textcoords="offset points",
                    fontsize=8,
                    fontweight="bold",
                )

        ax.set(
            xlabel=x_label,
            ylabel=y_label,
            title=title,
        )

    # Distribution of the directly saved flip-time values.
    ax = axes[8]
    times = column("flip_time_s")
    ax.hist(
        times[np.isfinite(times)],
        bins=40,
        color="#64748b",
        edgecolor="white",
        linewidth=0.5,
    )

    if selected_row is not None and np.isfinite(times[selected_row]):
        ax.axvline(
            times[selected_row],
            color="#ef4444",
            linewidth=2,
            label="Selected run",
        )
        ax.legend(frameon=False)

    ax.set(
        xlabel="Flip time (s)",
        ylabel="Number of saved runs",
        title="Flip-time distribution",
    )

    for ax in axes:
        ax.set_axisbelow(True)
        ax.grid(alpha=0.15)
        ax.spines[["top", "right"]].set_visible(False)

    fig.colorbar(
        ScalarMappable(norm=norm, cmap=cmap),
        ax=axes[:8].tolist(),
        label="Target flywheel speed (RPM)",
        shrink=0.75,
        pad=0.02,
    )

    fig.suptitle(
        f"Cube configuration comparison · {len(rows):,} saved runs",
        fontsize=18,
        fontweight="bold",
    )

    plt.show()
    return fig


if __name__ == "__main__":
    plot_csv_dashboard(
        "/Users/tarasfliorko/Documents/GitHub/Gimbaled-Reaction-Cube/"
        "static/FULL_RANGE_TEST_2026-11-03_20-08-21.csv",
        selected_row=18786,  
    )