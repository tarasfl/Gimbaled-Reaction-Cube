import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize


def plot_servo_dashboard(path, selected_row=110):
    with Path(path).open(newline="") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise ValueError("The CSV contains no results.")

    if selected_row is not None and not 0 <= selected_row < len(rows):
        raise ValueError(
            f"selected_row must be between 0 and {len(rows) - 1}."
        )

    def column(name):
        return np.array([
            float(row[name]) if row.get(name) else np.nan
            for row in rows
        ])

    plots = [
        (
            "torque", "max_rpm",
            "Servo rated torque (N·m)", "Servo maximum speed (RPM)",
            "Servo design space",
        ),
        (
            "torque", "flip_time_s",
            "Servo rated torque (N·m)", "Flip time (s)",
            "Rated torque vs flip time",
        ),
        (
            "max_rpm", "flip_time_s",
            "Servo maximum speed (RPM)", "Flip time (s)",
            "Rated speed vs flip time",
        ),
        (
            "peak_torque_nm", "peak_gimbal_speed_deg_s",
            "Peak applied torque (N·m)", "Peak gimbal speed (°/s)",
            "Actual torque and speed demand",
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
            "gimbal_travel_deg", "positive_work_j",
            "Gimbal travel (°)", "Positive gimbal work (J)",
            "Gimbal travel vs work",
        ),
    ]

    times = column("flip_time_s")
    finite_times = times[np.isfinite(times)]
    if not finite_times.size:
        raise ValueError("The CSV has no numeric flip times.")

    norm = Normalize(
        vmin=finite_times.min(),
        vmax=finite_times.max(),
    )
    cmap = "viridis_r"
    selected_color = "#ef4444"

    fig, grid = plt.subplots(
        3, 3,
        figsize=(17, 12),
        layout="constrained",
    )
    axes = grid.ravel()

    for ax, (x_name, y_name, xlabel, ylabel, title) in zip(
        axes, plots
    ):
        x, y = column(x_name), column(y_name)
        valid = (
            np.isfinite(x)
            & np.isfinite(y)
            & np.isfinite(times)
        )

        ax.scatter(
            x[valid], y[valid],
            c=times[valid],
            cmap=cmap,
            norm=norm,
            s=32,
            alpha=0.65,
            edgecolors="none",
            rasterized=True,
        )

        if selected_row is not None:
            i = selected_row
            if np.isfinite(x[i]) and np.isfinite(y[i]):
                ax.scatter(
                    x[i], y[i],
                    marker="*",
                    s=280,
                    color=selected_color,
                    edgecolors="black",
                    linewidths=0.8,
                    zorder=10,
                )
                ax.annotate(
                    "Selected servo",
                    (x[i], y[i]),
                    xytext=(9, 9),
                    textcoords="offset points",
                    fontsize=8,
                    fontweight="bold",
                )

        ax.set(xlabel=xlabel, ylabel=ylabel, title=title)

    # Distribution across saved successful runs.
    ax = axes[8]
    ax.hist(
        finite_times,
        bins=20,
        color="#64748b",
        edgecolor="white",
        linewidth=0.6,
    )

    if selected_row is not None and np.isfinite(times[selected_row]):
        ax.axvline(
            times[selected_row],
            color=selected_color,
            linewidth=2,
            label="Selected servo",
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
        label="Flip time (s)",
        shrink=0.75,
        pad=0.02,
    )

    title = f"Servo comparison · {len(rows):,} saved runs"

    if selected_row is not None:
        row = rows[selected_row]
        title += (
            f"\nSelected: {float(row['torque']):.3f} N·m"
            f" | {float(row['max_rpm']):.1f} RPM"
            f" | flip time {float(row['flip_time_s']):.3f} s"
        )

    fig.suptitle(title, fontsize=16, fontweight="bold")

    # Optional export:
    # fig.savefig("servo_dashboard.png", dpi=300)

    plt.show()
    return fig


if __name__ == "__main__":
    plot_servo_dashboard(
        "/Users/tarasfliorko/Documents/GitHub/Gimbaled-Reaction-Cube/"
        "static/SERVO_FULL_RANGE_TEST_2026-11-03_20-08-51.csv",
        selected_row=110,
    )