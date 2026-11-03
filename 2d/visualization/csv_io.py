import csv
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

STATIC_DIR = Path(__file__).resolve().parents[2] / "static"


def save_csv(results, specs=None, metadata=None):
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"{specs}_{timestamp}.csv" if specs else f"sweep_results_{timestamp}.csv"
    metadata = metadata or SimpleNamespace()
    rows = [
        SimpleNamespace(
            **vars(metadata),
            **{k: v for k, v in vars(r).items() if k != "result"},
        )
        for r in results
    ]
    if not rows:
        print("Nothing to save.")
        return
    STATIC_DIR.mkdir(parents=True, exist_ok=True)
    path = STATIC_DIR / filename
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(vars(rows[0]))
        writer.writerows(vars(row).values() for row in rows)
    print(f"Saved {len(rows)} runs to {path}")


def load_csv(path="sweep_results.csv"):
    csv_path = Path(path)
    if not csv_path.is_absolute():
        if csv_path.parts and csv_path.parts[0] == "static":
            csv_path = Path(__file__).resolve().parents[2] / csv_path
        else:
            csv_path = STATIC_DIR / csv_path
    with open(csv_path, newline="") as f:
        return [
            SimpleNamespace(**{k: _parse(v) for k, v in row.items()})
            for row in csv.DictReader(f)
        ]


def _parse(value):
    try:
        return float(value)
    except ValueError:
        return value


def fastest_row(rows):
    return min(rows, key=lambda r: r.flip_time_s)
