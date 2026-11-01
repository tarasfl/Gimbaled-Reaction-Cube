import csv
from datetime import datetime
from types import SimpleNamespace

def save_csv(results, specs=None, metadata=None):
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    path = f"{specs}_{timestamp}.csv" if specs else f"sweep_results_{timestamp}.csv"
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
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(vars(rows[0]))
        writer.writerows(vars(row).values() for row in rows)
    print(f"Saved {len(rows)} runs to {path}")


def load_csv(path="sweep_results.csv"):
    with open(path, newline="") as f:
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
