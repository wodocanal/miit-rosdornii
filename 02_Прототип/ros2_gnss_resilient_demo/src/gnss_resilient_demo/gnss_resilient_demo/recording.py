"""CSV/JSON experiment export."""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from .engine import ExperimentEngine


@dataclass(frozen=True)
class ExportResult:
    directory: Path
    samples_csv: Path
    summary_json: Path
    events_json: Path


def export_run(engine: ExperimentEngine, directory: str | Path) -> ExportResult:
    target = Path(directory)
    target.mkdir(parents=True, exist_ok=True)
    records = engine.records
    samples_path = target / "samples.csv"
    if records:
        with samples_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(records[0].keys()))
            writer.writeheader()
            writer.writerows(records)
    else:
        samples_path.write_text("sim_time\n", encoding="utf-8")
    frame = engine.frame()
    summary = {
        "created_at": datetime.now().astimezone().isoformat(),
        "seed": engine.seed,
        "sim_time": frame.sim_time,
        "truth_available": engine.truth_available,
        "navigation_mode": engine.navigation_mode.value,
        "metrics": asdict(frame.dispatcher_metrics) if frame.dispatcher_metrics else None,
        "active_effects": list(frame.active_effects),
        "limitations": [
            "Результаты симуляции не доказывают точность реального оборудования.",
            "Плавный спуфинг может пройти проверку без независимой привязки.",
            "Камера моделируется геометрически; распознавание изображения не моделируется.",
        ],
    }
    summary_path = target / "summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    events_path = target / "events.json"
    events_path.write_text(
        json.dumps([asdict(event) for event in frame.events], ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return ExportResult(target, samples_path, summary_path, events_path)
