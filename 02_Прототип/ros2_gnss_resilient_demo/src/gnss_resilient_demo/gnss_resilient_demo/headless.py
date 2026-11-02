"""Fast deterministic acceptance runner without Qt or ROS executors."""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

from .engine import ExperimentEngine
from .recording import ExportResult, export_run


def run_headless(
    duration: float,
    output_dir: str | Path,
    seed: int = 516,
) -> ExportResult:
    if duration <= 0.0:
        raise ValueError("duration must be positive")
    engine = ExperimentEngine(seed=seed)
    for _ in range(round(duration / engine.DT)):
        engine.step()
    return export_run(engine, output_dir)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Детерминированный прогон демонстратора без GUI")
    parser.add_argument("--duration", type=float, default=130.0, help="длительность симуляции, с")
    parser.add_argument("--seed", type=int, default=516)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    target = args.output or Path("run_logs") / datetime.now().strftime("%Y%m%d_%H%M%S")
    result = run_headless(args.duration, target, args.seed)
    summary = json.loads(result.summary_json.read_text(encoding="utf-8"))
    metrics = summary.get("metrics") or {}
    print(f"Прогон завершён: t={summary['sim_time']:.1f} с, seed={summary['seed']}")
    if metrics:
        print(
            f"Ошибка: сейчас {metrics['current_error']:.2f} м; "
            f"RMSE {metrics['rmse']:.2f} м; 95% {metrics['p95']:.2f} м; max {metrics['maximum']:.2f} м"
        )
    print(f"Отчёт: {result.directory.resolve()}")


if __name__ == "__main__":
    main()
