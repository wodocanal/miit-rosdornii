import json
import math

from gnss_resilient_demo.headless import run_headless


def test_headless_run_exports_finite_metrics_and_event_log(tmp_path):
    result = run_headless(duration=3.0, output_dir=tmp_path, seed=516)

    with result.summary_json.open(encoding="utf-8") as stream:
        summary = json.load(stream)
    assert result.samples_csv.exists()
    assert result.events_json.exists()
    assert summary["metrics"]["samples"] > 100
    assert math.isfinite(summary["metrics"]["rmse"])
    assert summary["seed"] == 516
