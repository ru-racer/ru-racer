#!/usr/bin/env python3
"""
Run scenario validation suite.

Usage:
  # Single scenario
  python run_scenario_suite.py --scenario scenarios/straight_dry.yaml --vehicle-cfg ../data/fullsize_car.cfg

  # Full suite (all YAML in scenarios/)
  python run_scenario_suite.py --suite scenarios/ --vehicle-cfg ../data/fullsize_car.cfg
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from framework import ScenarioDef, ScenarioRunner
from framework.report import write_summary_csv, print_summary


def main() -> int:
    ap = argparse.ArgumentParser(description="Scenario validation suite runner")
    ap.add_argument("--scenario", help="Path to single scenario YAML")
    ap.add_argument("--suite", help="Directory with scenario YAML files (run all)")
    ap.add_argument("--vehicle-cfg", required=True, help="Vehicle/planner config (.cfg)")
    ap.add_argument("--bin", default="cpp/build/ru_racer", help="ru_racer binary path")
    ap.add_argument("--output", default="cpp/results/scenario_validation", help="Output root")
    args = ap.parse_args()

    if not args.scenario and not args.suite:
        ap.error("Provide --scenario or --suite")
    if args.scenario and args.suite:
        ap.error("Provide only one of --scenario or --suite")

    # Resolve paths relative to repo root (parent of cpp/)
    repo_root = os.path.dirname(os.path.dirname(THIS_DIR))
    bin_path = os.path.join(repo_root, args.bin) if not os.path.isabs(args.bin) else args.bin
    vehicle_cfg = os.path.join(repo_root, args.vehicle_cfg) if not os.path.isabs(args.vehicle_cfg) else args.vehicle_cfg
    output_root = os.path.join(repo_root, args.output) if not os.path.isabs(args.output) else args.output

    scenarios: list[ScenarioDef] = []
    if args.scenario:
        path = os.path.join(repo_root, args.scenario) if not os.path.isabs(args.scenario) else args.scenario
        if not os.path.exists(path):
            print(f"ERROR: scenario not found: {path}")
            return 2
        scenarios.append(ScenarioDef.from_yaml(path))
    else:
        suite_dir = os.path.join(repo_root, args.suite) if not os.path.isabs(args.suite) else args.suite
        yamls = sorted(glob.glob(os.path.join(suite_dir, "*.yaml")))
        if not yamls:
            print(f"ERROR: no .yaml scenarios in {suite_dir}")
            return 2
        for p in yamls:
            scenarios.append(ScenarioDef.from_yaml(p))

    runner = ScenarioRunner(
        bin_path=bin_path,
        vehicle_cfg_path=vehicle_cfg,
        output_root=output_root,
    )

    all_results: list = []
    for sc in scenarios:
        try:
            results = runner.run_scenario(sc)
            all_results.extend(results)
        except Exception as e:
            print(f"ERROR running {sc.id}: {e}")
            raise

    if not all_results:
        print("No results to report")
        return 1

    # Write summary to output root
    os.makedirs(output_root, exist_ok=True)
    summary_csv = os.path.join(output_root, "scenario_suite_summary.csv")
    write_summary_csv(all_results, summary_csv)
    print_summary(all_results)

    passed = sum(1 for r in all_results if r.passed)
    return 0 if passed == len(all_results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
