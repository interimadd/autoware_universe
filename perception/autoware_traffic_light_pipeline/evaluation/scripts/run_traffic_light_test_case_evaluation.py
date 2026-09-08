#!/usr/bin/env python3

# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run the whole traffic light pipeline evaluation for one or many test case directories.

A "test case directory" is what the component test harness hands out: one t4dataset (a
UUID-named directory holding a "0" version directory with input_bag/map/annotation) plus one DLR
scenario yaml, both sitting directly inside it. For a single test case, this wraps the two manual
steps documented in ../README.md and scripts/README.md into one command:

    run_traffic_light_pipeline_evaluation --config ... --dataset <t4dataset>/0 --output-bag ...
    evaluate_traffic_light_recognition.py --scenario ... --dataset <t4dataset>/0 \
        --result-bag ... --output-dir ...

Usage:
    ros2 run autoware_traffic_light_pipeline run_traffic_light_test_case_evaluation.py \
        <dir> [--config <evaluation yaml>] [--output-root result] [--force]

<dir> is the only required argument. Two modes, auto-detected from <dir> itself:

- Single test case: <dir> directly holds one t4dataset + one scenario yaml, e.g.
  ".../Gen2_TLR_Shiojiri/TLR_UC_001530_shiojiri_gen2_sunny_01". Runs the pipeline once.
- Batch: <dir> does not directly qualify as a test case, so it is walked recursively for
  descendant directories that do (e.g. a "Gen2_TLR_Shiojiri" directory holding many
  "TLR_UC_*" test cases). Each match is run in turn; a failure or error in one does not stop
  the rest. --config/--scenario/--dataset cannot be used in this mode (they would be ambiguous
  across test cases) -- use single mode for that.

In both modes, the scenario yaml and the t4dataset are auto-discovered inside each test case
directory, the pipeline config yaml is auto-selected from this package's evaluation/config based
on the scenario's SensorModel, and results are written under
<output-root>/<test_case_dir name>/<t4dataset id>/ so the t4dataset id that produced them is
always visible in the path (see info.json in that directory for the rest of the provenance:
scenario/config paths, sensor/vehicle model, timestamp).
"""

from __future__ import annotations

import argparse
import concurrent.futures
import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import yaml


class UsageError(Exception):
    """A problem with a test case directory or arguments; report it, don't traceback."""


def is_t4dataset_dir(d: Path) -> bool:
    return d.is_dir() and not d.name.startswith(".") and (d / "0" / "input_bag").exists()


def is_test_case_dir(d: Path) -> bool:
    """True if `d` directly holds exactly one scenario yaml and exactly one t4dataset."""
    yamls = list(d.glob("*.yaml")) + list(d.glob("*.yml"))
    if len(yamls) != 1:
        return False
    return sum(1 for sub in d.iterdir() if is_t4dataset_dir(sub)) == 1


def find_test_case_dirs(root: Path) -> list[Path]:
    """Recursively find descendants of `root` that qualify as test case directories.

    Does not descend into a match (a t4dataset's own subdirectories -- map/, annotation/, ...
    -- never contain further test cases in the layouts this tool has seen).
    """
    found = []
    for dirpath, dirnames, _filenames in os.walk(root):
        d = Path(dirpath)
        if is_test_case_dir(d):
            found.append(d)
            dirnames[:] = []
    return sorted(found)


def find_scenario_yaml(test_case_dir: Path, override: Path | None) -> Path:
    if override is not None:
        return override.expanduser().resolve()
    candidates = sorted(test_case_dir.glob("*.yaml")) + sorted(test_case_dir.glob("*.yml"))
    if len(candidates) != 1:
        error_msg = (
            f"Expected exactly one scenario yaml directly inside {test_case_dir}, "
            f"found {len(candidates)}: {candidates}. Pass --scenario to disambiguate."
        )
        raise UsageError(error_msg)
    return candidates[0]


def find_t4dataset(test_case_dir: Path, override: Path | None) -> tuple[str, Path]:
    """Return (dataset_id, dataset_path), where dataset_path is "<id-dir>/0"."""
    if override is not None:
        dataset_path = override.expanduser().resolve()
        return dataset_path.parent.name, dataset_path
    candidates = [
        d / "0" for d in sorted(test_case_dir.iterdir()) if is_t4dataset_dir(d)
    ]
    if len(candidates) != 1:
        error_msg = (
            f"Expected exactly one t4dataset (<uuid>/0/input_bag) directly inside "
            f"{test_case_dir}, found {len(candidates)}: {candidates}. Pass --dataset to "
            "disambiguate."
        )
        raise UsageError(error_msg)
    dataset_path = candidates[0]
    return dataset_path.parent.name, dataset_path


def find_config_dir() -> Path:
    """Locate evaluation/config, whether this script is running installed or from source.

    Installed, this script sits under .../lib/autoware_traffic_light_pipeline/ (ament_cmake's
    flat script layout, see CMakeLists.txt), while evaluation/config is installed under
    .../share/autoware_traffic_light_pipeline/evaluation/config -- unrelated directory trees, so
    a relative "../config" from __file__ only works when run straight from the source tree.
    Prefer ament_index (works for both `ros2 run` and a plain `python3 <installed path>`), and
    fall back to the source-tree-relative path so this also works un-built.
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = Path(get_package_share_directory("autoware_traffic_light_pipeline"))
        return share_dir / "evaluation" / "config"
    except Exception:  # noqa: BLE001 - ament_index unavailable or package not found
        return Path(__file__).resolve().parent.parent / "config"


def find_pipeline_config(scenario: dict, override: Path | None) -> Path:
    """Auto-select evaluation/config/*.evaluation.yaml from the scenario's SensorModel.

    SensorModel is e.g. "aip_x2_gen2"; config files are named "<token>_v<ver>.evaluation.yaml"
    (e.g. "x2_v4.4.evaluation.yaml"). Match by the token sandwiched as "_<token>_" or a leading
    "<token>_" in the SensorModel, since that's the only naming convention currently in use.
    """
    if override is not None:
        return override.expanduser().resolve()

    config_dir = find_config_dir()
    all_configs = sorted(config_dir.glob("*.evaluation.yaml"))
    if not all_configs:
        error_msg = f"No *.evaluation.yaml found under {config_dir}. Pass --config explicitly."
        raise UsageError(error_msg)
    if len(all_configs) == 1:
        return all_configs[0]

    sensor_model = scenario.get("SensorModel", "")
    matches = [c for c in all_configs if c.name.split("_v")[0] in sensor_model.split("_")]
    if len(matches) == 1:
        return matches[0]
    error_msg = (
        f"Could not uniquely pick a config for SensorModel={sensor_model!r} among "
        f"{[c.name for c in all_configs]}. Pass --config explicitly."
    )
    raise UsageError(error_msg)


def run(cmd: list[str], **kwargs) -> None:  # noqa: ANN003
    """Run `cmd`, raising CalledProcessError on any non-zero exit."""
    print(f"+ {' '.join(str(c) for c in cmd)}", file=sys.stderr)  # noqa: T201
    subprocess.run(cmd, check=True, **kwargs)  # noqa: S603


def run_eval_script(cmd: list[str], **kwargs) -> bool:  # noqa: ANN003
    """Run evaluate_traffic_light_recognition.py, whose exit code is a PASS/FAIL verdict.

    Returns True (PASS) / False (FAIL) for exit codes 0/1; anything else is a real crash.
    """
    print(f"+ {' '.join(str(c) for c in cmd)}", file=sys.stderr)  # noqa: T201
    returncode = subprocess.run(cmd, **kwargs).returncode  # noqa: S603
    if returncode not in (0, 1):
        error_msg = f"{cmd[0]} exited with unexpected code {returncode} (not a PASS/FAIL verdict)"
        raise UsageError(error_msg)
    return returncode == 0


def run_test_case(
    test_case_dir: Path, args: argparse.Namespace, extra_env: dict[str, str] | None = None
) -> bool:
    """Run both evaluation steps for one test case directory. Returns the PASS/FAIL verdict.

    `extra_env` is merged on top of the current environment for both subprocess steps -- used in
    parallel batch mode to give each concurrently-running test case its own ROS_DOMAIN_ID, so
    their DDS traffic (topics/services from the pipeline node and bag playback) doesn't cross
    talk between test cases running at the same time.
    """
    scenario_path = find_scenario_yaml(test_case_dir, args.scenario)
    with scenario_path.open() as f:
        scenario = yaml.safe_load(f)

    dataset_id, dataset_path = find_t4dataset(test_case_dir, args.dataset)
    config_path = find_pipeline_config(scenario, args.config)

    out_dir = args.output_root.expanduser().resolve() / test_case_dir.name / dataset_id
    if out_dir.exists():
        if not args.force:
            error_msg = f"{out_dir} already exists. Re-run with --force to overwrite it."
            raise UsageError(error_msg)
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)

    output_bag = out_dir / "output_bag"
    eval_dir = out_dir / "evaluation"

    print(  # noqa: T201
        f"test case:    {test_case_dir.name}\n"
        f"t4dataset id: {dataset_id}\n"
        f"scenario:     {scenario_path}\n"
        f"config:       {config_path}\n"
        f"output:       {out_dir}\n",
        file=sys.stderr,
    )

    (out_dir / "info.json").write_text(
        json.dumps(
            {
                "test_case_dir": str(test_case_dir),
                "t4dataset_id": dataset_id,
                "dataset_path": str(dataset_path),
                "scenario_path": str(scenario_path),
                "scenario_name": scenario.get("ScenarioName"),
                "sensor_model": scenario.get("SensorModel"),
                "vehicle_model": scenario.get("VehicleModel"),
                "config_path": str(config_path),
                "generated_at": datetime.datetime.now().astimezone().isoformat(),
            },
            indent=2,
        )
        + "\n"
    )

    base_env = dict(os.environ, **(extra_env or {}))

    run(
        [
            "ros2",
            "run",
            "autoware_traffic_light_pipeline",
            "run_traffic_light_pipeline_evaluation",
            "--config",
            str(config_path),
            "--dataset",
            str(dataset_path),
            "--output-bag",
            str(output_bag),
        ],
        env=base_env,
    )

    # cv2's Qt plugin errors out under evaluate_traffic_light_recognition.py without a display.
    env = dict(base_env, QT_QPA_PLATFORM="offscreen")
    return run_eval_script(
        [
            "ros2",
            "run",
            "autoware_traffic_light_pipeline",
            "evaluate_traffic_light_recognition.py",
            "--scenario",
            str(scenario_path),
            "--dataset",
            str(dataset_path),
            "--result-bag",
            str(output_bag),
            "--output-dir",
            str(eval_dir),
        ],
        env=env,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "dir",
        type=Path,
        help="A single test case directory, or a directory tree containing many",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Pipeline evaluation yaml (auto-picked if omitted; single-test-case mode only)",
    )
    parser.add_argument(
        "--scenario",
        type=Path,
        default=None,
        help="DLR scenario yaml (auto-found if omitted; single-test-case mode only)",
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=None,
        help="t4dataset's version dir, e.g. <id>/0 (auto-found if omitted; single-test-case "
        "mode only)",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("result"),
        help="Results go under <output-root>/<test_case_dir name>/<t4dataset id>/ (default: result)",
    )
    parser.add_argument(
        "--force", action="store_true", help="Delete pre-existing output directories before running"
    )
    parser.add_argument(
        "--jobs",
        "-j",
        type=int,
        default=1,
        help="Run this many test cases concurrently in batch mode (default: 1, sequential). "
        "Each concurrent run gets its own ROS_DOMAIN_ID so their DDS traffic doesn't cross "
        "talk. Ignored in single-test-case mode. Output from concurrent runs interleaves in "
        "the terminal.",
    )
    return parser.parse_args()


def print_batch_summary(results: list[tuple[Path, str, str | None]]) -> None:
    print("\n=== batch summary ===")  # noqa: T201
    width = max(len(tc.name) for tc, _, _ in results)
    for test_case_dir, verdict, detail in results:
        line = f"{test_case_dir.name:<{width}}  {verdict}"
        if detail:
            line += f"  ({detail})"
        print(line)  # noqa: T201


def main() -> int:
    args = parse_args()
    root = args.dir.expanduser().resolve()
    if not root.is_dir():
        raise UsageError(f"Not a directory: {root}")

    if is_test_case_dir(root):
        return 0 if run_test_case(root, args) else 1

    if args.config or args.scenario or args.dataset:
        error_msg = (
            f"{root} is not itself a test case directory, so batch mode will run over its "
            "descendants -- --config/--scenario/--dataset are not allowed there (they would "
            "apply ambiguously to every test case found). Point at a single test case "
            "directory to use them."
        )
        raise UsageError(error_msg)

    test_case_dirs = find_test_case_dirs(root)
    if not test_case_dirs:
        error_msg = f"No test case directory (one scenario yaml + one t4dataset) found under {root}"
        raise UsageError(error_msg)

    jobs = max(1, args.jobs)
    print(  # noqa: T201
        f"found {len(test_case_dirs)} test case(s) under {root} (jobs={jobs}):", file=sys.stderr
    )
    for tc in test_case_dirs:
        print(f"  {tc}", file=sys.stderr)  # noqa: T201

    def run_one(index: int, test_case_dir: Path) -> tuple[Path, str, str | None]:
        # Distinct ROS_DOMAIN_ID per concurrently-running slot so simultaneous pipeline/bag
        # playback processes don't see each other's topics. Harmless (and unused) when jobs==1.
        extra_env = {"ROS_DOMAIN_ID": str(100 + index % jobs)}
        try:
            passed = run_test_case(test_case_dir, args, extra_env)
        except UsageError as e:
            return (test_case_dir, "ERROR", str(e))
        except subprocess.CalledProcessError as e:
            return (test_case_dir, "ERROR", f"command failed (exit {e.returncode})")
        else:
            return (test_case_dir, "PASS" if passed else "FAIL", None)

    results: list[tuple[Path, str, str | None] | None] = [None] * len(test_case_dirs)
    if jobs == 1:
        for i, test_case_dir in enumerate(test_case_dirs):
            results[i] = run_one(i, test_case_dir)
    else:
        with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as executor:
            future_to_index = {
                executor.submit(run_one, i, tc): i for i, tc in enumerate(test_case_dirs)
            }
            for future in concurrent.futures.as_completed(future_to_index):
                results[future_to_index[future]] = future.result()

    print_batch_summary(results)
    return 0 if all(verdict == "PASS" for _, verdict, _ in results) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except UsageError as e:
        print(f"error: {e}", file=sys.stderr)  # noqa: T201
        sys.exit(2)
    except subprocess.CalledProcessError as e:
        sys.exit(e.returncode)
