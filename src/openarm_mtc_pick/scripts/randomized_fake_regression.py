#!/usr/bin/env python3

"""Run reproducible randomized full-pipeline plan-only regressions.

Each scenario launches the same fake MoveIt + scene_manager + MTC data path as
fake_pick_place.launch.py.  The ROS domain defaults to 77 so this harness cannot
accidentally discover a real robot running on the normal domain 0.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
import os
from pathlib import Path
import random
import re
import shlex
import signal
import subprocess
import sys
from typing import Sequence


@dataclass(frozen=True)
class Scenario:
    box_x: float
    box_y: float
    box_z: float
    box_size_x: float
    box_size_y: float
    box_size_z: float
    box_yaw: float
    cube_x: float
    cube_y: float
    cube_z: float
    cube_size_x: float
    cube_size_y: float
    cube_size_z: float
    cube_yaw: float

    def launch_arguments(self) -> list[str]:
        return [f"{name}:={value:.9f}" for name, value in asdict(self).items()]


def generate_scenario(rng: random.Random) -> Scenario:
    """Sample the measured 08-21 operating envelope, not arbitrary workspace."""
    for _ in range(1000):
        box_x = rng.uniform(0.310, 0.338)
        box_y = rng.uniform(-0.052, -0.018)
        box_size_x = rng.uniform(0.225, 0.285)
        box_size_y = rng.uniform(0.150, 0.205)
        box_size_z = rng.uniform(0.215, 0.270)
        box_top = rng.uniform(0.232, 0.255)
        box_z = box_top - 0.5 * box_size_z
        box_yaw = rng.uniform(-math.pi, math.pi)

        cube_size_x = rng.uniform(0.027, 0.060)
        cube_size_y = rng.uniform(0.027, 0.060)
        cube_size_z = rng.uniform(0.025, 0.065)
        cube_yaw = rng.uniform(-math.pi, math.pi)

        # Project the rotated object OBB into the box-local axes, then sample
        # only footprints that retain at least 20 mm physical edge clearance.
        relative_yaw = cube_yaw - box_yaw
        projected_half_x = 0.5 * (
            abs(math.cos(relative_yaw)) * cube_size_x
            + abs(math.sin(relative_yaw)) * cube_size_y
        )
        projected_half_y = 0.5 * (
            abs(math.sin(relative_yaw)) * cube_size_x
            + abs(math.cos(relative_yaw)) * cube_size_y
        )
        allowed_x = 0.5 * box_size_x - projected_half_x - 0.020
        allowed_y = 0.5 * box_size_y - projected_half_y - 0.020
        if allowed_x <= 0.0 or allowed_y <= 0.0:
            continue
        local_x = rng.uniform(-0.70 * allowed_x, 0.70 * allowed_x)
        local_y = rng.uniform(-0.70 * allowed_y, 0.70 * allowed_y)
        cosine = math.cos(box_yaw)
        sine = math.sin(box_yaw)
        cube_x = box_x + cosine * local_x - sine * local_y
        cube_y = box_y + sine * local_x + cosine * local_y

        # Stay within the empirically exercised vertical-pick neighbourhood.
        if not (0.275 <= cube_x <= 0.355 and -0.095 <= cube_y <= 0.025):
            continue
        # Simulate independent RGB-D box/object Z estimates while remaining
        # within the runtime consistency gate.
        support_measurement_error = rng.uniform(-0.006, 0.006)
        cube_z = box_top + 0.5 * cube_size_z + support_measurement_error
        return Scenario(
            box_x, box_y, box_z, box_size_x, box_size_y, box_size_z,
            box_yaw, cube_x, cube_y, cube_z, cube_size_x, cube_size_y,
            cube_size_z, cube_yaw,
        )
    raise RuntimeError("could not sample a scene inside the configured envelope")


def command_for(scenario: Scenario) -> list[str]:
    return [
        "ros2", "launch", "openarm_mtc_pick", "fake_pick_place.launch.py",
        "run_mode:=plan_only", "execute_fake:=false",
        *scenario.launch_arguments(),
    ]


def run_command(command: Sequence[str], timeout: float, environment: dict[str, str]):
    process = subprocess.Popen(
        list(command), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, env=environment, start_new_session=True,
    )
    try:
        output, _ = process.communicate(timeout=timeout)
        return process.returncode, output, False
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate()
        return process.returncode, output, True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios", type=int, default=10)
    parser.add_argument("--seed", type=int, default=20260821)
    parser.add_argument("--only", type=int, help="run only this generated index")
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--domain-id", type=int, default=77)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--keep-success-logs", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.scenarios <= 0 or args.timeout <= 0.0:
        raise SystemExit("--scenarios and --timeout must be positive")
    if args.only is not None and not 0 <= args.only < args.scenarios:
        raise SystemExit("--only must be in [0, scenarios)")

    rng = random.Random(args.seed)
    scenarios = [generate_scenario(rng) for _ in range(args.scenarios)]
    selected = range(args.scenarios) if args.only is None else [args.only]
    if args.dry_run:
        for index in selected:
            print(json.dumps({"index": index, **asdict(scenarios[index])},
                             sort_keys=True))
        return 0

    if not shutil_which("ros2"):
        raise SystemExit("ros2 is not on PATH; source the ROS and workspace overlays")
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or Path.cwd() / (
        f"randomized_fake_{stamp}_seed{args.seed}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(args.domain_id)
    environment["ROS_LOCALHOST_ONLY"] = "1"
    environment["RCUTILS_COLORIZED_OUTPUT"] = "0"

    results = []
    for index in selected:
        scenario = scenarios[index]
        command = command_for(scenario)
        print(f"[{index + 1}/{args.scenarios}] seed={args.seed} planning...",
              flush=True)
        returncode, output, timed_out = run_command(
            command, args.timeout, environment
        )
        final_match = re.search(r"Final: solutions = (\d+)", output)
        final_solutions = int(final_match.group(1)) if final_match else 0
        failed_stage_match = re.search(r"FIRST FAILED STAGE: ([^\r\n]+)", output)
        first_failed_stage = (
            failed_stage_match.group(1).strip() if failed_stage_match else "not reported"
        )
        failure_messages = re.findall(
            r"Visual grasp task failed: ([^\r\n]+)", output
        )
        failure_reason = failure_messages[-1].strip() if failure_messages else ""
        success = (
            not timed_out and final_solutions > 0
            and "PLAN ONLY COMPLETE" in output
            and "FIRST FAILED STAGE: (none)" in output
        )
        result = {
            "index": index,
            "success": success,
            "returncode": returncode,
            "timed_out": timed_out,
            "final_solutions": final_solutions,
            "first_failed_stage": first_failed_stage,
            "failure_reason": failure_reason,
            "scenario": asdict(scenario),
            "command": command,
        }
        results.append(result)
        if not success or args.keep_success_logs:
            (output_dir / f"scenario_{index:03d}.log").write_text(
                output, encoding="utf-8"
            )
        status = "PASS" if success else "FAIL"
        print(f"  {status}: solutions={final_solutions}, "
              f"first_failed_stage={first_failed_stage}")
        if failure_reason:
            print(f"  reason: {failure_reason}")
        print("  reproduce with:")
        print(f"  ROS_DOMAIN_ID={args.domain_id} {shlex.join(command)}")

    summary = {
        "seed": args.seed,
        "domain_id": args.domain_id,
        "requested_scenarios": args.scenarios,
        "executed": len(results),
        "passed": sum(result["success"] for result in results),
        "failed": sum(not result["success"] for result in results),
        "results": results,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8"
    )
    print(f"Summary: {summary['passed']}/{summary['executed']} passed; "
          f"artifacts={output_dir}")
    return 0 if summary["failed"] == 0 else 1


def shutil_which(executable: str) -> str | None:
    # Kept local so the runtime dependency surface stays standard-library only.
    for directory in os.environ.get("PATH", "").split(os.pathsep):
        candidate = Path(directory) / executable
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


if __name__ == "__main__":
    sys.exit(main())
