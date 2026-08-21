#!/usr/bin/env python3
"""Offline statistics evaluator for the OpenArm RGB-D benchmark bags.

Statistics only: this script does NOT re-implement any detector algorithm. It
reads the topics that the current detectors already published into the bag
(/box/pose, /box/dimensions, /box/top_height, /box/footprint, /blue_cube/pose,
/blue_cube/dimensions) and computes baseline metrics in mm / deg / %:

  - bag_01_static:    frame counts, box coverage, lock availability, lock
                      losses, and LOCK-side position/size/yaw/top_z/support_z
                      standard deviations.
  - bag_02_box_move:  box stop phases, per-phase measurements, re-lock time
                      and cross-phase size stability.
  - bag_03_object_move: object detection rate, pose stability and
                      re-acquisition time.

The same generic analysis is applied to every bag; nothing here special-cases
a particular bag, timestamp, pixel or world coordinate.

Usage (host offline analysis, conda env openarm_vision):

    conda run -n openarm_vision python scripts/evaluate_vision_bags.py
    conda run -n openarm_vision python scripts/evaluate_vision_bags.py --bags /path/to/bag_dir
"""

from __future__ import annotations

import argparse
import glob
import math
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

NANOSECONDS = 1_000_000_000

RGB_TOPIC = "/camera/global_camera/color/image_raw"
DEPTH_TOPIC = "/camera/global_camera/aligned_depth_to_color/image_raw"
BOX_POSE_TOPIC = "/box/pose"
BOX_DIMS_TOPIC = "/box/dimensions"
BOX_TOP_TOPIC = "/box/top_height"
OBJ_POSE_TOPIC = "/blue_cube/pose"
OBJ_DIMS_TOPIC = "/blue_cube/dimensions"


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * NANOSECONDS + int(
        message.header.stamp.nanosec
    )


def axis_yaw_error(angles: np.ndarray, reference: float) -> np.ndarray:
    """Signed axis error for yaw values that are equivalent modulo pi."""
    return 0.5 * np.arctan2(
        np.sin(2.0 * (angles - reference)),
        np.cos(2.0 * (angles - reference)),
    )


def axis_yaw_mean(angles: np.ndarray) -> float:
    """Circular mean for an axis yaw (modulo pi)."""
    if angles.size == 0:
        return float("nan")
    return float(
        0.5
        * math.atan2(
            float(np.mean(np.sin(2.0 * angles))),
            float(np.mean(np.cos(2.0 * angles))),
        )
    )


def nearest_coverage(stream_times: np.ndarray, frame_times: np.ndarray,
                     window_ns: int) -> float:
    """Fraction of frame times that have a stream message within window_ns."""
    if stream_times.size == 0 or frame_times.size == 0:
        return 0.0
    positions = np.searchsorted(stream_times, frame_times)
    distances = np.full(frame_times.size, np.inf)
    for offset in (-1, 0):
        indices = positions + offset
        valid = (indices >= 0) & (indices < stream_times.size)
        clipped = np.clip(indices, 0, stream_times.size - 1)
        delta = np.abs(frame_times - stream_times[clipped])
        distances = np.minimum(distances, np.where(valid, delta, np.inf))
    return float(np.mean(distances <= window_ns))


def stream_gaps(times: np.ndarray, loss_threshold_ns: int) -> list[tuple[float, float]]:
    """Return (gap_start_s, gap_duration_s) for every inter-message gap > threshold."""
    gaps: list[tuple[float, float]] = []
    if times.size < 2:
        return gaps
    deltas = np.diff(times)
    for index in range(deltas.size):
        if deltas[index] > loss_threshold_ns:
            gaps.append(
                (float(times[index]) / NANOSECONDS,
                 float(deltas[index]) / NANOSECONDS)
            )
    return gaps


def active_time(times: np.ndarray, gap_threshold_ns: int) -> float:
    """Total seconds covered by runs whose inter-message gap stays below threshold."""
    if times.size == 0:
        return 0.0
    if times.size == 1:
        return 0.0
    deltas = np.diff(times)
    run_seconds = float(times[-1] - times[0]) / NANOSECONDS
    for index in range(deltas.size):
        if deltas[index] > gap_threshold_ns:
            run_seconds -= float(deltas[index]) / NANOSECONDS
    return max(0.0, run_seconds)


def split_segments(
    times: np.ndarray,
    values: np.ndarray,
    gap_threshold_ns: int,
    jump_threshold_m: float,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """Split a stream into stationary segments.

    A new segment starts whenever the inter-message gap exceeds
    ``gap_threshold_ns`` or the horizontal step displacement exceeds
    ``jump_threshold_m``. ``values`` must keep position in the first two
    columns.
    """
    segments: list[tuple[np.ndarray, np.ndarray]] = []
    if times.size == 0:
        return segments
    segment_times = [times[0]]
    segment_values = [values[0]]
    for index in range(1, times.size):
        gap = times[index] - times[index - 1]
        jump = float(
            np.linalg.norm(values[index, :2] - values[index - 1, :2])
        )
        if gap > gap_threshold_ns or jump > jump_threshold_m:
            segments.append(
                (np.asarray(segment_times), np.asarray(segment_values))
            )
            segment_times = []
            segment_values = []
        segment_times.append(times[index])
        segment_values.append(values[index])
    segments.append((np.asarray(segment_times), np.asarray(segment_values)))
    return segments


def settle_seconds(
    times: np.ndarray,
    values: np.ndarray,
    window: int,
    position_std_m: float,
    size_std_m: float,
    yaw_std_deg: float,
) -> float:
    """Seconds from segment start until a trailing window is stable.

    Mirrors the detectors' LOCK criterion: a fixed number of consecutive
    samples whose position/size/yaw standard deviations stay below physical
    thresholds. Columns: [x, y, z, yaw, L, W, H].
    """
    if values.shape[0] < window:
        return float("nan")
    yaw_std_rad = math.radians(yaw_std_deg)
    for end in range(window - 1, values.shape[0]):
        window_values = values[end - window + 1: end + 1]
        if (
            np.all(np.std(window_values[:, :2], axis=0) <= position_std_m)
            and float(np.std(window_values[:, 2])) <= position_std_m
            and np.all(np.std(window_values[:, 4:7], axis=0) <= size_std_m)
            and float(np.std(axis_yaw_error(window_values[:, 3],
                                            axis_yaw_mean(window_values[:, 3]))))
            <= yaw_std_rad
        ):
            return float(times[end] - times[0]) / NANOSECONDS
    return float("nan")


def phase_stats(
    times: np.ndarray,
    values: np.ndarray,
) -> dict[str, float]:
    """Mean/standard-deviation statistics of one stationary segment.

    ``values`` columns: [x, y, z, yaw, L, W, H]. ``std`` of position and size
    is reported in millimetres, yaw in degrees.
    """
    if values.shape[0] == 0:
        return {}
    center_mm = np.std(values[:, :3], axis=0) * 1000.0
    size_mm = np.std(values[:, 4:7], axis=0) * 1000.0
    yaw_mean = axis_yaw_mean(values[:, 3])
    yaw_std_deg = math.degrees(
        float(np.std(axis_yaw_error(values[:, 3], yaw_mean)))
    )
    return {
        "center_std_mm": float(np.max(center_mm)),
        "center_std_xyz_mm": center_mm.tolist(),
        "size_std_mm": float(np.max(size_mm)),
        "size_std_LWH_mm": size_mm.tolist(),
        "yaw_std_deg": yaw_std_deg,
        "mean_center": np.mean(values[:, :3], axis=0).tolist(),
        "mean_size": np.mean(values[:, 4:7], axis=0).tolist(),
        "mean_yaw_deg": math.degrees(yaw_mean),
        "samples": int(values.shape[0]),
    }


def print_phase_stats(indent: str, stats: dict[str, float]) -> None:
    center = stats["mean_center"]
    size = stats["mean_size"]
    print(
        f"{indent}center=[{center[0]:.3f},{center[1]:.3f},{center[2]:.3f}] m "
        f"size=[{size[0]:.3f},{size[1]:.3f},{size[2]:.3f}] m "
        f"yaw={stats['mean_yaw_deg']:.1f} deg"
    )
    print(
        f"{indent}  std: center {stats['center_std_xyz_mm'][0]:.1f}/"
        f"{stats['center_std_xyz_mm'][1]:.1f}/"
        f"{stats['center_std_xyz_mm'][2]:.1f} mm | size "
        f"{stats['size_std_LWH_mm'][0]:.1f}/{stats['size_std_LWH_mm'][1]:.1f}/"
        f"{stats['size_std_LWH_mm'][2]:.1f} mm | yaw {stats['yaw_std_deg']:.2f} deg"
        f" | n={stats['samples']}"
    )


@dataclass
class BagData:
    """Deserialized statistics-relevant content of one bag."""

    rgb_times: np.ndarray
    depth_times: np.ndarray
    box_samples: np.ndarray  # [t, x, y, z, yaw, L, W, H, top_z, support_z]
    box_pose_count: int
    object_samples: np.ndarray  # [t, x, y, z, yaw, L, W, H]
    object_pose_count: int


def load_bag(
    sensor_bag_paths: list[Path],
    output_bag_paths: list[Path],
    box_top_topic: str,
) -> BagData:
    """Read RGB/depth timestamps from the sensor bag and detector topics from
    the output bag, then extract only the small topics needed for statistics.

    Splitting the sources keeps recorded detector output clean: a bag that
    contains historical /box/pose (e.g. a baseline capture) is only used for
    its RGB/depth frame timestamps, never for detector statistics.
    """
    typestore = get_typestore(Stores.ROS2_JAZZY)
    rgb_times: list[int] = []
    depth_times: list[int] = []
    with AnyReader(sensor_bag_paths, default_typestore=typestore) as reader:
        for connection, timestamp, _rawdata in reader.messages():
            if connection.topic == RGB_TOPIC:
                rgb_times.append(timestamp)
            elif connection.topic == DEPTH_TOPIC:
                depth_times.append(timestamp)

    box_pose_records: list[tuple[int, int, list[float]]] = []
    box_dims_by_stamp: dict[int, list[float]] = {}
    box_top_records: list[tuple[int, float]] = []
    object_pose: dict[int, list[float]] = {}
    object_dims: dict[int, list[float]] = {}

    with AnyReader(output_bag_paths, default_typestore=typestore) as reader:
        for connection, timestamp, rawdata in reader.messages():
            topic = connection.topic
            if topic == BOX_POSE_TOPIC:
                message = reader.deserialize(rawdata, connection.msgtype)
                orientation = message.pose.orientation
                yaw = 2.0 * math.atan2(orientation.z, orientation.w)
                box_pose_records.append(
                    (
                        timestamp,
                        stamp_ns(message),
                        [
                            float(message.pose.position.x),
                            float(message.pose.position.y),
                            float(message.pose.position.z),
                            yaw,
                        ],
                    )
                )
            elif topic == BOX_DIMS_TOPIC:
                message = reader.deserialize(rawdata, connection.msgtype)
                box_dims_by_stamp[stamp_ns(message)] = [
                    float(message.vector.x),
                    float(message.vector.y),
                    float(message.vector.z),
                ]
            elif topic == box_top_topic:
                message = reader.deserialize(rawdata, connection.msgtype)
                box_top_records.append((timestamp, float(message.data)))
            elif topic == OBJ_POSE_TOPIC:
                message = reader.deserialize(rawdata, connection.msgtype)
                orientation = message.pose.orientation
                yaw = 2.0 * math.atan2(orientation.z, orientation.w)
                object_pose[stamp_ns(message)] = [
                    float(message.pose.position.x),
                    float(message.pose.position.y),
                    float(message.pose.position.z),
                    yaw,
                ]
            elif topic == OBJ_DIMS_TOPIC:
                message = reader.deserialize(rawdata, connection.msgtype)
                object_dims[stamp_ns(message)] = [
                    float(message.vector.x),
                    float(message.vector.y),
                    float(message.vector.z),
                ]

    # /box/top_height is a bare Float64 (no header). Match each pose to the
    # top_height record with the nearest bag record time (published together).
    top_times = np.asarray([record[0] for record in box_top_records], dtype=np.int64)
    top_values = np.asarray([record[1] for record in box_top_records], dtype=np.float64)

    box_rows: list[np.ndarray] = []
    for record_time, stamp, pose in sorted(
        box_pose_records, key=lambda item: item[0]
    ):
        dimensions = box_dims_by_stamp.get(stamp)
        if dimensions is None:
            continue
        if top_times.size:
            nearest = int(np.argmin(np.abs(top_times - record_time)))
            if abs(float(top_times[nearest] - record_time)) < 0.5 * NANOSECONDS:
                top_z = float(top_values[nearest])
            else:
                continue
        else:
            continue
        support_z = 2.0 * pose[2] - top_z
        box_rows.append(
            np.asarray(
                [stamp, pose[0], pose[1], pose[2], pose[3],
                 dimensions[0], dimensions[1], dimensions[2], top_z, support_z],
                dtype=np.float64,
            )
        )
    box_samples = (
        np.asarray(box_rows, dtype=np.float64)
        if box_rows
        else np.empty((0, 10), dtype=np.float64)
    )
    box_samples = box_samples[np.argsort(box_samples[:, 0])]

    object_rows: list[np.ndarray] = []
    for stamp, pose in object_pose.items():
        dimensions = object_dims.get(stamp)
        if dimensions is None:
            continue
        object_rows.append(
            np.asarray(
                [stamp, pose[0], pose[1], pose[2], pose[3],
                 dimensions[0], dimensions[1], dimensions[2]],
                dtype=np.float64,
            )
        )
    object_samples = (
        np.asarray(object_rows, dtype=np.float64)
        if object_rows
        else np.empty((0, 8), dtype=np.float64)
    )
    object_samples = object_samples[np.argsort(object_samples[:, 0])]

    return BagData(
        rgb_times=np.asarray(sorted(rgb_times), dtype=np.int64),
        depth_times=np.asarray(sorted(depth_times), dtype=np.int64),
        box_samples=box_samples,
        box_pose_count=len(box_pose_records),
        object_samples=object_samples,
        object_pose_count=len(object_pose),
    )


def report_bag(
    name: str,
    data: BagData,
    args: argparse.Namespace,
) -> None:
    """Print one bag's generic statistics (box + object sections)."""
    print(f"===== BAG: {name} =====")
    if data.rgb_times.size == 0:
        print("  (no RGB frames found)")
        print()
        return
    duration = float(data.rgb_times[-1] - data.rgb_times[0]) / NANOSECONDS
    print(f"duration            {duration:.1f} s")
    print(f"rgb frames          {data.rgb_times.size}")
    print(f"depth frames        {data.depth_times.size}")

    coverage_window = int(args.coverage_window_sec * NANOSECONDS)
    box_loss_ns = int(args.box_loss_sec * NANOSECONDS)
    box_gap_ns = int(args.gap_sec * NANOSECONDS)
    obj_loss_ns = int(args.object_loss_sec * NANOSECONDS)
    obj_gap_ns = int(args.gap_sec * NANOSECONDS)

    print("--- BOX ---")
    if data.box_samples.shape[0] == 0:
        print("  /box/pose: no messages")
    else:
        pose_times = data.box_samples[:, 0]
        coverage = nearest_coverage(pose_times, data.rgb_times, coverage_window)
        gaps = stream_gaps(pose_times, box_loss_ns)
        active = active_time(pose_times, box_loss_ns)
        print(f"/box/pose msgs      {data.box_pose_count}")
        print(f"pose publish rate   {100.0 * data.box_pose_count / data.rgb_times.size:.1f} %")
        print(f"box coverage        {100.0 * coverage:.1f} %")
        print(f"lock losses         {len(gaps)}  (total {sum(g for _, g in gaps):.2f} s)")
        print(f"lock availability   {100.0 * active / duration:.1f} %")

        values = data.box_samples[:, 1:8]  # [x, y, z, yaw, L, W, H]
        segments = split_segments(pose_times, values, box_gap_ns, args.jump_m)
        print(f"box segments        {len(segments)}")
        all_stats = []
        for index, (segment_times, segment_values) in enumerate(segments):
            stats = phase_stats(segment_times, segment_values)
            if not stats:
                continue
            all_stats.append(stats)
            if len(segments) > 1 or args.verbose:
                relock = settle_seconds(
                    segment_times, segment_values, args.window_samples,
                    args.settle_pos_m, args.settle_size_m, args.settle_yaw_deg,
                )
                relock_text = "n/a" if math.isnan(relock) else f"{relock:.2f} s"
                print(
                    f"  segment {index}: t="
                    f"{float(segment_times[0]) / NANOSECONDS:.1f}-"
                    f"{float(segment_times[-1]) / NANOSECONDS:.1f} s "
                    f"(relock {relock_text})"
                )
                print_phase_stats("    ", stats)

        if all_stats:
            size_stability_mm = (
                np.std(
                    np.asarray([s["mean_size"] for s in all_stats]), axis=0
                )
                * 1000.0
            )
            if len(all_stats) > 1:
                print(
                    f"  cross-phase size stability: "
                    f"{size_stability_mm[0]:.1f}/{size_stability_mm[1]:.1f}/"
                    f"{size_stability_mm[2]:.1f} mm (std of phase means)"
                )
            if len(all_stats) == 1:
                stats = all_stats[0]
                top_z_std_mm = float(np.std(data.box_samples[:, 8])) * 1000.0
                support_z_std_mm = float(np.std(data.box_samples[:, 9])) * 1000.0
                print("  LOCK statistics (single stationary segment):")
                print_phase_stats("    ", stats)
                print(
                    f"    top_z std {top_z_std_mm:.1f} mm | "
                    f"support_z std {support_z_std_mm:.1f} mm"
                )

    print("--- OBJECT ---")
    if data.object_samples.shape[0] == 0:
        print("  /blue_cube/pose: no messages")
    else:
        pose_times = data.object_samples[:, 0]
        coverage = nearest_coverage(pose_times, data.rgb_times, coverage_window)
        gaps = stream_gaps(pose_times, obj_loss_ns)
        active = active_time(pose_times, obj_loss_ns)
        print(f"/blue_cube/pose msgs {data.object_pose_count}")
        print(
            f"object publish rate {100.0 * data.object_pose_count / data.rgb_times.size:.1f} %"
        )
        print(f"object coverage     {100.0 * coverage:.1f} %")
        print(f"object losses       {len(gaps)}  (total {sum(g for _, g in gaps):.2f} s)")
        print(f"object availability {100.0 * active / duration:.1f} %")

        values = data.object_samples[:, 1:8]  # [x, y, z, yaw, L, W, H]
        segments = split_segments(pose_times, values, obj_gap_ns, args.jump_m)
        all_stats = [
            phase_stats(segment_times, segment_values)
            for segment_times, segment_values in segments
        ]
        all_stats = [s for s in all_stats if s]
        print(f"object segments     {len(all_stats)}")
        for index, stats in enumerate(all_stats):
            print(f"  segment {index}:")
            print_phase_stats("    ", stats)
        if gaps:
            mean_reacquire = float(np.mean([g for _, g in gaps]))
            print(f"mean reacquisition  {mean_reacquire:.2f} s")
    print()


def discover_bag_files(bag_path: str) -> list[Path]:
    """Expand a bag directory (or direct file) into mcap files."""
    if os.path.isfile(bag_path):
        return [Path(bag_path)]
    files = sorted(glob.glob(os.path.join(bag_path, "*.mcap")))
    if not files:
        raise SystemExit(f"no .mcap files found under {bag_path}")
    return [Path(file) for file in files]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Offline statistics baseline for OpenArm RGB-D benchmark bags."
    )
    parser.add_argument(
        "--bags",
        nargs="+",
        required=True,
        help="bag directories or .mcap files",
    )
    parser.add_argument(
        "--sensor-bags",
        nargs="+",
        default=None,
        help="bag(s) that only provide RGB/depth frame timestamps (default: "
        "same as --bags). Pass the baseline bag here when --bags points at a "
        "re-recorded detector-output bag that lacks camera images.",
    )
    parser.add_argument(
        "--box-top-topic",
        default=BOX_TOP_TOPIC,
        help="box top-height topic (default %(default)s)",
    )
    parser.add_argument("--coverage-window-sec", type=float, default=0.5)
    parser.add_argument("--box-loss-sec", type=float, default=1.0)
    parser.add_argument("--object-loss-sec", type=float, default=0.5)
    parser.add_argument("--gap-sec", type=float, default=1.5)
    parser.add_argument("--jump-m", type=float, default=0.04)
    parser.add_argument("--window-samples", type=int, default=5)
    parser.add_argument("--settle-pos-m", type=float, default=0.008)
    parser.add_argument("--settle-size-m", type=float, default=0.010)
    parser.add_argument("--settle-yaw-deg", type=float, default=2.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    for bag_path in args.bags:
        output_files = discover_bag_files(bag_path)
        sensor_files: list[Path] = []
        for sensor_path in args.sensor_bags or [bag_path]:
            sensor_files.extend(discover_bag_files(sensor_path))
        print(
            f"loading output {bag_path} ({len(output_files)} mcap), "
            f"sensor {len(sensor_files)} mcap ..."
        )
        data = load_bag(sensor_files, output_files, args.box_top_topic)
        name = os.path.basename(bag_path.rstrip("/"))
        report_bag(name, data, args)


if __name__ == "__main__":
    main()
