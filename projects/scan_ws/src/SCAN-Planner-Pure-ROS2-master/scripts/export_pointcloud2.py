#!/usr/bin/env python3

"""Export a ROS 2 PointCloud2 topic to PCD, PLY, or CSV."""

import argparse
import csv
import math
import sys
import time
from pathlib import Path
from typing import Iterable, List, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


COMMON_EXTRA_FIELDS = (
    "intensity",
    "reflectivity",
    "rgb",
    "rgba",
    "ring",
    "time",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Save samples from a sensor_msgs/msg/PointCloud2 topic."
    )
    parser.add_argument(
        "--topic",
        default="/robot_0/cloud_registered_world",
        help="PointCloud2 topic to export.",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Output path. Extension selects format when --format is omitted.",
    )
    parser.add_argument(
        "--format",
        choices=("pcd", "ply", "csv"),
        default=None,
        help="Output format. Defaults to the output file extension, then pcd.",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="Accumulate all received clouds for this many seconds. 0 saves one frame.",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=None,
        help="Number of PointCloud2 messages to accumulate. Defaults to 1 unless --seconds is set.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the first cloud before failing.",
    )
    parser.add_argument(
        "--qos",
        choices=("best_effort", "reliable"),
        default="best_effort",
        help="Subscriber reliability policy.",
    )
    parser.add_argument(
        "--depth",
        type=int,
        default=10,
        help="Subscriber QoS queue depth.",
    )
    parser.add_argument(
        "--keep-nans",
        action="store_true",
        help="Keep points with NaN values instead of dropping them.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=0,
        help="Stop after this many points. 0 means no point limit.",
    )
    parser.add_argument(
        "--fields",
        nargs="+",
        default=None,
        help="Fields to export. x y z are always included.",
    )
    return parser.parse_args()


def resolve_format(output: Path, requested: str | None) -> str:
    if requested:
        return requested
    suffix = output.suffix.lower().lstrip(".")
    return suffix if suffix in {"pcd", "ply", "csv"} else "pcd"


def default_output_path(topic: str, fmt: str) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    topic_name = topic.strip("/").replace("/", "_") or "pointcloud"
    return Path("exports") / f"{topic_name}_{stamp}.{fmt}"


def message_field_names(msg: PointCloud2, requested: Sequence[str] | None) -> List[str]:
    available = {field.name for field in msg.fields}
    missing_xyz = {"x", "y", "z"} - available
    if missing_xyz:
        raise RuntimeError(f"PointCloud2 is missing required fields: {sorted(missing_xyz)}")

    names = ["x", "y", "z"]
    if requested:
        for name in requested:
            if name not in names:
                names.append(name)
    else:
        for name in COMMON_EXTRA_FIELDS:
            if name in available:
                names.append(name)

    missing = [name for name in names if name not in available]
    if missing:
        raise RuntimeError(f"PointCloud2 does not contain requested fields: {missing}")
    return names


def scalar(value):
    if hasattr(value, "item"):
        value = value.item()
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def point_rows(
    msg: PointCloud2,
    names: Sequence[str],
    skip_nans: bool,
) -> Iterable[List[float]]:
    points = point_cloud2.read_points(msg, field_names=names, skip_nans=skip_nans)
    for point in points:
        dtype = getattr(point, "dtype", None)
        dtype_names = getattr(dtype, "names", None)
        if dtype_names:
            yield [scalar(point[name]) for name in names]
        else:
            yield [scalar(value) for value in point]


def is_finite_xyz(row: Sequence[float]) -> bool:
    try:
        return math.isfinite(float(row[0])) and math.isfinite(float(row[1])) and math.isfinite(float(row[2]))
    except (TypeError, ValueError):
        return False


class PointCloudExporter(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("pointcloud2_exporter")
        self.args = args
        self.fields: List[str] | None = None
        self.rows: List[List[float]] = []
        self.messages = 0
        self.first_message_time: float | None = None
        self.done = False

        reliability = (
            ReliabilityPolicy.RELIABLE
            if args.qos == "reliable"
            else ReliabilityPolicy.BEST_EFFORT
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=args.depth,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            args.topic,
            self.callback,
            qos,
        )

    def callback(self, msg: PointCloud2) -> None:
        try:
            if self.fields is None:
                self.fields = message_field_names(msg, self.args.fields)
            rows = point_rows(msg, self.fields, skip_nans=not self.args.keep_nans)
            added = 0
            for row in rows:
                if not self.args.keep_nans and not is_finite_xyz(row):
                    continue
                self.rows.append(row)
                added += 1
                if self.args.max_points > 0 and len(self.rows) >= self.args.max_points:
                    self.done = True
                    break
        except Exception as exc:  # noqa: BLE001 - report ROS callback errors cleanly.
            self.get_logger().error(str(exc))
            self.done = True
            return

        self.messages += 1
        if self.first_message_time is None:
            self.first_message_time = time.monotonic()
        self.get_logger().info(
            f"Captured frame {self.messages}: +{added} points, total={len(self.rows)}"
        )

        if self.args.frames and self.messages >= self.args.frames:
            self.done = True


def write_pcd(path: Path, fields: Sequence[str], rows: Sequence[Sequence[float]]) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
        stream.write("VERSION 0.7\n")
        stream.write(f"FIELDS {' '.join(fields)}\n")
        stream.write(f"SIZE {' '.join(['4'] * len(fields))}\n")
        stream.write(f"TYPE {' '.join(['F'] * len(fields))}\n")
        stream.write(f"COUNT {' '.join(['1'] * len(fields))}\n")
        stream.write(f"WIDTH {len(rows)}\n")
        stream.write("HEIGHT 1\n")
        stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(rows)}\n")
        stream.write("DATA ascii\n")
        for row in rows:
            stream.write(" ".join(str(float(value)) for value in row) + "\n")


def write_ply(path: Path, fields: Sequence[str], rows: Sequence[Sequence[float]]) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("ply\n")
        stream.write("format ascii 1.0\n")
        stream.write(f"element vertex {len(rows)}\n")
        for field in fields:
            stream.write(f"property float {field}\n")
        stream.write("end_header\n")
        for row in rows:
            stream.write(" ".join(str(float(value)) for value in row) + "\n")


def write_csv(path: Path, fields: Sequence[str], rows: Sequence[Sequence[float]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(rows)


def write_output(path: Path, fmt: str, fields: Sequence[str], rows: Sequence[Sequence[float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fmt == "pcd":
        write_pcd(path, fields, rows)
    elif fmt == "ply":
        write_ply(path, fields, rows)
    elif fmt == "csv":
        write_csv(path, fields, rows)
    else:
        raise RuntimeError(f"Unsupported output format: {fmt}")


def main() -> int:
    args = parse_args()
    if args.seconds < 0.0:
        print("--seconds must be >= 0", file=sys.stderr)
        return 2
    if args.frames is not None and args.frames < 1:
        print("--frames must be >= 1", file=sys.stderr)
        return 2
    if args.timeout <= 0.0:
        print("--timeout must be > 0", file=sys.stderr)
        return 2
    if args.max_points < 0:
        print("--max-points must be >= 0", file=sys.stderr)
        return 2

    requested_format = args.format
    provisional_output = Path(args.output) if args.output else Path("cloud.pcd")
    fmt = resolve_format(provisional_output, requested_format)
    output = Path(args.output) if args.output else default_output_path(args.topic, fmt)

    if args.frames is None and args.seconds <= 0.0:
        args.frames = 1

    rclpy.init()
    node = PointCloudExporter(args)
    start = time.monotonic()
    deadline = start + args.timeout
    capture_end = None

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if node.first_message_time is None and now > deadline:
                node.get_logger().error(f"Timed out waiting for {args.topic}")
                return 1
            if (
                node.first_message_time is not None
                and args.seconds > 0.0
                and capture_end is None
            ):
                capture_end = node.first_message_time + args.seconds
            if capture_end is not None and now >= capture_end:
                break

        if not node.rows or node.fields is None:
            node.get_logger().error("No points were captured; nothing to export.")
            return 1

        write_output(output, fmt, node.fields, node.rows)
        node.get_logger().info(
            f"Exported {len(node.rows)} points from {node.messages} frame(s) to {output}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
