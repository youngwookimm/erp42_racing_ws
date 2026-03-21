# waypoint_loader.py
import csv
import os
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ament_index_python.packages import get_package_share_directory


def _norm_key(k: str) -> str:
    return k.strip().lower().replace(" ", "")


def read_csv_points(csv_path: str) -> List[Tuple[float, float]]:
    """
    Read points from CSV.
    Supports common header variants:
      - UTM_X(East), UTM_Y(North)
      - UTM_X, UTM_Y
      - x, y
      - UTM_X(Easting), UTM_Y(Northing)
    """
    pts: List[Tuple[float, float]] = []
    with open(csv_path, newline="") as f:
        # delimiter 자동 추정(콤마/세미콜론/탭 흔함)
        sample = f.read(2048)
        f.seek(0)
        try:
            dialect = csv.Sniffer().sniff(sample, delimiters=[",", ";", "\t"])
        except csv.Error:
            dialect = csv.excel  # fallback: comma

        reader = csv.DictReader(f, dialect=dialect)
        fieldnames = reader.fieldnames or []
        norm_map = {_norm_key(k): k for k in fieldnames}

        x_candidates = ["utm_x(east)", "utm_x(easting)", "utm_x", "x", "east", "easting"]
        y_candidates = ["utm_y(north)", "utm_y(northing)", "utm_y", "y", "north", "northing"]

        x_key = next((norm_map.get(_norm_key(c)) for c in x_candidates if _norm_key(c) in norm_map), None)
        y_key = next((norm_map.get(_norm_key(c)) for c in y_candidates if _norm_key(c) in norm_map), None)

        if x_key is None or y_key is None:
            raise KeyError(
                f"CSV header mismatch: {csv_path}\n"
                f"fieldnames={fieldnames}\n"
                f"Need X in {x_candidates}, Y in {y_candidates}"
            )

        for row in reader:
            x = float(row[x_key])
            y = float(row[y_key])
            pts.append((x, y))
    return pts


def build_path(points_utm: List[Tuple[float, float]], origin_utm: Tuple[float, float], frame_id: str) -> Path:
    """Build nav_msgs/Path in frame_id using a shared origin for both roads."""
    ox, oy = origin_utm
    path = Path()
    path.header.frame_id = frame_id

    for x, y in points_utm:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(x - ox)
        ps.pose.position.y = float(y - oy)
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)

    return path


class WaypointLoader(Node):

    def __init__(self):
        super().__init__("waypoint_loader")

        # ---- resource paths ----
        pkg_share = get_package_share_directory("erp42_racing_planning")
        default_road1 = os.path.join(pkg_share, "resource", "L1_sejong.csv")
        default_road2 = os.path.join(pkg_share, "resource", "R1_sejong.csv")

        # ---- params (이름 통일) ----
        self.declare_parameter("frame_id", "map")

        self.declare_parameter("road1_csv", default_road1)
        self.declare_parameter("road2_csv", default_road2)

        self.declare_parameter("road1_topic", "/localization/right_lane")
        self.declare_parameter("road2_topic", "/localization/left_lane")
        self.declare_parameter("publish_period_sec", 1.0)

        frame_id = str(self.get_parameter("frame_id").value)
        road1_csv = str(self.get_parameter("road1_csv").value)
        road2_csv = str(self.get_parameter("road2_csv").value)

        self.road1_topic = str(self.get_parameter("road1_topic").value)
        self.road2_topic = str(self.get_parameter("road2_topic").value)

        period = float(self.get_parameter("publish_period_sec").value)

        # ---- pubs ----
        self.pub_road1 = self.create_publisher(Path, self.road1_topic, 10)
        self.pub_road2 = self.create_publisher(Path, self.road2_topic, 10)

        # ---- load CSVs ----
        road2_pts = read_csv_points(road2_csv)  # overtake lane
        road1_pts = read_csv_points(road1_csv)  # reference lane

        # 수정
        if len(road2_pts) < 2:
            raise RuntimeError(f"road2 CSV has <2 points: {road2_csv}")
        if len(road1_pts) < 2:
            raise RuntimeError(f"road1 CSV has <2 points: {road1_csv}")

        origin = road1_pts[0]

        self.path_road2 = build_path(road2_pts, origin, frame_id)
        self.path_road1 = build_path(road1_pts, origin, frame_id)

        self.get_logger().info(
            f"Loaded road2={len(self.path_road2.poses)} poses from {road2_csv}\n"
            f"Loaded road1={len(self.path_road1.poses)} poses from {road1_csv}\n"
            f"Publish topics: {self.road2_topic}, {self.road1_topic} (frame_id={frame_id})\n"
            f"Shared origin: ({origin[0]:.3f}, {origin[1]:.3f})"
        )
	
        # ---- timer ----
        self.timer = self.create_timer(period, self.publish_paths)
        

    def publish_paths(self):
        stamp = self.get_clock().now().to_msg()

        self.path_road2.header.stamp = stamp
        self.path_road1.header.stamp = stamp

        for ps in self.path_road2.poses:
            ps.header.stamp = stamp
        for ps in self.path_road1.poses:
            ps.header.stamp = stamp

        self.pub_road2.publish(self.path_road2)
        self.pub_road1.publish(self.path_road1)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()