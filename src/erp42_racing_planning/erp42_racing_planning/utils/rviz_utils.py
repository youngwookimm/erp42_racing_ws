# erp42_racing_planning/utils/rviz_utils.py
from __future__ import annotations

from typing import List, Sequence, Tuple

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from erp42_racing_planning.core.planner_types import LocalTrajectory


# =============================================================================
# Basic helpers
# =============================================================================

def make_point(x: float, y: float, z: float = 0.0) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def make_color_rgba(r: float, g: float, b: float, a: float) -> ColorRGBA:
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def _make_base_marker(
    frame_id: str,
    stamp_msg,
    ns: str,
    mid: int,
    marker_type: int,
) -> Marker:
    mk = Marker()
    mk.header.frame_id      = frame_id
    mk.header.stamp         = stamp_msg
    mk.ns                   = ns
    mk.id                   = int(mid)
    mk.type                 = marker_type
    mk.action               = Marker.ADD
    mk.pose.orientation.w   = 1.0

    return mk


def make_clear_all_marker(frame_id: str, stamp_msg, ns: str = "") -> Marker:
    """
    현재 topic에 게시된 기존 marker들을 지우기 위한 DELETEALL marker.
    ns는 시각적 구분용으로만 넣고, 실제 동작은 전체 삭제로 보는 것이 안전하다.
    """
    mk                  = Marker()
    mk.header.frame_id  = frame_id
    mk.header.stamp     = stamp_msg
    mk.ns               = ns
    mk.id               = 0
    mk.action           = Marker.DELETEALL

    return mk


# =============================================================================
# Generic markers
# =============================================================================

def make_sphere_marker(
    frame_id: str,
    stamp_msg,
    ns: str,
    mid: int,
    xy: Tuple[float, float],
    radius: float = 0.35,
    rgba: Tuple[float, float, float, float] = (1.0, 0.6, 0.1, 0.9),
    z: float = 0.25,
) -> Marker:
    mk = _make_base_marker(
        frame_id=frame_id,
        stamp_msg=stamp_msg,
        ns=ns,
        mid=mid,
        marker_type=Marker.SPHERE,
    )

    mk.pose.position.x = float(xy[0])
    mk.pose.position.y = float(xy[1])
    mk.pose.position.z = float(z)

    diameter    = float(radius * 2.0)
    mk.scale.x  = diameter
    mk.scale.y  = diameter
    mk.scale.z  = diameter

    r, g, b, a  = rgba
    mk.color    = make_color_rgba(r, g, b, a)

    return mk


def make_sphere_array(
    frame_id: str,
    stamp_msg,
    ns: str,
    points_xy: Sequence[Tuple[float, float]],
    radius: float = 0.35,
    rgba: Tuple[float, float, float, float] = (1.0, 0.6, 0.1, 0.9),
    z: float = 0.25,
    id_base: int = 7000,
    clear_first: bool = True,
) -> MarkerArray:
    arr = MarkerArray()

    if clear_first:
        arr.markers.append(make_clear_all_marker(frame_id=frame_id, stamp_msg=stamp_msg, ns=ns))

    for i, (x, y) in enumerate(points_xy):
        arr.markers.append(
            make_sphere_marker(
                frame_id=frame_id,
                stamp_msg=stamp_msg,
                ns=ns,
                mid=id_base + i,
                xy=(x, y),
                radius=radius,
                rgba=rgba,
                z=z,
            )
        )

    return arr


def make_box_lines_marker(
    frame_id: str,
    stamp_msg,
    ns: str,
    mid: int,
    corners_xy: Sequence[Tuple[float, float]],
    width: float = 0.06,
    rgba: Tuple[float, float, float, float] = (1.0, 0.2, 0.2, 0.9),
    z: float = 0.15,
) -> Marker:
    mk = _make_base_marker(
        frame_id=frame_id,
        stamp_msg=stamp_msg,
        ns=ns,
        mid=mid,
        marker_type=Marker.LINE_STRIP,
    )

    mk.scale.x  = float(width)

    r, g, b, a  = rgba
    mk.color    = make_color_rgba(r, g, b, a)

    pts = [make_point(x, y, z) for (x, y) in corners_xy]
    if pts:
        pts.append(make_point(corners_xy[0][0], corners_xy[0][1], z))  # 닫기
    mk.points = pts
    return mk


# =============================================================================
# Local planner markers
# =============================================================================

def make_line_strip_marker(
    frame_id: str,
    stamp_msg,
    ns: str,
    mid: int,
    points_xyz: Sequence[Tuple[float, float, float]],
    width: float = 0.10,
    rgba: Tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0),
) -> Marker:
    mk = _make_base_marker(
        frame_id=frame_id,
        stamp_msg=stamp_msg,
        ns=ns,
        mid=mid,
        marker_type=Marker.LINE_STRIP,
    )

    mk.scale.x  = float(width)

    r, g, b, a  = rgba
    mk.color    =  make_color_rgba(r, g, b, a)

    mk.points   = [make_point(x, y, z) for (x, y, z) in points_xyz]
    
    return mk


def make_points_marker(
    frame_id: str,
    stamp_msg,
    ns: str,
    mid: int,
    points_xyz: Sequence[Tuple[float, float, float]],
    size: float = 0.12,
    rgba: Tuple[float, float, float, float] = (1.0, 0.2, 0.2, 1.0),
) -> Marker:
    mk = _make_base_marker(
        frame_id=frame_id,
        stamp_msg=stamp_msg,
        ns=ns,
        mid=mid,
        marker_type=Marker.POINTS,
    )

    mk.scale.x  = float(size)
    mk.scale.y  = float(size)

    r, g, b, a  = rgba
    mk.color    = make_color_rgba(r, g, b, a)

    mk.points   = [make_point(x, y, z) for (x, y, z) in points_xyz]
    return mk


def build_selected_marker_array(
    trajectory: LocalTrajectory,
    frame_id: str,
    stamp_msg,
    clear_first: bool = False,
) -> MarkerArray:
    arr = MarkerArray()

    if clear_first:
        arr.markers.append(
            make_clear_all_marker(
                frame_id=frame_id,
                stamp_msg=stamp_msg,
                ns="selected_trajectory",
            )
        )

    if not trajectory.points:
        return arr

    points_xyz = [(float(pt.x), float(pt.y), 0.0) for pt in trajectory.points]

    arr.markers.append(
        make_line_strip_marker(
            frame_id=frame_id,
            stamp_msg=stamp_msg,
            ns="selected_trajectory",
            mid=0,
            points_xyz=points_xyz,
            width=0.18,
            rgba=(1.0, 0.0, 0.0, 1.0),
        )
    )

    arr.markers.append(
        make_points_marker(
            frame_id=frame_id,
            stamp_msg=stamp_msg,
            ns="selected_trajectory_points",
            mid=1,
            points_xyz=points_xyz,
            size=0.12,
            rgba=(1.0, 0.2, 0.2, 1.0),
        )
    )

    return arr


def build_candidate_marker_array(
    candidates: Sequence[LocalTrajectory],
    frame_id: str,
    stamp_msg,
    clear_first: bool = False,
) -> MarkerArray:
    arr = MarkerArray()

    if clear_first:
        arr.markers.append(
            make_clear_all_marker(
                frame_id=frame_id,
                stamp_msg=stamp_msg,
                ns="candidate_trajectories",
            )
        )

    for idx, cand in enumerate(candidates):
        if not cand.points:
            continue

        points_xyz = [(float(pt.x), float(pt.y), 0.0) for pt in cand.points]

        arr.markers.append(
            make_line_strip_marker(
                frame_id=frame_id,
                stamp_msg=stamp_msg,
                ns="candidate_trajectories",
                mid=1000 + idx,
                points_xyz=points_xyz,
                width=0.05,
                rgba=(1.0, 1.0, 1.0, 0.45),
            )
        )

    return arr