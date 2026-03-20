# erp42_racing_planning/core/reference_path.py
from __future__ import annotations

import bisect
import math
from typing import List, Tuple, Optional

from .planner_types import (
    Waypoint,
    FrenetCoord,
    ProjectionResult,
)

from ..utils.geometry_utils import project_point_to_segment, signed_lateral_offset, left_normal
from ..utils.math_utils import wrap_s


class ReferencePath:
    """
    Closed-loop Reference centerline for Frenet coordinate conversion (racing track).
    """
    def __init__(self, waypoints: List[Waypoint]):
        
        if len(waypoints) < 2:
            raise ValueError("ReferencePath requires at least 2 waypoints.")
        
        self.waypoints: List[Waypoint] = waypoints
        self.num_points: int = len(waypoints)

        # cumulative arc-length map
        self.s_map: List[float] = [0.0]
        self._build_s_map()

        # total track length
        self.s_total: float = self.s_map[-1]
        if self.s_total <=1e-9:
            raise ValueError("ReferencePath total length is zero (degenerate waypoints).")

        # per-segment cached data
        self.segment_lengths: List[float] = []
        self.segment_tangents: List[Tuple[float, float]] = []
        self._build_segment_cache()

        # curvature cache
        self.curvatures = self.compute_curvatures()


    def _build_s_map(self) -> None:
        """
        Build cumulative arc-length s map along a closed-loop polyline.
        """
        self.s_map = [0.0]
        s_acc = 0.0

        # cumulative length
        for i in range(self.num_points):
            p0 = self.waypoints[i]
            p1 = self.waypoints[(i+1) % self.num_points]

            dx = p1.x - p0.x
            dy = p1.y - p0.y
            seg_len = math.hypot(dx,dy)

            s_acc += seg_len
            self.s_map.append(s_acc)
    
    def _build_segment_cache(self) -> None:
        """
        Precompute segment lengths and unit tangents.
        """
        self.segment_lengths = []       # length of segment i
        self.segment_tangents = []      # unit tangent

        last_tx, last_ty = 1.0, 0.0
        for i in range(self.num_points):
            p0 = self.waypoints[i]
            p1 = self.waypoints[(i + 1) % self.num_points]

            dx = p1.x - p0.x
            dy = p1.y - p0.y
            seg_len = math.hypot(dx, dy)

            self.segment_lengths.append(seg_len)

            if seg_len > 1e-12:
                last_tx, last_ty = dx / seg_len, dy / seg_len
           
            self.segment_tangents.append((last_tx, last_ty))

    def _find_segment_index(self, s: float) -> int:
        """
        Find the polyline segment that contains the given arc-length s.
        """
        idx = bisect.bisect_right(self.s_map, s) - 1

        if idx < 0:
            return 0
        if idx >= self.num_points:
            return self.num_points - 1
        return idx
    
    def project(self, x: float, y: float) -> ProjectionResult:
        """
        Project a Cartesian point onto the reference path and return the closest projection.
        """
        best_dist_sq = float("inf")
        best: Optional[ProjectionResult] = None

        for i in range(self.num_points):
            p0 = self.waypoints[i]
            p1 = self.waypoints[(i + 1) % self.num_points]

            seg_t, x_proj, y_proj = project_point_to_segment(
                x, y, p0.x, p0.y, p1.x, p1.y
            )

            tx, ty = self.segment_tangents[i]
            d = signed_lateral_offset(x, y, x_proj, y_proj, tx, ty)

            dx = x - x_proj
            dy = y - y_proj
            dist_sq = dx * dx + dy * dy

            if dist_sq < best_dist_sq:
                seg_len = self.segment_lengths[i]
                s_unwrapped = self.s_map[i] + seg_t * seg_len
                s_wrapped = wrap_s(s_unwrapped, self.s_total)

                best_dist_sq = dist_sq
                best = ProjectionResult(
                    segment_index=i, 
                    segment_t=seg_t,
                    s=s_wrapped, 
                    d=d,
                    x_proj=x_proj, 
                    y_proj=y_proj,
                    tx=tx, ty=ty,
                )

        # there is always at least one segment
        assert best is not None
        return best

    def to_frenet(self, x: float, y: float) -> FrenetCoord:
        """
        Convert Cartesian (x,y) -> Frenet (s,d)
        """
        proj = self.project(x, y)

        return FrenetCoord(s=proj.s, d=proj.d)

    def from_frenet(self, s: float, d: float) -> Waypoint:
        """
        Convert Frenet (s,d) -> Cartesian (x,y)
        """
        s_wrapped = wrap_s(s, self.s_total)
        seg_idx = self._find_segment_index(s_wrapped)

        p0 = self.waypoints[seg_idx]
        p1 = self.waypoints[(seg_idx + 1) % self.num_points]

        seg_len = self.segment_lengths[seg_idx]
        tx, ty = self.segment_tangents[seg_idx]

        if seg_len <= 1e-12:
            x_base, y_base = p0.x, p0.y
        else:
            local_s = s_wrapped - self.s_map[seg_idx]
            seg_t = local_s / seg_len
            x_base = p0.x + seg_t * (p1.x - p0.x)
            y_base = p0.y + seg_t * (p1.y - p0.y)

        nx, ny = left_normal(tx, ty)
        return Waypoint(x=x_base + d * nx, y=y_base + d * ny)

    def compute_curvatures(self, window: int = 3) -> List[float]:
        
        N = self.num_points

        # heading array
        headings = [math.atan2(ty, tx) for tx, ty in self.segment_tangents]

        curvatures: List[float] = []

        for i in range(N):

            j = (i + window) % self.num_points

            theta1 = headings[i]
            theta2 = headings[j]

            # heading difference
            dtheta = theta2 - theta1

            # wrap to [-pi, pi]
            dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi

            # arc length difference
            ds = self.s_map[j] - self.s_map[i]

            # handle wrap-around
            if ds < 0:
                ds += self.s_total

            if abs(ds) < 1e-9:
                curvatures.append(0.0)
            else:
                curvatures.append(dtheta / ds)

        return curvatures

    def longitudinal_distance(self, s_from: float, s_to: float) -> float:
        """
        Compute forward distance from s_from to s_to along the loop
        """
        s0 = wrap_s(s_from, self.s_total)
        s1 = wrap_s(s_to, self.s_total)
        return wrap_s(s1 - s0, self.s_total)

    def tangent_at_s(self, s: float) -> Tuple[float, float]:
        """
        Return the unit tangent vector at arc-length s
        """
        s_wrapped = wrap_s(s, self.s_total)
        seg_idx = self._find_segment_index(s_wrapped)
        return self.segment_tangents[seg_idx]

    def heading_at_s(self, s: float) -> float:
        """
        Return the heading angle at arc-length s
        """
        tx, ty = self.tangent_at_s(s)
        return math.atan2(ty, tx)

    def curvature_at_s(self, s: float) -> float:
        """
        Return curvature at arc-length s
        """
        s_wrapped = wrap_s(s, self.s_total)
        seg_idx = self._find_segment_index(s_wrapped)
        return self.curvatures[seg_idx]