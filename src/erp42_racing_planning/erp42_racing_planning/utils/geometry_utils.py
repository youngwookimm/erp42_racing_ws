# erp42_racing_planning/utils/geometry_utils.py
from __future__ import annotations

import math
from typing import Tuple

from .math_utils import clamp

# ---------------------------------------------------------------------
# Projection
# ---------------------------------------------------------------------

def project_point_to_segment(
    px: float, py: float,
    x0: float, y0: float,
    x1: float, y1: float,
    eps: float = 1e-12,
    ) -> Tuple[float, float, float]:
    """
    Project a point P(px,py) onto the segment A(x0,y0)->B(x1,y1)
    """
    vx = x1 - x0
    vy = y1 - y0
    seg_len_sq = vx*vx + vy*vy

    # Degenerate segment (A == B)
    if seg_len_sq <= eps:
        return 0.0, x0, y0

    wx = px - x0
    wy = py - y0

    # Unclamped projection parameter
    t = (wx*vx + wy*vy) / seg_len_sq
    t = clamp(t, 0.0, 1.0)

    x_proj = x0 + t * vx
    y_proj = y0 + t * vy
    return t, x_proj, y_proj


# ---------------------------------------------------------------------
# Direction vectors
# ---------------------------------------------------------------------

def unit_tangent(
    x0: float, y0: float,
    x1: float, y1: float,
    fallback: Tuple[float, float] = (1.0, 0.0),
    eps: float = 1e-12,
    ) -> Tuple[float, float]:
    """
    Return unit tangent vector from (x0,y0) to (x1,y1).
    """
    dx = x1 - x0
    dy = y1 - y0
    L = math.hypot(dx, dy)
    if L <= eps:
        return fallback
    return dx / L, dy / L


def left_normal(tx: float, ty: float) -> Tuple[float, float]:
    """
    Return left normal vector of a unit tangent.
    """
    return -ty, tx


# ---------------------------------------------------------------------
# Signed lateral offset (Frenet d)
# ---------------------------------------------------------------------

def signed_lateral_offset(
    px: float, py: float,
    x_ref: float, y_ref: float,
    tx: float, ty: float,
    ) -> float:
    """
    Compute signed lateral offset (Frenet d) relative to a reference tangent.
    Positive means left of the tangent direction.
    """
    dx = px - x_ref
    dy = py - y_ref

    nx, ny = left_normal(tx, ty)
    return dx * nx + dy * ny