# erp42_racing_planning/utils/math_utils.py
from __future__ import annotations

import math


# ---------------------------------------------------------------------
# Basic helpers
# ---------------------------------------------------------------------

def clamp(x: float, lo: float, hi: float) -> float:
    """
    Clamp x into [lo, hi].
    """
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, x))


def lerp(a: float, b: float, t: float) -> float:
    """
    Linear interpolation.
    """
    return a + (b - a) * t


def safe_div(numer: float, denom: float, default: float = 0.0, eps: float = 1e-12) -> float:
    """
    Safe division.
    """
    return default if abs(denom) <= eps else numer / denom


# ---------------------------------------------------------------------
# Angle / track utilities
# ---------------------------------------------------------------------

def wrap_to_pi(angle: float) -> float:
    """
    Wrap angle (rad) into [-pi, pi).
    """
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return -math.pi if wrapped == math.pi else wrapped


def wrap_s(s: float, s_total: float) -> float:
    """
    Wrap arc-length s for a closed-loop track into [0, s_total).
    """
    if s_total <= 0.0:
        return 0.0
    return s % s_total


# ---------------------------------------------------------------------
# Related curvature
# ---------------------------------------------------------------------

def calc_curvature_based_speed(
    kappa: float,
    v_max: float,
    v_min: float,
    kappa_low: float,
    kappa_high: float,
) -> float:
    """
    Piecewise-linear speed profile based on curvature magnitude.

    abs(kappa) <= kappa_low   -> v_max
    abs(kappa) >= kappa_high  -> v_min
    otherwise                 -> linear interpolation
    """

    abs_kappa = abs(kappa)

    if kappa_low >= kappa_high:
        return v_min

    if abs_kappa <= kappa_low:
        return v_max

    if abs_kappa >= kappa_high:
        return v_min

    ratio = (abs_kappa - kappa_low) / (kappa_high - kappa_low)
    v = v_max - ratio * (v_max - v_min)

    return clamp(v, v_min, v_max)