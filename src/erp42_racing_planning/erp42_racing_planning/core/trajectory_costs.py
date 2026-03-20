# erp42_racing_planning/core/trajectory_costs.py
from __future__ import annotations

from .planner_types import LocalTrajectoryPlannerConfig


def integrate_squared(values: list[float], dt: float) -> float:
    
    if not values:
        return 0.0
    
    result = sum(v * v for v in values) * dt

    return result


def calc_lateral_cost(
    d_jerk_samples: list[float],
    T: float,
    d1: float,
    config: LocalTrajectoryPlannerConfig,
) -> float:
    
    J_d = integrate_squared(d_jerk_samples, config.dt)
    C_lat = config.k_j * J_d + config.k_t * T + config.k_d * (d1 ** 2)

    return C_lat


def calc_longitudinal_tracking_cost(
    s_jerk_samples: list[float],
    T: float,
    s1: float,
    s_d: float,
    config: LocalTrajectoryPlannerConfig,
) -> float:
    
    J_s = integrate_squared(s_jerk_samples, config.dt)
    C_lon = config.k_j * J_s + config.k_t * T + config.k_s * ((s1 - s_d) ** 2)

    return C_lon


def calc_longitudinal_velocity_cost(
    s_jerk_samples: list[float],
    T: float,
    s1_dot: float,
    s_dot_d: float,
    config: LocalTrajectoryPlannerConfig,
) -> float:
    
    J_s = integrate_squared(s_jerk_samples, config.dt)
    C_lon = config.k_j * J_s + config.k_t * T + config.k_s_dot * ((s1_dot - s_dot_d) ** 2)

    return C_lon


def calc_total_cost(C_lat: float, C_lon: float, config: LocalTrajectoryPlannerConfig) -> float:
    
    return config.k_lat * C_lat + config.k_lon * C_lon