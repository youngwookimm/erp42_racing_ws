# erp42_racing_planning/core/planner_types.py
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

from ..utils.quartic_polynomial import QuarticPolynomial
from ..utils.quintic_polynomial import QuinticPolynomial


# ==========================================================
# Basic geometry / path types
# ==========================================================

@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float


@dataclass(frozen=True)
class FrenetCoord:
    s: float
    d: float


@dataclass(frozen=True)
class ProjectionResult:
    segment_index: int
    segment_t: float
    s: float
    d: float
    x_proj: float
    y_proj: float
    tx: float
    ty: float


# ==========================================================
# Planner scene state types
# ==========================================================

class BehaviorMode(str, Enum):
    CRUISE = "CRUISE"      # velocity keeping
    FOLLOW = "FOLLOW"      # following
    OVERTAKE = "OVERTAKE"  # overtaking


@dataclass(frozen=True)
class FrenetState:
    # longitudinal initial state S0 = [s0, s_dot0, s_ddot0]
    s: float
    s_dot: float
    s_ddot: float

    # lateral initial state D0 = [d0, d_dot0, d_ddot0]
    d: float
    d_dot: float
    d_ddot: float


@dataclass(frozen=True)
class LeadingVehicleState:
    s: float
    s_dot: float
    s_ddot: float = 0.0


@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    vx: float
    vy: float = 0.0

@dataclass(frozen=True)
class EgoState:
    s: float
    s_dot: float
    d: float
    lane_id: int


@dataclass(frozen=True)
class ObstacleState:
    s: float
    s_dot: float
    d: float
    lane_id: int


# ==========================================================
# Trajectory types
# ==========================================================

@dataclass
class LateralCandidate:
    T: float
    poly: QuinticPolynomial
    d_target: float


@dataclass
class LongitudinalCandidate:
    T: float
    poly: QuarticPolynomial | QuinticPolynomial
    s_d: Optional[float] = None
    s_dot_d: Optional[float] = None


@dataclass
class FrenetSample:
    t: float

    s: float
    s_dot: float
    s_ddot: float
    s_jerk: float

    d: float
    d_dot: float
    d_ddot: float
    d_jerk: float


@dataclass(frozen=True)
class TrajectoryPoint:
    t: float

    x: float
    y: float
    yaw: float

    v: float
    a: float
    kappa: float
    
    s: float
    d: float


@dataclass
class LocalTrajectory:
    mode: BehaviorMode
    T: float

    # source candidates
    lat_cand: Optional[LateralCandidate] = None
    lon_cand: Optional[LongitudinalCandidate] = None

    # Frenet sample
    frenet_samples: List[FrenetSample] = field(default_factory=list)

    # cost terms
    C_d: float = 0.0
    C_t: float = 0.0
    C_v: float = 0.0
    C_lon: float = 0.0
    C_tot: float = 0.0

    # Cartesian trajectory
    points: List[TrajectoryPoint] = field(default_factory=list)

    # validity
    is_valid: bool = True
    invalid_reason: Optional[str] = None


@dataclass(frozen=True)
class SpeedProfileConfig:
    v_max: float = 5.5      # [m/s]
    v_min: float = 0.0      # [m/s]


@dataclass(frozen=True)
class LocalTrajectoryPlannerConfig:
    dt: float = 0.1
    T_candidates: tuple[float, ...] = (2.0, 3.0, 4.0)
    delta_v_candidates: tuple[float, ...] = (-0.5, 0.0, 0.5)

    lane_width: float = 2.0     # [m]
    # low_speed_threshold: float = 3.0

    speed_profile: SpeedProfileConfig = field(default_factory=SpeedProfileConfig)

    # cost weights
    k_j: float = 0.1
    k_t: float = 0.3
    k_d: float = 1.0
    k_s: float = 1.0
    k_s_dot: float = 1.0
    k_lat: float = 1.0
    k_lon: float = 1.0

    # safety / feasibility
    follow_time_gap: float = 1.0
    follow_distance_offset: float = 2.0
    max_curvature: float = 0.35
    vehicle_radius: float = 0.8