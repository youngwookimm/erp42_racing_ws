# erp42_racing_planning/core/trajectory_planner.py
from __future__ import annotations

from typing import List, Optional, Sequence
import math

from .reference_path import ReferencePath
from .planner_types import (
    BehaviorMode,
    FrenetState,
    LeadingVehicleState,
    Obstacle,
    LateralCandidate,
    LongitudinalCandidate,
    FrenetSample,
    TrajectoryPoint,
    LocalTrajectory,
    LocalTrajectoryPlannerConfig,
)
from .trajectory_costs import (
    calc_lateral_cost,
    calc_longitudinal_tracking_cost,
    calc_longitudinal_velocity_cost,
    calc_total_cost,
)
from ..utils.quartic_polynomial import QuarticPolynomial
from ..utils.quintic_polynomial import QuinticPolynomial
from ..utils.math_utils import clamp, wrap_s, wrap_to_pi, calc_curvature_based_speed


class TrajectoryPlanner:
    """
    Assumed ReferencePath interface:
        - ref_path.s_total : float
        - ref_path.calc_position_yaw_curvature(s) -> tuple[x, y, yaw, curvature]
    """

    def __init__(self, ref_path: ReferencePath, config: LocalTrajectoryPlannerConfig):
        
        # Parameters
        self.ref_path   = ref_path
        self.config     = config

        # Debug (visualization)
        self.last_all_candidates: List[LocalTrajectory]     = []
        self.last_valid_candidates: List[LocalTrajectory]   = []


    def generate_trajectory( 
        self, 
        frenet_state: FrenetState, 
        mode: BehaviorMode, 
        leading_vehicle: Optional[LeadingVehicleState] = None,
        obstacles: Optional[Sequence[Obstacle]] = None,
    ) -> Optional[LocalTrajectory]:
        
        # initialize candidates
        self.last_all_candidates    = []
        self.last_valid_candidates  = []
 
        # generate lateral set
        lateral_set = self._generate_lateral_set(frenet_state, mode)
        if not lateral_set: 
            return None

        # generate longitudinal set
        longitudinal_set = self._generate_longitudinal_set(frenet_state, mode, leading_vehicle)
        if not longitudinal_set: 
            return None

        # combine trajectory set
        candidates = self._combine_trajectory_set(lateral_set, longitudinal_set, mode)
        if not candidates: 
            return None
        
        # trajectory candidates
        for cand in candidates:

            # Frent trajectory
            self._sample_frenet_trajectory(cand)

            # cost calculation
            self._calculate_trajectory_cost(cand)

            # from Frenet to Cartesian
            self._convert_trajectory_to_cartesian(cand)

            # feasibility check & collision check
            self._check_trajectory_feasibility(cand, obstacles)
    
        self.last_all_candidates = candidates

        valid_candidates = [c for c in candidates if c.is_valid]
        if not valid_candidates: return None
        self.last_valid_candidates = valid_candidates

        # best trajectory selection
        local_trajectory = min(valid_candidates, key=lambda c: c.C_tot)

        return local_trajectory


    def _generate_lateral_set(
        self,
        frenet_state: FrenetState,
        mode: BehaviorMode,
    ) -> List[LateralCandidate]:
        """
        Generate lateral candidate polynomials d(t).
        """

        d0 = frenet_state.d
        d0_dot = frenet_state.d_dot
        d0_ddot = frenet_state.d_ddot

        lateral_set: List[LateralCandidate] = []

        if mode == BehaviorMode.OVERTAKE:
            d1_candidates = [self.config.lane_width]
        else:
            d1_candidates = [0.0]

        for T in self.config.T_candidates:
            for d1 in d1_candidates:

                lateral_poly = QuinticPolynomial(d0, d0_dot, d0_ddot, d1, 0.0, 0.0, T)

                lateral_set.append(LateralCandidate(T, lateral_poly, d1))

        return lateral_set


    def _generate_longitudinal_set(
        self,
        frenet_state: FrenetState,
        mode: BehaviorMode,
        leading_vehicle: Optional[LeadingVehicleState],
    ) -> List[LongitudinalCandidate]:
        """
        CRUISE / OVERTAKE -> QuarticPolynomial
        FOLLOW            -> QuinticPolynomial
        """
        
        s0 = frenet_state.s
        s0_dot = frenet_state.s_dot
        s0_ddot = frenet_state.s_ddot

        longitudinal_set: List[LongitudinalCandidate] = []

        if mode == BehaviorMode.CRUISE:
            
            kappa_ref = self._estimate_reference_curvature(s0, lookahead_s=10.0)
            s_dot_d = calc_curvature_based_speed(
                        kappa=kappa_ref,
                        v_max=5.5,
                        v_min=5.0,
                        kappa_low=0.05,
                        kappa_high=0.20,
                    )   

            for T in self.config.T_candidates:
                for delta_v in self.config.delta_v_candidates:

                    s1_dot  = clamp(s_dot_d + delta_v, self.config.speed_profile.v_min, self.config.speed_profile.v_max)
                    s1_ddot = 0.0
                    
                    longitudinal_poly = QuarticPolynomial(s0, s0_dot, s0_ddot, s1_dot, s1_ddot, T)
                    
                    longitudinal_set.append(LongitudinalCandidate(T=T, poly=longitudinal_poly, s_dot_d=s1_dot))

        elif mode == BehaviorMode.FOLLOW:
            
            if leading_vehicle is None:
                return []

            safety_margin = 5.0     # [m]

            for T in self.config.T_candidates:
                
                s1      = leading_vehicle.s + leading_vehicle.s_dot * T - safety_margin
                s1_dot  = leading_vehicle.s_dot
                s1_ddot = 0.0
                
                longitudinal_poly = QuinticPolynomial(s0, s0_dot, s0_ddot, s1, s1_dot, s1_ddot, T)
                
                longitudinal_set.append(LongitudinalCandidate(T=T, poly=longitudinal_poly, s_d=s1, s_dot_d=s1_dot))

        elif mode == BehaviorMode.OVERTAKE:

            kappa_ref = self._estimate_reference_curvature(s0, lookahead_s=10.0)
            s_dot_d = calc_curvature_based_speed(
                        kappa=kappa_ref,
                        v_max=5.5,
                        v_min=0.0,
                        kappa_low=0.05,
                        kappa_high=0.20,
                    )   

            for T in self.config.T_candidates:
                for delta_v in self.config.delta_v_candidates:

                    s1_dot = clamp(s_dot_d + delta_v, self.config.speed_profile.v_min, self.config.speed_profile.v_max)
                    s1_ddot = 0.0
                    
                    longitudinal_poly = QuarticPolynomial(s0, s0_dot, s0_ddot, s1_dot, s1_ddot, T)
                    
                    longitudinal_set.append(LongitudinalCandidate(T=T, poly=longitudinal_poly, s_dot_d=s1_dot))

        return longitudinal_set


    def _combine_trajectory_set(
        self,
        lateral_set: List[LateralCandidate],
        longitudinal_set: List[LongitudinalCandidate],
        mode: BehaviorMode,
    ) -> List[LocalTrajectory]:
        
        trajectory_candidates: List[LocalTrajectory] = []

        for lat_cand in lateral_set:
            for lon_cand in longitudinal_set:

                T = min(lat_cand.T, lon_cand.T)

                cand = LocalTrajectory(mode=mode, T=T)
                cand.lat_cand = lat_cand
                cand.lon_cand = lon_cand

                trajectory_candidates.append(cand)

        return trajectory_candidates
    

    def _sample_frenet_trajectory(self, cand: LocalTrajectory) -> None:

        cand.frenet_samples.clear()

        lat_poly = cand.lat_cand.poly
        lon_poly = cand.lon_cand.poly
        T = cand.T

        t = 0.0
        while t <= T + 1e-9:

            s = lon_poly.calc_point(t)
            s_dot = lon_poly.calc_first_derivative(t)
            s_ddot = lon_poly.calc_second_derivative(t)
            s_jerk = lon_poly.calc_third_derivative(t)

            d = lat_poly.calc_point(t)
            d_dot = lat_poly.calc_first_derivative(t)
            d_ddot = lat_poly.calc_second_derivative(t)
            d_jerk = lat_poly.calc_third_derivative(t)

            s_wrapped = wrap_s(s, self.ref_path.s_total)

            sample = FrenetSample(
                t=t,
                s=s_wrapped,
                s_dot=s_dot,
                s_ddot=s_ddot,
                s_jerk=s_jerk,
                d=d,
                d_dot=d_dot,
                d_ddot=d_ddot,
                d_jerk=d_jerk,
            )
            
            cand.frenet_samples.append(sample)

            t += self.config.dt


    def _calculate_trajectory_cost(self, cand: LocalTrajectory) -> None:

        d1 = cand.frenet_samples[-1].d
        s1 = cand.frenet_samples[-1].s
        s1_dot = cand.frenet_samples[-1].s_dot

        d_jerk_samples = [p.d_jerk for p in cand.frenet_samples]
        s_jerk_samples = [p.s_jerk for p in cand.frenet_samples]

        C_d = calc_lateral_cost(d_jerk_samples, cand.T, d1, self.config)

        if cand.mode == BehaviorMode.FOLLOW:
            s_d = cand.lon_cand.s_d
            C_lon = calc_longitudinal_tracking_cost(s_jerk_samples, cand.T, s1, s_d, self.config)
            cand.C_t = C_lon
            cand.C_v = 0.0

        else:  # CRUISE & OVERTAKE
            s_dot_d = cand.lon_cand.s_dot_d
            C_lon = calc_longitudinal_velocity_cost(
                s_jerk_samples=s_jerk_samples,
                T=cand.T,
                s1_dot=s1_dot,
                s_dot_d=s_dot_d,
                config=self.config,
            )
            cand.C_v = C_lon
            cand.C_t = 0.0

        cand.C_d = C_d
        cand.C_lon = C_lon
        cand.C_tot = calc_total_cost(C_d, C_lon, self.config)


    def _convert_trajectory_to_cartesian(self, cand: LocalTrajectory) -> None:
        
        cand.points.clear()

        for sample in cand.frenet_samples:

            s = sample.s
            d = sample.d
            s_dot = sample.s_dot
            d_dot = sample.d_dot
            s_ddot = sample.s_ddot
            d_ddot = sample.d_ddot

            waypoint = self.ref_path.from_frenet(s, d)
            x = waypoint.x
            y = waypoint.y

            yaw_r = self.ref_path.heading_at_s(s)
            yaw = yaw_r + math.atan2(d_dot, max(s_dot, 1e-6))
            yaw = wrap_to_pi(yaw)

            kappa_r = self.ref_path.curvature_at_s(s)

            v = math.sqrt(max(s_dot * s_dot + d_dot * d_dot, 0.0))
            a = math.sqrt(max(s_ddot * s_ddot + d_ddot * d_ddot, 0.0))

            point = TrajectoryPoint(
                t=sample.t,
                x=x,
                y=y,
                yaw=yaw,
                v=v,
                a=a,
                kappa=kappa_r,
                s=s,
                d=d,
            )

            cand.points.append(point)


    def _check_trajectory_feasibility(self, cand: LocalTrajectory, obstacles: Optional[Sequence[Obstacle]] = None) -> None:
        
        if not cand.points:
            cand.is_valid = False
            cand.invalid_reason = "empty_points"
            return

        safety_margin = 0.0

        for p in cand.points:

            # collision
            if obstacles is not None:
                for ob in obstacles:
                    dx = p.x - ob.x
                    dy = p.y - ob.y
                    dist_sq = dx * dx + dy * dy
                    limit_sq = (self.config.vehicle_radius + self.config.vehicle_radius + safety_margin) ** 2

                    if dist_sq <= limit_sq:
                        cand.is_valid = False
                        cand.invalid_reason = "collision"
                        return


            # # speed
            if p.v > self.config.speed_profile.v_max + 1e-6:
                cand.is_valid = False
                cand.invalid_reason = "speed_limit"
                return
            
    def _estimate_reference_curvature(self, s0: float, lookahead_s: float = 10.0) -> float:
        """
        Estimate representative reference curvature over a forward lookahead window.
        Here we use max abs curvature over [s0, s0 + lookahead_s].
        """

        sample_count = 10
        max_abs_kappa = 0.0

        for i in range(sample_count + 1):
            ds = lookahead_s * i / sample_count
            s = wrap_s(s0 + ds, self.ref_path.s_total)
            kappa = self.ref_path.curvature_at_s(s)
            max_abs_kappa = max(max_abs_kappa, abs(kappa))

        return max_abs_kappa