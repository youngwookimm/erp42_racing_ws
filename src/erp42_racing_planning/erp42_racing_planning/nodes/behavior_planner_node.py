#behavior_planner_node.py
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path

from erp42_racing_planning.core.reference_path import ReferencePath
from erp42_racing_planning.core.planner_types import (
    Waypoint,
    BehaviorMode,
    EgoState,
    ObstacleState,
)


# behavior planner가 최종적으로 내리는 의사결정 결과 
@dataclass
class BehaviorDecision:
    behavior: BehaviorMode
    gap: float = 999.0
    ttc: float = 999.0
    reason: str = ""
    has_lead: bool = False
    lead_s: float = 0.0
    lead_v: float = 0.0




# --------------------------------------------------
# behavior planner node
# --------------------------------------------------
class BehaviorPlannerNode(Node):
    def __init__(self):
        super().__init__("behavior_planner_node")

        # -------------------------
        # parameters
        # -------------------------
        self.declare_parameter("scene_topic", "/planning/behavior_scene")
        self.declare_parameter("ref_path_topic", "/localization/right_lane")
        self.declare_parameter("behavior_mode_topic", "/planning/behavior_mode")
        self.declare_parameter("leading_vehicle_topic", "/planning/lv_frenet_state")
        self.declare_parameter("reference_lane_id", 2) # 기본 복귀 차선
        self.declare_parameter("overtake_lane_id", 1)
       

        # 선행 차량이 이 거리 안이면 Overtake를 더 적극적으로 고려
        self.declare_parameter("overtake_distance_threshold", 15.0)

        # TTC가 이 값 이하이면 Overtake 고려
        self.declare_parameter("ttc_overtake_threshold", 3.0)

        self.declare_parameter("return_check_distance", 30.0)

        # lane change safety
        # target lane으로 차선 변경할 때 필요한 안전 여유
        # 목표 차선 앞 차량과 이 정도 이상 떨어져 있어야 안전
        self.declare_parameter("lane_change_front_margin", 12.0)
        
        # 목표 차선 뒤 차량과 이 정도 이상 떨어져 있어야 안전
        self.declare_parameter("lane_change_rear_margin", 10.0)

        # publish rate
        self.declare_parameter("rate_hz", 10.0)

        # no-overtake zone on closed-loop track
        # 이 s 구간에서는 차선이 1개로 합쳐지므로 추월 금지
        self.declare_parameter("no_overtake_zones", [78.0, 138.0, 190.0, 300.0])


        # -------------------------
        # 내부 상태 변수
        # -------------------------
        self.ref_path: Optional[ReferencePath] = None
        self.latest_scene: Optional[Dict] = None
        self.last_decision: Optional[BehaviorDecision] = None

        scene_topic = str(self.get_parameter("scene_topic").value)
        ref_topic = str(self.get_parameter("ref_path_topic").value)

        # -------------------------
        # subscribers
        # -------------------------
        self.create_subscription(String, scene_topic, self.scene_cb, 10)
        self.create_subscription(Path, ref_topic, self.ref_cb, 10)

        # -------------------------
        # publishers
        # -------------------------
        behavior_mode_topic = str(self.get_parameter("behavior_mode_topic").value)
        leading_vehicle_topic = str(self.get_parameter("leading_vehicle_topic").value)

        self.pub_behavior_mode = self.create_publisher(String, behavior_mode_topic, 10)
        self.pub_leading_vehicle = self.create_publisher(String, leading_vehicle_topic, 10)

        # -------------------------
        # timer
        # -------------------------
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate_hz, 1e-6), self.on_timer)

        self.get_logger().info(
            f"[behavior_planner] sub scene={scene_topic}, ref={ref_topic}, "
            f"pub_mode={behavior_mode_topic}, pub_lead={leading_vehicle_topic}"
        )

    # --------------------------------------------------
    # callback: reference path 수신
    # --------------------------------------------------
    def ref_cb(self, msg: Path) -> None:
        if len(msg.poses) < 2:
            self.get_logger().warn("[behavior_planner] ref path has <2 poses. ignore.")
            return

        try:
            wps = [
                Waypoint(
                    x=float(ps.pose.position.x),
                    y=float(ps.pose.position.y),
                )
                for ps in msg.poses
            ]
            self.ref_path = ReferencePath(wps)
        except Exception as e:
            self.ref_path = None
            self.get_logger().error(f"[behavior_planner] failed to build ref path: {e}")

    # callback: scene 수신
    def scene_cb(self, msg: String) -> None:
        try:
            self.latest_scene = json.loads(msg.data)
        except Exception as e:
            self.latest_scene = None
            self.get_logger().error(f"[behavior_planner] failed to parse scene json: {e}")

    # --------------------------------------------------
    # helper : scene dict -> EgoState, ObstacleState 로 파싱
    # --------------------------------------------------
    def parse_scene(self, scene: Dict) -> Tuple[EgoState, List[ObstacleState]]:
        ego_raw = scene["ego"]
        ego = EgoState(
            s=float(ego_raw["s"]),
            s_dot=float(ego_raw["s_dot"]),
            d=float(ego_raw["d"]),
            lane_id=int(ego_raw["lane_id"]),
        )

        obstacles : List[ObstacleState] = []
        for obj in scene.get("obstacles", []):
            obstacles.append(
                ObstacleState(
                    s=float(obj["s"]),
                    s_dot=float(obj["s_dot"]),
                    d=float(obj["d"]),
                    lane_id=int(obj["lane_id"]),
                )
            )

        return ego, obstacles

    # --------------------------------------------------
    # helper: Frenet s 기준 전방 거리 계산
    # --------------------------------------------------
    def forward_distance(self, s_from: float, s_to: float) -> float:
        if self.ref_path is None:
            diff = s_to - s_from
            if diff < 0.0:
                return 9999.0
            return diff
        return self.ref_path.longitudinal_distance(s_from, s_to)

    # --------------------------------------------------
    # helper: 같은 차선에서 가장 가까운 선행 차량 찾기
    # --------------------------------------------------
    def find_front_vehicle_in_lane(
        self,
        ego_s: float,
        lane_id: int,
        obstacles: List[ObstacleState],
        lookahead_distance: float = 30.0,
    ) -> Tuple[Optional[ObstacleState], float]:
        lead: Optional[ObstacleState] = None
        min_gap = float("inf")

        for obj in obstacles:
            if obj.lane_id != lane_id:
                continue

            gap = self.forward_distance(ego_s, obj.s)

            if gap <= 1e-3:
                continue
            if gap > lookahead_distance:
                continue

            if gap < min_gap:
                min_gap = gap
                lead = obj

        return lead, min_gap
    
    # --------------------------------------------------
    # helper
    # --------------------------------------------------
    def find_rear_vehicle_in_lane(
        self,
        ego_s: float,
        lane_id: int,
        obstacles: List[ObstacleState],
        lookbehind_distance: float = 30.0,
    ) -> Tuple[Optional[ObstacleState], float]:
        rear: Optional[ObstacleState] = None
        min_gap = float("inf")

        for obj in obstacles:
            if obj.lane_id != lane_id:
                continue

            gap = self.forward_distance(obj.s, ego_s)

            if gap <= 1e-3:
                continue
            if gap > lookbehind_distance:
                continue

            if gap < min_gap:
                min_gap = gap
                rear = obj

        return rear, min_gap

    # --------------------------------------------------
    # helper: TTC 계산
    # --------------------------------------------------
    def compute_ttc(self, ego_s_dot: float, obj_s_dot: float, gap: float) -> float:
        rel_s_dot = ego_s_dot - obj_s_dot
        if rel_s_dot <= 1e-3:
            return float("inf")
        return gap / rel_s_dot

    # --------------------------------------------------
    # helper: overtake safety check
    # 현재 차선 + 타겟 차선 검사
    # --------------------------------------------------
    def is_overtake_safe(
        self,
        ego: EgoState,
        obstacles: List[ObstacleState],
        target_lane: int,
    ) -> bool:

        front_margin = float(self.get_parameter("lane_change_front_margin").value)
        rear_margin = float(self.get_parameter("lane_change_rear_margin").value)

        for obj in obstacles:
            if obj.lane_id != target_lane:
                continue

            gap_front = self.forward_distance(ego.s, obj.s)
            gap_rear = self.forward_distance(obj.s, ego.s)

            # target lane 앞차 너무 가까움
            if gap_front > 1e-3 and gap_front <= front_margin:
                return False

            # target lane 뒤차 너무 가까움
            if gap_rear > 1e-3 and gap_rear <= rear_margin:
                return False

        return True

    # --------------------------------------------------
    # helper: 현재 ego s가 추월 금지 구간인지 확인
    # --------------------------------------------------
    def is_no_overtake_zone(self, s_ego: float) -> bool:
        zones = list(self.get_parameter("no_overtake_zones").value)

        if len(zones) % 2 != 0:
            self.get_logger().warn(
                "[behavior_planner] no_overtake_zones must have even length: [s1, e1, s2, e2, ...]"
            )
            return False

        for i in range(0, len(zones), 2):
            s_start = float(zones[i])
            s_end = float(zones[i + 1])

            if s_start <= s_ego <= s_end:
                return True

        return False
    

    def decide_behavior(
        self,
        ego: EgoState,
        obstacles: List[ObstacleState],
    ) -> BehaviorDecision:
        
        # parameters
        overtake_distance = float(self.get_parameter("overtake_distance_threshold").value)
        overtake_ttc_threshold = float(self.get_parameter("ttc_overtake_threshold").value)
        reference_lane = int(self.get_parameter("reference_lane_id").value)
        overtake_lane = int(self.get_parameter("overtake_lane_id").value)
        return_check_distance = float(self.get_parameter("return_check_distance").value)
        return_front_margin = float(self.get_parameter("lane_change_front_margin").value)
        return_rear_margin = float(self.get_parameter("lane_change_rear_margin").value)

        # 0) 현재 차선 lead vehicle 찾기
        lead_vehicle, gap = self.find_front_vehicle_in_lane(
            ego_s=ego.s,
            lane_id=reference_lane,
            obstacles=obstacles,
            lookahead_distance=return_check_distance,
        )
            
        ttc = float("inf")
        if lead_vehicle is not None:
            ttc = self.compute_ttc(ego.s_dot, lead_vehicle.s_dot, gap)
        
        # ==================================================
        # 1) 추월 금지 구간
        #    - 앞차 있으면 FOLLOW
        #    - 앞차 없으면 CRUISE
        # ==================================================
        # if self.is_no_overtake_zone(ego.s):
        #     if lead_vehicle is not None:
        #         return BehaviorDecision(
        #             behavior=BehaviorMode.FOLLOW,
        #             gap=gap,
        #             ttc=ttc,
        #             reason="FOLLOW: no-overtake zone with leading vehicle",
        #             has_lead=True,
        #             lead_s=lead_vehicle.s,
        #             lead_v=lead_vehicle.s_dot,
        #         )
        #     else:
        #         return BehaviorDecision(
        #             behavior=BehaviorMode.CRUISE,
        #             gap=float("inf"),
        #             ttc=float("inf"),
        #             reason="CRUISE: no-overtake zone and no leading vehicle",
        #             has_lead=False,
        #             lead_s=0.0,
        #             lead_v=0.0,
        #         )
        # ==================================================
        # 2) 추월 가능 구간 + ego가 기준 차선(reference lane)에 있음
        #    - 앞차 없으면 CRUISE
        #    - 앞차 있으면 TTC / 거리 기준으로 OVERTAKE or FOLLOW
        # ==================================================
        if ego.lane_id == reference_lane:
            if lead_vehicle is None:
                return BehaviorDecision(
                    behavior=BehaviorMode.CRUISE,
                    gap=float("inf"),
                    ttc=float("inf"),
                    reason="CRUISE: on reference lane and no leading vehicle",
                    has_lead=False,
                    lead_s=0.0,
                    lead_v=0.0,
                )
            lead_close = (gap < overtake_distance) or (ttc < overtake_ttc_threshold)
            
            if lead_close:
                if self.is_overtake_safe(ego,obstacles, overtake_lane):
                    return BehaviorDecision(
                        behavior=BehaviorMode.OVERTAKE,
                        gap=gap,
                        ttc=ttc,
                        reason=f"OVERTAKE: move to lane {overtake_lane}",
                        has_lead=True,
                        lead_s=lead_vehicle.s,
                        lead_v=lead_vehicle.s_dot,
                    )
                else:
                    return BehaviorDecision(
                        behavior=BehaviorMode.FOLLOW,
                        gap=gap,
                        ttc=ttc,
                        reason="FOLLOW: overtake lane not safe",
                        has_lead=True,
                        lead_s=lead_vehicle.s,
                        lead_v=lead_vehicle.s_dot,
                    )

            return BehaviorDecision(
                behavior=BehaviorMode.FOLLOW,
                gap=gap,
                ttc=ttc,
                reason="FOLLOW: keep distance on reference lane",
                has_lead=True,
                lead_s=lead_vehicle.s,
                lead_v=lead_vehicle.s_dot,
            )
        # ==================================================
        # 3) 추월 가능 구간 + ego가 추월 차선(overtake lane)에 있음
        #    - reference lane 앞/뒤가 비어 있으면 CRUISE(복귀)
        #    - 아니면 OVERTAKE 유지
        # ==================================================
        else: 
            ref_front_vehicle, ref_front_gap = self.find_front_vehicle_in_lane(
                ego_s=ego.s,
                lane_id=reference_lane,
                obstacles=obstacles,
                lookahead_distance=return_check_distance,
            )
            
            ref_rear_vehicle, ref_rear_gap = self.find_rear_vehicle_in_lane(
                ego_s=ego.s,
                lane_id=reference_lane,
                obstacles=obstacles,
                lookbehind_distance=return_check_distance,
            )
            
            ref_front_clear = (
                ref_front_vehicle is None or ref_front_gap >= return_front_margin
            )
            ref_rear_clear = (
                ref_rear_vehicle is None or ref_rear_gap >= return_rear_margin
            )

            if ref_front_clear and ref_rear_clear :
                return BehaviorDecision(
                    behavior=BehaviorMode.CRUISE,
                    gap=ref_front_gap if ref_front_vehicle is not None else float("inf"),
                    ttc=float("inf"),
                    reason=f"CRUISE: return to reference lane {reference_lane}",
                    has_lead=False,
                    lead_s=0.0,
                    lead_v=0.0,
                )

            return BehaviorDecision(
                behavior=BehaviorMode.OVERTAKE,
                gap=ref_front_gap if ref_front_vehicle is not None else float("inf"),
                ttc=float("inf"),
                reason=f"OVERTAKE: keep lane {overtake_lane}, reference lane blocked",
                has_lead=False,
                lead_s=0.0,
                lead_v=0.0,
            )


    # --------------------------------------------------
    # timer
    # --------------------------------------------------
    def on_timer(self) -> None:
        if self.latest_scene is None:
            return

        try:
            ego, obstacles = self.parse_scene(self.latest_scene)
            decision = self.decide_behavior(ego, obstacles)
            self.last_decision = decision


            mode_msg = String()
            mode_msg.data = decision.behavior.value
            self.pub_behavior_mode.publish(mode_msg)

            lead_msg = String()

            if decision.has_lead:
                lead_payload = {
                    "s": float(decision.lead_s),
                    "s_dot": float(decision.lead_v),
                    "s_ddot": 0.0,
                }
                lead_msg.data = json.dumps(lead_payload)
            else:
                lead_msg.data = ""

            self.pub_leading_vehicle.publish(lead_msg)

            ttc_log = float(decision.ttc) if math.isfinite(decision.ttc) else 9999.0

            self.get_logger().info(
                f"[behavior] mode={decision.behavior.value} "
                f"gap={decision.gap:.2f} "
                f"ttc={ttc_log:.2f} "
                f"reason={decision.reason}"
            )
        except Exception as e:
            self.get_logger().error(f"[behavior_planner] on_timer failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()