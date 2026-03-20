# erp42_racing_planning/nodes/scene_builder_node.py
from __future__ import annotations

from typing import List, Tuple, Optional
import json

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseArray, Pose, PointStamped
from std_msgs.msg import String
from obstacle_tracking.msg import Track2DArray

from erp42_racing_planning.core.reference_path import ReferencePath
from erp42_racing_planning.core.planner_types import (
    Waypoint,
    Obstacle,
)



# -----------------------------
# 인지에서 받은 장애물 1개를 저장하는 구조체
# -----------------------------

class SceneBuilderNode(Node):

    def __init__(self):
        super().__init__("scene_builder_node")

        # 동작 파라미터
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("scene_forward_range_s", 30.0)
        self.declare_parameter("scene_rear_range_s", 30.0)

        # lane 관련 파라미터
        self.declare_parameter("lane_delta_d", 2.0)   # lane2 중심에서 lane1 중심까지의 d 간격

        # Internal state
        self.frame_id                                       = "map"
        self.reference_path     : Optional[ReferencePath]   = None
        
        self.right_lane         : Optional[List[Waypoint]]  = None
        self.left_lane          : Optional[List[Waypoint]]  = None

        self.have_odom          : bool                      = False
        self.ego_pos            : Tuple[float, float]       = (0.0, 0.0)
        self.ego_vel            : Tuple[float, float]       = (0.0, 0.0)

        self.obstacles          : List[Obstacle]            = []

        # Subscriber
        self.right_lane_sub         = self.create_subscription(Path, "/localization/right_lane", self._reference_path_callback, 10)
        self.left_lane_sub          = self.create_subscription(Path, "/localization/left_lane", self._overtake_path_callback, 10)
        self.ego_state_sub          = self.create_subscription(Odometry, "/perception/ego_state", self._ego_state_callback, 10)
        self.obstacle_state_sub     = self.create_subscription(Track2DArray, "/perception/obstacles", self._obstacle_state_callback, 10)

        # Publisher 
        self.ego_frenet_state_pub   = self.create_publisher(PointStamped, "/planning/ego_frenet_state", 10)
        self.behavior_scene_pub     = self.create_publisher(String, "/planning/behavior_scene", 10)

        # Timer
        self.create_timer(0.1, self.on_timer)

        self.get_logger().info("[scene_builder_node] Started")


    # ======================================================================
    # Callback
    # ======================================================================
    def _reference_path_callback(self, msg: Path) -> None:
        if self.reference_path is not None:
            return

        if not msg.poses:
            self.get_logger().warn("[scene_builder_node] Received empty global path")
            return

        waypoints = []
        for pose_stamped in msg.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            waypoints.append(Waypoint(x=x, y=y))

        if len(waypoints) < 2:
            self.get_logger().warn("[scene_builder_node] Global path waypoints < 2")
            return

        self.right_lane     = waypoints
        self.reference_path = ReferencePath(self.right_lane)

        self.get_logger().info(f"[scene_builder_node] ReferencePath is created: {len(self.right_lane)} points")


    def _overtake_path_callback(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn("[scene_builder_node] Received empty overtake path")
            return

        waypoints: List[Waypoint] = []
        for pose_stamped in msg.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            waypoints.append(Waypoint(x=x, y=y))

        if len(waypoints) < 2:
            self.get_logger().warn("[scene_builder_node] Overtake path waypoints < 2")
            return

        self.left_lane = waypoints


    def _ego_state_callback(self, msg: Odometry):
        # ego 차량의 위치/속도 저장
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)

        self.ego_pos    = (x, y) 
        self.ego_vel    = (vx, vy)
        self.have_odom  = True


    def _obstacle_state_callback(self, msg: Track2DArray) -> None:
        obstacles: List[Obstacle] = []

        for trk in msg.tracks:
            # -----------------------------
            # planning에서는 map 기준 절대값 사용
            # -----------------------------
            x = float(trk.map_pose.position.x)
            y = float(trk.map_pose.position.y)

            vx = float(trk.map_twist.linear.x)
            vy = float(trk.map_twist.linear.y)

            # planner_types.Obstacle 구조에 맞게 생성
            obs = Obstacle(
                x=x,
                y=y,
                vx=vx,
                vy=vy,
            )

            obstacles.append(obs)

        self.obstacles = obstacles    


    def lane_id_from_d(self, d: float) -> int:
        lane_delta = float(self.get_parameter("lane_delta_d").value)

        lane2_center = + lane_delta
        lane1_center = 0.0

        if abs(d - lane1_center) < abs(d - lane2_center):
            return 1
        else:
            return 2
        


    # ======================================================================
    # Timer loop
    # ======================================================================
    def on_timer(self):
        if (self.reference_path is None) or (not self.have_odom):
            return

        now                     = self.get_clock().now().to_msg()
        ref                     = self.reference_path
        scene_forward_range_s   = float(self.get_parameter("scene_forward_range_s").value)
        scene_rear_range_s      = float(self.get_parameter("scene_rear_range_s").value)

        # ego frenet state
        # ego_frenet_state    = ref.to_frenet(self.ego_pos)

        # s_ego               = float(ego_frenet_state.s)
        # d_ego               = float(ego_frenet_state.d)
        
        # x, y        = self.ego_pos
        # proj        = ref.project(x, y)
        # vx, vy      = self.ego_vel
        # s_dot_ego   = vx * proj.tx + vy * proj.ty

        x, y = self.ego_pos
        vx, vy = self.ego_vel

        # ego Frenet state
        ego_frenet_state = ref.to_frenet(x, y)
        s_ego = float(ego_frenet_state.s)
        d_ego = float(ego_frenet_state.d)

        # ego longitudinal speed along reference path
        proj = ref.project(x, y)
        s_dot_ego = max(0.0, float(vx * proj.tx + vy * proj.ty))

        self.get_logger().info(f"vx_ego={vx:.3f}, vy_ego={vy:.3f}, s_dot_ego={s_dot_ego:.3f}")

        ego_frenet_state_msg                    = PointStamped()
        ego_frenet_state_msg.header.stamp       = now
        ego_frenet_state_msg.header.frame_id    = self.frame_id
        ego_frenet_state_msg.point.x            = s_ego
        ego_frenet_state_msg.point.y            = d_ego
        ego_frenet_state_msg.point.z            = s_dot_ego

        self.ego_frenet_state_pub.publish(ego_frenet_state_msg)

        # obstacles frenet state
        ego_lane_id = self.lane_id_from_d(d_ego)
        obstacles   = []
    
        for obstacle in self.obstacles:
            obstacle_frenet_state   = ref.to_frenet(obstacle.x, obstacle.y)
            s_obs                   = float(obstacle_frenet_state.s)
            d_obs                   = float(obstacle_frenet_state.d)

            # target 속도 계산
            proj        = ref.project(obstacle.x, obstacle.y)
            s_dot_obs   = max(0.0 ,float((obstacle.vx * proj.tx + obstacle.vy * proj.ty)))

            # ego 기준 signed difference 비슷하게 앞/뒤 모두 포함하도록 거리 계산
            ds_forward  = ref.longitudinal_distance(s_ego, s_obs)   # ego -> obj
            ds_backward = ref.longitudinal_distance(s_obs, s_ego)  # obj -> ego
            
            # 앞쪽 scene_range_s 이내 또는 뒤쪽 scene_range_s 이내면 scene에 포함
            is_in_forward_range = (0.0 < ds_forward <= scene_forward_range_s)
            is_in_rear_range = (0.0 < ds_backward <= scene_rear_range_s)

            if not (is_in_forward_range or is_in_rear_range):
                continue

            lane_id = self.lane_id_from_d(d_obs)

            obstacles.append({
                "s": s_obs,
                "s_dot": s_dot_obs,
                "d": d_obs,
                "lane_id": lane_id,
            })

        # behavior planner에 전달할 scene 구성
        behavior_scene = {
            "ego": {
                "s": float(s_ego),
                "s_dot": float(s_dot_ego),
                "d": float(d_ego),
                "lane_id": int(ego_lane_id),
            },
            "obstacles": obstacles
        }

        behavior_scene_msg = String()
        behavior_scene_msg.data = json.dumps(behavior_scene)
        self.behavior_scene_pub.publish(behavior_scene_msg)

  

def main(args=None):
    rclpy.init(args=args)
    node = SceneBuilderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()