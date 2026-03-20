# erp42_racing_planning/nodes/local_planner_node.py
from __future__ import annotations

import json
from typing import List, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

from erp42_racing_planning.core.reference_path import ReferencePath
from erp42_racing_planning.core.trajectory_planner import TrajectoryPlanner
from erp42_racing_planning.core.planner_types import (
    Waypoint,
    BehaviorMode,
    FrenetState,
    LeadingVehicleState,
    Obstacle,
    LocalTrajectory,
    LocalTrajectoryPlannerConfig,
)
from erp42_racing_planning.utils.rviz_utils import (
    build_selected_marker_array,
    build_candidate_marker_array,
)

from obstacle_tracking.msg import Track2DArray

class LocalPlannerNode(Node):

    def __init__(self) -> None:
        super().__init__("local_planner_node")

        
        # Internal state
        self.has_logged_waiting_state                           = False
        self.frame_id                                           = "map"
        
        self.right_lane         : Optional[List[Waypoint]]      = None
        self.left_lane          : Optional[List[Waypoint]]      = None

        self.ref_path           : Optional[ReferencePath]       = None

        self.leading_vehicle    : Optional[LeadingVehicleState] = None
        self.behavior_mode      : Optional[BehaviorMode]        = None
        self.ego_frenet_state   : Optional[FrenetState]         = None

        self.leading_vehicle    : Optional[LeadingVehicleState] = None
        self.obstacles          : List[Obstacle]                = []

        self.config                                             = LocalTrajectoryPlannerConfig()
        self.trajectory_planner : Optional[TrajectoryPlanner]   = None
        
        # Previous trajectory reuse parameters
        self.previous_trajectory: Optional[LocalTrajectory]     = None
        self.reuse_d_threshold                                  = 0.30

        # Publisher
        self.local_path_pub         = self.create_publisher(Path, "/planning/local_path", 10)
        self.local_marker_pub       = self.create_publisher(MarkerArray, "/planning/local_markers", 10)
        self.candidate_marker_pub   = self.create_publisher(MarkerArray, "/planning/candidate_markers", 10)

        # Subscriber
        self.right_lane_sub         = self.create_subscription(Path, "/localization/right_lane", self._reference_path_callback, 10)
        self.left_lane_sub          = self.create_subscription(Path, "/localization/left_lane", self._overtake_path_callback, 10)
        
        self.ego_frenet_state_sub   = self.create_subscription(PointStamped, "/planning/ego_frenet_state", self._ego_frenet_state_callback, 10)

        self.lv_frenet_state_sub    = self.create_subscription(String, "/planning/lv_frenet_state", self._lv_frenet_state_callback, 10)
        self.behavior_mode_sub      = self.create_subscription(String, "/planning/behavior_mode", self._behavior_mode_callback, 10)
  
        self.obstacles_sub          = self.create_subscription(Track2DArray, "/perception/obstacles", self._obstacle_state_callback, 10)

        # Timer
        self.timer = self.create_timer(0.1, self._on_timer)  # 10 Hz

        self.get_logger().info("[local_planner_node] Started")
    
    
    # ======================================================================
    # Subscriber callbacks
    # ======================================================================

    def _reference_path_callback(self, msg: Path) -> None:
        if self.ref_path is not None:
            return

        if not msg.poses:
            self.get_logger().warn("[local_planner_node] Received empty global path")
            return

        waypoints = []
        for pose_stamped in msg.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            waypoints.append(Waypoint(x=x, y=y))

        if len(waypoints) < 2:
            self.get_logger().warn("[local_planner_node] Global path waypoints < 2")
            return

        self.right_lane         = waypoints
        self.ref_path           = ReferencePath(self.right_lane)
        self.trajectory_planner = TrajectoryPlanner(self.ref_path, self.config)

        self.get_logger().info(f"[local_planner_node] ReferencePath is created: {len(self.right_lane)} points")

    
    def _overtake_path_callback(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn("[local_planner_node] Received empty overtake path")
            return

        waypoints: List[Waypoint] = []
        for pose_stamped in msg.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            waypoints.append(Waypoint(x=x, y=y))

        if len(waypoints) < 2:
            self.get_logger().warn("[local_planner_node] Overtake path waypoints < 2")
            return

        self.left_lane = waypoints


    def _behavior_mode_callback(self, msg: String) -> None:
        try:
            self.behavior_mode = BehaviorMode[msg.data.strip().upper()]
        except KeyError:
            self.get_logger().warn(f"[local_planner_node] Invalid behavior mode received: {msg.data}")


    def _ego_frenet_state_callback(self, msg: PointStamped) -> None:
        self.ego_frenet_state = FrenetState(
            s=float(msg.point.x),
            s_dot=float(msg.point.z),
            s_ddot=0.0,
            d=float(msg.point.y),
            d_dot=0.0,
            d_ddot=0.0,
        )


    def _lv_frenet_state_callback(self, msg: String) -> None:
        text = msg.data.strip()

        if text == "":
            self.leading_vehicle = None
            return

        try:
            data = json.loads(text)
            self.leading_vehicle = LeadingVehicleState(
                s=float(data.get("s", 0.0)),
                s_dot=float(data.get("s_dot", 0.0)),
                s_ddot=float(data.get("s_ddot", 0.0)),
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to parse leading vehicle JSON: {e}")


    def _obstacle_state_callback(self, msg: Track2DArray) -> None:
        obstacles: List[Obstacle] = []

        for obs in msg.tracks:
            # -----------------------------
            # planning에서는 map 기준 절대값 사용
            # -----------------------------
            x = float(obs.map_pose.position.x)
            y = float(obs.map_pose.position.y)

            vx = float(obs.map_twist.linear.x)
            vy = float(obs.map_twist.linear.y)

            # planner_types.Obstacle 구조에 맞게 생성
            obstalce = Obstacle(
                x=x,
                y=y,
                vx=vx,
                vy=vy,
            )

            obstacles.append(obstalce)

        self.obstacles = obstacles    
        

    # ======================================================================
    # Timer loop
    # ======================================================================


    def _is_ready(self) -> bool:
        if self.ref_path is None:
            return False
        if self.trajectory_planner is None:
            return False
        if self.behavior_mode is None:
            return False
        if self.ego_frenet_state is None:
            return False
        return True
    

    def _on_timer(self) -> None:

        if not self._is_ready():
            return

        self.has_logged_waiting_state = False
        stamp = self.get_clock().now().to_msg()

        try:
            local_trajectory = self.trajectory_planner.generate_trajectory(
                frenet_state=self.ego_frenet_state,
                mode=self.behavior_mode,
                leading_vehicle=self.leading_vehicle,
                obstacles=self.obstacles,
            )
        except Exception as e:
            self.get_logger().error(f"[local_planner_node] Planner execution failed: {e}")
            local_trajectory = None

        candidate_marker_array = build_candidate_marker_array(
            candidates=self.trajectory_planner.last_all_candidates,
            frame_id=self.frame_id,
            stamp_msg=stamp,
            clear_first=True,
        )
        self.candidate_marker_pub.publish(candidate_marker_array)

        if local_trajectory is None:
            self._publish_empty_path()
            self.local_marker_pub.publish(MarkerArray())
            return
        
        self.previous_trajectory = local_trajectory
        
        local_path_msg = self._local_path_to_msg(local_trajectory)
        selected_marker_array = build_selected_marker_array(
            trajectory=local_trajectory,
            frame_id=self.frame_id,
            stamp_msg=stamp,
            clear_first=True,
        )

        self.local_path_pub.publish(local_path_msg)
        self.local_marker_pub.publish(selected_marker_array)


    def _publish_empty_path(self) -> None:
        path_msg                    = Path()
        path_msg.header.stamp       = self.get_clock().now().to_msg()
        path_msg.header.frame_id    = self.frame_id
        self.local_path_pub.publish(path_msg)


    # ======================================================================
    # Message conversion helpers
    # ======================================================================

    def _local_path_to_msg(self, trajectory: LocalTrajectory) -> Path:
        local_path_msg                  = Path()
        local_path_msg.header.stamp     = self.get_clock().now().to_msg()
        local_path_msg.header.frame_id  = self.frame_id

        for point in trajectory.points:
            pose = PoseStamped()
            pose.header = local_path_msg.header

            pose.pose.position.x    = float(point.x)
            pose.pose.position.y    = float(point.y)
            pose.pose.position.z    = 0.0
            pose.pose.orientation.w = 1.0

            local_path_msg.poses.append(pose)

        return local_path_msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()