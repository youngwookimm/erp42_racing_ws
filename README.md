### setting

```bash
source /opt/ros/humble/setup.bash
source $HOME/workspace/ros2/erp42_racing_ros/install/setup.bash
source $HOME/workspace/ros2/erp42_racing_ws/install/setup.bash
```

## execution

```bash
ros2 launch erp42_racing_serial serial_bridge.launch.py
```

```bash
ros2 launch isro_p2_driver isro_p2_driver.launch.py
```

```bash
ros2 run isro_p2_driver ntrip.py
```


### UTM + translation (/utm_tm)

```bash
ros2 launch erp42_racing_localization ros2_lla_utm.launch.py
```

- /fix -> lla_utm_node -> /utm
- /utm -> utm_origin_shift_node.py -> /utm_tm (geometry_msgs/PoseStamped)
- /utm_tm + /imu/data -> vehicle_pose_viz_node.py -> /vehicle_pose (geometry_msgs/PoseStamped)
- /utm_tm은 pure_pursuit_node의 기본 위치 입력 토픽
- /vehicle_pose는 rviz에서 map 기준 차량 위치 + heading 확인용 토픽

현재 원점 기준:

- erp42_racing_planning/resource/fix_utm_v2.csv
- 첫 번째 row의 L1_UTM_X, L1_UTM_Y를 원점으로 사용


### global path publish

```bash
ros2 launch erp42_racing_planning waypoints.launch.py
```

- /L1/waypoints (nav_msgs/Path)
- /R1/waypoints (nav_msgs/Path)
- CSV 기준 전역 경로 생성
- pure_pursuit_node는 기본으로 /L1/waypoints를 사용

현재 CSV:

- erp42_racing_planning/resource/fix_utm_v2.csv

### Pure Pursuit

```bash
ros2 run erp42_racing_control pure_pursuit_node
```
