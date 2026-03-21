### Dependencies
```bash
sudo apt update
sudo apt install python3-pyqt6 libgeographic-dev
rosdep install --rosdistro humble --from-paths src --ignore-src -r -y
```

### Build
```bash
colcon build --symlink-install
source install/setup.bash
```

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
ros2 launch erp42_racing_bringup localization.launch.py
```
```bash
ros2 launch obstacle_tracking perception_pipeline.launch.py
```

```bash
ros2 launch velodyne velodyne-final-VLP32C-composed-launch.py
```

```bash
ros2 launch erp42_racing_planning planning.launch.py
```

```bash
ros2 run erp42_racing_control aeb_node
```

```bash
ros2 run erp42_racing_control pure_pursuit_node
```

```bash
ros2 run erp42_racing_control vehicle_cmd_gate_node
```

