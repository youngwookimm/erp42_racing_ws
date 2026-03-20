# AEB Test Runbook

## Terminal 1. ERP42 serial bridge

```bash
source /home/youngwoo/workspace/vil/erp42_racing_ros/install/setup.bash
source /home/youngwoo/workspace/vil/erp42_racing_ws/install/setup.bash
ros2 launch erp42_racing_serial serial_bridge.launch.py
```

## Terminal 2. GNSS / IMU driver

```bash
source /home/youngwoo/workspace/vil/erp42_racing_ros/install/setup.bash
source /home/youngwoo/workspace/vil/erp42_racing_ws/install/setup.bash
ros2 launch ISRO_P2_Driver ISRO_P2_Driver.launch.py
```

## Terminal 3. NTRIP

```bash
source /home/youngwoo/workspace/vil/erp42_racing_ros/install/setup.bash
source /home/youngwoo/workspace/vil/erp42_racing_ws/install/setup.bash
ros2 run ISRO_P2_Driver ntrip.py
```

## Terminal 4. Localization

```bash
source /home/youngwoo/workspace/vil/erp42_racing_ros/install/setup.bash
source /home/youngwoo/workspace/vil/erp42_racing_ws/install/setup.bash
ros2 launch erp42_racing_localization ros2_lla_utm.launch.py
```

## Terminal 5. Global path

```bash
source /home/youngwoo/workspace/vil/erp42_racing_ros/install/setup.bash
source /home/youngwoo/workspace/vil/erp42_racing_ws/install/setup.bash
ros2 launch erp42_racing_planning waypoints.launch.py
```

## Terminal 6. VLP32 LiDAR driver

```bash
source /home/youngwoo/workspace/vil/det_ws/install/setup.bash
ros2 launch velodyne velodyne-final-VLP32C-composed-launch.py
```

## Terminal 7. base_link <-> velodyne static TF

```bash
source /home/youngwoo/workspace/vil/det_ws/install/setup.bash
ros2 launch localization_tf localization_tf.launch.py

```

## Terminal 8. Ground removal

```bash
source /home/youngwoo/workspace/vil/det_ws/install/setup.bash
ros2 launch patchworkpp patchworkpp.launch.py
```

## Terminal 9. DBSCAN clustering

```bash
source /home/youngwoo/workspace/vil/det_ws/install/setup.bash
ros2 launch dbscan_clustering dbscan_clustering.launch.py
```

## Terminal 10. Obstacle filtering

```bash
source /home/youngwoo/workspace/vil/det_ws/install/setup.bash
ros2 launch obstacle_filtering_real obstacle_filtering.launch.py
```

## Terminal 11. L-shape fitting

```bash
source /home/youngwoo/workspace/vil/det_ws/install/setup.bash
ros2 launch lshape_fitting lshape_fitting.launch.py
```

## Terminal 12. Obstacle tracking

```bash
source /home/youngwoo/workspace/vil/det_ws/install/setup.bash
ros2 launch obstacle_tracking obstacle_tracking.launch.py
```

## Terminal 13. AEB test

```bash
source /home/youngwoo/workspace/vil/det_ws/install/setup.bash
source /home/youngwoo/workspace/vil/erp42_racing_ros/install/setup.bash
source /home/youngwoo/workspace/vil/erp42_racing_ws/install/setup.bash
ros2 launch erp42_racing_control aeb_test.launch.py
```
