#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import pandas as pd
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped

class CsvToUtmPublisher(Node):
    def __init__(self):
        super().__init__('csv_to_utm_pub')
        
        try:
            package_share_directory = get_package_share_directory('erp42_racing_localization')
        except Exception as e:
            self.get_logger().error(f'패키지를 찾을 수 없습니다: {e}')
            return

        # 1. Left (L1.csv) 설정
        left_csv_path = os.path.join(package_share_directory, 'data', 'L1.csv')
        self.left_pub = self.create_publisher(PoseStamped, '/utm_pose/left', 10)
        self.left_df = pd.read_csv(left_csv_path)
        self.get_logger().info(f'Left CSV 로드 완료: {left_csv_path}')
        
        # 2. Right (R1.csv) 설정
        right_csv_path = os.path.join(package_share_directory, 'data', 'R1.csv')
        self.right_pub = self.create_publisher(PoseStamped, '/utm_pose/right', 10)
        self.right_df = pd.read_csv(right_csv_path)
        self.get_logger().info(f'Right CSV 로드 완료: {right_csv_path}')
        
        self.current_idx = 0
        self.max_idx = max(len(self.left_df), len(self.right_df))
        
        # 10Hz (0.1초) 주기로 데이터 발행
        self.timer = self.create_timer(0.1, self.timer_callback)

    def create_pose_msg(self, df, idx):
        # 데이터 범위를 벗어나면 발행하지 않음
        if idx >= len(df):
            return None
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        try:
            msg.pose.position.x = float(df.iloc[idx]['UTM_X'])
            msg.pose.position.y = float(df.iloc[idx]['UTM_Y'])
            msg.pose.position.z = 0.0
            return msg
        except KeyError as e:
            self.get_logger().error(f'CSV 컬럼명을 확인하세요: {e}')
            return None

    def timer_callback(self):
        if self.current_idx < self.max_idx:
            # Left 차선 데이터 발행
            l_msg = self.create_pose_msg(self.left_df, self.current_idx)
            if l_msg:
                self.left_pub.publish(l_msg)
            
            # Right 차선 데이터 발행
            r_msg = self.create_pose_msg(self.right_df, self.current_idx)
            if r_msg:
                self.right_pub.publish(r_msg)
            
            self.current_idx += 1
        else:
            self.get_logger().info('모든 CSV 데이터를 전송했습니다.')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = CsvToUtmPublisher()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()