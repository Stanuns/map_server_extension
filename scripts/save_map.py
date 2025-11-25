#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import yaml
from PIL import Image
import numpy as np
import os

class MapSaver(Node):
    def __init__(self):
        super().__init__('custom_map_saver')
        
        # 设置与 octomap_server 匹配的 QoS 配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/projected_map',
            self.map_callback,
            qos_profile
        )
        
        self.map_received = False
        self.map_msg = None
        
    def map_callback(self, msg):
        self.get_logger().info('Received map data')
        self.map_msg = msg
        self.map_received = True
        
    def save_map(self, filename):
        if not self.map_received:
            self.get_logger().error('No map data received')
            return False
            
        try:
            # 保存 YAML 文件
            map_yaml = {
                'image': filename + '.pgm',
                'resolution': self.map_msg.info.resolution,
                'origin': [
                    self.map_msg.info.origin.position.x,
                    self.map_msg.info.origin.position.y,
                    self.map_msg.info.origin.position.z
                ],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.25
            }
            
            with open(filename + '.yaml', 'w') as yaml_file:
                yaml.dump(map_yaml, yaml_file, default_flow_style=False)
            
            # 保存 PGM 文件
            width = self.map_msg.info.width
            height = self.map_msg.info.height
            data = np.array(self.map_msg.data, dtype=np.int8).reshape((height, width))
            
            # 将 -1 (未知) 转换为 205, 0-100 转换为 0-254
            pgm_data = np.where(data == -1, 205, 
                               np.where(data == 0, 254, 
                                       np.where(data == 100, 0, 205)))
            
            pgm_data = pgm_data.astype(np.uint8)
            
            # 修复镜像问题：将图像上下翻转以匹配原始方向
            pgm_data = np.flipud(pgm_data)
            
            # 使用 PIL 保存图像
            img = Image.fromarray(pgm_data, mode='L')
            img.save(filename + '.pgm')
            
            self.get_logger().info(f'Map saved successfully: {filename}.yaml and {filename}.pgm')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to save map: {str(e)}')
            return False

def main():
    rclpy.init()
    
    map_saver = MapSaver()
    
    # 等待地图数据
    map_saver.get_logger().info('Waiting for map data...')
    
    # 创建旋转器并等待数据
    import time
    start_time = time.time()
    timeout = 10.0  # 10秒超时
    
    while not map_saver.map_received and time.time() - start_time < timeout:
        rclpy.spin_once(map_saver, timeout_sec=0.1)
    
    if map_saver.map_received:
        map_saver.get_logger().info('Map data received, saving...')
        success = map_saver.save_map('custom_saved_map')
        if success:
            map_saver.get_logger().info('Map saved successfully!')
        else:
            map_saver.get_logger().error('Failed to save map')
    else:
        map_saver.get_logger().error('Timeout waiting for map data')
    
    map_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()