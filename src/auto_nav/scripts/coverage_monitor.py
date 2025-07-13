#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs
from PIL import Image
import os
import time
import math

class CoverageMonitor:
    def __init__(self):
        rospy.init_node('coverage_monitor', anonymous=True)
        
        # 参数
        self.map_resolution = 0.09  # 地图分辨率
        self.robot_radius = 0.15    # 机器人半径
        self.coverage_radius = 0.3  # 清扫半径
        
        # 地图信息
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_origin = [0, 0]
        
        # 覆盖率数据
        self.covered_cells = set()
        self.total_accessible_cells = 0
        self.robot_path = []
        
        # 统计数据
        self.start_time = rospy.Time.now()
        self.goals_completed = 0
        self.total_goals = 0
        
        # 订阅器
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.path_sub = rospy.Subscriber('/plan_path', Path, self.path_callback)
        self.status_sub = rospy.Subscriber('/robot_status', String, self.status_callback)
        
        # 发布器
        self.coverage_pub = rospy.Publisher('/coverage_map', OccupancyGrid, queue_size=1)
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(5.0), self.publish_coverage_report)
        
        rospy.loginfo("Coverage monitor started")
        
    def map_callback(self, msg):
        """处理地图数据"""
        if self.map_data is None:
            self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.map_width = msg.info.width
            self.map_height = msg.info.height
            self.map_resolution = msg.info.resolution
            self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
            
            # 计算可访问的单元格总数
            self.calculate_accessible_cells()
            rospy.loginfo(f"Map loaded: {self.map_width}x{self.map_height}, accessible cells: {self.total_accessible_cells}")
    
    def calculate_accessible_cells(self):
        """计算地图中可访问的单元格数量"""
        if self.map_data is None:
            return
            
        # 认为值为0(自由空间)和-1(未知空间)的格子是可访问的
        # 值为100表示障碍物
        free_cells = np.where((self.map_data >= -1) & (self.map_data <= 10))
        self.total_accessible_cells = len(free_cells[0])
        
    def odom_callback(self, msg):
        """处理机器人位置，更新覆盖信息"""
        if self.map_data is None:
            return
            
        # 获取机器人在地图中的位置
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        
        # 转换为地图坐标
        map_x = int((robot_x - self.map_origin[0]) / self.map_resolution)
        map_y = int((robot_y - self.map_origin[1]) / self.map_resolution)
        
        # 记录路径
        self.robot_path.append((robot_x, robot_y))
        
        # 更新覆盖区域
        self.update_coverage(map_x, map_y)
        
    def update_coverage(self, center_x, center_y):
        """更新机器人周围的覆盖区域"""
        if self.map_data is None:
            return
            
        # 计算覆盖半径对应的格子数
        radius_cells = int(self.coverage_radius / self.map_resolution)
        
        # 在机器人周围标记已覆盖的区域
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx*dx + dy*dy <= radius_cells*radius_cells:
                    x = center_x + dx
                    y = center_y + dy
                    
                    if (0 <= x < self.map_width and 0 <= y < self.map_height):
                        # 只有自由空间才能被覆盖
                        if self.map_data[y, x] >= -1 and self.map_data[y, x] <= 10:
                            self.covered_cells.add((x, y))
    
    def path_callback(self, msg):
        """处理路径信息"""
        self.total_goals = len(msg.poses)
        rospy.loginfo(f"Path received with {self.total_goals} goals")
    
    def status_callback(self, msg):
        """处理机器人状态"""
        if "Goal" in msg.data and "reached" in msg.data:
            self.goals_completed += 1
            
    def calculate_coverage_percentage(self):
        """计算覆盖率百分比"""
        if self.total_accessible_cells == 0:
            return 0.0
        return (len(self.covered_cells) / self.total_accessible_cells) * 100.0
    
    def publish_coverage_report(self, event):
        """发布覆盖率报告"""
        coverage_percent = self.calculate_coverage_percentage()
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        
        rospy.loginfo("="*50)
        rospy.loginfo("COVERAGE REPORT")
        rospy.loginfo("="*50)
        rospy.loginfo(f"Running time: {elapsed_time:.1f} seconds")
        rospy.loginfo(f"Goals completed: {self.goals_completed}/{self.total_goals}")
        rospy.loginfo(f"Total accessible cells: {self.total_accessible_cells}")
        rospy.loginfo(f"Covered cells: {len(self.covered_cells)}")
        rospy.loginfo(f"Coverage percentage: {coverage_percent:.2f}%")
        rospy.loginfo(f"Path length: {len(self.robot_path)} points")
        rospy.loginfo("="*50)
        
        # 如果任务完成，保存最终报告
        if self.goals_completed >= self.total_goals and self.total_goals > 0:
            self.save_final_report(coverage_percent, elapsed_time)
    
    def save_final_report(self, coverage_percent, elapsed_time):
        """保存最终覆盖率报告"""
        report_file = os.path.expanduser("~/coverage_report.txt")
        
        with open(report_file, 'w') as f:
            f.write("FINAL COVERAGE REPORT\n")
            f.write("="*50 + "\n")
            f.write(f"Total running time: {elapsed_time:.1f} seconds\n")
            f.write(f"Goals completed: {self.goals_completed}/{self.total_goals}\n")
            f.write(f"Total accessible cells: {self.total_accessible_cells}\n")
            f.write(f"Covered cells: {len(self.covered_cells)}\n")
            f.write(f"Final coverage percentage: {coverage_percent:.2f}%\n")
            f.write(f"Total path length: {len(self.robot_path)} points\n")
            f.write(f"Average speed: {len(self.robot_path)/elapsed_time:.2f} points/sec\n")
            f.write("="*50 + "\n")
        
        rospy.loginfo(f"Final coverage report saved to: {report_file}")
        rospy.loginfo(f"FINAL COVERAGE: {coverage_percent:.2f}%")
        
    def run(self):
        """主循环"""
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = CoverageMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Coverage monitor shutting down")
        pass
