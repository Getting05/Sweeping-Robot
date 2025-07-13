#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
实时覆盖率监控脚本 v2.0
监控机器人清扫覆盖率 = 已清扫面积 / 自由区域总面积
支持完整的路径历史保存和实时覆盖率计算
"""

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import math
import time
from collections import deque

class CoverageMonitor:
    def __init__(self):
        rospy.init_node('coverage_monitor', anonymous=True)
        
        # 参数设置
        self.robot_radius = 0.15  # 机器人清扫半径(米)
        self.update_rate = 2.0    # 更新频率(Hz)
        self.min_point_distance = 0.05  # 最小点间距(米)
        
        # 地图相关
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        self.free_area_total = 0  # 总自由区域面积
        
        # 覆盖率相关
        self.covered_grid = None  # 已覆盖区域栅格
        self.covered_area = 0     # 已覆盖面积
        self.coverage_percentage = 0.0
        self.path_length = 0.0    # 路径总长度
        
        # 路径点历史
        self.path_history = deque(maxlen=20000)  # 保存完整历史路径点
        self.last_position = None
        self.last_path_size = 0  # 记录上次处理的路径大小
        
        # 统计信息
        self.start_time = time.time()
        self.total_goals = 0
        self.completed_goals = 0
        
        # 订阅器
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        self.path_sub = rospy.Subscriber('/passedPath', Path, self.path_callback, queue_size=1)
        self.plan_sub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.plan_callback, queue_size=1)
        
        # 发布器
        self.coverage_pub = rospy.Publisher('/coverage_percentage', Float32, queue_size=1)
        self.covered_grid_pub = rospy.Publisher('/covered_area_grid', OccupancyGrid, queue_size=1)
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_coverage)
        
        rospy.loginfo("=== Coverage Monitor v2.0 Started ===")
        rospy.loginfo("Robot cleaning radius: %.2f meters", self.robot_radius)
        rospy.loginfo("Update rate: %.1f Hz", self.update_rate)
    
    def map_callback(self, msg):
        """处理地图数据"""
        try:
            self.map_resolution = msg.info.resolution
            self.map_origin_x = msg.info.origin.position.x
            self.map_origin_y = msg.info.origin.position.y
            self.map_width = msg.info.width
            self.map_height = msg.info.height
            
            # 转换为numpy数组
            self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
            
            # 初始化覆盖栅格
            self.covered_grid = np.zeros_like(self.map_data, dtype=np.uint8)
            
            # 计算自由区域总面积
            free_cells = np.sum(self.map_data == 0)  # 0表示自由空间
            self.free_area_total = free_cells * (self.map_resolution ** 2)
            
            rospy.loginfo("Map loaded: %dx%d, resolution=%.3f, free_area=%.2f m²",
                          self.map_width, self.map_height, self.map_resolution, self.free_area_total)
                         
        except Exception as e:
            rospy.logerr("Error processing map: %s", str(e))
    
    def plan_callback(self, msg):
        """处理规划路径，获取总目标数"""
        self.total_goals = len(msg.poses)
        rospy.loginfo("Total planned goals: %d", self.total_goals)
    
    def path_callback(self, msg):
        """处理已走过的路径 - 累积保存所有历史点"""
        if not msg.poses or self.map_data is None:
            return
            
        try:
            current_path_length = len(msg.poses)
            
            # 如果当前路径比上次记录的短，说明可能重置了
            if current_path_length < self.last_path_size:
                rospy.loginfo("Path reset detected, clearing history")
                self.path_history.clear()
                self.covered_grid = np.zeros_like(self.map_data, dtype=np.uint8)
                self.last_path_size = 0
            
            # 只处理新增的路径点
            for i in range(self.last_path_size, current_path_length):
                pose = msg.poses[i]
                world_x = pose.pose.position.x
                world_y = pose.pose.position.y
                
                # 检查是否与上一个点距离足够
                if self.last_position is not None:
                    dist = math.sqrt((world_x - self.last_position[0])**2 +
                                    (world_y - self.last_position[1])**2)
                    if dist < self.min_point_distance:
                        continue
                
                # 转换为栅格坐标
                grid_x, grid_y = self.world_to_grid(world_x, world_y)
                
                if self.is_valid_grid_point(grid_x, grid_y):
                    self.path_history.append((world_x, world_y))
                    self.last_position = (world_x, world_y)
                    
                    # 标记覆盖区域
                    self.mark_covered_area(grid_x, grid_y)
            
            # 更新记录的路径大小
            self.last_path_size = current_path_length
            
            # 计算路径长度
            if len(self.path_history) > 1:
                self.path_length = self.calculate_path_length()
                
        except Exception as e:
            rospy.logerr("Error processing path: %s", str(e))
    
    def world_to_grid(self, world_x, world_y):
        """世界坐标转栅格坐标"""
        grid_x = int((world_x - self.map_origin_x) / self.map_resolution)
        grid_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return grid_x, grid_y
    
    def is_valid_grid_point(self, grid_x, grid_y):
        """检查栅格点是否有效"""
        return (0 <= grid_x < self.map_width and 
                0 <= grid_y < self.map_height and
                self.map_data[grid_y, grid_x] == 0)  # 只考虑自由空间
    
    def mark_covered_area(self, center_x, center_y):
        """标记以机器人为中心的覆盖区域"""
        try:
            # 计算覆盖半径对应的栅格数
            radius_cells = int(self.robot_radius / self.map_resolution)
            
            # 创建圆形覆盖区域
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    # 检查是否在圆形范围内
                    distance = math.sqrt(dx*dx + dy*dy) * self.map_resolution
                    if distance <= self.robot_radius:
                        
                        new_x = center_x + dx
                        new_y = center_y + dy
                        
                        # 检查边界和自由空间
                        if self.is_valid_grid_point(new_x, new_y):
                            self.covered_grid[new_y, new_x] = 100  # 标记为已覆盖
                            
        except Exception as e:
            rospy.logerr("Error marking covered area: %s", str(e))
    
    def calculate_path_length(self):
        """计算路径总长度"""
        if len(self.path_history) < 2:
            return 0.0
            
        total_length = 0.0
        points = list(self.path_history)
        
        for i in range(1, len(points)):
            dx = points[i][0] - points[i-1][0]
            dy = points[i][1] - points[i-1][1]
            total_length += math.sqrt(dx*dx + dy*dy)
            
        return total_length
    
    def update_coverage(self, event):
        """更新覆盖率计算和发布"""
        if self.map_data is None or self.covered_grid is None:
            return
            
        try:
            # 计算已覆盖面积
            covered_cells = np.sum(self.covered_grid == 100)
            self.covered_area = covered_cells * (self.map_resolution ** 2)
            
            # 计算覆盖率百分比
            if self.free_area_total > 0:
                self.coverage_percentage = (self.covered_area / self.free_area_total) * 100.0
            else:
                self.coverage_percentage = 0.0
            
            # 运行时间
            elapsed_time = time.time() - self.start_time
            
            # 估算目标完成率
            if self.total_goals > 0:
                # 基于路径长度估算完成的目标数
                if self.path_length > 0:
                    estimated_completed = min(len(self.path_history) // 3, self.total_goals)
                    self.completed_goals = estimated_completed
            
            # 发布覆盖率
            coverage_msg = Float32()
            coverage_msg.data = self.coverage_percentage
            self.coverage_pub.publish(coverage_msg)
            
            # 发布覆盖栅格地图
            self.publish_covered_grid()
            
            # 打印统计信息
            self.print_statistics(elapsed_time)
            
        except Exception as e:
            rospy.logerr("Error updating coverage: %s", str(e))
    
    def publish_covered_grid(self):
        """发布已覆盖区域的栅格地图"""
        try:
            grid_msg = OccupancyGrid()
            grid_msg.header.stamp = rospy.Time.now()
            grid_msg.header.frame_id = "map"
            
            grid_msg.info.resolution = self.map_resolution
            grid_msg.info.width = self.map_width
            grid_msg.info.height = self.map_height
            grid_msg.info.origin.position.x = self.map_origin_x
            grid_msg.info.origin.position.y = self.map_origin_y
            grid_msg.info.origin.position.z = 0.0
            grid_msg.info.origin.orientation.w = 1.0
            
            # 创建覆盖率可视化数据
            visual_data = np.copy(self.map_data).astype(np.int8)
            
            # 将已覆盖区域标记为特殊值
            covered_mask = self.covered_grid == 100
            visual_data[covered_mask] = 75  # 较深灰色表示已覆盖
            
            grid_msg.data = visual_data.flatten().tolist()
            self.covered_grid_pub.publish(grid_msg)
            
        except Exception as e:
            rospy.logerr("Error publishing covered grid: %s", str(e))
    
    def print_statistics(self, elapsed_time):
        """打印统计信息"""
        rospy.loginfo("=== 实时覆盖率统计 ===")
        rospy.loginfo("运行时间: %.1f 秒 (%.1f 分钟)", elapsed_time, elapsed_time/60)
        rospy.loginfo("历史路径点: %d", len(self.path_history))
        rospy.loginfo("路径总长度: %.2f 米", self.path_length)
        rospy.loginfo("已覆盖面积: %.2f m²", self.covered_area)
        rospy.loginfo("总自由面积: %.2f m²", self.free_area_total)
        rospy.loginfo("当前覆盖率: %.2f%%", self.coverage_percentage)
        
        if self.total_goals > 0:
            goal_percentage = (self.completed_goals / self.total_goals) * 100.0
            rospy.loginfo("估算目标完成: %d/%d (%.1f%%)",
                          self.completed_goals, self.total_goals, goal_percentage)
        
        # 计算效率指标
        if elapsed_time > 0:
            speed = self.path_length / elapsed_time
            coverage_rate = self.covered_area / elapsed_time
            rospy.loginfo("平均速度: %.3f m/s", speed)
            rospy.loginfo("覆盖速率: %.3f m²/s", coverage_rate)
        
        rospy.loginfo("====================")
    
    def save_final_report(self):
        """保存最终报告"""
        try:
            elapsed_time = time.time() - self.start_time
            
            report = f"""=== 清扫任务最终报告 ===
任务完成时间: {time.strftime('%Y-%m-%d %H:%M:%S')}
总运行时间: {elapsed_time:.1f} 秒 ({elapsed_time/60:.1f} 分钟)

路径统计:
- 保存的历史路径点: {len(self.path_history)}
- 总路径长度: {self.path_length:.2f} 米
- 平均移动速度: {self.path_length/elapsed_time:.3f} m/s

覆盖率统计:
- 已覆盖面积: {self.covered_area:.2f} m²
- 总自由面积: {self.free_area_total:.2f} m²
- 最终覆盖率: {self.coverage_percentage:.2f}%
- 覆盖效率: {self.covered_area/elapsed_time:.3f} m²/s

目标完成情况:
- 计划目标数: {self.total_goals}
- 估算完成数: {self.completed_goals}
- 估算完成率: {(self.completed_goals/self.total_goals*100) if self.total_goals > 0 else 0:.1f}%

系统参数:
- 机器人清扫半径: {self.robot_radius} 米
- 地图分辨率: {self.map_resolution} 米/像素
- 最小点间距: {self.min_point_distance} 米
- 最大保存路径点: 20000 点

性能评估:
- 路径点密度: {len(self.path_history)/self.path_length:.1f} 点/米
- 时间效率: {'优秀' if self.coverage_percentage/elapsed_time*60 > 1.0 else '良好' if self.coverage_percentage/elapsed_time*60 > 0.5 else '待优化'}
========================"""
            
            # 保存到文件
            filename = f"/tmp/coverage_report_{int(time.time())}.txt"
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(report)
            
            rospy.loginfo("Final report saved to: %s", filename)
            print(report)  # 同时打印到控制台
            
        except Exception as e:
            rospy.logerr("Error saving final report: %s", str(e))

def main():
    try:
        monitor = CoverageMonitor()
        
        # 等待关闭信号
        rospy.on_shutdown(lambda: monitor.save_final_report())
        
        rospy.loginfo("Coverage Monitor ready. Waiting for map and path data...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Coverage monitor shutdown")
    except Exception as e:
        rospy.logerr("Coverage monitor error: %s", str(e))

if __name__ == '__main__':
    main()