#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
实时覆盖率监控脚本 v2.2
监控机器人清扫覆盖率 = 已清扫面积 / 自由区域总面积
支持完整的路径历史保存和实时覆盖率计算
v2.1 新增功能：每30秒自动保存一次评估数据到CSV文件（追加模式）
v2.2 新增功能：覆盖率2分钟无变化时自动重启清扫任务
"""

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path, Odometry
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
        self.redundant_area = 0   # 重复清扫面积
        
        # 路径点历史
        self.path_history = deque(maxlen=20000)  # 保存完整历史路径点
        self.last_position = None
        self.last_path_size = 0  # 记录上次处理的路径大小
        
        # 统计信息
        self.start_time = time.time()
        self.total_goals = 0
        self.completed_goals = 0
        self.planned_path_points = 0  # 规划路径点总数
        
        # 运动学指标
        self.velocity_history = deque(maxlen=1000)  # 速度历史
        self.acceleration_history = deque(maxlen=1000)  # 加速度历史
        self.last_velocity = None
        self.last_time = None
        
        # 碰撞检测
        self.collision_count = 0
        self.last_position_for_collision = None
        self.stuck_threshold = 0.05  # 卡住检测阈值(米)
        self.stuck_time_threshold = 10.0  # 卡住时间阈值(秒)
        self.stuck_start_time = None
        self.collision_cooldown = 15.0  # 碰撞检测冷却时间(秒)
        self.last_collision_time = 0.0  # 上次碰撞检测时间
        
        # 计算时间监控
        self.computation_times = []
        self.last_computation_start = None
        
        # 定时CSV保存相关
        self.csv_save_interval = 30.0  # 30秒自动保存一次
        self.last_csv_save_time = time.time()
        self.csv_filename = f"/home/getting/tmp/sweeping_robot_realtime_data_{int(time.time())}.csv"
        self.csv_initialized = False
        
        # 覆盖率停滞检测和自动重启相关
        self.coverage_stagnation_threshold = 60.0  # 60秒无变化阈值
        self.last_coverage_change_time = time.time()
        self.last_recorded_coverage = 0.0
        self.coverage_change_tolerance = 0.001  # 覆盖率变化最小阈值 (0.1%)
        self.auto_restart_enabled = True  # 是否启用自动重启
        self.restart_command = "roslaunch auto_nav sequential_clean.launch"
        
        # 订阅器
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        self.path_sub = rospy.Subscriber('/passedPath', Path, self.path_callback, queue_size=1)
        self.plan_sub = rospy.Subscriber('/plan_path', Path, self.plan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        
        # 发布器
        self.coverage_pub = rospy.Publisher('/coverage_percentage', Float32, queue_size=1)
        self.covered_grid_pub = rospy.Publisher('/covered_area_grid', OccupancyGrid, queue_size=1)
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_coverage)
        
        rospy.loginfo("=== Coverage Monitor v2.2 Started ===")
        rospy.loginfo("Robot cleaning radius: %.2f meters", self.robot_radius)
        rospy.loginfo("Update rate: %.1f Hz", self.update_rate)
        rospy.loginfo("Auto CSV save interval: %.0f seconds", self.csv_save_interval)
        rospy.loginfo("Coverage stagnation threshold: %.0f seconds", self.coverage_stagnation_threshold)
        rospy.loginfo("Auto restart enabled: %s", self.auto_restart_enabled)
        rospy.loginfo("Collision detection cooldown: %.1f seconds", self.collision_cooldown)
        rospy.loginfo("Realtime CSV file: %s", self.csv_filename)
    
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
        self.planned_path_points = len(msg.poses)  # 记录规划路径点总数
        rospy.loginfo("规划路径加载: %d 个目标点", self.total_goals)
    
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
                            # 如果已经被覆盖过，计入重复面积
                            if self.covered_grid[new_y, new_x] == 100:
                                self.redundant_area += self.map_resolution ** 2
                            else:
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
            
            # 检查是否需要定时保存CSV数据
            self.check_and_save_csv_data(elapsed_time)
            
            # 检查覆盖率停滞并处理自动重启
            self.check_coverage_stagnation(elapsed_time)
            
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
        # 计算所有评估指标
        metrics = self.calculate_evaluation_metrics(elapsed_time)
        
        rospy.loginfo("=== 扫地机器人性能指标统计 ===")
        rospy.loginfo("运行时间: %.1f 秒 (%.1f 分钟)", elapsed_time, elapsed_time/60)
        rospy.loginfo("规划路径点总数: %d", metrics['Planned_Points'])
        rospy.loginfo("历史清扫点数: %d", len(self.path_history))
        rospy.loginfo("路径总长度: %.2f 米", self.path_length)
        
        rospy.loginfo("--- 核心指标 ---")
        rospy.loginfo("CR (覆盖率): %.3f (%.2f%%)", metrics['CR'], metrics['CR']*100)
        rospy.loginfo("ME (运动效率): %.3f m/m²", metrics['ME'])
        rospy.loginfo("SR (清洁冗余度): %.3f", metrics['SR'])
        rospy.loginfo("Collision (碰撞次数): %d", metrics['Collision'])
        rospy.loginfo("CT (平均计算时间): %.3f s", metrics['CT'])
        rospy.loginfo("FT (任务总耗时): %.1f s", metrics['FT'])
        
        rospy.loginfo("--- 运动学指标 ---")
        rospy.loginfo("Vel_avg (平均速度): %.3f m/s", metrics['Vel_avg'])
        rospy.loginfo("Acc_avg (平均加速度): %.3f m/s²", metrics['Acc_avg'])
        rospy.loginfo("Jerk_avg (平均加加速度): %.3f m/s³", metrics['Jerk_avg'])
        
        rospy.loginfo("--- 面积统计 ---")
        rospy.loginfo("已覆盖面积: %.2f m²", self.covered_area)
        rospy.loginfo("重复清扫面积: %.2f m²", self.redundant_area)
        rospy.loginfo("总自由面积: %.2f m²", self.free_area_total)
        
        if self.total_goals > 0:
            goal_percentage = (self.completed_goals / self.total_goals) * 100.0
            rospy.loginfo("估算目标完成: %d/%d (%.1f%%)",
                          self.completed_goals, self.total_goals, goal_percentage)
        
        rospy.loginfo("===========================")
    
    def save_final_report(self):
        """保存最终报告"""
        try:
            elapsed_time = time.time() - self.start_time
            metrics = self.calculate_evaluation_metrics(elapsed_time)
            
            report = f"""=== 清扫任务最终报告 ===
任务完成时间: {time.strftime('%Y-%m-%d %H:%M:%S')}
总运行时间: {elapsed_time:.1f} 秒 ({elapsed_time/60:.1f} 分钟)

=== 核心评估指标 ===
CR (覆盖率): {metrics['CR']:.3f} ({metrics['CR']*100:.2f}%)
ME (运动效率): {metrics['ME']:.3f} m/m²
SR (清洁冗余度): {metrics['SR']:.3f}
Collision (碰撞次数): {metrics['Collision']}
CT (平均计算时间): {metrics['CT']:.3f} s
FT (任务总耗时): {metrics['FT']:.1f} s
Vel_avg (平均速度): {metrics['Vel_avg']:.3f} m/s
Acc_avg (平均加速度): {metrics['Acc_avg']:.3f} m/s²
Jerk_avg (平均加加速度): {metrics['Jerk_avg']:.3f} m/s³

=== 路径统计 ===
规划路径点总数: {metrics['Planned_Points']}
实际清扫路径点: {len(self.path_history)}
总路径长度: {self.path_length:.2f} 米
平均移动速度: {self.path_length/elapsed_time:.3f} m/s

=== 面积统计 ===
已覆盖面积: {self.covered_area:.2f} m²
重复清扫面积: {self.redundant_area:.2f} m²
总自由面积: {self.free_area_total:.2f} m²
最终覆盖率: {self.coverage_percentage:.2f}%
覆盖效率: {self.covered_area/elapsed_time:.3f} m²/s

=== 目标完成情况 ===
计划目标数: {self.total_goals}
估算完成数: {self.completed_goals}
估算完成率: {(self.completed_goals/self.total_goals*100) if self.total_goals > 0 else 0:.1f}%

=== 系统参数 ===
机器人清扫半径: {self.robot_radius} 米
地图分辨率: {self.map_resolution} 米/像素
最小点间距: {self.min_point_distance} 米
卡住检测阈值: {self.stuck_threshold} 米
卡住时间阈值: {self.stuck_time_threshold} 秒

=== 性能评估 ===
路径点密度: {len(self.path_history)/self.path_length:.1f} 点/米
运动效率评级: {'优秀' if metrics['ME'] < 2.0 else '良好' if metrics['ME'] < 4.0 else '待优化'}
覆盖效率评级: {'优秀' if metrics['CR'] > 0.8 else '良好' if metrics['CR'] > 0.6 else '待优化'}
冗余度评级: {'优秀' if metrics['SR'] < 0.1 else '良好' if metrics['SR'] < 0.2 else '待优化'}
碰撞安全评级: {'优秀' if metrics['Collision'] == 0 else '良好' if metrics['Collision'] < 3 else '待优化'}
========================"""
            
            # 保存到文件
            filename = f"/home/getting/tmp/sweeping_robot_report_{int(time.time())}.txt"
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(report)
            
            # 同时保存CSV格式用于数据分析
            csv_filename = f"/home/getting/tmp/sweeping_robot_metrics_{int(time.time())}.csv"
            with open(csv_filename, 'w', encoding='utf-8') as f:
                f.write("指标,数值,单位,说明\n")
                f.write(f"CR,{metrics['CR']:.3f},,覆盖率\n")
                f.write(f"ME,{metrics['ME']:.3f},m/m²,运动效率\n")
                f.write(f"SR,{metrics['SR']:.3f},,清洁冗余度\n")
                f.write(f"Collision,{metrics['Collision']},,碰撞次数\n")
                f.write(f"CT,{metrics['CT']:.3f},s,平均计算时间\n")
                f.write(f"FT,{metrics['FT']:.1f},s,任务总耗时\n")
                f.write(f"Vel_avg,{metrics['Vel_avg']:.3f},m/s,平均速度\n")
                f.write(f"Acc_avg,{metrics['Acc_avg']:.3f},m/s²,平均加速度\n")
                f.write(f"Jerk_avg,{metrics['Jerk_avg']:.3f},m/s³,平均加加速度\n")
                f.write(f"Planned_Points,{metrics['Planned_Points']},,规划路径点总数\n")
                f.write(f"Path_Length,{self.path_length:.2f},m,路径总长度\n")
                f.write(f"Covered_Area,{self.covered_area:.2f},m²,已覆盖面积\n")
                f.write(f"Redundant_Area,{self.redundant_area:.2f},m²,重复清扫面积\n")
            
            rospy.loginfo("最终报告已保存: %s", filename)
            rospy.loginfo("评估指标CSV已保存: %s", csv_filename)
            rospy.loginfo("实时数据CSV文件: %s", self.csv_filename)
            print(report)  # 同时打印到控制台
            
        except Exception as e:
            rospy.logerr("保存最终报告错误: %s", str(e))

    def odom_callback(self, msg):
        """处理里程计数据，计算运动学指标"""
        try:
            current_time = time.time()
            position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            
            # 计算速度
            if self.last_position and self.last_time:
                dt = current_time - self.last_time
                if dt > 0:
                    dx = position[0] - self.last_position[0]
                    dy = position[1] - self.last_position[1]
                    velocity = math.sqrt(dx*dx + dy*dy) / dt
                    self.velocity_history.append(velocity)
                    
                    # 计算加速度
                    if self.last_velocity is not None:
                        acceleration = (velocity - self.last_velocity) / dt
                        self.acceleration_history.append(acceleration)
                    
                    self.last_velocity = velocity
            
            # 碰撞检测（基于是否卡住）
            self.detect_collision(position, current_time)
            
            self.last_time = current_time
            
        except Exception as e:
            rospy.logerr("里程计处理错误: %s", str(e))
    
    def detect_collision(self, position, current_time):
        """检测碰撞（通过卡住检测），带15秒冷却功能"""
        # 检查是否在冷却期内
        if current_time - self.last_collision_time < self.collision_cooldown:
            # 在冷却期内，不进行碰撞检测，但继续更新位置信息
            self.last_position_for_collision = position
            return
        
        if self.last_position_for_collision is not None:
            dist = math.sqrt((position[0] - self.last_position_for_collision[0])**2 +
                           (position[1] - self.last_position_for_collision[1])**2)
            
            if dist < self.stuck_threshold:
                if self.stuck_start_time is None:
                    self.stuck_start_time = current_time
                elif current_time - self.stuck_start_time > self.stuck_time_threshold:
                    self.collision_count += 1
                    self.last_collision_time = current_time  # 记录碰撞时间，开始冷却
                    rospy.logwarn("检测到碰撞/卡住事件，总计: %d (开始15秒冷却)", self.collision_count)
                    self.stuck_start_time = None
            else:
                self.stuck_start_time = None
        
        self.last_position_for_collision = position
    
    def start_computation_timer(self):
        """开始计算时间计时"""
        self.last_computation_start = time.time()
    
    def end_computation_timer(self):
        """结束计算时间计时"""
        if self.last_computation_start is not None:
            computation_time = time.time() - self.last_computation_start
            self.computation_times.append(computation_time)
            self.last_computation_start = None

    def check_and_save_csv_data(self, elapsed_time):
        """检查并执行定时CSV数据保存"""
        current_time = time.time()
        
        # 检查是否到达保存间隔
        if current_time - self.last_csv_save_time >= self.csv_save_interval:
            self.save_realtime_csv_data(elapsed_time)
            self.last_csv_save_time = current_time
    
    def save_realtime_csv_data(self, elapsed_time):
        """保存实时评估数据到CSV（追加模式）"""
        try:
            # 计算当前所有指标
            metrics = self.calculate_evaluation_metrics(elapsed_time)
            
            # 如果CSV文件未初始化，先写入表头
            if not self.csv_initialized:
                with open(self.csv_filename, 'w', encoding='utf-8') as f:
                    headers = [
                        "Timestamp", "Runtime_s", "Runtime_min", "Coverage_Rate", "Motion_Efficiency",
                        "Redundancy", "Collision_Count", "Avg_Computation_Time", "Total_Time",
                        "Avg_Velocity", "Avg_Acceleration", "Avg_Jerk", "Planned_Points",
                        "Path_Length", "Covered_Area", "Redundant_Area", "Free_Area_Total",
                        "Path_Points_Count", "Completed_Goals", "Goal_Progress_Rate"
                    ]
                    f.write(",".join(headers) + "\n")
                self.csv_initialized = True
                rospy.loginfo("创建实时CSV文件: %s", self.csv_filename)
            
            # 追加当前数据
            with open(self.csv_filename, 'a', encoding='utf-8') as f:
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                goal_progress = (self.completed_goals / self.total_goals * 100) if self.total_goals > 0 else 0
                
                data = [
                    timestamp,                          # 时间戳
                    f"{elapsed_time:.1f}",             # 运行时间(秒)
                    f"{elapsed_time/60:.2f}",          # 运行时间(分钟)
                    f"{metrics['CR']:.4f}",            # 覆盖率
                    f"{metrics['ME']:.4f}",            # 运动效率
                    f"{metrics['SR']:.4f}",            # 冗余度
                    f"{metrics['Collision']}",         # 碰撞次数
                    f"{metrics['CT']:.4f}",            # 平均计算时间
                    f"{metrics['FT']:.1f}",            # 总时间
                    f"{metrics['Vel_avg']:.4f}",       # 平均速度
                    f"{metrics['Acc_avg']:.4f}",       # 平均加速度
                    f"{metrics['Jerk_avg']:.4f}",      # 平均加加速度
                    f"{metrics['Planned_Points']}",    # 规划路径点数
                    f"{self.path_length:.2f}",         # 路径长度
                    f"{self.covered_area:.2f}",        # 已覆盖面积
                    f"{self.redundant_area:.2f}",      # 重复面积
                    f"{self.free_area_total:.2f}",     # 总自由面积
                    f"{len(self.path_history)}",       # 路径点数量
                    f"{self.completed_goals}",         # 完成目标数
                    f"{goal_progress:.2f}"             # 目标完成率
                ]
                f.write(",".join(data) + "\n")
            
            rospy.loginfo("已保存实时数据到CSV (运行时间: %.1f s, 覆盖率: %.2f%%)", 
                         elapsed_time, self.coverage_percentage)
            
        except Exception as e:
            rospy.logerr("保存实时CSV数据错误: %s", str(e))

    def check_coverage_stagnation(self, elapsed_time):
        """检查覆盖率是否停滞，如停滞超过阈值则触发自动重启"""
        if not self.auto_restart_enabled:
            return
            
        current_time = time.time()
        current_coverage = self.coverage_percentage / 100.0  # 转换为0-1范围
        
        # 检查覆盖率是否有显著变化
        coverage_change = abs(current_coverage - self.last_recorded_coverage)
        
        if coverage_change >= self.coverage_change_tolerance:
            # 覆盖率有显著变化，更新记录
            self.last_recorded_coverage = current_coverage
            self.last_coverage_change_time = current_time
            rospy.loginfo_throttle(30, "覆盖率正常变化: %.3f%% (变化量: %.3f%%)", 
                                 current_coverage * 100, coverage_change * 100)
        else:
            # 覆盖率无显著变化，检查是否超过停滞阈值
            stagnation_duration = current_time - self.last_coverage_change_time
            
            if stagnation_duration >= self.coverage_stagnation_threshold:
                rospy.logwarn("检测到覆盖率停滞 %.1f 秒 (阈值: %.1f 秒)", 
                            stagnation_duration, self.coverage_stagnation_threshold)
                rospy.logwarn("当前覆盖率: %.3f%%, 上次显著变化: %.3f%%", 
                            current_coverage * 100, self.last_recorded_coverage * 100)
                
                # 触发自动重启
                self.trigger_auto_restart(elapsed_time)
    
    def trigger_auto_restart(self, elapsed_time):
        """触发自动重启机制"""
        try:
            rospy.logwarn("=== 触发自动重启机制 ===")
            rospy.logwarn("原因: 覆盖率停滞超过 %.1f 秒", self.coverage_stagnation_threshold)
            rospy.logwarn("当前运行时间: %.1f 秒", elapsed_time)
            rospy.logwarn("当前覆盖率: %.3f%%", self.coverage_percentage)
            
            # 保存当前状态报告
            self.save_restart_report(elapsed_time)
            
            # 发布重启信号到话题
            self.publish_restart_signal()
            
            # 稍等片刻确保数据保存
            rospy.logwarn("正在保存数据并准备重启...")
            time.sleep(2.0)
            
            # 执行重启
            self.execute_restart()
            
        except Exception as e:
            rospy.logerr("自动重启过程中出错: %s", str(e))
    
    def save_restart_report(self, elapsed_time):
        """保存重启前的状态报告"""
        try:
            restart_report = f"""=== 自动重启报告 ===
触发时间: {time.strftime('%Y-%m-%d %H:%M:%S')}
重启原因: 覆盖率停滞超过 {self.coverage_stagnation_threshold:.1f} 秒
运行时间: {elapsed_time:.1f} 秒 ({elapsed_time/60:.1f} 分钟)
最终覆盖率: {self.coverage_percentage:.3f}%
已清扫面积: {self.covered_area:.2f} m²
路径长度: {self.path_length:.2f} 米
路径点数: {len(self.path_history)}
碰撞次数: {self.collision_count}

重启前性能指标:
- 运动效率: {self.path_length/self.covered_area if self.covered_area > 0 else 0:.3f} m/m²
- 清洁冗余度: {self.redundant_area/(self.covered_area + self.redundant_area) if (self.covered_area + self.redundant_area) > 0 else 0:.3f}
- 平均速度: {np.mean(list(self.velocity_history)) if self.velocity_history else 0:.3f} m/s

即将执行重启命令: {self.restart_command}
========================"""
            
            # 保存重启报告
            restart_filename = f"/home/getting/tmp/auto_restart_report_{int(time.time())}.txt"
            with open(restart_filename, 'w', encoding='utf-8') as f:
                f.write(restart_report)
            
            rospy.logwarn("重启报告已保存: %s", restart_filename)
            print(restart_report)  # 同时输出到控制台
            
        except Exception as e:
            rospy.logerr("保存重启报告失败: %s", str(e))
    
    def publish_restart_signal(self):
        """发布重启信号到ROS话题"""
        try:
            # 这里可以发布一个自定义消息通知其他节点即将重启
            rospy.loginfo("发布重启信号到ROS系统...")
            
            # 可以添加一个重启信号发布器
            # restart_msg = std_msgs.msg.String()
            # restart_msg.data = "auto_restart_triggered"
            # self.restart_signal_pub.publish(restart_msg)
            
        except Exception as e:
            rospy.logerr("发布重启信号失败: %s", str(e))
    
    def execute_restart(self):
        """执行系统重启"""
        import subprocess
        import signal
        import os
        
        try:
            rospy.logwarn("开始执行自动重启...")
            
            # 首先优雅地关闭当前所有ROS节点
            rospy.logwarn("正在关闭当前ROS节点...")
            
            # 发送关闭信号给所有相关节点
            try:
                subprocess.run(["rosnode", "kill", "-a"], timeout=10, check=False)
                rospy.loginfo("ROS节点关闭命令已发送")
            except Exception as e:
                rospy.logwarn("关闭ROS节点时出错: %s", str(e))
            
            # 等待节点关闭
            time.sleep(3.0)
            
            # 创建重启脚本
            restart_script = f"""#!/bin/bash
echo "=== 自动重启脚本开始执行 ==="
echo "时间: $(date)"
echo "原因: 覆盖率停滞超过 {self.coverage_stagnation_threshold:.1f} 秒"

# 等待确保之前的进程完全关闭
sleep 5

# 设置ROS环境
cd /home/getting/Sweeping-Robot
source devel/setup.bash

echo "启动新的清扫任务..."
echo "命令: {self.restart_command}"

# 启动新的清扫任务
{self.restart_command}
"""
            
            script_path = "/home/getting/tmp/auto_restart_script.sh"
            with open(script_path, 'w') as f:
                f.write(restart_script)
            
            # 添加执行权限
            os.chmod(script_path, 0o755)
            
            rospy.logwarn("重启脚本已创建: %s", script_path)
            rospy.logwarn("即将在新进程中执行重启...")
            
            # 在新的终端中执行重启脚本
            subprocess.Popen([
                "gnome-terminal", "--", "bash", "-c", 
                f"echo '自动重启执行中...'; {script_path}; read -p '按回车继续...'"
            ])
            
            # 延时后关闭当前程序
            rospy.logwarn("重启命令已启动，当前程序将在5秒后退出...")
            time.sleep(5.0)
            
            # 优雅地关闭当前程序
            rospy.signal_shutdown("自动重启触发")
            
        except Exception as e:
            rospy.logerr("执行重启失败: %s", str(e))
            rospy.logerr("请手动重启系统")

    def calculate_evaluation_metrics(self, elapsed_time):
        """计算所有评估指标"""
        metrics = {}
        
        # CR: 覆盖率
        metrics['CR'] = self.coverage_percentage / 100.0
        
        # ME: 运动效率 (总移动距离 / 覆盖面积)
        if self.covered_area > 0:
            metrics['ME'] = self.path_length / self.covered_area
        else:
            metrics['ME'] = float('inf')
        
        # SR: 清洁冗余度 (重复清扫面积 / 总清扫面积)
        total_cleaned_area = self.covered_area + self.redundant_area
        if total_cleaned_area > 0:
            metrics['SR'] = self.redundant_area / total_cleaned_area
        else:
            metrics['SR'] = 0.0
        
        # Collision: 碰撞次数
        metrics['Collision'] = self.collision_count
        
        # CT: 平均计算时间
        if self.computation_times:
            metrics['CT'] = np.mean(self.computation_times)
        else:
            metrics['CT'] = 0.0
        
        # FT: 任务总耗时
        metrics['FT'] = elapsed_time
        
        # Vel_avg: 平均速度
        if self.velocity_history:
            metrics['Vel_avg'] = np.mean(list(self.velocity_history))
        else:
            metrics['Vel_avg'] = 0.0
        
        # Acc_avg: 平均加速度
        if self.acceleration_history:
            metrics['Acc_avg'] = np.mean([abs(a) for a in self.acceleration_history])
        else:
            metrics['Acc_avg'] = 0.0
        
        # Jerk_avg: 平均加加速度 (加速度变化率)
        if len(self.acceleration_history) >= 2:
            jerks = []
            accs = list(self.acceleration_history)
            for i in range(1, len(accs)):
                jerk = abs(accs[i] - accs[i-1]) * self.update_rate
                jerks.append(jerk)
            metrics['Jerk_avg'] = np.mean(jerks) if jerks else 0.0
        else:
            metrics['Jerk_avg'] = 0.0
        
        # 规划路径点总数
        metrics['Planned_Points'] = self.planned_path_points
        
        return metrics

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