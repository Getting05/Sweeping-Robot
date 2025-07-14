#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
扫地机器人评估指标收集器
基于图片中的指标体系设计，监控并保存机器人清扫性能指标

终止条件：
1. 到达最后一个路径点后覆盖率下降
2. 30秒内覆盖率增长不超过1%

保存指标：CR, ME, SR, Collision, CT, FT, Vel_avg, Acc_avg, Jerk_avg
"""

import rospy
import numpy as np
import time
import os
import math
import csv
from datetime import datetime
from collections import deque
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
import json

# 尝试导入pandas，如果没有则使用内置csv模块
try:
    import pandas as pd
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False
    rospy.logwarn("pandas未安装，将使用内置csv模块")


class SweepingRobotEvaluator:
    def __init__(self):
        rospy.init_node('sweeping_robot_evaluator', anonymous=True)
        
        # 基本参数
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
        
        # 覆盖率相关指标
        self.covered_grid = None  # 已覆盖区域栅格
        self.covered_area = 0     # 已覆盖面积
        self.coverage_percentage = 0.0
        self.redundant_area = 0   # 重复清扫面积
        self.coverage_history = deque(maxlen=1000)  # 覆盖率历史
        
        # 运动相关指标
        self.path_history = deque(maxlen=20000)  # 完整路径历史
        self.velocity_history = deque(maxlen=1000)  # 速度历史
        self.acceleration_history = deque(maxlen=1000)  # 加速度历史
        self.path_length = 0.0    # 总路径长度
        self.last_position = None
        self.last_velocity = None
        self.last_time = None
        
        # 计算时间和任务时间
        self.start_time = time.time()
        self.computation_times = []  # 计算时间记录
        self.last_computation_start = None
        
        # 碰撞检测
        self.collision_count = 0
        self.last_position_for_collision = None
        self.stuck_threshold = 0.05  # 卡住检测阈值
        self.stuck_time_threshold = 3.0  # 卡住时间阈值
        self.stuck_start_time = None
        
        # 路径规划相关
        self.planned_path = []
        self.total_goals = 0
        self.completed_goals = 0
        self.current_goal_index = 0
        self.last_path_size = 0
        
        # 终止条件监控
        self.coverage_stagnation_start = None
        self.coverage_stagnation_threshold = 30.0  # 30秒
        self.coverage_growth_threshold = 0.01  # 1%
        self.last_coverage_check = 0.0
        self.path_completed = False
        self.evaluation_completed = False
        
        # 输出目录
        self.output_dir = "/home/getting/Sweeping-Robot/evaluation_results"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 数据记录
        self.step_data = []
        self.step_count = 0
        
        # ROS订阅器
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        self.path_sub = rospy.Subscriber('/passedPath', Path, self.path_callback, queue_size=1)
        self.plan_sub = rospy.Subscriber('/plan_path', Path, self.plan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback, queue_size=1)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback, queue_size=1)
        
        # ROS发布器
        self.coverage_pub = rospy.Publisher('/coverage_percentage', Float32, queue_size=1)
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_evaluation)
        
        rospy.loginfo("=== 扫地机器人评估系统启动 ===")
        rospy.loginfo("输出目录: %s", self.output_dir)
        
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
            
            rospy.loginfo("地图加载完成: %dx%d, 分辨率=%.3f, 自由面积=%.2f m²",
                          self.map_width, self.map_height, self.map_resolution, self.free_area_total)
                         
        except Exception as e:
            rospy.logerr("地图处理错误: %s", str(e))
    
    def plan_callback(self, msg):
        """处理规划路径"""
        if msg.poses:
            self.planned_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
            self.total_goals = len(self.planned_path)
            rospy.loginfo("规划路径加载: %d 个目标点", self.total_goals)
    
    def path_callback(self, msg):
        """处理已走过的路径"""
        if not msg.poses or self.map_data is None:
            return
            
        try:
            current_path_length = len(msg.poses)
            
            # 检测路径重置
            if current_path_length < self.last_path_size:
                rospy.loginfo("路径重置检测，清空历史")
                self.path_history.clear()
                self.covered_grid = np.zeros_like(self.map_data, dtype=np.uint8)
                self.last_path_size = 0
            
            # 处理新增路径点
            for i in range(self.last_path_size, current_path_length):
                pose = msg.poses[i]
                world_x = pose.pose.position.x
                world_y = pose.pose.position.y
                
                # 检查点间距
                if self.last_position is not None:
                    dist = math.sqrt((world_x - self.last_position[0])**2 +
                                    (world_y - self.last_position[1])**2)
                    if dist < self.min_point_distance:
                        continue
                
                # 转换为栅格坐标并标记覆盖
                grid_x, grid_y = self.world_to_grid(world_x, world_y)
                if self.is_valid_grid_point(grid_x, grid_y):
                    self.path_history.append((world_x, world_y, time.time()))
                    self.last_position = (world_x, world_y)
                    self.mark_covered_area(grid_x, grid_y)
            
            self.last_path_size = current_path_length
            
            # 计算路径长度
            if len(self.path_history) > 1:
                self.path_length = self.calculate_path_length()
                
        except Exception as e:
            rospy.logerr("路径处理错误: %s", str(e))
    
    def odom_callback(self, msg):
        """处理里程计数据"""
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
            
            self.last_position = position
            self.last_time = current_time
            
        except Exception as e:
            rospy.logerr("里程计处理错误: %s", str(e))
    
    def goal_callback(self, msg):
        """处理新目标"""
        self.start_computation_timer()
    
    def result_callback(self, msg):
        """处理目标完成结果"""
        self.end_computation_timer()
        if msg.status.status == 3:  # SUCCEEDED
            self.completed_goals += 1
            rospy.loginfo("目标完成: %d/%d", self.completed_goals, self.total_goals)
            
            # 检查是否完成所有目标
            if self.completed_goals >= self.total_goals:
                self.path_completed = True
                rospy.loginfo("所有路径点已完成！")
    
    def detect_collision(self, position, current_time):
        """检测碰撞（通过卡住检测）"""
        if self.last_position_for_collision is not None:
            dist = math.sqrt((position[0] - self.last_position_for_collision[0])**2 +
                           (position[1] - self.last_position_for_collision[1])**2)
            
            if dist < self.stuck_threshold:
                if self.stuck_start_time is None:
                    self.stuck_start_time = current_time
                elif current_time - self.stuck_start_time > self.stuck_time_threshold:
                    self.collision_count += 1
                    rospy.logwarn("检测到碰撞/卡住事件，总计: %d", self.collision_count)
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
    
    def world_to_grid(self, world_x, world_y):
        """世界坐标转栅格坐标"""
        grid_x = int((world_x - self.map_origin_x) / self.map_resolution)
        grid_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return grid_x, grid_y
    
    def is_valid_grid_point(self, grid_x, grid_y):
        """检查栅格点是否有效"""
        return (0 <= grid_x < self.map_width and 
                0 <= grid_y < self.map_height and
                self.map_data[grid_y, grid_x] == 0)
    
    def mark_covered_area(self, center_x, center_y):
        """标记覆盖区域"""
        try:
            radius_cells = int(self.robot_radius / self.map_resolution)
            
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx*dx + dy*dy <= radius_cells*radius_cells:
                        grid_x = center_x + dx
                        grid_y = center_y + dy
                        
                        if self.is_valid_grid_point(grid_x, grid_y):
                            if self.covered_grid[grid_y, grid_x] == 100:
                                self.redundant_area += self.map_resolution ** 2
                            else:
                                self.covered_grid[grid_y, grid_x] = 100
                                self.covered_area += self.map_resolution ** 2
                                
        except Exception as e:
            rospy.logerr("标记覆盖区域错误: %s", str(e))
    
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
    
    def calculate_current_metrics(self):
        """计算当前所有指标"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # CR: 覆盖率
        cr = self.covered_area / self.free_area_total if self.free_area_total > 0 else 0.0
        
        # ME: 运动效率 (总移动距离 / 覆盖面积)
        me = self.path_length / self.covered_area if self.covered_area > 0 else float('inf')
        
        # SR: 清洁冗余度 (重复清扫面积 / 总清扫面积)
        total_cleaned_area = self.covered_area + self.redundant_area
        sr = self.redundant_area / total_cleaned_area if total_cleaned_area > 0 else 0.0
        
        # Collision: 碰撞次数
        collision = self.collision_count
        
        # CT: 平均计算时间
        ct = np.mean(self.computation_times) if self.computation_times else 0.0
        
        # FT: 任务总耗时
        ft = elapsed_time
        
        # Vel_avg: 平均速度
        vel_avg = np.mean(list(self.velocity_history)) if self.velocity_history else 0.0
        
        # Acc_avg: 平均加速度
        acc_avg = np.mean([abs(a) for a in self.acceleration_history]) if self.acceleration_history else 0.0
        
        # Jerk_avg: 平均加加速度 (加速度变化率)
        jerk_avg = 0.0
        if len(self.acceleration_history) >= 2:
            jerks = []
            accs = list(self.acceleration_history)
            for i in range(1, len(accs)):
                jerk = abs(accs[i] - accs[i-1]) * self.update_rate  # 简化计算
                jerks.append(jerk)
            jerk_avg = np.mean(jerks) if jerks else 0.0
        
        return {
            'timestamp': elapsed_time,
            'step': self.step_count,
            'CR': cr,
            'ME': me,
            'SR': sr,
            'Collision': collision,
            'CT': ct,
            'FT': ft,
            'Vel_avg': vel_avg,
            'Acc_avg': acc_avg,
            'Jerk_avg': jerk_avg,
            'covered_area': self.covered_area,
            'path_length': self.path_length,
            'completed_goals': self.completed_goals,
            'total_goals': self.total_goals
        }
    
    def check_termination_conditions(self):
        """检查终止条件"""
        current_coverage = self.covered_area / self.free_area_total if self.free_area_total > 0 else 0.0
        
        # 条件1: 路径完成后覆盖率下降
        if self.path_completed and len(self.coverage_history) >= 2:
            if current_coverage < self.coverage_history[-1]:
                rospy.loginfo("终止条件1满足：路径完成后覆盖率下降")
                return True
        
        # 条件2: 30秒内覆盖率增长不超过1%
        if len(self.coverage_history) >= 2:
            if self.coverage_stagnation_start is None:
                # 检查覆盖率增长
                coverage_growth = current_coverage - self.last_coverage_check
                if coverage_growth <= self.coverage_growth_threshold:
                    self.coverage_stagnation_start = time.time()
                    rospy.loginfo("开始监控覆盖率停滞")
                else:
                    self.last_coverage_check = current_coverage
            else:
                # 检查停滞时间
                stagnation_time = time.time() - self.coverage_stagnation_start
                if stagnation_time >= self.coverage_stagnation_threshold:
                    rospy.loginfo("终止条件2满足：30秒内覆盖率增长不超过1%%")
                    return True
                
                # 如果覆盖率有显著增长，重置停滞计时
                coverage_growth = current_coverage - self.last_coverage_check
                if coverage_growth > self.coverage_growth_threshold:
                    self.coverage_stagnation_start = None
                    self.last_coverage_check = current_coverage
                    rospy.loginfo("覆盖率增长恢复，重置停滞计时")
        
        # 记录覆盖率历史
        self.coverage_history.append(current_coverage)
        
        return False
    
    def update_evaluation(self, event):
        """更新评估"""
        if self.evaluation_completed or self.map_data is None:
            return
        
        try:
            # 计算当前指标
            metrics = self.calculate_current_metrics()
            self.step_data.append(metrics)
            self.step_count += 1
            
            # 发布覆盖率
            coverage_msg = Float32()
            coverage_msg.data = metrics['CR']
            self.coverage_pub.publish(coverage_msg)
            
            # 检查终止条件
            if self.check_termination_conditions():
                self.save_evaluation_results()
                self.evaluation_completed = True
                rospy.loginfo("评估完成，正在关闭节点...")
                rospy.signal_shutdown("评估完成")
            
            # 定期打印状态
            if self.step_count % 10 == 0:
                rospy.loginfo("评估状态 - 覆盖率: %.2f%%, 路径长度: %.2fm, 碰撞: %d",
                             metrics['CR']*100, metrics['path_length'], metrics['Collision'])
                
        except Exception as e:
            rospy.logerr("评估更新错误: %s", str(e))
    
    def save_evaluation_results(self):
        """保存评估结果到CSV"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # 保存详细步骤数据
            if self.step_data:
                step_file = os.path.join(self.output_dir, f"sweeping_metrics_steps_{timestamp}.csv")
                
                if HAS_PANDAS:
                    # 使用pandas保存
                    df_steps = pd.DataFrame(self.step_data)
                    df_steps.to_csv(step_file, index=False)
                else:
                    # 使用内置csv模块保存
                    with open(step_file, 'w', newline='', encoding='utf-8') as csvfile:
                        if self.step_data:
                            fieldnames = self.step_data[0].keys()
                            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                            writer.writeheader()
                            writer.writerows(self.step_data)
                
                rospy.loginfo("步骤指标已保存: %s", step_file)
            
            # 保存最终汇总指标
            final_metrics = self.calculate_current_metrics()
            
            summary_data = {
                '实验时间': timestamp,
                '覆盖率_CR': final_metrics['CR'],
                '运动效率_ME': final_metrics['ME'],
                '清洁冗余度_SR': final_metrics['SR'],
                '碰撞次数_Collision': final_metrics['Collision'],
                '平均计算时间_CT': final_metrics['CT'],
                '任务总耗时_FT': final_metrics['FT'],
                '平均速度_Vel_avg': final_metrics['Vel_avg'],
                '平均加速度_Acc_avg': final_metrics['Acc_avg'],
                '平均加加速度_Jerk_avg': final_metrics['Jerk_avg'],
                '已覆盖面积_m2': final_metrics['covered_area'],
                '总路径长度_m': final_metrics['path_length'],
                '完成目标数': final_metrics['completed_goals'],
                '总目标数': final_metrics['total_goals']
            }
            
            summary_file = os.path.join(self.output_dir, f"sweeping_metrics_summary_{timestamp}.csv")
            
            if HAS_PANDAS:
                # 使用pandas保存
                df_summary = pd.DataFrame([summary_data])
                df_summary.to_csv(summary_file, index=False)
            else:
                # 使用内置csv模块保存
                with open(summary_file, 'w', newline='', encoding='utf-8') as csvfile:
                    fieldnames = summary_data.keys()
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerow(summary_data)
            
            rospy.loginfo("汇总指标已保存: %s", summary_file)
            
            # 保存JSON格式的详细报告
            report = {
                'experiment_info': {
                    'timestamp': timestamp,
                    'duration': final_metrics['FT'],
                    'robot_radius': self.robot_radius,
                    'map_size': f"{self.map_width}x{self.map_height}",
                    'map_resolution': self.map_resolution,
                    'free_area_total': self.free_area_total
                },
                'final_metrics': final_metrics,
                'performance_summary': {
                    'coverage_efficiency': final_metrics['CR'],
                    'motion_efficiency': final_metrics['ME'],
                    'redundancy_rate': final_metrics['SR'],
                    'collision_rate': final_metrics['Collision'] / final_metrics['FT'] if final_metrics['FT'] > 0 else 0,
                    'average_speed': final_metrics['Vel_avg']
                }
            }
            
            report_file = os.path.join(self.output_dir, f"sweeping_evaluation_report_{timestamp}.json")
            with open(report_file, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
            rospy.loginfo("评估报告已保存: %s", report_file)
            
            self.print_final_summary(final_metrics)
            
        except Exception as e:
            rospy.logerr("保存评估结果错误: %s", str(e))
    
    def print_final_summary(self, metrics):
        """打印最终评估摘要"""
        rospy.loginfo("=" * 50)
        rospy.loginfo("扫地机器人性能评估完成")
        rospy.loginfo("=" * 50)
        rospy.loginfo("覆盖率 (CR): %.2f%%", metrics['CR'] * 100)
        rospy.loginfo("运动效率 (ME): %.3f m/m²", metrics['ME'])
        rospy.loginfo("清洁冗余度 (SR): %.2f%%", metrics['SR'] * 100)
        rospy.loginfo("碰撞次数: %d", metrics['Collision'])
        rospy.loginfo("平均计算时间: %.3f s", metrics['CT'])
        rospy.loginfo("任务总耗时: %.1f s", metrics['FT'])
        rospy.loginfo("平均速度: %.3f m/s", metrics['Vel_avg'])
        rospy.loginfo("平均加速度: %.3f m/s²", metrics['Acc_avg'])
        rospy.loginfo("平均加加速度: %.3f m/s³", metrics['Jerk_avg'])
        rospy.loginfo("=" * 50)


def main():
    try:
        evaluator = SweepingRobotEvaluator()
        rospy.loginfo("扫地机器人评估系统准备就绪...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("评估系统关闭")
    except Exception as e:
        rospy.logerr("评估系统错误: %s", str(e))


if __name__ == '__main__':
    main()
