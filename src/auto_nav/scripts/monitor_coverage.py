#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
实时覆盖率监控终端工具
用于在单独终端中监控清扫机器人的覆盖率统计
"""

import rospy
import sys
import time
from std_msgs.msg import Float32
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

class CoverageTerminalMonitor:
    def __init__(self):
        rospy.init_node('coverage_terminal_monitor', anonymous=True)
        
        # 数据存储
        self.coverage_percentage = 0.0
        self.path_length = 0
        self.cleaned_path_length = 0
        self.start_time = time.time()
        
        # 状态标志
        self.coverage_received = False
        self.path_received = False
        self.cleaned_path_received = False
        self.covered_grid_received = False
        
        # 订阅话题
        self.coverage_sub = rospy.Subscriber('/coverage_percentage', Float32, self.coverage_callback)
        self.path_sub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        self.cleaned_path_sub = rospy.Subscriber('/passedPath', Path, self.cleaned_path_callback)
        self.covered_grid_sub = rospy.Subscriber('/covered_area_grid', OccupancyGrid, self.covered_grid_callback)
        
        rospy.loginfo("覆盖率监控器已启动")
        print("\n" + "="*60)
        print("清扫机器人覆盖率实时监控")
        print("="*60)
        print("监控话题:")
        print("  /coverage_percentage - 覆盖率百分比")
        print("  /cleaned_path - 已清扫路径")
        print("  /move_base/NavfnROS/plan - 当前规划路径")
        print("  /covered_area_grid - 覆盖区域栅格")
        print("="*60)
        
    def coverage_callback(self, msg):
        self.coverage_received = True
        self.coverage_percentage = msg.data
        
    def path_callback(self, msg):
        self.path_received = True
        self.path_length = len(msg.poses)
        
    def cleaned_path_callback(self, msg):
        self.cleaned_path_received = True
        self.cleaned_path_length = len(msg.poses)
        
    def covered_grid_callback(self, msg):
        self.covered_grid_received = True
        
    def print_status(self):
        """清屏并打印当前状态"""
        # 清屏
        print('\033[2J\033[H', end='')
        
        # 计算运行时间
        elapsed = time.time() - self.start_time
        
        print("="*60)
        print("清扫机器人覆盖率实时监控")
        print("="*60)
        
        # 显示时间信息
        print(f"运行时间: {elapsed:.1f} 秒 ({elapsed/60:.1f} 分钟)")
        print(f"当前时间: {time.strftime('%H:%M:%S')}")
        print("")
        
        # 显示连接状态
        print("话题连接状态:")
        print(f"  覆盖率监控: {'✓ 已连接' if self.coverage_received else '✗ 未连接'}")
        print(f"  路径规划: {'✓ 已连接' if self.path_received else '✗ 未连接'}")
        print(f"  已清扫路径: {'✓ 已连接' if self.cleaned_path_received else '✗ 未连接'}")
        print(f"  覆盖栅格: {'✓ 已连接' if self.covered_grid_received else '✗ 未连接'}")
        print("")
        
        # 显示核心数据
        print("实时覆盖率数据:")
        print(f"  当前覆盖率: {self.coverage_percentage:.2f}%")
        
        # 进度条显示
        bar_length = 40
        filled_length = int(bar_length * self.coverage_percentage / 100)
        bar = '█' * filled_length + '░' * (bar_length - filled_length)
        print(f"  进度条: [{bar}] {self.coverage_percentage:.1f}%")
        print("")
        
        # 显示路径信息
        print("路径统计:")
        print(f"  当前规划路径点数: {self.path_length}")
        print(f"  历史清扫路径点数: {self.cleaned_path_length}")
        
        if self.cleaned_path_length > 0:
            # 估算路径长度（假设每个点间距0.1米）
            estimated_length = self.cleaned_path_length * 0.1
            print(f"  估算清扫路径长度: {estimated_length:.1f} 米")
            
            if elapsed > 0:
                speed = estimated_length / elapsed
                print(f"  平均清扫速度: {speed:.2f} m/s")
        print("")
        
        # 显示效率指标
        if elapsed > 0 and self.coverage_percentage > 0:
            coverage_rate = self.coverage_percentage / elapsed
            print("效率指标:")
            print(f"  覆盖率增长速度: {coverage_rate:.3f} %/秒")
            print(f"  预计完成时间: {(100 - self.coverage_percentage) / coverage_rate / 60:.1f} 分钟")
        
        print("="*60)
        print("按 Ctrl+C 退出监控")
        
    def monitor_loop(self):
        """主监控循环"""
        rate = rospy.Rate(2)  # 2Hz更新频率
        
        while not rospy.is_shutdown():
            self.print_status()
            rate.sleep()

def main():
    try:
        monitor = CoverageTerminalMonitor()
        
        # 等待一秒让连接建立
        rospy.sleep(1.0)
        
        # 开始监控循环
        monitor.monitor_loop()
        
    except rospy.ROSInterruptException:
        print("\n监控已停止")
    except KeyboardInterrupt:
        print("\n用户中断，监控已停止")
    except Exception as e:
        print(f"\n监控过程中发生错误: {e}")

if __name__ == '__main__':
    main()
