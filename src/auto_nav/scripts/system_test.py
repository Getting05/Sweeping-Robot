#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
系统完整性测试脚本
测试清扫机器人的覆盖率监控系统和路径可视化
"""

import rospy
import sys
from std_msgs.msg import Float32
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

class SystemTester:
    def __init__(self):
        rospy.init_node('system_tester', anonymous=True)
        
        # 测试统计
        self.coverage_received = False
        self.path_received = False
        self.covered_grid_received = False
        self.cleaned_path_received = False
        
        # 最新数据
        self.latest_coverage = 0.0
        self.path_length = 0
        self.cleaned_path_length = 0
        
        # 订阅话题
        self.coverage_sub = rospy.Subscriber('/coverage_percentage', Float32, self.coverage_callback)
        self.path_sub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        self.cleaned_path_sub = rospy.Subscriber('/cleaned_path', Path, self.cleaned_path_callback)
        self.covered_grid_sub = rospy.Subscriber('/covered_area_grid', OccupancyGrid, self.covered_grid_callback)
        
        rospy.loginfo("系统测试器已启动，开始监控各个话题...")
        rospy.loginfo("预期话题：")
        rospy.loginfo("  /coverage_percentage - 覆盖率百分比")
        rospy.loginfo("  /move_base/NavfnROS/plan - 当前路径规划")
        rospy.loginfo("  /cleaned_path - 已清扫路径（红线）")
        rospy.loginfo("  /covered_area_grid - 已覆盖区域栅格地图")
        
    def coverage_callback(self, msg):
        self.coverage_received = True
        self.latest_coverage = msg.data
        
    def path_callback(self, msg):
        self.path_received = True
        self.path_length = len(msg.poses)
        
    def cleaned_path_callback(self, msg):
        self.cleaned_path_received = True
        self.cleaned_path_length = len(msg.poses)
        
    def covered_grid_callback(self, msg):
        self.covered_grid_received = True
        
    def print_status(self):
        """打印当前系统状态"""
        print("\n" + "="*60)
        print("清扫机器人系统状态报告")
        print("="*60)
        
        # 话题接收状态
        print("话题接收状态:")
        print(f"  覆盖率话题 (/coverage_percentage): {'✓' if self.coverage_received else '✗'}")
        print(f"  路径规划话题 (/move_base/NavfnROS/plan): {'✓' if self.path_received else '✗'}")
        print(f"  已清扫路径话题 (/cleaned_path): {'✓' if self.cleaned_path_received else '✗'}")
        print(f"  覆盖区域栅格话题 (/covered_area_grid): {'✓' if self.covered_grid_received else '✗'}")
        
        # 数据统计
        print("\n实时数据:")
        print(f"  当前覆盖率: {self.latest_coverage:.2f}%")
        print(f"  当前路径点数: {self.path_length}")
        print(f"  历史清扫路径点数: {self.cleaned_path_length}")
        
        # 系统健康度评估
        all_topics_active = all([
            self.coverage_received,
            self.path_received, 
            self.cleaned_path_received,
            self.covered_grid_received
        ])
        
        print(f"\n系统健康度: {'健康' if all_topics_active else '部分功能缺失'}")
        
        if not all_topics_active:
            print("建议检查:")
            if not self.coverage_received:
                print("  - coverage_monitor.py脚本是否正在运行")
            if not self.path_received:
                print("  - move_base导航是否正常工作")
            if not self.cleaned_path_received:
                print("  - sequential_goal节点是否正常运行")
            if not self.covered_grid_received:
                print("  - coverage_monitor.py的栅格地图发布是否正常")
        
        print("="*60)
        
    def run_test(self, duration=30):
        """运行测试指定时间"""
        rospy.loginfo(f"开始运行系统测试，持续时间: {duration}秒")
        
        rate = rospy.Rate(1)  # 1Hz
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed = (current_time - start_time).to_sec()
            
            if elapsed >= duration:
                break
                
            # 每5秒打印一次状态
            if int(elapsed) % 5 == 0:
                self.print_status()
                
            rate.sleep()
            
        # 最终报告
        rospy.loginfo("测试完成，生成最终报告:")
        self.print_status()
        
        return all([
            self.coverage_received,
            self.path_received,
            self.cleaned_path_received,
            self.covered_grid_received
        ])

def main():
    try:
        tester = SystemTester()
        
        # 设置测试时间（默认30秒）
        test_duration = 30
        if len(sys.argv) > 1:
            try:
                test_duration = int(sys.argv[1])
            except ValueError:
                rospy.logwarn("无效的测试时间参数，使用默认值30秒")
        
        # 运行测试
        success = tester.run_test(test_duration)
        
        if success:
            rospy.loginfo("系统测试通过！所有核心功能正常工作。")
            sys.exit(0)
        else:
            rospy.logwarn("系统测试未完全通过，请检查上述建议。")
            sys.exit(1)
            
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被用户中断")
        sys.exit(0)
    except Exception as e:
        rospy.logerr(f"测试过程中发生错误: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
