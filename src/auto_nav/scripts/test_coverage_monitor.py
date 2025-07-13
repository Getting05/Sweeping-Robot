#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
覆盖率监控测试脚本
用于测试覆盖率计算功能
"""

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid

class CoverageTestMonitor:
    def __init__(self):
        rospy.init_node('coverage_test_monitor', anonymous=True)
        
        # 订阅覆盖率话题
        self.coverage_sub = rospy.Subscriber('/coverage_percentage', Float32, self.coverage_callback)
        self.covered_grid_sub = rospy.Subscriber('/covered_area_grid', OccupancyGrid, self.covered_grid_callback)
        
        self.latest_coverage = 0.0
        self.grid_received = False
        
        rospy.loginfo("Coverage Test Monitor started")
    
    def coverage_callback(self, msg):
        """接收覆盖率数据"""
        self.latest_coverage = msg.data
        rospy.loginfo("实时覆盖率: %.2f%%", self.latest_coverage)
    
    def covered_grid_callback(self, msg):
        """接收覆盖区域栅格数据"""
        if not self.grid_received:
            rospy.loginfo("接收到覆盖区域栅格地图: %dx%d, 分辨率: %.3f", 
                         msg.info.width, msg.info.height, msg.info.resolution)
            self.grid_received = True
    
    def print_status(self):
        """打印状态信息"""
        rospy.loginfo("=== 覆盖率监控状态 ===")
        rospy.loginfo("当前覆盖率: %.2f%%", self.latest_coverage)
        rospy.loginfo("栅格地图状态: %s", "已接收" if self.grid_received else "未接收")
        rospy.loginfo("===================")

def main():
    try:
        monitor = CoverageTestMonitor()
        
        rate = rospy.Rate(0.5)  # 2秒打印一次状态
        
        while not rospy.is_shutdown():
            monitor.print_status()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Coverage test monitor shutdown")

if __name__ == '__main__':
    main()
