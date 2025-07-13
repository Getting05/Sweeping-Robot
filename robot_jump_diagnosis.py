#!/usr/bin/env python3
"""
深度诊断机器人瞬移问题的脚本
分析TF变换链、坐标系一致性、时间戳同步等关键问题
"""

import rospy
import tf
import tf2_ros
import threading
import time
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import matplotlib.pyplot as plt
from collections import deque
import yaml
import os

class RobotJumpDiagnostic:
    def __init__(self):
        rospy.init_node('robot_jump_diagnostic', anonymous=True)
        
        # TF监听器
        self.tf_listener = tf.TransformListener()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener2 = tf2_ros.TransformListener(self.tf_buffer)
        
        # 数据收集
        self.odom_data = deque(maxlen=1000)
        self.amcl_data = deque(maxlen=1000) 
        self.goal_data = deque(maxlen=100)
        self.cmd_vel_data = deque(maxlen=500)
        self.tf_data = deque(maxlen=1000)
        
        # 诊断标志
        self.jump_detected = False
        self.tf_discontinuity = False
        self.coord_mismatch = False
        
        # 阈值设置
        self.position_jump_threshold = 0.5  # 位置跳跃阈值(m)
        self.velocity_threshold = 2.0       # 速度异常阈值(m/s)
        self.timestamp_tolerance = 0.1      # 时间戳容差(s)
        
        # 订阅器
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseStamped, self.amcl_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # 发布器
        self.marker_pub = rospy.Publisher('/diagnostic_markers', MarkerArray, queue_size=10)
        
        # 结果记录
        self.diagnostic_results = {
            'tf_chain_issues': [],
            'coordinate_mismatches': [],
            'timestamp_issues': [],
            'velocity_anomalies': [],
            'position_jumps': [],
            'frame_id_issues': []
        }
        
        print("机器人瞬移诊断系统已启动...")
        print("监控指标：")
        print("- TF变换链连续性")
        print("- 坐标系frame_id一致性")
        print("- 时间戳同步") 
        print("- 位置/速度异常跳跃")
        print("- 目标点坐标系匹配")
        
    def odom_callback(self, msg):
        """里程计数据回调"""
        current_time = rospy.Time.now()
        
        odom_info = {
            'timestamp': msg.header.stamp,
            'frame_id': msg.header.frame_id,
            'position': [msg.pose.pose.position.x, msg.pose.pose.position.y],
            'orientation': [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w],
            'linear_vel': [msg.twist.twist.linear.x, msg.twist.twist.linear.y],
            'angular_vel': msg.twist.twist.angular.z,
            'receive_time': current_time
        }
        
        self.odom_data.append(odom_info)
        
        # 检测位置跳跃
        if len(self.odom_data) >= 2:
            self.check_position_jump(self.odom_data[-2], self.odom_data[-1])
            
        # 检测frame_id一致性
        self.check_frame_id_consistency('odom', msg.header.frame_id)
    
    def amcl_callback(self, msg):
        """AMCL位姿回调"""
        current_time = rospy.Time.now()
        
        amcl_info = {
            'timestamp': msg.header.stamp,
            'frame_id': msg.header.frame_id,
            'position': [msg.pose.position.x, msg.pose.position.y],
            'orientation': [msg.pose.orientation.x, msg.pose.orientation.y,
                          msg.pose.orientation.z, msg.pose.orientation.w],
            'receive_time': current_time
        }
        
        self.amcl_data.append(amcl_info)
        
        # 检测AMCL坐标系
        self.check_frame_id_consistency('amcl', msg.header.frame_id)
        
        # 检测AMCL与odom位置差异
        if self.odom_data:
            self.check_odom_amcl_consistency()
    
    def goal_callback(self, msg):
        """目标点回调"""
        goal_info = {
            'timestamp': msg.header.stamp,
            'frame_id': msg.header.frame_id,
            'position': [msg.pose.position.x, msg.pose.position.y],
            'receive_time': rospy.Time.now()
        }
        
        self.goal_data.append(goal_info)
        
        # 检测目标点frame_id
        self.check_frame_id_consistency('goal', msg.header.frame_id)
        
        print(f"新目标点: frame_id='{msg.header.frame_id}', "
              f"位置=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})")
    
    def cmd_vel_callback(self, msg):
        """速度命令回调"""
        vel_info = {
            'timestamp': rospy.Time.now(),
            'linear': [msg.linear.x, msg.linear.y],
            'angular': msg.angular.z
        }
        
        self.cmd_vel_data.append(vel_info)
        
        # 检测速度异常
        speed = np.sqrt(msg.linear.x**2 + msg.linear.y**2)
        if speed > self.velocity_threshold:
            self.diagnostic_results['velocity_anomalies'].append({
                'timestamp': rospy.Time.now(),
                'speed': speed,
                'cmd_vel': vel_info
            })
            print(f"警告: 检测到异常速度命令 {speed:.3f} m/s")
    
    def check_position_jump(self, prev_data, curr_data):
        """检测位置跳跃"""
        if not prev_data or not curr_data:
            return
            
        prev_pos = np.array(prev_data['position'])
        curr_pos = np.array(curr_data['position'])
        
        distance = np.linalg.norm(curr_pos - prev_pos)
        time_diff = (curr_data['timestamp'] - prev_data['timestamp']).to_sec()
        
        if time_diff > 0:
            velocity = distance / time_diff
            
            if distance > self.position_jump_threshold and velocity > self.velocity_threshold:
                jump_info = {
                    'timestamp': curr_data['timestamp'],
                    'distance': distance,
                    'velocity': velocity,
                    'prev_pos': prev_pos.tolist(),
                    'curr_pos': curr_pos.tolist(),
                    'time_diff': time_diff
                }
                
                self.diagnostic_results['position_jumps'].append(jump_info)
                self.jump_detected = True
                
                print(f"检测到位置跳跃: 距离={distance:.3f}m, 速度={velocity:.3f}m/s")
                print(f"  从 ({prev_pos[0]:.3f}, {prev_pos[1]:.3f}) 到 ({curr_pos[0]:.3f}, {curr_pos[1]:.3f})")
    
    def check_frame_id_consistency(self, topic, frame_id):
        """检查frame_id一致性"""
        expected_frames = {
            'odom': 'odom',
            'amcl': 'map', 
            'goal': 'map'  # 建议使用map而不是odom
        }
        
        if topic in expected_frames and frame_id != expected_frames[topic]:
            issue = {
                'topic': topic,
                'expected': expected_frames[topic],
                'actual': frame_id,
                'timestamp': rospy.Time.now()
            }
            
            if issue not in self.diagnostic_results['frame_id_issues']:
                self.diagnostic_results['frame_id_issues'].append(issue)
                print(f"坐标系不匹配: {topic}使用'{frame_id}', 建议使用'{expected_frames[topic]}'")
    
    def check_odom_amcl_consistency(self):
        """检查odom与amcl位置一致性"""
        if not self.odom_data or not self.amcl_data:
            return
            
        try:
            # 获取map到odom的变换
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/odom', rospy.Time(0))
            
            # 转换odom坐标到map坐标
            odom_pos = self.odom_data[-1]['position']
            transformed_pos = self.transform_point(odom_pos, trans, rot)
            
            amcl_pos = self.amcl_data[-1]['position']
            diff = np.linalg.norm(np.array(transformed_pos) - np.array(amcl_pos))
            
            if diff > 0.5:  # 差异超过0.5m
                mismatch = {
                    'timestamp': rospy.Time.now(),
                    'odom_pos': odom_pos,
                    'amcl_pos': amcl_pos,
                    'transformed_odom': transformed_pos,
                    'difference': diff
                }
                self.diagnostic_results['coordinate_mismatches'].append(mismatch)
                print(f"AMCL与odom位置差异过大: {diff:.3f}m")
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            tf_issue = {
                'timestamp': rospy.Time.now(),
                'error': str(e),
                'transform': 'map -> odom'
            }
            self.diagnostic_results['tf_chain_issues'].append(tf_issue)
            print(f"TF变换错误: {e}")
    
    def transform_point(self, point, trans, rot):
        """应用TF变换到点"""
        # 简单的2D变换
        x, y = point[0], point[1]
        tx, ty, tz = trans
        
        # 这里简化处理，只考虑平移
        return [x + tx, y + ty]
    
    def check_tf_continuity(self):
        """检查TF变换链连续性"""
        important_frames = ['/map', '/odom', '/base_footprint', '/base_link']
        
        for i in range(len(important_frames)):
            for j in range(i+1, len(important_frames)):
                frame_a = important_frames[i]
                frame_b = important_frames[j]
                
                try:
                    # 检查变换是否可用
                    self.tf_listener.waitForTransform(frame_a, frame_b, rospy.Time(), rospy.Duration(1.0))
                    (trans, rot) = self.tf_listener.lookupTransform(frame_a, frame_b, rospy.Time(0))
                    
                    # 记录变换
                    tf_info = {
                        'timestamp': rospy.Time.now(),
                        'from_frame': frame_a,
                        'to_frame': frame_b,
                        'translation': trans,
                        'rotation': rot,
                        'success': True
                    }
                    self.tf_data.append(tf_info)
                    
                except Exception as e:
                    tf_error = {
                        'timestamp': rospy.Time.now(),
                        'from_frame': frame_a,
                        'to_frame': frame_b,
                        'error': str(e),
                        'success': False
                    }
                    self.tf_data.append(tf_error)
                    self.diagnostic_results['tf_chain_issues'].append(tf_error)
                    print(f"TF链断裂: {frame_a} -> {frame_b}: {e}")
    
    def monitor_timestamps(self):
        """监控时间戳同步"""
        current_time = rospy.Time.now()
        
        # 检查各话题时间戳
        for data_source, data_queue in [('odom', self.odom_data), ('amcl', self.amcl_data)]:
            if data_queue:
                latest_data = data_queue[-1]
                msg_time = latest_data['timestamp']
                receive_time = latest_data['receive_time']
                
                # 检查消息时间戳与当前时间差异
                time_diff = abs((current_time - msg_time).to_sec())
                if time_diff > self.timestamp_tolerance:
                    timestamp_issue = {
                        'source': data_source,
                        'msg_timestamp': msg_time,
                        'current_time': current_time,
                        'difference': time_diff
                    }
                    self.diagnostic_results['timestamp_issues'].append(timestamp_issue)
                    print(f"时间戳异常: {data_source} 延迟 {time_diff:.3f}s")
    
    def analyze_next_goal_issues(self):
        """分析next_goal.cpp中的潜在问题"""
        print("\n=== next_goal.cpp 问题分析 ===")
        
        issues = []
        
        # 1. 目标点frame_id使用odom而非map
        issues.append({
            'type': 'frame_id_mismatch',
            'description': 'next_goal.cpp使用odom坐标系发布目标点，应该使用map坐标系',
            'file': 'next_goal.cpp',
            'line': '134: goal_msgs.header.frame_id = "odom"',
            'recommendation': '改为 goal_msgs.header.frame_id = "map"'
        })
        
        # 2. 数组越界风险
        issues.append({
            'type': 'array_bounds',
            'description': '缺少数组边界检查，可能导致越界访问',
            'file': 'next_goal.cpp', 
            'line': '139-142: 访问 pathPlanner.Path[count+1] 时未检查边界',
            'recommendation': '添加 if (count + 1 < pathPlanner.Path.size()) 检查'
        })
        
        # 3. 目标点发布逻辑
        issues.append({
            'type': 'goal_logic',
            'description': '目标点到达判断基于odom坐标，但发布到move_base',
            'file': 'next_goal.cpp',
            'line': '130-133: 距离计算与目标发布坐标系不一致',
            'recommendation': '统一使用map坐标系或添加坐标变换'
        })
        
        for issue in issues:
            print(f"问题类型: {issue['type']}")
            print(f"描述: {issue['description']}")
            print(f"位置: {issue['line']}")
            print(f"建议: {issue['recommendation']}")
            print("-" * 50)
        
        return issues
    
    def generate_report(self):
        """生成诊断报告"""
        report_file = "/home/getting/Sweeping-Robot/robot_jump_diagnostic_report.txt"
        
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write("机器人瞬移问题诊断报告\n")
            f.write("=" * 50 + "\n")
            f.write(f"生成时间: {rospy.Time.now()}\n\n")
            
            # 1. 问题汇总
            f.write("1. 问题汇总\n")
            f.write("-" * 20 + "\n")
            f.write(f"位置跳跃检测: {len(self.diagnostic_results['position_jumps'])} 次\n")
            f.write(f"TF链问题: {len(self.diagnostic_results['tf_chain_issues'])} 个\n")
            f.write(f"坐标系不匹配: {len(self.diagnostic_results['frame_id_issues'])} 个\n")
            f.write(f"时间戳异常: {len(self.diagnostic_results['timestamp_issues'])} 次\n")
            f.write(f"速度异常: {len(self.diagnostic_results['velocity_anomalies'])} 次\n\n")
            
            # 2. 详细分析
            for category, issues in self.diagnostic_results.items():
                if issues:
                    f.write(f"2.{category} 详细信息\n")
                    f.write("-" * 30 + "\n")
                    for issue in issues[-5:]:  # 只显示最近5个
                        f.write(f"{issue}\n")
                    f.write("\n")
            
            # 3. next_goal.cpp问题分析
            f.write("3. next_goal.cpp 源码问题分析\n")
            f.write("-" * 30 + "\n")
            issues = self.analyze_next_goal_issues()
            for issue in issues:
                f.write(f"{issue}\n")
            f.write("\n")
            
            # 4. 修复建议
            f.write("4. 修复建议\n")
            f.write("-" * 20 + "\n")
            f.write("高优先级修复:\n")
            f.write("1. 修改next_goal.cpp中目标点frame_id从'odom'改为'map'\n")
            f.write("2. 添加数组边界检查防止越界访问\n")
            f.write("3. 检查AMCL参数中的transform_tolerance设置\n")
            f.write("4. 确保所有节点使用一致的坐标系\n\n")
            
            f.write("中优先级修复:\n")
            f.write("1. 优化AMCL粒子滤波器参数\n")
            f.write("2. 检查目标点容差设置\n")
            f.write("3. 添加更多边界和异常处理\n\n")
            
        print(f"\n诊断报告已保存到: {report_file}")
        return report_file
    
    def run_diagnosis(self, duration=30):
        """运行诊断"""
        print(f"\n开始 {duration} 秒诊断...")
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if (current_time - start_time).to_sec() > duration:
                break
                
            # 执行各项检查
            self.check_tf_continuity()
            self.monitor_timestamps()
            
            rate.sleep()
        
        print("诊断完成，正在生成报告...")
        report_file = self.generate_report()
        
        # 分析结果
        print("\n=== 诊断结果汇总 ===")
        if self.jump_detected:
            print("❌ 检测到位置跳跃问题")
        else:
            print("✅ 未检测到明显位置跳跃")
            
        if self.diagnostic_results['tf_chain_issues']:
            print("❌ TF变换链存在问题")
        else:
            print("✅ TF变换链正常")
            
        if self.diagnostic_results['frame_id_issues']:
            print("❌ 坐标系frame_id不一致")
            for issue in self.diagnostic_results['frame_id_issues']:
                print(f"   {issue['topic']}: 使用'{issue['actual']}', 建议'{issue['expected']}'")
        else:
            print("✅ 坐标系frame_id一致")
        
        return report_file

def main():
    try:
        diagnostic = RobotJumpDiagnostic()
        
        # 运行诊断
        report_file = diagnostic.run_diagnosis(duration=30)
        
        print(f"\n详细报告请查看: {report_file}")
        print("\n建议优先修复:")
        print("1. 将next_goal.cpp中的目标点frame_id改为'map'")
        print("2. 添加数组边界检查")
        print("3. 统一所有节点的坐标系使用")
        
    except rospy.ROSInterruptException:
        print("诊断被中断")
    except Exception as e:
        print(f"诊断过程中出现错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
