#!/usr/bin/env python3
"""
实时监控机器人跳跃和多节点目标冲突
"""

import subprocess
import time
import threading
from collections import deque
import re

class RobotMonitor:
    def __init__(self):
        self.odom_history = deque(maxlen=50)
        self.goal_publishers = []
        self.tf_issues = []
        
    def check_goal_publishers(self):
        """检查有多少节点在发布目标点"""
        try:
            result = subprocess.run(['rostopic', 'info', '/move_base_simple/goal'], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                # 解析发布者信息
                publishers_section = False
                publishers = []
                
                for line in result.stdout.split('\n'):
                    if 'Publishers:' in line:
                        publishers_section = True
                        continue
                    elif 'Subscribers:' in line:
                        publishers_section = False
                        continue
                    elif publishers_section and line.strip():
                        # 提取节点名称
                        if '*' in line:
                            node_info = line.strip()
                            publishers.append(node_info)
                
                self.goal_publishers = publishers
                
                if len(publishers) > 1:
                    print(f"⚠️  警告: 发现多个节点在发布目标点:")
                    for pub in publishers:
                        print(f"    {pub}")
                    print("   这可能导致目标点冲突和机器人跳跃!")
                elif len(publishers) == 1:
                    print(f"✅ 目标点发布者: {publishers[0]}")
                else:
                    print("❌ 没有发现目标点发布者")
                    
        except Exception as e:
            print(f"检查目标点发布者时出错: {e}")
    
    def monitor_odom_jumps(self):
        """监控里程计跳跃"""
        try:
            process = subprocess.Popen(['rostopic', 'echo', '/odom', '-n', '1'], 
                                     stdout=subprocess.PIPE, 
                                     stderr=subprocess.PIPE, 
                                     text=True)
            
            output, _ = process.communicate(timeout=2)
            
            # 简单解析位置信息
            x_match = re.search(r'x:\s*(-?\d+\.?\d*)', output)
            y_match = re.search(r'y:\s*(-?\d+\.?\d*)', output)
            
            if x_match and y_match:
                x = float(x_match.group(1))
                y = float(y_match.group(1))
                
                current_pos = (x, y, time.time())
                
                if self.odom_history:
                    prev_pos = self.odom_history[-1]
                    distance = ((x - prev_pos[0])**2 + (y - prev_pos[1])**2)**0.5
                    time_diff = current_pos[2] - prev_pos[2]
                    
                    if time_diff > 0:
                        velocity = distance / time_diff
                        
                        if distance > 0.5:  # 跳跃阈值
                            print(f"🚨 检测到位置跳跃: 距离={distance:.3f}m, 速度={velocity:.3f}m/s")
                            print(f"   从 ({prev_pos[0]:.3f}, {prev_pos[1]:.3f}) 到 ({x:.3f}, {y:.3f})")
                        elif velocity > 2.0:  # 速度异常
                            print(f"⚠️  高速移动: 速度={velocity:.3f}m/s")
                
                self.odom_history.append(current_pos)
                
        except Exception as e:
            pass  # 静默处理错误
    
    def check_tf_status(self):
        """检查TF状态"""
        try:
            result = subprocess.run(['rosrun', 'tf', 'tf_echo', 'map', 'odom'], 
                                  capture_output=True, text=True, timeout=3)
            
            if result.returncode != 0:
                print(f"❌ TF变换 map->odom 失败: {result.stderr.strip()}")
            else:
                # 检查变换是否包含异常值
                if 'nan' in result.stdout.lower() or 'inf' in result.stdout.lower():
                    print("❌ TF变换包含无效值 (NaN/Inf)")
                    
        except subprocess.TimeoutExpired:
            print("❌ TF变换查询超时")
        except Exception as e:
            print(f"❌ TF检查错误: {e}")
    
    def check_active_nodes(self):
        """检查相关节点状态"""
        critical_nodes = ['sequential_goal', 'next_goal', 'amcl', 'move_base']
        
        try:
            result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True, timeout=5)
            active_nodes = result.stdout.split('\n')
            
            print("\n📋 关键节点状态:")
            for node in critical_nodes:
                matching_nodes = [n for n in active_nodes if node in n]
                if matching_nodes:
                    print(f"  ✅ {node}: {matching_nodes}")
                else:
                    print(f"  ❌ {node}: 未运行")
                    
        except Exception as e:
            print(f"检查节点状态错误: {e}")
    
    def run_diagnosis(self):
        """运行完整诊断"""
        print("=== 机器人跳跃问题实时诊断 ===")
        print(f"开始时间: {time.strftime('%H:%M:%S')}")
        print()
        
        # 检查节点状态
        self.check_active_nodes()
        
        print("\n🎯 检查目标点发布冲突:")
        self.check_goal_publishers()
        
        print("\n🔄 监控位置跳跃 (10秒)...")
        for i in range(10):
            self.monitor_odom_jumps()
            time.sleep(1)
            if i % 3 == 0:
                print(f"  监控中... {i+1}/10")
        
        print("\n🔧 检查TF变换状态:")
        self.check_tf_status()
        
        print("\n=== 诊断建议 ===")
        if len(self.goal_publishers) > 1:
            print("🚨 发现多个目标点发布者!")
            print("建议:")
            print("  1. 检查launch文件，确保只启动一个目标管理节点")
            print("  2. 停止不需要的节点: rosnode kill /node_name")
            print("  3. 修改launch文件移除冲突的节点")
        
        if not self.goal_publishers:
            print("❌ 没有目标点发布者!")
            print("建议: 检查sequential_goal或next_goal节点是否正常启动")
        
        print("\n💡 进一步排查:")
        print("  1. 检查话题: rostopic echo /move_base_simple/goal")
        print("  2. 监控TF: rosrun tf view_frames")
        print("  3. 查看节点图: rqt_graph")

def main():
    monitor = RobotMonitor()
    monitor.run_diagnosis()

if __name__ == '__main__':
    main()
