#!/usr/bin/env python3
"""
RViz显示问题诊断脚本
专门检测RViz中的瞬移显示问题
"""

import subprocess
import time
import re
from collections import deque

class RVizDiagnostic:
    def __init__(self):
        self.path_data = deque(maxlen=20)
        
    def check_rviz_config(self):
        """检查RViz配置中的Fixed Frame设置"""
        print("🔍 检查RViz配置...")
        
        rviz_file = "/home/getting/Sweeping-Robot/src/auto_nav/rviz/clean_work.rviz"
        try:
            with open(rviz_file, 'r') as f:
                content = f.read()
                
            # 查找Fixed Frame设置
            if "Fixed Frame:" in content:
                print("  ✅ 找到Fixed Frame配置")
                # 提取Fixed Frame值
                frame_match = re.search(r'Fixed Frame:\s*(\w+)', content)
                if frame_match:
                    frame = frame_match.group(1)
                    print(f"  📋 当前Fixed Frame: {frame}")
                    if frame != "map":
                        print(f"  ⚠️  建议将Fixed Frame设置为'map'")
                else:
                    print("  ❌ 无法解析Fixed Frame值")
            else:
                print("  ❌ 未找到Fixed Frame设置")
                
        except Exception as e:
            print(f"  ❌ 读取RViz配置失败: {e}")
    
    def check_tf_timestamp_issues(self):
        """检查TF时间戳问题"""
        print("\n🕒 检查TF时间戳问题...")
        
        try:
            # 检查map->odom变换的时间戳
            result = subprocess.run(['rosrun', 'tf', 'tf_echo', 'map', 'odom'], 
                                  capture_output=True, text=True, timeout=3)
            
            if result.returncode == 0:
                # 检查是否有时间戳相关警告
                if "extrapolation" in result.stderr.lower():
                    print("  ⚠️  检测到TF外推警告，可能导致RViz显示跳跃")
                elif "future" in result.stderr.lower():
                    print("  ⚠️  检测到TF未来时间警告")
                else:
                    print("  ✅ TF时间戳正常")
            else:
                print(f"  ❌ TF查询失败: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            print("  ❌ TF查询超时")
        except Exception as e:
            print(f"  ❌ TF检查错误: {e}")
    
    def monitor_path_messages(self):
        """监控路径消息的发布频率和内容"""
        print("\n📊 监控路径消息 (10秒)...")
        
        start_time = time.time()
        message_count = 0
        
        try:
            process = subprocess.Popen(['rostopic', 'hz', '/passedPath'], 
                                     stdout=subprocess.PIPE, 
                                     stderr=subprocess.PIPE, 
                                     text=True)
            
            time.sleep(10)  # 监控10秒
            process.terminate()
            
            output, _ = process.communicate()
            
            # 解析频率信息
            if "average rate:" in output:
                rate_match = re.search(r'average rate:\s*(\d+\.?\d*)', output)
                if rate_match:
                    rate = float(rate_match.group(1))
                    print(f"  📈 /passedPath 平均发布频率: {rate:.2f} Hz")
                    if rate > 5:
                        print("  ⚠️  发布频率过高可能导致RViz显示问题")
                    else:
                        print("  ✅ 发布频率合理")
            else:
                print("  ❌ 无法获取频率信息")
                
        except Exception as e:
            print(f"  ❌ 监控路径消息错误: {e}")
    
    def check_path_message_content(self):
        """检查路径消息内容"""
        print("\n📋 检查路径消息内容...")
        
        try:
            result = subprocess.run(['rostopic', 'echo', '/passedPath', '-n', '1'], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                output = result.stdout
                
                # 检查header信息
                if "frame_id:" in output:
                    frame_match = re.search(r'frame_id:\s*[\'"]?(\w+)[\'"]?', output)
                    if frame_match:
                        frame_id = frame_match.group(1)
                        print(f"  📋 路径消息frame_id: {frame_id}")
                        if frame_id != "map":
                            print(f"  ⚠️  frame_id应该是'map'而不是'{frame_id}'")
                        else:
                            print("  ✅ frame_id正确")
                
                # 检查时间戳
                if "stamp:" in output:
                    # 查找时间戳模式
                    if "secs: 0" in output and "nsecs: 0" in output:
                        print("  📋 使用固定时间戳 (0,0) - 有助于避免RViz跳跃")
                    else:
                        print("  📋 使用动态时间戳 - 可能导致RViz显示问题")
                
                # 检查位置数据是否合理
                positions = re.findall(r'x:\s*(-?\d+\.?\d*)\s*y:\s*(-?\d+\.?\d*)', output)
                if positions:
                    print(f"  📍 检测到 {len(positions)} 个路径点")
                    
                    # 检查是否有异常大的坐标值
                    for i, (x, y) in enumerate(positions[:5]):  # 只检查前5个点
                        x, y = float(x), float(y)
                        if abs(x) > 100 or abs(y) > 100:
                            print(f"    ⚠️  点{i}: ({x:.2f}, {y:.2f}) 坐标值异常大")
                        elif abs(x) > 0.001 or abs(y) > 0.001:  # 有效坐标
                            print(f"    ✅ 点{i}: ({x:.2f}, {y:.2f}) 坐标正常")
                
            else:
                print("  ❌ 无法获取路径消息")
                
        except Exception as e:
            print(f"  ❌ 检查路径消息错误: {e}")
    
    def suggest_rviz_fixes(self):
        """提供RViz修复建议"""
        print("\n💡 RViz显示问题修复建议:")
        print()
        print("1. 🎯 固定坐标系设置:")
        print("   - 在RViz中将Fixed Frame设置为'map'")
        print("   - 避免使用'odom'作为Fixed Frame")
        print()
        print("2. ⏱️ 时间戳优化:")
        print("   - 路径消息使用固定时间戳 ros::Time(0)")
        print("   - TF变换使用稍旧的时间戳避免未来时间问题")
        print()
        print("3. 📈 降低更新频率:")
        print("   - 减少路径发布频率 (已修改为每10次更新发布一次)")
        print("   - 在RViz中调整Path显示的Queue Size")
        print()
        print("4. 🔧 RViz显示设置:")
        print("   - Path显示器设置:")
        print("     • Color: 自定义颜色以便观察")
        print("     • Alpha: 0.8-1.0")
        print("     • Line Width: 0.05-0.1")
        print("     • Queue Size: 1 (只显示最新的)")
        print()
        print("5. 🚀 重启RViz测试:")
        print("   - 关闭RViz")
        print("   - 重新启动系统和RViz")
        print("   - 观察机器人在Gazebo中的实际运动是否正常")
    
    def run_diagnosis(self):
        """运行完整的RViz诊断"""
        print("=== RViz显示问题诊断 ===")
        print("针对'瞬移只在RViz中出现，Gazebo中正常'的问题")
        print()
        
        self.check_rviz_config()
        self.check_tf_timestamp_issues()
        self.monitor_path_messages()
        self.check_path_message_content()
        self.suggest_rviz_fixes()
        
        print("\n=== 诊断总结 ===")
        print("如果机器人在Gazebo中运动正常，但在RViz中显示跳跃，")
        print("主要原因是可视化层面的时间戳和坐标系问题。")
        print("按照上述建议修复后应该能解决RViz显示跳跃问题。")

def main():
    diagnostic = RVizDiagnostic()
    diagnostic.run_diagnosis()

if __name__ == '__main__':
    main()
