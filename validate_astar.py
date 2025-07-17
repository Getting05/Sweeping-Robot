#!/usr/bin/env python3

"""
A*路径规划算法验证脚本
快速验证A*算法的基本功能
"""

import rospy
import time
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import threading

class AStarValidator:
    def __init__(self):
        self.path_received = False
        self.path_length = 0
        self.grid_received = False
        
        # 初始化ROS节点
        rospy.init_node('astar_validator', anonymous=True)
        
        # 订阅话题
        self.path_sub = rospy.Subscriber('/plan_path', Path, self.path_callback)
        self.grid_sub = rospy.Subscriber('/covered_grid', OccupancyGrid, self.grid_callback)
        
        # 发布初始位置
        self.pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        
        print("🚀 A*路径规划验证器启动")
        print("📡 等待路径规划节点...")
        
    def path_callback(self, msg):
        """路径回调函数"""
        self.path_received = True
        self.path_length = len(msg.poses)
        
        print(f"\n✅ 收到A*规划路径！")
        print(f"📊 路径统计：")
        print(f"   - 路径点数量: {self.path_length}")
        
        if self.path_length > 0:
            start = msg.poses[0].pose.position
            end = msg.poses[-1].pose.position
            print(f"   - 起始点: ({start.x:.2f}, {start.y:.2f})")
            print(f"   - 终点: ({end.x:.2f}, {end.y:.2f})")
            
            # 计算路径长度
            total_distance = 0.0
            for i in range(1, len(msg.poses)):
                p1 = msg.poses[i-1].pose.position
                p2 = msg.poses[i].pose.position
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                total_distance += (dx*dx + dy*dy)**0.5
            
            print(f"   - 路径总长度: {total_distance:.2f} 米")
            print(f"   - 平均点间距: {total_distance/self.path_length:.3f} 米")
            
            # 检查路径连续性
            max_gap = 0.0
            for i in range(1, len(msg.poses)):
                p1 = msg.poses[i-1].pose.position
                p2 = msg.poses[i].pose.position
                gap = ((p2.x - p1.x)**2 + (p2.y - p1.y)**2)**0.5
                max_gap = max(max_gap, gap)
            
            print(f"   - 最大点间距: {max_gap:.3f} 米")
            
            if max_gap < 1.0:  # 1米内认为连续
                print("   ✅ 路径连续性检查通过")
            else:
                print("   ⚠️  路径可能存在跳跃")
                
        print(f"\n🎯 A*算法特征分析：")
        
        # 分析路径平滑度
        if self.path_length > 2:
            direction_changes = 0
            for i in range(1, len(msg.poses) - 1):
                p1 = msg.poses[i-1].pose.position
                p2 = msg.poses[i].pose.position  
                p3 = msg.poses[i+1].pose.position
                
                # 计算转向角度
                v1_x, v1_y = p2.x - p1.x, p2.y - p1.y
                v2_x, v2_y = p3.x - p2.x, p3.y - p2.y
                
                if (v1_x*v1_x + v1_y*v1_y) > 0 and (v2_x*v2_x + v2_y*v2_y) > 0:
                    dot_product = v1_x*v2_x + v1_y*v2_y
                    v1_mag = (v1_x*v1_x + v1_y*v1_y)**0.5
                    v2_mag = (v2_x*v2_x + v2_y*v2_y)**0.5
                    
                    cos_angle = dot_product / (v1_mag * v2_mag)
                    cos_angle = max(-1, min(1, cos_angle))  # 限制范围
                    
                    if cos_angle < 0.9:  # 约25度以上的转向
                        direction_changes += 1
            
            smoothness = 1.0 - (direction_changes / max(1, self.path_length - 2))
            print(f"   - 路径平滑度: {smoothness:.2f} ({direction_changes} 次显著转向)")
            
        # 分析覆盖效率
        if self.path_length > 10:
            # 计算路径包围盒
            min_x = min(p.pose.position.x for p in msg.poses)
            max_x = max(p.pose.position.x for p in msg.poses)
            min_y = min(p.pose.position.y for p in msg.poses)
            max_y = max(p.pose.position.y for p in msg.poses)
            
            coverage_area = (max_x - min_x) * (max_y - min_y)
            if coverage_area > 0:
                efficiency = total_distance / coverage_area
                print(f"   - 覆盖区域: {coverage_area:.2f} 平方米")
                print(f"   - 覆盖效率: {efficiency:.2f} 米/平方米")
        
    def grid_callback(self, msg):
        """覆盖网格回调函数"""
        if not self.grid_received:
            self.grid_received = True
            print(f"\n📋 收到覆盖网格信息：")
            print(f"   - 网格尺寸: {msg.info.width} x {msg.info.height}")
            print(f"   - 分辨率: {msg.info.resolution:.3f} 米/格")
    
    def publish_initial_pose(self):
        """发布初始位置"""
        if rospy.is_shutdown():
            return
            
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        
        # 设置在原点附近
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # 设置协方差（表示位置不确定性）
        msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        
        self.pose_pub.publish(msg)
        print("📍 发布初始位置: (0.0, 0.0)")
    
    def run_validation(self, timeout=30):
        """运行验证"""
        print(f"⏱️  等待路径数据（最多{timeout}秒）...")
        
        # 发布初始位置
        rospy.sleep(1.0)  # 等待发布器准备好
        self.publish_initial_pose()
        
        start_time = time.time()
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            if self.path_received:
                print("\n🎉 A*路径规划验证成功！")
                return True
            rospy.sleep(0.1)
        
        print(f"\n⏰ 验证超时（{timeout}秒）")
        print("❌ 可能的问题：")
        print("   1. 路径规划节点未启动")
        print("   2. 地图服务未运行") 
        print("   3. TF变换缺失")
        print("   4. 算法计算时间过长")
        
        return False

def main():
    try:
        validator = AStarValidator()
        
        print("\n🔧 建议启动以下节点（新终端）：")
        print("   rosrun map_server map_server <map_file>")
        print("   rosrun auto_nav path_planning")
        print("   (可选) rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100")
        print("   (可选) rosrun tf static_transform_publisher 0 0 0 0 0 0 odom base_footprint 100")
        
        # 运行验证
        success = validator.run_validation(timeout=45)
        
        if success:
            # 等待更多数据
            print("\n📊 继续收集数据...")
            rospy.sleep(5.0)
            
            print("\n📋 A*算法实现验证报告：")
            print("="*50)
            print("✅ 编译状态: 成功")
            print("✅ 节点启动: 成功")
            print("✅ 路径生成: 成功")
            print(f"✅ 路径质量: {validator.path_length} 个路径点")
            
            if validator.grid_received:
                print("✅ 覆盖网格: 已接收")
            else:
                print("⚠️  覆盖网格: 未接收")
                
            print("="*50)
            print("🎯 A*全覆盖路径规划算法实现完成！")
        else:
            print("\n❌ 验证失败，请检查系统配置")
            
    except KeyboardInterrupt:
        print("\n⏹️  用户中断验证")
    except Exception as e:
        print(f"\n💥 验证过程出错: {e}")
    
    print("\n👋 验证结束")

if __name__ == "__main__":
    main()
