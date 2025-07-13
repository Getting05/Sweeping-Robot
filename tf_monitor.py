#!/usr/bin/env python3
"""
TF变换链实时监控脚本
专门监控机器人坐标变换中的异常跳跃
"""

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time
from collections import deque

class TFMonitor:
    def __init__(self):
        rospy.init_node('tf_monitor', anonymous=True)
        
        self.tf_listener = tf.TransformListener()
        
        # 记录历史变换数据
        self.map_odom_history = deque(maxlen=100)
        self.odom_base_history = deque(maxlen=100)
        
        # 跳跃检测阈值
        self.position_threshold = 0.1  # m
        self.rotation_threshold = 0.2  # rad
        
        # 订阅器
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseStamped, self.amcl_callback)
        
        print("TF变换监控器已启动...")
        print("监控变换链: /map -> /odom -> /base_footprint")
        
    def odom_callback(self, msg):
        """里程计回调"""
        self.check_tf_transforms()
    
    def amcl_callback(self, msg):
        """AMCL回调"""
        self.check_tf_transforms()
    
    def check_tf_transforms(self):
        """检查TF变换"""
        try:
            current_time = rospy.Time.now()
            
            # 检查map -> odom变换
            if self.tf_listener.canTransform('/map', '/odom', rospy.Time(0)):
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/odom', rospy.Time(0))
                
                transform_data = {
                    'timestamp': current_time,
                    'translation': trans,
                    'rotation': rot
                }
                
                # 检测跳跃
                if self.map_odom_history:
                    self.detect_transform_jump('map->odom', self.map_odom_history[-1], transform_data)
                
                self.map_odom_history.append(transform_data)
            
            # 检查odom -> base_footprint变换
            if self.tf_listener.canTransform('/odom', '/base_footprint', rospy.Time(0)):
                (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
                
                transform_data = {
                    'timestamp': current_time,
                    'translation': trans,
                    'rotation': rot
                }
                
                if self.odom_base_history:
                    self.detect_transform_jump('odom->base', self.odom_base_history[-1], transform_data)
                
                self.odom_base_history.append(transform_data)
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF查询失败: {e}")
    
    def detect_transform_jump(self, transform_name, prev_data, curr_data):
        """检测变换跳跃"""
        prev_trans = np.array(prev_data['translation'])
        curr_trans = np.array(curr_data['translation'])
        
        # 计算位置变化
        pos_diff = np.linalg.norm(curr_trans - prev_trans)
        time_diff = (curr_data['timestamp'] - prev_data['timestamp']).to_sec()
        
        if time_diff > 0 and pos_diff > self.position_threshold:
            velocity = pos_diff / time_diff
            
            rospy.logwarn(f"检测到{transform_name}位置跳跃:")
            rospy.logwarn(f"  位置变化: {pos_diff:.4f}m")
            rospy.logwarn(f"  时间间隔: {time_diff:.4f}s")
            rospy.logwarn(f"  等效速度: {velocity:.4f}m/s")
            rospy.logwarn(f"  前一位置: ({prev_trans[0]:.4f}, {prev_trans[1]:.4f}, {prev_trans[2]:.4f})")
            rospy.logwarn(f"  当前位置: ({curr_trans[0]:.4f}, {curr_trans[1]:.4f}, {curr_trans[2]:.4f})")
    
    def run(self):
        """运行监控"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        monitor = TFMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
