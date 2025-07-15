#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
碰撞检测冷却功能测试脚本
测试15秒冷却功能是否正常工作
"""

import time
import math


class MockCollisionDetector:
    """模拟碰撞检测器，用于测试冷却功能"""
    
    def __init__(self):
        self.collision_count = 0
        self.last_position_for_collision = None
        self.stuck_threshold = 0.05  # 卡住检测阈值
        self.stuck_time_threshold = 2.0  # 缩短到2秒用于测试
        self.stuck_start_time = None
        self.collision_cooldown = 5.0  # 缩短到5秒用于测试
        self.last_collision_time = 0.0  # 上次碰撞检测时间
        
    def detect_collision(self, position, current_time):
        """检测碰撞（通过卡住检测），带冷却功能"""
        # 检查是否在冷却期内
        if current_time - self.last_collision_time < self.collision_cooldown:
            # 在冷却期内，不进行碰撞检测，但继续更新位置信息
            print(f"在冷却期内，剩余: {self.collision_cooldown - (current_time - self.last_collision_time):.1f}秒")
            self.last_position_for_collision = position
            return
        
        if self.last_position_for_collision is not None:
            dist = math.sqrt((position[0] - self.last_position_for_collision[0])**2 +
                           (position[1] - self.last_position_for_collision[1])**2)
            
            if dist < self.stuck_threshold:
                if self.stuck_start_time is None:
                    self.stuck_start_time = current_time
                    print(f"开始检测卡住状态，位置: {position}, 距离: {dist:.3f}")
                elif current_time - self.stuck_start_time > self.stuck_time_threshold:
                    self.collision_count += 1
                    self.last_collision_time = current_time  # 记录碰撞时间，开始冷却
                    print(f"*** 检测到碰撞/卡住事件，总计: {self.collision_count} (开始{self.collision_cooldown}秒冷却) ***")
                    self.stuck_start_time = None
            else:
                if self.stuck_start_time is not None:
                    print(f"机器人移动恢复，距离: {dist:.3f}")
                self.stuck_start_time = None
        
        self.last_position_for_collision = position


def test_collision_cooldown():
    """测试碰撞检测冷却功能"""
    print("=== 碰撞检测冷却功能测试 ===")
    
    detector = MockCollisionDetector()
    start_time = time.time()
    
    # 模拟机器人运动轨迹
    positions = [
        (0.0, 0.0),   # 起始位置
        (0.01, 0.0),  # 微小移动（低于阈值，应触发卡住检测）
        (0.01, 0.0),  # 保持不动
        (0.01, 0.0),  # 保持不动 - 应在2秒后检测到碰撞
        (0.01, 0.0),  # 冷却期内
        (0.01, 0.0),  # 冷却期内
        (0.01, 0.0),  # 冷却期内
        (0.01, 0.0),  # 冷却期内
        (0.01, 0.0),  # 冷却期内
        (0.01, 0.0),  # 冷却期结束
        (0.01, 0.0),  # 再次检测到卡住
        (0.01, 0.0),  # 保持不动
        (0.01, 0.0),  # 应再次触发碰撞检测
        (1.0, 0.0),   # 大幅移动，恢复正常
    ]
    
    for i, pos in enumerate(positions):
        current_time = start_time + i * 1.0  # 每秒一个位置更新
        print(f"\n时间: {i}s, 位置: {pos}")
        detector.detect_collision(pos, current_time)
        
        if i < len(positions) - 1:
            time.sleep(1)  # 实际等待1秒
    
    print(f"\n=== 测试完成 ===")
    print(f"总碰撞次数: {detector.collision_count}")
    print(f"预期应有2次碰撞检测（中间有冷却期隔开）")


if __name__ == "__main__":
    test_collision_cooldown()
