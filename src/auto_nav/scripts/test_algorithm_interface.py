#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简单的算法接口测试脚本
用于验证路径规划算法接口是否正常工作
"""

import rospy
import time
from auto_nav.srv import SetPathPlanningAlgorithm, SetAlgorithmParameter

def test_algorithm_interface():
    """测试算法接口功能"""
    rospy.init_node('algorithm_interface_test')
    
    print("=== 路径规划算法接口测试 ===")
    
    # 等待服务可用
    print("等待服务启动...")
    try:
        rospy.wait_for_service('/path_planning/set_algorithm', timeout=30.0)
        rospy.wait_for_service('/path_planning/set_parameter', timeout=10.0)
        print("✓ 服务已就绪")
    except rospy.ROSException:
        print("✗ 服务不可用，测试失败")
        return False
    
    # 创建服务客户端
    set_algorithm = rospy.ServiceProxy('/path_planning/set_algorithm', SetPathPlanningAlgorithm)
    set_parameter = rospy.ServiceProxy('/path_planning/set_parameter', SetAlgorithmParameter)
    
    success_count = 0
    total_tests = 0
    
    # 测试1: 获取可用算法列表
    print("\n1. 测试获取可用算法列表...")
    total_tests += 1
    try:
        response = set_algorithm("astar")  # 通过调用获取列表
        print(f"可用算法: {response.available_algorithms}")
        if len(response.available_algorithms) >= 2:
            print("✓ 成功获取算法列表")
            success_count += 1
        else:
            print("✗ 算法列表不完整")
    except Exception as e:
        print(f"✗ 获取算法列表失败: {e}")
    
    # 测试2: 切换到A*算法
    print("\n2. 测试切换到A*算法...")
    total_tests += 1
    try:
        response = set_algorithm("astar")
        if response.success:
            print("✓ 成功切换到A*算法")
            success_count += 1
        else:
            print(f"✗ 切换失败: {response.message}")
    except Exception as e:
        print(f"✗ 切换A*算法失败: {e}")
    
    # 测试3: 设置A*算法参数
    print("\n3. 测试设置A*算法参数...")
    total_tests += 1
    try:
        response = set_parameter("heuristic_weight", 1.5)
        if response.success:
            print("✓ 成功设置启发式权重")
            success_count += 1
        else:
            print(f"✗ 参数设置失败: {response.message}")
    except Exception as e:
        print(f"✗ 设置参数失败: {e}")
    
    # 测试4: 切换到神经网络算法
    print("\n4. 测试切换到神经网络算法...")
    total_tests += 1
    try:
        response = set_algorithm("neural")
        if response.success:
            print("✓ 成功切换到神经网络算法")
            success_count += 1
        else:
            print(f"✗ 切换失败: {response.message}")
    except Exception as e:
        print(f"✗ 切换神经网络算法失败: {e}")
    
    # 测试5: 设置神经网络算法参数
    print("\n5. 测试设置神经网络算法参数...")
    total_tests += 1
    try:
        response = set_parameter("c_0", 60.0)
        if response.success:
            print("✓ 成功设置神经网络参数")
            success_count += 1
        else:
            print(f"✗ 参数设置失败: {response.message}")
    except Exception as e:
        print(f"✗ 设置神经网络参数失败: {e}")
    
    # 测试6: 测试无效算法
    print("\n6. 测试无效算法处理...")
    total_tests += 1
    try:
        response = set_algorithm("invalid_algorithm")
        if not response.success:
            print("✓ 正确拒绝无效算法")
            success_count += 1
        else:
            print("✗ 应该拒绝无效算法")
    except Exception as e:
        print(f"✗ 无效算法测试失败: {e}")
    
    # 输出测试结果
    print(f"\n=== 测试结果 ===")
    print(f"通过: {success_count}/{total_tests}")
    print(f"成功率: {success_count/total_tests*100:.1f}%")
    
    if success_count == total_tests:
        print("🎉 所有测试通过！算法接口工作正常")
        return True
    else:
        print("⚠️  部分测试失败，请检查系统状态")
        return False

if __name__ == '__main__':
    try:
        test_algorithm_interface()
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"测试出现异常: {e}")
