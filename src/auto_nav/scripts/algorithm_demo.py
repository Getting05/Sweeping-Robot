#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
路径规划算法切换演示脚本
展示如何动态切换不同的路径规划算法
"""

import rospy
from auto_nav.srv import SetPathPlanningAlgorithm, SetAlgorithmParameter
import sys

def switch_algorithm_demo():
    """演示算法切换功能"""
    rospy.init_node('algorithm_switch_demo')
    
    # 等待服务可用
    rospy.loginfo("等待路径规划服务...")
    try:
        rospy.wait_for_service('/path_planning/set_algorithm', timeout=10.0)
        rospy.wait_for_service('/path_planning/set_parameter', timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("路径规划服务不可用，请确保path_planning节点正在运行")
        return
    
    # 创建服务客户端
    set_algorithm = rospy.ServiceProxy('/path_planning/set_algorithm', SetPathPlanningAlgorithm)
    set_parameter = rospy.ServiceProxy('/path_planning/set_parameter', SetAlgorithmParameter)
    
    print("\n=== 路径规划算法切换演示 ===")
    
    # 1. 切换到A*算法
    print("\n1. 切换到A*算法...")
    try:
        response = set_algorithm("astar")
        if response.success:
            print(f"✓ 成功切换到A*算法")
            print(f"可用算法: {response.available_algorithms}")
            
            # 设置A*算法参数
            print("  设置A*算法参数...")
            set_parameter("heuristic_weight", 1.2)
            set_parameter("coverage_strategy", "nearest")
            print("  ✓ 参数设置完成")
        else:
            print(f"✗ 切换失败: {response.message}")
    except rospy.ServiceException as e:
        print(f"✗ 服务调用失败: {e}")
    
    # 2. 切换到神经网络算法
    print("\n2. 切换到神经网络算法...")
    try:
        response = set_algorithm("neural")
        if response.success:
            print(f"✓ 成功切换到神经网络算法")
            
            # 设置神经网络算法参数
            print("  设置神经网络算法参数...")
            set_parameter("c_0", 60.0)
            set_parameter("max_iterations", 8000)
            print("  ✓ 参数设置完成")
        else:
            print(f"✗ 切换失败: {response.message}")
    except rospy.ServiceException as e:
        print(f"✗ 服务调用失败: {e}")
    
    # 3. 测试无效算法
    print("\n3. 测试无效算法...")
    try:
        response = set_algorithm("invalid_algorithm")
        if not response.success:
            print(f"✓ 正确处理无效算法: {response.message}")
            print(f"可用算法: {response.available_algorithms}")
        else:
            print("✗ 应该拒绝无效算法")
    except rospy.ServiceException as e:
        print(f"✗ 服务调用失败: {e}")
    
    print("\n=== 演示完成 ===")

def interactive_mode():
    """交互式模式"""
    rospy.init_node('algorithm_interactive_demo')
    
    # 等待服务可用
    try:
        rospy.wait_for_service('/path_planning/set_algorithm', timeout=10.0)
        rospy.wait_for_service('/path_planning/set_parameter', timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("路径规划服务不可用，请确保path_planning节点正在运行")
        return
    
    # 创建服务客户端
    set_algorithm = rospy.ServiceProxy('/path_planning/set_algorithm', SetPathPlanningAlgorithm)
    set_parameter = rospy.ServiceProxy('/path_planning/set_parameter', SetAlgorithmParameter)
    
    print("\n=== 路径规划算法交互式模式 ===")
    print("输入 'help' 查看帮助，输入 'quit' 退出")
    
    while not rospy.is_shutdown():
        try:
            command = input("\n> ").strip().split()
            if not command:
                continue
            
            if command[0] == 'help':
                print("可用命令:")
                print("  switch <algorithm>     - 切换算法 (astar, neural)")
                print("  set <param> <value>    - 设置参数")
                print("  list                   - 列出可用算法")
                print("  quit                   - 退出")
                
            elif command[0] == 'switch':
                if len(command) != 2:
                    print("用法: switch <algorithm>")
                    continue
                
                algorithm = command[1]
                response = set_algorithm(algorithm)
                if response.success:
                    print(f"✓ 成功切换到 {algorithm}")
                else:
                    print(f"✗ 切换失败: {response.message}")
                    
            elif command[0] == 'set':
                if len(command) != 3:
                    print("用法: set <param_name> <param_value>")
                    continue
                
                param_name = command[1]
                try:
                    param_value = float(command[2])
                    response = set_parameter(param_name, param_value)
                    if response.success:
                        print(f"✓ 设置参数 {param_name} = {param_value}")
                    else:
                        print(f"✗ 设置失败: {response.message}")
                except ValueError:
                    print("参数值必须是数字")
                    
            elif command[0] == 'list':
                response = set_algorithm("astar")  # 获取可用算法列表
                print(f"可用算法: {response.available_algorithms}")
                
            elif command[0] == 'quit':
                break
                
            else:
                print(f"未知命令: {command[0]}，输入 'help' 查看帮助")
                
        except KeyboardInterrupt:
            break
        except rospy.ServiceException as e:
            print(f"服务调用失败: {e}")
        except Exception as e:
            print(f"错误: {e}")
    
    print("\n再见！")

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'interactive':
        interactive_mode()
    else:
        switch_algorithm_demo()
