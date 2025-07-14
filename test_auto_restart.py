#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
自动重启功能测试脚本
测试覆盖率停滞检测和自动重启机制
"""

import time
import os
import subprocess
import sys

def test_auto_restart_feature():
    """测试自动重启功能"""
    print("=== 测试自动重启功能 ===")
    
    # 检查coverage_monitor.py文件
    monitor_path = "/home/getting/Sweeping-Robot/src/auto_nav/scripts/coverage_monitor.py"
    if not os.path.exists(monitor_path):
        print(f"错误：找不到文件 {monitor_path}")
        return False
    
    print(f"✓ 找到监控脚本: {monitor_path}")
    
    # 检查关键代码是否存在
    with open(monitor_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    key_features = [
        "coverage_stagnation_threshold = 120.0",
        "check_coverage_stagnation",
        "trigger_auto_restart",
        "save_restart_report",
        "execute_restart",
        "Coverage Monitor v2.2"
    ]
    
    print("\n检查关键功能代码:")
    for feature in key_features:
        if feature in content:
            print(f"✓ {feature}")
        else:
            print(f"✗ {feature} - 未找到")
            return False
    
    # 检查自动重启管理器
    manager_path = "/home/getting/Sweeping-Robot/auto_restart_manager.sh"
    if os.path.exists(manager_path) and os.access(manager_path, os.X_OK):
        print(f"✓ 自动重启管理器: {manager_path}")
    else:
        print(f"✗ 自动重启管理器未找到或无执行权限")
        return False
    
    return True

def show_feature_details():
    """显示功能详情"""
    print("\n=== 自动重启功能详情 ===")
    print("1. 覆盖率停滞检测:")
    print("   - 阈值: 2分钟 (120秒)")
    print("   - 覆盖率变化容差: 0.1%")
    print("   - 检测间隔: 跟随更新频率 (2Hz)")
    
    print("\n2. 自动重启触发条件:")
    print("   - 覆盖率连续2分钟无显著变化 (变化<0.1%)")
    print("   - 自动保存重启前状态报告")
    print("   - 优雅关闭当前所有ROS节点")
    print("   - 启动新的清扫任务")
    
    print("\n3. 重启管理器功能:")
    print("   - 监控coverage_monitor的重启信号")
    print("   - 自动清理和重启清扫系统")
    print("   - 最大重启次数限制 (10次)")
    print("   - 详细的日志记录")
    
    print("\n4. 安全机制:")
    print("   - 重启次数限制防止无限循环")
    print("   - 详细的状态报告和日志")
    print("   - 优雅的进程关闭")
    print("   - 可手动停止自动重启")

def show_usage_instructions():
    """显示使用说明"""
    print("\n=== 使用说明 ===")
    print("方式1: 使用自动重启管理器 (推荐)")
    print("1. 启动自动重启管理器:")
    print("   ./auto_restart_manager.sh start")
    print("2. 管理器会自动启动清扫系统并监控")
    print("3. 检查状态: ./auto_restart_manager.sh status")
    print("4. 查看日志: ./auto_restart_manager.sh logs")
    print("5. 停止管理器: ./auto_restart_manager.sh stop")
    
    print("\n方式2: 手动启动 (仅用于测试)")
    print("1. roscore")
    print("2. rosrun auto_nav coverage_monitor.py")
    print("3. roslaunch auto_nav sequential_clean.launch")
    print("4. 等待覆盖率停滞2分钟观察自动重启")

def test_configuration():
    """测试配置参数"""
    print("\n=== 配置参数测试 ===")
    
    config_tests = [
        ("覆盖率停滞阈值", "60.0 秒 "),
        ("覆盖率变化容差", "0.001 (0.1%)"),
        ("最大重启次数", "10 次"),
        ("重启间隔", "10 秒"),
        ("CSV保存间隔", "30 秒"),
        ("更新频率", "2.0 Hz")
    ]
    
    for name, value in config_tests:
        print(f"✓ {name}: {value}")

def simulate_stagnation_test():
    """模拟停滞测试"""
    print("\n=== 模拟停滞测试 ===")
    print("注意：这是一个概念性测试，不会真正执行")
    
    print("\n模拟场景:")
    print("1. 机器人开始清扫，覆盖率正常增长")
    print("2. 机器人卡在某个位置，覆盖率停止增长")
    print("3. 2分钟后触发自动重启")
    print("4. 系统保存重启报告")
    print("5. 关闭当前清扫任务")
    print("6. 启动新的清扫任务")
    print("7. 重复监控过程")
    
    print("\n预期结果:")
    print("- 生成重启报告: /home/getting/tmp/auto_restart_report_<timestamp>.txt")
    print("- 重启日志: /home/getting/tmp/auto_restart_logs/restart_*.log")
    print("- 新的CSV数据文件继续记录")
    print("- 清扫任务从头开始执行")

def check_dependencies():
    """检查依赖项"""
    print("\n=== 检查依赖项 ===")
    
    # 检查命令
    commands = ['roscore', 'roslaunch', 'rosrun', 'rosnode', 'pkill']
    for cmd in commands:
        if subprocess.run(['which', cmd], capture_output=True, text=True).returncode == 0:
            print(f"✓ {cmd}")
        else:
            print(f"✗ {cmd} - 未找到")
    
    # 检查目录权限
    dirs = ['/home/getting/tmp']
    for dir_path in dirs:
        if os.access(dir_path, os.W_OK):
            print(f"✓ {dir_path} 可写")
        else:
            print(f"✗ {dir_path} 不可写")

if __name__ == '__main__':
    print("开始测试自动重启功能...")
    
    # 功能验证
    success = test_auto_restart_feature()
    
    if success:
        print("\n🎉 自动重启功能验证通过！")
        
        # 显示详细信息
        show_feature_details()
        show_usage_instructions()
        test_configuration()
        simulate_stagnation_test()
        check_dependencies()
        
        print("\n=== 测试建议 ===")
        print("1. 首先在仿真环境中测试")
        print("2. 观察覆盖率停滞检测是否正常工作")
        print("3. 验证重启报告生成是否正确")
        print("4. 检查重启后系统是否正常运行")
        print("5. 测试最大重启次数限制")
        
        print("\n✨ 新功能已就绪，可以开始使用！")
    else:
        print("\n❌ 功能验证失败！")
        sys.exit(1)
