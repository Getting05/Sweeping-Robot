#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
扫地机器人实时CSV数据分析脚本
用于分析coverage_monitor.py生成的实时CSV数据
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
import sys

def find_latest_csv():
    """查找最新的实时CSV文件"""
    pattern = "/home/getting/tmp/sweeping_robot_realtime_data_*.csv"
    files = glob.glob(pattern)
    
    if not files:
        print("未找到实时CSV数据文件")
        print(f"查找路径: {pattern}")
        return None
    
    # 按修改时间排序，返回最新的
    latest_file = max(files, key=os.path.getmtime)
    print(f"找到最新CSV文件: {latest_file}")
    return latest_file

def load_and_validate_data(filename):
    """加载并验证CSV数据"""
    try:
        df = pd.read_csv(filename)
        print(f"数据加载成功，共 {len(df)} 行记录")
        
        # 检查必要列是否存在
        required_columns = [
            'Timestamp', 'Runtime_min', 'Coverage_Rate', 
            'Motion_Efficiency', 'Avg_Velocity'
        ]
        
        missing_columns = [col for col in required_columns if col not in df.columns]
        if missing_columns:
            print(f"警告：缺少必要列: {missing_columns}")
        
        return df
        
    except Exception as e:
        print(f"数据加载失败: {e}")
        return None

def analyze_performance_metrics(df):
    """分析性能指标"""
    print("\n=== 性能指标分析 ===")
    
    if len(df) == 0:
        print("无数据可分析")
        return
    
    # 基本统计
    latest = df.iloc[-1]
    print(f"最新记录时间: {latest['Timestamp']}")
    print(f"总运行时间: {latest['Runtime_min']:.1f} 分钟")
    print(f"当前覆盖率: {latest['Coverage_Rate']*100:.2f}%")
    print(f"最终路径长度: {latest['Path_Length']:.2f} 米")
    
    # 性能趋势分析
    print("\n--- 覆盖率趋势 ---")
    coverage_change = df['Coverage_Rate'].diff().mean() * 100
    print(f"平均覆盖率增长速度: {coverage_change:.3f}%/记录")
    
    print("\n--- 运动效率趋势 ---")
    if 'Motion_Efficiency' in df.columns:
        avg_efficiency = df['Motion_Efficiency'].mean()
        print(f"平均运动效率: {avg_efficiency:.3f} m/m²")
        
        if avg_efficiency < 2.0:
            print("✓ 运动效率优秀")
        elif avg_efficiency < 4.0:
            print("◐ 运动效率良好")
        else:
            print("△ 运动效率需要优化")
    
    print("\n--- 碰撞情况 ---")
    if 'Collision_Count' in df.columns:
        final_collisions = latest['Collision_Count']
        print(f"总碰撞次数: {final_collisions}")
        
        if final_collisions == 0:
            print("✓ 无碰撞，安全性优秀")
        elif final_collisions <= 2:
            print("◐ 碰撞较少，安全性良好")
        else:
            print("△ 碰撞较多，需要优化")

def create_performance_plots(df, output_dir="/home/getting/tmp"):
    """创建性能分析图表"""
    print("\n=== 生成性能图表 ===")
    
    if len(df) < 2:
        print("数据点不足，无法生成图表")
        return
    
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei']
    plt.rcParams['axes.unicode_minus'] = False
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('Sweeping Robot Performance Analysis', fontsize=16)
    
    # 1. 覆盖率随时间变化
    axes[0, 0].plot(df['Runtime_min'], df['Coverage_Rate'] * 100, 'b-', linewidth=2)
    axes[0, 0].set_title('Coverage Rate Over Time')
    axes[0, 0].set_xlabel('Runtime (minutes)')
    axes[0, 0].set_ylabel('Coverage Rate (%)')
    axes[0, 0].grid(True)
    
    # 2. 运动效率变化
    if 'Motion_Efficiency' in df.columns:
        axes[0, 1].plot(df['Runtime_min'], df['Motion_Efficiency'], 'g-', linewidth=2)
        axes[0, 1].set_title('Motion Efficiency Over Time')
        axes[0, 1].set_xlabel('Runtime (minutes)')
        axes[0, 1].set_ylabel('Motion Efficiency (m/m²)')
        axes[0, 1].grid(True)
    
    # 3. 平均速度变化
    if 'Avg_Velocity' in df.columns:
        axes[0, 2].plot(df['Runtime_min'], df['Avg_Velocity'], 'r-', linewidth=2)
        axes[0, 2].set_title('Average Velocity Over Time')
        axes[0, 2].set_xlabel('Runtime (minutes)')
        axes[0, 2].set_ylabel('Average Velocity (m/s)')
        axes[0, 2].grid(True)
    
    # 4. 碰撞累积
    if 'Collision_Count' in df.columns:
        axes[1, 0].plot(df['Runtime_min'], df['Collision_Count'], 'orange', linewidth=2, marker='o')
        axes[1, 0].set_title('Collision Count Over Time')
        axes[1, 0].set_xlabel('Runtime (minutes)')
        axes[1, 0].set_ylabel('Collision Count')
        axes[1, 0].grid(True)
    
    # 5. 路径长度增长
    if 'Path_Length' in df.columns:
        axes[1, 1].plot(df['Runtime_min'], df['Path_Length'], 'purple', linewidth=2)
        axes[1, 1].set_title('Path Length Growth')
        axes[1, 1].set_xlabel('Runtime (minutes)')
        axes[1, 1].set_ylabel('Path Length (m)')
        axes[1, 1].grid(True)
    
    # 6. 冗余度变化
    if 'Redundancy' in df.columns:
        axes[1, 2].plot(df['Runtime_min'], df['Redundancy'], 'brown', linewidth=2)
        axes[1, 2].set_title('Redundancy Over Time')
        axes[1, 2].set_xlabel('Runtime (minutes)')
        axes[1, 2].set_ylabel('Redundancy Ratio')
        axes[1, 2].grid(True)
    
    plt.tight_layout()
    
    # 保存图表
    plot_filename = os.path.join(output_dir, f"robot_performance_analysis_{int(pd.Timestamp.now().timestamp())}.png")
    plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
    print(f"图表已保存: {plot_filename}")
    
    # 显示图表
    plt.show()

def generate_summary_report(df):
    """生成汇总报告"""
    print("\n=== 生成汇总报告 ===")
    
    if len(df) == 0:
        return
    
    latest = df.iloc[-1]
    
    # 计算关键指标
    total_runtime = latest['Runtime_min']
    final_coverage = latest['Coverage_Rate'] * 100
    avg_efficiency = df['Motion_Efficiency'].mean() if 'Motion_Efficiency' in df.columns else 0
    total_collisions = latest['Collision_Count'] if 'Collision_Count' in df.columns else 0
    
    # 生成评级
    def get_rating(value, thresholds, higher_better=True):
        if higher_better:
            if value >= thresholds[0]:
                return "优秀"
            elif value >= thresholds[1]:
                return "良好"
            else:
                return "需要改进"
        else:
            if value <= thresholds[0]:
                return "优秀"
            elif value <= thresholds[1]:
                return "良好"
            else:
                return "需要改进"
    
    coverage_rating = get_rating(final_coverage, [90, 70])
    efficiency_rating = get_rating(avg_efficiency, [2.0, 4.0], False)
    safety_rating = get_rating(total_collisions, [0, 2], False)
    
    report = f"""
=== 扫地机器人性能汇总报告 ===
生成时间: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}
数据记录数: {len(df)}

--- 核心指标 ---
总运行时间: {total_runtime:.1f} 分钟
最终覆盖率: {final_coverage:.2f}% ({coverage_rating})
平均运动效率: {avg_efficiency:.3f} m/m² ({efficiency_rating})
总碰撞次数: {total_collisions} ({safety_rating})

--- 趋势分析 ---
覆盖率增长趋势: {'稳定增长' if df['Coverage_Rate'].diff().mean() > 0 else '增长缓慢'}
效率变化趋势: {'持续改善' if df['Motion_Efficiency'].diff().mean() < 0 else '有待优化'}

--- 性能评估 ---
整体性能: {get_rating((final_coverage + (100-avg_efficiency*10) + (100-total_collisions*10))/3, [80, 60])}

--- 建议 ---"""
    
    if final_coverage < 80:
        report += "\n• 覆盖率偏低，建议检查路径规划算法"
    if avg_efficiency > 3.0:
        report += "\n• 运动效率偏低，建议优化路径或增加清扫半径"
    if total_collisions > 2:
        report += "\n• 碰撞次数较多，建议改进避障算法"
    if len(df) < 10:
        report += "\n• 数据记录较少，建议延长测试时间"
    
    report += "\n============================="
    
    print(report)
    
    # 保存报告到文件
    report_filename = f"/home/getting/tmp/robot_analysis_report_{int(pd.Timestamp.now().timestamp())}.txt"
    with open(report_filename, 'w', encoding='utf-8') as f:
        f.write(report)
    
    print(f"\n报告已保存: {report_filename}")

def main():
    """主函数"""
    print("=== 扫地机器人实时数据分析工具 ===")
    
    # 查找CSV文件
    csv_file = find_latest_csv()
    if not csv_file:
        return
    
    # 加载数据
    df = load_and_validate_data(csv_file)
    if df is None:
        return
    
    # 分析性能指标
    analyze_performance_metrics(df)
    
    # 生成图表
    try:
        create_performance_plots(df)
    except Exception as e:
        print(f"生成图表时出错: {e}")
    
    # 生成汇总报告
    generate_summary_report(df)
    
    print("\n✓ 分析完成！")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n分析被用户中断")
    except Exception as e:
        print(f"分析过程中出错: {e}")
        sys.exit(1)
