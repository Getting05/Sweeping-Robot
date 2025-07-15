# 路径规划算法接口使用指南

## 概述

本接口系统为扫地机器人提供了灵活的路径规划算法切换功能，支持多种算法：
- **neural_network**: 原有的神经网络算法
- **astar**: A*路径规划算法
- **dstar**: D*路径规划算法
- **mcp**: 覆盖路径规划算法（支持蛇形、螺旋、分区模式）

## 核心文件

### 接口文件
- `path_planning_interface.h` - 算法接口定义
- `path_planning_interface.cpp` - 算法实现
- `path_planning.h` - 修改后的主类头文件
- `path_planning.cpp` - 修改后的主类实现

### 配置文件
- `config/path_planning_algorithms.yaml` - 算法参数配置

### 演示文件
- `algorithm_switcher_demo.cpp` - 算法切换演示节点

## 使用方法

### 1. 编译项目

```bash
cd /home/getting/Sweeping-Robot
catkin_make
source devel/setup.bash
```

### 2. 配置算法参数

编辑 `src/auto_nav/config/path_planning_algorithms.yaml`：

```yaml
# 选择算法类型
algorithm_type: "astar"  # 可选: neural_network, astar, dstar, mcp

# A*算法参数
heuristic_weight: 1.5

# MCP算法参数 (覆盖模式)
coverage_pattern: 0  # 0-蛇形, 1-螺旋, 2-分区

# 保持原有参数不变
size_of_cell: 3
grid_covered_value: 0
```

### 3. 在launch文件中使用

修改 `sequential_clean.launch`，添加算法配置：

```xml
<!-- 路径规划器 -->
<node pkg="auto_nav" type="path_planning" respawn="false" name="path_planning" output="screen" clear_params="true">
  <rosparam file="$(find auto_nav)/config/costmap_common_params.yaml" command="load" ns="cleaning_costmap" />
  <rosparam file="$(find auto_nav)/config/cleaning_costmap_params.yaml" command="load" />
  <rosparam file="$(find auto_nav)/config/path_planning_algorithms.yaml" command="load" />
</node>
```

### 4. 运行时切换算法

使用ROS话题动态切换算法：

```bash
# 切换到A*算法
rostopic pub /switch_algorithm std_msgs/String "data: 'astar'"

# 切换到神经网络算法
rostopic pub /switch_algorithm std_msgs/String "data: 'neural_network'"

# 切换到MCP算法
rostopic pub /switch_algorithm std_msgs/String "data: 'mcp'"
```

## 算法详细说明

### 1. Neural Network (神经网络算法)
- **特点**: 原有算法，基于神经网络的动态规划
- **适用场景**: 复杂环境的自适应路径规划
- **参数**: 无特殊参数

### 2. A* Algorithm (A*算法)
- **特点**: 基于启发式搜索的最优路径规划
- **适用场景**: 已知目标点的点到点路径规划
- **参数**: 
  - `heuristic_weight`: 启发式权重 (默认: 1.0)

### 3. D* Algorithm (D*算法)
- **特点**: 动态环境下的增量式路径规划
- **适用场景**: 动态障碍物环境
- **参数**: 无特殊参数

### 4. MCP Algorithm (覆盖路径规划)
- **特点**: 专门为区域覆盖设计的路径规划
- **适用场景**: 扫地机器人的完全覆盖任务
- **参数**:
  - `coverage_pattern`: 覆盖模式
    - 0: 蛇形模式 (推荐)
    - 1: 螺旋模式
    - 2: 分区模式

## 编程接口

### 在代码中切换算法

```cpp
#include "path_planning.h"

// 创建路径规划实例
PathPlanning path_planner(&costmap_ros);

// 切换到A*算法
path_planner.setAlgorithmType("astar");

// 设置算法参数
std::map<std::string, double> params;
params["heuristic_weight"] = 1.5;
path_planner.setAlgorithmParameters(params);

// 获取当前算法名称
std::string current_algo = path_planner.getCurrentAlgorithmName();
ROS_INFO("Current algorithm: %s", current_algo.c_str());

// 获取可用算法列表
auto algorithms = path_planner.getAvailableAlgorithms();
for (const auto& algo : algorithms) {
    ROS_INFO("Available: %s", algo.c_str());
}
```

## 性能对比

| 算法 | 计算速度 | 路径质量 | 内存使用 | 适用场景 |
|------|----------|----------|----------|----------|
| Neural Network | 中等 | 好 | 高 | 复杂环境 |
| A* | 快 | 最优 | 中等 | 点到点规划 |
| D* | 慢 | 好 | 高 | 动态环境 |
| MCP | 快 | 专用 | 低 | 区域覆盖 |

## 故障排除

### 1. 编译错误
```bash
# 确保包含了正确的头文件
find_package(OpenCV REQUIRED)

# 检查CMakeLists.txt中的依赖
```

### 2. 运行时错误
```bash
# 检查参数配置
rosparam list | grep algorithm

# 查看当前算法状态
rostopic echo /algorithm_status
```

### 3. 算法切换失败
- 检查算法名称拼写
- 确认算法已正确实现
- 查看ROS日志获取详细错误信息

## 扩展新算法

要添加新的路径规划算法：

1. 继承 `PathPlannerInterface` 基类
2. 实现必要的虚函数
3. 在 `PathPlannerFactory` 中添加创建逻辑
4. 更新配置文件和文档

```cpp
class MyCustomPlanner : public PathPlannerInterface
{
public:
    bool initialize(const Mat& map, const CellIndex& start_point) override;
    vector<CellIndex> planPath() override;
    std::string getAlgorithmName() const override { return "my_custom"; }
};
```

## 注意事项

1. **兼容性**: 保持与原有参数的兼容性
2. **性能**: 不同算法有不同的性能特点
3. **内存**: 某些算法可能需要更多内存
4. **实时性**: 考虑算法的实时性要求
5. **参数调优**: 根据实际环境调整算法参数

## 技术支持

如有问题，请检查：
1. ROS日志: `rosnode info path_planning`
2. 参数配置: `rosparam get /algorithm_type`
3. 编译日志: `catkin_make` 输出
