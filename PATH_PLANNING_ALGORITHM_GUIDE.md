# 路径规划算法接口使用指南

## 概述

本系统提供了一个可插拔的路径规划算法接口，允许用户在不同的路径规划算法之间灵活切换。目前支持A*算法和神经网络算法，并且可以方便地扩展新的算法。

## 架构设计

### 核心组件

1. **PathPlanningAlgorithm**: 路径规划算法的抽象基类
2. **PathPlanningAlgorithmFactory**: 算法工厂类，负责创建和管理算法实例
3. **AStarAlgorithm**: A*算法的具体实现
4. **NeuralAlgorithm**: 神经网络算法的具体实现
5. **PathPlanning**: 主路径规划类，集成了算法接口

### 类图

```
PathPlanningAlgorithm (接口)
├── AStarAlgorithm
└── NeuralAlgorithm

PathPlanningAlgorithmFactory (工厂)
└── 创建和管理算法实例

PathPlanning (主类)
└── 使用算法接口进行路径规划
```

## 支持的算法

### 1. A*算法 (astar)

**特点:**
- 基于启发式搜索的最优路径规划
- 支持覆盖路径规划，自动访问所有自由空间
- 可配置的启发式权重和移动代价

**参数:**
- `heuristic_weight`: 启发式权重 (默认: 1.0)
- `move_cost_straight`: 直线移动代价 (默认: 1.0)
- `move_cost_diagonal`: 对角线移动代价 (默认: 1.414)
- `max_iterations`: 最大迭代次数 (默认: 10000)

**覆盖策略:**
- `nearest`: 优先访问最近的未覆盖点
- `farthest`: 优先访问最远的未覆盖点
- `spiral`: 螺旋式覆盖策略

### 2. 神经网络算法 (neural)

**特点:**
- 基于传统神经网络的路径规划方法
- 考虑方向变化的惩罚
- 适合连续的路径生成

**参数:**
- `c_0`: 神经网络权重参数 (默认: 50.0)
- `max_iterations`: 最大迭代次数 (默认: 9000)

## 使用方法

### 1. 配置文件设置

在 `config/path_planning_params.yaml` 中设置算法类型和参数：

```yaml
# 设置使用的路径规划算法类型
path_planning_algorithm: "astar"

# A*算法参数
astar:
  heuristic_weight: 1.0
  move_cost_straight: 1.0
  move_cost_diagonal: 1.414
  max_iterations: 10000
  coverage_strategy: "nearest"

# 神经网络算法参数
neural:
  c_0: 50.0
  max_iterations: 9000
```

### 2. 启动文件设置

在launch文件中加载参数：

```xml
<launch>
  <!-- 加载路径规划算法参数 -->
  <rosparam file="$(find auto_nav)/config/path_planning_params.yaml" command="load" />
  
  <!-- 路径规划器 -->
  <node pkg="auto_nav" type="path_planning" name="path_planning" output="screen">
    <param name="path_planning_algorithm" value="astar" />
  </node>
</launch>
```

### 3. 动态切换算法

#### 使用ROS服务

```bash
# 切换到A*算法
rosservice call /path_planning/set_algorithm "algorithm_type: 'astar'"

# 切换到神经网络算法
rosservice call /path_planning/set_algorithm "algorithm_type: 'neural'"

# 设置算法参数
rosservice call /path_planning/set_parameter "param_name: 'heuristic_weight' param_value: 1.2"
```

#### 使用演示脚本

```bash
# 运行自动演示
rosrun auto_nav algorithm_demo.py

# 运行交互式模式
rosrun auto_nav algorithm_demo.py interactive
```

### 4. 编程接口

在C++代码中使用：

```cpp
#include "path_planning.h"

// 创建路径规划器
PathPlanning planner(costmap_ros);

// 切换算法
planner.setAlgorithm("astar");

// 设置参数
planner.setAlgorithmParameter("heuristic_weight", 1.2);

// 获取当前算法
std::string current_alg = planner.getCurrentAlgorithmName();

// 获取可用算法列表
std::vector<std::string> algorithms = planner.getAvailableAlgorithms();
```

## 扩展新算法

### 1. 创建算法类

```cpp
#include "path_planning_algorithm.h"

class MyAlgorithm : public PathPlanningAlgorithm {
public:
    bool initialize(costmap_2d::Costmap2D* costmap, int cell_size) override {
        // 初始化算法
        return true;
    }
    
    vector<CellIndex> planPath(int start_row, int start_col, 
                              const vector<CellIndex>& free_space_vec,
                              const Mat& cell_mat) override {
        // 实现路径规划逻辑
        vector<CellIndex> path;
        // ... 算法实现 ...
        return path;
    }
    
    string getAlgorithmName() const override {
        return "My Algorithm";
    }
    
    // 实现其他虚函数...
};
```

### 2. 注册算法

```cpp
// 在算法实现文件末尾添加
class MyAlgorithmRegistrar {
public:
    MyAlgorithmRegistrar() {
        PathPlanningAlgorithmFactory::registerAlgorithm("myalgorithm", []() {
            return make_shared<MyAlgorithm>();
        });
    }
};

static MyAlgorithmRegistrar my_algorithm_registrar;
```

### 3. 更新编译文件

在 `CMakeLists.txt` 中添加新的源文件：

```cmake
add_executable(path_planning 
  src/path_planning.cpp 
  src/path_planning_node.cpp
  src/path_planning_algorithm.cpp
  src/astar_algorithm.cpp
  src/neural_algorithm.cpp
  src/my_algorithm.cpp  # 新增
)
```

## 性能对比

| 算法 | 适用场景 | 路径质量 | 计算复杂度 | 覆盖率 |
|------|----------|----------|------------|--------|
| A*   | 精确覆盖 | 优秀     | 中等       | 高     |
| Neural | 快速响应 | 良好     | 低         | 中等   |

## 调试和监控

### 1. 日志输出

系统会输出详细的日志信息：

```
[PathPlanning] Initializing with algorithm: astar
[A* Algorithm] Initialized with cell_size: 3
[A* Algorithm] Planning coverage path from (10, 15)
[A* Algorithm] Coverage planning completed. Total path points: 1250
```

### 2. 算法状态查询

```bash
# 查看当前算法状态
rostopic echo /path_planning/status

# 查看路径规划结果
rostopic echo /plan_path
```

### 3. 性能指标

- 路径点数量
- 规划时间
- 覆盖率
- 路径平滑度

## 常见问题

### Q1: 算法切换后路径质量下降？
A1: 检查算法参数设置，不同算法有不同的最优参数配置。

### Q2: 编译时找不到头文件？
A2: 确保所有头文件都在 `include/` 目录下，并且CMakeLists.txt正确配置。

### Q3: 自定义算法无法注册？
A3: 确保注册代码在静态初始化时执行，检查链接是否正确。

## 未来扩展

1. **RRT算法**: 快速随机树算法
2. **Dijkstra算法**: 经典最短路径算法
3. **遗传算法**: 进化计算方法
4. **强化学习**: 基于深度学习的路径规划

## 总结

本系统提供了一个灵活、可扩展的路径规划算法接口，支持多种算法的无缝切换和参数调整。通过模块化设计，用户可以轻松添加新的算法并进行性能对比，为不同的应用场景选择最适合的路径规划方法。
