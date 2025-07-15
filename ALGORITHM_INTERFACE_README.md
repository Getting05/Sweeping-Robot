# 扫地机器人路径规划算法接口系统

## 项目概述

本项目为扫地机器人提供了一个可插拔的路径规划算法接口系统，支持多种路径规划算法的动态切换，包括A*算法和神经网络算法。系统采用工厂模式和策略模式，便于扩展新的算法。

## 主要特性

### ✨ 核心功能
- **可插拔算法接口**: 支持多种路径规划算法
- **动态算法切换**: 运行时无需重启即可切换算法
- **参数实时调整**: 支持在线调整算法参数
- **扩展性设计**: 易于添加新的路径规划算法

### 🔧 支持的算法
1. **A*算法** (`astar`)
   - 基于启发式搜索的最优路径规划
   - 支持完整覆盖路径规划
   - 可配置启发式权重和移动代价
   - 多种覆盖策略：最近优先、最远优先、螺旋式

2. **神经网络算法** (`neural`)
   - 传统神经网络路径规划方法
   - 考虑方向变化惩罚
   - 适合连续路径生成

## 文件结构

```
src/auto_nav/
├── include/
│   ├── path_planning_algorithm.h      # 算法接口基类
│   ├── astar_algorithm.h              # A*算法头文件
│   ├── neural_algorithm.h             # 神经网络算法头文件
│   └── path_planning.h                # 主路径规划类
├── src/
│   ├── path_planning_algorithm.cpp    # 算法工厂实现
│   ├── astar_algorithm.cpp            # A*算法实现
│   ├── neural_algorithm.cpp           # 神经网络算法实现
│   ├── path_planning.cpp              # 主路径规划类实现
│   └── path_planning_node.cpp         # ROS节点入口
├── srv/
│   ├── SetPathPlanningAlgorithm.srv   # 算法切换服务
│   └── SetAlgorithmParameter.srv      # 参数设置服务
├── config/
│   └── path_planning_params.yaml      # 算法配置文件
├── launch/
│   └── sequential_clean_with_algorithms.launch  # 支持算法选择的启动文件
└── scripts/
    └── algorithm_demo.py               # 算法切换演示脚本
```

## 快速开始

### 1. 编译项目

```bash
cd /home/getting/Sweeping-Robot
catkin_make
source devel/setup.bash
```

### 2. 启动系统

#### 使用A*算法启动：
```bash
roslaunch auto_nav sequential_clean_with_algorithms.launch algorithm:=astar
```

#### 使用神经网络算法启动：
```bash
roslaunch auto_nav sequential_clean_with_algorithms.launch algorithm:=neural
```

### 3. 动态切换算法

#### 使用ROS服务：
```bash
# 切换到A*算法
rosservice call /path_planning/set_algorithm "algorithm_type: 'astar'"

# 切换到神经网络算法
rosservice call /path_planning/set_algorithm "algorithm_type: 'neural'"

# 设置A*算法参数
rosservice call /path_planning/set_parameter "param_name: 'heuristic_weight' param_value: 1.2"
```

#### 使用演示脚本：
```bash
# 自动演示
rosrun auto_nav algorithm_demo.py

# 交互式模式
rosrun auto_nav algorithm_demo.py interactive
```

## 配置说明

### 算法配置文件 (`config/path_planning_params.yaml`)

```yaml
# 默认算法类型
path_planning_algorithm: "astar"

# 栅格参数
size_of_cell: 3
grid_covered_value: 0

# A*算法参数
astar:
  heuristic_weight: 1.0        # 启发式权重
  move_cost_straight: 1.0      # 直线移动代价
  move_cost_diagonal: 1.414    # 对角线移动代价
  max_iterations: 10000        # 最大迭代次数
  coverage_strategy: "nearest" # 覆盖策略

# 神经网络算法参数
neural:
  c_0: 50.0                    # 神经网络参数
  max_iterations: 9000         # 最大迭代次数
```

## API接口

### ROS服务

#### 1. 算法切换服务
- **服务名**: `/path_planning/set_algorithm`
- **类型**: `auto_nav/SetPathPlanningAlgorithm`
- **请求**: `algorithm_type` (string)
- **响应**: `success` (bool), `message` (string), `available_algorithms` (string[])

#### 2. 参数设置服务
- **服务名**: `/path_planning/set_parameter`
- **类型**: `auto_nav/SetAlgorithmParameter`
- **请求**: `param_name` (string), `param_value` (float64)
- **响应**: `success` (bool), `message` (string)

### C++ API

```cpp
#include "path_planning.h"

// 创建路径规划器
PathPlanning planner(costmap_ros);

// 切换算法
planner.setAlgorithm("astar");

// 设置参数
planner.setAlgorithmParameter("heuristic_weight", 1.2);

// 获取当前算法
std::string current = planner.getCurrentAlgorithmName();

// 获取可用算法
std::vector<std::string> algorithms = planner.getAvailableAlgorithms();
```

## 扩展新算法

### 1. 创建算法类

```cpp
class MyAlgorithm : public PathPlanningAlgorithm {
public:
    bool initialize(costmap_2d::Costmap2D* costmap, int cell_size) override;
    vector<CellIndex> planPath(int start_row, int start_col, 
                              const vector<CellIndex>& free_space_vec,
                              const Mat& cell_mat) override;
    string getAlgorithmName() const override { return "My Algorithm"; }
    // ... 实现其他虚函数
};
```

### 2. 注册算法

```cpp
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

### 3. 更新编译配置

在 `CMakeLists.txt` 中添加源文件到 `path_planning` 目标。

## 性能对比

| 算法 | 路径质量 | 计算速度 | 覆盖率 | 适用场景 |
|------|----------|----------|--------|----------|
| A*   | 优秀     | 中等     | 高     | 精确覆盖 |
| Neural | 良好   | 快       | 中等   | 快速响应 |

## 测试和验证

### 运行测试脚本
```bash
./test_algorithm_interface.sh
```

### 手动测试
1. 启动系统
2. 检查算法切换功能
3. 验证参数设置功能
4. 观察路径规划效果

## 故障排除

### 常见问题

1. **编译错误**
   - 检查所有头文件路径
   - 确保CMakeLists.txt配置正确
   - 验证依赖包是否安装

2. **服务不可用**
   - 确保path_planning节点正在运行
   - 检查ROS网络配置
   - 验证服务定义文件

3. **算法切换失败**
   - 检查算法名称拼写
   - 验证算法是否正确注册
   - 查看日志输出

### 调试技巧

```bash
# 查看可用服务
rosservice list | grep path_planning

# 检查节点状态
rosnode info /path_planning

# 查看日志
rostopic echo /rosout | grep path_planning
```

## 未来规划

### 待实现算法
- [ ] RRT (Rapidly-exploring Random Tree)
- [ ] Dijkstra算法
- [ ] 遗传算法
- [ ] 深度强化学习

### 功能增强
- [ ] 算法性能基准测试
- [ ] 可视化算法对比
- [ ] 路径质量评估指标
- [ ] 自适应算法选择

## 贡献指南

1. Fork项目仓库
2. 创建功能分支
3. 实现新算法或功能
4. 添加相应测试
5. 提交Pull Request

## 许可证

本项目遵循MIT许可证。

## 联系信息

如有问题或建议，请通过以下方式联系：
- 创建Issue
- 发送邮件

---

**注意**: 这是一个演示项目，展示了如何为ROS系统设计可插拔的算法接口。在实际应用中，请根据具体需求调整算法参数和配置。
