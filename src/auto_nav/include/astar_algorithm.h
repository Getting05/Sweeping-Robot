#ifndef ASTAR_ALGORITHM_H
#define ASTAR_ALGORITHM_H

#include "path_planning_algorithm.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>

/**
 * @brief A*路径规划算法实现
 */
class AStarAlgorithm : public PathPlanningAlgorithm 
{
public:
    AStarAlgorithm();
    virtual ~AStarAlgorithm() = default;

    bool initialize(costmap_2d::Costmap2D* costmap, int cell_size) override;
    
    vector<CellIndex> planPath(int start_row, int start_col, 
                              const vector<CellIndex>& free_space_vec,
                              const Mat& cell_mat) override;
    
    string getAlgorithmName() const override;
    string getParameterDescription() const override;
    void setParameter(const string& param_name, double param_value) override;
    void reset() override;

private:
    struct Node {
        CellIndex cell;
        double g_cost;  // 从起点到当前节点的代价
        double h_cost;  // 从当前节点到终点的启发式代价
        double f_cost;  // f = g + h
        shared_ptr<Node> parent;
        
        Node(const CellIndex& c, double g = 0, double h = 0) 
            : cell(c), g_cost(g), h_cost(h), f_cost(g + h), parent(nullptr) {}
    };

    struct NodeComparator {
        bool operator()(const shared_ptr<Node>& a, const shared_ptr<Node>& b) const {
            if (abs(a->f_cost - b->f_cost) < 1e-6) {
                return a->h_cost > b->h_cost;  // f相等时选择h较小的
            }
            return a->f_cost > b->f_cost;
        }
    };

    // A*算法核心函数
    vector<CellIndex> findPathAStar(const CellIndex& start, const CellIndex& goal, 
                                   const Mat& cell_mat, const vector<CellIndex>& free_space);
    
    // 启发式函数（曼哈顿距离）
    double calculateHeuristic(const CellIndex& current, const CellIndex& goal);
    
    // 获取相邻节点
    vector<CellIndex> getNeighbors(const CellIndex& current, const Mat& cell_mat);
    
    // 检查节点是否在自由空间中
    bool isInFreeSpace(const CellIndex& cell, const vector<CellIndex>& free_space);
    
    // 计算两个相邻节点间的移动代价
    double calculateMoveCost(const CellIndex& from, const CellIndex& to);
    
    // 根据覆盖策略选择下一个目标点
    CellIndex selectNextTarget(const vector<CellIndex>& free_space, 
                              const vector<CellIndex>& visited);
    
    // 重建路径
    vector<CellIndex> reconstructPath(shared_ptr<Node> goal_node);
    
    // 覆盖路径规划 - 将所有自由空间连接起来
    vector<CellIndex> planCoveragePath(const CellIndex& start,
                                      const vector<CellIndex>& free_space,
                                      const Mat& cell_mat);

    // 算法参数
    double heuristic_weight_;    // 启发式权重
    double move_cost_straight_;  // 直线移动代价
    double move_cost_diagonal_;  // 对角线移动代价
    int max_iterations_;         // 最大迭代次数
    string coverage_strategy_;   // 覆盖策略："nearest", "farthest", "spiral"
    
    // 内部状态
    unordered_set<string> visited_cells_;  // 已访问的单元格
    CellIndex current_target_;             // 当前目标点
};

// 辅助函数：将CellIndex转换为字符串key
string cellToString(const CellIndex& cell);

// 辅助函数：计算两点间距离
double calculateDistance(const CellIndex& a, const CellIndex& b);

#endif // ASTAR_ALGORITHM_H
