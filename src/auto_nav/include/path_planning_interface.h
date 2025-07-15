#ifndef PATH_PLANNING_INTERFACE_H
#define PATH_PLANNING_INTERFACE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace cv;
using namespace std;

// 路径点结构体
struct CellIndex
{
    int row;
    int col;
    double theta;       // {0,45,90,135,180,225,270,315}
    
    CellIndex() : row(0), col(0), theta(0) {}
    CellIndex(int r, int c, double t = 0) : row(r), col(c), theta(t) {}
};

// 路径规划算法类型枚举
enum class PlannerType {
    NEURAL_NETWORK,   // 原有的神经网络算法
    ASTAR,           // A*算法
    DSTAR,           // D*算法
    MCP              // MCP算法 (假设是某种覆盖路径规划算法)
};

// 路径规划算法基类接口
class PathPlannerInterface
{
public:
    PathPlannerInterface() = default;
    virtual ~PathPlannerInterface() = default;

    // 初始化算法
    virtual bool initialize(const Mat& map, const CellIndex& start_point) = 0;
    
    // 计算路径
    virtual vector<CellIndex> planPath() = 0;
    
    // 设置算法参数
    virtual void setParameters(const std::map<std::string, double>& params) {}
    
    // 获取算法名称
    virtual std::string getAlgorithmName() const = 0;
    
    // 检查边界
    virtual bool isValidCell(int row, int col) const {
        return row >= 0 && row < map_rows_ && col >= 0 && col < map_cols_;
    }
    
    // 检查是否为自由空间
    virtual bool isFreeSpace(int row, int col) const {
        if (!isValidCell(row, col)) return false;
        return cell_map_.at<uchar>(row, col) == 0; // 0表示自由空间
    }

protected:
    Mat cell_map_;           // 栅格地图
    CellIndex start_point_;  // 起始点
    int map_rows_;           // 地图行数
    int map_cols_;           // 地图列数
    vector<CellIndex> free_space_vec_;  // 自由空间向量
};

// 神经网络路径规划算法（原有算法）
class NeuralNetworkPlanner : public PathPlannerInterface
{
public:
    NeuralNetworkPlanner();
    virtual ~NeuralNetworkPlanner() = default;

    bool initialize(const Mat& map, const CellIndex& start_point) override;
    vector<CellIndex> planPath() override;
    void setParameters(const std::map<std::string, double>& params) override;
    std::string getAlgorithmName() const override { return "Neural Network"; }

private:
    Mat neural_mat_;         // 神经网络地图
    void initNeuralMat();    // 初始化神经网络矩阵
    bool boundingJudge(int a, int b);  // 边界判断
};

// A*路径规划算法
class AStarPlanner : public PathPlannerInterface
{
public:
    struct Node {
        CellIndex cell;
        double g_cost;    // 从起点到当前节点的代价
        double h_cost;    // 从当前节点到终点的启发式代价
        double f_cost;    // g_cost + h_cost
        Node* parent;
        
        Node(const CellIndex& c) : cell(c), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}
    };

    AStarPlanner();
    virtual ~AStarPlanner() = default;

    bool initialize(const Mat& map, const CellIndex& start_point) override;
    vector<CellIndex> planPath() override;
    void setParameters(const std::map<std::string, double>& params) override;
    std::string getAlgorithmName() const override { return "A*"; }

private:
    vector<CellIndex> target_points_;  // 目标点集合
    double heuristic_weight_;          // 启发式权重
    
    double calculateHeuristic(const CellIndex& current, const CellIndex& goal);
    vector<CellIndex> getNeighbors(const CellIndex& current);
    vector<CellIndex> reconstructPath(Node* goal_node);
    void generateCoverageTargets();    // 生成覆盖路径的目标点
};

// D*路径规划算法
class DStarPlanner : public PathPlannerInterface
{
public:
    DStarPlanner();
    virtual ~DStarPlanner() = default;

    bool initialize(const Mat& map, const CellIndex& start_point) override;
    vector<CellIndex> planPath() override;
    void setParameters(const std::map<std::string, double>& params) override;
    std::string getAlgorithmName() const override { return "D*"; }

private:
    // D*算法的具体实现细节
    vector<vector<double>> cost_map_;
    void computeShortestPath();
};

// MCP (Coverage Path Planning) 算法
class MCPPlanner : public PathPlannerInterface
{
public:
    MCPPlanner();
    virtual ~MCPPlanner() = default;

    bool initialize(const Mat& map, const CellIndex& start_point) override;
    vector<CellIndex> planPath() override;
    void setParameters(const std::map<std::string, double>& params) override;
    std::string getAlgorithmName() const override { return "MCP"; }

private:
    int coverage_pattern_;  // 覆盖模式：0-蛇形，1-螺旋形，2-分区覆盖
    void generateSnakePattern(vector<CellIndex>& path);
    void generateSpiralPattern(vector<CellIndex>& path);
    void generateZonePattern(vector<CellIndex>& path);
};

// 路径规划算法工厂类
class PathPlannerFactory
{
public:
    static std::shared_ptr<PathPlannerInterface> createPlanner(PlannerType type);
    static PlannerType stringToType(const std::string& type_str);
    static std::string typeToString(PlannerType type);
    static std::vector<std::string> getAvailableAlgorithms();
};

#endif // PATH_PLANNING_INTERFACE_H
