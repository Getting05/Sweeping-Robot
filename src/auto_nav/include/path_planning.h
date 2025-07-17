#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <set>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace cv;
using namespace std;


constexpr double PI = 3.14159;


struct CellIndex
{
    int row;
    int col;
    double theta;       // {0,45,90,135,180,225,270,315}
};

// A*算法节点结构
struct AStarNode 
{
    int x, y;           // 栅格坐标
    double g_cost;      // 从起点到当前节点的实际代价
    double h_cost;      // 从当前节点到目标的启发式代价
    double f_cost;      // f = g + h
    AStarNode* parent;  // 父节点指针
    bool visited;       // 是否已访问
    
    AStarNode(int x = 0, int y = 0) : x(x), y(y), g_cost(0), h_cost(0), f_cost(0), parent(nullptr), visited(false) {}
    
    bool operator<(const AStarNode& other) const {
        return f_cost > other.f_cost; // 优先队列中f_cost小的优先
    }
};

// 覆盖路径规划的目标点结构
struct CoverageTarget 
{
    int x, y;
    bool covered;
    double priority;    // 优先级，用于决定访问顺序
    
    CoverageTarget(int x = 0, int y = 0) : x(x), y(y), covered(false), priority(0.0) {}
};


class PathPlanning
{
public:
    PathPlanning(costmap_2d::Costmap2DROS *costmap2dRos);         

    vector<geometry_msgs::PoseStamped> getPathInROS();      // 得到最终在ROS中的路径

    void publishCoveragePath();                             // 用于可视化

    int getSizeOfCell() { return this->m_cellSize; }        // 获取一个单元格有几个栅格，只能为奇数

private:
    void initMat();                         // 初始化m_cellMat和m_neuralMat
    void initCoveredGrid();                 // 初始化m_coveredPathGrid
    void getPathInCV();                     // 利用A*算法生成全覆盖路径
    bool boundingJudge(int a, int b);       // 判断
    void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);
    
    // A*算法相关方法
    vector<CellIndex> generateCoveragePath();                           // 生成全覆盖路径的主函数
    vector<AStarNode*> aStarPathPlanning(int start_x, int start_y, int goal_x, int goal_y);  // A*路径规划
    vector<CoverageTarget> generateCoverageTargets();                   // 生成覆盖目标点
    double calculateHeuristic(int x1, int y1, int x2, int y2);         // 计算启发式函数
    bool isValidCell(int x, int y);                                     // 检查栅格是否有效
    vector<pair<int, int>> getNeighbors(int x, int y);                  // 获取邻居节点
    void optimizePath(vector<CellIndex>& path);                         // 路径优化
    double calculatePathLength(const vector<CellIndex>& path);          // 计算路径长度
    CellIndex findNearestUnvisitedTarget(int current_x, int current_y, vector<CoverageTarget>& targets); // 寻找最近未访问目标

    bool m_initialized;                         // 类初始化成功与否标志位
    costmap_2d::Costmap2DROS *m_cosmap2dRos;    // A ROS wrapper for a 2D Costmap，处理订阅以PointCloud或LaserScan消息形式提供障碍物观察结果的主题。
    costmap_2d::Costmap2D* m_costmap2d;         // 原始代价地图的指针
    Mat m_srcMap;                               // 原始代价地图转为Mat
    Mat m_cellMat;                              // 原始栅格地图按m_CELL_SIZE的大小合并后的地图
    Mat m_neuralMat;                            // 该地图用于路径规划
    vector<CellIndex> m_freeSpaceVec;           // m_cellMat中无障碍的地方
    nav_msgs::OccupancyGrid m_coveredPathGrid;  // 占据栅格地图， 0 表示未占据, 1 表示占据, -1 表示未知 

    geometry_msgs::PoseStamped m_initPose;              // 初始位姿
    vector<CellIndex> m_pathVec;                        // 利用A*算法求得的全覆盖路径
    vector<geometry_msgs::PoseStamped> m_pathVecInROS;  // m_pathVec转到ros中的路径

    ros::Publisher m_planPub;               // 发布规划好的路径，rviz显示
    ros::Publisher m_gridPub;               // 发布机器人走过的路径，rviz显示

    int m_cellSize;             // 新的栅格大小，必须是原来栅格的奇数倍
    int m_gridCoveredValue;
    
    // A*算法相关成员变量
    vector<vector<bool>> m_visited;         // 记录已访问的栅格
    vector<CoverageTarget> m_targets;       // 覆盖目标点列表
    double m_coverageSpacing;               // 覆盖间距
    static constexpr double DIAGONAL_COST = 1.414;  // 对角线移动代价
    static constexpr double STRAIGHT_COST = 1.0;    // 直线移动代价    
};

#endif