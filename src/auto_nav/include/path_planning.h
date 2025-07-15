#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
#include <memory>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "path_planning_interface.h"

using namespace cv;
using namespace std;


constexpr double PI =3.14159;


class PathPlanning
{
public:
    PathPlanning(costmap_2d::Costmap2DROS *costmap2dRos);         

    vector<geometry_msgs::PoseStamped> getPathInROS();      // 得到最终在ROS中的路径
    void publishCoveragePath();                             // 用于可视化
    int getSizeOfCell() { return this->m_cellSize; }        // 获取一个单元格有几个栅格，只能为奇数
    
    // 新增的算法切换接口
    void setAlgorithmType(PlannerType type);                // 设置算法类型
    void setAlgorithmType(const std::string& type_str);     // 通过字符串设置算法类型
    void setAlgorithmParameters(const std::map<std::string, double>& params); // 设置算法参数
    std::string getCurrentAlgorithmName() const;            // 获取当前算法名称
    std::vector<std::string> getAvailableAlgorithms() const; // 获取可用算法列表

private:
    void initMat();                         // 初始化m_cellMat
    void initCoveredGrid();                 // 初始化m_coveredPathGrid
    void getPathInCV();                     // 利用算法接口求出规划路径，之后再转到ros中
    void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);
    bool boundingJudge(int a, int b);       // 边界判断函数

    bool m_initialized;                         // 类初始化成功与否标志位
    costmap_2d::Costmap2DROS *m_cosmap2dRos;    // A ROS wrapper for a 2D Costmap
    costmap_2d::Costmap2D* m_costmap2d;         // 原始代价地图的指针
    Mat m_srcMap;                               // 原始代价地图转为Mat
    Mat m_cellMat;                              // 原始栅格地图按m_CELL_SIZE的大小合并后的地图
    nav_msgs::OccupancyGrid m_coveredPathGrid;  // 占据栅格地图

    geometry_msgs::PoseStamped m_initPose;              // 初始位姿
    vector<CellIndex> m_pathVec;                        // 利用算法接口求得的路径
    vector<geometry_msgs::PoseStamped> m_pathVecInROS;  // m_pathVec转到ros中的路径

    ros::Publisher m_planPub;               // 发布规划好的路径，rviz显示
    ros::Publisher m_gridPub;               // 发布机器人走过的路径，rviz显示

    int m_cellSize;             // 新的栅格大小，必须是原来栅格的奇数倍
    int m_gridCoveredValue;
    
    // 新增的算法接口相关成员
    PlannerType m_currentPlannerType;                          // 当前使用的算法类型
    std::shared_ptr<PathPlannerInterface> m_pathPlanner;       // 路径规划算法实例
    std::map<std::string, double> m_algorithmParams;           // 算法参数    
};

#endif