#include "path_planning.h"
#include "astar_algorithm.h"
#include "neural_algorithm.h"
#include <costmap_2d/cost_values.h>
#include <ros/ros.h>


PathPlanning::PathPlanning(costmap_2d::Costmap2DROS *costmap2dRos)
{
    m_cosmap2dRos = costmap2dRos;
    m_costmap2d = costmap2dRos->getCostmap();

    ros::NodeHandle private_nh;
    m_planPub = private_nh.advertise<nav_msgs::Path>("plan_path", 1);               // 用于在rviz中画出规划路径
    m_gridPub = private_nh.advertise<nav_msgs::OccupancyGrid>("covered_grid", 1);   // 用于在rviz中画出已清扫区域

    string sizeOfCellString, coveredValueStr;
    m_cellSize = 3;
    if (private_nh.searchParam("size_of_cell", sizeOfCellString))   // 搜索参数,根据名称"size of cell"搜索参数，将对应名称下的参数值赋给sizeOfCellString.
        private_nh.param("size_of_cell", m_cellSize, 3);            // 设置机器人占据n*n的栅格，决定规划的稀疏   

    m_gridCoveredValue = 0;
    if (private_nh.searchParam("grid_covered_value", coveredValueStr))
        private_nh.param("grid_covered_value", m_gridCoveredValue, 0);

    // 初始化算法接口 - 从参数获取算法类型，默认为神经网络算法
    string algorithm_type = "neural";
    private_nh.param("path_planning_algorithm", algorithm_type, string("neural"));
    
    ROS_INFO("PathPlanning: Initializing with algorithm: %s", algorithm_type.c_str());

    // 获取原始地图尺寸
    int costmap2dX = m_costmap2d->getSizeInCellsX();
    int costmap2dY = m_costmap2d->getSizeInCellsY();

    m_srcMap = Mat(costmap2dY, costmap2dX, CV_8U);  // 先raws后cols
    // Mat中原点在左上，Costmap2D中原点在左下，需要做一个变换
    for (int r=0; r<costmap2dY; r++) {
        for (int c=0; c<costmap2dX; c++) {
            m_srcMap.at<uchar>(r, c) = m_costmap2d->getCost(c, costmap2dY-r-1);
        }
    }

    initMat();  
    initCoveredGrid();

    // 初始化路径规划算法
    setAlgorithm(algorithm_type);

    if (!m_srcMap.empty() && m_algorithm)
        m_initialized = true;
    else
        m_initialized = false;
        
    ROS_INFO("PathPlanning: Initialization %s", m_initialized ? "successful" : "failed");
}


vector<geometry_msgs::PoseStamped> PathPlanning::getPathInROS()
{
    if (!m_pathVecInROS.empty()) m_pathVecInROS.clear();
    geometry_msgs::PoseStamped posestamped;
    geometry_msgs::Pose pose;
    getPathInCV();      // 得到m_pathVec，即在CV中的路径
    
    /*trasnsform*/
    vector<CellIndex>::iterator iter;
    int sizey = m_cellMat.rows;

    for (iter = m_pathVec.begin(); iter != m_pathVec.end(); iter++)
    {
        m_costmap2d->mapToWorld((*iter).col * m_cellSize + m_cellSize / 2, (sizey - (*iter).row - 1) * m_cellSize + m_cellSize / 2, pose.position.x, pose.position.y);
        pose.orientation.w = cos((*iter).theta * PI / 180 / 2); //(sizey-(*iter).row-1)
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = sin((*iter).theta * PI / 180 / 2);
        posestamped.header.stamp = ros::Time::now();
        posestamped.header.frame_id = "map";
        posestamped.pose = pose;

        m_pathVecInROS.push_back(posestamped);
    }
    // publishPlan(m_pathVecInROS);
    return m_pathVecInROS;
}


void PathPlanning::initMat()
{
    // 以下代码为初始化m_cellMat和m_freeSpaceVec
    m_cellMat = Mat(m_srcMap.rows / m_cellSize, m_srcMap.cols / m_cellSize, m_srcMap.type()); // cellMat是以之前规定的cell为最小单位重新划分的地图
    m_freeSpaceVec.clear();
    bool isFree = true;
    for (int r=0; r<m_cellMat.rows; r++)
    {
        for (int c=0; c<m_cellMat.cols; c++)
        {
            isFree = true;
            // 新划分的地图栅格中若有一个原来栅格是有障碍的，就把它所处的新单元格也设置为有障碍的
            for (int i=0; i<m_cellSize; i++)
            {
                for (int j=0; j<m_cellSize; j++)
                {
                    if (m_srcMap.at<uchar>(r * m_cellSize + i, c * m_cellSize + j) != costmap_2d::FREE_SPACE)
                    {
                        isFree = false;
                        break;
                    }
                }
            }
            if (isFree)
            {
                CellIndex ci;
                ci.row = r;
                ci.col = c;
                ci.theta = 0;
                m_freeSpaceVec.push_back(ci);
                m_cellMat.at<uchar>(r, c) = costmap_2d::FREE_SPACE;         // 0
            }
            else m_cellMat.at<uchar>(r, c) = costmap_2d::LETHAL_OBSTACLE;   // 254
        }
    }

    // 以下代码为初始化m_neuralMat
    m_neuralMat = Mat(m_cellMat.rows, m_cellMat.cols, CV_32F);  
    for (int i=0; i<m_neuralMat.rows; i++)
    {
        for (int j=0; j<m_neuralMat.cols; j++)
        {
            if (m_cellMat.at<uchar>(i, j) == costmap_2d::LETHAL_OBSTACLE)
                m_neuralMat.at<float>(i, j) = -100000.0; 
            else
                m_neuralMat.at<float>(i, j) = 50.0;
        }
    }
    return;
}


void PathPlanning::initCoveredGrid()
{
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(m_costmap2d->getMutex())); // 获取m_costmap2d的互斥锁，因为 Costmap2D 对象可能会被多个线程访问和修改
    double resolution = m_costmap2d->getResolution();       // 分辨率，即每个栅格单元的尺寸

    m_coveredPathGrid.header.frame_id = "map";              // m_coveredPathGrid是costmap中的占据栅格地图消息。
    m_coveredPathGrid.header.stamp = ros::Time::now();
    m_coveredPathGrid.info.resolution = resolution;

    m_coveredPathGrid.info.width = m_costmap2d->getSizeInCellsX();
    m_coveredPathGrid.info.height = m_costmap2d->getSizeInCellsY();

    double wx, wy;
    m_costmap2d->mapToWorld(0, 0, wx, wy);              // 从地图坐标系转换至世界坐标系。
    m_coveredPathGrid.info.origin.position.x = wx - resolution / 2;
    m_coveredPathGrid.info.origin.position.y = wy - resolution / 2;
    m_coveredPathGrid.info.origin.position.z = 0.0;
    m_coveredPathGrid.info.origin.orientation.w = 1.0;

    m_coveredPathGrid.data.resize(m_coveredPathGrid.info.width * m_coveredPathGrid.info.height);

    unsigned char *data = m_costmap2d->getCharMap();
    for (unsigned int i = 0; i < m_coveredPathGrid.data.size(); i++)
        m_coveredPathGrid.data[i] = data[i];             // 将代价值赋予到栅格地图的每个对应栅格

}


void PathPlanning::getPathInCV()
{
    if (!m_algorithm) {
        ROS_ERROR("PathPlanning: No algorithm set!");
        return;
    }

    CellIndex initPoint;
    initPoint.theta = 0;
    bool isok = m_cosmap2dRos->getRobotPose(m_initPose);    // Get the pose of the robot in the global frame of the costmap

    unsigned int mx, my;
    double wx = m_initPose.pose.position.x;     // 获取原点的x坐标（世界）
    double wy = m_initPose.pose.position.y;     // 获取原点的y坐标（世界）
    bool getmapcoor = m_costmap2d->worldToMap(wx, wy, mx, my);
    initPoint.row = m_cellMat.rows - my / m_cellSize - 1;   // 初始点的转换
    initPoint.col = mx / m_cellSize;

    ROS_INFO("PathPlanning: Starting path planning with %s from (%d, %d)", 
             m_algorithm->getAlgorithmName().c_str(), initPoint.row, initPoint.col);

    // 使用选定的算法进行路径规划
    m_pathVec = m_algorithm->planPath(initPoint.row, initPoint.col, m_freeSpaceVec, m_cellMat);
    
    ROS_INFO("PathPlanning: Generated path with %lu points", m_pathVec.size());
}


bool PathPlanning::boundingJudge(int a, int b)
{
    // 检查边界
    if (a < 0 || a >= m_cellMat.rows || b < 0 || b >= m_cellMat.cols) {
        return false;
    }
    
    // 检查是否为自由空间（简化版本）
    return m_cellMat.at<uchar>(a, b) == 0;  // 0表示自由空间
}


void PathPlanning::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
{
    if (!m_initialized)
    {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
    }

    m_planPub.publish(gui_path);
}


void PathPlanning::publishCoveragePath()
{
    publishPlan(this->m_pathVecInROS);
}

// 算法管理接口实现
void PathPlanning::setAlgorithm(const string& algorithm_type) {
    m_algorithmType = algorithm_type;
    m_algorithm = PathPlanningAlgorithmFactory::createAlgorithm(algorithm_type);
    
    if (m_algorithm) {
        bool init_success = m_algorithm->initialize(m_costmap2d, m_cellSize);
        if (init_success) {
            ROS_INFO("PathPlanning: Successfully switched to %s", m_algorithm->getAlgorithmName().c_str());
            ROS_INFO("PathPlanning: %s", m_algorithm->getParameterDescription().c_str());
        } else {
            ROS_ERROR("PathPlanning: Failed to initialize %s", algorithm_type.c_str());
            m_algorithm.reset();
        }
    } else {
        ROS_ERROR("PathPlanning: Failed to create algorithm: %s", algorithm_type.c_str());
        
        // 显示可用算法
        vector<string> available = getAvailableAlgorithms();
        ROS_INFO("Available algorithms:");
        for (const auto& alg : available) {
            ROS_INFO("  - %s", alg.c_str());
        }
    }
}

void PathPlanning::setAlgorithmParameter(const string& param_name, double param_value) {
    if (m_algorithm) {
        m_algorithm->setParameter(param_name, param_value);
        ROS_INFO("PathPlanning: Set parameter %s = %.3f for %s", 
                 param_name.c_str(), param_value, m_algorithm->getAlgorithmName().c_str());
    } else {
        ROS_WARN("PathPlanning: No algorithm set, cannot set parameter %s", param_name.c_str());
    }
}

string PathPlanning::getCurrentAlgorithmName() const {
    if (m_algorithm) {
        return m_algorithm->getAlgorithmName();
    }
    return "None";
}

vector<string> PathPlanning::getAvailableAlgorithms() const {
    return PathPlanningAlgorithmFactory::getAvailableAlgorithms();
}

