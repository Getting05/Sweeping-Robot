#include "path_planning.h"
#include <costmap_2d/cost_values.h>
#include <ros/ros.h>


PathPlanning::PathPlanning(costmap_2d::Costmap2DROS *costmap2dRos)
{
    m_cosmap2dRos = costmap2dRos;
    m_costmap2d = costmap2dRos->getCostmap();

    ros::NodeHandle private_nh;
    m_planPub = private_nh.advertise<nav_msgs::Path>("plan_path", 1);               // 用于在rviz中画出规划路径
    m_gridPub = private_nh.advertise<nav_msgs::OccupancyGrid>("covered_grid", 1);   // 用于在rviz中画出已清扫区域

    // 读取参数
    string sizeOfCellString, coveredValueStr, algorithmTypeStr;
    m_cellSize = 3;
    if (private_nh.searchParam("size_of_cell", sizeOfCellString))   // 搜索参数,根据名称"size of cell"搜索参数，将对应名称下的参数值赋给sizeOfCellString.
        private_nh.param("size_of_cell", m_cellSize, 3);            // 设置机器人占据n*n的栅格，决定规划的稀疏   

    m_gridCoveredValue = 0;
    if (private_nh.searchParam("grid_covered_value", coveredValueStr))
        private_nh.param("grid_covered_value", m_gridCoveredValue, 0);

    // 读取算法类型参数
    private_nh.param("algorithm_type", algorithmTypeStr, string("neural_network"));
    m_currentPlannerType = PathPlannerFactory::stringToType(algorithmTypeStr);
    
    // 读取算法参数
    private_nh.param("heuristic_weight", m_algorithmParams["heuristic_weight"], 1.0);
    private_nh.param("coverage_pattern", m_algorithmParams["coverage_pattern"], 0.0);

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
    
    // 创建路径规划算法实例
    m_pathPlanner = PathPlannerFactory::createPlanner(m_currentPlannerType);

    if (!m_srcMap.empty())
        m_initialized = true;
    else
        m_initialized = false;
        
    ROS_INFO("PathPlanning initialized with algorithm: %s", getCurrentAlgorithmName().c_str());
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
    // 初始化m_cellMat - 将原始地图按cellSize重新划分
    m_cellMat = Mat(m_srcMap.rows / m_cellSize, m_srcMap.cols / m_cellSize, m_srcMap.type());
    
    // 清空并重新构建自由空间向量
    vector<CellIndex> m_freeSpaceVec; // 现在这个变量已经移到了接口中，这里创建一个局部的
    bool isFree = true;
    
    for (int r = 0; r < m_cellMat.rows; r++)
    {
        for (int c = 0; c < m_cellMat.cols; c++)
        {
            isFree = true;
            // 检查该cell内的所有原始栅格
            for (int i = 0; i < m_cellSize && isFree; i++)
            {
                for (int j = 0; j < m_cellSize && isFree; j++)
                {
                    int src_r = r * m_cellSize + i;
                    int src_c = c * m_cellSize + j;
                    
                    // 边界检查
                    if (src_r >= m_srcMap.rows || src_c >= m_srcMap.cols) {
                        isFree = false;
                        break;
                    }
                    
                    if (m_srcMap.at<uchar>(src_r, src_c) != costmap_2d::FREE_SPACE)
                    {
                        isFree = false;
                    }
                }
            }
            
            if (isFree)
            {
                m_cellMat.at<uchar>(r, c) = costmap_2d::FREE_SPACE;         // 0
            }
            else 
            {
                m_cellMat.at<uchar>(r, c) = costmap_2d::LETHAL_OBSTACLE;   // 254
            }
        }
    }
    
    ROS_INFO("Cell map initialized: %dx%d cells (from %dx%d original map)", 
             m_cellMat.rows, m_cellMat.cols, m_srcMap.rows, m_srcMap.cols);
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
    if (!m_pathPlanner) {
        ROS_ERROR("Path planner not initialized!");
        return;
    }

    CellIndex initPoint;
    initPoint.theta = 0;
    
    // 获取机器人位置
    bool isok = m_cosmap2dRos->getRobotPose(m_initPose);
    if (!isok) {
        ROS_ERROR("Cannot get robot pose!");
        return;
    }

    unsigned int mx, my;
    double wx = m_initPose.pose.position.x;
    double wy = m_initPose.pose.position.y;
    bool getmapcoor = m_costmap2d->worldToMap(wx, wy, mx, my);
    if (!getmapcoor) {
        ROS_ERROR("Cannot convert world coordinates to map coordinates!");
        return;
    }
    
    initPoint.row = m_cellMat.rows - my / m_cellSize - 1;
    initPoint.col = mx / m_cellSize;

    // 设置算法参数
    m_pathPlanner->setParameters(m_algorithmParams);
    
    // 初始化算法
    if (!m_pathPlanner->initialize(m_cellMat, initPoint)) {
        ROS_ERROR("Failed to initialize path planner!");
        return;
    }

    ROS_INFO("Using %s algorithm for path planning", m_pathPlanner->getAlgorithmName().c_str());
    
    // 执行路径规划
    m_pathVec = m_pathPlanner->planPath();
    
    ROS_INFO("Path planning completed. Generated %zu waypoints", m_pathVec.size());
}


void PathPlanning::setAlgorithmType(PlannerType type)
{
    if (type != m_currentPlannerType) {
        m_currentPlannerType = type;
        m_pathPlanner = PathPlannerFactory::createPlanner(type);
        ROS_INFO("Switched to algorithm: %s", getCurrentAlgorithmName().c_str());
    }
}

void PathPlanning::setAlgorithmType(const std::string& type_str)
{
    PlannerType type = PathPlannerFactory::stringToType(type_str);
    setAlgorithmType(type);
}

void PathPlanning::setAlgorithmParameters(const std::map<std::string, double>& params)
{
    m_algorithmParams = params;
    if (m_pathPlanner) {
        m_pathPlanner->setParameters(params);
    }
}

std::string PathPlanning::getCurrentAlgorithmName() const
{
    if (m_pathPlanner) {
        return m_pathPlanner->getAlgorithmName();
    }
    return "None";
}

std::vector<std::string> PathPlanning::getAvailableAlgorithms() const
{
    return PathPlannerFactory::getAvailableAlgorithms();
}

// 保留原有的 boundingJudge 函数（为了兼容性）
bool PathPlanning::boundingJudge(int a, int b)
{
    // 这个函数现在主要用于兼容性，实际边界判断已移到接口中
    if (a < 0 || a >= m_cellMat.rows || b < 0 || b >= m_cellMat.cols) {
        return false;
    }
    
    int num = 0;
    for (int i = -1; i <= 1; i++) {
        for (int m = -1; m <= 1; m++) {
            if (i == 0 && m == 0) continue;
            int new_r = a + i;
            int new_c = b + m;
            if (new_r >= 0 && new_r < m_cellMat.rows && new_c >= 0 && new_c < m_cellMat.cols) {
                if (m_cellMat.at<uchar>(new_r, new_c) != 0) { // 不是自由空间
                    num++;
                }
            }
        }
    }
    return num != 0;
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

