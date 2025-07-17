#include "path_planning.h"
#include <costmap_2d/cost_values.h>
#include <float.h>
#include <climits>


PathPlanning::PathPlanning(costmap_2d::Costmap2DROS *costmap2dRos)
{
    m_cosmap2dRos = costmap2dRos;
    m_costmap2d = costmap2dRos->getCostmap();

    ros::NodeHandle private_nh;
    m_planPub = private_nh.advertise<nav_msgs::Path>("plan_path", 1);               // 用于在rviz中画出规划路径
    m_gridPub = private_nh.advertise<nav_msgs::OccupancyGrid>("covered_grid", 1);   // 用于在rviz中画出已清扫区域

    string sizeOfCellString, coveredValueStr;
    m_cellSize = 3;  // 改回默认值3，避免cell太大
    if (private_nh.searchParam("size_of_cell", sizeOfCellString))   // 搜索参数,根据名称"size of cell"搜索参数，将对应名称下的参数值赋给sizeOfCellString.
        private_nh.param("size_of_cell", m_cellSize, 3);            // 设置机器人占据n*n的栅格，决定规划的稀疏   

    m_gridCoveredValue = 0;
    if (private_nh.searchParam("grid_covered_value", coveredValueStr))
        private_nh.param("grid_covered_value", m_gridCoveredValue, 0);

    // 设置覆盖间距 - 改为更密集的覆盖
    m_coverageSpacing = m_cellSize * 0.8;  // 覆盖间距为cell大小的0.8倍，更密集

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

    if (!m_srcMap.empty())
        m_initialized = true;
    else
        m_initialized = false;
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
                if (!isFree) break;
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

    // 初始化访问标记矩阵
    m_visited.clear();
    m_visited.resize(m_cellMat.rows, vector<bool>(m_cellMat.cols, false));

    // 以下代码为初始化m_neuralMat (保留用于调试)
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
    
    ROS_INFO("Map initialized: %d x %d cells, %lu free spaces", 
             m_cellMat.rows, m_cellMat.cols, m_freeSpaceVec.size());
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
    ROS_INFO("Starting A* based coverage path planning...");
    
    // 获取机器人当前位置
    bool isok = m_cosmap2dRos->getRobotPose(m_initPose);
    if (!isok) {
        ROS_ERROR("Failed to get robot pose!");
        return;
    }

    unsigned int mx, my;
    double wx = m_initPose.pose.position.x;
    double wy = m_initPose.pose.position.y;
    bool getmapcoor = m_costmap2d->worldToMap(wx, wy, mx, my);
    if (!getmapcoor) {
        ROS_ERROR("Failed to convert world coordinates to map coordinates!");
        return;
    }

    // 转换为cell坐标
    int start_row = m_cellMat.rows - my / m_cellSize - 1;
    int start_col = mx / m_cellSize;
    
    // 边界检查
    if (start_row < 0 || start_row >= m_cellMat.rows || 
        start_col < 0 || start_col >= m_cellMat.cols) {
        ROS_ERROR("Starting position is out of bounds!");
        return;
    }

    ROS_INFO("Starting position: (%d, %d) in cell coordinates", start_col, start_row);

    // 生成全覆盖路径
    m_pathVec = generateCoveragePath();
    
    // 确保起始点在路径开头
    if (!m_pathVec.empty() && (m_pathVec[0].row != start_row || m_pathVec[0].col != start_col)) {
        CellIndex startPoint;
        startPoint.row = start_row;
        startPoint.col = start_col;
        startPoint.theta = 0;
        m_pathVec.insert(m_pathVec.begin(), startPoint);
    }

    ROS_INFO("Generated coverage path with %lu points", m_pathVec.size());
}


bool PathPlanning::boundingJudge(int a, int b)
{
    int num = 0;
    for (int i = -1; i <= 1; i++)
    {
        for (int m = -1; m <= 1; m++)
        {
            if (i == 0 && m == 0)
            {
                continue;
            }
            if (m_neuralMat.at<float>((a + i), (b + m)) == -250.0)
                num++;
        }
    }
    if (num != 0)
        return true;
    else
        return false;
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

// A*算法主要实现函数

vector<CellIndex> PathPlanning::generateCoveragePath()
{
    ROS_INFO("Generating coverage targets...");
    
    // 生成覆盖目标点
    m_targets = generateCoverageTargets();
    ROS_INFO("Generated %lu coverage targets", m_targets.size());

    vector<CellIndex> fullPath;
    
    // 获取起始位置
    unsigned int mx, my;
    double wx = m_initPose.pose.position.x;
    double wy = m_initPose.pose.position.y;
    m_costmap2d->worldToMap(wx, wy, mx, my);
    int current_x = mx / m_cellSize;
    int current_y = m_cellMat.rows - my / m_cellSize - 1;

    CellIndex startPoint;
    startPoint.row = current_y;
    startPoint.col = current_x;
    startPoint.theta = 0;
    fullPath.push_back(startPoint);

    int visited_targets = 0;
    int max_targets = m_targets.size();
    
    // 使用贪心策略访问所有目标点
    while (visited_targets < max_targets && visited_targets < 1000) { // 限制最大目标数量防止计算过久
        // 寻找最近的未访问目标
        CellIndex nearestTarget = findNearestUnvisitedTarget(current_x, current_y, m_targets);
        
        if (nearestTarget.col == -1) {
            ROS_WARN("No more unvisited targets found");
            break;
        }

        // 使用A*算法规划到目标的路径
        vector<AStarNode*> pathToTarget = aStarPathPlanning(current_x, current_y, 
                                                           nearestTarget.col, nearestTarget.row);
        
        if (pathToTarget.empty()) {
            ROS_WARN("Failed to find path to target (%d, %d)", nearestTarget.col, nearestTarget.row);
            // 标记目标为已访问，避免无限循环
            for (auto& target : m_targets) {
                if (target.x == nearestTarget.col && target.y == nearestTarget.row) {
                    target.covered = true;
                    break;
                }
            }
            visited_targets++;
            continue;
        }

        // 将路径转换为CellIndex并添加到完整路径中
        for (size_t i = 1; i < pathToTarget.size(); i++) { // 跳过起始点，避免重复
            CellIndex pathPoint;
            pathPoint.col = pathToTarget[i]->x;
            pathPoint.row = pathToTarget[i]->y;
            
            // 计算方向
            if (i < pathToTarget.size() - 1) {
                int dx = pathToTarget[i+1]->x - pathToTarget[i]->x;
                int dy = pathToTarget[i+1]->y - pathToTarget[i]->y;
                pathPoint.theta = atan2(dy, dx) * 180.0 / PI;
            } else {
                pathPoint.theta = fullPath.back().theta; // 保持最后的方向
            }
            
            fullPath.push_back(pathPoint);
        }

        // 清理内存
        for (auto node : pathToTarget) {
            delete node;
        }

        // 更新当前位置
        current_x = nearestTarget.col;
        current_y = nearestTarget.row;
        visited_targets++;
        
        ROS_INFO("Reached target %d/%d at (%d, %d), path length: %lu", 
                 visited_targets, max_targets, current_x, current_y, fullPath.size());
    }

    // 路径优化
    optimizePath(fullPath);
    
    ROS_INFO("Coverage path generation completed. Total path points: %lu", fullPath.size());
    return fullPath;
}

vector<CoverageTarget> PathPlanning::generateCoverageTargets()
{
    vector<CoverageTarget> targets;
    
    // 使用网格模式生成覆盖目标点 - 改进间距计算
    int spacing = max(1, (int)(m_coverageSpacing)); // 直接使用覆盖间距，不再除以cellSize
    
    ROS_INFO("Generating coverage targets with spacing: %d, map size: %d x %d", 
             spacing, m_cellMat.rows, m_cellMat.cols);
    
    for (int row = 0; row < m_cellMat.rows; row += spacing) {
        for (int col = 0; col < m_cellMat.cols; col += spacing) {
            if (isValidCell(col, row)) {
                CoverageTarget target(col, row);
                // 计算优先级（距离边界的距离，边界附近优先级高）
                int dist_to_edge = min({row, col, m_cellMat.rows - row - 1, m_cellMat.cols - col - 1});
                target.priority = 1.0 / (dist_to_edge + 1); // 离边界越近优先级越高
                targets.push_back(target);
            }
        }
    }
    
    ROS_INFO("Generated %lu coverage targets before sorting", targets.size());
    
    // 按优先级排序（高优先级在前）
    sort(targets.begin(), targets.end(), 
         [](const CoverageTarget& a, const CoverageTarget& b) {
             return a.priority > b.priority;
         });
    
    ROS_INFO("Coverage targets sorted, first 5 targets:");
    for (int i = 0; i < min(5, (int)targets.size()); i++) {
        ROS_INFO("  Target %d: (%d, %d) priority: %.3f", 
                 i, targets[i].x, targets[i].y, targets[i].priority);
    }
    
    return targets;
}

vector<AStarNode*> PathPlanning::aStarPathPlanning(int start_x, int start_y, int goal_x, int goal_y)
{
    // 边界检查
    if (!isValidCell(start_x, start_y) || !isValidCell(goal_x, goal_y)) {
        return vector<AStarNode*>();
    }

    // 创建节点矩阵
    vector<vector<AStarNode*>> nodes(m_cellMat.rows, vector<AStarNode*>(m_cellMat.cols, nullptr));
    
    // 初始化所有可访问的节点
    for (int y = 0; y < m_cellMat.rows; y++) {
        for (int x = 0; x < m_cellMat.cols; x++) {
            if (isValidCell(x, y)) {
                nodes[y][x] = new AStarNode(x, y);
            }
        }
    }

    // 开放列表和关闭列表
    priority_queue<AStarNode*> openList;
    set<AStarNode*> closedSet;

    // 初始化起始节点
    AStarNode* startNode = nodes[start_y][start_x];
    if (!startNode) {
        // 清理内存
        for (int y = 0; y < m_cellMat.rows; y++) {
            for (int x = 0; x < m_cellMat.cols; x++) {
                delete nodes[y][x];
            }
        }
        return vector<AStarNode*>();
    }

    startNode->g_cost = 0;
    startNode->h_cost = calculateHeuristic(start_x, start_y, goal_x, goal_y);
    startNode->f_cost = startNode->g_cost + startNode->h_cost;
    openList.push(startNode);

    AStarNode* goalNode = nullptr;

    // A*主循环
    while (!openList.empty()) {
        // 获取f_cost最小的节点
        AStarNode* current = openList.top();
        openList.pop();

        // 检查是否已在关闭列表中
        if (closedSet.find(current) != closedSet.end()) {
            continue;
        }

        // 添加到关闭列表
        closedSet.insert(current);

        // 检查是否到达目标
        if (current->x == goal_x && current->y == goal_y) {
            goalNode = current;
            break;
        }

        // 检查所有邻居
        vector<pair<int, int>> neighbors = getNeighbors(current->x, current->y);
        
        for (auto& neighbor : neighbors) {
            int nx = neighbor.first;
            int ny = neighbor.second;

            if (!isValidCell(nx, ny) || !nodes[ny][nx]) {
                continue;
            }

            AStarNode* neighborNode = nodes[ny][nx];

            // 如果邻居在关闭列表中，跳过
            if (closedSet.find(neighborNode) != closedSet.end()) {
                continue;
            }

            // 计算新的g_cost
            double tentative_g = current->g_cost;
            
            // 添加移动代价
            if (abs(nx - current->x) + abs(ny - current->y) == 2) {
                tentative_g += DIAGONAL_COST; // 对角线移动
            } else {
                tentative_g += STRAIGHT_COST;  // 直线移动
            }

            // 如果这条路径更好，或者邻居不在开放列表中
            if (tentative_g < neighborNode->g_cost || neighborNode->g_cost == 0) {
                neighborNode->parent = current;
                neighborNode->g_cost = tentative_g;
                neighborNode->h_cost = calculateHeuristic(nx, ny, goal_x, goal_y);
                neighborNode->f_cost = neighborNode->g_cost + neighborNode->h_cost;
                
                openList.push(neighborNode);
            }
        }
    }

    // 重建路径
    vector<AStarNode*> path;
    if (goalNode) {
        AStarNode* current = goalNode;
        while (current != nullptr) {
            path.push_back(new AStarNode(current->x, current->y));
            current = current->parent;
        }
        reverse(path.begin(), path.end());
    }

    // 清理内存
    for (int y = 0; y < m_cellMat.rows; y++) {
        for (int x = 0; x < m_cellMat.cols; x++) {
            delete nodes[y][x];
        }
    }

    return path;
}

double PathPlanning::calculateHeuristic(int x1, int y1, int x2, int y2)
{
    // 使用欧几里得距离作为启发式函数
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    return sqrt(dx * dx + dy * dy);
}

bool PathPlanning::isValidCell(int x, int y)
{
    return x >= 0 && x < m_cellMat.cols && 
           y >= 0 && y < m_cellMat.rows && 
           m_cellMat.at<uchar>(y, x) == costmap_2d::FREE_SPACE;
}

vector<pair<int, int>> PathPlanning::getNeighbors(int x, int y)
{
    vector<pair<int, int>> neighbors;
    
    // 8连通邻域
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue; // 跳过自己
            
            int nx = x + dx;
            int ny = y + dy;
            
            if (isValidCell(nx, ny)) {
                neighbors.push_back(make_pair(nx, ny));
            }
        }
    }
    
    return neighbors;
}

CellIndex PathPlanning::findNearestUnvisitedTarget(int current_x, int current_y, vector<CoverageTarget>& targets)
{
    CellIndex nearest;
    nearest.col = -1; // 表示未找到
    nearest.row = -1;
    nearest.theta = 0;
    
    double minDistance = DBL_MAX;
    
    for (auto& target : targets) {
        if (!target.covered) {
            double distance = calculateHeuristic(current_x, current_y, target.x, target.y);
            if (distance < minDistance) {
                minDistance = distance;
                nearest.col = target.x;
                nearest.row = target.y;
            }
        }
    }
    
    // 标记找到的目标为已访问
    if (nearest.col != -1) {
        for (auto& target : targets) {
            if (target.x == nearest.col && target.y == nearest.row) {
                target.covered = true;
                break;
            }
        }
    }
    
    return nearest;
}

void PathPlanning::optimizePath(vector<CellIndex>& path)
{
    if (path.size() < 3) return;
    
    // 简单的路径平滑：移除冗余的中间点
    vector<CellIndex> optimizedPath;
    optimizedPath.push_back(path[0]);
    
    for (size_t i = 1; i < path.size() - 1; i++) {
        // 检查是否可以直接从前一个点到下一个点
        int prev_idx = optimizedPath.size() - 1;
        bool canSkip = true;
        
        // 简单检查：如果三点几乎共线，可以跳过中间点
        double dx1 = path[i].col - optimizedPath[prev_idx].col;
        double dy1 = path[i].row - optimizedPath[prev_idx].row;
        double dx2 = path[i+1].col - path[i].col;
        double dy2 = path[i+1].row - path[i].row;
        
        // 计算方向向量的夹角
        double dot = dx1 * dx2 + dy1 * dy2;
        double mag1 = sqrt(dx1 * dx1 + dy1 * dy1);
        double mag2 = sqrt(dx2 * dx2 + dy2 * dy2);
        
        if (mag1 > 0 && mag2 > 0) {
            double cos_angle = dot / (mag1 * mag2);
            if (cos_angle > 0.95) { // 角度小于约18度，认为共线
                canSkip = true;
            } else {
                canSkip = false;
            }
        }
        
        if (!canSkip) {
            optimizedPath.push_back(path[i]);
        }
    }
    
    optimizedPath.push_back(path.back());
    path = optimizedPath;
    
    ROS_INFO("Path optimized: %lu -> %lu points", path.size(), optimizedPath.size());
}

double PathPlanning::calculatePathLength(const vector<CellIndex>& path)
{
    double length = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
        double dx = path[i].col - path[i-1].col;
        double dy = path[i].row - path[i-1].row;
        length += sqrt(dx * dx + dy * dy);
    }
    return length;
}

