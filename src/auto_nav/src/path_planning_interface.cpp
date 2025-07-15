#include "path_planning_interface.h"
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <ros/ros.h>

// ===== 神经网络路径规划算法实现 =====
NeuralNetworkPlanner::NeuralNetworkPlanner() {}

bool NeuralNetworkPlanner::initialize(const Mat& map, const CellIndex& start_point)
{
    cell_map_ = map.clone();
    start_point_ = start_point;
    map_rows_ = map.rows;
    map_cols_ = map.cols;
    
    // 初始化自由空间
    free_space_vec_.clear();
    for (int r = 0; r < map_rows_; r++) {
        for (int c = 0; c < map_cols_; c++) {
            if (isFreeSpace(r, c)) {
                free_space_vec_.push_back(CellIndex(r, c, 0));
            }
        }
    }
    
    initNeuralMat();
    return true;
}

void NeuralNetworkPlanner::initNeuralMat()
{
    neural_mat_ = Mat(map_rows_, map_cols_, CV_32F);
    for (int i = 0; i < map_rows_; i++) {
        for (int j = 0; j < map_cols_; j++) {
            if (cell_map_.at<uchar>(i, j) == 254) { // 障碍物
                neural_mat_.at<float>(i, j) = -100000.0;
            } else {
                neural_mat_.at<float>(i, j) = 50.0;
            }
        }
    }
}

vector<CellIndex> NeuralNetworkPlanner::planPath()
{
    vector<CellIndex> path;
    CellIndex current_point = start_point_;
    path.push_back(current_point);
    
    // 这里是原有神经网络算法的简化版本
    // 实际实现应该包含完整的神经网络逻辑
    const float c_0 = 50;
    float e = 0.0, v = 0.0, v_1 = 0.0;
    vector<float> theta_vec = {0, 45, 90, 135, 180, 225, 270, 315};
    
    for (int loop = 0; loop < 1000 && !free_space_vec_.empty(); loop++) {
        // 神经网络计算逻辑（简化）
        vector<float> candidate_values(8, -1000000.0);
        
        for (int dir = 0; dir < 8; dir++) {
            CellIndex next_point = current_point;
            
            // 根据方向计算下一个点
            switch (dir) {
                case 0: next_point.row -= 1; break;                    // 北
                case 1: next_point.row -= 1; next_point.col += 1; break; // 东北
                case 2: next_point.col += 1; break;                    // 东
                case 3: next_point.row += 1; next_point.col += 1; break; // 东南
                case 4: next_point.row += 1; break;                    // 南
                case 5: next_point.row += 1; next_point.col -= 1; break; // 西南
                case 6: next_point.col -= 1; break;                    // 西
                case 7: next_point.row -= 1; next_point.col -= 1; break; // 西北
            }
            
            if (isValidCell(next_point.row, next_point.col) && 
                isFreeSpace(next_point.row, next_point.col)) {
                candidate_values[dir] = neural_mat_.at<float>(next_point.row, next_point.col);
            }
        }
        
        // 选择最大值方向
        int max_index = 0;
        for (int i = 1; i < 8; i++) {
            if (candidate_values[i] > candidate_values[max_index]) {
                max_index = i;
            }
        }
        
        // 移动到下一个点
        switch (max_index) {
            case 0: current_point.row -= 1; break;
            case 1: current_point.row -= 1; current_point.col += 1; break;
            case 2: current_point.col += 1; break;
            case 3: current_point.row += 1; current_point.col += 1; break;
            case 4: current_point.row += 1; break;
            case 5: current_point.row += 1; current_point.col -= 1; break;
            case 6: current_point.col -= 1; break;
            case 7: current_point.row -= 1; current_point.col -= 1; break;
        }
        
        current_point.theta = theta_vec[max_index];
        path.push_back(current_point);
        
        // 更新神经网络值
        neural_mat_.at<float>(current_point.row, current_point.col) -= 10.0;
        
        // 检查是否完成覆盖
        if (loop % 100 == 0) {
            int uncovered_count = 0;
            for (const auto& free_cell : free_space_vec_) {
                if (neural_mat_.at<float>(free_cell.row, free_cell.col) > 40.0) {
                    uncovered_count++;
                }
            }
            if (uncovered_count < free_space_vec_.size() * 0.05) { // 95%覆盖率
                break;
            }
        }
    }
    
    return path;
}

bool NeuralNetworkPlanner::boundingJudge(int a, int b)
{
    int num = 0;
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            if (isValidCell(a + i, b + j) && !isFreeSpace(a + i, b + j)) {
                num++;
            }
        }
    }
    return num != 0;
}

void NeuralNetworkPlanner::setParameters(const std::map<std::string, double>& params)
{
    // 设置神经网络相关参数
    auto it = params.find("c_0");
    if (it != params.end()) {
        // 设置常数参数
    }
}

// A*路径规划算法实现
AStarPlanner::AStarPlanner() : heuristic_weight_(1.0) {}

// 为A*算法定义比较函数对象
struct NodeComparator {
    bool operator()(const AStarPlanner::Node* a, const AStarPlanner::Node* b) const {
        return a->f_cost > b->f_cost;
    }
};

bool AStarPlanner::initialize(const Mat& map, const CellIndex& start_point)
{
    cell_map_ = map.clone();
    start_point_ = start_point;
    map_rows_ = map.rows;
    map_cols_ = map.cols;
    
    // 生成覆盖目标点
    generateCoverageTargets();
    return true;
}

void AStarPlanner::generateCoverageTargets()
{
    target_points_.clear();
    // 生成网格覆盖点
    for (int r = 1; r < map_rows_; r += 2) {
        for (int c = 1; c < map_cols_; c += 2) {
            if (isFreeSpace(r, c)) {
                target_points_.push_back(CellIndex(r, c));
            }
        }
    }
}

vector<CellIndex> AStarPlanner::planPath()
{
    vector<CellIndex> complete_path;
    CellIndex current_start = start_point_;
    
    // 对每个目标点使用A*算法规划路径
    for (const auto& target : target_points_) {
        std::priority_queue<Node*, std::vector<Node*>, NodeComparator> open_set;
        std::unordered_set<int> closed_set;
        
        Node* start_node = new Node(current_start);
        open_set.push(start_node);
        
        Node* goal_node = nullptr;
        
        while (!open_set.empty()) {
            Node* current = open_set.top();
            open_set.pop();
            
            int current_id = current->cell.row * map_cols_ + current->cell.col;
            if (closed_set.find(current_id) != closed_set.end()) {
                delete current;
                continue;
            }
            closed_set.insert(current_id);
            
            // 检查是否到达目标
            if (current->cell.row == target.row && current->cell.col == target.col) {
                goal_node = current;
                break;
            }
            
            // 扩展邻居节点
            auto neighbors = getNeighbors(current->cell);
            for (const auto& neighbor : neighbors) {
                int neighbor_id = neighbor.row * map_cols_ + neighbor.col;
                if (closed_set.find(neighbor_id) != closed_set.end()) continue;
                
                Node* neighbor_node = new Node(neighbor);
                neighbor_node->parent = current;
                neighbor_node->g_cost = current->g_cost + 1.0;
                neighbor_node->h_cost = calculateHeuristic(neighbor, target);
                neighbor_node->f_cost = neighbor_node->g_cost + heuristic_weight_ * neighbor_node->h_cost;
                
                open_set.push(neighbor_node);
            }
        }
        
        if (goal_node) {
            auto segment_path = reconstructPath(goal_node);
            complete_path.insert(complete_path.end(), segment_path.begin(), segment_path.end());
            current_start = target;
        }
        
        // 清理内存
        while (!open_set.empty()) {
            delete open_set.top();
            open_set.pop();
        }
    }
    
    return complete_path;
}

double AStarPlanner::calculateHeuristic(const CellIndex& current, const CellIndex& goal)
{
    // 使用曼哈顿距离作为启发式函数
    return abs(current.row - goal.row) + abs(current.col - goal.col);
}

vector<CellIndex> AStarPlanner::getNeighbors(const CellIndex& current)
{
    vector<CellIndex> neighbors;
    vector<pair<int, int>> directions = {{-1,0}, {1,0}, {0,-1}, {0,1}, 
                                        {-1,-1}, {-1,1}, {1,-1}, {1,1}};
    
    for (const auto& dir : directions) {
        int new_row = current.row + dir.first;
        int new_col = current.col + dir.second;
        
        if (isValidCell(new_row, new_col) && isFreeSpace(new_row, new_col)) {
            neighbors.push_back(CellIndex(new_row, new_col));
        }
    }
    
    return neighbors;
}

vector<CellIndex> AStarPlanner::reconstructPath(Node* goal_node)
{
    vector<CellIndex> path;
    Node* current = goal_node;
    
    while (current != nullptr) {
        path.push_back(current->cell);
        current = current->parent;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

void AStarPlanner::setParameters(const std::map<std::string, double>& params)
{
    auto it = params.find("heuristic_weight");
    if (it != params.end()) {
        heuristic_weight_ = it->second;
    }
}

// ===== D*路径规划算法实现 =====
DStarPlanner::DStarPlanner() {}

bool DStarPlanner::initialize(const Mat& map, const CellIndex& start_point)
{
    cell_map_ = map.clone();
    start_point_ = start_point;
    map_rows_ = map.rows;
    map_cols_ = map.cols;
    
    // 初始化代价地图
    cost_map_.resize(map_rows_, vector<double>(map_cols_, std::numeric_limits<double>::infinity()));
    
    return true;
}

vector<CellIndex> DStarPlanner::planPath()
{
    vector<CellIndex> path;
    // D*算法的简化实现
    // 实际应该包含完整的D*算法逻辑
    
    // 这里提供一个基本的覆盖路径作为示例
    for (int r = 0; r < map_rows_; r++) {
        bool left_to_right = (r % 2 == 0);
        if (left_to_right) {
            for (int c = 0; c < map_cols_; c++) {
                if (isFreeSpace(r, c)) {
                    path.push_back(CellIndex(r, c));
                }
            }
        } else {
            for (int c = map_cols_ - 1; c >= 0; c--) {
                if (isFreeSpace(r, c)) {
                    path.push_back(CellIndex(r, c));
                }
            }
        }
    }
    
    return path;
}

void DStarPlanner::setParameters(const std::map<std::string, double>& params)
{
    // 设置D*算法参数
}

// ===== MCP算法实现 =====
MCPPlanner::MCPPlanner() : coverage_pattern_(0) {}

bool MCPPlanner::initialize(const Mat& map, const CellIndex& start_point)
{
    cell_map_ = map.clone();
    start_point_ = start_point;
    map_rows_ = map.rows;
    map_cols_ = map.cols;
    return true;
}

vector<CellIndex> MCPPlanner::planPath()
{
    vector<CellIndex> path;
    
    switch (coverage_pattern_) {
        case 0:
            generateSnakePattern(path);
            break;
        case 1:
            generateSpiralPattern(path);
            break;
        case 2:
            generateZonePattern(path);
            break;
        default:
            generateSnakePattern(path);
    }
    
    return path;
}

void MCPPlanner::generateSnakePattern(vector<CellIndex>& path)
{
    // 蛇形覆盖模式
    for (int r = 0; r < map_rows_; r++) {
        bool left_to_right = (r % 2 == 0);
        if (left_to_right) {
            for (int c = 0; c < map_cols_; c++) {
                if (isFreeSpace(r, c)) {
                    path.push_back(CellIndex(r, c));
                }
            }
        } else {
            for (int c = map_cols_ - 1; c >= 0; c--) {
                if (isFreeSpace(r, c)) {
                    path.push_back(CellIndex(r, c));
                }
            }
        }
    }
}

void MCPPlanner::generateSpiralPattern(vector<CellIndex>& path)
{
    // 螺旋覆盖模式
    int top = 0, bottom = map_rows_ - 1;
    int left = 0, right = map_cols_ - 1;
    
    while (top <= bottom && left <= right) {
        // 从左到右
        for (int c = left; c <= right; c++) {
            if (isFreeSpace(top, c)) {
                path.push_back(CellIndex(top, c));
            }
        }
        top++;
        
        // 从上到下
        for (int r = top; r <= bottom; r++) {
            if (isFreeSpace(r, right)) {
                path.push_back(CellIndex(r, right));
            }
        }
        right--;
        
        // 从右到左
        if (top <= bottom) {
            for (int c = right; c >= left; c--) {
                if (isFreeSpace(bottom, c)) {
                    path.push_back(CellIndex(bottom, c));
                }
            }
            bottom--;
        }
        
        // 从下到上
        if (left <= right) {
            for (int r = bottom; r >= top; r--) {
                if (isFreeSpace(r, left)) {
                    path.push_back(CellIndex(r, left));
                }
            }
            left++;
        }
    }
}

void MCPPlanner::generateZonePattern(vector<CellIndex>& path)
{
    // 分区覆盖模式
    int zone_size = 10; // 每个区域的大小
    
    for (int zone_r = 0; zone_r < map_rows_; zone_r += zone_size) {
        for (int zone_c = 0; zone_c < map_cols_; zone_c += zone_size) {
            // 在每个区域内使用蛇形模式
            int end_r = std::min(zone_r + zone_size, map_rows_);
            int end_c = std::min(zone_c + zone_size, map_cols_);
            
            for (int r = zone_r; r < end_r; r++) {
                bool left_to_right = ((r - zone_r) % 2 == 0);
                if (left_to_right) {
                    for (int c = zone_c; c < end_c; c++) {
                        if (isFreeSpace(r, c)) {
                            path.push_back(CellIndex(r, c));
                        }
                    }
                } else {
                    for (int c = end_c - 1; c >= zone_c; c--) {
                        if (isFreeSpace(r, c)) {
                            path.push_back(CellIndex(r, c));
                        }
                    }
                }
            }
        }
    }
}

void MCPPlanner::setParameters(const std::map<std::string, double>& params)
{
    auto it = params.find("coverage_pattern");
    if (it != params.end()) {
        coverage_pattern_ = static_cast<int>(it->second);
    }
}

// ===== 工厂类实现 =====
std::shared_ptr<PathPlannerInterface> PathPlannerFactory::createPlanner(PlannerType type)
{
    switch (type) {
        case PlannerType::NEURAL_NETWORK:
            return std::make_shared<NeuralNetworkPlanner>();
        case PlannerType::ASTAR:
            return std::make_shared<AStarPlanner>();
        case PlannerType::DSTAR:
            return std::make_shared<DStarPlanner>();
        case PlannerType::MCP:
            return std::make_shared<MCPPlanner>();
        default:
            ROS_ERROR("Unknown planner type");
            return nullptr;
    }
}

PlannerType PathPlannerFactory::stringToType(const std::string& type_str)
{
    if (type_str == "neural_network" || type_str == "neural") return PlannerType::NEURAL_NETWORK;
    else if (type_str == "astar" || type_str == "a_star") return PlannerType::ASTAR;
    else if (type_str == "dstar" || type_str == "d_star") return PlannerType::DSTAR;
    else if (type_str == "mcp") return PlannerType::MCP;
    else return PlannerType::NEURAL_NETWORK; // 默认
}

std::string PathPlannerFactory::typeToString(PlannerType type)
{
    switch (type) {
        case PlannerType::NEURAL_NETWORK: return "neural_network";
        case PlannerType::ASTAR: return "astar";
        case PlannerType::DSTAR: return "dstar";
        case PlannerType::MCP: return "mcp";
        default: return "neural_network";
    }
}

std::vector<std::string> PathPlannerFactory::getAvailableAlgorithms()
{
    return {"neural_network", "astar", "dstar", "mcp"};
}
