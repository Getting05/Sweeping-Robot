#include "astar_algorithm.h"
#include "path_planning_algorithm.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

using namespace std;

// 辅助函数实现
string cellToString(const CellIndex& cell) {
    return to_string(cell.row) + "," + to_string(cell.col);
}

double calculateDistance(const CellIndex& a, const CellIndex& b) {
    return sqrt(pow(a.row - b.row, 2) + pow(a.col - b.col, 2));
}

AStarAlgorithm::AStarAlgorithm() 
    : heuristic_weight_(1.0)
    , move_cost_straight_(1.0)
    , move_cost_diagonal_(1.414)  // sqrt(2)
    , max_iterations_(10000)
    , coverage_strategy_("nearest")
{
}

bool AStarAlgorithm::initialize(costmap_2d::Costmap2D* costmap, int cell_size) {
    costmap_ = costmap;
    cell_size_ = cell_size;
    initialized_ = (costmap_ != nullptr && cell_size_ > 0);
    
    if (initialized_) {
        cout << "[" << getAlgorithmName() << "] Initialized with cell_size: " << cell_size_ << endl;
    }
    
    return initialized_;
}

vector<CellIndex> AStarAlgorithm::planPath(int start_row, int start_col, 
                                          const vector<CellIndex>& free_space_vec,
                                          const Mat& cell_mat) {
    if (!initialized_) {
        cout << "[" << getAlgorithmName() << "] Error: Algorithm not initialized!" << endl;
        return vector<CellIndex>();
    }

    CellIndex start(start_row, start_col, 0);
    cout << "[" << getAlgorithmName() << "] Planning coverage path from (" 
         << start_row << ", " << start_col << ")" << endl;
    cout << "[" << getAlgorithmName() << "] Free space size: " << free_space_vec.size() << endl;

    return planCoveragePath(start, free_space_vec, cell_mat);
}

vector<CellIndex> AStarAlgorithm::planCoveragePath(const CellIndex& start,
                                                  const vector<CellIndex>& free_space,
                                                  const Mat& cell_mat) {
    vector<CellIndex> full_path;
    vector<CellIndex> remaining_targets = free_space;
    CellIndex current_pos = start;
    
    // 确保起始点在路径中
    full_path.push_back(current_pos);
    visited_cells_.insert(cellToString(current_pos));
    
    int iteration = 0;
    while (!remaining_targets.empty() && iteration < max_iterations_) {
        iteration++;
        
        // 选择下一个目标点
        CellIndex next_target = selectNextTarget(remaining_targets, full_path);
        
        if (next_target.row == -1) {
            cout << "[" << getAlgorithmName() << "] No more reachable targets." << endl;
            break;
        }
        
        // 使用A*算法规划到下一个目标的路径
        vector<CellIndex> segment_path = findPathAStar(current_pos, next_target, cell_mat, free_space);
        
        if (segment_path.empty()) {
            cout << "[" << getAlgorithmName() << "] Failed to find path to target (" 
                 << next_target.row << ", " << next_target.col << ")" << endl;
            
            // 移除无法到达的目标
            remaining_targets.erase(
                remove_if(remaining_targets.begin(), remaining_targets.end(),
                         [&next_target](const CellIndex& cell) {
                             return cell.row == next_target.row && cell.col == next_target.col;
                         }),
                remaining_targets.end());
            continue;
        }
        
        // 将路径段添加到完整路径中（跳过第一个点以避免重复）
        for (size_t i = 1; i < segment_path.size(); i++) {
            full_path.push_back(segment_path[i]);
            visited_cells_.insert(cellToString(segment_path[i]));
        }
        
        current_pos = next_target;
        
        // 从剩余目标中移除已访问的点
        remaining_targets.erase(
            remove_if(remaining_targets.begin(), remaining_targets.end(),
                     [this](const CellIndex& cell) {
                         return visited_cells_.count(cellToString(cell)) > 0;
                     }),
            remaining_targets.end());
        
        if (iteration % 100 == 0) {
            cout << "[" << getAlgorithmName() << "] Iteration " << iteration 
                 << ", remaining targets: " << remaining_targets.size() << endl;
        }
    }
    
    cout << "[" << getAlgorithmName() << "] Coverage planning completed. Total path points: " 
         << full_path.size() << ", visited targets: " << visited_cells_.size() << endl;
    
    return full_path;
}

vector<CellIndex> AStarAlgorithm::findPathAStar(const CellIndex& start, const CellIndex& goal,
                                               const Mat& cell_mat, const vector<CellIndex>& free_space) {
    priority_queue<shared_ptr<Node>, vector<shared_ptr<Node>>, NodeComparator> open_list;
    unordered_map<string, shared_ptr<Node>> open_map;
    unordered_set<string> closed_set;
    
    // 创建起始节点
    auto start_node = make_shared<Node>(start, 0, calculateHeuristic(start, goal));
    open_list.push(start_node);
    open_map[cellToString(start)] = start_node;
    
    int iterations = 0;
    while (!open_list.empty() && iterations < max_iterations_) {
        iterations++;
        
        // 获取f值最小的节点
        auto current = open_list.top();
        open_list.pop();
        
        string current_key = cellToString(current->cell);
        open_map.erase(current_key);
        closed_set.insert(current_key);
        
        // 检查是否到达目标
        if (current->cell.row == goal.row && current->cell.col == goal.col) {
            return reconstructPath(current);
        }
        
        // 探索相邻节点
        vector<CellIndex> neighbors = getNeighbors(current->cell, cell_mat);
        
        for (const auto& neighbor : neighbors) {
            string neighbor_key = cellToString(neighbor);
            
            // 跳过已在关闭列表中的节点
            if (closed_set.count(neighbor_key)) {
                continue;
            }
            
            // 跳过不在自由空间中的节点
            if (!isInFreeSpace(neighbor, free_space)) {
                continue;
            }
            
            double tentative_g = current->g_cost + calculateMoveCost(current->cell, neighbor);
            
            auto existing_node = open_map.find(neighbor_key);
            if (existing_node != open_map.end()) {
                // 节点已在开放列表中，检查是否找到更好的路径
                if (tentative_g < existing_node->second->g_cost) {
                    existing_node->second->g_cost = tentative_g;
                    existing_node->second->f_cost = tentative_g + existing_node->second->h_cost;
                    existing_node->second->parent = current;
                }
            } else {
                // 新节点，添加到开放列表
                double h_cost = heuristic_weight_ * calculateHeuristic(neighbor, goal);
                auto new_node = make_shared<Node>(neighbor, tentative_g, h_cost);
                new_node->parent = current;
                
                open_list.push(new_node);
                open_map[neighbor_key] = new_node;
            }
        }
    }
    
    // 未找到路径
    return vector<CellIndex>();
}

double AStarAlgorithm::calculateHeuristic(const CellIndex& current, const CellIndex& goal) {
    // 使用曼哈顿距离作为启发式函数
    return abs(current.row - goal.row) + abs(current.col - goal.col);
}

vector<CellIndex> AStarAlgorithm::getNeighbors(const CellIndex& current, const Mat& cell_mat) {
    vector<CellIndex> neighbors;
    
    // 8方向邻居（包括对角线）
    vector<pair<int, int>> directions = {
        {-1, -1}, {-1, 0}, {-1, 1},
        { 0, -1},          { 0, 1},
        { 1, -1}, { 1, 0}, { 1, 1}
    };
    
    for (const auto& dir : directions) {
        int new_row = current.row + dir.first;
        int new_col = current.col + dir.second;
        
        // 检查边界
        if (new_row >= 0 && new_row < cell_mat.rows && 
            new_col >= 0 && new_col < cell_mat.cols) {
            
            // 计算角度
            double theta = atan2(dir.first, dir.second) * 180.0 / M_PI;
            if (theta < 0) theta += 360.0;
            
            neighbors.emplace_back(new_row, new_col, theta);
        }
    }
    
    return neighbors;
}

bool AStarAlgorithm::isInFreeSpace(const CellIndex& cell, const vector<CellIndex>& free_space) {
    return find_if(free_space.begin(), free_space.end(),
                   [&cell](const CellIndex& free_cell) {
                       return free_cell.row == cell.row && free_cell.col == cell.col;
                   }) != free_space.end();
}

double AStarAlgorithm::calculateMoveCost(const CellIndex& from, const CellIndex& to) {
    int dr = abs(to.row - from.row);
    int dc = abs(to.col - from.col);
    
    if (dr == 1 && dc == 1) {
        return move_cost_diagonal_;  // 对角线移动
    } else if ((dr == 1 && dc == 0) || (dr == 0 && dc == 1)) {
        return move_cost_straight_;  // 直线移动
    } else {
        return 999999.0;  // 不应该发生的情况
    }
}

CellIndex AStarAlgorithm::selectNextTarget(const vector<CellIndex>& free_space, 
                                          const vector<CellIndex>& visited) {
    if (free_space.empty()) {
        return CellIndex(-1, -1, 0);  // 无效的目标
    }
    
    CellIndex current_pos = visited.back();
    CellIndex best_target(-1, -1, 0);
    double best_score = -1;
    
    for (const auto& candidate : free_space) {
        // 跳过已访问的点
        if (visited_cells_.count(cellToString(candidate))) {
            continue;
        }
        
        double distance = calculateDistance(current_pos, candidate);
        double score;
        
        if (coverage_strategy_ == "nearest") {
            score = 1.0 / (distance + 1e-6);  // 距离越近分数越高
        } else if (coverage_strategy_ == "farthest") {
            score = distance;  // 距离越远分数越高
        } else {  // spiral strategy
            // 螺旋策略：优先选择接近但未完全相邻的点
            score = 1.0 / (abs(distance - 2.0) + 1e-6);
        }
        
        if (best_score < 0 || score > best_score) {
            best_score = score;
            best_target = candidate;
        }
    }
    
    return best_target;
}

vector<CellIndex> AStarAlgorithm::reconstructPath(shared_ptr<Node> goal_node) {
    vector<CellIndex> path;
    auto current = goal_node;
    
    while (current != nullptr) {
        path.push_back(current->cell);
        current = current->parent;
    }
    
    reverse(path.begin(), path.end());
    return path;
}

string AStarAlgorithm::getAlgorithmName() const {
    return "A* Algorithm";
}

string AStarAlgorithm::getParameterDescription() const {
    stringstream ss;
    ss << "A* Algorithm Parameters:\n";
    ss << "- heuristic_weight: " << heuristic_weight_ << "\n";
    ss << "- move_cost_straight: " << move_cost_straight_ << "\n";
    ss << "- move_cost_diagonal: " << move_cost_diagonal_ << "\n";
    ss << "- max_iterations: " << max_iterations_ << "\n";
    ss << "- coverage_strategy: " << coverage_strategy_;
    return ss.str();
}

void AStarAlgorithm::setParameter(const string& param_name, double param_value) {
    if (param_name == "heuristic_weight") {
        heuristic_weight_ = param_value;
    } else if (param_name == "move_cost_straight") {
        move_cost_straight_ = param_value;
    } else if (param_name == "move_cost_diagonal") {
        move_cost_diagonal_ = param_value;
    } else if (param_name == "max_iterations") {
        max_iterations_ = static_cast<int>(param_value);
    } else {
        cout << "[" << getAlgorithmName() << "] Warning: Unknown parameter: " << param_name << endl;
    }
}

void AStarAlgorithm::reset() {
    visited_cells_.clear();
    current_target_ = CellIndex(-1, -1, 0);
}

// 注册A*算法到工厂
class AStarAlgorithmRegistrar {
public:
    AStarAlgorithmRegistrar() {
        PathPlanningAlgorithmFactory::registerAlgorithm("astar", []() {
            return make_shared<AStarAlgorithm>();
        });
    }
};

static AStarAlgorithmRegistrar astar_registrar;
