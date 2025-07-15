#include "neural_algorithm.h"
#include "path_planning_algorithm.h"
#include <iostream>
#include <cmath>
#include <sstream>

using namespace std;

NeuralAlgorithm::NeuralAlgorithm() 
    : c_0_(50.0)
    , max_iterations_(9000)
    , theta_vec_({0, 45, 90, 135, 180, 225, 270, 315})
{
}

bool NeuralAlgorithm::initialize(costmap_2d::Costmap2D* costmap, int cell_size) {
    costmap_ = costmap;
    cell_size_ = cell_size;
    initialized_ = (costmap_ != nullptr && cell_size_ > 0);
    
    if (initialized_) {
        cout << "[" << getAlgorithmName() << "] Initialized with cell_size: " << cell_size_ << endl;
    }
    
    return initialized_;
}

vector<CellIndex> NeuralAlgorithm::planPath(int start_row, int start_col, 
                                           const vector<CellIndex>& free_space_vec,
                                           const Mat& cell_mat) {
    if (!initialized_) {
        cout << "[" << getAlgorithmName() << "] Error: Algorithm not initialized!" << endl;
        return vector<CellIndex>();
    }

    CellIndex start_point(start_row, start_col, 0);
    cout << "[" << getAlgorithmName() << "] Planning neural network path from (" 
         << start_row << ", " << start_col << ")" << endl;
    cout << "[" << getAlgorithmName() << "] Free space size: " << free_space_vec.size() << endl;

    // 初始化神经网络矩阵
    initNeuralMat(cell_mat, free_space_vec);

    return getPathInCV(start_point, free_space_vec, cell_mat);
}

void NeuralAlgorithm::initNeuralMat(const Mat& cell_mat, const vector<CellIndex>& free_space_vec) {
    neural_mat_ = Mat(cell_mat.rows, cell_mat.cols, CV_32F);
    
    // 初始化所有点为障碍物权值
    for (int i = 0; i < neural_mat_.rows; i++) {
        for (int j = 0; j < neural_mat_.cols; j++) {
            neural_mat_.at<float>(i, j) = -250.0;  // 障碍物权值
        }
    }
    
    // 设置自由空间的权值
    for (const auto& free_cell : free_space_vec) {
        if (free_cell.row >= 0 && free_cell.row < neural_mat_.rows &&
            free_cell.col >= 0 && free_cell.col < neural_mat_.cols) {
            neural_mat_.at<float>(free_cell.row, free_cell.col) = 50.0;  // 自由空间权值
        }
    }
}

vector<CellIndex> NeuralAlgorithm::getPathInCV(const CellIndex& start_point, 
                                              const vector<CellIndex>& free_space_vec,
                                              const Mat& cell_mat) {
    vector<CellIndex> path_vec;
    CellIndex current_point = start_point;
    path_vec.push_back(current_point);
    
    float init_theta = start_point.theta;
    float e = 0.0, v = 0.0, v_1 = 0.0, delta_theta = 0.0;
    float last_theta = init_theta;
    const float PI = 3.14159;
    
    cout << "[" << getAlgorithmName() << "] Starting neural network iterations..." << endl;
    
    for (int loop = 0; loop < max_iterations_; loop++) {
        int max_index = 0;
        float max_v = -300.0;  // v阈值初始值
        
        neural_mat_.at<float>(current_point.row, current_point.col) = -250.0; // 当前点权值设为已访问
        last_theta = current_point.theta;
        
        // 遍历所有可能的方向角度
        for (int i = 0; i < theta_vec_.size(); i++) {
            float theta = theta_vec_[i];
            CellIndex next_point;
            
            // 根据角度计算下一个点的位置
            switch (i) {
                case 0: // 0度 - 向右
                    next_point.row = current_point.row;
                    next_point.col = current_point.col + 1;
                    break;
                case 1: // 45度 - 右上
                    next_point.row = current_point.row - 1;
                    next_point.col = current_point.col + 1;
                    break;
                case 2: // 90度 - 向上
                    next_point.row = current_point.row - 1;
                    next_point.col = current_point.col;
                    break;
                case 3: // 135度 - 左上
                    next_point.row = current_point.row - 1;
                    next_point.col = current_point.col - 1;
                    break;
                case 4: // 180度 - 向左
                    next_point.row = current_point.row;
                    next_point.col = current_point.col - 1;
                    break;
                case 5: // 225度 - 左下
                    next_point.row = current_point.row + 1;
                    next_point.col = current_point.col - 1;
                    break;
                case 6: // 270度 - 向下
                    next_point.row = current_point.row + 1;
                    next_point.col = current_point.col;
                    break;
                case 7: // 315度 - 右下
                    next_point.row = current_point.row + 1;
                    next_point.col = current_point.col + 1;
                    break;
                default:
                    continue;
            }
            
            // 检查边界和障碍物
            if (next_point.row < 0 || next_point.row >= cell_mat.rows ||
                next_point.col < 0 || next_point.col >= cell_mat.cols) {
                continue;
            }
            
            if (!boundingJudge(next_point.row, next_point.col, cell_mat)) {
                continue;
            }
            
            // 计算神经网络输出
            float current_neural_value = neural_mat_.at<float>(next_point.row, next_point.col);
            
            // 方向改变惩罚
            delta_theta = theta - last_theta;
            if (delta_theta > 180) delta_theta -= 360;
            if (delta_theta < -180) delta_theta += 360;
            
            // 神经网络公式计算
            e = current_neural_value;
            v_1 = v;
            v = e - 0.1 * abs(delta_theta);  // 简化的神经网络模型
            
            if (v > max_v) {
                max_v = v;
                max_index = i;
            }
        }
        
        // 更新当前位置
        CellIndex next_point;
        switch (max_index) {
            case 0: // 0度
                next_point.row = current_point.row;
                next_point.col = current_point.col + 1;
                next_point.theta = 0;
                break;
            case 1: // 45度
                next_point.row = current_point.row - 1;
                next_point.col = current_point.col + 1;
                next_point.theta = 45;
                break;
            case 2: // 90度
                next_point.row = current_point.row - 1;
                next_point.col = current_point.col;
                next_point.theta = 90;
                break;
            case 3: // 135度
                next_point.row = current_point.row - 1;
                next_point.col = current_point.col - 1;
                next_point.theta = 135;
                break;
            case 4: // 180度
                next_point.row = current_point.row;
                next_point.col = current_point.col - 1;
                next_point.theta = 180;
                break;
            case 5: // 225度
                next_point.row = current_point.row + 1;
                next_point.col = current_point.col - 1;
                next_point.theta = 225;
                break;
            case 6: // 270度
                next_point.row = current_point.row + 1;
                next_point.col = current_point.col;
                next_point.theta = 270;
                break;
            case 7: // 315度
                next_point.row = current_point.row + 1;
                next_point.col = current_point.col + 1;
                next_point.theta = 315;
                break;
            default:
                // 无法继续，结束路径规划
                cout << "[" << getAlgorithmName() << "] No valid direction found. Stopping at iteration " << loop << endl;
                goto planning_complete;
        }
        
        // 边界检查
        if (next_point.row < 0 || next_point.row >= cell_mat.rows ||
            next_point.col < 0 || next_point.col >= cell_mat.cols ||
            !boundingJudge(next_point.row, next_point.col, cell_mat)) {
            cout << "[" << getAlgorithmName() << "] Reached boundary or obstacle. Stopping at iteration " << loop << endl;
            break;
        }
        
        current_point = next_point;
        path_vec.push_back(current_point);
        
        // 每1000次迭代输出一次进度
        if (loop % 1000 == 0) {
            cout << "[" << getAlgorithmName() << "] Iteration " << loop 
                 << ", current position: (" << current_point.row << ", " << current_point.col << ")" << endl;
        }
    }
    
planning_complete:
    cout << "[" << getAlgorithmName() << "] Neural network planning completed. Total path points: " 
         << path_vec.size() << endl;
    
    return path_vec;
}

bool NeuralAlgorithm::boundingJudge(int row, int col, const Mat& cell_mat) {
    if (row < 0 || row >= cell_mat.rows || col < 0 || col >= cell_mat.cols) {
        return false;
    }
    
    // 检查3x3邻域是否有障碍物
    int obstacle_count = 0;
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            int check_row = row + i;
            int check_col = col + j;
            
            if (check_row >= 0 && check_row < cell_mat.rows &&
                check_col >= 0 && check_col < cell_mat.cols) {
                
                if (cell_mat.at<uchar>(check_row, check_col) != 0) {  // 0表示自由空间
                    obstacle_count++;
                }
            }
        }
    }
    
    return obstacle_count == 0;  // 周围没有障碍物才可通行
}

string NeuralAlgorithm::getAlgorithmName() const {
    return "Neural Network Algorithm";
}

string NeuralAlgorithm::getParameterDescription() const {
    stringstream ss;
    ss << "Neural Network Algorithm Parameters:\n";
    ss << "- c_0: " << c_0_ << "\n";
    ss << "- max_iterations: " << max_iterations_ << "\n";
    ss << "- theta_directions: 8 (0, 45, 90, 135, 180, 225, 270, 315 degrees)";
    return ss.str();
}

void NeuralAlgorithm::setParameter(const string& param_name, double param_value) {
    if (param_name == "c_0") {
        c_0_ = static_cast<float>(param_value);
    } else if (param_name == "max_iterations") {
        max_iterations_ = static_cast<int>(param_value);
    } else {
        cout << "[" << getAlgorithmName() << "] Warning: Unknown parameter: " << param_name << endl;
    }
}

void NeuralAlgorithm::reset() {
    neural_mat_.release();
}

// 注册Neural算法到工厂
class NeuralAlgorithmRegistrar {
public:
    NeuralAlgorithmRegistrar() {
        PathPlanningAlgorithmFactory::registerAlgorithm("neural", []() {
            return make_shared<NeuralAlgorithm>();
        });
    }
};

static NeuralAlgorithmRegistrar neural_registrar;
