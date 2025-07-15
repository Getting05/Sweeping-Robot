#ifndef NEURAL_ALGORITHM_H
#define NEURAL_ALGORITHM_H

#include "path_planning_algorithm.h"

/**
 * @brief 原有神经网络路径规划算法的包装器
 * 将原有的PathPlanning类的算法包装成新的接口格式
 */
class NeuralAlgorithm : public PathPlanningAlgorithm 
{
public:
    NeuralAlgorithm();
    virtual ~NeuralAlgorithm() = default;

    bool initialize(costmap_2d::Costmap2D* costmap, int cell_size) override;
    
    vector<CellIndex> planPath(int start_row, int start_col, 
                              const vector<CellIndex>& free_space_vec,
                              const Mat& cell_mat) override;
    
    string getAlgorithmName() const override;
    string getParameterDescription() const override;
    void setParameter(const string& param_name, double param_value) override;
    void reset() override;

private:
    // 原有神经网络算法的核心逻辑
    vector<CellIndex> getPathInCV(const CellIndex& start_point, 
                                 const vector<CellIndex>& free_space_vec,
                                 const Mat& cell_mat);
    
    bool boundingJudge(int a, int b, const Mat& cell_mat);
    
    // 神经网络相关参数
    Mat neural_mat_;                    // 神经网络权值矩阵
    float c_0_;                        // 神经网络参数
    int max_iterations_;               // 最大迭代次数
    vector<float> theta_vec_;          // 方向角度集合
    
    // 初始化神经网络矩阵
    void initNeuralMat(const Mat& cell_mat, const vector<CellIndex>& free_space_vec);
};

#endif // NEURAL_ALGORITHM_H
