#ifndef PATH_PLANNING_ALGORITHM_H
#define PATH_PLANNING_ALGORITHM_H

#include <vector>
#include <memory>
#include <string>
#include <functional>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>

using namespace cv;
using namespace std;

/**
 * @brief CellIndex结构体定义 - 必须在类定义之前声明
 */
struct CellIndex
{
    int row;
    int col;
    double theta;       // {0,45,90,135,180,225,270,315}
    
    CellIndex() : row(0), col(0), theta(0) {}
    CellIndex(int r, int c, double t = 0) : row(r), col(c), theta(t) {}
    
    bool operator==(const CellIndex& other) const {
        return row == other.row && col == other.col;
    }
    
    bool operator<(const CellIndex& other) const {
        if (row != other.row) return row < other.row;
        return col < other.col;
    }
};

/**
 * @brief 路径规划算法的抽象基类接口
 * 所有路径规划算法都应该继承此接口并实现相应的方法
 */
class PathPlanningAlgorithm 
{
public:
    virtual ~PathPlanningAlgorithm() = default;

    /**
     * @brief 初始化算法
     * @param costmap 代价地图指针
     * @param cell_size 栅格单元大小
     * @return 初始化是否成功
     */
    virtual bool initialize(costmap_2d::Costmap2D* costmap, int cell_size) = 0;

    /**
     * @brief 执行路径规划
     * @param start_row 起始行
     * @param start_col 起始列
     * @param free_space_vec 自由空间向量
     * @param cell_mat 栅格地图
     * @return 规划的路径点序列
     */
    virtual vector<CellIndex> planPath(int start_row, int start_col, 
                                      const vector<CellIndex>& free_space_vec,
                                      const Mat& cell_mat) = 0;

    /**
     * @brief 获取算法名称
     * @return 算法名称字符串
     */
    virtual string getAlgorithmName() const = 0;

    /**
     * @brief 获取算法参数配置
     * @return 参数描述字符串
     */
    virtual string getParameterDescription() const = 0;

    /**
     * @brief 设置算法参数
     * @param param_name 参数名称
     * @param param_value 参数值
     */
    virtual void setParameter(const string& param_name, double param_value) = 0;

    /**
     * @brief 重置算法状态
     */
    virtual void reset() = 0;

protected:
    costmap_2d::Costmap2D* costmap_;
    int cell_size_;
    bool initialized_ = false;
};

/**
 * @brief 路径规划算法工厂类
 */
class PathPlanningAlgorithmFactory 
{
public:
    /**
     * @brief 创建指定类型的路径规划算法
     * @param algorithm_type 算法类型名称
     * @return 算法实例的智能指针
     */
    static shared_ptr<PathPlanningAlgorithm> createAlgorithm(const string& algorithm_type);

    /**
     * @brief 获取所有可用的算法类型
     * @return 算法类型名称列表
     */
    static vector<string> getAvailableAlgorithms();

    /**
     * @brief 注册新的算法类型
     * @param algorithm_type 算法类型名称
     * @param creator 创建函数
     */
    static void registerAlgorithm(const string& algorithm_type, 
                                 function<shared_ptr<PathPlanningAlgorithm>()> creator);
};

#endif // PATH_PLANNING_ALGORITHM_H
