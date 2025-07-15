#include <ros/ros.h>
#include <std_msgs/String.h>
#include "path_planning.h"

class AlgorithmSwitcher
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber algorithm_switch_sub_;
    PathPlanning* path_planning_;
    
public:
    AlgorithmSwitcher(PathPlanning* pp) : path_planning_(pp)
    {
        // 订阅算法切换话题
        algorithm_switch_sub_ = nh_.subscribe("switch_algorithm", 1, 
                                            &AlgorithmSwitcher::algorithmSwitchCallback, this);
        
        // 输出可用算法列表
        auto algorithms = path_planning_->getAvailableAlgorithms();
        ROS_INFO("Available algorithms:");
        for (const auto& algo : algorithms) {
            ROS_INFO("  - %s", algo.c_str());
        }
        ROS_INFO("Current algorithm: %s", path_planning_->getCurrentAlgorithmName().c_str());
        ROS_INFO("To switch algorithm, publish to topic: /switch_algorithm");
        ROS_INFO("Example: rostopic pub /switch_algorithm std_msgs/String \"data: 'astar'\"");
    }
    
    void algorithmSwitchCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::string algorithm_name = msg->data;
        ROS_INFO("Switching to algorithm: %s", algorithm_name.c_str());
        
        try {
            path_planning_->setAlgorithmType(algorithm_name);
            ROS_INFO("Successfully switched to: %s", path_planning_->getCurrentAlgorithmName().c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to switch algorithm: %s", e.what());
        }
    }
    
    void setAlgorithmParameters(const std::string& param_name, double value)
    {
        std::map<std::string, double> params;
        params[param_name] = value;
        path_planning_->setAlgorithmParameters(params);
        ROS_INFO("Set parameter %s = %.2f", param_name.c_str(), value);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "algorithm_switcher_demo");
    
    // 这里应该从你的主程序中获取 PathPlanning 实例
    // 这只是一个演示，实际使用时需要适配你的代码结构
    ROS_INFO("Algorithm Switcher Demo Node");
    ROS_INFO("This is a demonstration of the path planning algorithm interface.");
    ROS_INFO("To use in your actual system, integrate the AlgorithmSwitcher class");
    ROS_INFO("with your existing PathPlanning instance.");
    
    ros::spin();
    return 0;
}
