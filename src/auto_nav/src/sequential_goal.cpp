#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace tf;

// TF监听器
tf::TransformListener* tf_listener;

// 机器人当前位置（在map坐标系中）
float x_current_map = 0.0;
float y_current_map = 0.0;
bool position_initialized = false;

// 路径管理
vector<geometry_msgs::PoseStamped> path_points;
int current_goal_index = 0;
bool path_received = false;
bool goal_published = false;

// 参数
float tolerance_goal = 0.3;  // 目标容差
float goal_timeout = 5;   // 目标超时时间(秒) - 增加到10秒
ros::Time goal_start_time;
int consecutive_timeouts = 0;
int max_consecutive_timeouts = 5;  // 增加容错次数

// 可视化路径
nav_msgs::Path cleaned_path;
geometry_msgs::PoseStamped last_position;
float min_distance_threshold = 0.1;  // 最小距离阈值
float max_jump_threshold = 2.0;      // 最大跳跃阈值

// 发布器
ros::Publisher goal_pub;
ros::Publisher cleaned_path_pub;

// 欧拉角到四元数的转换
void eulerAngles2Quaternion(float yaw, float& w, float& x, float& y, float& z)
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(0 * 0.5);
    float sr = sin(0 * 0.5);
    float cp = cos(0 * 0.5);
    float sp = sin(0 * 0.5);

    w = cy * cr * cp + sy * sr * sp;
    x = cy * sr * cp - sy * cr * sp;
    y = cy * cr * sp + sy * sr * cp;
    z = sy * cr * cp - cy * sr * sp;
}

// 计算两点间距离
float calculateDistance(float x1, float y1, float x2, float y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// 将odom坐标系的位置转换到map坐标系
bool transformOdomToMap(float odom_x, float odom_y, float& map_x, float& map_y)
{
    try {
        // 使用稍旧的时间戳避免TF未来时间问题
        ros::Time transform_time = ros::Time::now() - ros::Duration(0.1);
        
        // 检查变换是否可用
        if (!tf_listener->canTransform("map", "odom", transform_time)) {
            // 尝试使用最新可用的变换
            transform_time = ros::Time(0);
            if (!tf_listener->canTransform("map", "odom", transform_time)) {
                ROS_WARN_THROTTLE(1.0, "Cannot transform from odom to map");
                return false;
            }
        }
        
        // 创建odom坐标系中的点
        geometry_msgs::PointStamped odom_point;
        odom_point.header.frame_id = "odom";
        odom_point.header.stamp = transform_time;  // 使用一致的时间戳
        odom_point.point.x = odom_x;
        odom_point.point.y = odom_y;
        odom_point.point.z = 0.0;
        
        // 转换到map坐标系
        geometry_msgs::PointStamped map_point;
        tf_listener->transformPoint("map", odom_point, map_point);
        
        map_x = map_point.point.x;
        map_y = map_point.point.y;
        return true;
    }
    catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "TF transform failed: %s", ex.what());
        return false;
    }
}

// 里程计回调函数
void pose_callback(const nav_msgs::Odometry &poses)
{
    // 从odom获取位置
    float odom_x = poses.pose.pose.position.x;
    float odom_y = poses.pose.pose.position.y;
    
    // 转换到map坐标系
    float new_map_x, new_map_y;
    if (!transformOdomToMap(odom_x, odom_y, new_map_x, new_map_y)) {
        return;  // TF变换失败，跳过此次更新
    }
    
    // 瞬移检测和过滤
    if (position_initialized) {
        float jump_distance = calculateDistance(x_current_map, y_current_map, new_map_x, new_map_y);
        
        // 如果位置跳跃过大，过滤掉这次更新
        if (jump_distance > max_jump_threshold) {
            ROS_WARN_THROTTLE(1.0, "Position jump detected (%.3f m), filtering out update from (%.3f,%.3f) to (%.3f,%.3f)", 
                            jump_distance, x_current_map, y_current_map, new_map_x, new_map_y);
            return;  // 拒绝这次更新
        }
        
        // 如果变化太小，也跳过（减少噪声）
        if (jump_distance < 0.01) {
            return;  // 位置变化太小，跳过
        }
    }
    
    // 更新位置
    x_current_map = new_map_x;
    y_current_map = new_map_y;
    position_initialized = true;
    
    // 更新已清扫路径（在map坐标系中）
    geometry_msgs::PoseStamped current_pose;
    current_pose.header.frame_id = "map";
    current_pose.header.stamp = ros::Time(0);  // 使用固定时间戳避免RViz显示跳跃
    current_pose.pose.position.x = x_current_map;
    current_pose.pose.position.y = y_current_map;
    current_pose.pose.position.z = 0.0;
    current_pose.pose.orientation.w = 1.0;
    
    // 检查是否应该添加新点
    bool should_add_point = false;
    
    if (cleaned_path.poses.empty()) {
        // 第一个点，直接添加
        should_add_point = true;
        ROS_INFO("Starting cleaned path at (%.2f, %.2f)", x_current_map, y_current_map);
    } else {
        // 计算与上一个点的距离
        float distance = calculateDistance(
            last_position.pose.position.x, last_position.pose.position.y,
            x_current_map, y_current_map
        );
        
        // 过滤跳跃和添加有意义的点
        if (distance > max_jump_threshold) {
            ROS_WARN_THROTTLE(1.0, "Large position jump detected (%.2f m), filtering out", distance);
        } else if (distance >= min_distance_threshold) {
            should_add_point = true;
        }
    }
    
    if (should_add_point) {
        // 永久保存到历史路径中，不删除任何点
        cleaned_path.poses.push_back(current_pose);
        last_position = current_pose;
        
        // 更新路径头部信息
        cleaned_path.header.frame_id = "map";
        cleaned_path.header.stamp = ros::Time(0);  // 使用固定时间戳避免RViz显示问题
        
        // 降低发布频率以减少RViz显示跳跃
        static int publish_counter = 0;
        publish_counter++;
        if (publish_counter >= 10) {  // 改为每10次更新发布一次，降低频率
            cleaned_path_pub.publish(cleaned_path);
            publish_counter = 0;
            
            // 输出路径统计信息
            if (cleaned_path.poses.size() % 100 == 0) {  // 每100个点打印一次
                ROS_INFO("Cleaned path now has %d points, current position: (%.2f, %.2f)", 
                        (int)cleaned_path.poses.size(), x_current_map, y_current_map);
            }
        }
    }
}

// 路径回调函数
void path_callback(const nav_msgs::Path &path)
{
    if (path.poses.size() == 0) {
        ROS_WARN("Received empty path!");
        return;
    }
    
    // 只在第一次接收到路径时或路径大小改变时更新
    if (!path_received || path.poses.size() != path_points.size()) {
        path_points.clear();
        path_points = path.poses;
        current_goal_index = 0;
        goal_published = false;
        consecutive_timeouts = 0;
        
        // 只在首次接收路径时清空历史轨迹，后续更新保留历史
        if (!path_received) {
            // 首次接收路径，初始化清扫路径
            cleaned_path.poses.clear();
            cleaned_path.header.frame_id = "map";
            cleaned_path.header.stamp = ros::Time(0);  // 使用固定时间戳避免RViz跳跃
            ROS_INFO("First path received with %d goals, initializing cleaned path", (int)path_points.size());
        } else {
            // 路径更新，保留已有的清扫历史
            ROS_INFO("Path updated with %d goals, keeping existing cleaned path (%d points)", 
                    (int)path_points.size(), (int)cleaned_path.poses.size());
        }
        
        path_received = true;
    }
}

// 发布下一个目标
void publishNextGoal()
{
    if (current_goal_index >= path_points.size()) {
        ROS_INFO("All goals completed! Mission finished.");
        return;
    }
    
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = ros::Time::now();
    
    // 设置目标位置
    goal_msg.pose.position.x = path_points[current_goal_index].pose.position.x;
    goal_msg.pose.position.y = path_points[current_goal_index].pose.position.y;
    goal_msg.pose.position.z = 0.0;
    
    // 计算朝向下一个点的角度
    float angle = 0.0;
    if (current_goal_index < path_points.size() - 1) {
        // 朝向下一个点
        float dx = path_points[current_goal_index + 1].pose.position.x - path_points[current_goal_index].pose.position.x;
        float dy = path_points[current_goal_index + 1].pose.position.y - path_points[current_goal_index].pose.position.y;
        angle = atan2(dy, dx);
    } else {
        // 最后一个点，朝向第一个点
        float dx = path_points[0].pose.position.x - path_points[current_goal_index].pose.position.x;
        float dy = path_points[0].pose.position.y - path_points[current_goal_index].pose.position.y;
        angle = atan2(dy, dx);
    }
    
    float w, x, y, z;
    eulerAngles2Quaternion(angle, w, x, y, z);
    goal_msg.pose.orientation.w = w;
    goal_msg.pose.orientation.x = x;
    goal_msg.pose.orientation.y = y;
    goal_msg.pose.orientation.z = z;
    
    // 发布目标
    goal_pub.publish(goal_msg);
    goal_start_time = ros::Time::now();
    goal_published = true;
    
    float distance = calculateDistance(x_current_map, y_current_map, 
                                      goal_msg.pose.position.x, 
                                      goal_msg.pose.position.y);
    
    ROS_INFO("Publishing goal %d: (%.2f, %.2f), distance: %.2f", 
             current_goal_index, 
             goal_msg.pose.position.x, 
             goal_msg.pose.position.y, 
             distance);
}

// 检查是否到达当前目标
bool isGoalReached()
{
    if (current_goal_index >= path_points.size() || !position_initialized) {
        return false;
    }
    
    float distance = calculateDistance(x_current_map, y_current_map,
                                     path_points[current_goal_index].pose.position.x,
                                     path_points[current_goal_index].pose.position.y);
    
    return distance <= tolerance_goal;
}

// 检查目标是否超时
bool isGoalTimeout()
{
    if (!goal_published) {
        return false;
    }
    
    double elapsed_time = (ros::Time::now() - goal_start_time).toSec();
    return elapsed_time > goal_timeout;
}

// 跳过当前目标
void skipCurrentGoal()
{
    consecutive_timeouts++;
    ROS_WARN("Goal %d timeout after %.1f seconds (consecutive timeouts: %d), skipping to next goal", 
             current_goal_index, goal_timeout, consecutive_timeouts);
    
    current_goal_index++;
    goal_published = false;
    
    // 如果连续超时太多次，跳过更多目标
    if (consecutive_timeouts >= max_consecutive_timeouts) {
        int skip_count = consecutive_timeouts / max_consecutive_timeouts;
        current_goal_index += skip_count;
        ROS_WARN("Too many consecutive timeouts, skipping %d additional goals", skip_count);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sequential_goal");
    ros::NodeHandle nh;
    
    // 初始化TF监听器
    tf_listener = new tf::TransformListener();
    
    // 等待TF缓冲区填充和关键变换可用
    ROS_INFO("Waiting for TF transforms...");
    ros::Duration(2.0).sleep();
    
    // 等待关键的TF变换可用
    try {
        tf_listener->waitForTransform("map", "odom", ros::Time::now(), ros::Duration(10.0));
        tf_listener->waitForTransform("map", "base_footprint", ros::Time::now(), ros::Duration(10.0));
        ROS_INFO("TF transforms are ready");
    }
    catch (tf::TransformException& ex) {
        ROS_ERROR("Failed to get TF transforms: %s", ex.what());
        ROS_ERROR("This may cause position jumping issues!");
    }
    
    // 订阅器
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1000, pose_callback);
    ros::Subscriber path_sub = nh.subscribe("/plan_path", 1000, path_callback);
    
    // 发布器
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
    cleaned_path_pub = nh.advertise<nav_msgs::Path>("passedPath", 1000);
    
    // 获取参数
    if (!nh.getParam("/NextGoal/tolerance_goal", tolerance_goal)) {
        ROS_WARN("Using default tolerance_goal: %.2f", tolerance_goal);
    }
    
    ros::Rate loop_rate(10);  // 10Hz
    
    ROS_INFO("Sequential goal node started. Waiting for path...");
    
    while (ros::ok()) {
        ros::spinOnce();
        
        if (path_received && path_points.size() > 0 && position_initialized) {
            
            // 检查是否到达当前目标
            if (goal_published && isGoalReached()) {
                ROS_INFO("Goal %d reached! Moving to next goal.", current_goal_index);
                current_goal_index++;
                goal_published = false;
                consecutive_timeouts = 0;  // 重置连续超时计数
            }
            
            // 检查目标是否超时
            else if (goal_published && isGoalTimeout()) {
                skipCurrentGoal();
            }
            
            // 发布下一个目标
            if (!goal_published && current_goal_index < path_points.size()) {
                publishNextGoal();
            }
            
            // 检查是否完成所有目标
            if (current_goal_index >= path_points.size()) {
                ROS_INFO("All %d goals completed! Cleaning mission finished.", (int)path_points.size());
                ROS_INFO("Total consecutive timeouts: %d", consecutive_timeouts);
                ROS_INFO("Total cleaned path points: %d", (int)cleaned_path.poses.size());
                break;
            }
        }
        
        loop_rate.sleep();
    }
    
    // 清理
    delete tf_listener;
    
    return 0;
}
