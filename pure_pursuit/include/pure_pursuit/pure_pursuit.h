#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP
#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>

class PurePursuit
{
private:
    double ld_gain_;
    double ld_;
    double min_ld_;
    double car_wheel_base_;
    double alpha_;
    double car_speed_;
    int controller_freq_;
    double max_speed_;
    int point_idx_;
    int last_p_idx_;
    double last_dist_ = std::numeric_limits<double>::infinity();
    bool got_path_ = false;
    bool path_done_ = true;
    bool loop_ = false;
    std::string map_frame_ = "map";
    std::string base_frame_ = "base_link";
    ros::Time last_msg_time_;
    std::vector<geometry_msgs::PoseStamped> path_;
    geometry_msgs::PoseStamped target_point_;
    geometry_msgs::PointStamped lookahead_p;
    geometry_msgs::TwistStamped cmd_vel_msg_;

    ros::Publisher control_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher l_point_pub_;
    ros::Publisher current_speed_pub_;
    geometry_msgs::TransformStamped base_location_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener *tfListener_;

    ros::Rate *ros_rate_;

    void cmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void globalPathCallback(const nav_msgs::Path::ConstPtr &msg);
    void control_loop_();

public:
    PurePursuit();
    ~PurePursuit();
};

template <typename T1, typename T2>
double distance(T1 pt1, T2 pt2)
{
    return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2));
}

#endif