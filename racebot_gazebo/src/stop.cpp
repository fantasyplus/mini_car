#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "ros/subscriber.h"
#include "ros/publisher.h"
#include "nav_msgs/Odometry.h"
#include <csignal>
#include <unistd.h>
#include <chrono>
#include <thread>

nav_msgs::Odometry odom;

void signalHandler(int signum)
{
    if (signum == SIGINT)
    {
        std::cout << "Received Ctrl+C signal" << std::endl;
        exit(0);
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);

    // 初始化ROS节点
    ros::init(argc, argv, "teb_stop_node");

    // 创建ROS节点句柄
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // 定义ROS话题、服务等相关的发布、订阅、服务等操作

    // 创建循环频率（以Hz为单位）
    ros::Rate rate(10);  // 每秒执行10次循环

    geometry_msgs::PoseStamped stop_pose;
    stop_pose.pose.position.x = 10.0013618469;
    stop_pose.pose.position.y = -4.35984659195;
    stop_pose.pose.position.z = 0.00437545776367;

    stop_pose.pose.orientation.w = 1;

    while (ros::ok())
    {
        ros::spinOnce();

        double distance = sqrt(pow(stop_pose.pose.position.x - odom.pose.pose.position.x, 2) +
                               pow(stop_pose.pose.position.y - odom.pose.pose.position.y, 2));
        if (distance < 0.5)
        {
            ROS_INFO("enter stop area");
            while (1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                geometry_msgs::Twist vel;
                vel.linear.x = 0;
                vel_pub.publish(vel);
            }
        }

        rate.sleep();
    }

    return 0;
}
