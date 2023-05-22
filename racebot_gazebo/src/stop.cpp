#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "ros/subscriber.h"
#include "ros/publisher.h"
#include "nav_msgs/Odometry.h"

#include <csignal>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <condition_variable>

#include <termios.h>
#include <unistd.h>

nav_msgs::Odometry odom;

std::condition_variable cv;
std::mutex mtx;
std::atomic<bool> exitFlag(false);

void signalHandler(int signum)
{
    if (signum == SIGINT)
    {
        ROS_INFO("stop node shutdown");

        // 设置退出标志
        {
            std::lock_guard<std::mutex> lock(mtx);
            exitFlag = true;
        }

        // 通知所有线程退出循环
        cv.notify_all();
        exit(0);
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
}

void readFromKeyboard(char &key)
{
    while (1)
    {
        struct termios old_tio, new_tio;

        // 检查退出标志
        {
            std::unique_lock<std::mutex> lock(mtx);
            if (exitFlag)
            {
                tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                break;
            }
        }

        // 设置终端属性，禁用行缓冲和回显
        tcgetattr(STDIN_FILENO, &old_tio);
        new_tio = old_tio;
        new_tio.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

        // 读取键盘输入
        ROS_INFO("Press s to drive out of the stop area");
        std::cin >> key;
        ROS_INFO("You pressed: %c", key);

        // 恢复终端属性
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::this_thread::yield();
    }
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);

    char key;
    std::thread t1(readFromKeyboard, std::ref(key));
    t1.detach();

    ros::init(argc, argv, "teb_stop_node");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher stop_move_base_pub = nh.advertise<std_msgs::Bool>("/stop_move_base", 10);

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

            geometry_msgs::Twist vel;
            vel.linear.x = 0;
            vel_pub.publish(vel);

            std_msgs::Bool stop_move_base;
            if (key == 's')
            {
                stop_move_base.data = false;
                stop_move_base_pub.publish(stop_move_base);

                exitFlag = true;
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                break;
            }
            else
            {
                stop_move_base.data = true;
                stop_move_base_pub.publish(stop_move_base);
            }
        }

        rate.sleep();
    }

    return 0;
}
