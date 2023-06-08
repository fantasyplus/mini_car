#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/timer.h"
#include "std_msgs/Bool.h"
#include "ros/subscriber.h"
#include "ros/publisher.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/convert.h>

#include <csignal>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <functional>

#include <termios.h>
#include <unistd.h>

nav_msgs::Odometry odom;
geometry_msgs::Twist vel;
geometry_msgs::PointStamped stop_point;
bool is_clicked_point = false;

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

        struct termios old_tio, new_tio;
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        exit(0);
    }
}

void callbackTimerPublishTF(const ros::TimerEvent &e, tf::TransformBroadcaster &tf_broadcaster)
{
    if (is_clicked_point == false)
    {
        return;
    }

    geometry_msgs::TransformStamped stop_point_tf;

    stop_point_tf.header.frame_id = "map";
    stop_point_tf.header.stamp = ros::Time::now();

    stop_point_tf.child_frame_id = "stop_point";

    geometry_msgs::Pose stop_pose;
    stop_pose.position = stop_point.point;

    stop_point_tf.transform.translation.x = stop_pose.position.x;
    stop_point_tf.transform.translation.y = stop_pose.position.y;
    stop_point_tf.transform.translation.z = stop_pose.position.z;
    stop_point_tf.transform.rotation.w = 1.0;

    tf_broadcaster.sendTransform(stop_point_tf);
}

double getTFDistance(const std::string stop_frame, const std::string odom_frame, tf2_ros::Buffer *tf_buffer)
{
    geometry_msgs::TransformStamped transformStampedOdom;
    geometry_msgs::TransformStamped transformStampedStop;
    try
    {
        transformStampedOdom = tf_buffer->lookupTransform("map", odom_frame, ros::Time(0));
        // ROS_INFO("get transform from map to odom");
        // ROS_INFO("translation: (%f, %f, %f)", transformStampedOdom.transform.translation.x,
        //          transformStampedOdom.transform.translation.y, transformStampedOdom.transform.translation.z);
        // ROS_INFO("rotation: (%f, %f, %f, %f)", transformStampedOdom.transform.rotation.x,
        //          transformStampedOdom.transform.rotation.y, transformStampedOdom.transform.rotation.z,
        //          transformStampedOdom.transform.rotation.w);
        odom.pose.pose.position.x = transformStampedOdom.transform.translation.x;
        odom.pose.pose.position.y = transformStampedOdom.transform.translation.y;
        odom.pose.pose.position.z = transformStampedOdom.transform.translation.z;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    try
    {
        transformStampedStop = tf_buffer->lookupTransform("map", stop_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    double distance =
        sqrt(pow(transformStampedOdom.transform.translation.x - transformStampedStop.transform.translation.x, 2) +
             pow(transformStampedOdom.transform.translation.y - transformStampedStop.transform.translation.y, 2));

    return distance;
}

void velCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    vel = *msg;
}

void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    ROS_INFO("set stop point at (%f, %f)", msg->point.x, msg->point.y);
    stop_point = *msg;

    is_clicked_point = true;
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
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
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

    // tf related
    tf::TransformBroadcaster tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    ros::Timer timer_stop_point = nh.createTimer(
        ros::Duration(0.02), std::bind(callbackTimerPublishTF, std::placeholders::_1, std::ref(tf_broadcaster)));

    ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 1, velCallback);
    ros::Subscriber point_sub = nh.subscribe("/clicked_point", 1, pointCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/new_cmd_vel", 10);

    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        geometry_msgs::PoseStamped stop_pose;
        stop_pose.header.frame_id = "map";
        stop_pose.pose.position.x = stop_point.point.x;
        stop_pose.pose.position.y = stop_point.point.y;
        stop_pose.pose.position.z = 0.05;

        stop_pose.pose.orientation.w = 1;

        double distance = getTFDistance("stop_point", "base_link", tf_buffer.get());

        ROS_INFO("car's velocity : %f", vel.linear.x);
        ROS_INFO("Stop point at: (%f, %f)", stop_pose.pose.position.x, stop_pose.pose.position.y);
        ROS_INFO("Car pose at: (%f, %f)", odom.pose.pose.position.x, odom.pose.pose.position.y);
        ROS_INFO("Distance to stop point: %f\n", distance);

        if (distance < 0.5 && is_clicked_point)
        {
            ROS_INFO("---------enter stop area--------\n");

            geometry_msgs::Twist zero_vel;
            zero_vel.linear.x = 0;
            vel_pub.publish(zero_vel);

            if (key == 's')
            {
                ROS_INFO("---------drive out of stop area--------");

                exitFlag = true;

                double duration = 2.0;

                ros::Time start_time = ros::Time::now();
                ros::Rate rate(10);

                auto publishCmdVel = [&]()
                {
                    while ((ros::Time::now() - start_time).toSec() < duration)
                    {
                        vel_pub.publish(vel);

                        // ROS_INFO("publish vel");
                        rate.sleep();
                    }
                };

                std::thread t1(publishCmdVel);
                t1.join();
            }
        }
        else
        {
            vel_pub.publish(vel);
        }

        rate.sleep();
    }

    return 0;
}
