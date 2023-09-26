#include <fcntl.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <limits>
#include <string>
#include <thread>
#include <functional>
#include <vector>
#include <sstream>
#include <csignal>
#include <cstdlib>
#include <math.h> /* atan, sin */

// ros 相关
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/publisher.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "geometry_msgs/Transform.h"
#include "pure_pursuit/PIDConfig.h"

#define _USE_MATH_DEFINES

using namespace std;

// 全局变量
int control_rate = 10;
int nearest_idx = 0;
int idx = 0;
float xc = 0.;
float yc = 0.;
float vel = 0.;
float yaw = 0.;
float v_prev_error = 0.;
float max_speed = 0.;
vector<vector<float>> waypoints;

float kp = 0.1;
float ki = 0.;
float kd = 0.;

bool get_path = false;

// 小车变量
float look_head_dis = 0.1;
float wheel_base = 0.04;  // 轴距

// 程序变量
bool pure_pursuit_flag = true;

// 初始化 Twist 消息
geometry_msgs::Twist msg;

ros::Publisher vel_pub;

// SIGINT 信号处理函数
void signalHandler(int signum)
{
    std::cout << "收到 Ctrl+C 信号。退出..." << std::endl;
    // 发布消息
    msg.linear.x = 0.;
    msg.angular.z = 0.;
    vel_pub.publish(msg);
    // 在这里执行清理操作
    exit(signum);
}

// ----- 里程计 ------ //
float norm(vector<float> vect)
{
    float sum = 0.;
    for (int i = 0; i < vect.size(); ++i)
    {
        sum += pow(vect[i], 2.);
    }
    return sqrt(sum);
}

void reconfigureCallback(pure_pursuit::PIDConfig &config, uint32_t level)
{
    control_rate = config.control_rate;
    look_head_dis = config.min_ld;
    wheel_base = config.car_wheel_base;
    max_speed = config.max_speed;
    kp = config.kp;
    ki = config.ki;
    kd = config.kd;

    ROS_INFO("reconfigure:kp = %f, ki = %f, kd = %f", kp, ki, kd);
}

void getCurPos(tf2_ros::Buffer &tfBuffer)
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("TransformException: %s", ex.what());
        throw ex;
    }

    geometry_msgs::Transform tf = transformStamped.transform;

    xc = tf.translation.x;
    yc = tf.translation.y;
    float zc = tf.translation.z;

    float qx = tf.rotation.x;
    float qy = tf.rotation.y;
    float qz = tf.rotation.z;
    float qw = tf.rotation.w;

    // 从四元数计算欧拉角
    tf::Quaternion quat(qx, qy, qz, qw);
    tf::Matrix3x3 mat(quat);

    double curr_roll, curr_pitch, curr_yaw;
    mat.getEulerYPR(curr_yaw, curr_pitch, curr_roll);

    // 赋值给全局变量
    yaw = curr_yaw;
}

void velCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // 线速度
    float linear_vx = msg->twist.twist.linear.x;
    float linear_vy = msg->twist.twist.linear.y;
    float linear_vz = msg->twist.twist.linear.z;

    vector<float> vect_vel = {linear_vx, linear_vy, linear_vz};
    float linear_vel = norm(vect_vel);

    // 赋值给全局变量
    vel = linear_vel;
}

void globalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    waypoints.clear();
    for (int i = 0; i < msg->poses.size(); i++)
    {
        vector<float> wp;
        wp.push_back(msg->poses[i].pose.position.x);
        wp.push_back(msg->poses[i].pose.position.y);
        waypoints.push_back(wp);
    }
    get_path = true;
}

float find_distance(float x1, float y1)
{
    float P = 2.0;
    float distance = sqrt(pow(x1 - xc, P) + pow(y1 - yc, P));
    return distance;
}

float find_distance_index_based(int idx)
{
    float x1 = waypoints[idx][0];
    float y1 = waypoints[idx][1];
    return find_distance(x1, y1);
}

int find_nearest_waypoint()
{
    int nearest_idx = 0;
    float smallest_dist = numeric_limits<float>::max();
    float P = 2.;
    for (int i = 0; i < waypoints.size(); i++)
    {
        float wpx = waypoints[i][0];
        float wpy = waypoints[i][1];
        float idx_dist = pow(xc - wpx, P) + pow(yc - wpy, P);

        if (idx_dist < smallest_dist)
        {
            smallest_dist = idx_dist;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

int idx_close_to_lookahead(int idx)
{
    while (find_distance_index_based(idx) < look_head_dis)
    {
        idx += 1;
        if (idx == waypoints.size())
        {
            break;
        }
    }
    return idx - 1;
}

void PurePursuit(ros::Publisher &lookahead_pub)
{
    // 获取最近的路径点
    nearest_idx = find_nearest_waypoint();
    idx = idx_close_to_lookahead(nearest_idx);
    float target_x = waypoints[idx][0];
    float target_y = waypoints[idx][1];

    // 视觉前视点
    geometry_msgs::PointStamped lookhead_point;
    lookhead_point.header.frame_id = "map";
    lookhead_point.header.stamp = ros::Time::now();
    lookhead_point.point.x = target_x;
    lookhead_point.point.y = target_y;

    lookahead_pub.publish(lookhead_point);

    // 速度 PID 控制器
    float dt = 1. / control_rate;
    float v_desired = max_speed;
    float v_error = v_desired - vel;

    float P_vel = kp * v_error;
    float I_vel = ki * v_error * dt;
    float D_vel = kd * (v_error - v_prev_error) / dt;

    float velocity = P_vel + I_vel + D_vel;
    v_prev_error = v_error;

    // 纯追踪控制器
    float x_delta = target_x - xc;
    float y_delta = target_y - yc;
    float alpha = atan(y_delta / x_delta) - yaw;

    if (alpha > M_PI_2)
    {
        alpha -= M_PI;
    }
    if (alpha < -M_PI_2)
    {
        alpha += M_PI;
    }

    // 根据速度设置前视距离
    float lookahead = find_distance(target_x, target_y);
    float steering_angle = atan((2. * wheel_base * sin(alpha)) / lookahead);

    float k = 2 * sin(alpha) / lookahead;
    // ROS_INFO("k: %f", k);
    if (fabs(k) > 0.8)
    {
        velocity /= fabs(k);
        steering_angle *= fabs(k) * 2;
    }

    // 发布消息
    msg.linear.x = velocity;
    msg.angular.z = steering_angle;
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // 读取参数
    dynamic_reconfigure::Server<pure_pursuit::PIDConfig> server;
    dynamic_reconfigure::Server<pure_pursuit::PIDConfig>::CallbackType f;
    f = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(f);

    // 订阅者
    ros::Subscriber controller_sub = nh.subscribe("odom", 1, velCallback);
    ros::Subscriber path_sub = nh.subscribe("move_base/NavfnROS/plan", 1, globalPathCallback);

    // 发布者
    vel_pub = nh.advertise<geometry_msgs::Twist>("new_cmd_vel", 1);
    ros::Publisher lookahead_pub = nh.advertise<geometry_msgs::PointStamped>("lookahead_point", 1);

    signal(SIGINT, signalHandler);

    std::thread tfThread([&]() {
        ros::Rate rate(50.0);
        while (ros::ok())
        {
            try
            {
                // 填充 tf2 缓存
                tfBuffer.canTransform("map", "base_footprint", ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
            }
            rate.sleep();
        }
    });

    ros::Rate rate(control_rate);
    ros::spinOnce();

    while (ros::ok())
    {
        if (!get_path)
        {
            ROS_INFO("wait path...");
            this_thread::sleep_for(chrono::milliseconds(1000));
            ros::spinOnce();
            continue;
        }

        getCurPos(tfBuffer);

        if (nearest_idx < waypoints.size() - 1)
        {
            PurePursuit(lookahead_pub);
            vel_pub.publish(msg);
            ROS_INFO("speed:%f,yaw:%f",msg.linear.x,msg.angular.z);

            // 等待直到下一次迭代。
            ros::spinOnce();
            rate.sleep();
        }
        else
        {
            msg.linear.x = 0.;
            msg.angular.z = 0.;
            vel_pub.publish(msg);

            get_path = false;
            break;
        }
    }

    return 0;
}
