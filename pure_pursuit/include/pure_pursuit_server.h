#ifndef PURE_PURSUIT_SERVER_H
#define PURE_PURSUIT_SERVER_H

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
#include "pure_pursuit_fp/PIDConfig.h"
#include "pure_pursuit_fp/PurePursuitActionGoal.h"
#include "pure_pursuit_fp/PurePursuitGoal.h"
#include "tf2_ros/buffer.h"

// action
#include <actionlib/server/simple_action_server.h>
#include <pure_pursuit_fp/PurePursuitAction.h>

using namespace std;

class PurePursuitServer
{
public:
    PurePursuitServer()
      : nh("")
      , nh_private("~")
      , tfListener(tfBuffer)
      , tf_thread(bind(&PurePursuitServer::tfThread, this, ref(tfBuffer)))
      , as(nh, "pure_pursuit_server", bind(&PurePursuitServer::executeCallback, this, _1), false)
    {
        // 读取参数
        f = bind(&PurePursuitServer::recongifureCallback, this, _1, _2);
        server.setCallback(f);

        // 订阅者
        ros::Subscriber controller_sub = nh.subscribe("odom", 1, &PurePursuitServer::velCallback, this);

        // 发布者
        vel_pub = nh.advertise<geometry_msgs::Twist>("new_cmd_vel", 1);
        lookahead_pub = nh.advertise<geometry_msgs::PointStamped>("lookahead_point", 1);

        as.start();
    };
    ~PurePursuitServer(){};

    void signalHandler(int signum);
    static PurePursuitServer *globalServerInstance;  // 静态成员指针
    // 中介函数，转发到类成员函数
    static void handleSignal(int signal)
    {
        globalServerInstance->signalHandler(signal);
    }

private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Publisher lookahead_pub;
    ros::Publisher vel_pub;
    ros::Subscriber controller_sub;
    ros::Subscriber path_sub;

    pure_pursuit_fp::PurePursuitFeedback feedback;
    pure_pursuit_fp::PurePursuitResult result;

    dynamic_reconfigure::Server<pure_pursuit_fp::PIDConfig> server;
    dynamic_reconfigure::Server<pure_pursuit_fp::PIDConfig>::CallbackType f;

    thread tf_thread;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    // action server
    actionlib::SimpleActionServer<pure_pursuit_fp::PurePursuitAction> as;

private:
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


    // 小车变量
    float look_head_dis = 0.1;
    float wheel_base = 0.04;  // 轴距

    // 程序变量
    bool pure_pursuit_flag = true;

    // 初始化 Twist 消息
    geometry_msgs::Twist msg;

private:
    void executeCallback(const pure_pursuit_fp::PurePursuitGoalConstPtr &goal);
    void recongifureCallback(pure_pursuit_fp::PIDConfig &config, uint32_t level);
    void velCallback(const nav_msgs::OdometryConstPtr &msg);

private:
    float norm(vector<float> vect);
    void getCurPos(tf2_ros::Buffer &tfBuffer);
    float find_distance(float x1, float y1);
    float find_distance_index_based(int idx);
    int find_nearest_waypoint();
    int idx_close_to_lookahead(int idx);
    void init_waypoints(const pure_pursuit_fp::PurePursuitGoalConstPtr &global_path);

    // worker function
    void PurePursuit(ros::Publisher &lookahead_pub);

    // tf listern thread
    void tfThread(tf2_ros::Buffer &tfBuffer);
};

#endif  // PURE_PURSUIT_SERVER_H