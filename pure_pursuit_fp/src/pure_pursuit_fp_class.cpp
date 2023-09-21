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
#include "pure_pursuit_server.h"

using namespace std;

class PurePursuitController
{
private:
    static PurePursuitController *instance;  // 静态指针

    ros::NodeHandle nh;
    ros::Subscriber controller_sub, path_sub;
    ros::Publisher vel_pub, lookahead_pub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    int control_rate;
    float look_head_dis;
    float wheel_base;
    float max_speed;
    float kp, ki, kd;

    int nearest_idx;
    int idx;
    float xc, yc, vel, yaw, v_prev_error;
    bool get_path;
    vector<vector<float>> waypoints;

    geometry_msgs::Twist msg;

    dynamic_reconfigure::Server<pure_pursuit_fp::PIDConfig> server;
    dynamic_reconfigure::Server<pure_pursuit_fp::PIDConfig>::CallbackType f;

public:
    PurePursuitController()
      : tfListener(tfBuffer)
      , nearest_idx(0)
      , idx(0)
      , xc(0.0)
      , yc(0.0)
      , vel(0.0)
      , yaw(0.0)
      , v_prev_error(0.0)
      , get_path(false)
      , control_rate(10)
    {
        f = bind(&PurePursuitController::reconfigureCallback, this, _1, _2);
        server.setCallback(f);

        controller_sub = nh.subscribe("odom", 1, &PurePursuitController::velCallback, this);
        path_sub = nh.subscribe("move_base/NavfnROS/plan", 1, &PurePursuitController::globalPathCallback, this);
        vel_pub = nh.advertise<geometry_msgs::Twist>("new_cmd_vel", 1);
        lookahead_pub = nh.advertise<geometry_msgs::PointStamped>("lookahead_point", 1);

        std::thread tf_thread(bind(&PurePursuitController::tfThread, this, ref(tfBuffer)));
        tf_thread.detach();

        // 在构造函数中设置指针并绑定信号
        instance = this;
        signal(SIGINT, signalHandler);
    }

    void run()
    {
        ros::Rate rate(control_rate);
        ros::spinOnce();

        while (ros::ok())
        {
            if (!get_path)
            {
                ROS_INFO("...wait for path...");
                this_thread::sleep_for(chrono::milliseconds(1000));
                ros::spinOnce();
                continue;
            }

            getCurPos(tfBuffer);

            if (nearest_idx < waypoints.size() - 1)
            {
                PurePursuit(lookahead_pub);
                vel_pub.publish(msg);
                ROS_INFO("[velo,yaw] = [%f],[%f]", msg.linear.x, msg.angular.z);

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
    }

    void tfThread(tf2_ros::Buffer &tfBuffer)
    {
        ros::Rate rate(100.0);
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
    }

private:
    // 在类中的静态信号处理函数
    static void signalHandler(int signum)
    {
        if (instance)
        {
            instance->handleSigint();
        }
        exit(signum);
    }

    // 这个函数被上面的静态函数调用
    void handleSigint()
    {
        ROS_INFO("收到 Ctrl+C 信号。退出...");

        // 发布0速度
        geometry_msgs::Twist stopMsg;
        stopMsg.linear.x = 0.;
        stopMsg.angular.z = 0.;
        vel_pub.publish(stopMsg);
    }

    void reconfigureCallback(pure_pursuit_fp::PIDConfig &config, uint32_t level)
    {
        control_rate = config.control_rate;
        look_head_dis = config.min_ld;
        wheel_base = config.car_wheel_base;
        max_speed = config.max_speed;
        kp = config.kp;
        ki = config.ki;
        kd = config.kd;
        ROS_INFO("Reconfigured with kp=%f, ki=%f, kd=%f", kp, ki, kd);
    }

    void velCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        vel = msg->twist.twist.linear.x;
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
};

PurePursuitController *PurePursuitController::instance = nullptr;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit");
    PurePursuitController controller;
    controller.run();
    return 0;
}
