#include <fcntl.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <vector>
#include <sstream>
#include "geometry_msgs/PointStamped.h"
#include "ros/publisher.h"
#include <math.h> /* atan, sin */

#define _USE_MATH_DEFINES

using namespace std;

// GLOBAL VARIABLES
int control_rate = 10;
int idx = 0;
float xc = 0.;
float yc = 0.;
float vel = 0.;
float yaw = 0.;
float v_prev_error = 0.;
float max_speed = 0.;
vector<vector<float>> waypoints;

float kp = 0.1;
float kd = 0.;
float ki = 0.;

bool get_path = false;

// CAR VARIABLES
float look_head_dis = 0.1;
float wheel_base = 0.04;  // wheelbase

// PORGRAM VARIABLES
bool pure_pursuit_flag = true;

// Initialize Twist message
geometry_msgs::Twist msg;

// ----- ODOMETRY ------ //
float norm(vector<float> vect)
{
    float sum = 0.;
    for (int i = 0; i < vect.size(); ++i)
    {
        sum += pow(vect[i], 2.);
    }
    return sqrt(sum);
}

/**
 * Callback function executes when new topic data comes.
 * Collects latest data for postion, orientation and velocity
 */
void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Position
    xc = msg->pose.pose.position.x;
    yc = msg->pose.pose.position.y;
    float zc = msg->pose.pose.position.z;

    // Orientation
    float qx = msg->pose.pose.orientation.x;
    float qy = msg->pose.pose.orientation.y;
    float qz = msg->pose.pose.orientation.z;
    float qw = msg->pose.pose.orientation.w;

    // Linear velocity
    float linear_vx = msg->twist.twist.linear.x;
    float linear_vy = msg->twist.twist.linear.y;
    float linear_vz = msg->twist.twist.linear.z;

    vector<float> vect_vel = {linear_vx, linear_vy, linear_vz};
    float linear_vel = norm(vect_vel);

    // Angular velocity
    float angular_vel = msg->twist.twist.angular.z;

    // Euler from Quaternion
    tf::Quaternion quat(qx, qy, qz, qw);
    tf::Matrix3x3 mat(quat);

    double curr_roll, curr_pitch, curr_yaw;
    mat.getEulerYPR(curr_yaw, curr_pitch, curr_roll);

    // Assign to global variables
    vel = linear_vel;
    yaw = curr_yaw;

    // TESTING: Print values
    // ROS_INFO("GLOBAL Position [%f, %f, %f], vel [%f], yaw [%f]", xc, yc, zc, vel, yaw);
    // ROS_INFO("Roll, Pitch, Yaw = [%f, %f, %f]", roll, pitch, yaw);
    // ROS_INFO("Seq -> [%d], Vel (linear x, norm, angular) -> [%f, %f, %f]", msg->header.seq, linear_vx, linear_vel,
    // angular_vel); ROS_INFO("Vel (linear x, norm) -> [%f, %f]", linear_vx, linear_vel); ROS_INFO("Position (xc, yc,
    // zc) -> [%f, %f, %f]", xc, yc, zc); ROS_INFO("Orientation (qx, qy, qz, qw) -> [%f, %f, %f, %f]", qx, qy, qz, qw);
    // ROS_INFO("\n");
}

void globalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    ROS_INFO("get path");

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

// ----- ARRAY MANIOULATION ------ //
float find_distance(float x1, float y1)
{
    float P = 2.0;  // power of 2
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
    float smallest_dist = 0.;
    float P = 2.;
    for (int i = 0; i < waypoints.size(); i++)
    {
        float wpx = waypoints[i][0];
        float wpy = waypoints[i][1];
        float idx_dist = pow(xc - wpx, P) + pow(yc - wpy, P);

        if (i == 0)
        {
            smallest_dist = idx_dist;
            nearest_idx = i;
        }
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
    // Get the closest waypoint
    int nearest_idx = find_nearest_waypoint();
    idx = idx_close_to_lookahead(nearest_idx);
    float target_x = waypoints[idx][0];
    float target_y = waypoints[idx][1];

    // visual lookhead point
    geometry_msgs::PointStamped lookhead_point;
    lookhead_point.header.frame_id = "map";
    lookhead_point.header.stamp = ros::Time::now();
    lookhead_point.point.x = target_x;
    lookhead_point.point.y = target_y;

    lookahead_pub.publish(lookhead_point);

    // Velocity PID controller

    float dt = 1. / control_rate;
    float v_desired = max_speed;
    float v_error = v_desired - vel;

    float P_vel = kp * v_error;
    float I_vel = v_error * dt;
    float D_vel = kd * (v_error - v_prev_error) / dt;

    float velocity = P_vel + I_vel + D_vel;
    v_prev_error = v_error;

    // Pure Pursuit controller
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

    // Set lookahead distance depending on the speed
    float lookahead = find_distance(target_x, target_y);
    float steering_angle = atan((2. * wheel_base * sin(alpha)) / lookahead);

    float k = 2 * sin(alpha) / lookahead;
    ROS_INFO("k: %f", k);
    if (fabs(k) > 1)
    {
        // velocity *= 0.5;
        steering_angle *= 1.5;
    }

    // Publish the message
    msg.linear.x = velocity;
    msg.angular.z = steering_angle;
}

// ------ MAIN FUNCTION ------ //
int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param<int>("controller_freq", control_rate, 10);
    nh_private.param<float>("min_ld", look_head_dis, 0.5);
    nh_private.param<float>("car_wheel_base", wheel_base, 0.2);
    nh_private.param<float>("max_speed", max_speed, 0.8);
    nh_private.param<float>("kp", kp, 1.2);
    nh_private.param<float>("kd", kd, 0.0);
    nh_private.param<float>("ki", ki, 0.0);

    // Initialize Subscriber
    ros::Subscriber controller_sub = nh.subscribe("odom", 1, poseCallback);
    ros::Subscriber path_sub = nh.subscribe("move_base/NavfnROS/plan", 1, globalPathCallback);

    // Initizlize Publisher
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("new_cmd_vel", 1);
    ros::Publisher lookahead_pub = nh.advertise<geometry_msgs::PointStamped>("lookahead_point", 1);

    // Initialize rate
    ros::Rate rate(control_rate);
    ros::spinOnce();

    // MOVE ROBOT
    while (ros::ok())
    {
        if (!get_path)
        {
            ROS_INFO("...waiting for path...");
            this_thread::sleep_for(chrono::milliseconds(1000));
            ros::spinOnce();
            continue;
        }

        // Send a message to rosout with the details.
        ROS_INFO_STREAM("index = " << waypoints.size() << " idx = " << idx << " current pose [" << xc << "," << yc
                                   << "] [vel, yaw] = [" << msg.linear.x << "," << msg.angular.z << "]");
        if (idx < waypoints.size() - 1)
        {
            PurePursuit(lookahead_pub);
            vel_pub.publish(msg);

            // Wait until it's time for another iteration.
            ros::spinOnce();
            rate.sleep();
        }
        else
        {
            // Publish the message
            msg.linear.x = 0.;
            msg.angular.z = 0.;
            vel_pub.publish(msg);

            get_path = false;
            break;
        }
    }

    ROS_INFO_STREAM("DESTENATION REACHED!!!");
    return 0;
}
