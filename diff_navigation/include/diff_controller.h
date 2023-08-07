#include "ros/ros.h"
#include <iostream>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
// message
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetPlan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
using namespace std;

enum class MODE
{
    IDLE,
    PATH_RECEIVED,
    TRACKING,
    TRANSITION
};

class RobotPose
{
public:
    double x;
    double y;
    double theta;
};

class pathTracking
{
public:
    pathTracking(ros::NodeHandle &nh, ros::NodeHandle &nh_local);
    ~pathTracking();
    void initialize();
    bool initializeParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;

    RobotPose goal_pose;
    RobotPose cur_pose;
    void pathRequest(RobotPose cur_, RobotPose goal_);
    std::vector<RobotPose> global_path;

    RobotPose rollingwindow(RobotPose cur);
    bool set_goal_as_localgoal;
    double circularMotion_R;
    void diff_controller(RobotPose localgoal, RobotPose cur);
    double velocityProfile(double vel, int direction);
    double angleLimiting(double theta);

    MODE mode;
    MODE past_mode;
    void switchMode(MODE next);

    // publisher
    ros::Publisher vel_pub;
    ros::Publisher localgoal_pub;

    // subscriber
    ros::Subscriber pose_sub;
    // void poseCallback(const nav_msgs::Odometry::ConstPtr &msg); //base_pose_ground_truth
    // void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg); // ekf_pose
    void poseCallback(const nav_msgs::Odometry::ConstPtr &msg);  // odom
    ros::Subscriber goal_sub;
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // timer
    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent &e);

    RobotPose velocity;
    void publishVelocity();

    double countdistance(RobotPose pose1, RobotPose pose2);
    bool if_xy_reached(RobotPose cur, RobotPose goal);
    bool if_theta_reached(RobotPose cur, RobotPose goal);
    bool xy_reached;
    int signDetermine(double a);

    // param
    bool p_active_;
    double control_frequency_;
    double lookahead_distance_;
    // robot
    double robot_L_;
    // linear
    double xy_tolerance_;
    double linear_max_vel_;
    double linear_acceleration_;
    double linear_kp_;
    double linear_brake_distance_;
    double linear_min_brake_distance_;
    double linear_brake_distance_ratio_;
    // angular
    double theta_tolerance_;
    double angular_max_vel_;
    double angular_acceleration_;
    double angular_kp_;
    double angular_brake_distance_;
    double angular_min_brake_distance_;
    double angular_brake_distance_ratio_;
};