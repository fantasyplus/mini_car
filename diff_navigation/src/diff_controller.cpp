#include "diff_controller.h"

pathTracking::pathTracking(ros::NodeHandle &nh, ros::NodeHandle &nh_local)
{
    nh_ = nh;
    nh_local_ = nh_local;
    std_srvs::Empty e;
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("/params", &pathTracking::initializeParams, this);
    initializeParams(e.request, e.response);
    initialize();
}

void pathTracking::initialize()
{
    timer_ = nh_.createTimer(ros::Duration(1 / control_frequency_), &pathTracking::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency_), false);
    timer_.start();
    mode = MODE::IDLE;
    past_mode = MODE::IDLE;
    set_goal_as_localgoal = false;
    xy_reached = false;
    while (!ros::service::waitForService("/move_base/make_plan", ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for service /move_base/make_plan to become available");
    }

    ROS_INFO("Initialized!");
}

pathTracking::~pathTracking()
{
    nh_local_.deleteParam("/active");
    nh_local_.deleteParam("/control_frequency");
    nh_local_.deleteParam("/lookahead_distance");
    nh_local_.deleteParam("/robot_L");
    nh_local_.deleteParam("/xy_tolerance");
    nh_local_.deleteParam("/linear_max_vel");
    nh_local_.deleteParam("/linear_acceleration");
    nh_local_.deleteParam("/linear_kp");
    nh_local_.deleteParam("/linear_brake_distance");
    nh_local_.deleteParam("/linear_min_brake_distance");
    nh_local_.deleteParam("/linear_brake_distance_ratio");
    nh_local_.deleteParam("/theta_tolerance");
    nh_local_.deleteParam("/angular_max_vel");
    nh_local_.deleteParam("/angular_acceleration");
    nh_local_.deleteParam("/angular_kp");
    nh_local_.deleteParam("/angular_brake_distance");
    nh_local_.deleteParam("/angular_min_brake_distance");
    nh_local_.deleteParam("/angular_brake_distance_ratio");
}

bool pathTracking::initializeParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    bool get_param_ok = true;
    bool prev_active = p_active_;

    get_param_ok = nh_local_.param<bool>("/active", p_active_, true);
    get_param_ok = nh_local_.param<double>("/control_frequency", control_frequency_, 70);
    get_param_ok = nh_local_.param<double>("/lookahead_distance", lookahead_distance_, 0.3);
    get_param_ok = nh_local_.param<double>("/robot_L", robot_L_, 0.3);

    get_param_ok = nh_local_.param<double>("/xy_tolerance", xy_tolerance_, 0.02);
    get_param_ok = nh_local_.param<double>("/linear_max_vel", linear_max_vel_, 0.8);
    get_param_ok = nh_local_.param<double>("/linear_acceleration", linear_acceleration_, 0.3);
    get_param_ok = nh_local_.param<double>("/linear_kp", linear_kp_, 0.8);
    get_param_ok = nh_local_.param<double>("/linear_brake_distance", linear_brake_distance_, 0.15);
    get_param_ok = nh_local_.param<double>("/linear_min_brake_distance", linear_min_brake_distance_, 0.25);
    get_param_ok = nh_local_.param<double>("/linear_brake_distance_ratio", linear_brake_distance_ratio_, 0.3);

    get_param_ok = nh_local_.param<double>("/theta_tolerance", theta_tolerance_, 0.03);
    get_param_ok = nh_local_.param<double>("/angular_max_vel", angular_max_vel_, 3.0);
    get_param_ok = nh_local_.param<double>("/angular_acceleration", angular_acceleration_, 0.15);
    get_param_ok = nh_local_.param<double>("/angular_kp", angular_kp_, 1.5);
    get_param_ok = nh_local_.param<double>("/angular_brake_distance", angular_brake_distance_, 0.15);
    get_param_ok = nh_local_.param<double>("/angular_min_brake_distance", angular_min_brake_distance_, 0.15);
    get_param_ok = nh_local_.param<double>("/angular_brake_distance_ratio", angular_brake_distance_ratio_, 0.15);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
            vel_pub = nh_.advertise<geometry_msgs::Twist>("/new_cmd_vel", 10);
            localgoal_pub = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 10);
            // pose_sub = nh_.subscribe("/base_pose_ground_truth", 10, &pathTracking::poseCallback, this);
            // pose_sub = nh_.subscribe("/ekf_pose", 10, &pathTracking::poseCallback, this);
            pose_sub = nh_.subscribe("/odom", 1, &pathTracking::poseCallback, this);
            goal_sub = nh_.subscribe("/nav_goal", 10, &pathTracking::goalCallback, this);
        }
        else
        {
            vel_pub.shutdown();
            localgoal_pub.shutdown();
            pose_sub.shutdown();
            goal_sub.shutdown();
        }
    }

    if (get_param_ok)
    {
        ROS_INFO("set param ok");
    }
    else
    {
        ROS_WARN_STREAM("set param failed");
    }
    return true;
}

void pathTracking::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO("goalgoal - %lf %lf ", msg->pose.position.x, msg->pose.position.y);
    goal_pose.x = msg->pose.position.x;
    goal_pose.y = msg->pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    goal_pose.theta = yaw;
    ROS_INFO("RECEIVED GOAL - %f %f %f", goal_pose.x, goal_pose.y, goal_pose.theta);
    linear_brake_distance_ = countdistance(cur_pose, goal_pose) * linear_brake_distance_ratio_;
    if (linear_brake_distance_ < linear_min_brake_distance_)
    {
        linear_brake_distance_ = linear_min_brake_distance_;
    }
    pathRequest(cur_pose, goal_pose);
    switchMode(MODE::PATH_RECEIVED);
    xy_reached = false;
}

// void pathTracking::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) // base_pose_ground_truth
// {
//     cur_pose.x = msg->pose.pose.position.x;
//     cur_pose.y = msg->pose.pose.position.y;
//     tf2::Quaternion q;
//     tf2::fromMsg(msg->pose.pose.orientation, q);
//     tf2::Matrix3x3 qt(q);
//     double _, yaw;
//     qt.getRPY(_, _, yaw);
//     cur_pose.theta = yaw;
// }

// void pathTracking::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)// ekf_pose
// {  
//     cur_pose.x = msg->pose.pose.position.x;
//     cur_pose.y = msg->pose.pose.position.y;
//     tf2::Quaternion q;
//     tf2::fromMsg(msg->pose.pose.orientation, q);
//     tf2::Matrix3x3 qt(q);
//     double _, yaw;
//     qt.getRPY(_, _, yaw);
//     cur_pose.theta = yaw;
// }

void pathTracking::poseCallback(const nav_msgs::Odometry::ConstPtr &msg)  // odom
{
    cur_pose.x = msg->pose.pose.position.x;
    cur_pose.y = msg->pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    cur_pose.theta = yaw;
}

void pathTracking::pathRequest(RobotPose cur_, RobotPose goal_)
{
    ROS_INFO("cur_ - %f %f %f", cur_.x, cur_.y, cur_.theta);
    ROS_INFO("goal_ - %f %f %f", goal_.x, goal_.y, goal_.theta);

    geometry_msgs::PoseStamped cur;
    cur.header.frame_id = "map";
    cur.pose.position.x = cur_.x;
    cur.pose.position.y = cur_.y;
    cur.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, cur_.theta);
    cur.pose.orientation.x = q.x();
    cur.pose.orientation.y = q.y();
    cur.pose.orientation.z = q.z();
    cur.pose.orientation.w = q.w();

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = goal_.x;
    goal.pose.position.y = goal_.y;
    goal.pose.position.z = 0;

    q.setRPY(0, 0, goal_pose.theta);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    if (!client)
    {
        ROS_FATAL("Couid not initialize get plan service from %s", client.getService().c_str());
    }

    nav_msgs::GetPlan srv;
    srv.request.start = cur;
    srv.request.goal = goal;

    ROS_INFO("cur - %f %f %f %f %f %f", cur.pose.position.x, cur.pose.position.y, cur.pose.orientation.x,
             cur.pose.orientation.y, cur.pose.orientation.z, cur.pose.orientation.w);
    ROS_INFO("goal - %f %f %f %f %f %f", goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.x,
             goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);

    if (client.call(srv))
    {
        nav_msgs::Path path_msg;
        path_msg.poses = srv.response.plan.poses;
        global_path.clear();

        for (int i = 0; i < path_msg.poses.size(); i++)
        {
            RobotPose pose;
            pose.x = path_msg.poses[i].pose.position.x;
            pose.y = path_msg.poses[i].pose.position.y;
            tf2::Quaternion q;
            tf2::fromMsg(path_msg.poses[i].pose.orientation, q);
            tf2::Matrix3x3 qt(q);
            double _, yaw;
            qt.getRPY(_, _, yaw);
            pose.theta = yaw;
            global_path.push_back(pose);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service make_plan");
    }
}

void pathTracking::switchMode(MODE next)
{
    past_mode = mode;
    mode = next;
}

void pathTracking::timerCallback(const ros::TimerEvent &e)
{
    switch (mode)
    {
        case MODE::IDLE:
            ROS_INFO("IDLE");
            velocity.x = 0;
            velocity.y = 0;
            velocity.theta = 0;
            publishVelocity();
            break;
        case MODE::PATH_RECEIVED:
            ROS_INFO("PATH_RECEIVED");
            switchMode(MODE::TRACKING);
            break;
        case MODE::TRACKING:
            ROS_INFO("TRACKING");
            if (if_xy_reached(cur_pose, goal_pose) && if_theta_reached(cur_pose, goal_pose))  // reached
            {
                velocity.x = 0;
                velocity.y = 0;
                velocity.theta = 0;
                switchMode(MODE::IDLE);
                break;
            }
            RobotPose local_goal = rollingwindow(cur_pose);
            diff_controller(local_goal, cur_pose);
            break;
    }
}

RobotPose pathTracking::rollingwindow(RobotPose cur)
{
    ROS_INFO("Rolling Window START");
    RobotPose local_goal, past_local_goal;
    if (countdistance(cur_pose, goal_pose) < lookahead_distance_ + 0.2)
    {
        ROS_INFO("** special on the path");
        local_goal = goal_pose;
        set_goal_as_localgoal = true;
    }
    else
    {
        bool localgoal_found = false;
        int flag = 1, past_flag = 1;
        for (int i = 0; i < global_path.size(); i++)
        {
            past_flag = flag;
            if (countdistance(cur, global_path.at(i)) >= lookahead_distance_)
                flag = 1;  // exterior
            else
                flag = 0;  // interior

            if (flag - past_flag == 1)
            {
                local_goal = global_path.at(i);
                past_local_goal = global_path.at(i - 1);
                localgoal_found = true;
                ROS_INFO("on the path");
                break;
            }
        }

        if (localgoal_found)
        {
            local_goal.x = (local_goal.x + past_local_goal.x) / 2;
            local_goal.y = (local_goal.y + past_local_goal.y) / 2;
        }
        else
        {
            ROS_INFO("NOT on the path");
            double mindis = 10000, past_dis, dis = 0;
            for (int i = 0; i < global_path.size(); i++)
            {
                past_dis = dis;
                dis = countdistance(cur, global_path.at(i));
                if (mindis > dis)
                {
                    local_goal = global_path.at(i);
                    mindis = dis;
                }
            }
        }
        if (local_goal.x == goal_pose.x && local_goal.y == goal_pose.y)
        {
            set_goal_as_localgoal = true;
        }
    }

    // geometry_msgs::PoseStamped local;
    // local.header.stamp = ros::Time::now();
    // local.header.frame_id = "map";
    // local.pose.position.x = local_goal.x;
    // local.pose.position.y = local_goal.y;
    // local.pose.position.z = 0;
    // tf2::Quaternion q;
    // q.setRPY(0, 0, local_goal.theta);
    // local.pose.orientation.x = q.x();
    // local.pose.orientation.y = q.y();
    // local.pose.orientation.z = q.z();
    // local.pose.orientation.w = q.w();
    // localgoal_pub.publish(local);
    return local_goal;
}

void pathTracking::diff_controller(RobotPose localgoal, RobotPose cur)
{
    // count circular motion radius R // (robot line, cur_pose, localgoal)

    RobotPose robotLine, center;
    robotLine.x = sin(cur.theta) / cos(cur.theta);
    robotLine.y = -1;
    robotLine.theta = robotLine.x * cur.x + robotLine.y * cur.y;
    if (robotLine.x == 0)  // vertical -> horizon
    {
        center.y = cur.y;
        center.x = (pow(localgoal.x, 2) + pow(localgoal.y, 2) - pow(cur.x, 2) - pow(cur.y, 2) +
                    2 * center.y * (cur.y - localgoal.y)) /
                   2 * (localgoal.x - cur.x);
    }
    else if (robotLine.y == 0)  // horizon -> vertical
    {
        center.x = cur.x;
        center.y = (pow(localgoal.x, 2) + pow(localgoal.y, 2) - pow(cur.x, 2) - pow(cur.y, 2) +
                    2 * center.x * (cur.x - localgoal.x)) /
                   2 * (localgoal.y - cur.y);
    }
    else
    {
        // ax+by=c
        double a = -1 * robotLine.y;
        double b = robotLine.x;
        double c = a * cur.x + b * cur.y;
        center.x = (pow(cur.x, 2) + pow(cur.y, 2) - pow(localgoal.x, 2) - pow(localgoal.y, 2) -
                    2 * (cur.y - localgoal.y) * c / b) /
                   (2 * (cur.x - localgoal.x) - 2 * (cur.y - localgoal.y) * a / b);
        center.y = (c - a * center.x) / b;
    }
    circularMotion_R = countdistance(center, cur);

    // direction  go_forward_or_backward
    int go_forward_or_backward = 0;
    RobotPose vertical_robotLine;
    vertical_robotLine.x = 1;
    vertical_robotLine.y = robotLine.x;
    vertical_robotLine.theta = vertical_robotLine.x * cur.x + vertical_robotLine.y * cur.y;
    RobotPose fake;
    fake.x = cur.x + cos(cur.theta);
    fake.y = cur.y + sin(cur.theta);
    double local_ = vertical_robotLine.x * localgoal.x + vertical_robotLine.y * localgoal.y - vertical_robotLine.theta;
    double fake_ = vertical_robotLine.x * fake.x + vertical_robotLine.y * fake.y - vertical_robotLine.theta;
    if (local_ > 0 && fake_ > 0 || local_ < 0 && fake_ < 0)
    {
        go_forward_or_backward = 1;
    }
    else
    {
        go_forward_or_backward = -1;
    }
    ROS_INFO("go_forward_or_backward  =  %d", go_forward_or_backward);

    // localgoal_theta // count the slope : center, localgoal
    if (!xy_reached)
    {
        double slope = (localgoal.y - center.y) / (localgoal.x - center.x);
        localgoal.theta = atan2(-1, slope);
        RobotPose vec_cur, vec_local;
        vec_cur.x = (cur.x - center.x) / countdistance(cur, center);
        vec_cur.y = (cur.y - center.y) / countdistance(cur, center);
        vec_local.x = (localgoal.x - center.x) / countdistance(localgoal, center);
        vec_local.y = (localgoal.y - center.y) / countdistance(localgoal, center);
        double angle = acos(vec_cur.x * vec_local.x + vec_cur.y * vec_local.y);
        if (!(fabs(cur.theta - localgoal.theta) > angle - 0.03 && fabs(cur.theta - localgoal.theta) < angle + 0.03))
        {
            localgoal.theta = angleLimiting(atan2(-1, slope) + M_PI);
        }
    }
    else
    {
        localgoal.theta = goal_pose.theta;
    }

    geometry_msgs::PoseStamped local;
    local.header.stamp = ros::Time::now();
    local.header.frame_id = "map";
    local.pose.position.x = localgoal.x;
    local.pose.position.y = localgoal.y;
    local.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, localgoal.theta);
    local.pose.orientation.x = q.x();
    local.pose.orientation.y = q.y();
    local.pose.orientation.z = q.z();
    local.pose.orientation.w = q.w();
    localgoal_pub.publish(local);

    // rotate direction
    int rotate_direction = 0;
    // double value = robotLine.x * localgoal.x + robotLine.y * localgoal.y - robotLine.theta;
    // ROS_INFO("value = %f", value);
    // if ((value < 0 && go_forward_or_backward == 1) || (value > 0 && go_forward_or_backward == -1))
    // {
    //     rotate_direction = 1; // counter clock-wise
    // }
    // else
    // {
    //     rotate_direction = -1; // clock-wise
    // }

    double fake_x = cos(localgoal.theta);
    double fake_y = sin(localgoal.theta);
    double cross = cos(cur.theta) * fake_y - sin(cur.theta) * fake_x;
    if (cross > 0)
        rotate_direction = 1;
    else
        rotate_direction = -1;
    ROS_INFO("rotate_direction  =  %d", rotate_direction);

    // velocity
    // linear
    double v = (velocity.x + velocity.y) / 2;  // last
    if (if_xy_reached(cur_pose, goal_pose))
    {
        xy_reached = true;
        velocity.x = 0;
        velocity.y = 0;
    }
    else
    {
        // velocity.x = right wheel // velocity.y = left wheel
        double output_v = velocityProfile(v, go_forward_or_backward);
        velocity.x = (2 * output_v + output_v * robot_L_ / circularMotion_R) / 2;
        velocity.y = (2 * output_v - output_v * robot_L_ / circularMotion_R) / 2;
    }
    // angular
    if (if_xy_reached(cur_pose, goal_pose))
    {
        if (if_theta_reached(cur_pose, goal_pose))
        {
            velocity.theta = 0;
            switchMode(MODE::IDLE);
        }
        else
        {
            double theta_err = fabs(angleLimiting(cur_pose.theta - goal_pose.theta));
            ROS_INFO("theta_errrrrrrrr = %f", fabs(angleLimiting(cur_pose.theta - goal_pose.theta)));
            ROS_INFO("kp = %f", angular_kp_);
            velocity.theta = theta_err * angular_kp_;
            ROS_INFO("ppppp %f", velocity.theta);
            velocity.theta *= rotate_direction;
        }
    }
    else
    {
        velocity.theta = fabs(velocity.x - velocity.y) / robot_L_;
        if (velocity.theta > angular_max_vel_)
        {
            velocity.theta = angular_max_vel_;
        }
        velocity.theta *= rotate_direction;
    }

    publishVelocity();
}

double pathTracking::velocityProfile(double vel, int direction)
{
    double output_vel;
    double xy_err = countdistance(cur_pose, goal_pose);
    if (xy_err > linear_brake_distance_)
    {
        // acceleration
        double d_vel = linear_acceleration_ / control_frequency_;
        output_vel = abs(vel) + d_vel;
        if (output_vel > linear_max_vel_)
        {
            output_vel = linear_max_vel_;
        }
    }
    else
    {
        // deceleration
        output_vel = xy_err * linear_kp_;
    }
    output_vel *= direction;
    return output_vel;
}

double pathTracking::angleLimiting(double theta)
{
    while (theta > M_PI)
    {
        theta -= 2 * M_PI;
    }
    while (theta < -1 * M_PI)
    {
        theta += 2 * M_PI;
    }
    return theta;
}

void pathTracking::publishVelocity()
{
    geometry_msgs::Twist vel;
    // vel.linear.x = velocity.x;
    // vel.linear.y = velocity.y;
    vel.linear.x = (velocity.x + velocity.y) / 2;
    vel.linear.y = 0;
    vel.angular.z = velocity.theta;
    ROS_INFO("velocity = (%f, %f, %f)", vel.linear.x, vel.linear.y, velocity.theta);

    vel_pub.publish(vel);
}

double pathTracking::countdistance(RobotPose pose1, RobotPose pose2)
{
    return sqrt(pow(pose1.x - pose2.x, 2) + pow(pose1.y - pose2.y, 2));
}

int pathTracking::signDetermine(double a)
{
    if (a > 0)
        return 1;
    else if (a == 0)
        return 0;
    else
        return -1;
}

bool pathTracking::if_xy_reached(RobotPose cur, RobotPose goal)
{
    if (xy_tolerance_ >= countdistance(cur, goal))
        return true;
    else
        return false;
}

bool pathTracking::if_theta_reached(RobotPose cur, RobotPose goal)
{
    ROS_INFO("cur theta = %f", cur.theta);
    ROS_INFO("goal theta = %f", goal.theta);
    ROS_INFO("theta_err = %f", fabs(angleLimiting(cur.theta - goal.theta)));
    if (theta_tolerance_ >= fabs(angleLimiting(cur.theta - goal.theta)))
        return true;
    else
        return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diff_controller");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");
    pathTracking pathTracking_(nh, nh_local);

    while (ros::ok())
    {
        ros::spin();
    }
}