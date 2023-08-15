#include "pure_pursuit_server.h"
#include "ros/init.h"

PurePursuitServer *PurePursuitServer::globalServerInstance = nullptr;

// SIGINT 信号处理函数
void PurePursuitServer::signalHandler(int signum)
{
    std::cout << "收到 Ctrl+C 信号。退出..." << std::endl;
    // 发布消息
    msg.linear.x = 0.;
    msg.angular.z = 0.;
    vel_pub.publish(msg);
    // 在这里执行清理操作
    exit(signum);
}

float PurePursuitServer::norm(vector<float> vect)
{
    float sum = 0.;
    for (int i = 0; i < vect.size(); ++i)
    {
        sum += pow(vect[i], 2.);
    }
    return sqrt(sum);
}

void PurePursuitServer::getCurPos(tf2_ros::Buffer &tfBuffer)
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

void PurePursuitServer::tfThread(tf2_ros::Buffer &tfBuffer)
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

void PurePursuitServer::recongifureCallback(pure_pursuit_fp::PIDConfig &config, uint32_t level)
{
    control_rate = config.control_rate;
    look_head_dis = config.min_ld;
    wheel_base = config.car_wheel_base;
    max_speed = config.max_speed;
    kp = config.kp;
    ki = config.ki;
    kd = config.kd;

    ROS_INFO("reconfigure:kp = %f, ki = %f, kd = %f", kp, ki, kd);
    ROS_INFO("reconfigure:control_rate = %d, look_head_dis = %f, wheel_base = %f, max_speed = %f", control_rate,
             look_head_dis, wheel_base, max_speed);
}

void PurePursuitServer::velCallback(const nav_msgs::Odometry::ConstPtr &msg)
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

float PurePursuitServer::find_distance(float x1, float y1)
{
    float P = 2.0;
    float distance = sqrt(pow(x1 - xc, P) + pow(y1 - yc, P));
    return distance;
}

float PurePursuitServer::find_distance_index_based(int idx)
{
    float x1 = waypoints[idx][0];
    float y1 = waypoints[idx][1];
    return find_distance(x1, y1);
}

int PurePursuitServer::find_nearest_waypoint()
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

int PurePursuitServer::idx_close_to_lookahead(int idx)
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

void PurePursuitServer::init_waypoints(const pure_pursuit_fp::PurePursuitGoalConstPtr &global_path)
{
    // init waypoints
    waypoints.clear();
    for (int i = 0; i < global_path->GlobalPath.poses.size(); i++)
    {
        vector<float> wp;
        wp.push_back(global_path->GlobalPath.poses[i].pose.position.x);
        wp.push_back(global_path->GlobalPath.poses[i].pose.position.y);
        waypoints.push_back(wp);
    }
}

void PurePursuitServer::PurePursuit(ros::Publisher &lookahead_pub)
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

void PurePursuitServer::executeCallback(const pure_pursuit_fp::PurePursuitGoalConstPtr &global_path)
{
    ros::Rate rate(control_rate);
    while (ros::ok())
    {
        ros::spinOnce();

        if (as.isPreemptRequested())
        {
            // ROS_INFO("Get New Path");
            as.setPreempted();

            result.is_success = false;
            break;
        }

        init_waypoints(global_path);

        getCurPos(tfBuffer);

        if (nearest_idx < waypoints.size() - 1)
        {
            PurePursuit(lookahead_pub);
            vel_pub.publish(msg);

            feedback.vel = msg.linear.x;
            feedback.yaw = msg.angular.z;
            as.publishFeedback(feedback);

            // 等待直到下一次迭代。
            ros::spinOnce();
            rate.sleep();
        }
        else
        {
            // ROS_INFO("Goal Reached");
            msg.linear.x = 0.;
            msg.angular.z = 0.;
            vel_pub.publish(msg);

            result.is_success = true;
            as.setSucceeded(result);
            break;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_server_node");

    PurePursuitServer pure_pursuit_server;
    PurePursuitServer::globalServerInstance = &pure_pursuit_server;

    signal(SIGINT, &PurePursuitServer::handleSignal);

    ros::spin();

    return 0;
}