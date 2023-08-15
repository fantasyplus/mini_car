#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pure_pursuit_fp/PurePursuitAction.h>
#include "actionlib/client/simple_client_goal_state.h"
#include "nav_msgs/Path.h"
#include "pure_pursuit_fp/PurePursuitActionResult.h"
#include "pure_pursuit_fp/PurePursuitFeedback.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"

pure_pursuit_fp::PurePursuitGoal goal;
bool flag = false;
bool is_success = false;
void globalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    flag = true;
    goal.GlobalPath.header = msg->header;
    goal.GlobalPath.poses = msg->poses;
}

void feedbackCallback(const pure_pursuit_fp::PurePursuitActionFeedback::ConstPtr &msg)
{
    ROS_INFO("get vel:%f,yaw:%f", msg->feedback.vel, msg->feedback.yaw);
}

void resCallback(const pure_pursuit_fp::PurePursuitActionResultConstPtr &msg)
{
    // ROS_INFO("result:%d", msg->result.is_success);
    is_success = msg->result.is_success;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_client_node");
    ros::NodeHandle nh;

    ros::Subscriber path_sub = nh.subscribe("move_base/NavfnROS/plan", 1, globalPathCallback);
    ros::Subscriber feedback_sub = nh.subscribe("pure_pursuit_server/feedback", 1, feedbackCallback);
    ros::Subscriber res_sub = nh.subscribe("pure_pursuit_server/result", 1, resCallback);

    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<pure_pursuit_fp::PurePursuitAction> ac("pure_pursuit_server", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ros::Rate rate(50);
    ros::spinOnce();
    while (ros::ok())
    {
        if (is_success)
        {
            ROS_INFO("Goal Reached");
            break;
        }

        if (!flag)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            ros::spinOnce();
            continue;
        }
        ROS_INFO("...get new global path,sending path...");
        ac.sendGoal(goal);

        flag = false;
        ros::spinOnce();
    }
    // Wait for the goal to finish or be preempted
    bool finishedBeforeTimeout = ac.waitForResult(ros::Duration(30.0));
    if (finishedBeforeTimeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_WARN("Action did not finish before the time out.");
    }

    return 0;
}