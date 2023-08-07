#include "pure_pursuit/pure_pursuit.h"
#include "ros/subscriber.h"

PurePursuit::PurePursuit()
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private("~");
    // Node paramters
    nh_private.param<double>("ld_gain", ld_gain_, 1.0);
    nh_private.param<double>("min_ld", min_ld_, 0.5);
    nh_private.param<double>("car_wheel_base", car_wheel_base_, 0.2);
    nh_private.param<int>("controller_freq", controller_freq_, 10);
    nh_private.param<double>("max_speed", max_speed_, 0.8);
    nh_private.param<std::string>("map_frame", map_frame_, "map");
    nh_private.param<std::string>("base_frame", base_frame_, "base_link");
    ld_ = min_ld_;
    // Publishers and subscribers

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/new_cmd_vel", 1);
    l_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/pure_pursuit/lookahead_point", 1);

    ros::Subscriber odom_sub_ = nh_.subscribe("/odom", 1, &PurePursuit::odomCallback, this);
    ros::Subscriber path_sub_ = nh_.subscribe("/move_base/NavfnROS/plan", 1, &PurePursuit::globalPathCallback, this);

    ros_rate_ = new ros::Rate(controller_freq_);
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
    // main loop
    control_loop_();
}

void PurePursuit::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    car_speed_ = msg->twist.twist.linear.x;
    ld_ = std::max(ld_gain_ * car_speed_, min_ld_);
}
void PurePursuit::globalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    ROS_INFO("New path is received.");
    path_ = msg->poses;
    // path_.push_back(msg->poses[0]);
    got_path_ = true;
    path_done_ = false;
    point_idx_ = 0;
    double start_end_dist = distance(path_[0].pose.position, path_.back().pose.position);
    ROS_INFO("Start to End Distance: %f", start_end_dist);
    ROS_INFO("Min lookup distance: %f", min_ld_);
    if (start_end_dist <= min_ld_)
    {
        loop_ = true;
        ROS_INFO("Is Loop: True");
    }
}

void PurePursuit::control_loop_()
{
    double y_t = 0, ld_2 = 0, delta = 0;
    double distance_ = 0;
    while (ros::ok())
    {
        if (got_path_)
        {
            // get the current robot location by tf base_link -> map
            // iterate over the path points
            // if the distance between a point and robot > lookahead break and take
            // this point transform this point to the robot base_link the y component
            // of this point is y_t delta can be computed as atan2(2 * yt * L_, ld_2)
            try
            {
                base_location_ = tfBuffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));

                for (; point_idx_ < path_.size(); point_idx_++)
                {
                    distance_ = distance(path_[point_idx_].pose.position, base_location_.transform.translation);
                    ROS_INFO("Point ID: %d, Distance %f", point_idx_, distance_);
                    if (distance_ >= ld_)
                    {
                        path_[point_idx_].header.stamp =
                            ros::Time::now();  // Set the timestamp to now for the transform
                                               // to work, because it tries to transform the
                                               // point at the time stamp of the input point
                        tfBuffer_.transform(path_[point_idx_], target_point_, base_frame_, ros::Duration(0.1));
                        break;
                    }
                }
                // Calculate the steering angle
                ld_2 = ld_ * ld_;
                y_t = target_point_.pose.position.y;
                delta = atan2(2 * car_wheel_base_ * y_t, ld_2);
                geometry_msgs::Twist cmd_vel_msg;
                cmd_vel_msg.linear.x = max_speed_;
                cmd_vel_msg.angular.z = delta * 1.4;
                cmd_vel_pub_.publish(cmd_vel_msg);

                last_p_idx_ = point_idx_;
                last_dist_ = distance_;
                if (point_idx_ == path_.size() && loop_)
                {
                    point_idx_ = 0;
                }
                else if (point_idx_ == path_.size())
                {
                    ROS_INFO("Reached final point");
                    geometry_msgs::Twist cmd_vel_msg;
                    cmd_vel_msg.linear.x = 0.0;
                    cmd_vel_msg.angular.z = 0.0;
                    cmd_vel_pub_.publish(cmd_vel_msg);
                    got_path_ = false;
                    point_idx_ = 0;
                }
                lookahead_p.point = path_[point_idx_].pose.position;
                lookahead_p.header = path_[point_idx_].header;
                l_point_pub_.publish(lookahead_p);  // Publish the lookahead point
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
            }
        }

        ros::spinOnce();
        ros_rate_->sleep();
    }
}

PurePursuit::~PurePursuit()
{
    delete tfListener_;
    delete ros_rate_;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit");
    PurePursuit pp_node;
}