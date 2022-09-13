#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry msg_odom;
geometry_msgs::Twist msg_twist;
geometry_msgs::Pose msg_pose;
bool newTwist = false, newPose = false;

void callback_odom_twist(const geometry_msgs::Twist::ConstPtr& msg)
{
    msg_twist = *msg;
    newTwist = true;
}

void callback_odom_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
    msg_pose = *msg;
    newPose = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xz_controller");
    ros::NodeHandle nh;

    ros::Subscriber sub_odom_twist = nh.subscribe("/odom/twist", 0, callback_odom_twist);
    ros::Subscriber sub_odom_pose = nh.subscribe("/odom/pose", 0, callback_odom_pose);
    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 0);

    msg_odom.header.frame_id = "frame_id";
    // msg_odom.header.seq;
    msg_odom.header.stamp = ros::Time::now();
    msg_odom.child_frame_id = "child_id";
    

    ros::Rate loopRate(30);

    while (ros::ok())
    {
        loopRate.sleep();
        ros::spinOnce();

        if (newTwist == true || newPose == true) {
            newTwist = false;
            newPose = false;

            msg_odom.pose.pose = msg_pose;
            msg_odom.twist.twist = msg_twist;

            ROS_INFO("Updating odom");
        }

        pub_odom.publish(msg_odom);
    }

    return 0;
}
