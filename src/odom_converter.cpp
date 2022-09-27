#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

nav_msgs::Odometry msg_odom;
geometry_msgs::Twist msg_twist;
geometry_msgs::Pose msg_pose;
bool newTwist = false, newPose = false;

void callback_odom_twist(const geometry_msgs::Twist::ConstPtr& msg)
{
    msg_twist = *msg;
    newTwist = true;
    msg_odom.twist.twist = msg_twist;
}

void callback_odom_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
    msg_pose = *msg;
    newPose = true;
    msg_odom.pose.pose = msg_pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_converter");
    ros::NodeHandle nh;

    ros::Subscriber sub_odom_twist = nh.subscribe("/odom/twist", 0, callback_odom_twist);
    ros::Subscriber sub_odom_pose = nh.subscribe("/odom/pose", 0, callback_odom_pose);
    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 0);

    msg_odom.header.frame_id = "map";
    msg_odom.header.stamp = ros::Time::now();
    msg_odom.child_frame_id = "odom";

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate loopRate(30);

    while (ros::ok())
    {
        loopRate.sleep();
        ros::spinOnce();

        if (newTwist == true || newPose == true) {
            newTwist = false;
            newPose = false;

            // Tf
            transform.setOrigin( tf::Vector3(
                msg_odom.pose.pose.position.x, 
                msg_odom.pose.pose.position.y, 
                msg_odom.pose.pose.position.z) );
            tf::Quaternion q;
            q.setX(msg_odom.pose.pose.orientation.x);
            q.setY(msg_odom.pose.pose.orientation.y);
            q.setZ(msg_odom.pose.pose.orientation.z);
            q.setW(msg_odom.pose.pose.orientation.w);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));

            // Pub odom
            pub_odom.publish(msg_odom);

            ROS_INFO("Updating odom tf...");
        }
        
    }

    return 0;
}
