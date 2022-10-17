#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

bool newTwist = false, newPose = false;

nav_msgs::Odometry msg_odom;
geometry_msgs::Twist msg_twist;
geometry_msgs::Pose msg_pose;

void Callback_odom_twist(const geometry_msgs::Twist::ConstPtr& msg)
{
    msg_twist = *msg;
    newTwist = true;
    msg_odom.twist.twist = msg_twist;
}

void Callback_odom_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
    msg_pose = *msg;
    newPose = true;
    msg_odom.pose.pose = msg_pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_converter");
    ros::NodeHandle nh;

    ros::Subscriber sub_odom_twist = nh.subscribe("/odom/twist", 0, Callback_odom_twist);
    ros::Subscriber sub_odom_pose = nh.subscribe("/odom/pose", 0, Callback_odom_pose);
    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 0);

    tf::TransformBroadcaster tfBroadcaster;
    tf::Transform tf;

    ros::Rate loopRate(30);

    msg_odom.header.frame_id = "map";
    msg_odom.header.stamp = ros::Time::now();
    msg_odom.child_frame_id = "base_link";

    ROS_INFO("Looping...");

    tf::Vector3 position(0.0, 0.0, 0.0);
    tf::Quaternion orientation(0.0, 0.0, 0.0, 1.0);

    while (ros::ok())
    {
        loopRate.sleep();
        ros::spinOnce();

        if (newTwist == true || newPose == true) {
            newTwist = false;
            newPose = false;

            // Atualiza posição
            position.setX(msg_odom.pose.pose.position.x);
            position.setY(msg_odom.pose.pose.position.y);
            position.setZ(msg_odom.pose.pose.position.z);

            // Atualiza orientação
            orientation.setX(msg_odom.pose.pose.orientation.x);
            orientation.setY(msg_odom.pose.pose.orientation.y);
            orientation.setZ(msg_odom.pose.pose.orientation.z);
            orientation.setW(msg_odom.pose.pose.orientation.w);
        }

        // Tf
        ROS_INFO("Updating odom tf...");
        tf.setOrigin(position);
        tf.setRotation(orientation);
        tfBroadcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "map", "base_link"));

        // Pub odom
        ROS_INFO("Publishing odom...");
        pub_odom.publish(msg_odom);
        
    }

    return 0;
}
