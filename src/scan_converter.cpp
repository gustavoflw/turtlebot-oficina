#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

# define PI 3.14159265358979323846

bool newAngle = false, newRange = false;

std_msgs::Int32 msg_scanAngle;
std_msgs::Float64 msg_scanRange;
sensor_msgs::LaserScan msg_scan;

void Callback_scanAngle(const std_msgs::Int32::ConstPtr& msg)
{
    msg_scanAngle = *msg;
    newAngle = true;
    ROS_INFO("New angle! %d", msg_scanAngle.data);
}

void Callback_scanRange(const std_msgs::Float64::ConstPtr& msg)
{
    msg_scanRange = *msg;
    newRange = true;
    ROS_INFO("New range! %f", msg_scanRange.data);
}

// void Callback_odom(const nav_msgs::Odometry::ConstPtr& msg)
// {

// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_converter");
    ros::NodeHandle nh;

    ros::Subscriber sub_scanAngle = nh.subscribe("/scan/angle", 1, Callback_scanAngle);
    ros::Subscriber sub_scanRange = nh.subscribe("/scan/range", 1, Callback_scanRange);
    ros::Publisher pub_scan = nh.advertise<sensor_msgs::LaserScan>("/scan", 0);

    tf::TransformBroadcaster tfBroadcaster;
    tf::Transform tf;

    ros::Rate loopRate(30);

    msg_scan.header.frame_id = "scan_link";
    msg_scan.angle_min = 0;
    msg_scan.angle_max = PI;
    msg_scan.angle_increment = PI / 180;
    msg_scan.range_max = 5;
    msg_scan.range_min = 0;
    msg_scan.ranges.resize(180);

    ROS_INFO("Looping...");

    int tmp = 0;

    while (ros::ok())
    {
        loopRate.sleep();
        ros::spinOnce();

        if (newAngle == true || newRange == true) {
            newAngle = false;
            newRange = false;

            // Limpa o array
            // for (int i=0; i < msg_scan.ranges.size(); i++)
                // msg_scan.ranges[i] = 20.0;
            
            // Atualiza ranges
            ROS_INFO("Updating range %f for angle %d", msg_scanRange.data, msg_scanAngle.data);
            msg_scan.ranges[msg_scanAngle.data-1] = msg_scanRange.data;
        }

        // Tf
        ROS_INFO("Updating scan tf...");
        tf.setOrigin( tf::Vector3(0.0, 0.0, 0.1) );
        tf.setRotation( tf::Quaternion(0.0, 0.0, 0.0, 1.0) );
        tfBroadcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "base_link", "scan_link"));

        // Scan conversion
        ROS_INFO("Publishing scan...");
        pub_scan.publish(msg_scan);
        
    }

    return 0;
}
