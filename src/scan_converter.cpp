#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

#define PI 3.14159265358979323846
#define ANGLE_RISING 1
#define ANGLE_FALLING 2

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

void publishAll(tf::TransformBroadcaster* tfBroadcaster, tf::Transform* tf, ros::Publisher* pub_scan)
{
    // Tf
    // ROS_INFO("Publishing scan tf...");
    tf->setOrigin( tf::Vector3(0.0, 0.0, 0.1) );
    tf->setRotation( tf::Quaternion(0.0, 0.0, 0.7068252, -0.7073883) );
    tfBroadcaster->sendTransform(tf::StampedTransform(*tf, ros::Time::now(), "base_link", "scan_link"));

    // Scan conversion
    // ROS_INFO("Publishing scan...");
    
    msg_scan.header.stamp = ros::Time::now();
    pub_scan->publish(msg_scan);
}

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
    msg_scan.scan_time = 2;
    msg_scan.angle_min = 0;
    msg_scan.angle_max = PI;
    msg_scan.angle_increment = PI / 180;
    msg_scan.range_max = 5;
    msg_scan.range_min = 0;
    msg_scan.ranges.clear();
    msg_scan.ranges.resize(180);

    ROS_INFO("Looping...");

    while (ros::ok())
    {
        loopRate.sleep();
        ros::spinOnce();

        int index = 0;

        if (newAngle == true || newRange == true) {
            newAngle = false;
            newRange = false;

            // Limpa o array
            for (int i=0; i < msg_scan.ranges.size(); i++)
                msg_scan.ranges[i] = 0.0;

            // Atualiza range
            ROS_INFO("Updating range %f for angle %d", msg_scanRange.data, msg_scanAngle.data);
            if (msg_scanAngle.data == 0)
                index = 0;
            else
                index = msg_scanAngle.data - 1;
            msg_scan.ranges[index] = msg_scanRange.data;


            // Publica
            publishAll(&tfBroadcaster, &tf, &pub_scan);
        }
    }

    return 0;
}
