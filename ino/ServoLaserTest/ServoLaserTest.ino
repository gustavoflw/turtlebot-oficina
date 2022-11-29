#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>

#include "LidarController.h"

#define PIN_SERVO 35

LidarController lidar;

ros::NodeHandle nh;
std_msgs::Int64 msg_scanAngle;
std_msgs::Float64 msg_scanRange;
ros::Publisher pub_scanAngle("/scan/angle", &msg_scanAngle);
ros::Publisher pub_scanRange("/scan/range", &msg_scanRange);

void setup() {
  lidar.Setup(PIN_SERVO, 3);
  RosSetup();
}

void loop() {
  delay(50);

  RosUpdate();

  lidar.Update();
  msg_scanAngle.data = lidar.GetAngle();
  msg_scanRange.data = lidar.GetRange();
  // msg_scanRange.data = 1.0;
}

void RosSetup() {
  nh.initNode();
  nh.advertise(pub_scanAngle);
  nh.advertise(pub_scanRange);
}

void RosUpdate() {
  nh.spinOnce();
  pub_scanAngle.publish(&msg_scanAngle);
  pub_scanRange.publish(&msg_scanRange);
}