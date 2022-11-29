#ifndef SERIALCOM_HPP
#define SERIALCOM_HPP

#include <SoftwareSerial.h>

#define RECEIVEDFLOATS 13
#define RECEIVEDLEN RECEIVEDFLOATS * sizeof(float) + 2

class SerialCom {

public:

  bool newData = false;

  void ReceiveMsg(geometry_msgs::Twist* msg_odom_twist, geometry_msgs::Pose* msg_odom_pose,
                  std_msgs::Float64* msg_rpmL, std_msgs::Float64* msg_rpmR,
                  std_msgs::Float64* msg_odom_theta,
                  std_msgs::Int32* msg_laser_angle, std_msgs::Float64* msg_laser_range) {
    char rc;
    float data[RECEIVEDFLOATS] = { 0 };
    bool okMsg = false;

    unsigned char buffer[RECEIVEDLEN] = { 0 };
    if (Serial.available() > 0) {
      rc = Serial.read();
      if (rc == '<') {
        buffer[0] = (unsigned char)rc;
        Serial.readBytes(&buffer[1], RECEIVEDLEN - 1);
        // for (int i = 0; i < RECEIVEDLEN; i++)
        //   Serial.println((char)buffer[i]);
        if (buffer[RECEIVEDLEN - 1] == '>') {
          Serial.println("Ok msg!");
          int i = 0;
          int j = 1;
          Serial.print("Received info: ");
          while (i != RECEIVEDFLOATS) {
            memcpy(&data[i], &buffer[j], sizeof(float));
            Serial.print(data[i]);
            Serial.print(",");
            i = i + 1;
            j = j + 4;
          }
          Serial.println();
          msg_odom_pose->position.x = data[0];
          msg_odom_pose->position.y = data[1];
          msg_odom_theta->data = data[2];
          msg_odom_twist->linear.x = data[3];
          msg_odom_twist->linear.y = data[4];
          msg_odom_pose->orientation.x = data[5];
          msg_odom_pose->orientation.y = data[6];
          msg_odom_pose->orientation.z = data[7];
          msg_odom_pose->orientation.w = data[8];
          msg_rpmL->data = data[9];
          msg_rpmR->data = data[10];
          msg_laser_angle->data = data[11];
          msg_laser_range->data = data[12];
        }
      }
    }
  }

  void SendSpeedControl(double v_x, double w_z) {
    float data[2] = { v_x, w_z };
    Serial.print('<');
    Serial.write((char*)data, 2 * sizeof(float));
    Serial.print('>');
  }
};

#endif