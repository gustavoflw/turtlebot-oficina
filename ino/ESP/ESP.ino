#define ROSSERIAL_ARDUINO_TCP

#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/Bool.h>                // http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html
#include <std_msgs/Int32.h>               // http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Int32.html
#include <std_msgs/Float64.h>             // http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html
#include <sensor_msgs/Temperature.h>      // https://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html
#include <sensor_msgs/RelativeHumidity.h> // http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/RelativeHumidity.html
#include <geometry_msgs/Twist.h>          // http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
#include <geometry_msgs/Vector3.h>        // http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html
#include <geometry_msgs/Pose.h>           // http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html

#include "SerialCom.hpp"
#include "MyWiFi.hpp"

/* CONSTANTES */
#define t_delay 50
#define serialRate 57600

/* WIFI */
const char *wifi_ssid      = "elonmusk";
const char *wifi_password  = "elonmusk";
int        rosmaster_port  = 11411;
uint8_t    newMACAddress[] = {0x5C, 0xE8, 0x83, 0x36, 0x9A, 0xC3};
IPAddress  gateway(192, 168, 4, 1);
IPAddress  rosmaster(gateway[0], gateway[1], gateway[2], 31        );
IPAddress  localIP  (gateway[0], gateway[1], gateway[2], 32        );
IPAddress  dns      (gateway[0], gateway[1], gateway[2], gateway[3]);
IPAddress  subnet   (       255,        255,        255,        0  );

/* TEMPO */
unsigned long t_now   = millis();
unsigned long t_last  = t_now;
unsigned long dt      = 0;

/* MENSAGENS ROS */
std_msgs::Int32       msg_time;
std_msgs::Float64     msg_rpmL;
std_msgs::Float64     msg_rpmR;
std_msgs::Float64     msg_odom_theta;
geometry_msgs::Twist  msg_cmd_vel;
geometry_msgs::Twist  msg_odom_twist;
geometry_msgs::Pose   msg_odom_pose;

/* COMUNICAÇÃO SERIAL */
SerialCom serialCom;

/* PARA SABER SE TEM NOVO CMD_VEL */
bool new_cmd_vel = false;

/* NODO ROS */
ros::NodeHandle nh;

/* CALLBACKS */
void callback_cmd_vel(const geometry_msgs::Twist& msg);

/* PUBLISHERS */
ros::Publisher pub_time("/esp/time", &msg_time);
ros::Publisher pub_rpmL("/rpm/l", &msg_rpmL);
ros::Publisher pub_rpmR("/rpm/r", &msg_rpmR);
ros::Publisher pub_odom_theta("/odom/theta", &msg_odom_theta);
ros::Publisher pub_odom_twist("/odom/twist", &msg_odom_twist);
ros::Publisher pub_odom_pose("/odom/pose", &msg_odom_pose);

/* SUBSCRIBERS */
ros::Subscriber <geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", &callback_cmd_vel);

/***** SETUP *****/
void setup() 
{
  Serial.begin(serialRate);
  SetupWifi();
  SetupROS();
}

/***** LOOP *****/
void loop() 
{
  delay(1);

  if (IsAllowedToLoop(t_last, t_delay)) {
    nh.spinOnce(); // "F5"
    UpdateTimeVariables();
    UpdateSerial();
    UpdateROS();
  }
}

void callback_cmd_vel(const geometry_msgs::Twist& msg)
{
  new_cmd_vel = true;
  msg_cmd_vel = msg;

  if (fabs(msg_cmd_vel.linear.x) < 0.1)
    msg_cmd_vel.linear.x = 0.0; 
  if (fabs(msg_cmd_vel.angular.z) < 0.1)
    msg_cmd_vel.angular.z = 0.0;
}

void SetupWifi()
{
  // MyWifi::SetupClient(wifi_ssid, wifi_password, localIP, gateway, subnet, dns);
  MyWifi::SetupAP(wifi_ssid, wifi_password, newMACAddress); // IP padrão = 192.168.4.1
  pinMode           (LED_BUILTIN, OUTPUT);
  digitalWrite      (LED_BUILTIN, LOW   ); // LOW acende o led (??????)
}

void SetupROS()
{
  nh.getHardware()->setConnection(rosmaster, rosmaster_port);
  nh.initNode();
  
  // Publishers
  nh.advertise(pub_time);
  nh.advertise(pub_rpmL);
  nh.advertise(pub_rpmR);
  nh.advertise(pub_odom_theta);
  nh.advertise(pub_odom_twist);
  nh.advertise(pub_odom_pose);

  // Subscribers
  nh.subscribe(sub_cmd_vel);
}

// Retorna true se o dt atual for maior que um certo valor
bool IsAllowedToLoop(unsigned long millis_lastLoop, unsigned long dt_min)
{
  unsigned long millis_now = millis();
  unsigned long dt = millis_now - millis_lastLoop;
  if (dt > dt_min)
    return true;
  else
    return false;
}

// Atualiza variáveis de tempo
void UpdateTimeVariables()
{
  t_last  = t_now;
  dt = t_now - t_last;
  t_now   = millis();
  msg_time.data = t_now;
  pub_time.publish(&msg_time);
}

// Atualiza serial
void UpdateSerial()
{
  // Manda comando de velocidade
  if (new_cmd_vel == true) {
    new_cmd_vel = false;
    serialCom.SendSpeedControl(msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
  }

  // Recebe informações e processa
  serialCom.ReceiveMsgWithMarkers();
  // serialCom.ProcessNewData();
}

// Atualiza ROS
void UpdateROS()
{
  pub_time.publish(&msg_time);
  pub_rpmL.publish(&msg_rpmL);
  pub_rpmR.publish(&msg_rpmR);
  pub_odom_theta.publish(&msg_odom_theta);
  pub_odom_twist.publish(&msg_odom_twist);
  pub_odom_pose.publish(&msg_odom_pose);
}