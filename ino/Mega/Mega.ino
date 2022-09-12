#include <ros.h>                  // Rosserial (usar 0.7.9)
#include <std_msgs/Int32.h>       // Msg do ROS para int
#include <nav_msgs/Odometry.h>    // http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
#include "Encoder.hpp"
#include "MotorController.hpp"
#include "Kinematics.hpp"
#include "Odometry.hpp"

// Constantes
#define pin_motor_L         4
#define pin_motor_R         3
#define pin_5V_encoder_L    30
#define pin_5V_encoder_R    28
#define pin_data_encoder_L  20
#define pin_data_encoder_R  19
#define t_delay             100
#define serialRate          9600
#define wheelRadius         4.0
#define wheelsAxisLength    10.0 
#define kp                  0.3
#define ki                  0.0
#define kd                  0.0

// Motors
MotorController motor_L(pin_motor_L);
MotorController motor_R(pin_motor_R);

// Encoders
Encoder encoder_L;
Encoder encoder_R;

// Funções de incremento dos encoders
void IncrementCounter_encoder_L() { encoder_L.IncrementCounter(); }
void IncrementCounter_encoder_R() { encoder_R.IncrementCounter(); }

// // Variáveis de tempo
unsigned long t_now   = millis();
unsigned long t_last  = t_now;
unsigned long dt      = 0;
std_msgs::Int32 msg_time;

// Cinemática
Kinematics            kinematics;
Odometry              odometry;
geometry_msgs::Twist  msg_cmd_vel;
nav_msgs::Odometry    msg_odom;

// Nodo ROS
ros::NodeHandle nh; // Consome mta memória se tem mtos publishers e subscribers

// Pra saber se recebeu uma nova msg de velocidade
bool new_cmd_vel = false;

// Funções de callback (chamadas pelo ROS quando um subscriber tem msg nova)
void callback_cmd_vel(const geometry_msgs::Twist& msg)
{
  new_cmd_vel = true;
  msg_cmd_vel = msg;
}

// Publishers
ros::Publisher pub_time("/mc/time", &msg_time);
// ros::Publisher pub_odom("/odom", &msg_odom);

// Subscribers
ros::Subscriber <geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", &callback_cmd_vel);

void setup()
{
  // Inicia o rosserial - conflita com o serial normal!
  nh.getHardware()->setBaud(serialRate);
  nh.initNode();
  Serial.begin(serialRate);

  // Cadastra os publishers e subscribers
  nh.advertise(pub_time);
  // nh.advertise(pub_odom);
  nh.subscribe(sub_cmd_vel);

  // Setup dos encoders
  attachInterrupt(digitalPinToInterrupt(pin_data_encoder_L), IncrementCounter_encoder_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_data_encoder_R), IncrementCounter_encoder_R, CHANGE);
  encoder_L.Setup(pin_5V_encoder_L, pin_data_encoder_L);
  encoder_R.Setup(pin_5V_encoder_R, pin_data_encoder_R);

  // // Setup dos motores
  motor_L.Setup();
  motor_R.Setup();
}

void loop()
{
  delay(t_delay);
  nh.spinOnce(); // "F5"

  UpdateTimeVariables();

  motor_L.SetTargetRPM(kinematics.Inverse_L(msg_cmd_vel.linear.x, msg_cmd_vel.angular.z, wheelRadius, wheelsAxisLength));  
  encoder_L.Update();
  motor_L.UpdatePID(encoder_L.GetRPM(), kp, ki, kd);
  motor_L.UpdateSpeed();

  // UpdateOdom();
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

// Se tem nova mensagem de velocidade, calcula valores de RPM desejados (cinemática inversa)
void UpdateTargetRPM()
{
  if (new_cmd_vel == true) {
    new_cmd_vel = false;
    motor_L.SetTargetRPM(kinematics.Inverse_L(msg_cmd_vel.linear.x, msg_cmd_vel.angular.z, wheelRadius, wheelsAxisLength));
    motor_R.SetTargetRPM(kinematics.Inverse_R(msg_cmd_vel.linear.x, msg_cmd_vel.angular.z, wheelRadius, wheelsAxisLength));
  }
}

// // Se o encoder tem uma nova leitura, atualiza a velocidade do motor
// void UpdateMotors()
// {
//   if (encoder_L.Update() == true) {
//     motor_L.UpdatePID(encoder_L.GetRPM(), kp, ki, kd);
//     motor_L.UpdateSpeed();
//   }
//   if (encoder_R.Update() == true) {
//     motor_R.UpdatePID(encoder_R.GetRPM(), kp, ki, kd);
//     motor_R.UpdateSpeed();
//   }
// }

// // Atualiza odometria (cinemática direta)
// void UpdateOdom()
// {
//   msg_odom.twist.twist.linear.x   = kinematics.Direct_linear(encoder_L.GetRPM(), encoder_R.GetRPM(), wheelRadius, wheelsAxisLength);
//   msg_odom.twist.twist.angular.z  = kinematics.Direct_angular(encoder_L.GetRPM(), encoder_R.GetRPM(), wheelRadius, wheelsAxisLength);
//   odometry.UpdateOdom(msg_odom.twist.twist.linear.x, msg_odom.twist.twist.angular.z);
//   msg_odom.pose.pose.position.x = odometry.x;
//   msg_odom.pose.pose.position.y = odometry.y;
//   msg_odom.pose.pose.orientation = odometry.RpyToQuaternion(odometry.theta, 0, 0);
//   // pub_odom.publish(&msg_odom);
// }