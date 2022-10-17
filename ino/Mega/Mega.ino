#include <ros.h>                  // Rosserial (usar 0.7.9)
#include <std_msgs/Int32.h>       // Msg do ROS para int

#include <std_msgs/Float64.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

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
#define t_delay             50
#define serialRate          57600
#define wheelRadius         3.3
#define wheelsAxisLength    15.6 
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

// Variáveis de tempo
unsigned long t_now   = millis();
unsigned long t_last  = t_now;
unsigned long dt      = 0;
std_msgs::Int32 msg_time;

// Cinemática
Kinematics            kinematics;
Odometry              odometry;
std_msgs::Float64     msg_rpmL;
std_msgs::Float64     msg_rpmR;
std_msgs::Float64     msg_odom_theta;
geometry_msgs::Twist  msg_cmd_vel;
geometry_msgs::Twist  msg_odom_twist;
geometry_msgs::Pose   msg_odom_pose;

// Nodo ROS
ros::NodeHandle nh; // Consome mta memória se tem mtos publishers e subscribers

// Pra saber se recebeu uma nova msg de velocidade
bool new_cmd_vel = false;

// Funções de callback (chamadas pelo ROS quando um subscriber tem msg nova)
void callback_cmd_vel(const geometry_msgs::Twist& msg)
{
  new_cmd_vel = true;
  msg_cmd_vel = msg;

  if (fabs(msg_cmd_vel.linear.x) < 0.1)
    msg_cmd_vel.linear.x = 0.0; 
  if (fabs(msg_cmd_vel.angular.z) < 0.1)
    msg_cmd_vel.angular.z = 0.0;
}



// Publishers
ros::Publisher pub_time("/mc/time", &msg_time);
ros::Publisher pub_rpmL("/rpm/l", &msg_rpmL);
ros::Publisher pub_rpmR("/rpm/r", &msg_rpmR);
ros::Publisher pub_odom_theta("/odom/theta", &msg_odom_theta);
ros::Publisher pub_odom_twist("/odom/twist", &msg_odom_twist);
ros::Publisher pub_odom_pose("/odom/pose", &msg_odom_pose);

// Subscribers
ros::Subscriber <geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", &callback_cmd_vel);

void setup()
{
  // Inicia o rosserial - conflita com o serial normal!
  // nh.getHardware()->setBaud(serialRate);
  nh.initNode();
  // Serial.begin(serialRate);

  // TESTES (apagar dps)
  pinMode(53, OUTPUT);
  digitalWrite(53, LOW);

  // Cadastra os publishers e subscribers
  nh.advertise(pub_rpmL);
  nh.advertise(pub_rpmR);
  nh.advertise(pub_odom_theta);
  nh.advertise(pub_odom_twist);
  nh.advertise(pub_odom_pose);
  nh.advertise(pub_time);
  nh.subscribe(sub_cmd_vel);

  // Setup dos encoders
  attachInterrupt(digitalPinToInterrupt(pin_data_encoder_L), IncrementCounter_encoder_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_data_encoder_R), IncrementCounter_encoder_R, CHANGE);
  encoder_L.Setup(pin_5V_encoder_L, pin_data_encoder_L);
  encoder_R.Setup(pin_5V_encoder_R, pin_data_encoder_R);

  // Setup dos motores
  motor_L.Setup();
  motor_R.Setup();
}

void loop()
{
  delay(5); 
  nh.spinOnce();
  
  if (IsAllowedToLoop(t_last, t_delay)) {
    UpdateTimeVariables();

    nh.spinOnce(); // "F5"    

    UpdateMotors();

    UpdateOdom();
  }
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

// Se tem nova mensagem de velocidade, calcula valores de RPM desejados (cinemática inversa)
void UpdateTargetRPM()
{
  if (new_cmd_vel == true) {
    new_cmd_vel = false;
    motor_L.SetTargetRPM(kinematics.Inverse_L(msg_cmd_vel.linear.x, msg_cmd_vel.angular.z, wheelRadius, wheelsAxisLength));
    motor_R.SetTargetRPM(kinematics.Inverse_R(msg_cmd_vel.linear.x, msg_cmd_vel.angular.z, wheelRadius, wheelsAxisLength));
  }
}

// Se o encoder tem uma nova leitura, atualiza a velocidade do motor
void UpdateMotors()
{
  motor_L.SetTargetRPM(kinematics.Inverse_L(msg_cmd_vel.linear.x, msg_cmd_vel.angular.z, wheelRadius, wheelsAxisLength));
  encoder_L.Update(motor_L.status);
  motor_L.UpdatePID(encoder_L.GetRPM(), kp, ki, kd);
  motor_L.UpdateSpeed();
  msg_rpmL.data = encoder_L.GetRPM();
  pub_rpmL.publish(&msg_rpmL);

  motor_R.SetTargetRPM(kinematics.Inverse_R(msg_cmd_vel.linear.x, msg_cmd_vel.angular.z, wheelRadius, wheelsAxisLength));
  encoder_R.Update(motor_R.status);
  motor_R.UpdatePID(encoder_R.GetRPM(), kp, ki, kd);
  motor_R.UpdateSpeed();
  msg_rpmR.data = encoder_R.GetRPM();
  pub_rpmR.publish(&msg_rpmR);
}

// Atualiza odometria (cinemática direta)
void UpdateOdom()
{
  msg_odom_twist.linear.x = kinematics.Direct_linear(encoder_L.GetRPM(), encoder_R.GetRPM(), wheelRadius, wheelsAxisLength);
  msg_odom_twist.angular.z  = kinematics.Direct_angular(encoder_L.GetRPM(), encoder_R.GetRPM(), wheelRadius, wheelsAxisLength);
  odometry.UpdateOdom(msg_odom_twist.linear.x, msg_odom_twist.angular.z);
  msg_odom_pose.position.x = odometry.x;
  msg_odom_pose.position.y = odometry.y;
  msg_odom_pose.orientation = odometry.RpyToQuaternion(odometry.theta, 0, 0);
  msg_odom_theta.data = odometry.theta;

  pub_odom_theta.publish(&msg_odom_theta);
  pub_odom_twist.publish(&msg_odom_twist);
  pub_odom_pose.publish(&msg_odom_pose);
}