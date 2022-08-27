#include <ros.h>              // Rosserial (usar 0.7.9)
#include <std_msgs/Int32.h>   // Mensagem do ROS pra int
#include <std_msgs/Float32.h> // Mensagem do ROS pra float

#include "kinematics.h"
#include "encoder.h"
#include "motor.h"

// Constantes
#define delayTime         50    // Milissegundos
#define serialRate        9600  // Frequência do serial
#define Kp                1.03  // Constante de proporcao do erro de RPM (calibrado empiricamente)
#define maxRPM            200   // RPM máximo em 6V
#define wheelRadius       4.0   // Raio das rodas
#define wheelsAxisLength  10.0  // Comprimento do eixo das rodas
#define pinMotorL         1     // Pino da ponte H
#define pinMotorR         2     // Pino da ponte H

// Motor
AF_DCMotor leftMotor  (pinMotorL);
AF_DCMotor rightMotor (pinMotorR);

// Nodo ROS
ros::NodeHandle nh;
//ros::NodeHandle_<ArduinoHardware, 2, 5, 80, 200> nh;

// Velocidades
float v_x             = 0.0; // v. linear do robô
float w_z             = 0.0; // v. angular do robô
float targetLeftRPM   = 0.0; // RPM desejado (esquerda)
float targetRightRPM  = 0.0; // RPM desejado (direita)

// Pra saber se recebeu uma nova msg
bool newMsg = false;

// Funções de callback (são chamadas pelo ROS quando um subscriber tem mensagem nova)

void callback_v_x(const std_msgs::Float32& msg)
{
  newMsg = true;
  v_x = msg.data;
}

void callback_w_z(const std_msgs::Float32& msg)
{
  newMsg = true;
  w_z = msg.data;
}

// Publishers
std_msgs::Int32     msg_time;
std_msgs::Float32   msg_odom_v_x;
std_msgs::Float32   msg_odom_w_z;
std_msgs::Float32   msg_encoderLeftRPM;
std_msgs::Float32   msg_encoderRightRPM;
ros::Publisher      pub_time            ("/motor/time",         &msg_time);
ros::Publisher      pub_odom_v_x        ("/odom_v_x",           &msg_odom_v_x);
ros::Publisher      pub_odom_w_z        ("/odom_w_z",           &msg_odom_w_z);
ros::Publisher      pub_encoderLeftRPM  ("/encoder_left_rpm",   &msg_encoderLeftRPM);
ros::Publisher      pub_encoderRightRPM ("/encoder_right_rpm",  &msg_encoderRightRPM);

// Subscribers
ros::Subscriber<std_msgs::Float32> sub_v_x("/cmd_vel/linear/x",   &callback_v_x);
ros::Subscriber<std_msgs::Float32> sub_w_z("/cmd_vel/angular/z",  &callback_w_z);

void setup()
{
  // Inicia o nodo serial (se tentar usar o monitor serial o rosserial no pc nao vai conseguir ler nada!)
  nh.getHardware()->setBaud(serialRate);
  nh.initNode();

  // Cadastra os publishers no nodo
  nh.advertise(pub_time);
  nh.advertise(pub_odom_v_x);
  nh.advertise(pub_odom_w_z);
  nh.advertise(pub_encoderLeftRPM);
  nh.advertise(pub_encoderRightRPM);

  // Cadastra os subscribers no nodo/
  nh.subscribe(sub_v_x);
  nh.subscribe(sub_w_z);
}

void loop()
{
  delay(delayTime);
  nh.spinOnce();  // "F5"

  // Pega o tempo de execucao do arduino e publica
  msg_time.data = millis();
  pub_time.publish(&msg_time);

  // TESTES - APAGAR DEPOIS
  v_x     = 5.0;
  w_z     = 0.0;
  newMsg  = true;

  // Se tem nova mensagem de velocidade, calcula valores de RPM desejados
  if (newMsg == true) {
    newMsg = false;
    targetLeftRPM     = getLeftWheelRPM (v_x, w_z, wheelRadius, wheelsAxisLength);
    targetRightRPM    = getRightWheelRPM(v_x, w_z, wheelRadius, wheelsAxisLength);
  }

  // Lê os encoders [PENDENTE]
  msg_encoderLeftRPM.data   = 0.0;
  msg_encoderRightRPM.data  = 0.0;

  // Calcula erro de RPM (velocidade desejada - velocidade lida)
  float errorLeftRPM  = targetLeftRPM   - msg_encoderLeftRPM.data;
  float errorRightRPM = targetRightRPM  - msg_encoderRightRPM.data;

  // Calcula velocidade de cada motor proporcional ao erro
  float leftRPM   = errorLeftRPM  * Kp;
  float rightRPM  = errorRightRPM * Kp;

  // Calcula a proporcao dos RPM de cada motor em relacao ao RPM maximo do motor
  float proportionalLeftRPM   = leftRPM   / maxRPM;
  float proportionalRightRPM  = rightRPM  / maxRPM;

  // Coloca as proporcoes de RPM na escala de [0, 255]
  float leftWheelSpeed  = proportionalLeftRPM   * 255.0;
  float rightWheelSpeed = proportionalRightRPM  * 255.0;

  // Ativa os motores
  activateMotor(&leftMotor,   (int)leftWheelSpeed);
  activateMotor(&rightMotor,  (int)rightWheelSpeed);

  // Cinemática direta: calcula v. linear e angular a partir dos RPM lido pelos encoders
  msg_odom_v_x.data = getLinearSpeed  (msg_encoderLeftRPM.data, msg_encoderRightRPM.data, wheelRadius, wheelsAxisLength);
  msg_odom_w_z.data = getAngularSpeed (msg_encoderLeftRPM.data, msg_encoderRightRPM.data, wheelRadius, wheelsAxisLength);

  // Publica mensagens
  pub_encoderLeftRPM.publish  (&msg_encoderLeftRPM);
  pub_encoderRightRPM.publish (&msg_encoderRightRPM);
  pub_odom_v_x.publish        (&msg_odom_v_x);
  pub_odom_w_z.publish        (&msg_odom_w_z);
}
