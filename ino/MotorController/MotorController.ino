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
#define pinMotorL         1     // Pino da ponte H (não é pino analógico do arduino)
#define pinMotorR         2     // Pino da ponte H (não é pino analógico do arduino)
#define pinEncoderL       3     // Pino do encoder
#define pinEncoderR       2     // Pino do enconder
#define buracosEncoder    20    // Quantidade de buracos do encoder

// Encoder
volatile int contL=0;
volatile int contR=0;

void IntL()
{
  contL++;
}

void IntR()
{
  contR++;
}

// Motor
AF_DCMotor leftMotor  (pinMotorL);
AF_DCMotor rightMotor (pinMotorR);

// Nodo ROS
ros::NodeHandle nh; // Come mta memória!!!
//ros::NodeHandle_<ArduinoHardware, 2, 5, 80, 200> nh;

// Velocidades
float v_x             = 0.0;  // v. linear do robô
float w_z             = 0.0;  // v. angular do robô
float targetLeftRPM   = 0.0;  // RPM desejado (esquerda)
float targetRightRPM  = 0.0;  // RPM desejado (direita)

// Posicao, orientacao
float x               = 0.0;  // Posicao em x
float y               = 0.0;  // Posicao em y
float theta           = 0.0;  // Angulo em torno de z

// Tempo
unsigned long t_now   = millis();
unsigned long t_last  = t_now;
unsigned long d_t     = t_now - t_last;

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
ros::Publisher      pub_time            ("/mc/time",                      &msg_time);
ros::Publisher      pub_odom_v_x        ("/mc/odom/v/x",                  &msg_odom_v_x);
ros::Publisher      pub_odom_w_z        ("/mc/odom/w/z",                  &msg_odom_w_z);
ros::Publisher      pub_encoderLeftRPM  ("/mc/motor/left/encoder/rpm",    &msg_encoderLeftRPM);
ros::Publisher      pub_encoderRightRPM ("/mc/motor/right/encoder/rpm",   &msg_encoderRightRPM);

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

  // Cadastra os subscribers no nodo
  nh.subscribe(sub_v_x);
  nh.subscribe(sub_w_z);

  // Encoders
  pinMode(pinEncoderL, INPUT);
  pinMode(pinEncoderR, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinEncoderL), IntL, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncoderR), IntR, RISING);
}

void loop()
{
  // Atualiza variáveis de tempo
  t_last  = t_now;
  t_now   = millis();
  d_t     = t_now - t_last;
  msg_time.data = t_now;
  pub_time.publish(&msg_time);
  
  delay(delayTime);
  nh.spinOnce();  // "F5"  

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

  // Lê os encoders
  detachInterrupt(digitalPinToInterrupt(pinEncoderL));
  detachInterrupt(digitalPinToInterrupt(pinEncoderR));
  msg_encoderLeftRPM.data   = contL*60/buracosEncoder;
  msg_encoderRightRPM.data  = contR*60/buracosEncoder;
  contL                     = 0;
  contR                     = 0;
  attachInterrupt(digitalPinToInterrupt(pinEncoderL), IntL, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncoderR), IntR, RISING);

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

  // Atualiza odometria
  updateOdometry(v_x, w_z, d_t, &x, &y, &theta);

  // Publica mensagens
  pub_encoderLeftRPM.publish  (&msg_encoderLeftRPM);
  pub_encoderRightRPM.publish (&msg_encoderRightRPM);
  pub_odom_v_x.publish        (&msg_odom_v_x);
  pub_odom_w_z.publish        (&msg_odom_w_z);
}
