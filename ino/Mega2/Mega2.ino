#include <ros.h>

#include "Encoder.hpp"
#include "MotorController.hpp"
#include "Kinematics.hpp"
#include "Odometry.hpp"
#include "SerialCom.hpp"
#include "LidarController.hpp"

/* CONSTANTES */
#define pin_motor_L         4
#define pin_motor_R         3
#define pin_data_encoder_L  43
#define pin_data_encoder_R  42
#define pin_servo           10
#define angle_increment     1
#define t_delay             50
#define serialRate          57600
#define wheelRadius         3.3
#define wheelsAxisLength    15.6 
#define kp                  0.3
#define ki                  0.0
#define kd                  0.0

// /* MOTORS */
// MotorController motor_L(pin_motor_L);
// MotorController motor_R(pin_motor_R);

// /* ENCODERS */
// Encoder encoder_L;
// Encoder encoder_R;
// void IncrementCounter_encoder_L() { encoder_L.IncrementCounter(); }
// void IncrementCounter_encoder_R() { encoder_R.IncrementCounter(); }

/* SERIAL */
SerialCom serialCom;

/* TEMPO */
unsigned long t_now   = millis();
unsigned long t_last  = t_now;
unsigned long dt      = 0;

/* CINEMÁTICA */
Kinematics kinematics;
Odometry odometry;
float rpmL = 0, rpmR = 0;
double cmd_v_x = 0, cmd_w_z = 0;
double odom_x = 0, odom_y = 0, odom_theta = 0; 
double odom_v_x = 0, odom_w_z = 0;
double odom_orientation_x = 0, odom_orientation_y = 0, odom_orientation_z = 0, odom_orientation_w = 0;

/* LASER */
double scan_range = 0;
int scan_angle = 0;
LidarController lidar;

/***** SETUP *****/
void setup()
{
  // Serial
  Serial.begin(serialRate);
  Serial.println("Hello");

  // Setup do lidar
  lidar.Setup(pin_servo, angle_increment);

  // // Setup dos encoders
  // attachInterrupt(digitalPinToInterrupt(pin_data_encoder_L), IncrementCounter_encoder_L, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(pin_data_encoder_R), IncrementCounter_encoder_R, CHANGE);
  // encoder_L.Setup(pin_data_encoder_L);
  // encoder_R.Setup(pin_data_encoder_R);

  // // Setup dos motores
  // motor_L.Setup();
  // motor_R.Setup();

  Serial.println("Setup OK");
}

/***** LOOP *****/
void loop()
{
  delay(5);
  
  if (IsAllowedToLoop(t_last, t_delay)) {
    UpdateTimeVariables();
    // UpdateSerial();
    UpdateLidar();
    // UpdateTargetRPM();
    // UpdateMotors();
    // UpdateOdom();
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
}

// // Se tem nova mensagem de velocidade, calcula valores de RPM desejados (cinemática inversa)
// void UpdateTargetRPM()
// {
//   motor_L.SetTargetRPM(kinematics.Inverse_L(cmd_v_x, cmd_w_z, wheelRadius, wheelsAxisLength));
//   motor_R.SetTargetRPM(kinematics.Inverse_R(cmd_v_x, cmd_w_z, wheelRadius, wheelsAxisLength));
// }

// Atualiza serial
void UpdateSerial()
{
  serialCom.SendInfo(
      odom_x, odom_y, odom_theta, 
      odom_v_x, odom_w_z,
      odom_orientation_x, odom_orientation_y, odom_orientation_z, odom_orientation_w,
      rpmL, rpmR,
      scan_angle, scan_range);
}

// Atualiza lidar
void UpdateLidar()
{
  lidar.Update();
  scan_angle = lidar.GetAngle();
  scan_range = lidar.GetRange();
  Serial.println(scan_range);
}

// // Se o encoder tem uma nova leitura, atualiza a velocidade do motor
// void UpdateMotors()
// {
//   motor_L.SetTargetRPM(kinematics.Inverse_L(cmd_v_x, cmd_w_z, wheelRadius, wheelsAxisLength));
//   encoder_L.Update(motor_L.status);
//   motor_L.UpdatePID(encoder_L.GetRPM(), kp, ki, kd);
//   motor_L.UpdateSpeed();
//   rpmL = encoder_L.GetRPM();

//   motor_R.SetTargetRPM(kinematics.Inverse_R(cmd_v_x, cmd_w_z, wheelRadius, wheelsAxisLength));
//   encoder_R.Update(motor_R.status);
//   motor_R.UpdatePID(encoder_R.GetRPM(), kp, ki, kd);
//   motor_R.UpdateSpeed();
//   rpmR = encoder_R.GetRPM();
// }

// // Atualiza odometria (cinemática direta)
// void UpdateOdom()
// {
//   odom_v_x = kinematics.Direct_linear(encoder_L.GetRPM(), encoder_R.GetRPM(), wheelRadius, wheelsAxisLength);
//   odom_w_z  = kinematics.Direct_angular(encoder_L.GetRPM(), encoder_R.GetRPM(), wheelRadius, wheelsAxisLength);
//   odometry.UpdateOdom(odom_v_x, odom_w_z);
//   odom_x = odometry.x;
//   odom_y = odometry.y;
//   odom_theta = odometry.theta;
//   geometry_msgs::Quaternion orientation = odometry.RpyToQuaternion(odometry.theta, 0, 0);
//   odom_orientation_x = orientation.x;
//   odom_orientation_y = orientation.y;
//   odom_orientation_z = orientation.z;
//   odom_orientation_w = orientation.w;
// }