#ifndef KINEMATICS_H
#define KINEMATICS_H

// --------- CINEMATICA INVERSA (usado para ativar os motores) --------------------------------
float getLeftWheelRPM(float robotLinearSpeed, float robotAngularSpeed, const float wheelRadius, const float wheelsAxisLength)
{
  // [PENDENTE]
}

float getRightWheelRPM(float robotLinearSpeed, float robotAngularSpeed, const float wheelRadius, const float wheelsAxisLength)
{
 // Giro em rad/s
 float radPerS = (robotAngularSpeed * wheelsAxisLength / (2*wheelRadius)) + (robotLinearSpeed/wheelRadius);

 // Giro em rps (rotacao por segundo)
 float rps = radPerS/ (2 * PI);

 // Giro em rpm
 float rpm = rps * 60.0;

 return rpm;
}
// --------------------------------------------------------------------------------------------

// --------- CINEMATICA DIRETA (usado pra estimar a velocidade do robô) -----------------------
float getLinearSpeed(float leftWheelRPM, float rightWheelRPM, const float wheelRadius, const float wheelsAxisLength)
{
  // [PENDENTE]
}

float getAngularSpeed(float leftWheelRPM, float rightWheelRPM, const float wheelRadius, const float wheelsAxisLength)
{
  // [PENDENTE]
}
// --------------------------------------------------------------------------------------------

// --------- ODOMETRIA ------------------------------------------------------------------------
void updateOdometry(float v_x, float w_z, unsigned long d_t, float* x, float* y, float* theta)
{
  // Calcula o quanto girou:
  float d_theta = w_z * d_t;
  
  // Calcula o quanto andou:
  float d_x = sqrt(v_x * v_x)*cos(d_theta) * d_t;
  float d_y = sqrt(v_x * v_x)*sin(d_theta) * d_t;

  // Integrando, calcula o novo valor (soma do antigo mais a nova variação)
  *x = *x + d_x;
  *y = *y + d_y;
  *theta = *theta + d_theta;
}

#endif
