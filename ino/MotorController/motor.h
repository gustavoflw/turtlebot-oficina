#ifndef MOTOR_H
#define MOTOR_H

#include <AFMotor.h> // Adafruit Motor shield library (af_motor)

/* Ativa o motor
    - wheelSpeed tem que estar entre [-255, 255]
    - o sinal negativo define o sentido da rotacao */
void activateMotor(AF_DCMotor* motor, int wheelSpeed)
{
  // Corrige os limites de wheelSpeed (máximo 255 e mínimo -255)
  if (wheelSpeed > 255)
    wheelSpeed = 255;
  else if (wheelSpeed < -255)
    wheelSpeed = -255;
  
  // Módulo de wheelSpeed
  int positiveWheelSpeed = wheelSpeed;

  // Corrige o sinal
  if (positiveWheelSpeed < 0)
    positiveWheelSpeed = -positiveWheelSpeed;

  // Determina o sentido da rotacao (frente, tras ou parado)
  if (wheelSpeed > 0)
    motor->run(FORWARD);
  else if (wheelSpeed < 0)
    motor->run(BACKWARD);
  else
    motor->run(RELEASE);

  // Coloca o motor na velocidade desejada (entre 0 e 255)
  motor->setSpeed(positiveWheelSpeed);
}

#endif
