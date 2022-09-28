#include "MotorHandle.hpp"
#include "EncoderHandle.hpp"

#define kp 0.3
#define ki 0.0
#define kd 0.0
#define motorUpdateInterval 250
#define pinEncoderL       20     // Pino do enconder
#define pinEncoderR       19     // Pino do enconder
#define numEncoderHoles   20
#define pinMotorR 3
#define pinMotorL 4

EncoderHandle encoderL;
EncoderHandle encoderR;
MotorHandle motorL(pinMotorL);
MotorHandle motorR(pinMotorR);

void IncrementCounter_encoderL()
{
  encoderL.IncrementCounter();
}
void IncrementCounter_encoderR()
{
  encoderR.IncrementCounter();
}

void setup() {
  Serial.begin(57600);

  // Alimentacao 5V
  pinMode(28, OUTPUT);
  pinMode(30, OUTPUT);
  digitalWrite(28, HIGH);
  digitalWrite(30, HIGH);
  
  pinMode(pinEncoderL, INPUT_PULLUP);
  pinMode(pinEncoderR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncoderL), IncrementCounter_encoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderR), IncrementCounter_encoderR, CHANGE);

  motorL.motor.run(FORWARD);
  motorL.motor.setSpeed(0);
  motorL.SetTargetRPM(60);

  motorR.motor.run(FORWARD);
  motorR.motor.setSpeed(0);
  motorR.SetTargetRPM(60);

  delay(25);
  
  encoderL.ResetCounter();
  encoderR.ResetCounter();
}

void loop() {
  delay(30);

  // Lento
  if (encoderL.Update() == true) {
    
  }
  motorL.UpdatePID(encoderL.GetRPM(), kp, ki, kd);
  motorL.UpdateSpeed();

  // // Lento
  // if (encoderR.Update() == true) {
    
  // }
  // motorR.UpdatePID(encoderR.GetRPM(), kp, ki, kd);
  // motorR.UpdateSpeed();

  // Rápido
  // encoderL.Update();
  // motorL.UpdatePID(encoderL.GetRPM(), kp, ki, kd);
  // motorL.UpdateSpeed();

  // encoderR.Update();
  // motorR.Update(encoderR.GetRPM());
}
