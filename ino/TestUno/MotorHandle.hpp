#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <AFMotor.h> // Adafruit Motor shield library (af_motor)

/* Classe para motor */
class MotorHandle {
  // Variáveis de RPM
  double rpm_now         = 0;
  double rpm_last        = 0;
  double rpm_target      = 0;
  double rpm_error_now   = 0;
  double rpm_error_last  = 0;

  // Variáveis de velocidade do motor
  int speed_now           = 0;
  int speed_last          = 0;
  
  // Fator de correção da velocidade (output do PID)
  double fspeed_correction = 0.0;

  double output_i = 0.0;

  // Variáveis de tempo (milissegundos)
  unsigned long t_last = 0;
  unsigned long t_update_interval = 100;
  
  public:
    // Objeto do motor
    AF_DCMotor motor;

    // Inicialização
    MotorHandle(int pin) : motor(pin) 
    {
      // Nada
    }

    // Atualiza as variáveis do PID
    void UpdatePID(double _rpm_now, double kp, double ki, double kd)
    {
      unsigned long t_now = millis();
      double dt = t_now - t_last;
      t_last = t_now;
      
      rpm_last = rpm_now;
      rpm_now = _rpm_now;
      rpm_error_last = rpm_error_now;
      rpm_error_now = rpm_target - rpm_now;
      
      double output_p = kp * rpm_error_now;
      double output_d = kd * (rpm_error_now - rpm_error_last) / (dt*1000.0*60.0);
      output_i = ki * (output_i + (rpm_error_now * dt));
      fspeed_correction = output_p + output_d + output_i;
      speed_now = speed_now + fspeed_correction;

      speed_now = min(speed_now, 255);
      speed_now = max(speed_now, 60);

      Log();
    }

    // Atualiza a velocidade do motor
    void UpdateSpeed()
    {
      if (speed_now != speed_last) {
        speed_last = speed_now;
        motor.setSpeed(speed_now);
      }
    }

    // Define o RPM alvo
    void SetTargetRPM(float target) 
    {
      rpm_target = target;
    }

    // Printa log
    void Log()
    {
      
      Serial.print("rpm_now:");
      Serial.print(rpm_now);
      Serial.print(",");
      Serial.print("rpm_target:");
      Serial.print(rpm_target);
      Serial.print(",");
      // Serial.print("t_update_interval: ");
      // Serial.print(t_update_interval);
      // Serial.print(",");
      // Serial.print("rpm_last:");
      // Serial.print(rpm_last);
      // Serial.print(",");
      // Serial.print("rpm_target:");
      // Serial.print(rpm_target);
      // Serial.print(",");
      // Serial.print("rpm_error_now:");
      // Serial.print(rpm_error_now);
      // Serial.print(",");
      // Serial.print("rpm_error_last:");
      // Serial.print(rpm_error_last);
      // Serial.print(",");
      // Serial.print("fspeed_correction:");
      // Serial.print(fspeed_correction);
      // Serial.print(",");
      Serial.print("speed_now:");
      Serial.println(speed_now);
    }
};

#endif
