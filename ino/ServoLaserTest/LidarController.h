#ifndef LIDARCONTROLLER_H
#define LIDARCONTROLLER_H

#define ANGLE_RISING 0
#define ANGLE_FALLING 90

#include <Servo.h>
#include <Adafruit_VL53L0X.h>

class LidarController
{
  // Ângulo
  int angle_now = 0;
  int angle_target = 0;
  int angle_status = RISING;
  int angle_increment = 1;
  int angle_min = 0;
  int angle_max = 180;

  // Distância
  double range_now = 0.0;

  // Tempo
  unsigned long t = millis();
  unsigned long dt_update = 10;

  // Servo
  Servo servo;

  // Laser
  Adafruit_VL53L0X lox = Adafruit_VL53L0X();

  public:

    /* Setup
      - Faz o servo começar em 0
      - Define o incremento */
    void Setup(int pin, int _angle_increment)
    {
      servo.attach(pin);
      angle_increment = _angle_increment;
      servo.write(0);

      if (!lox.begin())
        delay(5);

    }

    /* Retorna o incremento */
    int GetIncrement() 
    {
      return angle_increment;
    }

    /* Retorna o ângulo atual (última vez medido) */
    int GetAngle()
    {
      return angle_now;
    }

    /* Retorna a distância atual (última vez medida) */
    double GetRange()
    {
      return range_now;
    }

    /* Retorna true se angle_now == angle_target */
    bool IsAngleCorrect()
    {
      if (angle_now == angle_target)
        return true;
      else
        return false;
    }

    /* Atualiza qual é o próximo ângulo */
    void UpdateAngleTarget()
    {
      // Angulo aumentando
      if (angle_status == RISING) {
        if (servo.read() == angle_max)
          angle_status = FALLING;
        else
          angle_target = angle_target + angle_increment;
      }

      // Angulo diminuindo
      else if (angle_status == FALLING) {
        if (servo.read() == angle_min)
          angle_status = RISING;
        else
          angle_target = angle_target - angle_increment;
      }

      // Precaução
      angle_target = max(angle_target, angle_min);
      angle_target = min(angle_target, angle_max);
    }

    /* Atualiza a distância */
    void UpdateRange()
    {
      VL53L0X_RangingMeasurementData_t measure;
      lox.rangingTest(&measure, false);
      if (measure.RangeStatus != 4)
        range_now = ((double)measure.RangeMilliMeter) * 1.0 / 1000;
    }

    void Update()
    {
      unsigned long t_now = millis();
      unsigned long dt = t_now - t;
      if (dt < dt_update)
        return;

      // Atualiza ângulo atual
      angle_now = servo.read();

      // Ângulo atual já está no desejado -> determina a próxima posição
      if (IsAngleCorrect() == true) {
        UpdateAngleTarget();
        UpdateRange();
      }
      // Ângulo atual NÃO está no desejado -> vai até lá
      else {
        servo.write(angle_target);
      }
    }
};

#endif





// class LaserController 
// {
//   Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//   public:

//     void Setup(float increment, sensor_msgs::LaserScan* msg_scan)
//     {
//       msg_scan->header.frame_id = "scan_link";
//       msg_scan->angle_min = 0; // [rad]
//       msg_scan->angle_max = PI; // [rad]
//       msg_scan->angle_increment = increment; // [rad]
//       // msg_scan->time_increment = 0.250; // [s]
//       // msg_scan->scan_time = 0.5; // [s]
//       msg_scan->range_min = 0; // [m]
//       msg_scan->range_max = 20; // [m]

//       // msg_scan->ranges[2] = {0};
//       float ranges[1] = {0};
//       msg_scan->ranges = ranges;
//       msg_scan->ranges_length = 1;

//       for (int i=0; i < msg_scan->ranges_length; i++) {
//         msg_scan->ranges[i] = 0.0;
//       }

//       // Serial.println("Setup laser...");
//       if (!lox.begin()) {
//         // Serial.println("Laser???????");
//         delay(5);
//       }
        
//     }
//     void Update(int angle, sensor_msgs::LaserScan* msg_scan)
//     {
//       VL53L0X_RangingMeasurementData_t measure;
//       lox.rangingTest(&measure, false);
//       double range = 0.0;
//       if (measure.RangeStatus != 4) {
//         range = ((double)measure.RangeMilliMeter) * 0.8 / 1000;
//       } 
//     }
// };