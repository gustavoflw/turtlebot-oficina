#include <Servo.h>
#include <ros.h>
#include <Adafruit_VL53L0X.h>
#include <sensor_msgs/LaserScan.h>

#define RISING 0
#define FALLING 90

int servoPin = 3;

class ServoController
{
  int angle = 0;
  int angleStatus = RISING;
  int increment = 5;

  unsigned long t = millis();
  unsigned long dt_update = 250;

  Servo servo;

  public:
    void Setup(int pin, int _increment)
    {
      servo.attach(pin);
      increment = _increment;
      servo.write(0);
    }
    int GetIncrement() 
    {
      return increment;
    }
    int GetAngle()
    {
      return angle;
    }
    void ChangeAngle()
    {
      unsigned long t_now = millis();

      if (t_now - t > dt_update) {
        t = t_now;

        angle = servo.read();

        if (angleStatus == RISING) {
          if (servo.read() == 180)
            angleStatus = FALLING;
          else
            angle = angle + increment;
        }
        else if (angleStatus == FALLING) {
          if (servo.read() == 0)
            angleStatus = RISING;
          else
            angle = angle - increment;
        }

        angle = max(angle, 0);
        angle = min(angle, 180);

        // Serial.print("angle:");
        // Serial.print(angle);
        // Serial.print(",");
        // Serial.print("angleStatus:");
        // Serial.print(angleStatus);
        // Serial.println();

        servo.write(angle);
      }

      
    }
};

class LaserController 
{
  sensor_msgs::LaserScan msg_scan;
  Adafruit_VL53L0X lox = Adafruit_VL53L0X();

  public:
    void Setup(float increment)
    {
      msg_scan.angle_min = 0; // [rad]
      msg_scan.angle_max = PI; // [rad]
      msg_scan.angle_increment = increment; // [rad]
      msg_scan.time_increment = 0.250; // [s]
      msg_scan.scan_time = 0.5; // [s]
      msg_scan.range_min = 0.03; // [m]
      msg_scan.range_max = 2; // [m]
      msg_scan.ranges[180] = {0}; // [m]

      if (!lox.begin())
        delay(5);
    }
    void Update(int angle)
    {
      VL53L0X_RangingMeasurementData_t measure;
      lox.rangingTest(&measure, false);
      float range = 0.0;
      if (measure.RangeStatus != 4) {
        range = ((float)measure.RangeMilliMeter) / 1000;
      }
      
      for (int i = 0; i < 180; i++){
        if (angle == i) {
          msg_scan.ranges[i] = range;
        }
      }
    }
};

ServoController sc;
LaserController lc;

void setup() { 
  Serial.begin(9600);
  sc.Setup(servoPin, 180);
  lc.Setup((float)sc.GetIncrement());
}

void loop() {
  delay(100);
  sc.ChangeAngle();
  lc.Update(sc.GetAngle());
}