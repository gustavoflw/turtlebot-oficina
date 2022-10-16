#include <Servo.h>
#include <ros.h>
#include <Adafruit_VL53L0X.h>
#include <sensor_msgs/LaserScan.h>

#define RISING 0
#define FALLING 90

#define PIN_SERVO 10

class ServoController
{
  int angle = 0;
  int angleStatus = RISING;
  int increment = 5;

  unsigned long t = millis();
  unsigned long dt_update = 10;

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
  Adafruit_VL53L0X lox = Adafruit_VL53L0X();

  public:

    void Setup(float increment, sensor_msgs::LaserScan* msg_scan)
    {
      msg_scan->header.frame_id = "scan_link";
      msg_scan->angle_min = 0; // [rad]
      msg_scan->angle_max = PI; // [rad]
      msg_scan->angle_increment = increment; // [rad]
      // msg_scan->time_increment = 0.250; // [s]
      // msg_scan->scan_time = 0.5; // [s]
      msg_scan->range_min = 0; // [m]
      msg_scan->range_max = 20; // [m]

      // msg_scan->ranges[2] = {0};
      float ranges[1] = {0};
      msg_scan->ranges = ranges;
      msg_scan->ranges_length = 1;

      for (int i=0; i < msg_scan->ranges_length; i++) {
        msg_scan->ranges[i] = 0.0;
      }

      // Serial.println("Setup laser...");
      if (!lox.begin()) {
        // Serial.println("Laser???????");
        delay(5);
      }
        
    }
    void Update(int angle, sensor_msgs::LaserScan* msg_scan)
    {
      VL53L0X_RangingMeasurementData_t measure;
      lox.rangingTest(&measure, false);
      double range = 0.0;
      if (measure.RangeStatus != 4) {
        range = ((double)measure.RangeMilliMeter) * 0.8 / 1000;
        // Serial.print("range:");
        // Serial.print(range);
        // Serial.println();

        msg_scan->ranges[0] = range;

        // for (int i = 0; i < 180; i++) {
        //   if (angle == i) {
        //     // Serial.println(i);
        //     msg_scan->ranges[i] = range;            
        //   }
        // }
      } 
    }
};

ServoController sc;
LaserController lc;

ros::NodeHandle nh;

sensor_msgs::LaserScan msg_scan;
ros::Publisher pub_time("/scan", &msg_scan);


void setup() { 
  // Serial.begin(57600);
  sc.Setup(PIN_SERVO, 1);
  lc.Setup(10, &msg_scan);
  // Serial.println("setup ok");

  nh.initNode();
  nh.advertise(pub_time);
  // Serial.println("Ros ok");
}

void loop() {
  delay(5);
  sc.ChangeAngle();
  lc.Update(sc.GetAngle(), &msg_scan);

  nh.spinOnce();

  msg_scan.header.stamp = nh.now();
  pub_time.publish(&msg_scan);

  // for (int i = 0; i < 180; i++){
  //   Serial.print(lc.msg_scan.ranges[i]);
  //   // Serial.print(",");
  //   // Serial.print(i);
  //   // Serial.println();
  // }
  // Serial.println();
}