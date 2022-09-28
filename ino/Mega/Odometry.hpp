#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <geometry_msgs/Quaternion.h>

/* Classe para cinemática */
class Odometry {
  unsigned long t = 0;
 
  public:
    double x = 0.0, y = 0.0, theta = 0.0;

    // Atualiza odometria
    void UpdateOdom(double v_x, double w_z)
    {
      // Atualiza tempo
      unsigned long t_now = millis();
      unsigned long d_t = t_now - t;
      t = t_now;

      // Coloca v_x e w_z em [u]/ms e rad/ms (porque d_t está em ms)
      v_x = v_x / 1000.0;
      w_z = w_z / 1000.0;

      // Calcula o quanto girou
      theta = w_z * d_t + theta;
      // double d_theta = w_z * d_t + d_theta;
      
      // Calcula o quanto andou:
      // double d_x = v_x * cos(d_theta) * d_t;
      // double d_y = v_x * sin(d_theta) * d_t;

      double d_x = v_x * cos(theta);
      double d_y = v_x * sin(theta);

      // Calcula os novos valores (soma do antigo mais a nova variação)
      x      += d_x;
      y      += d_y;
      // theta  += d_theta;
    }

    // Converte 3 angulos de rotação para quatérnio (fonte: Wikipedia)
    geometry_msgs::Quaternion RpyToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
    {
        // Abbreviations for the various angular functions
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        geometry_msgs::Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }
};

#endif
 
 
