#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

/* Classe para cinemática */
class Kinematics { 
  public:

    // Cinemática inversa (retorna em RPM)
    double Inverse_L(double v_x, double w_z, const float wheelRadius, const float wheelsAxisLength)
    {
      return RadPerS_to_RPM((-w_z * wheelsAxisLength / (2*wheelRadius)) + (v_x/wheelRadius));
    }

    // Cinemática inversa do motor direito (retorna em RPM)
    double Inverse_R(double v_x, double w_z, const float wheelRadius, const float wheelsAxisLength)
    {
      return RadPerS_to_RPM((w_z * wheelsAxisLength / (2*wheelRadius)) + (v_x/wheelRadius));
    }

    // Cinemática direta linear
    double Direct_linear(double rpm_L, double rpm_R, const double wheelRadius, const double wheelsAxisLength)
    {
      return (wheelRadius/2) * (RPM_to_RadPerS(rpm_L) + RPM_to_RadPerS(rpm_R));
    }

    // Cinemática direta angular
    double Direct_angular(double rpm_L, double rpm_R, const double wheelRadius, const double wheelsAxisLength)
    {
      return (wheelRadius / wheelsAxisLength) * (RPM_to_RadPerS(rpm_R) - RPM_to_RadPerS(rpm_L));
    }

    // RPM para rad/s
    double RPM_to_RadPerS(double rpm)
    {
      return rpm * 2 * PI / 60.0;
    }

    // rad/s para RPM
    double RadPerS_to_RPM(double radPerS)
    {
      return radPerS * 60.0 / (2 * PI);
    }
};

#endif
 
