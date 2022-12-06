#ifndef SERIALCOM_HPP
#define SERIALCOM_HPP

#define RECEIVEDFLOATS 2
#define RECEIVEDLEN RECEIVEDFLOATS * sizeof(float) + 2

class SerialCom {
public:

  void ReceiveMsg(double* cmd_v_x, double* cmd_w_z) {
    char rc;
    float data[RECEIVEDFLOATS] = { 0 };
    bool okMsg = false;

    unsigned char buffer[RECEIVEDLEN] = { 0 };
    if (Serial3.available() > 0) {
      rc = Serial3.read();
      if (rc == '<') {
        buffer[0] = (unsigned char)rc;
        Serial3.readBytes(&buffer[1], RECEIVEDLEN - 1);
        // for (int i = 0; i < RECEIVEDLEN; i++)
        //   Serial.println((char)buffer[i]);
        if (buffer[RECEIVEDLEN - 1] == '>') {
          // Serial.println("Ok msg!");
          int i = 0;
          int j = 1;
          // Serial.print("Received info: ");
          while (i != RECEIVEDFLOATS) {
            memcpy(&data[i], &buffer[j], sizeof(float));
            // Serial.print(data[i]);
            // Serial.print(",");
            i = i + 1;
            j = j + 4;
          }
          // Serial.println();
          *cmd_v_x = data[0];
          *cmd_w_z = data[1];
        }
        // Serial3.flush();
        // Serial.flush();
      }
    }
  }

  void SendInfo(
    double x, double y, double theta,
    double v_x, double w_z,
    double orientation_x, double orientation_y, double orientation_z, double orientation_w,
    double rpm_L, double rpm_R,
    int laserAngle, double laserRange) {

    float data[13] = {
      (float)x, (float)y, (float)theta,
      (float)v_x, (float)w_z,
      (float)orientation_x, (float)orientation_y, (float)orientation_z, (float)orientation_w,
      (float)rpm_L, (float)rpm_R,
      (float)laserAngle, (float)laserRange
    };

    Serial.print("Sending info:");
    Serial.print('<');
    Serial.write((char*)data, 13 * sizeof(float));
    Serial.println('>');
    Serial.flush();

    Serial3.print('<');
    Serial3.write((char*)data, 13 * sizeof(float));
    Serial3.print('>');
    // Serial3.flush();
  }
};

#endif
