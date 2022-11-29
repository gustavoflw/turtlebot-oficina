#ifndef SERIALCOM_HPP
#define SERIALCOM_HPP

#include <SoftwareSerial.h>

#define SERIALCOMBUFFER 64
#define NUM_FLOATS 2


class SerialCom {
  const byte numChars = SERIALCOMBUFFER;
  char receivedChars[SERIALCOMBUFFER];
  char tempChars[SERIALCOMBUFFER];
  char message[SERIALCOMBUFFER] = { 0 };
  bool newData = false;
  bool parsed = false;

public:

  /* Recebe mensagem com string no formato "<Mensagem>" */
  void ReceiveMsgWithMarkers(SoftwareSerial* customSerial) {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char rc;

    // Serial.println("rcv serial");
    while (customSerial->available() > 0 && newData == false) {
      rc = customSerial->read();
      Serial.print(rc);
      Serial.print(',');

      if (recvInProgress == true) {
        if (rc != '>') {
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars) {
            ndx = numChars - 1;
          }
        } else {
          receivedChars[ndx] = '\0';  // terminate the string
          // Serial.print("\nRECEIVED: ");
          // Serial.println(receivedChars);
          recvInProgress = false;
          ndx = 0;
          newData = true;
          parsed = false;
        }
      }

      else if (rc == '<')
        recvInProgress = true;
    }
  }

  /* Filtra os dados de "<CarInfo, %f, %f, %f, %f, %f ...>"
      - Variáveis:  
        - x, y, theta, 
        - v_x, w_z, 
        - orientation_x, orientation_y, orientation_z, orientation_w, 
        - rpm_L, rpmR, 
        - laserAngle, laserRange */
  void ParseData(
    geometry_msgs::Twist* msg_odom_twist, geometry_msgs::Pose* msg_odom_pose,
    std_msgs::Float64* msg_rpmL, std_msgs::Float64* msg_rpmR,
    std_msgs::Float64* msg_odom_theta,
    std_msgs::Int32* msg_laser_angle, std_msgs::Float64* msg_laser_range) {

    float floats[NUM_FLOATS];

    int i = 0; // contador do array de bytes
    int j = 0; // contador do array de floats
    while (i < NUM_FLOATS * sizeof(float))
    {
      memcpy(&tempChars[i], &floats[j], sizeof(float));
      Serial.println(floats[j]);

      i = i + sizeof(float);
      j = j + 1;
    }

    // char* strtokIndx;

    // Corta tempChars em ',', tirando o  texto inicial
    // strtokIndx = strtok(tempChars, ",");
    // strcpy(message, strtokIndx);

    // /* NOTA: o próximo strtok com arg1 = NULL continua de onde a ultima
    //   chamada ao strtok() parou */

    // if (String(strtokIndx) == "xyt") {
    //   Serial.println("SERIAL FOUND: xyt");
    // } else if (String(strtokIndx) == "vxwz") {
    //   Serial.println("SERIAL FOUND: vxqz");
    // } else if (String(strtokIndx) == "ori") {
    //   Serial.println("SERIAL FOUND: ori");
    // } else if (String(strtokIndx) == "rpm") {
    //   Serial.println("SERIAL FOUND: rpm");
    // } else if (String(strtokIndx) == "laser") {
    //   Serial.println("SERIAL FOUND: laser");
    //   strtokIndx = strtok(NULL, ",");
    //   if (!String(strtokIndx).isEmpty())
    //     msg_laser_angle->data = atoi(strtokIndx);
    //   strtokIndx = strtok(NULL, ",");
    //   if (!String(strtokIndx).isEmpty())
    //     msg_laser_range->data = atof(strtokIndx);
    // }
  }

  /* Processa novos dados */
  void ProcessNewData() {
    if (newData == true) {
      strcpy(tempChars, receivedChars);
      newData = false;
    }
  }

  /* Manda mensagem de controle da velocidade do carro */
  void SendSpeedControl(SoftwareSerial* customSerial, double v_x, double w_z) {
    char buff[SERIALCOMBUFFER] = { 0 };
    snprintf(buff, SERIALCOMBUFFER, "<SpeedControl, %f, %f>\n", v_x, w_z);
    bool clear = false;
    for (int i = 0; i < SERIALCOMBUFFER; i++) {
      if (buff[i] == '>')
        clear = true;
      else if (clear == true)
        buff[i] = '>';
    }
    customSerial->write(buff, SERIALCOMBUFFER);
    Serial.print("\nSENDING: ");
    Serial.write(buff, SERIALCOMBUFFER);
  }

  /* Printa os dados */
  void ShowParsedData(char* messageFromUno, int distance) {
    Serial.print("Message: ");
    Serial.println(message);
  }
};

#endif