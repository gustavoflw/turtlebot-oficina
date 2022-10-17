#ifndef SERIALCOM_HPP
#define SERIALCOM_HPP

#include <SoftwareSerial.h>

#define SERIALCOMBUFFER 1024

class SerialCom {
  const byte numChars = SERIALCOMBUFFER;
  char receivedChars[SERIALCOMBUFFER];
  char tempChars[SERIALCOMBUFFER];
  char message[SERIALCOMBUFFER] = {0};
  bool newData = false;
  
  public:

    /* Recebe mensagem com string no formato "<Mensagem>" */
    void ReceiveMsgWithMarkers()
    {
      static boolean recvInProgress = false;
      static byte ndx = 0;
      char rc;

        while (Serial.available() > 0 && newData == false) {
          rc = Serial.read();
          
          if (recvInProgress == true) {
              if (rc != '>') {
                  receivedChars[ndx] = rc;
                  ndx++;
                  if (ndx >= numChars) {
                      ndx = numChars - 1;
                  }
              }
              else {
                  receivedChars[ndx] = '\0'; // terminate the string
                  //Serial.println(receivedChars);
                  recvInProgress = false;
                  ndx = 0;
                  newData = true;
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
    void ParseData(float* rpmL, float* rpmR, float* laserAngle, double* laserRange)
    {
      // char* strtokIndx;

      // // Corta tempChars em ',', tirando o  texto inicial
      // strtokIndx = strtok(tempChars, ",");  
      // strcpy(message, strtokIndx);
      
      // /* NOTA: o próximo strtok com arg1 = NULL continua de onde a ultima 
      //   chamada ao strtok() parou */
      
      // // Filtra rpmL
      // strtokIndx = strtok(NULL, ",");       
      // *rpmL = atoi(strtokIndx);

      // // Filtra rpmR
      // strtokIndx = strtok(NULL, ",");       
      // *rpmR = atoi(strtokIndx);

      // // Filtra laserAngle
      // strtokIndx = strtok(NULL, ",");       
      // *laserAngle = atoi(strtokIndx);

      // // Filtra laserRange
      // strtokIndx = strtok(NULL, ",");       
      // *laserRange = atof(strtokIndx);
    }

    /* Processa novos dados */
    void ProcessNewData(int* rpmL, int* rpmR, int* laserAngle, double* laserRange)
    {
      if (newData == true) {
        strcpy(tempChars, receivedChars);
        ParseData(rpmL, rpmR, laserAngle, laserRange);
        newData = false;
      }
    }

    /* Manda mensagem de controle da velocidade do carro */
    void SendSpeedControl(double v_x, double w_z)
    {
      char buff[SERIALCOMBUFFER] = {0};
      snprintf(buff, SERIALCOMBUFFER, "<SpeedControl, %f, %f>\n", v_x, w_z);
      Serial.write(buff, SERIALCOMBUFFER);
    }

    /* Printa os dados */
    void ShowParsedData(char* messageFromUno, int distance) 
    {
        Serial.print("Message: ");
        Serial.println(message);
    }
};

#endif
