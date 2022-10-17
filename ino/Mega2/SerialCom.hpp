#ifndef SERIALCOM_HPP
#define SERIALCOM_HPP

#include <SoftwareSerial.h>

#define SERIALCOMBUFFER 128

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

    /* Filtra os dados de "<CarControl, %f, %f>"
      - Variáveis:  
        - v_x, w_z */
    void ParseData(double* v_x, double* w_z)
    {
      char* strtokIndx;

      // Corta tempChars em ',', tirando o  texto inicial
      strtokIndx = strtok(tempChars, ",");  
      strcpy(message, strtokIndx);
      
      /* NOTA: o próximo strtok com arg1 = NULL continua de onde a ultima 
        chamada ao strtok() parou */
      
      // Filtra v_x
      strtokIndx = strtok(NULL, ",");       
      *v_x = atof(strtokIndx);

      // Filtra w_z
      strtokIndx = strtok(NULL, ",");       
      *w_z = atof(strtokIndx);
    }

    /* Processa novos dados */
    void ProcessNewData(double* v_x, double* w_z)
    {
      if (newData == true) {
        strcpy(tempChars, receivedChars);
        ParseData(v_x, w_z);
        newData = false;
      }
    }

    /* Manda mensagem de informações */
    void SendInfo(
      double x, double y, double theta, 
      double v_x, double w_z,
      double orientation_x, double orientation_y, double orientation_z, double orientation_w,
      double rpm_L, double rpm_R,
      int laserAngle, double laserRange)
    {
      char buff[SERIALCOMBUFFER] = {0};
      snprintf(
        buff, SERIALCOMBUFFER, 
        "<Info, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s>\n", 
        String(x).c_str(), String(y).c_str(), String(theta).c_str(), 
        String(v_x).c_str(), String(w_z).c_str(),
        String(orientation_x).c_str(), String(orientation_y).c_str(), String(orientation_z).c_str(), String(orientation_w).c_str(),
        String(rpm_L).c_str(), String(rpm_R).c_str(),
        String(laserAngle).c_str(), String(laserRange).c_str());

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
