#ifndef ENCODER_HPP
#define ENCODER_HPP

/* Classe para encoders ópticos */
class EncoderHandle {
  // Constante
  int numHoles      = 20;
  
  // Variáveis usadas no incremento do contador (microssegundos)
  unsigned int  counter         = 0;
  unsigned long t_inc_last      = 0;
  unsigned long t_inc_interval  = 5000;
  unsigned long dt_inc          = 0;

  // Variáveis usadas para atualizar velocidade do motor (milissegundos)
  unsigned long t_update_last     = 0;
  unsigned long t_update_interval = 500;
  double rpm = 0.0;
 
  public:
    // Incrementa o contador e adiciona o tempo decorrido ao dt_inc
    void IncrementCounter()
    {
      unsigned long t_now = micros();
      unsigned long dt = t_now - t_inc_last;
      if (dt >= t_inc_interval) {
        t_inc_last = t_now;
        dt_inc = dt_inc + dt;
        counter++;
        // Serial.print("counter: ");
        // Serial.print(counter);
        // Serial.print(", ");
        // Serial.print("dt_inc [ms]: ");
        // Serial.print(dt_inc/1000.0);
        // Serial.print(", ");
        // Serial.print("dt [ms]: ");
        // Serial.println(dt/1000.0);
      }
    }

    // Reseta o contador e o dt_inc
    void ResetCounter()
    {
      dt_inc = 0;
      counter = 0;
    }

    // Atualiza o RPM do motor, resetando o contador
    boolean Update()
    {
      // Rapido
      t_update_interval = 50;

      // Lento
      // if (rpm <= 20)
      //   t_update_interval = 100;        
      // else if (rpm <= 120)
      //   t_update_interval = 500;
      // else if (rpm <= 220)
      //   t_update_interval = 250;
      // else
      //   t_update_interval = 250;

      unsigned long t_now = millis();
      unsigned long dt = t_now - t_update_last;
      if (dt >= t_update_interval) {
        t_update_last = t_now;
        rpm = counter*0.5;        // Qtd de espaços a roda percorreu
        rpm = rpm / numHoles; // Qtd de rotações completas feitas
        rpm = rpm / dt;       // Rotação por ms
        rpm = rpm * 1000;     // RPS
        rpm = rpm * 60;       // RPM
        ResetCounter();
        // Log();
        return true;
      }
      return false;
    }
    
    // Retorna o RPM
    float GetRPM() 
    {
      return rpm;
    }
    
    // Printa log
    void Log()
    {
      Serial.print("rpm:");
      Serial.print(rpm);
      Serial.print(",");
      Serial.print("t_update_interval:");
      Serial.println(t_update_interval);
    }
};

#endif
