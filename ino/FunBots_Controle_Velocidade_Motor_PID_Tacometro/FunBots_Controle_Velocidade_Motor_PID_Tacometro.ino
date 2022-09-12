// Bibliotecas
#include <AFMotor.h>
#include <PID_v1.h>

// Configuração do Motor
AF_DCMotor motor(3);

// Constantes do Controle PID
#define MIN_PWM 0
#define MAX_PWM 255
#define KP 0.1
#define KI 0.2
#define KD 0.005

// Variáveis do Sensor Infravermelho e PID
double rpm;
volatile byte pulsos;
unsigned long timeold;
int pinoSensor = 19;               //Pino do Arduino ligado ao pino D0 do sensor
unsigned int pulsosDisco = 20;    //Altere o valor conforme disco encoder
double velocidade = 0;
double velocidadeSetpoint = 130;  // Alterar conforme velocidade desejada

// Cria PID para controle
PID motorPID(&rpm, &velocidade, &velocidadeSetpoint, KP, KI, KD, DIRECT);

// Função executada a cada interrupção
void contador()
{
  pulsos++;  //Incrementa contador
}

/*
 *  SETUP
 */
void setup() {
  // Inicia Serial
  Serial.begin(9600);

  // Alimentacao 5V
  pinMode(28, OUTPUT);
  pinMode(30, OUTPUT);
  digitalWrite(28, HIGH);
  digitalWrite(30, HIGH);
 
  // Configura Interrupção
  pinMode(pinoSensor, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinoSensor), contador, FALLING);
  pulsos = 0;
  rpm = 0;
  timeold = 0;

  // Configura controle PID
  motorPID.SetOutputLimits(MIN_PWM, MAX_PWM);
  motorPID.SetMode(AUTOMATIC);
}

/*
 * LOOP
 */
void loop() {
  delay(30);
  // Calcula RPM a cada 1 Segundo
  unsigned long dt = 250;
  if (millis() - timeold >= dt)
  {
    detachInterrupt(0);    //Desabilita interrupção durante o cálculo para evitar sair do IF
    rpm = (60.0 * (float)dt / (float)pulsosDisco ) / (millis() - timeold) * pulsos;
    timeold = millis();
    pulsos = 0;
    
    // Exibe valores no serial monitor
    Serial.print("Vel: ");
    Serial.print(velocidade, 2);
    Serial.print("    ");
    Serial.print("RPM: ");
    Serial.println(rpm, 0);
    
    // Habilita novamente a interrupção
    attachInterrupt(19, contador, FALLING);
  }

  // Calcula o PWM do motor conforme Controle PID
  motorPID.Compute();

  // Ajusta PWM no motor
  motor.setSpeed(velocidade);   // Utiliza velocidade calculada
  motor.run(FORWARD);          // Movimenta motor
}
