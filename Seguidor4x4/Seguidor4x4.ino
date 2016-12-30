#include <CustomQTRSensors.h>
#include <avr/interrupt.h>

#define NUM_SENSORS   8     //Número de sensores usados
#define TIMEOUT       2000  //Tiempo de lectura de sensores
#define EMITTER_PIN   6     //Control on-off

#define PWM9 OCR1A
#define PWM10 OCR1B
#define PWM5 OCR3A
#define PWM13L OCR4A

//Instanciar Objetos
QTRSensorsRC qtrrc((unsigned char[]) {
  A5, A4, A3, A2, A1, A0, 254, 8
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];
unsigned int position;

double KP = 0.06,
       KI = 0,
       KD = 2.2;

int max = 500,
    lastError,
    integral;

void setup() {
  init_IO();
  while (millis() < 3000) qtrrc.calibrate( QTR_EMITTERS_ON_AND_OFF);
  digitalWrite(30, LOW);
  init_pwm10bits();
}
void loop() {
  position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON_AND_OFF, 0, 200);//Lectura de sensores
  int error = position - 3500; //Calculo del error
  integral += (KI * error); //Correción respecto a la linea
  //Acotar termino integral
  if (integral > max) integral = max;
  else if (integral < -max) integral = -max;

  int motorSpeed = KP * error + integral + KD * (error - lastError);
  lastError = error;
  int leftMotorSpeed = 0x3FF + motorSpeed;
  int rightMotorSpeed = 0x3FF - motorSpeed;

  if (leftMotorSpeed > (max * 2) ) leftMotorSpeed = max;
  else if (leftMotorSpeed < (-max * 2)) leftMotorSpeed = -max;

  if (rightMotorSpeed > max ) rightMotorSpeed = max;
  else if (rightMotorSpeed < (-max * 2)) rightMotorSpeed = -max;
  if(abs(leftMotorSpeed) <= 0x3FF){
  if (leftMotorSpeed > 0) {
    PWM10 = 0;
    PWM9 = leftMotorSpeed;
  } else {
    PWM9 = 0;
    PWM10 = abs(leftMotorSpeed);
  }
  }
  if(abs(rightMotorSpeed) <= 0x3FF){
  if (rightMotorSpeed > 0) {
    PWM5 = 0;
    TC4H = rightMotorSpeed >> 8;
    PWM13L = rightMotorSpeed & 0xFF;
  } else {
    TC4H = 0;
    PWM13L = 0;
    PWM5 = abs(rightMotorSpeed);
  }
  }
}
void init_IO() {
  DDRB |= ((1 << 5) | (1 << 6)); //Establece los pines como slaidas
  PORTB &= (~(1 << 5) & ~(1 << 6)); //Pone en bajos los pines de salida 9 y 10
  DDRC |= ((1 << 6) | (1 << 7)); //Establece los pines como slaidas
  PORTC &= (~(1 << 6) & ~(1 << 7)); //Pone en bajos los pines de salida 13 y 5
  pinMode(7, INPUT_PULLUP);
  delayMicroseconds(5);
}
void init_pwm10bits() {
  cli(); //Deshabilita las interrupciones
  //Configuracion de PWM - Fast PWM de 10 bits (1023) en los pines 9, 10 y 5
  TCCR3A = TCCR3B  = TCCR1A = TCCR1B = 0;
  TCCR3A  = ((1 << COM1A1) | (1 << WGM11) | (1 << WGM10));
  TCCR1A  = ((1 << COM1A1) | (1 << COM1B1) | (1 << WGM11) | (1 << WGM10)); //Selecciona los pines pwm de salida
  TCCR1B = TCCR3B  = ((1 << WGM12) | (1 << CS11) | (1 << CS10)); //Seleciona prescaler de 8 -> Freq= 2KHz

  //Configuracion timer4 PWM 10bits
  TCCR4A = TCCR4B = TCCR4C = TCCR4D = TCCR4E = 0;
  TCCR4A = ((1 << COM4A1) | (1 << PWM4A)); //Activa la salida del pin 13
  PLLFRQ &= (~(1 << PLLTM1) & ~(1 << PLLTM0)); // Deshabilita el postcaler del PLL
  //PLLFRQ = (PLLFRQ & 0xCF) | 0x30; Postcaler de 48 Mhz
  TCCR4B = (1 << CS42); //Prescaler de 8
  TC4H = 0x03; //Asigna la resolucion de 10bits
  OCR4C = 0xFF;
  PWM9 = PWM10 = PWM5 = PWM13L = 0;
  sei(); //Habilita las interrupciones
  digitalWrite(30, LOW);
}

