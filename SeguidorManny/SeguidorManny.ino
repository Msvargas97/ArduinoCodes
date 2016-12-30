#include <DriverMotors.h>
#include <QTRSensors.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#define DEBUG 0
#define TIME_STOP  40000
#define NUM_MOTORS 2 //Numero de motores a usar 
#define STOP_MODE SHORT_BRAKE //Tipo de freno aplicado cuando la velocidad sea igual a 0
#define NUM_SENSORS  6     // number of sensors used
#define TIMEOUT       2000  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN
// max speed of the robot
#define rightMaxSpeed 80 // max speed of the robot
#define leftMaxSpeed 80 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define rightBaseSpeed 70 // this is the speed at which the motors should spin when the robot is perfectly on the line 
#define leftBaseSpeed 70
//Reinicio usando el perro guardian
#define soft_restart()      \
  do                          \
  {                           \
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }                       \
  }while(0)

//Instanciar Objetos
//Control PWM pins - All pins must be PWM
DriverMotors motors((unsigned char []) {
  3, 5, 6 , 9
}, STOP_MODE, NUM_MOTORS);
QTRSensorsRC qtrrc((unsigned char[]) {
  10, 16, 14, 15, A0, A1, A2, A3
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
unsigned int position, proportional, last_proportional;
int lastError;
int max = 120;
float KP = 0.6;
float KD = 20;
unsigned long integral;
boolean enableStop = false;
unsigned long time;
void cal() {
  qtrrc.calibrate();
  qtrrc.calibratedMinimumOn[0] =  208 ;
  qtrrc.calibratedMinimumOn[1] =  208 ;
  qtrrc.calibratedMinimumOn[2] = 108;
  qtrrc.calibratedMinimumOn[3] = 108;
  qtrrc.calibratedMinimumOn[4] = 208;
  qtrrc.calibratedMinimumOn[5] = 208;
  qtrrc.calibratedMinimumOn[6] = 212;
  qtrrc.calibratedMinimumOn[7] = 264;
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    qtrrc.calibratedMaximumOn[i] = TIMEOUT;
  }

}
void setup() {

  motors.ShortBrake();
  delay(1000);
  pinMode(2, INPUT_PULLUP);
  pinMode(8, OUTPUT);
  DDRB |= (1 << 0);
  PORTD &= ~(1 << 5);
  PORTB &= ~(1 << 0);
#if DEBUG
  Serial.begin(9600);
#endif
  while (digitalRead(2));
  digitalWrite(8, HIGH);    //Encender LED
  // Calibration();
  cal();
  digitalWrite(8, LOW);
  while (digitalRead(2));
  delay(1000);
  PORTD |= (1 << 5);
  PORTB |= (1 << 0);
  resetMillis();
}

int setPoint = 3500;
int contCurvasR,contCurvasL,FlagCurvasL,FlagCurvasR;
void loop() {
  static bool FlagL, FlagR, FlagCruces;
  static unsigned char contL, contR, contCruces;
  time = millis();

  if (!bitRead(PIND, 1)) {
    digitalWrite(8, LOW);
    motors.ShortBrake();
    soft_restart();
  }
  int position = qtrrc.readLine(sensorValues);
//  if (sensorValues[7] > 200 && sensorValues[1] > 200 && sensorValues[3] > 200 && sensorValues[4] > 200) {
//    if (!FlagCruces) {
//      contCruces++;
//      FlagCruces = true;
//      if (contCruces >= 28 && contCruces <= 30){
//        setPoint =4500;
//      }else if(contCruces == 31 || contCruces == 32){
//        setPoint = 3500;
//      }
//    }
//  }else{
//    FlagCruces = false;
//  }

  if (position >= 6500 || position <= 500) {
    //KP = 1;
    //KD = 0;
    if (position >= 6500) {
      FlagCurvasL = false;
      motors.setSpeeds(-127, 127);
      _delay_ms(2);
    } else {
       FlagCurvasR = false;
      if(!FlagCurvasL){
        FlagCurvasL = true;
        contCurvasL++;
      }
      motors.setSpeeds(127, -127);
      _delay_ms(2);
    }

  } 
//  if(contCurvasR >= 13 && sensorValues[0] > 200 & (sensorValues[3] > 200 || sensorValues[4] > 200)){
//        digitalWrite(8, LOW);
//    motors.ShortBrake();
//    soft_restart();
//  }
  // El término proporcional debe ser 0 cuando estamos en línea
  proportional = ((int)position) - setPoint;
  // Calculos PD
  int derivative = proportional - last_proportional;

  integral += proportional;
  last_proportional = proportional;

  int power_difference = proportional / KP + (integral * 0) + derivative * KD;

  if (power_difference > max)
    power_difference = max;
  if (power_difference < -max)
    power_difference = -max;

  if (power_difference < 0)
    motors.setSpeeds(max , max + power_difference);
  else
    motors.setSpeeds(max - power_difference, max);
  

  //
  //  if (time >= TIME_STOP){
  //    enableStop = true;
  //    digitalWrite(8,HIGH);
  //        digitalWrite(8, LOW);
  //    motors.ShortBrake();
  //    soft_restart();
  //  }
  //  if (enableStop && (sensorValues[0] >= 400  || sensorValues[1] >= 400) && (sensorValues[3] >= 400 || sensorValues[4] >= 400)) {
  //    digitalWrite(8, LOW);
  //    motors.ShortBrake();
  //    soft_restart();
  //  }
}

void Calibration() {
  for (int i = 0; i < 100; i++) {
    qtrrc.calibrate();
    digitalWrite(8, !digitalRead(8));
    PORTB ^= (1 << 0);
    PORTD ^= (1 << 5);
    delay(20);
  }
  motors.ShortBrake();
}

void debugCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}
