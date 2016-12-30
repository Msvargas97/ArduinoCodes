#include <DriverMotors.h>
#include <CustomQTRSensors.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2000  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN
#define STOP_MODE  SHORT_BRAKE
#define MANUAL false
#define AUTO  true
#define NUM_MOTORS  2
#define START_TIME 500
#define NOISE_MIN 200
#define NOISE_MAX 700
#define RANGEBRAKE 3500

#define DEBUG false //Activa el monitor serial
#define BAUD  57600
#define MODE_CALIBRATION  MANUAL
#define SCLK 15
#define MISO 14
#define MOSI 16
#define LED_GREEN SCK
#define LED_RED MISO
#define LED_BLUE 11
#define S340K  7
#define ENABLE_SENSORS 6
#define BLACK_BTN A5
#define RED_BTN A4
#define RST_BTN 3

//Reinicio usando el perro guardian
#define soft_restart()      \
  do                        \
  {                         \
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }                       \
  }while(0)

//Instanciar Objetos

//Control PWM pins - All pins must be PWM
DriverMotors motors((unsigned char []) {
  9 , 10 , 13 , 5
}, STOP_MODE, NUM_MOTORS);

QTRSensorsRC qtrrc((unsigned char[]) {
  A5, A4, A3, A2, A1, A0, 254, 8
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);


boolean obstacule;
unsigned int sensorValues[NUM_SENSORS];

int  position,
     setPoint = (NUM_SENSORS - 1) * 500,
     last_proportional = 0,
     max = 100;
     
float Kp = 1;
float Kd = 0;
//float Ki = 0.001;

void setup() {
  init_Escarabajo();
  resetMillis();
}

void loop() {
  // readSensors();
  PID();
  /*
    testPID();
    testSensors();
  */
}

//######################## ESPACIO PARA FUNCIONES ######################
inline void PID() {
  position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, NOISE_MIN, NOISE_MAX);
  // El término proporcional debe ser 0 cuando estamos en línea
  int proportional = ((int)position) - setPoint;

  // Si entra en el rango de freno, aplicarlo en la
  // direccion de la curva
  if ( proportional <= -RANGEBRAKE )
  {
    motors.setSpeedRight(0);
    delay(1);
  }
  else if ( proportional >= RANGEBRAKE )
  {
    motors.setSpeedLeft(0);
    delay(1);
  }

  // Calcula el término derivativo (cambio) de la posición
  int derivative = proportional - last_proportional;

  // Recordando la última posición
  int last_proportional = proportional;

  // Calcula la diferencia entre la potencia de los dos motores [ m1 - m2 ].
  // Si es un número positivo, el robot gira a la [ derecha ]
  // Si es un número negativo, el robot gira a la [ izquierda ]
  //  y la magnitud del número determina el ángulo de giro.
  int power_difference = ( proportional * Kp ) + ( derivative * Kd );

  // Si velocidad diferencial es mayor a la posible tanto positiva como negativa,
  // asignar la máxima permitida
  if ( power_difference > max ) power_difference = max;
  else if ( power_difference < -max ) power_difference = -max;

  // Asignar velocidad calculada en el poder diferencial de los motores
  ( power_difference < 0 ) ?
  motors.setSpeeds(max + power_difference, max) : motors.setSpeeds(max, max - power_difference);

}


void init_Escarabajo() {
  cli();
  wdt_disable();
  sei();
  pinMode(ENABLE_SENSORS, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(S340K, INPUT_PULLUP);
  DDRD |= (1 << 5);
  PORTD &= ~(1 << 5);
  pinMode(RED_BTN, INPUT_PULLUP);
  pinMode(BLACK_BTN, INPUT_PULLUP);
  pinMode(RST_BTN, INPUT_PULLUP);
#if DEBUG
  Serial.begin(BAUD);
  delay(1000);
#endif
  delay(200);
  static boolean calibrate = false;
  while (true) {
    if (!digitalRead(BLACK_BTN)) {
      calibrate = true;
      delay(150);
      break;
    } else if (!digitalRead(RED_BTN)) {
      calibrate = false;
      delay(150);
      break;
    }

  }
  rgbLED(1, 0, 0);
  if (calibrate) {
    analogWrite(6, 255);
#if DEBUG
    Serial.println("Calibrando sensores...");
#endif
    calibrateSensors(MODE_CALIBRATION);
    qtrrc.saveCalibration();
    while (digitalRead(RED_BTN));
  } else {
    rgbLED(1, 1, 1);
    qtrrc.restoreCalibration();
#if DEBUG
    Serial.println("Valores de sensores Restaurados");
#endif
  }
  rgbLED(0, 1, 0);

#if DEBUG
  printValues();
#endif
  delay(START_TIME);
  rgbLED(0, 0, 0);
  analogWrite(6, 255);
}
void calibrateSensors(boolean mode) {
  if (!mode) {
    for (int i = 0; i < 250; i++)  position = qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  } else {
    int speed = 10, lastPosition;
    delay(1000);
    motors.setSpeeds(-100, 60);
    while (true) {
      position = qtrrc.calibrate();
#if DEBUG
      Serial.println(position);
#endif
      /*Positions:
         1 2 3 4 5 6 7 8
         - - - x x - - -  ===> 0b010 = 2
         1 2 3 4 5 6 7 8
         x - - - - - - -  ===> 0b100 = 4
         1 2 3 4 5 6 7 8
         - - - - - - - x  ===> 0b001 = 1
      */


      if (position == 1) motors.ShortBrake();
    }
    /*     position = qtrrc.calibrate();
          if (position == 2) motors.setSpeeds(-(speed+40), speed);
          else if (position == 1) {
            motors.ShortBrake();
            break;
          }
          lastPosition = position;
          delay(10);
          speed++;
        }*/
  }
}
inline void readSensors() {
  obstacule = !bitRead(PINE, 6);
  resetTwo();
  position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, NOISE_MIN, NOISE_MAX);
}
inline void rgbLED(boolean red, boolean green, boolean blue) {
  if (red) digitalWrite(LED_RED, 1);
  else digitalWrite(LED_RED, 0);
  if (green) digitalWrite(LED_GREEN, 1);
  else digitalWrite(LED_GREEN, 0);
  if (blue) digitalWrite(LED_BLUE, 1);
  else digitalWrite(LED_BLUE, 0);
}
inline void resetTwo() {
  if (!bitRead(PIND, 0)) {
    rgbLED(1, 1, 0);
    delay(100);
    soft_restart();
  }
}

inline void printValues() {
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}

inline void testSensors() {
#if DEBUG
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print(position); // comment this line out if you are using raw values
  Serial.println(" S340k=" + (String)obstacule);
  delay(250);
#endif
}

