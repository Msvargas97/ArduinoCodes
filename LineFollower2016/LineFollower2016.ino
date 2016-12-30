#include <CustomQTRSensors.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#define DEBUG false //Activa el monitor serial
#define START_TIME 2000
#define NUM_SENSORS   8
#define TIMEOUT       2000
#define EMITTER_PIN   255
#define BAUD  57600
#define LED_GREEN SCK
#define LED_RED MISO
#define LED_BLUE 11
#define S340K  7
#define ENABLE_SENSORS 6
#define BLACK_BTN A5
#define RED_BTN A4
#define SMD_BTN 3
#define KP                    .3
#define KD                    3
#define DEFAULT_SPEED        400
//Reinicio usando el perro guardian
#define soft_restart()      \
  do                        \
  {                         \
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }                       \
  }while(0)

QTRSensorsRC qtrrc((unsigned char[]) {
  A5, A4, A3, A2, A1, A0, 254, 8
}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];
unsigned int position, max = 100;
int lastError = 0;
unsigned int vel_max = DEFAULT_SPEED;
boolean stopMode = true,
        obstacule;

void setup() {
  wdt_disable();
  pinMode(ENABLE_SENSORS, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(S340K, INPUT_PULLUP);
  pinMode(RED_BTN, INPUT_PULLUP);
  pinMode(BLACK_BTN, INPUT_PULLUP);
  pinMode(SMD_BTN, INPUT_PULLUP);
  initialize_pwm();
  delay(500);
#if DEBUG
  Serial.begin(BAUD);
  while (!Serial);
  Serial.flush();
  Serial.println("Monitor Serial Activado...!");
#else
  power_adc_disable();
#endif
  digitalWrite(30, LOW);

  static boolean calibrate = false;
  static boolean velConfi;
  delay(500);
  while (true) {
    if (!digitalRead(BLACK_BTN)) {
      if (!velConfi) {
        calibrate = true; break;
      }
      else {
        rgbLED(1, 1, 1);
        if (vel_max > 0) vel_max--;
        delay(150);
        rgbLED(0, 0, 0);
      }
#if DEBUG
      Serial.println(vel_max);
#endif

    } else if (!digitalRead(RED_BTN)) {
      if (!velConfi) {
        calibrate = false; break;
      }
      else {
        rgbLED(1, 0, 1);
        if (vel_max < 400) vel_max++;
        delay(150);
        rgbLED(0, 0, 0);
      }
#if DEBUG
      Serial.println(vel_max);
#endif
    } else if (!digitalRead(SMD_BTN)) {
      velConfi = !velConfi;
      if (velConfi)
        rgbLED(1, 1, 0);
      else rgbLED(0, 0, 1);
      delay(250);
      rgbLED(0, 0, 0);
    }
  }
  rgbLED(1, 0, 0);
  if (calibrate) {
#if DEBUG
    Serial.println("Calibrando sensores...");
#endif
    calibrateSensors();
    qtrrc.saveCalibration();
    delay(1000);
#if DEBUG
    printValues();
#endif
    while (digitalRead(RED_BTN));
  } else {
    rgbLED(1, 1, 1);
    qtrrc.restoreCalibration();
#if DEBUG
    Serial.println("Valores de sensores Restaurados");
#endif
  }
  rgbLED(0, 1, 0);
  delay(START_TIME);
  rgbLED(0, 0, 0);
  resetMillis();
  digitalWrite(ENABLE_SENSORS, 1);
  delay(100);
}

void loop() {
  position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0, 200);
  // rgbLED(0, qtrrc.obstacule, 0);
  int error = position - 3500;
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
  if (motorSpeed > max) motorSpeed = max;
  else if (motorSpeed < -max) motorSpeed = -max;
  int leftMotorSpeed = vel_max + motorSpeed;
  int rightMotorSpeed = vel_max - motorSpeed;
  setSpeeds(leftMotorSpeed, rightMotorSpeed);

  //for (int i = -400; i < 401; i += 5) {
  //    setSpeeds(i,-i);//Da reversa y luego gira el motor hacia adelante
  //    delay(100);
  //  }
  //  for (int i = -400; i < 401; i += 5) {
  //    setSpeeds(-i,i);//Da reversa y luego gira el motor hacia adelante
  //    delay(100);
  //  }
  //  setSpeeds(0,0);
  //  delay(2000); //Tiempo de espera para observar el frenado

  //testSensors();
}

void calibrateSensors() {
  qtrrc.resetCalibration();
  pinMode(RED_BTN, OUTPUT);
  pinMode(BLACK_BTN, OUTPUT);
  digitalWrite(BLACK_BTN, LOW);
  digitalWrite(RED_BTN, LOW);
  delay(500);
  pinMode(RED_BTN, INPUT);
  pinMode(BLACK_BTN, INPUT);
  rgbLED(1, 0, 0);
  resetMillis();
  digitalWrite(ENABLE_SENSORS, 1);
  delay(100);
  while (millis() < 7000) qtrrc.calibrate(QTR_EMITTERS_ON);
  digitalWrite(ENABLE_SENSORS, 0);
  /*Positions:
     1 2 3 4 5 6 7 8
     - - - x x - - -  ===> 0b010 = 2
     1 2 3 4 5 6 7 8
     x - - - - - - -  ===> 0b100 = 4
     1 2 3 4 5 6 7 8
     - - - - - - - x  ===> 0b001 = 1
  */

}

void setSpeeds(int left, int right) {
  speedRight(right, stopMode);
  speedLeft(left, stopMode);
}
void speedLeft(int speed, boolean stop) {
  boolean reverse;
  reverse = false;
  if (speed < 0 ) {
    reverse = true;
    speed = abs(speed);
  }
  if (speed == 0) {
    if (stop) {
      OCR1A = OCR1B = 400;
    }
    else {
      OCR1A = OCR1B = 0;
    }
  } else {
    if (speed > 400) speed = 400;
    if (!reverse) {
      OCR1B = 0;
      OCR1A = speed; // Set PWM value
    } else {
      OCR1B = speed;
      OCR1A = 0; // Set PWM value
    }
  }
}
void speedRight(int speed, boolean stop) {
  boolean reverse;
  reverse = false;
  if (speed < 0 ) {
    reverse = true;
    speed = abs(speed);
  }
  if (speed == 0) {
    if (stop) {
      OCR3A = 400;
      TC4H = 400 >> 8;
      OCR4A = (400 &  0xFF);
    } else {
      OCR3A = TC4H = 0;
      OCR4A = 0;
    }
  } else {
    if (speed > 400) speed = 400;
    if (!reverse) {
      TC4H = (speed >> 8);
      OCR4A = (unsigned char) (speed & 0xFF); // Set PWM value
      OCR3A = 0;
    } else {
      OCR3A = speed;
      TC4H = 0;
      OCR4A = 0;
    }
  }
}
void initialize_pwm() {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(5, OUTPUT);
  noInterrupts();
  TCCR4A =  TCCR4B = TCCR4C = TCCR4D = 0;
  TCCR4A = 0b10000010;
  TCCR4B = 0x03;//0b1001;
  TCCR4D = 0b00000001;
  PLLFRQ = (PLLFRQ & 0xCF) | 0x20; // 64MHz  0x10; 96 MHz 0x30; 48 MHz
  TC4H = 400 >> 8;
  OCR4C = 400 & 0xFF;
  //Configurar PWM de 10 bits
  TCCR1A = TCCR3A = 0b10100010;
  TCCR1B = TCCR3B = 0b00011010;
  ICR1 = ICR3 = 400;
  interrupts();
}
void rgbLED(boolean red, boolean green, boolean blue) {
  digitalWrite(LED_RED, red);
  digitalWrite(LED_GREEN, green);
  digitalWrite(LED_BLUE, blue);
}

inline void printValues() {
  Serial.println("Emitters ON:");
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
  Serial.println("####################################");
  Serial.println("Emitters OFF:");
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOff[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOff[i]);
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
  Serial.println(" S340k=" + (String)qtrrc.obstacule);
  delay(250);
#endif
}


