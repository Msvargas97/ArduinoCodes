#include <CustomQTRSensors.h>
#include <DriverMotors.h>

#define NUM_MOTORS 2 //Numero de motores a usar 
#define STOP_MODE SHORT_BRAKE //Tipo de freno aplicado cuando la velocidad sea igual a 0

#define DEBUG false //Activa el monitor serial
#define START_TIME 2000
#define NUM_SENSORS   8
#define TIMEOUT       2000
#define EMITTER_PIN   QTR_NO_EMITTER_PIN
#define BAUD  57600
#define LED_GREEN SCK
#define LED_RED MISO
#define LED_BLUE 11
#define S340K  7
#define ENABLE_SENSORS 6
#define BLACK_BTN A5
#define RED_BTN A4
#define SMD_BTN 3
#define KP                    .17
#define KD                    2.2
#define DEFAULT_SPEED        255
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
DriverMotors motors((unsigned char []){9 ,10 ,13 ,5},STOP_MODE,NUM_MOTORS);

unsigned int sensorValues[NUM_SENSORS];
unsigned int position,max = 300;
int lastError = 0;
unsigned int vel_max = DEFAULT_SPEED;
boolean stopMode = true,
        obstacule;

void setup() {
  pinMode(ENABLE_SENSORS, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(S340K, INPUT_PULLUP);
  pinMode(RED_BTN, INPUT_PULLUP);
  pinMode(BLACK_BTN, INPUT_PULLUP);
  pinMode(SMD_BTN, INPUT_PULLUP);
  delay(500);
#if DEBUG
  Serial.begin(BAUD);
  while (!Serial);
  Serial.flush();
  Serial.println("Monitor Serial Activado...!");

#endif
  digitalWrite(30, LOW);
  digitalWrite(6,1);
  static boolean calibrate = false;
  static boolean velConfi;
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
  motors.ShortBrake();
  rgbLED(0, 0, 0);
  resetMillis();
}

void loop() {
  position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0,200);
 // rgbLED(0, qtrrc.obstacule, 0);
  int error = position - 3500;
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
  int leftMotorSpeed = vel_max + motorSpeed;
  int rightMotorSpeed = vel_max - motorSpeed;
  motors.setSpeeds(leftMotorSpeed ,rightMotorSpeed);

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
  while (millis() < 7000) qtrrc.calibrate(QTR_EMITTERS_ON);
  /*Positions:
     1 2 3 4 5 6 7 8
     - - - x x - - -  ===> 0b010 = 2
     1 2 3 4 5 6 7 8
     x - - - - - - -  ===> 0b100 = 4
     1 2 3 4 5 6 7 8
     - - - - - - - x  ===> 0b001 = 1
  */

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


