#include <DriverMotors.h>
#include <VS1838.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "definitions.h"

//####################### DECLARACION DE OBJETOS   #################################
//PWM pins           //Control pin    INA INB...
DriverMotors motors((unsigned char []) {
  9, 10

}, (unsigned char []) {
  8 , 6 , 5 , 13
});
VS1838 IRModule(RECEPTOR_IR, LED_RED);

//####################### DECLARACIOM DE CONSTANTES ################################
const unsigned int key[21] PROGMEM = {
  0x9768, //0
  0xCF30, //1
  0xE718, //2
  0x857A, //3
  0xEF10, //4
  0xC738, //5
  0xA55A, //6
  0xBD42, //7
  0xB54A, //8
  0xAD52, //9
  0x5DA2, // CH-
  0x9D62, //CH
  0x1DE2,//CH+
  0xDD22,//|<<
  0xFD02,//>>|
  0x3DC2,//>||
  0x1FE0,//-
  0x57A8,//+
  0x6F90,//EQ
  0x6798,//100+
  0x4FB0//200+
};

//####################### DECLARACION DE VARIABLES #################################

int  i = 0, sensorValueF = 0;
bool  FlagR = false, FlagL = false, FlagF = false, FlagS;
int speedLeft, speedRight, max, speed = FULL_SPEED; //RFL
byte sensorByte, bitMaskSensors = ALL_SENSORS, steepsFront, steepsTurn;
unsigned int keyIR;
char direction, lastDirection, initialPosition;
unsigned long lastTime;
volatile  bool stateSensorR, stateSensorL, stateSensorF, stateQTRR, stateQTRL, Run;


void setup() {
  //Inicializa el robot
  initMinisumo();
  speed = FULL_SPEED;
  waitPressButton();
  max = speed;
  if (initialPosition == 'S') {
    steepsTurn = 10;
  } else {
    steepsTurn = 30;
  }
    if(initialPosition=='R'){
    motors.setSpeeds(255,-255);
    delay(75);
  }
 if(initialPosition=='L'){
    motors.setSpeeds(-255,255);
    delay(75);
  }

    motors.Stop();
  delay(20);
};

void loop() {
  static unsigned long lastAttackTime;

  if (micros() - lastTime >= 5) {
    readSensorAnalog();
    digitalWrite(LED_RED, stateSensorF);
    if (!READ_BTN) {
      delay(200);
      stopAll();
    }
    lastTime = micros();
  }

  if (millis() - lastAttackTime >= 5) {
    attack();
    lastAttackTime = millis();
  }
#if DEBUG
  static unsigned long lastMillis;
  if (millis() - lastMillis >= 200) {
    Serial.print("Vel:");
    Serial.print(max);
    Serial.print("\t D:");
    Serial.print((char)direction);
    Serial.print("\t LD:");
    Serial.print((char)lastDirection);
    Serial.print('\t');
    Serial.print("ALL=");
    Serial.print(sensorByte, BIN);
    Serial.print('\t');
    Serial.print("L=");
    Serial.print(stateSensorL);
    Serial.print('\t');
    Serial.print("F=");
    Serial.print(stateSensorF);
    Serial.print('\t');
    Serial.print("R=");
    Serial.print(stateSensorR);
    Serial.print('\t');
    Serial.print("QTR L=");
    Serial.print(stateQTRL);
    Serial.print('\t');
    Serial.print("QTR R=");
    Serial.println(stateQTRR);
    lastMillis = millis();
  }
#endif

};
//########################## ESPACIO PARA FUNCIONES ####################
void attack() {
  static byte buffer, lastBuffer;
  buffer = (sensorByte & bitMaskSensors);
  if (buffer != lastBuffer) {
    lastDirection = direction;
  }
  switch (buffer) {
    case 0: direction = 'S'; speedLeft  = -NORMAL_SPEED; speedRight = NORMAL_SPEED;  break;
    case 0b010: direction = 'F';  speedLeft  = max; speedRight = max;  break;
    case 0b100: direction = 'R';  speedLeft  = max; speedRight = -max; break;
    case 0b001: direction = 'L';   speedLeft  = -max; speedRight = max; break;
  }  
  motors.setSpeeds(speedLeft, speedRight);
  lastBuffer = buffer;
}
//Lectura de sensores digitales
ISR(TIMER1_OVF_vect) {
  static int i, sum, sum2;
  static bool sR, sL;
  if (i == 9) {
    if (sum >= 8) stateSensorR = true;
    else stateSensorR = false;
    if (sum2 >= 8) stateSensorL = true;
    else stateSensorL = false;

    stateQTRL = digitalRead(QTR_L); // 1 para fondo negro y 0 para fondo blanco
    stateQTRR = digitalRead(QTR_R);
    if (!Run) {
      stateSensorF = (analogRead(SENSOR_F) >= 900) ? true : false;
    }
    bitWrite(sensorByte, 0, stateSensorL);
    bitWrite(sensorByte, 1, stateSensorF);
    bitWrite(sensorByte, 2, stateSensorR);
    bitWrite(sensorByte, 3, stateQTRL);
    bitWrite(sensorByte, 4, stateQTRR);
    i = 0;
    sum = 0;
    sum2 = 0;
  } else {
    sR = !digitalRead(SENSOR_R);
    sL = !digitalRead(SENSOR_L);
    if (sL) stateSensorL = sL;
    if (sR) stateSensorR = sR;
    sum += sR;
    sum2 += sL;
    i++;
  }
}
void waitPressButton() {
  while (true) {
    if (!READ_BTN) {
      digitalWrite(LED_RED, HIGH);
      delay(200);
      break;
    }
    keyIR = IRModule.read(); //Obtener la direccion de la tecla oprimida
    if (keyIR) { //Si el valor de la tecla IR es mayor de 0 es una tecla valida
      i = 0;
      while (i < 21 && (pgm_read_word_near(key + i) != keyIR))++i;
      if (i == NUM_KEY_START) {
        digitalWrite(LED_RED, HIGH);
        break;
      } else if (i == 10 || i == 11 || i == 12) {
        LED_GREEN_ON;
        switch (i) {
          case 10: speed = LOW_SPEED;  break;
          case 11: speed = NORMAL_SPEED; break;
          case 12: speed = FULL_SPEED; break;
        }
      }
    }
  }
#if DEBUG
  Serial.println("Wait 5 seconds please...!");
#endif
  digitalWrite(POWER_SENSOR, HIGH);
  delay(100);
  motors.Stop();
  while (true) {
    static byte cont;
    if (millis() - lastTime >= 975) {
      digitalWrite(LED_RED, !digitalRead(LED_RED));
      cont++;
      if (cont >= 5) break;
#if DEBUG
      Serial.print(cont);
      Serial.println(" s");
#endif
      //Determina la posicion inicial del oponente
      switch ((sensorByte & bitMaskSensors)) {
        case 0: initialPosition = 'S'; break;
        case 0b100: initialPosition = 'R'; break;
        case 0b010: initialPosition = 'F'; break;
        case 0b001: initialPosition = 'L'; break;
      }
      lastTime = millis();
    }
  }
  Run = true;
  lastTime = 0;

#if DEBUG
  Serial.print("Pos. init:");
  Serial.println((char)initialPosition);
  delay(1000);
#endif

}
void initMinisumo() {
#if DEBUG
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Mini Blade");
#endif
  DDRD |= (1 << 5);
  LED_GREEN_OFF;
  PORTE |= (1 << 2); //Activa el pull-up del pulsador
  DDRE |= (1 << 2);
  analogReference(INTERNAL); //Selecciona la referencia de voltaje a 2.56V
  pinMode(POWER_SENSOR, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  //Activa la resistencia pull-up de los sensores
  pinMode(SENSOR_L, INPUT_PULLUP);
  pinMode(SENSOR_R, INPUT_PULLUP);
  pinMode(QTR_L, INPUT_PULLUP);
  pinMode(QTR_R, INPUT_PULLUP);
#if DEBUG
  digitalWrite(POWER_SENSOR, 1);
#endif
  initTimer1(); //Iniializa el timer1
}
void initTimer1() {

  cli();         //Desactiva interrupciones
  TIMSK1 |= (1<<TOIE1);
  TCCR1B = TCCR1B & 0b11111000;
  TCCR1B |= ((1 << CS11)); //Configura la frequencia de PWM a 7.2KHz
  sei();
}
void readSensorAnalog() {
  static unsigned long sensorsum = 0, start, delta;
  static int mean, lastMean = 0, samples = 30, n = 0, sensorValue;
  static char tol;
  static bool enable;
  sensorValue =  analogRead(SENSOR_F);
  if (sensorValue < 1023) {
    sensorsum += sensorValue;
    if (n == samples) {
      mean = sensorsum / n;
      if (mean < (MIN_DISTANCE / 2) ) {
        samples = 500;
        tol = 10;
      }
      else {
        tol = 2;
        samples = 30;
      }
      if (abs(mean - lastMean) <= tol) mean = lastMean;
      lastMean = mean;
      sensorsum = 0;
      n = 0;
    } else {
      n++;
    }
  }
  if (mean >= MIN_DISTANCE || sensorValue == 1023) {
    sensorValueF = mean;
    stateSensorF = true;
  }
  else {
    stateSensorF = false;
  }

}
void stopAll() {
  digitalWrite(BUZZER, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(POWER_SENSOR, LOW);
  LED_GREEN_OFF;
  while (true) {
    digitalWrite(LED_RED, !digitalRead(LED_RED));
    motors.Stop();
    delay(500);
  }
}

