//NOTA: CONSUMO FULL BATTERY 8.5 -> 0.36mA with motors sin carga, without motors = 150mA
#include <DriverMotors.h>
#include <IRremote.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#define DEBUG   0
#define INIT_TIME   250
#define VEL_REMOTE  60

//Directivas para las teclas del control Remoto
#define FORWARD 0xC20370A1
#define BACK    0x81930A09
#define RIGHT   0x21035431
#define LEFT    0x983AB4C1
#define STOP    0xBB0ED9E1
#define POWER   0xF61E2A57
#define PLAY_UP 0x80BFBB44
#define PLAY_DOWN 0x80BF31CE
#define PRESSED 0xFFFFFFFF
#define LAST 0xD28EF217
#define _1      0xF7283C77
#define _2      0x757FB4DF
#define _3      0xB33B4597
#define _4      0x3C03E507
#define _5      0xE705551F
#define _6      0xA4A58EC7
#define _7      0xE2E45F7F
#define _8      0x6BACFEEF
#define _9      0xE88E91F
#define _0      0x7D168BCF


#define TURN_RIGHT  true
#define TURN_LEFT   false

#define SPEED_L 170
#define SPEED_R 255
#define CARPINTERO  1
#define TORNADO 2
#define AVANCE_LARGO 3
//Reinicio usando el perro guardian
#define soft_restart()      \
  do                          \
  {                           \
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }                       \
  }while(0)

//Control para el puente H VNH51180A
//PWM pins           //Control pin    INA INB...
DriverMotors motors((unsigned char []) {
  10, 9
}, (unsigned char []) {
  6, 8, 13 , 5,
}, SHORT_BRAKE, 2);

IRrecv irrecv(0);
decode_results results;

//Crea variables volatiles para poder acceder desde funcion del timer
boolean track, //Variable para habilitar la memoria hacia donde se perdio el robot
        isRun,
        direction;

unsigned char object = 0;

uint8_t strategies;
int time_fwd;

void setup() {
  init_minisumo(); //Inicialización del robot
  //Si el modo depuración se activa, se activa la comunicación serial
#if DEBUG
  Serial.begin(9600); //Velocidad de c.s
  while (!Serial);
  Serial.println("Estrategia:" + (String)strategies);
  Serial.print("Direccion:");
  if (direction == TURN_RIGHT) Serial.println("DERECHA");
  else Serial.println("IZQUIERDA");
  delay(1000);

#endif
  strategies = CARPINTERO;
  while (!isRun) {
    readIRControl();
  }
  track = true; //Activa la memoria para seguir al objetivo
  digitalWrite(4, 0);
  for (int i = 0; i < 5; i++) {
    digitalWrite(4, 1);
    delay(INIT_TIME);
    digitalWrite(4, 0);
    delay(INIT_TIME);
  }
  if (strategies == AVANCE_LARGO) {
    time_fwd = 100;
    strategies = CARPINTERO;
  } else {
    time_fwd = 50;
  }
  motors.ShortBrake();
  if (strategies == CARPINTERO) {
    resetMillis();
    motors.setSpeeds(SPEED_L * 0.8, SPEED_R * 0.8);
    while (millis() <= time_fwd) {
      object = detectObject(!bitRead(PINF, 0), !bitRead(PINF, 6));
      if (object >= 1 && object <= 3 ) break;
    }
    motors.ShortBrake();
  }
 /* object = detectObject(!bitRead(PINF, 0), !bitRead(PINF, 6));
  if (object == 0xFF && strategies == 0) strategies = TORNADO;*/
  resetMillis();
  track = false;
}
void loop() {
  static boolean Flag = direction;
  static int count;
  resetMillis();
  object = detectObject(!bitRead(PINF, 0), !bitRead(PINF, 6));
  switch (strategies) {
    case CARPINTERO: //Carpintero
      while (millis() < 3000 && !Flag) {
        object = detectObject(!bitRead(PINF, 0), !bitRead(PINF, 6));
        if (object == 0xFF) Flag = false;
        else Flag = true;
      }
      resetMillis();
      if (!Flag) {
        if (count < 4) {
          motors.setSpeeds(SPEED_L, SPEED_R);
          while (millis() <= 50 && object == 0xFF) object = detectObject(!bitRead(PINF, 0), !bitRead(PINF, 6));
          motors.ShortBrake();
          count++;
        } else {
          count = 0;
          strategies = TORNADO;
        }

      } else {
        attack();
        Flag = false;
      }
      break;
    case TORNADO: // Tornado
      Flag = !Flag;
      if (!Flag) {
        motors.ShortBrake();          delay(10);
        motors.setSpeeds(SPEED_L, -SPEED_R);
        delay(10);
      } else {
        motors.ShortBrake();          delay(10);
        motors.setSpeeds(-SPEED_L, SPEED_R);
        delay(10);
      }
      while (millis() < 3500 && object == 0xFF)  object = detectObject(!bitRead(PINF, 0), !bitRead(PINF, 6));
      attack();
      break;
    default:
      attack();
      break;
  }
  while (!isRun) {
    motors.ShortBrake();
    soft_restart();
  }
  delay(2);

};//end void loop
//############################# FUNCIONES ###############################
void attack() {
  while (object != 255) {
    if (strategies != TORNADO)  track = true;
    object = detectObject(!bitRead(PINF, 0), !bitRead(PINF, 6));
    switch (object) {
      case 1:
        motors.setSpeeds(SPEED_L, -SPEED_R);
        while (millis() < 25 ) {
          object = detectObject(!bitRead(PINF, 0), !bitRead(PINF, 6));
          if (object == 2) break;
        }
        break;
      case 2:
        motors.setSpeeds(-SPEED_L, SPEED_R);
        while (millis() < 25 ) {
          object = detectObject(!bitRead(PINF, 0), !bitRead(PINF, 6));
          if (object == 1) break;
        }
        break;
      case 3:
        motors.setSpeeds(SPEED_L, SPEED_R);
        if (strategies == TORNADO || strategies == CARPINTERO) strategies = 0;
        break;
      case 0xFF:
        motors.ShortBrake();
        break;
    }
  }
  track = false;
}
void init_minisumo() {
  //Seteo de perifericos
  DDRF = 0xFF;
  PORTF = 0;
  DDRD |= ((1 << 5) | (1 << 4)); //LEDS
  PORTD &= ~(1 << 5); //Apaga el led verde del bootloader
  irrecv.enableIRIn(); // Inicializa el modulo IR
  pinMode(A1, INPUT_PULLUP); //Activa las resistencias pull-up para los sensores
  pinMode(A5, INPUT_PULLUP);
  cli();
  wdt_disable();
  sei();
  //Selecciona la rutina que desea ejecutar
#if !DEBUG
  while (!isRun) readIRControl();
#endif
}
unsigned char detectObject(boolean sensorR, boolean sensorL)
{
  static unsigned  char lastValue = 0xFF, avg;
  avg = 0xFF;
  if (sensorR & !sensorL) {
    avg = 1;
  } else if (!sensorR & sensorL) {
    avg = 2;
  } else if (sensorR & sensorL) {
    avg = 3;
  }
  readIRControl();
  if (avg ==  0xFF && lastValue != 3 && track == true) {
    return lastValue;
  }
  lastValue = avg;
  return (unsigned char)avg;
}

void readIRControl()
{
  //Lectura del control Remoto
  static unsigned long thread;
  static boolean cmd;
  static uint16_t time;
  if (millis() - thread >= time) {
    cmd = false; //Bandera para pardear el led si se ingresa una tecla correcta
    if (irrecv.decode(&results)) {
#if DEBUG
      Serial.print("Key address=0x");
      Serial.println(results.value, HEX);
#endif
      switch (results.value) {
        case POWER:
          isRun = true;
          cmd = true;
          break;
        case FORWARD:
          if (!isRun) motors.setSpeeds(170, 255);
          cmd = true;
          break;
        case BACK:
          if (!isRun) motors.setSpeeds(-VEL_REMOTE, -VEL_REMOTE);
          cmd = true;
          break;
        case LEFT:
          if (!isRun) motors.setSpeeds(170, -255);
          cmd = true;
          break;
        case RIGHT:
          if (!isRun) motors.setSpeeds(-136, 255);
          cmd = true;
          break;
        case STOP:
          motors.ShortBrake();
          if (isRun)  soft_restart(); //Se reinicia el robot
          cmd = true;
          break;
        case _1:
          strategies = CARPINTERO;
          cmd = true;
          break;
        case _2:
          strategies = TORNADO;
          cmd = true;
          break;
        case _3:
          strategies = AVANCE_LARGO;
          cmd = true;
          break;
        case _4:
          direction = TURN_LEFT;
          cmd = true;
          break;
        case _5:
          direction = TURN_RIGHT;
          cmd = true;
          break;
        case _6:
          strategies = 6;
          cmd = true;
          break;
        case _7:
          strategies = 7;
          cmd = true;
          break;
        case _8:
          strategies = 8;
          cmd = true;
          break;
        case _9:
          strategies = 9;
          cmd = true;
          break;
        case _0:
          strategies = 10;
          cmd = true;
          break;
        case LAST:
          cmd = true;
          break;
      }
      irrecv.resume(); // Recibe el siguiente valor
    }
    if (!isRun) {
      if (cmd) {
        PORTD |= (1 << 4);
        time = 500; //Cambia el tiempo del thread para parpader mas tiempo el led
      }
      else {
        PORTD &= ~(1 << 4);
        time = 50;
      }
    }
    thread = millis();
  }
}
