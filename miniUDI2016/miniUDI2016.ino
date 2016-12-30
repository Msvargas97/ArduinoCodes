//NOTA: CONSUMO FULL BATTERY 8.5 -> 0.36mA with motors sin carga, without motors = 150mA
#include <DriverMotors.h>
#include <IRremote.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#define DEBUG   false//Activa o desactiva el monitor Serial
#define IR_TRIGGER false
#define INIT_TIME   499 //tiempo inicial para el parpadeo 
#define VEL_REMOTE  100 //Velocidad de prueba

/*
   Remote Robot Trigger Codigos
   GO: 0xA8895765
   RDY: 0x5448BF41
   RST: 0x8570FF64
*/
#define GO 0xA8895765
#define GO_2 0x49F7B62F
#define GO_3 0x825A1DF5
#define RDY 0x5448BF41
#define RST 0x8570FF64
//Directivas para las teclas del control Remoto
#define FORWARD 0xC20370A1
#define BACK    0x81930A09
#define RIGHT   0x21035431
#define LEFT    0x983AB4C1
#define STOP    0xBB0ED9E1
#define POWER   0xF61E2A57
#define POWER_2 0x406A954D
#define PLAY_UP 0x80BFBB44
#define PLAY_DOWN 0x80BF31CE
#define PRESSED 0xFFFFFFFF
#define LAST 0xD28EF217
#define VOL_MORE 0xBE3BB37
#define VOL_LESS 0xD4D9F2A7
#define BTN_RED 0x240C9161
#define BTN_GREEN 0xA26409C9
#define BTN_YELLOW  0xE01F9A81
#define BTN_BLUE  0x68E839F1
//NUMEROS
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

#define SPEED_L 150
#define SPEED_R 200
#define CARPINTERO  1
#define TORNADO 2
#define AVANCE_LARGO 3
#define TURN_90 4
#define TURN_180 5
#define ARCO 6

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
        enableSensors =
        true,
        Flag,
        controlIR,
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
#endif
  pinMode(1, INPUT_PULLUP);
  strategies = CARPINTERO;
  while (!isRun) {
#if DEBUG
    Serial.println(digitalRead(1));
#endif
#if IR_TRIGGER 
    if ( digitalRead(1)) isRun = true;
#endif
    readIRControl();
  }
  isRun = true;
  track = true; //Activa la memoria para seguir al objetivo
  digitalWrite(4, 0);
#if !DEBUG
  for (int i = 0; i < 5; i++) {
    digitalWrite(4, 1);
    delay(INIT_TIME);
    digitalWrite(4, 0);
    delay(INIT_TIME);
    }
#else
  Serial.println("Estrategia:" + (String)strategies);
  Serial.print("Direccion:");
  if (direction == TURN_RIGHT) Serial.println("DERECHA");
  else Serial.println("IZQUIERDA");
  delay(1000);
#endif
  motors.ShortBrake();
  if (strategies == AVANCE_LARGO) {
    time_fwd = 500;
    strategies = CARPINTERO;
  } else {
    time_fwd = 50;
  }
  switch (strategies) {
    case CARPINTERO:
      resetMillis();
      motors.setSpeeds(SPEED_L, SPEED_R );
      delay(time_fwd);

      /*while (millis() <= time_fwd) {
        object = detectObject(!bitRead(PINF, 5), !bitRead(PINF, 0));
        if (object >= 1 && object <= 3 ) break;
        }*/
      break;
    case TURN_90:
      turn90();
      strategies = TORNADO;
      break;
    case TURN_180:
      turn180();
      strategies = TORNADO;
      break;
    case ARCO:
      arco();
      strategies = TORNADO;
      break;
  }
  motors.ShortBrake();
  /* while (true) {
    readIRControl();
    }*/
  if (strategies == TORNADO) {
    motors.setSpeeds(SPEED_L * 0.8, SPEED_R * 0.8);
    delay(100);
    motors.setSpeeds(SPEED_L , SPEED_R);
    resetMillis();
    while (millis() <= 100) {
      object = detectObject(!bitRead(PINF, 5), !bitRead(PINF, 0));
      if (object >= 1 && object <= 3 ) break;
    }
  }
  motors.ShortBrake();
  object = detectObject(!bitRead(PINF, 5), !bitRead(PINF, 0));
  if (strategies == TORNADO && object == 3) {
    while (object == 3) {
      object = detectObject(!bitRead(PINF, 5), !bitRead(PINF, 0));
      motors.setSpeeds(SPEED_L, SPEED_R);
    }
  }
  /* while (true) {
     readIRControl();
    }*/
  //object = detectObject(!bitRead(PINF, 5), !bitRead(PINF, 0));
  //  if (object == 0xFF && strategies >= 4 ) strategies = TORNADO;
  if ( strategies != CARPINTERO) Flag = direction;
  else Flag = false;
  resetMillis();
  track = false;
}
void loop() {
  //static boolean Flag = direction;
  if (isRun) {
    static int count;
    resetMillis();
    object = detectObject(!bitRead(PINF, 5), !bitRead(PINF, 0));
    switch (strategies) {
      case CARPINTERO: //Carpintero
        while (millis() < 4000 && !Flag) {
          object = detectObject(!bitRead(PINF, 5), !bitRead(PINF, 0));
          if (object == 0xFF && enableSensors == true) Flag = false;
          else Flag = true;
          if (object != 0xFF ) strategies = TORNADO;
        }

        resetMillis();
        if (!Flag) {
          if (count < 6) {
            motors.setSpeeds(SPEED_L+40, SPEED_R);
            while (millis() <= 50) {
              object = detectObject(!bitRead(PINF, 5), !bitRead(PINF, 0));
              if (object != 0xFF) {
                strategies = TORNADO;
                break;
              }
            }
            motors.ShortBrake();
            count++;
          } else {
            count = 0;
            strategies = TORNADO;
          }
        } else {
#if DEBUG
          Serial.println("Attack...");
#endif
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
        while (millis() < 3500 && object == 0xFF)  object = detectObject(!bitRead(PINF, 5), !bitRead(PINF, 0));
        attack();
        break;
      default:
        attack();
        break;
    }
    delay(2);
  } else {
    motors.ShortBrake();
    soft_restart();
  }

};//end void loop
//############################# FUNCIONES ###############################
void arco() {
  motors.setSpeeds(SPEED_L * 0.5, SPEED_R);
  delay(150);
  turn180();
}

void turn180() {
  turn90();
  turn90();
}
void turn90() {
  if (direction == TURN_LEFT) motors.setSpeeds(255, -255);
  else {
    motors.setSpeeds(-255, 255);
  }
  delay(150);
  motors.ShortBrake();
}
void turn45() {
  if (direction == TURN_LEFT) motors.setSpeeds(255, -255);
  else {
    motors.setSpeeds(-255, 255);
    delay(5);
  }
  delay(50);
  motors.ShortBrake();
}
void attack() {
  static float speed;
  static int time;
  while (object != 0xFF) {
    if (strategies != TORNADO)  track = true;
    object = detectObject(!bitRead(PINF, 5), !bitRead(PINF, 0));
    switch (object) {
      case 1:
        speed = 0.3;
        motors.setSpeeds(SPEED_L, -SPEED_R);
        while (millis() < 20 ) {
          object = detectObject(!bitRead(PINF, 5), !bitRead(PINF, 0));
          if (object == 2) break;
        }
        break;
      case 2:
        speed = 0.3;
        motors.setSpeeds(-SPEED_L, SPEED_R);
        while (millis() < 20 ) {
          object = detectObject(!bitRead(PINF, 5), !bitRead(PINF, 0));
          if (object == 1) break;
        }
        break;
      case 3:

        time++;
        motors.setSpeeds(SPEED_L * speed, SPEED_R * speed);
        if (strategies == CARPINTERO) {
          if (time >= 800 && speed <= 0.9) {
            speed = speed + 0.1;
            time = 0;
          }
        } else {
          if (time >= 100 && speed <= 0.9) {
            speed = speed + 0.1;
            time = 0;
          }
        }
        if (strategies == TORNADO || strategies == CARPINTERO) strategies = 0;
        break;
        /*case 0xFF:
          motors.ShortBrake();
          break;*/
    }

  }
  track = false;
}
void init_minisumo() {
  //Seteo de perifericos
  DDRF = 0;
  PORTF = 0xFF;
  DDRD |= ((1 << 5) | (1 << 4)); //LEDS
  PORTD &= ~(1 << 5); //Apaga el led verde del bootloader
  irrecv.enableIRIn(); // Inicializa el modulo IR
  pinMode(A1, INPUT_PULLUP); //Activa las resistencias pull-up para los sensores
  pinMode(A5, INPUT_PULLUP);
  cli();
  wdt_disable(); //Activa el perro guardian
  sei();
  pinMode(1, INPUT);
}
unsigned char detectObject(boolean sensorR, boolean sensorL)
{
#if DEBUG
  Serial.print(sensorL);
  Serial.print(" ");
  Serial.print(sensorR);
#endif
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
  if (!digitalRead(1) && !controlIR) {
    isRun = false;
    motors.ShortBrake();
    soft_restart();
  }
  if (avg ==  0xFF && lastValue != 3 && track == true) {
    return lastValue;
  }
  lastValue = avg;
  return (unsigned char)avg;
}

void readIRControl()   //Lectura del control Remoto
{
  static unsigned long thread;
  static boolean cmd, readyModule = false;
  static uint16_t time;
  if (millis() - thread >= time) {
    cmd = false; //Bandera para pardear el led si se ingresa una tecla correcta
    if (irrecv.decode(&results)) {
#if DEBUG
      Serial.print("Key address=0x");
      Serial.println(results.value, HEX);
#endif
      switch (results.value) {
        /*case GO:
          case GO_2:
          case GO_3:
          case 0x1E7DFEEC:
          case 0xB0F3481B:
          case 0x14F6D440:*/
        case POWER:
#if DEBUG
          Serial.println("GO");
#endif
          controlIR = true;
          isRun = true;
          //cmd = true;
          break;
        case 0xFEAC02E5:
          if (readyModule) isRun = true;
          break;
        case FORWARD:
          if (!isRun) motors.setSpeeds(SPEED_L, SPEED_R);
          cmd = true;
          break;
        case BACK:
          if (!isRun) motors.setSpeeds(-VEL_REMOTE, -VEL_REMOTE);
          cmd = true;
          break;
        case LEFT:
          if (!isRun) motors.setSpeeds(SPEED_L, -SPEED_R);
          cmd = true;
          break;
        case RIGHT:
          if (!isRun) motors.setSpeeds(-SPEED_L, SPEED_R);
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
          strategies = TURN_90;
          cmd = true;
          break;
        case _5:
          strategies = TURN_180;
          cmd = true;
          break;
        case _6:
          strategies = ARCO;
          cmd = true;
          break;
        case _7:
          //strategies = 7;
          cmd = true;
          break;
        case _8:
          //strategies = 8;
          cmd = true;
          break;
        case _9:
          //strategies = 9;
          cmd = true;
          break;
        case _0:
          //strategies = 10;
          cmd = true;
          break;
        case 0xDBE32B77:
        case RDY:
          readyModule = true;
          PORTD |= (1 << 4);
#if DEBUG
          Serial.println("RDY");
#endif
          break;
        case RST:
          readyModule = false;
#if DEBUG
          Serial.println("RST");
#endif
          cmd = true;
          break;
        case VOL_MORE:
          direction = TURN_RIGHT;
          cmd = true;
          break;
        case VOL_LESS:
          direction = TURN_LEFT;
          cmd = true;
          break;
        case BTN_RED:
          enableSensors = false;
          cmd = true;
          break;
          /*case LAST:
            cmd = true;
            break;*/
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
