#include <DualVNH5019MotorShield.h>
#include <IRremote.h>
#include <avr/interrupt.h>

DualVNH5019MotorShield motors;
IRrecv irrecv(3);
decode_results results;

//M1 Derecha
// M2 Izquierda
#define DEBUG false
#define PISO_R  11
#define PISO_L  13

/*
   Remote Robot Trigger Codigos
   GO: 0xA8895765
   RDY: 0x5448BF41
   RST: 0x8570FF64
*/
#define GO  0xA8895765
#define RDY 0x5448BF41
#define RST 0x8570FF64
#define POWER 0xF61E2A57
#define _1  0xF7283C77
#define _2 0x757FB4DF
#define _3 0xB33B4597
#define _4 0x3C03E507
#define _5 0xE705551F
#define _6 0xA4A58EC7
#define _7 0xE2E45F7F
#define _8 0x6BACFEEF
#define _9 0xE88E91F
#define _0 0x7D168BCF
#define OK 0xBB0ED9E1
#define EXIT 0xE8EDC338

#define INIT_TIME 500
#define CARPINTERO  1
#define TORNADO 2
#define AVANCE_LARGO 3
#define BORDE_BLANCO 0xFF

const unsigned char pin_sensors[] = { 5, A3, A4, A2 , A5 };
unsigned char object, piso;
boolean sensores[5];        //Lateral L  DL  C   DR   Lateral R
boolean isRun = false, track = false, enablePiso = true, enablePisoIR = true, enableDiagSensors = true, white = false;
int strategies, time_fwd;

void setup() {
#if DEBUG
  Serial.begin(115200);
#endif

  for (int i = 0; i < 5; i++) //Activa las resistencias pull-up de los sensores
    pinMode(pin_sensors[i], INPUT_PULLUP);

  pinMode(PISO_R , INPUT_PULLUP); //Resistencias pull-up de piso
  pinMode(PISO_L, INPUT_PULLUP);
  irrecv.enableIRIn(); // Inicializa el receptor IR
  pinMode(0, INPUT);
  pinMode(1, OUTPUT);
  digitalWrite(0, 0);
  for (int i = 0; i < 2 ; i++) {
    digitalWrite(1, 1);
    delay(INIT_TIME);
    digitalWrite(1, 0);
    delay(INIT_TIME);
  }
  isRun =  false;
  strategies = 0;
  while (!isRun) {
    #if DEBUG
    Serial.println(digitalRead(0));
    #endif
    if (digitalRead(0)) isRun = true;
    readIRControl();
  }
  #if DEBUG
  Serial.println("READY");
  #endif
  motors.init(); //Inicializa los motores

  /*for (int i = 0; i < 5 ; i++) {
    digitalWrite(1, 1);
    delay(INIT_TIME);
    digitalWrite(1, 0);
    delay(INIT_TIME);
    }*/

  /* Test Sensores de piso
    white = true;
    lineaBlanca();
    while(true);*/
  if (strategies == AVANCE_LARGO) {
    time_fwd = 1000;
    strategies = CARPINTERO;
  } else {
    time_fwd = 200;
  }
  stop(); //Frena los motores
  if (strategies == CARPINTERO) {
    resetMillis();
    setSpeeds(400, 400); //Avance inicial
    delayReading(time_fwd);
    stop();
  }
  if (strategies == 0) {
    object = detectObject();
    if (object == 20) {
      setSpeeds(400, 400);
    } else {
      strategies = TORNADO;
    }
  }
}

void loop() {
  static boolean Flag; //Bandera para cambiar el sentido de giro
  static int count;
  resetMillis();
  object = detectObject();
  if (!white) { //Si esta en linea negra
    switch (strategies) {
      case CARPINTERO: //Carpintero
        while (millis() < 3000 && !Flag) {
          object = detectObject();
          if (object == 0xFF) Flag = false;
          else Flag = true;
        }
        resetMillis();
        if (!Flag) {
          if (count < 3) {
            setSpeeds(400, 400);
            while (millis() <= 200 && object == 0xFF) object = detectObject();
            stop();
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
        static int time = 3500;
        Flag = !Flag;
        count++;
        if (count < 6) {
          time = 2500;
          if (!Flag) {
            stop();
            delay(10);
            setSpeeds(-400, 400);
            delay(10);
          } else {
            stop();
            delay(10);
            setSpeeds(400, -400);
            delay(10);
          }
        } else {
          time = 200;
          setSpeeds(400, 400);
          count = 0;
        }
        while (millis() < time && object == 0xFF) object = detectObject();
        attack();
        break;
      default:
        attack();
        break;
    }
    delay(2);
  } else {
    lineaBlanca();
    //while (digitalRead(PISO_L) == HIGH  && digitalRead(PISO_R) == HIGH && enablePisoIR == true );
    white = false;
    count = 0;
    strategies = TORNADO;
  }
}
void attack() {
  while (object != 255) {
    object = detectObject();
    switch (object) {
      case 0:
        setSpeeds(-400, 400);
        break;
      case 40:
        setSpeeds(400, -400);
        break;
      case 20:
        setSpeeds(400, 400);
        break;
      case 10:
        setSpeeds(400, 200);
        break;
      case 30:
        setSpeeds(200, 400);
        break;
        /* case 0xFF:
           stop();
           break;*/
    }
  }
}
void delayReading(int time) {
  resetMillis();
  while (millis() <= time) {
    object = detectObject();
    if (object != 0xFF) break;
  }
}
void readSensors() {
  for (int i = 0; i < 5; i++)
    sensores[i]  = digitalRead(pin_sensors[i]);
}
unsigned char detectObject() {
  static unsigned char lastValue = 0xFF;
  unsigned char value = 0xFF;
  //Lectura de sensores y condiciones
  readSensors();
  if (sensores[0] == 1 && sensores[1] == 0 && sensores[2] == 0 && sensores[3] == 0 && sensores[4] == 0 ) value = 0; //Oponente a la izquierda
  else if (sensores[0] == 0 && sensores[1] == 1 && sensores[2] == 0 && sensores[3] == 0 && sensores[4] == 0  && enableDiagSensors == true ) value = 10; //Oponente diagonal derecha
  else if (sensores[0] == 0 && (sensores[1] == 0 ||  enableDiagSensors == false)  && sensores[2] == 1 && (sensores[3] == 0 ||  enableDiagSensors == false) && sensores[4] == 0 ) value = 20; //Ataque
  else if (sensores[0] == 0 && sensores[1] == 1 && sensores[2] == 1 && sensores[3] == 1 && sensores[4] == 0  ) value = 20; //Ataque
  else if (sensores[0] == 0 && sensores[1] == 1 && sensores[2] == 1 && sensores[3] == 0 && sensores[4] == 0 && enableDiagSensors == true  ) value = 10; //Ataque perfilado derecha
  else if (sensores[0] == 0 && sensores[1] == 0 && sensores[2] == 1 && sensores[3] == 1 && sensores[4] == 0 && enableDiagSensors == true ) value = 30; //Ataque perfilado izquierda
  else if (sensores[0] == 0 && sensores[1] == 0 && sensores[2] == 0 && sensores[3] == 1 && sensores[4] == 0 && enableDiagSensors == true ) value = 30; //Oponente diagonal izquierda
  else if (sensores[0] == 0 && sensores[1] == 0 && sensores[2] == 0 && sensores[3] == 0 && sensores[4] == 1 ) value = 40; //Oponente a la derecha

  if (value == 20) enablePiso = false;
  else enablePiso = true;
 #if DEBUG
Serial.println(digitalRead(11));
 #endif
  if (!digitalRead(0)) {
    isRun = false;
    while(true) stop();
  }
  //Verifica si ve linea blanca
  bitWrite(piso, 0, digitalRead(PISO_L));
  bitWrite(piso, 1, digitalRead(PISO_R));
  //Sale de las rutinas de ataque
  if (piso != 0 && enablePiso == true && enablePisoIR == true) {
    value = 0xFF;
    white = true;
    return value;
  }

  if (value == 0xFF && (lastValue == 40 || lastValue == 0 )) {
    return lastValue;
  }
  if (value == 20 ) digitalWrite(1, 1);
  else digitalWrite(1, 0);
  lastValue = value;
  return (value);
}
void lineaBlanca() {
  stop();
  delay(10);
  setSpeeds(-400, -400);
  delay(800);

  stop();
  delay(10);
  setSpeeds(-400, 400);
  delay(10);
  resetMillis();
  while (millis() < 500 && white == true && object == 0xFF) object = detectObject();
  stop();
}
/*int getCurrentsMed() {
  return (motors.getM1CurrentMilliamps() + motors.getM2CurrentMilliamps()) / 2;
  }*/
void setSpeeds(int left, int right) {
  motors.setM2Speed(-left);
  motors.setM1Speed(right);
}
void stop() {
  motors.setM2Brake(400);
  motors.setM1Brake(400);
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
       // case GO:
        case POWER:
          isRun = true;
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
          enableDiagSensors = false; //Desactiva sensores diagonales
          cmd = true;
          break;
        case _9:
          enablePisoIR = false; //Desactiva sensores de piso
          cmd = true;
          break;
  //      case RDY:
    //      cmd = true;
      //    break;
        //case RST:
          //cmd = true;
          //break;
      }
      irrecv.resume(); // Recibe el siguiente valor
    }
    if (!isRun) {
      if (cmd) {
        digitalWrite(1, 1);
        time = 500; //Cambia el tiempo del thread para parpader mas tiempo el led
      }
      else {
        digitalWrite(1, 0);
        time = 50;
      }
    }
    thread = millis();
  }
}
#if DEBUG
void printSensors() {
  for (int i = 0; i < 5; i++) {
    Serial.print(sensores[i]);
    Serial.print("  ");
  }
}
#endif
