#include <PololuQTRSensors.h>
#include <SoftwareSerial.h>
#include <OrangutanTime.h>
#include <EEPROM.h>
#include <MemoryFree.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define F_CPU 20000000UL //Conmpilar a 20Mhz Baby Orangutan
#define NUM_SENSORS 6 //Numero de sensores para seguir linea
#define TIMEOUT 2000 //Tiempo de muestreo
#define BAUD_SPEED 38400 //Velocidad de transmision de bits bluetooth
#define EMITTER_PIN 1 //Control de los sensores
#define RED OCR1A //Define el registro PWM para el led Rojo RGB
#define portRGB PORTB //Define el puerto de los leds RGB
#define ledBlue 0 //Define el pin 8 conectado al led Azul (PB0)
#define ledGreen 1 //Define el pin 9 conectado al led verde (PB1)
#define ledRed 2 //Define el pin 10 conectado al led verde (PB2)
#define GREEN OCR1B //Define el registro PWM para el led Rojo RGB
#define pinPintaR PIND //define el puerto al que esta conectado el sensor
#define pinPintaL PINC //define el puerto al que esta conectado el sensor
#define pintaR 7 //Define el pin del sensor de pinta derecho
#define pintaL 0  //Define el pin del sensor de pinta izquierdo
#define RxD 12 // Pin TX HC-05
#define TxD 13 // Pin RX HC-05
#define FRENO_BRAKE true //Activa el freno break, deja girando la llanta libre mientras frena
#define FRENO_OFF false //Activa el freno de alta impedancia, freno inmediato
#define RANGEBRAKE 2500 // Constante para Rango de Freno (Range Brake)
#define TIME_SERIAL_EVENT 5 //Tiempo de muestro en ms de datos recibidos por bluetooth
#define PRESCALER 8 //Se multiplica por el prescaler para arreglar los delays
#define BUFFER 256 //(bytes) Establece el valor del buffer para la recepcion de BT

//############# CONSTANTES Y VARIABLES ##############################
unsigned char qtr_pins[NUM_SENSORS] = {4, 2, 0, A3, A2, A1}; //Pines de sensores
unsigned int sensorValues[NUM_SENSORS]; //Vector para el almacenamiento de la lectura de los sensores
String inputString = "";         //Buffer para la recepcion de datos BT
boolean stringComplete = false; //booleano para determinar el envio de un dato con '\n'
boolean isCalibration = false;  //booleano para determinar si se calibro el robot
boolean isRun = false; //Booleano para determinar si el robot esta en movimiento
unsigned long thread1, thread2, thread3; //Variables para ejecutar varias funciones como thread o hilos tipo Java
//###################################################################
//Variables para el control PID
unsigned int position = 0; // posición actual de los sensores
int derivative = 0;        // derivada
int proportional = 0;      // proporcional
int power_difference = 0;  // velocidad diferencial
byte max = 123;// Máxima velocidad en el poder diferencial
int last_proportional; // Ultima Proporcional
unsigned int setPoint = ((NUM_SENSORS - 1) * 500); //Variable que controla la posicion deseada del robot por defecto el centro del robot
float KP = 0.06; // Constantes Proporcional y Derivativa
float KD = PI;
byte count = 0, posicion = 0;
boolean Flag;
//###################################################################
//LISTA DE COMANDOS PARA RECEPCION DE BLUETOOTH GUARDADOS EN LA FLASH
unsigned int CMD, lastCMD;
String buffer;
//###################################################################

//Creacion de objetos
SoftwareSerial BTSerial(RxD, TxD); //Se establece los pines para emular la comunicación serial por Software

void setup() {
  init_Odin();
  while (!isRun) {
    receiveAndSendAndroidBT();
  }
  qtr_emitters_on();
  rgbColor(0,0,false);
  BTSerial.println("Let's go!");
  setMotors(90, 90);
  delay(300 * PRESCALER);
};
void loop() {
  controlPID();
};
//********************* ESPACIO PARA FUNCIONES ******************************
//######################## Funcion No 1 #####################################
void init_Odin() {                                                           /*Funcion para inicializar el robot */
  DDRB |= _BV(ledRed) | _BV(ledGreen) | _BV(ledBlue) | _BV(3); //Establece los pines de los leds RGB como salidas
  DDRD |=  _BV(3) |  _BV(6) | _BV(5); //Establece como salida los pines de control del TB662FNG
  DDRD |= (0 << pintaR); //Establece como entrada el sensor de pinta derecha
  DDRC |= (0 << pintaL); //Establece como entrada el sensor de pinta izquierda
  analogWrite(9, 127); //Inicializa el PWM para variar el color de los LEDS RGB
  analogWrite(10, 127);
  rgbColor(0, 0, 0); // establece el valor PWM del led Rojo
  BTSerial.begin(BAUD_SPEED); //Configura la velocidad del bluetooth
  inputString.reserve(BUFFER);//Crear un buffer para la recepcion de datos
  buffer.reserve(BUFFER);
  qtr_rc_init(qtr_pins, NUM_SENSORS, TIMEOUT, EMITTER_PIN); //Inicializar sensores
  qtr_emitters_off(); // Apaga la regleta de sensores
  cli();//Desactiva las interrupciones
  TCCR0A = TCCR2A = 0xF3; //Configura el fastPWM
  TCCR0B = TCCR2B = 0x02; // 0x01 --> 40Khz , 0x02-->10Khz -<---Freq. PWM
  OCR0A = OCR0B = OCR2A = OCR2B = 0;//Inicializar PWM en 0
  sei(); //Activa las interrupciones
}
//######################## Funcion No 2 #####################################
void threadTimer(void (* fp)(), unsigned long *lastTime, unsigned int time) {/*Funcion para crear multitarea*/
  if (get_ms() - *lastTime >= time) {
    fp();  // acceso a la computación de la función señalada por fp
    *lastTime = get_ms();
  }
}
//######################## Funcion No 3 #####################################
void setMotorLeft(int speed) {                                               /*Funcion para controlar motor izquierdo*/
  static boolean reverse;
  reverse = false;
  if (speed < 0)
  {
    speed = -speed; // Mantiene la velocidad positiva
    reverse = true;  // Preservar la direccion
  }
  if (speed > 0xFF) // 0xFF = 255
    speed = 0xFF;
  if (reverse)
  {
    OCR2B = 0;
    OCR2A = speed;  //Asigna PWM
  }
  else  //Adelante
  {
    OCR2B = speed;
    OCR2A = 0;    //Asigna PWM
  }
}
//######################## Funcion No 4 #####################################
// función Velocidad Motor Izquierdo
void setMotorRight(int speed) {                                               /*Funcion para controlar motor derecho */
  static boolean reverse;
  reverse = false;
  if (speed < 0)
  {
    speed = -speed; //Mantiene la valor positivo
    reverse = true;
  }
  if (speed > 0xFF) // 0xFF = 255
    speed = 0xFF;
  if (reverse)
  {
    OCR0B = 0;
    OCR0A = speed;  //Asigna el PWM
  }
  else  // Adelante
  {
    OCR0B = speed;
    OCR0A = 0;
  }
}
//######################## Funcion No 5 #####################################
// función Velocidad Motores
void setMotors(int left, int right) {                                         /*Funcion para controlar ambos motores*/
  setMotorRight(right);
  setMotorLeft(left);
}
//######################## Funcion No 5 #####################################
// función Freno en Motores
void setBrake(boolean left, boolean right, boolean MODE) {                   /*Funcion para aplicar un tipo de freno */
  if ( left )
  {
    if (MODE) OCR2A = OCR2B = 0xFF; //Realiza el freno tipo Brake, dejando girar libre el motor
    else OCR2A = OCR2B = 0; //Realiza el freno de alta impedencia frenando por completo el motor
  }
  if ( right )
  {
    if (MODE) OCR0A = OCR0B = 0xFF; //Realiza el freno tipo Brake, dejando girar libre el motor
    else OCR0A = OCR0B = 0; //Realiza el freno de alta impedencia frenando por completo el motor
  }
}
//######################## Funcion No 6 #####################################
void rgbColor(byte rojo, byte verde, boolean azul) {                         /*Funcion para controlar los dos LED´s RGB*/
  RED = rojo; //Establece los PWM
  GREEN = verde;
  if (azul) portRGB |= _BV(ledBlue);
  else portRGB &= ~_BV(ledBlue);
}
//######################## Funcion No 7 #####################################
void centrarRobot() {                                                        /*Funcion para posicionar el robot en la linea */
  memset(sensorValues, NUM_SENSORS, 0);
  if (isCalibration && !isRun) {
    while (analogRead(6) > 10) {
      position = qtr_read_line(sensorValues, QTR_EMITTERS_ON);
      BTSerial.print(position);
      BTSerial.println("\t" + (String)analogRead(A6));
      if ( abs(position - setPoint) <= 150) {
        rgbColor(255, 0, 0);
      } else {
        rgbColor(0, 0, 0);
      }
      delay( 80 * PRESCALER );
    }
    qtr_emitters_off();
    finishCMD();
  } else {
    errorCMD();
  }

}
//######################## Funcion No 8 #####################################
void printCalibration() {                                                    /*Funcion para mostrar valores de calibracion en pantalla*/
  for (int i = 0; i < 6; i++) {
    BTSerial.print(*(qtr_calibrated_maximum_on() + i));
    BTSerial.print(" \t");
    BTSerial.println(*(qtr_calibrated_minimum_on() + i));
  }
}
//######################## Funcion No 9 #####################################
void printReadSensors() {                                                    /*Funcion para leer los valores Raw de los sensores ya calibrados*/
  qtr_read_line(sensorValues, QTR_EMITTERS_ON);
  for (int i = 0; i < NUM_SENSORS; i++) {
    BTSerial.print(sensorValues[i]);
    BTSerial.print("\t");
  }
  BTSerial.println();
}
//######################## Funcion No 10 #####################################
void serialEvent() {                                                         /*Funcion para leer la información enviada por Bluetooth*/
  while (BTSerial.available()) {
    // Obtiene el nuevo caracter
    char inChar = (char)BTSerial.read();
    // Lo añade al inputString
    inputString += inChar;
    if (inChar == '\n') {// Solo finaliza cuando haya un cambio de linea
      stringComplete = true;
    }
  }
}
//######################## Funcion No 11 #####################################
void controlPID() {                                                          /*Calculo PID*/
  position = qtr_read_line(sensorValues, QTR_EMITTERS_ON); //Obtiene el calculo ponderado de los sensores
  proportional = ((int)position) - setPoint;   // El término proporcional o error debe ser 0 cuando estamos en línea
  // Si entra en el rango de freno, aplicarlo en la  direccion de la curva
  if ( proportional <= -RANGEBRAKE)
  {
    setMotorRight(0);
    setBrake(true, false, FRENO_BRAKE);
    rgbColor(0,255,0);
    delay(1 * PRESCALER);
  }
  else if ( proportional >= RANGEBRAKE )
  {
    setMotorLeft(0);
    setBrake(false, true, FRENO_BRAKE);
    rgbColor(0,0,true);
    delay(1 * PRESCALER);
  }
  derivative = proportional - last_proportional;    // Calcula el término derivativo (cambio) de la posición
  last_proportional = proportional;  // Recordando la última posición
  // Calcula la diferencia entre la potencia de los dos motores [ m1 - m2 ].
  // Si es un número positivo, el robot gira a la [ derecha ]
  // Si es un número negativo, el robot gira a la [ izquierda ]
  //  y la magnitud del número determina el ángulo de giro.
  int power_difference = ( proportional * KP ) + ( derivative * KD );
  if ( power_difference > max ) power_difference = max;
  else if ( power_difference < -max ) power_difference = -max;

  ( power_difference < 0 ) ?   // Asignar velocidad calculada en el poder diferencial de los motores
  setMotors(max, max + power_difference) : setMotors(max - power_difference, max);
}
//######################## Funcion No 12 #####################################
void calibrationWithMotor() {
  posicion = 0;
  count = 0;
  Flag = false;
  rgbColor(0, 0, false);
  while (count < 3) {
    qtr_read(sensorValues, QTR_EMITTERS_ON);
    qtr_calibrate(QTR_EMITTERS_ON);
    if (posicion == 0) {
      setMotors(35, -20);
      rgbColor(255, 255, false);
      if (Flag) Flag = false;
    }
    if (sensorValues[NUM_SENSORS - 1] >= TIMEOUT && posicion == 0) {
      posicion = 1;
      setBrake(1, 1, FRENO_BRAKE);
      delay(480 * PRESCALER);
      setMotors(-20, 35);
      rgbColor(255, 0, false);
    }
    if (sensorValues[0] >= TIMEOUT && posicion == 1) {
      posicion = 0;
      rgbColor(0, 0, true);
      if (!Flag) {
        count++;
        Flag = true;
      }
      setBrake(1, 1, FRENO_BRAKE);
      delay(480 * PRESCALER);
    }
    delay(20 * PRESCALER);
  }
  while (posicion == 0) {
    qtr_read(sensorValues, QTR_EMITTERS_ON);
    if (posicion == 0) {
      setMotors(40, -35);
    }
    if (sensorValues[2] >= TIMEOUT && sensorValues[3] >= TIMEOUT) {
      rgbColor(255, 255, 0);
      setBrake(1, 1, FRENO_BRAKE);
      posicion = 0xFF;
    }
  }
  qtr_emitters_off();
  isCalibration = true;
  finishCMD();
}
//######################## Funcion No 13 #####################################
void listenerOnChangeHC05(void (*funcion)()) {
  serialEvent();
  if (stringComplete) {
    funcion();
    inputString = "";
    stringComplete = false;
  }
}
//######################## Funcion No 14 #####################################
void readCommandsAndroid(void) {
  inputString.trim();
  if (inputString.length() > 0) {
    if (inputString.charAt(0) == '$') {
      if (inputString.length() <= 3) {
        CMD = inputString.substring(1).toInt();
      } else {
        if (inputString.charAt(2) == '#' ) {
          CMD = inputString.substring(1, 3).toInt();
          buffer = inputString.substring(3);
        } else if (inputString.charAt(3) == '#') {
          CMD = inputString.substring(1, 4).toInt();
          buffer = inputString.substring(4);
        }
      }

    }
  }
}
//######################## Funcion No 15 #####################################
void listenerAndroidBT() {
  listenerOnChangeHC05(readCommandsAndroid);
}
//####################### Funcion No 16 ######################################
void finishCMD() {
  CMD = 0xFF;
  BTSerial.println("OK");
  rgbColor(255, 0, true);
  thread1 = get_ms();
}
//####################### Funcion No 17 ######################################
void errorCMD() {
  CMD = 0xFF;
  BTSerial.println("ERROR 0!");
  rgbColor(255, 0, false);
  thread1 = get_ms();
}
//####################### Funcion No 18 ######################################
void measureSensors() {
  if (!isRun) {
    memset(sensorValues, NUM_SENSORS, 0x00);
    rgbColor(0, 0, 0);
    while (analogRead(6) > 10) {
      printReadSensors();
      delay( 100 * PRESCALER);
    }
    qtr_emitters_off();
    finishCMD();
  } else {
    qtr_emitters_off();
    errorCMD();
  }
}
//####################### Funcion No 19 ######################################
void saveCalibration() {
  if (isCalibration && !isRun) {
    qtr_save_calibration();
    delay(500);
    printCalibration();
    qtr_emitters_off();
    finishCMD();
  } else {
    qtr_emitters_off();
    errorCMD();
  }
}
//####################### Funcion No 20 ######################################
void restoreCalibration() {
  if (!isRun) {
    qtr_calibrate(QTR_EMITTERS_ON);
    qtr_reset_calibration();
    if (qtr_restore_calibration()) {
      delay(500);
      isCalibration = true;
      finishCMD();
    } else {
      isCalibration = false;
      errorCMD();
    }
  } else {
    qtr_emitters_off();
    errorCMD();
  }
}
//####################### Funcion No 21 ######################################
void viewCalibration() {
  if (isCalibration && !isRun) {
    printCalibration();
    finishCMD();
  }
  else {
    errorCMD();
  }
}
//####################### Funcion No 22 ######################################
void setVelMax(uint16_t vel) {
  if (!isRun) {
    max = vel;
    BTSerial.println("Velocidad máxima asignada: " + (String)max);
    buffer = " ";
    finishCMD();
  } else {
  }
}
//####################### Funcion No 22 ######################################
uint16_t getVelMax() {
  if (!isRun) {
    BTSerial.println("Velocidad máxima actual: " + (String)max);
    CMD = 0xFF;
    return max;
  } else {
    return false;
  }
}
//####################### Funcion No 22 ######################################
void setKP(float _kp) {
  if (!isRun) {
    KP = _kp;
    BTSerial.println("KP asignado: " + (String)KP);
    buffer = " ";
    finishCMD();
  } else {
    errorCMD();
  }
}
//####################### Funcion No 22 ######################################
float getKP() {
  if (!isRun) {
    BTSerial.println("KP actual: " + (String)KP);
    CMD = 0xFF;
    return KP;
  } else {
    errorCMD();
    return false;
  }

}
//####################### Funcion No 23 ######################################
void setKD(float _kd) {
  if (!isRun) {
    KD = _kd;
    BTSerial.println("KP asignado: " + (String)KD);
    buffer = " ";
    finishCMD();
  } else {
    errorCMD();
  }
}
//####################### Funcion No 24 ######################################
float getKD() {
  if (!isRun) {
    BTSerial.println("KP actual: " + (String)KD);
    CMD = 0xFF;
    return KD;
  } else {
    errorCMD();
    return false;
  }
}
//####################### Funcion No 25 ######################################
uint16_t getFreeMemory() {
  if (!isRun) {
    uint16_t mem = freeMemory();
    BTSerial.println(mem);
    finishCMD();
    return mem;
  } else {
    errorCMD();
    return false;
  }
}
//####################### Funcion No 26 ######################################
String getDateTimeCompilation() {
  if (!isRun) {
    String dateTime;
    dateTime = __DATE__;
    BTSerial.println("Fecha de Compilación: " + dateTime);
    dateTime = __TIME__;
    BTSerial.println("Hora de Compilación: " + dateTime);
    dateTime.concat(__DATE__);
    dateTime.concat(" ");
    dateTime.concat(__TIME__);
    finishCMD();
    return dateTime;
  } else {
    errorCMD();
  }
}
//####################### Funcion No 27 ######################################
void start() {
  if (isCalibration) {
    isRun = true;
    BTSerial.println("Run: true");
    finishCMD();
    rgbColor(0, 0, 0);
  } else {
    errorCMD();
  }
}
//####################### Funcion No 28 ######################################
void stop() {
  if (isRun) {
    isRun = false;
    BTSerial.println("Run: false");
    finishCMD();
    setBrake(1, 1, FRENO_BRAKE);
    rgbColor(255, 0, false);
  } else {
    errorCMD();
  }
}
//####################### Funcion No 29 ######################################
void receiveAndSendAndroidBT() {
  threadTimer(listenerAndroidBT, &thread1, TIME_SERIAL_EVENT); //Crea multitarea para obtener el valor del comando que se desea ejecutar
  if (lastCMD != CMD) {
    if (CMD != 0xFF)  BTSerial.println(CMD);
    switch (CMD) {
      case 1:
        calibrationWithMotor();
        break;
      case 2:
        saveCalibration();
        break;
      case 3:
        restoreCalibration();
        break;
      case 4:
        viewCalibration();
        break;
      case 5:
        centrarRobot();
        break;
      case 6:
        measureSensors();
        break;
      case 7:
        setVelMax(buffer.toInt());
        break;
      case 8:
        getVelMax();
        break;
      case 9:
        setKP(buffer.toFloat());
        break;
      case 10:
        getKP();
        break;
      case 11:
        setKD(buffer.toFloat());
        break;
      case 12:
        getKD();
        break;
      case 13:
        getFreeMemory();
        break;
      case 14:
        getDateTimeCompilation() ;
        break;
      case 15:
        start();
        break;
      case 16:
        stop();
        break;
      default:
        CMD = 0xFF;
        break;
    }
  }
  lastCMD = CMD;
}

