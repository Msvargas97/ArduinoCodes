#include <PololuQTRSensors.h>
#include <SoftwareSerial.h>
#include <OrangutanTime.h>
#include <EEPROM.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 20000000UL //Conmpilar a 20Mhz
#define NUM_SENSORS 6 //Numero de sensores para seguir linea
#define TIMEOUT 1800 //Tiempo de muestreo
#define BAUD_SPEED 115200UL //Velocidad de transmision de bits bluetooth
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
#define pintaL A0  //Define el pin del sensor de pinta izquierdo
#define RxD 12 // Pin TX HC-05
#define TxD 13 // Pin RX HC-05
#define FRENO_BRAKE true //Activa el freno break
#define FRENO_OFF false
#define TIME_READ_BT 5 //Tiempo de muestro en ms de datos recibidos por bluetooth
#define TIME_BLINK_CALIBRATION 1000
#define PRESCALER 8 //Se multiplica por el prescaler para arreglar los delays

//Creacion de variables
unsigned char qtr_pins[NUM_SENSORS] = {4, 2, 0, A3, A2, A1}; //Pines de sensores
unsigned int sensorValues[NUM_SENSORS];
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;
boolean isCalibration = false;
unsigned long thread1, thread2, thread3;
//Creacion de objetos
SoftwareSerial BTSerial(RxD, TxD);

void setup() {
  init_Odin();
  rgbColor(0, 0, 0);
  setMotors(127,127);
  while (analogRead(6) > 100) {
  qtr_read(sensorValues, QTR_EMITTERS_ON);
  if (sensorValues[0] < TIMEOUT && sensorValues[NUM_SENSORS - 1] < TIMEOUT && sensorValues[2] >= TIMEOUT && sensorValues[3] >= TIMEOUT) {
    rgbColor(255, 0, 0);
  } else {
    rgbColor(0, 0, 0);
  }
  delay(100 * 8);  
  }
setMotors(0,0);
  while (true) {
    static byte posicion = 0;
      threadTimer(calibrationSensors, &thread1,20);
      qtr_read(sensorValues,QTR_EMITTERS_ON);
      if (posicion = 0) {
        rgbColor(0, 0, 0);
        setMotors(35, -30);
      }
  }
}
void loop() {

}


//##################### ESPACIO PARA FUNCIONES #######################
void init_Odin() {
  DDRB |= _BV(ledRed) | _BV(ledGreen) | _BV(ledBlue) | _BV(3);
  DDRD |=  _BV(3) |  _BV(6) | _BV(5);
  DDRD |= (0 << pintaR);
  DDRC |= (0 << pintaL);
  analogWrite(9, 127);
  analogWrite(10, 127);
  rgbColor(255, 0, 0);
  BTSerial.begin(BAUD_SPEED); //Configura la velocidad del bluetooth
  inputString.reserve(200);//Crear un buffer para la recepcion de datos
  qtr_rc_init(qtr_pins, NUM_SENSORS, TIMEOUT, EMITTER_PIN); //Inicializar sensores
  qtr_emitters_off();
  cli();//Desactiva las interrupciones
  TCCR0A = TCCR2A = 0xF3;
  TCCR0B = TCCR2B = 0x01; // 0x01 --> 40Khz , 0x02-->10Khz
  OCR0A = OCR0B = OCR2A = OCR2B = 0;//Inicializar PWM en 0
  sei(); //Activa las interrupciones
}
void threadTimer(void (* fp)(), unsigned long *lastTime, unsigned int time) { // funcion aceptando un puntero-a-función fp
  if (get_ms() - *lastTime >= time) {
    fp();  // acceso a la computación de la función señalada por fp
    *lastTime = get_ms();
  }
}

void setMotorLeft(int speed)
{
  static boolean reverse;
  reverse = false;
  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = true;  // preserve the direction
  }
  if (speed > 0xFF) // 0xFF = 255
    speed = 0xFF;
  if (reverse)
  {
    OCR2B = 0;    // hold one driver input high
    OCR2A = speed;  // pwm the other input
  }
  else  // forward
  {
    OCR2B = speed;  // pwm one driver input
    OCR2A = 0;    // hold the other driver input high
  }
}
// función Velocidad Motor Izquierdo
void setMotorRight(int speed)
{
  static boolean reverse;
  reverse = false;
  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = true;  // preserve the direction
  }
  if (speed > 0xFF) // 0xFF = 255
    speed = 0xFF;
  if (reverse)
  {
    OCR0B = 0;    // hold one driver input high
    OCR0A = speed;  // pwm the other input
  }
  else  // forward
  {
    OCR0B = speed;  // pwm one driver input
    OCR0A = 0;    // hold the other driver input high
  }
}
// función Velocidad Motores
void setMotors(int left, int right)
{
  setMotorRight(right);
  setMotorLeft(left);
}
// función Freno en Motores
void setBrake(boolean left, boolean right, boolean MODE)
{
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
void rgbColor(byte rojo, byte verde, boolean azul) {
  RED = rojo;
  GREEN = verde;
  if (azul) portRGB |= _BV(ledBlue);
  else portRGB &= ~_BV(ledBlue);
}
void alignRobot() {

}
void printCalibration() {
  for (int i = 0; i < 6; i++) {
    BTSerial.print(*(qtr_calibrated_maximum_on() + i));
    BTSerial.print(" \t");
    BTSerial.println(*(qtr_calibrated_minimum_off() + i));
  }
}
void printReadSensor() {
  for (int i = 0; i < NUM_SENSORS; i++) {
     BTSerial.print(sensorValues[i]);
    BTSerial.print("\t");
  }
}
void calibrationSensors() {
  qtr_calibrate(QTR_EMITTERS_ON);
}
void serialEvent() {
  while (BTSerial.available()) {
    // get the new byte:
    char inChar = (char)BTSerial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
