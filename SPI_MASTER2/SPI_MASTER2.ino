#include <SPI.h>

//LISTA DE COMANDOS
#define START           0x01
#define STOP            0x02
#define RST             0x03
#define SLEEP_MODE      0x04
#define NUM_SENSORS     0x05
#define BLACK_LINE      0x06
#define WHITE_LINE      0x07
#define NOISE           0X08
#define CALIBRATE       0x09
#define SAVE_CAL        0x10
#define RESTORE_CAL     0x11
#define RST_CAL         0x12
#define READ_POSITION   0x13
#define NATCAR_SENSORS  0x14

void setup (void)
{
  Serial.flush();
  Serial.begin(115200);
  Serial.flush();
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  turboSensors_begin();
 writeTurboSensors2(NUM_SENSORS, 10);
}
void loop (void)
{


}  // end of loop

boolean turboSensors_begin() {
  boolean val = false;
  //Reinicia los sensores
  if (writeTurboSensors(RST) == RST) val = true;
  delay(100); //Tiempo de espera para que se inicialice
  writeTurboSensors(START); //Enciende el LED
  return val;
}
byte transferAndWait(const byte what)
{
  byte a = SPI.transfer (what);
  delayMicroseconds (20);
  return a;
} // end of transferAndWait
byte writeTurboSensors(const byte cmd) {
  static byte receive;
  // enable Slave Select
  digitalWrite(SS, LOW);
  transferAndWait(cmd);  // add command
  receive = transferAndWait(0xFF);
  // disable Slave Select
  digitalWrite(SS, HIGH);
  return receive;
}
byte writeTurboSensors2(const byte cmd, byte value) {
  static byte receive;
  // enable Slave Select
  digitalWrite(SS, LOW);
  transferAndWait(cmd);  // add command
  receive = transferAndWait(value);
//  transferAndWait(value);
  // disable Slave Select
  digitalWrite(SS, HIGH);
  return receive;
}
void wakeUp() {
  pinMode(MISO, OUTPUT);
  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  digitalWrite(MISO, 1);
  digitalWrite(SS, 1);
  digitalWrite(MOSI, 1);
  digitalWrite(SCK, 1);
}


