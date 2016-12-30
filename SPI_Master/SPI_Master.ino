#define F_CPU 8000000UL
#include <SPI.h>
#include <CustomQTRSensors.h>

#define LED A0
#define NUM_SENSORS   10// number of sensors used
#define TIMEOUT       2000  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN  255     // emitter is controlled by digital pin 2
QTRSensorsRC qtrrc((unsigned char[]) {
  A4, A5, 0, 1, 2, 3, 4, 5, 6, 7,
}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
unsigned int position;

void setup (void)
{

  digitalWrite(SS, HIGH);  // ensure SS stays high for now
  delay(500);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode


  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin ();

  // Slow down the master a bit
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  digitalWrite(SS, LOW);

  char c;
  for (const char * p = "Hello, world!\n" ; c = *p; p++)
    SPI.transfer (c);
  // disable Slave Select
  digitalWrite(SS, HIGH);

}  // end of setup


void loop (void)
{

  // position = qtrrc.readLine(sensorValues/*, QTR_EMITTERS_ON_AND_OFF, 0, 200*/);

  char c;
  int sensorValue;
  digitalWrite(0, HIGH);   // make sensor line an output
  pinMode(0, OUTPUT);      // drive sensor line high
  delayMicroseconds(500);
  pinMode(0, INPUT);       // make sensor line an input
  digitalWrite(0, LOW);
  sensorValue = 2000;
  resetMillis();
  while (micros() < 2000) {
    if (digitalRead(0) == LOW && micros() < sensorValue) sensorValue = micros();
  }
  String msg = String(sensorValue, DEC) + "\n";
  digitalWrite(LED, 1);
  // enable Slave Select
  digitalWrite(SS, LOW);
  for (const char * p = msg.c_str()  ; c = *p; p++)
  SPI.transfer (c);
  //for (const char * p = "Hello, world!\n" ; c = *p; p++)
  //  SPI.transfer (c);
  // disable Slave Select
  digitalWrite(SS, HIGH);

  digitalWrite(LED, 1);
  delay(100);
  digitalWrite(LED, 0);
  delay(100);

  //delay(800);  // 1 seconds delay
  digitalWrite(LED, 0);
}  // end of loop
