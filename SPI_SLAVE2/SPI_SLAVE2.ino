#define F_CPU 8000000UL

#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <stdlib.h>


#define NUM_SENSORS_MAX  14
#define LED A0
#define LED_ON  PORTC |= _BV(0);
#define LED_OFF PORTC &= ~_BV(0);
#define EMITTER_PIN A1
#define SENSORS_ON  PORTC |= _BV(1);
#define SENSORS_OFF PORTC &= ~_BV(1);

//LISTA DE COMANDOS
#define START           0x01
#define STOP            0x02
#define RST             0x03
#define SLEEP_MODE      0x04
#define NUM_SENSORS     0x05
#define TIMEOUT         0x06
#define BLACK_LINE      0x07
#define WHITE_LINE      0x08
#define NOISE           0X09
#define CALIBRATE       0x10
#define SAVE_CAL        0x11
#define RESTORE_CAL     0x12
#define RST_CAL         0x13
#define READ_POSITION   0x14
#define NATCAR_SENSORS  0x15
//Reinicio usando el perro guardian
#define soft_restart()      \
  do                          \
  {                           \
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }                       \
  }while(0)

volatile byte command = 0;
volatile int numSensors = NUM_SENSORS_MAX;
volatile unsigned int *sensorValues;
volatile bool isRun = false,
              reset = false,
              read = false;
uint16_t timeout = 2000;

void setup (void)
{
  cli();
  wdt_disable(); //Activa el perro guardian
  sei();
  //Pin de salida del esclavo
  pinMode(MISO, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(EMITTER_PIN, OUTPUT);
  //SPI MODO ESCLAVO
  SPCR |= _BV(SPE);
  //Activa la interrupción
  SPCR |= _BV(SPIE);
  sleep_disable();
  while (!isRun) {
    if (digitalRead (SS) == HIGH) command = 0;
    if (reset) {
      soft_restart();
    }
  }
  //Crea el vector según el número de sensores escodigos
  if (sensorValues == 0) sensorValues = (unsigned int*)malloc(sizeof(unsigned int) * numSensors);
}  // end of setup
// Rutina de interrupción SPI
ISR (SPI_STC_vect)
{
  unsigned char c = SPDR;
  if (!command) command = c;
  switch (command)
  {
    case START: //Comando de inicialización
      static uint8_t i;
      LED_ON;
      if (i >= 1) {
        isRun = true; //  SPDR = 0x01;
        SENSORS_ON;
      }
      i++;
      break;
    case STOP:
      LED_OFF;
      SENSORS_OFF;
      isRun = false;  // SPDR = 0x02;
      break;
    case RST:
      SENSORS_OFF;
      reset = true;
      LED_ON;
      break;
    case NUM_SENSORS:
      numSensors = c;
      if (numSensors > NUM_SENSORS_MAX)
        numSensors = NUM_SENSORS_MAX;
      break;
    case TIMEOUT:
      break;
    case SLEEP_MODE:
      sleep();
      break;
    case READ_POSITION:
      read = true;
      pinMode(SS,OUTPUT);
      break;
  }
  SPDR = command;
  // end of switch
}  // end of interrupt service routine (ISR) SPI_STC_vect

void loop (void)
{
  static int i;
  if (reset) soft_restart();
  if (read) {
    noInterrupts();
    //Desactiva la interrupción
    SPCR &= ~_BV(SPIE);
    interrupts();
  }
  // Si la comunicación SPI no esta activada
  if (digitalRead (SS))  command = 0;
  if (read) {
    digitalWrite(0, HIGH);   // make sensor line an output
    pinMode(0, OUTPUT);      // drive sensor line high
    delayMicroseconds(10);
    pinMode(0, INPUT);       // make sensor line an input
    digitalWrite(0, LOW);
    for (i = 0; i < numSensors; i++) sensorValues[i] = timeout;
    resetMillis();
    while (micros() < 2000) {
      if (digitalRead(0) == LOW && micros() < sensorValues[0]) sensorValues[0] = micros();
    }
    digitalWrite(SS,LOW);
    transfer(sensorValues[0]);
    digitalWrite(SS,HIGH);
  }
}  // end of loop
inline static uint8_t transfer(uint8_t data) {
  SPDR = data;
  /*
     The following NOP introduces a small delay that can prevent the wait
     loop form iterating when running at the maximum speed. This gives
     about 10% more speed, even if it seems counter-intuitive. At lower
     speeds it is unnoticed.
  */
  asm volatile("nop");
  while (!(SPSR & _BV(SPIF))) ; // wait
  return SPDR;
}
void sleep() {
  pinMode(MISO, INPUT);
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(SS, INPUT);
  isRun = false;
  PCMSK0 |= (1 << PCINT2); //Use PCINT2 = PB2 es el mismo pin SS
  PCICR |= (1 << PCIE0); //Activa las interrupcions por cambio de estado
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);//Establecemos el modo de bajo consumo.
  power_adc_disable();
  power_twi_disable();
  cli();
  sleep_enable();
  sei();
  sleep_cpu();
}
//Rutina de interrupción //Despierta al microcontrolador del bajo consumo
ISR(PCINT0_vect)
{
  if (!isRun)
  {
    if (digitalRead (SS) && digitalRead (MOSI) && digitalRead (MISO) && digitalRead(SCK)) {
      sleep_disable();
      PCMSK0 &= ~(1 << PCINT2);
      PCICR &= ~(1 << PCIE0); //Desactiva las interrupcions por cambio de estado
    }
  }
}


