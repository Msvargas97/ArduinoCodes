#include <SoftwareSerial.h>
#include <util/atomic.h>
/*

   256 128 256 256 256 256 128 128 128 128 128 128 128 128
  2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000

*/
SoftwareSerial mySerial(11, 12); // RX, TX
volatile uint32_t  us;
uint16_t volatile servoTime = 0;

// This is the pulse width we want in units of 0.5us.
uint16_t volatile servoHighTime = 1000;

// This is true if the servo pin is currently high.
boolean volatile servoHigh = false;
ISR(TIMER0_COMPA_vect)
{
  // The time that passed since the last interrupt is OCR2A + 1
  // because the timer value will equal OCR2A before going to 0.
  servoTime += OCR0A + 1;
  us += OCR0A + 1;
  static uint16_t highTimeCopy = 1000;
  static uint8_t interruptCount = 0;

  if (servoHigh)
  {
    if (++interruptCount == 2)
    {
      OCR0A = 127;
    }

    // The servo pin is currently high.
    // Check to see if is time for a falling edge.
    // Note: We could == instead of >=.
    if (servoTime >= highTimeCopy)
    {
      // The pin has been high enough, so do a falling edge.
      PORTB &= ~(1 << 5);
      servoHigh = false;
      interruptCount = 0;
    }
  }
  else
  {
    // The servo pin is currently low.

    if (servoTime >= 20000)
    {
      // We've hit the end of the period (20 ms),
      // so do a rising edge.
      highTimeCopy = servoHighTime;
      PORTB |= (1 << 5);
      servoHigh = true;
      servoTime = 0;
      interruptCount = 0;
      OCR0A = (unsigned char) ((highTimeCopy % 128) + 128) / 2 - 1;
    }
  }
}

void servoInit()
{
  PORTB &= ~(1 << 5);
  DDRB |= (1 << 5);
  // Turn on CTC mode.  Timer 2 will count up to OCR2A, then
  // reset to 0 and cause an interrupt.
  TCCR0A = (1 << WGM01);
  // Set a 1:8 prescaler.  This gives us 0.5us resolution.
  TCCR0B = (1 << CS01);
  // Put the timer in a good default state.
  TCNT0 = 0;
  OCR0A = 127;
  TIMSK0 |= (1 << OCIE0A);  // Enable timer compare interrupt.
  sei();   // Enable interrupts.
}
void servoSetPosition(uint16_t highTimeget_microseconds)
{
  TIMSK0 &= ~(1 << OCIE0A); // disable timer compare interrupt
  servoHighTime = highTimeget_microseconds;
  TIMSK0 |= (1 << OCIE0A); // enable timer compare interrupt
}
unsigned long get_micros() {
  uint8_t oldSREG = SREG;
  cli();
  unsigned long m;
  m = us;

  SREG = oldSREG;
  return m;
}

#define NUM_SENSORS  12
#define TIMEOUT 2000
#define QTR_EMITTERS_OFF 0
#define QTR_EMITTERS_ON 1
#define QTR_EMITTERS_ON_AND_OFF 2

#define QTR_NO_EMITTER_PIN  255
#define QTR_MAX_SENSORS 16
#define PIN_MOTR    2
#define PWM_R       OCR1B
#define PIN_MOTL    3
#define PWM_L       OCR2A

#define emittersOn() \
  PORTB |= (1<<6); \
  PORTD |= (1<<4)
#define emittersOff() \
  PORTB &= ~(1<<6); \
  PORTD &= ~(1<<4)


//################################ FUNCIONES PARA LOS SENSORES #####################################
//Variables para los sensores
uint8_t   numSensors = 12, readMode = QTR_EMITTERS_ON, used;
uint16_t  calibratedMinimumOn[NUM_SENSORS],
          calibratedMaximumOn[NUM_SENSORS],
          calibratedMinimumOff[NUM_SENSORS],
          calibratedMaximumOff[NUM_SENSORS],
          _lastValue = 0, // assume initially that the line is left.
          maxValue = TIMEOUT,
          sensorValues[NUM_SENSORS];

void QTRSensorsinit()
{
  DDRD |= (1 << 4);
  DDRB |= (1 << 6);
  for (int i = 0; i < NUM_SENSORS; i ++)
  {
    calibratedMinimumOn[i] = 0;
    calibratedMaximumOn[i] = 0;
    calibratedMinimumOff[i] = 0;
    calibratedMaximumOff[i] = 0;
  }
  _lastValue = 0; // assume initially that the line is left.
  if (numSensors > QTR_MAX_SENSORS)
    numSensors = NUM_SENSORS;
  emittersOff();
}
void readPrivate()
{
  unsigned char i;
  uint16_t binSensors;
  PORTC |= 0b111100;
  PORTD |= 0b11100111;
  PORTB |= 0b10000001;
  DDRC  |= 0b111100;
  DDRD |= 0b11100111;
  DDRB |= 0b10000001;
  for (i = 0; i < NUM_SENSORS; i++)
    sensorValues[i] = maxValue;
  _delay_us(20);          // charge lines for 10 us
  //pinMode(_pins[i], INPUT);       // make sensor line an input
  //digitalWrite(_pins[i], LOW);        // important: disable internal pull-up!
  DDRC  &= ~(0b111100);
  DDRD &= ~(0b11100111);
  DDRB &= ~(0b10000001);
  PORTC &= ~(0b111100);
  PORTD &= ~(0b11100111);
  PORTB &= ~(0b10000001);

  unsigned long startTime = get_micros();
  while (get_micros() - startTime < maxValue)
  {
    unsigned int time = get_micros() - startTime;
    binSensors = 0;
    bitWrite(binSensors, 0, bit_is_set(PINC, 3));
    bitWrite(binSensors, 1, bit_is_set(PINC, 4));
    bitWrite(binSensors, 2, bit_is_set(PINC, 5));
    bitWrite(binSensors, 3, bit_is_set(PIND, 0));
    bitWrite(binSensors, 4, bit_is_set(PINC, 2));
    bitWrite(binSensors, 5, bit_is_set(PIND, 2));
    bitWrite(binSensors, 6, bit_is_set(PIND, 1));
    bitWrite(binSensors, 7, bit_is_set(PINB, 7));
    bitWrite(binSensors, 8, bit_is_set(PIND, 5));
    bitWrite(binSensors, 9, bit_is_set(PIND, 6));
    bitWrite(binSensors, 10, bit_is_set(PIND, 7));
    bitWrite(binSensors, 11, bit_is_set(PINB, 0));

    for (i = 0; i < NUM_SENSORS; i++)
    {
      if (bitRead(binSensors, i) == false && time < sensorValues[i])
        sensorValues[i] = time;
    }
  }
}
void read()
{
  unsigned int off_values[QTR_MAX_SENSORS];
  unsigned char i;

  if (readMode == QTR_EMITTERS_ON || readMode == QTR_EMITTERS_ON_AND_OFF) {
    emittersOn();
    _delay_us(200);
  }

  else {
    emittersOff();
    _delay_us(200);
  }

  readPrivate();
  emittersOff();

  if (readMode == QTR_EMITTERS_ON_AND_OFF)
  {
    readPrivate();
    for (i = 0; i < NUM_SENSORS; i++)
    {
      sensorValues[i] += maxValue - off_values[i];
    }
  }
}
void resetCalibration()
{
  unsigned char i;
  for (i = 0; i < NUM_SENSORS; i++)
  {
    calibratedMinimumOn[i] = maxValue;
    calibratedMinimumOff[i] = maxValue;
    calibratedMaximumOn[i] = 0;
    calibratedMaximumOff[i] = 0;
  }
}
void calibrate(unsigned char _readMode)
{
  if (_readMode == QTR_EMITTERS_ON_AND_OFF || _readMode == QTR_EMITTERS_ON)
  {
    calibrateOnOrOff(calibratedMinimumOn,
                     calibratedMaximumOn,
                     QTR_EMITTERS_ON);
  }


  if (_readMode == QTR_EMITTERS_ON_AND_OFF || _readMode == QTR_EMITTERS_OFF)
  {
    calibrateOnOrOff(calibratedMinimumOff,
                     calibratedMaximumOff,
                     QTR_EMITTERS_OFF);
  }
}
void calibrateOnOrOff(unsigned int calibratedMinimum[],
                      unsigned int calibratedMaximum[],
                      unsigned char _readMode)
{
  unsigned int max_sensor_values[16];
  unsigned int min_sensor_values[16];
  int i, j;
  readMode = _readMode;
  for (j = 0; j < 10; j++)
  {
    read();
    for (i = 0; i < NUM_SENSORS; i++)
    {
      // set the max we found THIS time
      if (j == 0 || max_sensor_values[i] < sensorValues[i])
        max_sensor_values[i] = sensorValues[i];

      // set the min we found THIS time
      if (j == 0 || min_sensor_values[i] > sensorValues[i])
        min_sensor_values[i] = sensorValues[i];
    }
  }

  // record the min and max calibration values
  for (i = 0; i < NUM_SENSORS; i++)
  {
    if (min_sensor_values[i] > calibratedMaximum[i]) {
      calibratedMaximum[i] = min_sensor_values[i];
    }
    if (max_sensor_values[i] < calibratedMinimum[i]) {
      calibratedMinimum[i] = max_sensor_values[i];

    }
  }
}
void readCalibrated(unsigned int noiseMax)
{
  int i;
  // read the needed values
  read();
  for (i = 0; i < NUM_SENSORS; i++)
  {
    unsigned int calmin, calmax;
    unsigned int denominator;

    // find the correct calibration
    if (readMode == QTR_EMITTERS_ON)
    {
      calmax = calibratedMaximumOn[i];
      calmin = calibratedMinimumOn[i];
    }
    else if (readMode == QTR_EMITTERS_OFF)
    {
      calmax = calibratedMaximumOff[i];
      calmin = calibratedMinimumOff[i];
    }
    else // QTR_EMITTERS_ON_AND_OFF
    {

      if (calibratedMinimumOff[i] < calibratedMinimumOn[i]) // no meaningful signal
        calmin = maxValue;
      else
        calmin = calibratedMinimumOn[i] + maxValue - calibratedMinimumOff[i]; // this won't go past _maxValue

      if (calibratedMaximumOff[i] < calibratedMaximumOn[i]) // no meaningful signal
        calmax = maxValue;
      else
        calmax = calibratedMaximumOn[i] + maxValue - calibratedMaximumOff[i]; // this won't go past _maxValue
    }

    denominator = calmax - calmin;
    signed int x = 0;
    if (denominator != 0)
      x = (((signed long)sensorValues[i]) - calmin)
          * 1000 / denominator;
    if (x < 0)
      x = 0;
    else if (x > noiseMax)
      x = 1000;
    sensorValues[i] = x;
  }

}
unsigned int readLine(unsigned char white_line, unsigned int noiseMin, unsigned int noiseMax)
{
  unsigned char i, on_line = 0, j = 0;
  unsigned long avg; // this is for the weighted total, which is long
  // before division
  unsigned int sum; // this is for the denominator which is <= 64000
  readCalibrated(noiseMax);

  avg = 0;
  sum = 0;
  i = 0;
  j = 0;
  for (i = (NUM_SENSORS - numSensors)/2; i < NUM_SENSORS - (NUM_SENSORS - numSensors)/2; i++) {

    int value = sensorValues[i];
    if (white_line)
      value = 1000 - value;

    // keep track of whether we see the line at all
    if (value > 200) {
      on_line = 1;
    }

    // only average in values that are above a noise threshold
    if (value > noiseMin) {
      avg += (long)(value) * (j * 1000);
      sum += value;
    }
    j++;
    if(sensorValues[i] >= 500 && ( i == 5 || i == 6)) sensorValues[i] = 1000;
  }

  if (!on_line)
  {
    // If it last read to the left of center, return 0.
    if (_lastValue < (numSensors - 1) * 500)
      return 0;

    // If it last read to the right of center, return the max.
    else
      return ((numSensors - 1) * 1000);

  }

  _lastValue = avg / sum;

  return _lastValue;
}

void cal() {
  //calibrate();

  calibratedMinimumOn[0] =  384;
  calibratedMinimumOn[1] =  256 ;
  calibratedMinimumOn[2] = 384;
  calibratedMinimumOn[3] =  384;
  calibratedMinimumOn[4] = 384;
  calibratedMinimumOn[5] = 384;
  calibratedMinimumOn[6] = 256;
  calibratedMinimumOn[7] = 256;
  calibratedMinimumOn[8] = 256;
  calibratedMinimumOn[9] = 256;
  calibratedMinimumOn[10] = 256;
  calibratedMinimumOn[11] = 256;
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    calibratedMaximumOn[i] = TIMEOUT;
  }

}

void setup() {
  _delay_ms(1000);
  // set the data rate for the SoftwareSerial port
  servoInit();

  _delay_ms (1);
  DDRD = 0xFF;
  //  DDRB =  0b11101111;
  //  PORTB =  0b11111111;
  DDRC = 0;
  PORTC = 0b11;
  QTRSensorsinit();
  resetCalibration();
  cal();
  DDRB |= (1 << 6);
  PORTB &= ~(1 << 6);
  
   for (int i = 0; i < 400; i++) {
      calibrate(QTR_EMITTERS_ON);
      _delay_ms(20);
   }
  mySerial.begin(9600);
  // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    mySerial.print(calibratedMinimumOn[i]);
    mySerial.print(' ');
  }
  mySerial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    mySerial.print(calibratedMaximumOn[i]);
    mySerial.print(' ');
  }
  mySerial.println();
  mySerial.println();
  _delay_ms(1000);
}

void loop() {
numSensors = 8;

  int position = readLine(0, 200, 1000);
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    mySerial.print(sensorValues[i]);
    mySerial.print("  ");
  }
  mySerial.println(position);
  _delay_ms(100);

}

