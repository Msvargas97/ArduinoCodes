
//data storage variables
byte newData = 0;
byte prevData = 0;
unsigned int time = 0;//keeps time and sends vales to store in timer[] occasionally
int timer[10];//storage for timing of events
int slope[10];//storage for slope of events
unsigned int totalTimer;//used to calculate period
unsigned int period;//storage for period of wave
byte index = 0;//current storage index
double frequency;//storage for frequency calculations
int maxSlope = 0;//used to calculate max slope as trigger point
int newSlope;//storage for incoming slope data

//variables for deciding whether you have a match
byte noMatch = 0;//counts how many non-matches you've received to reset variables if it's been too long
byte slopeTol = 4;//slope tolerance- adjust this if you need
int timerTol = 10;//timer tolerance- adjust this if you need

//variables for amp detection
unsigned int ampTimer = 0;
byte maxAmp = 0;
volatile byte checkMaxAmp;
byte ampThreshold = 20; //raise if you have a very noisy signal

//variables for tuning
double correctFrequency;//the correct frequency for the string being played
int cont;
int numReadings = 2;

int readings[5];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int inputPin = A15;
void setup() {
  Serial.begin(115200);
  cli();//disable interrupts
  //set up continuous sampling of analog pin 0 at 38.5kHz
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  analogRead(A15);
  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements
  sei();//enable interrupts
}

ISR(ADC_vect) {//when new ADC value ready
  prevData = newData;//store previous value
  newData = ADCH;//get value from A0
  if (prevData < 127 && newData >= 127) { //if increasing and crossing midpoint
    newSlope = newData - prevData;//calculate slope
    if (abs(newSlope - maxSlope) < slopeTol) { //if slopes are ==
      //record new data and reset time
      slope[index] = newSlope;
      timer[index] = time;
      time = 0;
      if (index == 0) { //new max slope just reset
        noMatch = 0;
        index++;//increment index
      }
      else if (abs(timer[0] - timer[index]) < timerTol && abs(slope[0] - newSlope) < slopeTol) { //if timer duration and slopes match
        //sum timer values
        totalTimer = 0;
        for (byte i = 0; i < index; i++) {
          totalTimer += timer[i];
        }
        period = totalTimer;//set period
        //reset new zero index values to compare with
        timer[0] = timer[index];
        slope[0] = slope[index];
        index = 1;//set index to 1
        noMatch = 0;
      }
      else { //crossing midpoint but not match
        index++;//increment index
        if (index > 9) {
          reset();
        }
      }
    }
    else if (newSlope > maxSlope) { //if new slope is much larger than max slope
      maxSlope = newSlope;
      time = 0;//reset clock
      noMatch = 0;
      index = 0;//reset index
    }
    else { //slope not steep enough
      noMatch++;//increment no match counter
      if (noMatch > 9) {
        reset();
      }
    }
  }

  time++;//increment timer at rate of 38.5kHz

  ampTimer++;//increment amplitude timer
  if (abs(127 - ADCH) > maxAmp) {
    maxAmp = abs(127 - ADCH);
  }
  if (ampTimer == 1000) {
    ampTimer = 0;
    checkMaxAmp = maxAmp;
    maxAmp = 0;
  }
}

void reset() { //clean out some variables
  index = 0;//reset index
  noMatch = 0;//reset match couner
  maxSlope = 0;//reset slope
}

bool enable[6];
//Determine the correct frequency and light up
//the appropriate LED for the string being played
void stringCheck() {

  if (frequency > 70 & frequency < 90 ) {
    Serial.println("E gruesa");
    enable[0] = true;
    correctFrequency = 82.4;
  }
  if (frequency > 100 & frequency < 120  & enable[1]) {
    Serial.println("A");
    correctFrequency = 110;
  }
  if (frequency > 135 & frequency < 155  & enable[2]) {
    Serial.println("D");
    correctFrequency = 146.8;
  }
  if (frequency > 186 & frequency < 205  & enable[3]) {
    Serial.println("G");
    correctFrequency = 196;
  }
  if (frequency > 235 & frequency < 255  & enable[4]) {
    Serial.println("B");
    correctFrequency = 246.9;
  }
  if (frequency > 320 & frequency < 340  & enable[5]) {
    Serial.println("E delgada");
    correctFrequency = 329.6;
  }
}

//Compare the frequency input to the correct
//frequency and light up the appropriate LEDS
void frequencyCheck() {
  if (frequency > 70 & frequency < 90 ) {
    if (frequency > correctFrequency + 2) {
      Serial.println("Girar Sentido Horario");
    }
    if (frequency > correctFrequency + 4) {
      //  Serial.println("Sentido Anti-Horario");
    }
    if (frequency > correctFrequency + 6) {
      //  Serial.println(correctFrequency + 6);
    }
    if (frequency < correctFrequency - 2) {
      Serial.println("Girar Anti-Horario");

    }
    if (frequency < correctFrequency - 4) {
      //Serial.println(correctFrequency - 4);

    }
    if (frequency < correctFrequency - 6) {
      //Serial.println(correctFrequency - 6);

    }
    if (frequency > correctFrequency - 1 & frequency < correctFrequency + 1) {
      Serial.println( correctFrequency);
    }
  }

}
double FundFreq  = 89;
byte maxAmplitud;
bool Flag;
void loop() {
  if (checkMaxAmp > ampThreshold) {
    frequency = 38462 / float(period); //calculate frequency timer rate/period
    if (frequency > 70 && frequency < 90 )
    {
      if(maxAmplitud < checkMaxAmp){
        maxAmplitud = checkMaxAmp;
      }
      if(frequency < FundFreq)       FundFreq = frequency;
      Serial.print(FundFreq);
      Serial.print(" Hz ");
      Serial.print(maxAmplitud);
      Serial.println(" A");      
    }
  }else{
    if(frequency != 0){
      maxAmplitud = 0;
      FundFreq  = 89;
      frequency = 0;
    }
  }
  delay(1);
}
void insercion(double *x, int n)
{
  int i, j, a;
  double index;
  for (i = 1; i < n; i++)
  {
    index = x[i];
    a = i - 1;
    while (a >= 0 && x[a] > index)
    {
      x[a + 1] = x[a];
      a--;
    }
    x[a + 1] = index;
  }
}
