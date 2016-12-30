#include <PID_v1.h>
#include <CustomQTRSensors.h>
#include <DriverMotors.h>

#define DEBUG false
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       3500 // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN  6
#define NOISE_MIN 200
#define NOISE_MAX 700
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp = 0.34, aggKi = 0.02, aggKd =4.4;
double consKp =0.17, consKi = 0.01, consKd = 2.2;
//double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd,REVERSE);
QTRSensorsRC qtrrc((unsigned char[]) {
  A5, A4, A3, A2, A1, A0, 254, 8
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
DriverMotors motors((unsigned char []) {
 5 , 13 ,10 , 9 
}, true, 2);
unsigned int sensorValues[NUM_SENSORS];
int max = 150;

void setup()
{
#if DEBUG
  Serial.begin(9600);
#endif
  Input = qtrrc.readLine(sensorValues);
  Setpoint = 3500;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-max, max);
  delay(500);
#if DEBUG
  while (!Serial);
#endif
  /*Serial.println("Calibrando Sensores...");
    for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
    {
    position = qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }
    digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

    // print the calibration minimum values measured when emitters were
    printValues();
    qtrrc.saveCalibration();
    delay(1000);
    Serial.println("Borrando valores de calibracion...");
    qtrrc.resetCalibration();
    printValues();
    delay(200);*/

  qtrrc.restoreCalibration();
  //  printValues();
  pinMode(A4, INPUT_PULLUP);
  delay(500);
#if DEBUG
  Serial.println("Valores restaurados con exito! CustomQTRSensores :)");
#endif

  while (digitalRead(A4));
}

void loop()
{
  Input =  qtrrc.readLine(sensorValues/*, QTR_EMITTERS_ON, 0, NOISE_MIN, NOISE_MAX*/);
  if(Input >= 6500 || Input <= 500){
       myPID.SetOutputLimits(-200,200);
   //    myPID.SetTunings(aggKp, aggKi, aggKd);
     }
    else {
     myPID.SetOutputLimits(-max, max);
     //myPID.SetTunings(consKp, consKi, consKd);
    }
  myPID.Compute();
  if (Output < 0 ) motors.setSpeeds(max, max + Output);
  else motors.setSpeeds(max - Output, max);
#if DEBUG
  Serial.print(Input);
  Serial.print(" \t ");
  Serial.println(Output);
#endif
}
void printValues() {
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}

