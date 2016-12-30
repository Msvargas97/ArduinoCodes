#include <Servo.h>
#include <DriverMotors.h>
#include <avr/wdt.h>
#define NUM_MOTORS 2 //Numero de motores a usar 
#define STOP_MODE SHORT_BRAKE //Tipo de freno aplicado cuando la velocidad sea igual a 0
#define soft_restart()      \
  do                          \
  {                           \
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }                       \
  }while(0)
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
Servo turbina;
/* Values below should not be changed normally */
const int maxPulse = 2000;
const int minPulse = 1000;

DriverMotors motors((unsigned char []) {
  6, 13
}, (unsigned char []) {
  8, 9, 10 , 5
}, STOP_MODE, NUM_MOTORS);

int pos = 0;
void setup() {
  motors.ShortBrake();
  delay(500);
  pinMode(A1, INPUT_PULLUP);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  turbina.attach(7, minPulse, maxPulse);
  InitESC();
  delay(1000);
}

void loop() {
  static int count, speed;
  if (!digitalRead(A1)) {
    count++;
    delay(200);
    if (count == 1 ) {
      turbina.write(minPulse + 100);
      delay(200);
    } else if (count == 2) {
      motors.ShortBrake();
      turbina.write(0);
      delay(100);
      turbina.detach();
      speed = 0;
      soft_restart();
    }
  }else{
   //   turbina.write(constrain(map(analogRead(A5),0,1023,1000,2000),1000,2000));
     // delay(200);
  }
//  if (count == 1) {
//    if (++speed <= 255 ) {
//      motors.setSpeeds(speed, speed);
//    } else {
//      speed = 0;
//    }
//    delay(100);
//  }

}

void InitESC() {
  turbina.write(0);
  delay(150);
  turbina.write(0);
}


